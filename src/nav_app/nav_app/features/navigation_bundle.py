"""Generate companion navigation assets from measured, gravity-aligned XYZ."""
import hashlib
import gc
import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid
import numpy as np
from ..map_processing import load_pcd,pointcloud_to_grid,save_grid_map
from ..scene_catalog import normalize_scene_title
from .capture_quality import atomic_json
from .resource_lease import ResourceLease


GRID_POLICY_VERSION=2


class NavigationBuildCancelled(RuntimeError):
    """The current derived stage was cancelled; committed stages are reusable."""


def _check_cancel(cancel):
    if cancel is not None and cancel():
        raise NavigationBuildCancelled('地图处理已暂停，已完成步骤保留')


def _report(progress, stage, done=None, total=None, state='running'):
    if progress is not None:
        progress(dict(stage=stage, state=state, done=done, total=total))


def digest(path, *, cancel=None, progress=None):
    value=hashlib.sha256()
    total=Path(path).stat().st_size;done=0;reported_at=time.monotonic()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(2**20),b''):
            _check_cancel(cancel)
            value.update(block);done+=len(block)
            if progress is not None and (done==total or time.monotonic()-reported_at>=.25):
                progress(done,total);reported_at=time.monotonic()
    return value.hexdigest()


def terrain_relative_points(points, floor_z, cell=.5):
    """Classify measured returns against a local ground surface in unchanged XY.

    A single horizontal Z band loses gently sloping paths. Find the lowest
    supported 10 cm height band per half-metre cell, then reject isolated low
    returns with a neighbourhood median. Unobserved areas remain unknown.
    This is a single-level walking map, not a multi-floor terrain reconstruction.
    """
    from scipy.ndimage import distance_transform_edt, median_filter
    xyz=np.asarray(points,dtype=np.float32)
    xyz=xyz[np.isfinite(xyz).all(axis=1)]
    low=np.floor(xyz[:,:2].min(axis=0)/cell).astype(np.int64)
    xy=np.floor(xyz[:,:2]/cell).astype(np.int64)-low
    size=xy.max(axis=0)+1
    while int(size[0])*int(size[1])>2_000_000:
        cell*=2;low=np.floor(xyz[:,:2].min(axis=0)/cell).astype(np.int64)
        xy=np.floor(xyz[:,:2]/cell).astype(np.int64)-low;size=xy.max(axis=0)+1
    candidates=(xyz[:,2]>=floor_z-1.5)&(xyz[:,2]<=floor_z+1.)
    keys=np.c_[xy[candidates],np.floor(xyz[candidates,2]/.1).astype(np.int64)]
    bins,counts=np.unique(keys,axis=0,return_counts=True)
    if not len(bins):raise ValueError('地面观测不足，请检查地图覆盖范围')
    indices=bins[:,1]*size[0]+bins[:,0]
    maximum=np.zeros(int(size.prod()),dtype=np.int32)
    np.maximum.at(maximum,indices,counts)
    supported=(counts>=3)&(counts>=maximum[indices]*.2)
    surface=np.full(int(size.prod()),np.inf,dtype=np.float32)
    np.minimum.at(surface,indices[supported],(bins[supported,2]+.5)*.1)
    surface=surface.reshape(int(size[1]),int(size[0]))
    valid=np.isfinite(surface)
    if np.count_nonzero(valid)<3:raise ValueError('地面观测不足，请检查地图覆盖范围')
    distance,nearest=distance_transform_edt(~valid,return_indices=True)
    smooth=median_filter(surface[tuple(nearest)],size=3,mode='nearest')
    # An isolated tabletop/roof must not become ground just because its own
    # returns form a horizontal surface. Connect to the estimated floor only
    # across small height changes, with nearby measured support at every cell.
    from collections import deque
    supported=distance*cell<=1.
    connected=supported & (np.abs(smooth-floor_z)<=.15)
    pending=deque(map(tuple,np.argwhere(connected)))
    while pending:
        y,x=pending.popleft()
        for dy,dx in ((0,1),(0,-1),(1,0),(-1,0)):
            j,i=y+dy,x+dx
            if (0<=j<smooth.shape[0] and 0<=i<smooth.shape[1] and supported[j,i]
                    and not connected[j,i] and abs(smooth[j,i]-smooth[y,x])<=.20):
                connected[j,i]=True;pending.append((j,i))
    relative=xyz.copy();relative[:,2]-=smooth[xy[:,1],xy[:,0]]
    # Nearest-cell interpolation is only used near actual supporting returns.
    relative[~connected[xy[:,1],xy[:,0]],2]=np.nan
    return relative


def ground_sampling_resolution(floor,minimum=.05):
    """Coordinate navigation cells with measured ground-return spacing.

    A 15 cm voxel map projected into 5 cm cells produces isolated free pixels,
    even on an observed continuous floor. Choose the cell scale from actual
    distinct XY neighbours, without creating additional ground observations.
    A bounded query sample keeps this deterministic; the original points and
    their XY coordinates are never transformed. Sampling alone cannot request
    cells above 25 cm, so very sparse returns do not turn large gaps into free
    space. The separate whole-map raster budget may still require coarser cells.
    """
    from scipy.spatial import cKDTree
    xy=np.unique(np.asarray(floor,dtype=np.float64)[:,:2],axis=0)
    details=dict(method='distinct_ground_xy_nearest_neighbors',requested_resolution_m=float(minimum),
                 unique_ground_samples=len(xy),query_samples=0,sampling_cell_limit_m=.25)
    result=float(minimum)
    if len(xy)>=30:
        sample=xy[np.linspace(0,len(xy)-1,min(8192,len(xy)),dtype=np.int64)]
        tree=cKDTree(xy,leafsize=64,compact_nodes=False,balanced_tree=False)
        distances=tree.query(sample,k=2,workers=1)[0][:,1]
        distances=distances[np.isfinite(distances)&(distances>1e-6)]
        if len(distances)>=30:
            spacing=float(np.quantile(distances,.75))
            # Centroids can vary slightly inside their source voxel. The 5%
            # tolerance avoids promoting a nominal 15 cm lattice to 20 cm due
            # only to millimetre rounding; it never fills a raster cell itself.
            result=max(result,min(.25,round(math.ceil(spacing*.95/.05)*.05,8)))
            details.update(query_samples=len(sample),ground_spacing_p75_m=spacing,
                           ground_spacing_median_m=float(np.median(distances)))
    details['sampling_resolution_m']=result
    return result,details


def observed_floor_grid(points,floor_z,resolution=.05,*,follow_terrain=False,sampling_metadata=None):
    """Only measured floor returns certify free cells; obstacles always win.

    floor_z is in ORIGINAL gravity-aligned map coordinates.
    Cell resolution follows the actual floor sampling scale. There is no axis
    swap, hole filling or global unknown->free operation.
    A single level is supported; stairs/multiple floors need separate maps.
    """
    if type(floor_z) not in (float,int) or not math.isfinite(floor_z) or abs(floor_z)>1000:
        raise ValueError('请填写原始点云中的地面高度')
    xyz=np.asarray(points,dtype=np.float32)
    if xyz.ndim!=2 or xyz.shape[1]!=3:raise ValueError('点云坐标格式无效')
    if follow_terrain:
        xyz=terrain_relative_points(xyz,floor_z)
        floor_z=0.
    finite=np.isfinite(xyz).all(axis=1)
    floor_tolerance=.15 if follow_terrain else .06
    obstacle_height=.20 if follow_terrain else .12
    floor=xyz[finite & (np.abs(xyz[:,2]-floor_z)<=floor_tolerance)]
    if len(floor)<30:raise ValueError('指定地面高度附近观测不足，请检查高度或补充采集')
    if not math.isfinite(resolution) or resolution<=0:raise ValueError('栅格分辨率必须大于0')
    resolution,scale=ground_sampling_resolution(floor,minimum=resolution)
    obstacles=xyz[finite & (xyz[:,2]>=floor_z+obstacle_height) & (xyz[:,2]<=floor_z+2.1)]
    bounds=np.vstack((floor[:,:2],obstacles[:,:2]))
    # Preserve the whole map; large scenes use coarser navigation cells instead
    # of failing a fixed cell budget or allocating an unbounded raster.
    while True:
        low=np.floor((bounds.min(axis=0)-.5)/resolution)*resolution
        high=np.ceil((bounds.max(axis=0)+.5)/resolution)*resolution
        size=np.ceil((high-low)/resolution).astype(np.int64)+1
        if int(size[0])*int(size[1])<=8_000_000:break
        resolution*=1.25
    grid=pointcloud_to_grid(xyz,resolution=resolution,z_min=floor_z+obstacle_height,z_max=floor_z+2.1,
                           free_points=floor,inflation_radius=.1,max_cells=8_000_000)
    if np.count_nonzero(grid.occupancy==0)<20:
        raise ValueError('实测地面不足以生成通道，请检查地面高度和采集覆盖')
    scale['effective_resolution_m']=float(grid.resolution)
    if sampling_metadata is not None:sampling_metadata.update(scale)
    return grid


def estimate_floor(points):
    """Find the broad lower horizontal return band in unchanged LIO XYZ.

    Vote once per 20 cm XY cell so a dense wall cannot outweigh the floor.
    This is a single-level estimate, not plane leveling or a traversability proof.
    """
    xyz=np.asarray(points,dtype=np.float32)
    xyz=xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz)<30:raise ValueError('点云观测太少，地图已保存，可补采后重新处理')
    _, inverse=np.unique(np.floor(xyz[:,:2]/.2).astype(np.int64),axis=0,return_inverse=True)
    low=np.full(int(inverse.max())+1,np.inf)
    np.minimum.at(low,inverse,xyz[:,2])
    # LIO starts at the device; avoid selecting a ceiling above the initial sensor.
    candidates=low[low<=.15]
    if len(candidates)<30:raise ValueError('未找到足够地面观测，请在地图处理中核对地面高度')
    bins,counts=np.unique(np.rint(candidates/.05).astype(np.int64),return_counts=True)
    scores=np.array([counts[np.abs(bins-b)<=1].sum() for b in bins])
    best=int(bins[int(np.argmax(scores))])
    band=candidates[np.abs(candidates-best*.05)<=.075]
    if len(band)<30:raise ValueError('地面观测不连续，地图已保存，可调整地面高度后重试')
    return float(np.median(band))


def scan_context(cloud,log,*,progress=None,cancel=None):
    from .localization_tools import validate_scan_context_cache
    _check_cancel(cancel)
    log=Path(log);log.parent.mkdir(parents=True,exist_ok=True)
    target=Path(str(cloud)+'.sc.v2')
    verification_at=0.
    def report_verification(done,total):
        nonlocal verification_at
        now=time.monotonic()
        if done in (0,total) or now-verification_at>=.25:
            _report(progress,'verify_index',done,total);verification_at=now
    valid=False
    if target.exists():
        valid,result=validate_scan_context_cache(str(cloud),str(target),cancelled=cancel,
            progress=report_verification)
        _check_cancel(cancel)
    if not valid:
        # The tool revalidates an implicit cache and atomically replaces an
        # invalid one only after the rebuilt content is complete. A valid
        # existing cache is reused, and interrupted rebuilds keep the old file.
        with log.open('wb') as stream:
            environment=os.environ.copy();environment['NAV_SC_PROGRESS']='1'
            # Join the task worker's group so cancellation cannot orphan the
            # expensive SC descendant. Standalone calls still reap their child.
            # Execute the actual tool, not the ros2 CLI parent which itself
            # spawns a child that could survive cooperative termination.
            from ament_index_python.packages import get_package_prefix
            executable=Path(get_package_prefix('localizer'))/'lib'/'localizer'/'generate_sc_tool'
            process=subprocess.Popen([str(executable),str(cloud)],
                stdout=stream,stderr=subprocess.STDOUT,env=environment)
            start=time.monotonic()
            # File size bounds initial loading, not the entire spatial search:
            # candidate count/density can vary greatly for equally sized PCDs.
            # Once work advances, only a lack of progress expires the child.
            startup_timeout=max(180.,min(1800.,Path(cloud).stat().st_size/(128*1024)))
            idle_timeout=180.
            tail=b'';progress_total=None;progress_done=-1;advanced_at=None
            def read_progress(reader):
                nonlocal tail,progress_total,progress_done,advanced_at
                tail+=reader.read(64*1024)
                lines=tail.split(b'\n');tail=lines.pop()[-4096:]
                for line in lines:
                    if line.startswith(b'NAV_SC_PROGRESS '):
                        try:done,total=map(int,line.split()[1:])
                        except (ValueError,TypeError):continue
                        if not 0<=done<=total or total<=0:continue
                        # The tool emits one immutable candidate count. A
                        # repeated/regressed count or changed denominator is
                        # not evidence of work and must never renew the lease.
                        if progress_total is None:progress_total=total
                        if total!=progress_total or done<=progress_done:continue
                        progress_done=done
                        if done>0:advanced_at=time.monotonic()
                        _report(progress,'index',done,total)
            try:
                with log.open('rb') as reader:
                    while process.poll() is None:
                        _check_cancel(cancel)
                        read_progress(reader)
                        if os.fstat(stream.fileno()).st_size>8*2**20:
                            raise ValueError('定位索引日志过大，已停止处理；点云与已有索引已保留')
                        now=time.monotonic()
                        if advanced_at is None and now-start>startup_timeout:
                            raise ValueError('定位索引加载超时，尚未开始处理候选位置；点云与已有索引已保留')
                        if advanced_at is not None and now-advanced_at>idle_timeout:
                            if progress_done==progress_total:
                                raise ValueError('定位索引处理完成但保存超时；点云与已有索引已保留')
                            raise ValueError(f'定位索引连续{idle_timeout:g}秒没有新进展（{progress_done}/{progress_total}）；点云与已有索引已保留')
                        time.sleep(.1)
                    read_progress(reader)
                _check_cancel(cancel)
                if process.returncode:raise ValueError('定位索引生成失败，栅格与点云已保留')
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5)
    _check_cancel(cancel)
    valid,result=validate_scan_context_cache(str(cloud),str(target),cancelled=cancel,
        progress=report_verification)
    _check_cancel(cancel)
    if not valid:raise ValueError('定位索引与点云不匹配，已有索引未覆盖')
    return result


def _sync_files(paths):
    for path in paths:
        with Path(path).open('r+b') as stream:os.fsync(stream.fileno())


def _sync_directory(path):
    if os.name=='posix':
        fd=os.open(str(path),os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)


def _valid_outputs(record,root):
    """Treat checkpoints as untrusted metadata, never arbitrary cleanup paths."""
    try:
        if not isinstance(record,dict):return False
        outputs=record['files']
        if not isinstance(outputs,dict) or not outputs:return False
        for name,expected in outputs.items():
            path=(root/name).resolve(strict=True)
            if not path.is_relative_to(root) or not path.is_file() or digest(path)!=expected:return False
        return True
    except (KeyError,TypeError,ValueError,OSError):return False


def build(cloud_path,directory,title,floor_z=None,*,index_builder=scan_context,gravity_aligned=False,
          work_directory=None,progress=None,cancel=None,require_index=False,grid_method="observed"):
    """Build disk-backed stages; interrupted stages restart without redoing grid.

    ``progress`` receives small stage/state/done/total dictionaries. ``cancel``
    is polled between Python batches and while waiting for SC; callers should
    run this function in an isolated process for bounded cancellation of native
    NumPy/PCL calls. The task manager owns that process and its descendants.
    """
    if grid_method not in ("observed", "terrain"):
        raise ValueError("不支持的栅格生成方法")
    cloud=Path(cloud_path).expanduser().resolve(strict=True)
    if cloud.suffix.lower()!='.pcd':raise ValueError('请选择已保存的 PCD 点云')
    title=normalize_scene_title(title);root=Path(directory).resolve();root.mkdir(parents=True,exist_ok=True)
    work=Path(work_directory).expanduser().resolve() if work_directory is not None else root/'.navigation-tasks'/uuid.uuid4().hex
    work.mkdir(parents=True,exist_ok=True)
    state_path=work/'stage.json'
    _check_cancel(cancel)
    with ResourceLease('live'):
        _report(progress,'source')
        before=digest(cloud,cancel=cancel,progress=lambda done,total:_report(progress,'source',done,total))
        identity=dict(version=2,grid_policy_version=GRID_POLICY_VERSION,source=str(cloud),source_sha256=before,directory=str(root),
            title=title,floor_z_m=floor_z,gravity_aligned=bool(gravity_aligned),grid_method=grid_method)
        try:
            state=json.loads(state_path.read_text(encoding='utf-8'))
            if not isinstance(state,dict) or state.get('identity')!=identity:state={}
        except (OSError,ValueError,TypeError):state={}
        if not state:
            state=dict(identity=identity,name=cloud.stem+'-nav-'+uuid.uuid4().hex[:8])
            atomic_json(state_path,state)
        name=state['name']
        if not isinstance(name,str) or Path(name).name!=name or '/' in name or '\\' in name:
            raise ValueError('地图处理检查点无效，请重新准备')
        asset=root/(name+'.yaml');image=asset.with_suffix('.pgm');scene=root/(name+'.scene.json')
        automatic_floor=floor_z is None
        checkpoint=state.get('grid',{})
        _check_cancel(cancel)
        if _valid_outputs(checkpoint,root):
            floor_z=checkpoint['floor_z_m'];resolution=checkpoint['resolution_m']
            grid_sampling=checkpoint['sampling']
            grid_path=asset
            _report(progress,'grid',1,1,'reused')
        else:
            _report(progress,'grid')
            points=load_pcd(cloud)
            _check_cancel(cancel)
            if automatic_floor:floor_z=estimate_floor(points.points)
            grid_sampling={}
            if grid_method == 'terrain':
                from .terrain_grid import build_terrain_grid
                grid, layers, grid_sampling = build_terrain_grid(points.points, floor_z=floor_z, cancel=cancel)
                del layers
            else:
                grid=observed_floor_grid(points.points,floor_z,follow_terrain=automatic_floor,
                                         sampling_metadata=grid_sampling)
            grid.frame_id='map'
            resolution=grid.resolution
            _check_cancel(cancel)
            # The scene is published only after both grid files are durable.
            # Staging beside their destination keeps rename on one filesystem.
            staging=root/('.'+name+'.staging');staging.mkdir(exist_ok=True)
            temporary_grid,temporary_image=save_grid_map(staging/asset.name,grid)
            _sync_files((temporary_grid,temporary_image))
            _check_cancel(cancel)
            os.replace(temporary_image,image);os.replace(temporary_grid,asset);_sync_directory(root)
            try:staging.rmdir()
            except OSError:pass
            grid_path=asset
            state['grid']=dict(files={asset.name:digest(asset),image.name:digest(image)},
                               floor_z_m=floor_z,resolution_m=resolution,sampling=grid_sampling)
            atomic_json(state_path,state)
            # SC loads the source again. Release Python geometry before that
            # child is started instead of keeping two full maps resident.
            del points,grid
            gc.collect()
            _report(progress,'grid',1,1,'complete')
        _check_cancel(cancel)
        if grid_method == 'terrain':
            warning=f"局部高程单元{grid_sampling['surface_resolution_m']*100:g}厘米，输出栅格{resolution*100:g}厘米；未观测区域保留未知。"
        elif resolution>grid_sampling['sampling_resolution_m']+1e-8:
            warning=f'地图范围较大，通行图使用{resolution*100:g}厘米网格；点云保持原始精度。'
        elif resolution>.05+1e-8:
            warning=f'通行图使用{resolution*100:g}厘米网格，与点云实测采样尺度一致；点云保持原始精度。'
        else:warning=''
        sc=''
        index=state.get('index',{})
        try:
            candidate=Path(index.get('path',''))
            if candidate.is_file() and digest(candidate,cancel=cancel)==index.get('sha256'):
                sc=str(candidate)
                if index_builder is scan_context:
                    sc=scan_context(cloud,work/(name+'.index.log'),progress=progress,cancel=cancel)
                _report(progress,'index',1,1,'reused')
            else:
                _report(progress,'index')
                if index_builder is scan_context:
                    sc=str(index_builder(cloud,work/(name+'.index.log'),progress=progress,cancel=cancel))
                else:sc=str(index_builder(cloud,work/(name+'.index.log')))
                _check_cancel(cancel)
                if sc:
                    index_path=Path(sc).resolve(strict=True)
                    state['index']=dict(path=str(index_path),sha256=digest(index_path,cancel=cancel))
                    atomic_json(state_path,state)
                    _report(progress,'index',1,1,'complete')
            if not sc and require_index:raise ValueError('定位索引未生成，通行图已保留，可重新处理')
        except NavigationBuildCancelled:raise
        except (OSError,ValueError,ImportError,subprocess.SubprocessError) as exc:
            if require_index:raise ValueError('定位索引准备失败，通行图已保留：'+str(exc)) from exc
            sc='';warning+='定位索引未生成或未通过核对；可先预览地图，自动重定位仍需索引。'
        _check_cancel(cancel)
        _report(progress,'verify')
        if digest(cloud,cancel=cancel)!=before:raise ValueError('生成期间点云发生变化，请重新选择地图')
        _check_cancel(cancel)
        _report(progress,'register')
        previous=state.get('result',{})
        previous_scene=Path(previous.get('scene','')) if isinstance(previous,dict) else Path()
        if (isinstance(previous,dict) and previous_scene.is_file()
                and previous_scene.resolve().is_relative_to(root)
                and digest(previous_scene)==state.get('scene_sha256')
                and previous.get('scan_context_ready')==bool(sc)):
            _report(progress,'register',1,1,'reused')
            return previous
        if scene.exists():
            # A user may have edited destinations after an earlier successful
            # run. Publish a new manifest rather than resetting their changes.
            scene=root/(name+'-'+uuid.uuid4().hex[:8]+'.scene.json')
        def stored(path):
            path=Path(path).resolve()
            return path.relative_to(root).as_posix() if path.is_relative_to(root) else str(path)
        atomic_json(scene,dict(version=2,title=title,cloud=stored(cloud),grid=stored(grid_path),frame='map',
            coordinates_verified=True,scan_context=stored(sc) if sc else '',scan_context_verified=bool(sc),destinations=[],
            generated=dict(source_sha256=before,grid_sha256=digest(grid_path),floor_z_m=floor_z,
                           floor_estimated=automatic_floor,resolution_m=resolution,coordinates='original_XYZ',
                           grid_policy_version=GRID_POLICY_VERSION,grid_sampling=grid_sampling,grid_method=grid_method,
                           gravity_aligned=bool(gravity_aligned),
                           free_space=('local_surface_traversability' if grid_method == 'terrain' else
                                       'observed_local_floor_cells' if automatic_floor else 'observed_floor_cells'),
                           field_validated=False)))
        result=dict(scene=str(scene),warning=warning,floor_z_m=floor_z,grid_method=grid_method,
                    floor_estimated=automatic_floor,scan_context_ready=bool(sc))
        state['result']=result
        state['scene_sha256']=digest(scene)
        atomic_json(state_path,state)
        _report(progress,'register',1,1,'complete')
        return result
