"""Local terrain analysis of upstream grid_map_pcl elevation, in unchanged XY."""
from collections import deque
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import Delaunay
from ..map_processing import GridMap


def terrain_grid(points, columns, *, surface_resolution=.15, resolution=.05,
                 ground_z, max_slope_degrees=20., max_step=.10,
                 max_roughness=.04, min_obstacle_height=.12,
                 clearance_height=1.5, max_gap=.45):
    """Estimate traversability; short supported surface gaps are explicit inference.

    No rigid transform is applied. Empty regions beyond short interpolation
    triangles stay unknown; original cloud returns, not the inferred surface,
    mark obstacles in the robot's vertical clearance band.
    """
    if not all(math.isfinite(v) and v > 0 for v in (surface_resolution, resolution,
            max_step, max_roughness, min_obstacle_height, clearance_height, max_gap)):
        raise ValueError('地形分析的分辨率、距离和高度参数必须为正数')
    if not math.isfinite(ground_z) or not 0 < max_slope_degrees < 90 or min_obstacle_height >= clearance_height:
        raise ValueError('地形分析的地面参考、坡度或净空范围无效')
    xyz = np.asarray(points, dtype=np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    columns = np.asarray(columns, dtype=np.float64)
    low_center = columns[:, :2].min(axis=0)
    shape_xy = np.rint((columns[:, :2].max(axis=0)-low_center)/surface_resolution).astype(int)+1
    ij = np.rint((columns[:, :2]-low_center)/surface_resolution).astype(int)
    elevation = np.full((shape_xy[1], shape_xy[0]), np.nan)
    elevation[ij[:, 1], ij[:, 0]] = columns[:, 2]
    raw_elevation = elevation.copy()
    # Select one building level; high ceiling columns are not ground anchors.
    elevation[elevation > ground_z+clearance_height] = np.nan
    measured = np.isfinite(elevation)
    if measured.sum() < 6:
        raise ValueError('局部高程观测不足，请降低高程单元最少点数或增大高程分辨率')
    # Interpolate only inside short triangles of actual measured columns.
    measured_yx = np.argwhere(measured)
    missing_yx = np.argwhere(~measured)
    triangulation = Delaunay(measured_yx*surface_resolution)
    simplex = triangulation.find_simplex(missing_yx*surface_resolution)
    inside = simplex >= 0
    query_yx = missing_yx[inside]
    simplex = simplex[inside]
    vertices = triangulation.simplices[simplex]
    triangles = measured_yx[vertices]*surface_resolution
    edges = np.maximum.reduce([np.linalg.norm(triangles[:, 0]-triangles[:, 1], axis=1),
                               np.linalg.norm(triangles[:, 1]-triangles[:, 2], axis=1),
                               np.linalg.norm(triangles[:, 0]-triangles[:, 2], axis=1)])
    short = edges <= max_gap
    query_yx, simplex, vertices = query_yx[short], simplex[short], vertices[short]
    transform = triangulation.transform[simplex]
    bary = np.einsum('nij,nj->ni', transform[:, :2], query_yx*surface_resolution-transform[:, 2])
    weights = np.c_[bary, 1-bary.sum(axis=1)]
    values = elevation[measured_yx[:, 0], measured_yx[:, 1]]
    elevation[query_yx[:, 0], query_yx[:, 1]] = np.sum(values[vertices]*weights, axis=1)
    interpolated = np.isfinite(elevation) & ~measured
    slope = np.full_like(elevation, np.nan)
    roughness = np.full_like(elevation, np.nan)
    drop = np.zeros_like(measured)
    tangent = math.tan(math.radians(max_slope_degrees))
    # Local planes exclude distant-height wall returns from ground normals.
    for y, x in np.argwhere(np.isfinite(elevation)):
        y0, y1 = max(0, y-1), min(len(elevation), y+2)
        x0, x1 = max(0, x-1), min(elevation.shape[1], x+2)
        yy, xx = np.mgrid[y0:y1, x0:x1]
        dx, dy = (xx-x)*surface_resolution, (yy-y)*surface_resolution
        z = elevation[y0:y1, x0:x1]
        finite = np.isfinite(z)
        close = finite & (np.abs(z-elevation[y, x]) <= max_step+np.hypot(dx, dy)*tangent)
        if close.sum() < 4:
            continue
        design = np.c_[dx[close], dy[close], np.ones(close.sum())]
        plane, _, rank, _ = np.linalg.lstsq(design, z[close], rcond=None)
        if rank < 3:
            continue
        slope[y, x] = math.degrees(math.atan(math.hypot(plane[0], plane[1])))
        roughness[y, x] = np.sqrt(np.mean((z[close]-design@plane)**2))
        drop[y, x] = np.any(finite & (z < elevation[y, x]-max_step-np.hypot(dx, dy)*tangent))
    candidate = (np.isfinite(slope) & (slope <= max_slope_degrees)
                 & (roughness <= max_roughness) & ~drop)
    # The reference starts a ground component; it is NOT a global height cut.
    ground = candidate & (np.abs(elevation-ground_z) <= max_step)
    pending = deque(map(tuple, np.argwhere(ground)))
    while pending:
        y, x = pending.popleft()
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            j, i = y+dy, x+dx
            if (0 <= j < ground.shape[0] and 0 <= i < ground.shape[1]
                    and candidate[j, i] and not ground[j, i]
                    and abs(elevation[j, i]-elevation[y, x]) <= max_step):
                ground[j, i] = True
                pending.append((j, i))
    if not ground.any():
        raise ValueError('没有与参考高度连通的可通行地面，请核对地面参考或地形参数')
    origin = low_center-surface_resolution/2
    width, height = np.ceil(shape_xy*(surface_resolution/resolution)).astype(int)
    if int(width)*int(height) > 25_000_000:
        raise ValueError('输出栅格超过2500万格，请增大栅格分辨率')
    fine_y, fine_x = np.mgrid[:height, :width]
    coarse_x = np.minimum(((fine_x+.5)*resolution/surface_resolution).astype(int), shape_xy[0]-1)
    coarse_y = np.minimum(((fine_y+.5)*resolution/surface_resolution).astype(int), shape_xy[1]-1)
    occupancy = np.full((height, width), -1, dtype=np.int8)
    occupancy[ground[coarse_y, coarse_x]] = 0
    distance, nearest = distance_transform_edt(~ground, sampling=surface_resolution, return_indices=True)
    nearest_ground = elevation[tuple(nearest)]
    point_cells = np.floor((xyz[:, :2]-origin)/surface_resolution).astype(int)
    inside = ((point_cells >= 0) & (point_cells < shape_xy)).all(axis=1)
    raw, point_cells = xyz[inside], point_cells[inside]
    px, py = point_cells[:, 0], point_cells[:, 1]
    relative = raw[:, 2]-nearest_ground[py, px]
    obstacles = ((distance[py, px] <= .6) & (relative >= min_obstacle_height)
                 & (relative <= clearance_height))
    # Preserve directly observed vertical obstacles even where the floor is
    # missing. Ceiling-only returns do not certify an obstacle at robot height.
    level = ((raw[:, 2] >= ground_z+min_obstacle_height)
             & (raw[:, 2] <= ground_z+clearance_height))
    lowest = np.full_like(elevation, np.inf)
    highest = np.full_like(elevation, -np.inf)
    counts = np.zeros(elevation.shape, dtype=np.int32)
    np.minimum.at(lowest, (py[level], px[level]), raw[level, 2])
    np.maximum.at(highest, (py[level], px[level]), raw[level, 2])
    np.add.at(counts, (py[level], px[level]), 1)
    vertical = (counts >= 3) & (highest-lowest >= .30)
    obstacles |= level & vertical[py, px]
    fine_cells = np.floor((raw[obstacles, :2]-origin)/resolution).astype(int)
    occupancy[fine_cells[:, 1], fine_cells[:, 0]] = 100
    grid = GridMap(occupancy, resolution, np.array([*origin, 0.]))
    layers = dict(raw_elevation=raw_elevation, elevation=elevation, measured=measured, interpolated=interpolated,
                  ground=ground, slope_degrees=slope, roughness=roughness,
                  surface_origin=origin, surface_resolution=np.array(surface_resolution))
    metadata = dict(method='grid_map_pcl_local_terrain', coordinates='unchanged_XYZ',
                    surface_resolution_m=surface_resolution, resolution_m=resolution,
                    ground_reference_z_m=ground_z, max_slope_degrees=max_slope_degrees,
                    max_step_m=max_step, max_roughness_m=max_roughness,
                    obstacle_height_m=[min_obstacle_height, clearance_height],
                    selected_level_max_elevation_m=ground_z+clearance_height,
                    max_interpolation_triangle_edge_m=max_gap,
                    measured_surface_cells=int(measured.sum()),
                    interpolated_surface_cells=int(interpolated.sum()),
                    connected_ground_cells=int(ground.sum()),
                    unsupported_obstacles="observed_vertical_structure_in_reference_level",
                    occupied_cells=int(np.count_nonzero(occupancy == 100)),
                    free_cells=int(np.count_nonzero(occupancy == 0)),
                    unknown_cells=int(np.count_nonzero(occupancy == -1)))
    return grid, layers, metadata


def build_terrain_grid(points, *, resolution=.05, floor_z=None, extractor=None,
                       surface_resolution=.20, max_slope_degrees=20., max_step=.10,
                       max_roughness=.04, max_gap=.45, cancel=None):
    """Run official grid_map_pcl on complete edited XYZ, then analyze terrain."""
    import os
    from pathlib import Path
    import subprocess
    import tempfile
    import time
    import yaml
    from ..map_processing import save_pcd
    from .navigation_bundle import estimate_floor, _check_cancel
    xyz = np.asarray(points, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not len(xyz) or not np.isfinite(xyz).all():
        raise ValueError('局部地形分析需要有限的非空 N×3 点云')
    if not .03 <= surface_resolution <= 1. or not .01 <= resolution <= 1.:
        raise ValueError('高程分辨率须为0.03～1米，栅格分辨率须为0.01～1米')
    if np.prod(np.ceil(np.ptp(xyz[:, :2], axis=0)/surface_resolution)+1) > 2_000_000:
        raise ValueError('高程图超过200万格，请增大高程分辨率')
    root = Path(__file__).resolve().parents[4]
    executable = Path(extractor or os.environ.get('NAV_GRID_MAP_EXTRACTOR',
                      root/'build/nav_grid_map_preview/nav_grid_map_pcl_export')).expanduser().resolve()
    if not executable.is_file():
        raise ValueError('局部高程工具尚未构建，请按 tools/grid_map_preview/README.md 安装构建')
    parameters = dict(pcl_grid_map_extraction=dict(num_processing_threads=2,
        cloud_transform=dict(translation=dict(x=0., y=0., z=0.), rotation=dict(r=0., p=0., y=0.)),
        cluster_extraction=dict(cluster_tolerance=.08, min_num_points=1, max_num_points=1000000),
        outlier_removal=dict(is_remove_outliers=False, mean_K=10, stddev_threshold=1.),
        downsampling=dict(is_downsample_cloud=False, voxel_size=dict(x=.02, y=.02, z=.02)),
        grid_map=dict(min_num_points_per_cell=1, resolution=surface_resolution, height_type=0, height_thresh=1.)))
    environment = os.environ.copy()
    prefix = Path(environment.get('NAV_GRID_PREFIX', Path.home()/'nav_dependencies/grid_map/opt/ros/humble'))
    environment['LD_LIBRARY_PATH'] = str(prefix/'lib')+os.pathsep+environment.get('LD_LIBRARY_PATH', '')
    _check_cancel(cancel)
    with tempfile.TemporaryDirectory(prefix='nav-grid-map-pcl-') as temporary:
        directory = Path(temporary)
        source = save_pcd(directory/'edited.pcd', xyz)
        config = directory/'parameters.yaml'; config.write_text(yaml.safe_dump(parameters))
        csv = directory/'elevation.csv'; log_path = directory/'extractor.log'
        with log_path.open('w') as log:
            process = subprocess.Popen([str(executable), str(source), str(config), str(csv)],
                                       stdout=log, stderr=subprocess.STDOUT, env=environment)
            try:
                while process.poll() is None:
                    _check_cancel(cancel)
                    time.sleep(.05)
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait()
        if process.returncode:
            raise ValueError('局部高程提取失败：'+log_path.read_text()[-3000:])
        columns = np.loadtxt(csv, delimiter=',', skiprows=1)
    _check_cancel(cancel)
    reference = estimate_floor(xyz) if floor_z is None else floor_z
    grid, layers, metadata = terrain_grid(xyz, columns, surface_resolution=surface_resolution,
        resolution=resolution, ground_z=reference, max_slope_degrees=max_slope_degrees,
        max_step=max_step, max_roughness=max_roughness, max_gap=max_gap)
    metadata.update(floor_reference_estimated=floor_z is None, grid_map_pcl_parameters=parameters)
    _check_cancel(cancel)
    return grid, layers, metadata
