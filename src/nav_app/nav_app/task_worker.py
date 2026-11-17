"""Non-GUI, non-subscribing map task worker; invoked only by the coordinator."""
import argparse
import json
from pathlib import Path
import signal
import time
import os
import threading

from .features.capture_quality import atomic_json


def execute(spec, progress, cancelled):
    kind, data = spec['kind'], spec['payload']
    work = spec['work_directory']
    from .features.task_storage import task_storage_reason
    storage_error = task_storage_reason(kind, data, work)
    if storage_error:
        raise OSError(storage_error)
    if kind in ('navigation', 'scan_context') and 'source_revision' in data:
        from .features.task_coordinator import source_revision
        current = source_revision(data['cloud'])
        if current is None or current != data['source_revision']:
            raise ValueError('所选点云已更换或删除，请重新选择地图后处理')
    if kind == 'navigation':
        from .features.navigation_bundle import build, digest
        from .features.task_coordinator import source_revision
        preserved = None
        source_scene = Path(data['preserve_scene']) if data.get('preserve_scene') else None
        def navigation_cancelled():
            if source_scene is not None and source_revision(source_scene) != data.get('preserve_scene_revision'):
                raise ValueError('原地图设置已变化，请重新选择地图后准备')
            return cancelled()
        if source_scene is not None:
            navigation_cancelled()
            if source_scene.stat().st_size > 2*1024**2:
                raise ValueError('原地图设置过大，请检查地图文件')
            preserved = json.loads(source_scene.read_text(encoding='utf-8'))
            if not isinstance(preserved,dict):raise ValueError('原地图设置不完整，请重新选择')
        value = build(data['cloud'], data['directory'], data['title'], data.get('floor_z'),
                     gravity_aligned=data.get('gravity_aligned',False), work_directory=work,
                     progress=progress, cancel=navigation_cancelled, require_index=True,
                     grid_method=data.get("grid_method", "observed"))
        if preserved is not None:
            if navigation_cancelled():raise InterruptedError('地图处理已暂停')
            scene = Path(value['scene'])
            if scene.resolve() == source_scene.resolve():raise ValueError('新版地图不能覆盖原地图文件')
            document = json.loads(scene.read_text(encoding='utf-8'))
            for key in ('destinations','viewer_url'):
                if key in preserved:document[key] = preserved[key]
            atomic_json(scene,document)
            # Preserve staged reuse after adding the original user metadata.
            checkpoint = Path(work)/'stage.json'
            if checkpoint.is_file():
                state = json.loads(checkpoint.read_text(encoding='utf-8'))
                if (state.get('result') or {}).get('scene') == str(scene):
                    state['scene_sha256'] = digest(scene)
                    atomic_json(checkpoint,state)
        return value
    if kind == 'scan_context':
        from .features.navigation_bundle import scan_context
        path = scan_context(Path(data['cloud']), Path(work).parent/'index.log',cancel=cancelled,progress=progress)
        return dict(path=str(path))
    if kind == 'audit':
        from .features.bag_audit import finalize_recording
        return finalize_recording(data['directory'], stage_progress=lambda stage,done,total: progress(dict(stage=stage,done=done,total=total)), cancelled=cancelled)
    if kind == 'recovery':
        from .features.map_recovery import recover_checkpoint
        return dict(map=recover_checkpoint(data['source'],data['directory']))
    raise ValueError('不支持的地图处理任务')


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--spec',required=True)
    parser.add_argument('--owner-pid',type=int,default=0)
    parser.add_argument('--owner-start',default='')
    args = parser.parse_args(); path = Path(args.spec); stop = False; last = 0.; storage_at = 0.
    spec = json.loads(path.read_text(encoding='utf-8'))
    def request_stop(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    finished = threading.Event()
    def owner_watch():
        while not finished.wait(1):
            try:
                os.kill(args.owner_pid,0)
                if args.owner_start:
                    identity=Path(f'/proc/{args.owner_pid}/stat').read_text().rsplit(')',1)[1].split()[19]
                    if identity!=args.owner_start:raise ProcessLookupError()
            except (ProcessLookupError,FileNotFoundError):
                os.kill(os.getpid(),signal.SIGTERM)
                if not finished.wait(5) and os.name=='posix':os.killpg(os.getpgrp(),signal.SIGKILL)
                return
            except PermissionError:pass
    if args.owner_pid:threading.Thread(target=owner_watch,name='task-owner',daemon=True).start()
    def progress(value):
        nonlocal last, storage_at
        if stop: raise InterruptedError('后台处理等待继续')
        if time.monotonic()-storage_at >= 1.:
            from .features.task_storage import task_storage_reason
            error = task_storage_reason(spec['kind'], spec['payload'], spec['work_directory'], temporary=False)
            if error: raise OSError(error)
            storage_at = time.monotonic()
        if time.monotonic()-last >= .3 or value.get('state') in ('complete','succeeded'):
            atomic_json(path.parent/'progress.json',value); last = time.monotonic()
        if stop: raise InterruptedError('后台处理等待继续')
    try:
        value = execute(spec,progress,lambda:stop)
        atomic_json(path.parent/'result.json',dict(ok=True,value=value))
        return 0
    except Exception as exc:
        atomic_json(path.parent/'result.json',dict(ok=False,error=str(exc)))
        return 2
    finally:finished.set()


if __name__ == '__main__': raise SystemExit(main())
