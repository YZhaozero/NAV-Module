"""Bounded inventory and explicitly confirmed deletion of recording folders."""
from dataclasses import dataclass
from pathlib import Path
import json
import os
import shutil
import stat
from .rosbag import read_bag_metadata


@dataclass
class BagEntry:
    path: Path
    size_bytes: int
    modified: float
    duration_seconds: float = 0.
    messages: int = 0
    topics: tuple = ()
    status: str = "未校验"
    detail: str = ""
    can_load: bool = False
    started: float = 0.
    recording_profile: str = 'full'


def inspect_bag(path, files):
    entry = BagEntry(path, sum(f.stat().st_size for f in files if not f.is_symlink()), path.stat().st_mtime)
    try:
        meta = read_bag_metadata(path)
        entry.duration_seconds, entry.messages, entry.topics = meta.duration_seconds, meta.message_count, meta.topics
        entry.recording_profile = meta.recording_profile
        entry.started = meta.starting_time_ns / 1e9
        entry.can_load = True
        document = json.loads((path/'nav_audit.json').read_text()) if (path/'nav_audit.json').is_file() else {}
        manifest = json.loads((path/'nav_manifest.json').read_text()) if (path/'nav_manifest.json').is_file() else {}
        if not document:
            document = manifest.get('audit', {})
        if document:
            entry.status = '校验通过' if document.get('complete') else '数据不完整'
            entry.detail = '\n'.join(str(e) for e in document.get('errors', ()))
        else:
            entry.detail = '已有录包目录信息；尚无完整性校验结果。'
        from .capture_quality import capture_quality_warnings
        if entry.recording_profile == 'keyframes':
            warnings = list(document.get('warnings', []))
        else:
            warnings = capture_quality_warnings(document, manifest.get('capture_protocol', {})) if document else []
        if warnings:
            if entry.status == '校验通过': entry.status = '校验通过，有质量提示'
            entry.detail += '\n' + '\n'.join(warnings)
        if entry.recording_profile == 'keyframes':
            entry.detail = '旧格式稀疏关键帧数据，无法重跑原始建图。\n' + entry.detail
        if manifest.get('state') == 'stored' and manifest.get('storage_flush_confirmed'):
            entry.status, entry.can_load = '已保存，等待校验', False
            entry.detail = '采集数据已完成落盘，等待后台校验；可在录包列表中选择校验。'
            if manifest.get('storage_errors'):
                entry.detail += '\n保存时出现问题：'+'；'.join(map(str,manifest['storage_errors']))
        elif manifest.get('state') == 'incomplete' and manifest.get('storage_errors'):
            entry.status = '采集有异常，数据已保留'
            entry.detail += '\n'+'；'.join(map(str,manifest['storage_errors']))
        if manifest.get('state') in ('checking', 'recording', 'running', 'flushing', 'auditing', 'compressing'):
            entry.status, entry.can_load = '正在录制或整理', False
            entry.detail = '录包仍在写入或整理，结束后可校验并加载回放。'
    except (OSError, ValueError, TypeError, AttributeError, KeyError) as exc:
        entry.status = '需要检查'
        entry.detail = '无法读取完整录包信息：' + str(exc)
    return entry


def scan_bags(roots, active_path=None):
    rows, notices, seen = [], [], set()
    visited = 0
    for raw in roots:
        root = Path(raw).expanduser()
        if not root.is_dir(): continue
        for directory, dirs, names in os.walk(root, followlinks=False,
                                            onerror=lambda e: notices.append(str(e))):
            visited += 1
            if visited > 2000:
                notices.append('目录较多，已停止扫描；请选择更具体的录包文件夹。')
                return sorted(rows, key=lambda r:r.modified, reverse=True), notices
            path = Path(directory)
            dirs[:] = [d for d in dirs if not (path/d).is_symlink() and not d.startswith('.')]
            if len(path.relative_to(root).parts) >= 3: dirs[:] = []
            if not ('metadata.yaml' in names or any(n.endswith(('.db3','.mcap','.bag','.db3.zstd','.mcap.zstd')) for n in names)):
                continue
            resolved = path.resolve()
            if resolved in seen: continue
            seen.add(resolved)
            dirs[:] = []
            try:
                row = inspect_bag(resolved, [path/n for n in names])
                if active_path and resolved == Path(active_path).resolve():
                    row.status, row.can_load = '正在录制或整理', False
                rows.append(row)
            except OSError as exc:
                notices.append(str(exc))
    return sorted(rows, key=lambda r:r.modified, reverse=True), notices


def disk_summary(path):
    parent = Path(path).expanduser()
    while not parent.exists() and parent != parent.parent: parent = parent.parent
    usage = shutil.disk_usage(parent)
    return f'保存磁盘剩余 {usage.free/1024**3:.1f} / {usage.total/1024**3:.1f} GiB'


@dataclass(frozen=True)
class BagDeletion:
    path: Path
    roots: tuple
    fingerprint: tuple
    size_bytes: int


def _plain_directory(path):
    """Reject links/junctions in the full path, before resolving a delete target."""
    path = Path(os.path.abspath(Path(path).expanduser()))
    for part in (path, *path.parents):
        info = part.lstat()
        if (stat.S_ISLNK(info.st_mode)
                or getattr(info, 'st_file_attributes', 0) & 0x400):
            raise ValueError('目录包含链接，不能从录包管理删除')
    if not path.is_dir():
        raise ValueError('录包目录不存在')
    return path.resolve()


def prepare_bag_deletion(path, roots):
    """Snapshot exactly one bag, never a library root or linked data tree."""
    target = _plain_directory(path)
    allowed = tuple(Path(p).expanduser().resolve() for p in roots)
    if (target in allowed or target == Path.home().resolve()
            or not any(root in target.parents and root != Path(root.anchor) for root in allowed)):
        raise ValueError('只能删除录包目录内的单个录包，不能删除保存根目录')
    files = tuple(target.iterdir())
    if not any(p.is_file() and p.name.endswith(('.db3', '.mcap', '.bag', '.db3.zstd', '.mcap.zstd')) for p in files):
        raise ValueError('没有识别到录包分片，不能删除此目录')
    signature, size = [], 0
    for directory, dirs, names in os.walk(target, followlinks=False):
        for item in (Path(directory), *(Path(directory)/n for n in sorted(names)),
                     *(Path(directory)/n for n in sorted(dirs))):
            info = item.lstat()
            if item.name.endswith(('.pcd', '.ply', '.scene.json')):
                raise ValueError('录包目录内还有地图资产，请由维护人员分开整理后再删除')
            if (stat.S_ISLNK(info.st_mode)
                    or getattr(info, 'st_file_attributes', 0) & 0x400
                    or not (stat.S_ISREG(info.st_mode) or stat.S_ISDIR(info.st_mode))):
                raise ValueError('录包含链接或特殊文件，请由维护人员检查')
            if item != target and item.is_dir() and (item/'metadata.yaml').exists():
                raise ValueError('目录中包含其他录包，不能整体删除')
            # Directories appear once as children and once as traversal roots.
            if item != Path(directory) and stat.S_ISDIR(info.st_mode):
                continue
            signature.append((str(item.relative_to(target)), info.st_dev, info.st_ino,
                              info.st_mode, info.st_size, info.st_mtime_ns))
            if stat.S_ISREG(info.st_mode):
                size += info.st_size
            if len(signature) > 10000:
                raise ValueError('录包目录文件过多，请由维护人员检查')
    return BagDeletion(target, allowed, tuple(sorted(signature)), size)


def delete_bag(plan):
    """Revalidate the confirmed snapshot; caller must keep record/replay idle."""
    current = prepare_bag_deletion(plan.path, plan.roots)
    if current.fingerprint != plan.fingerprint:
        raise ValueError('录包内容已变化，请刷新后重新确认删除')
    # Target is absolute, verified within a recording root, and contains no links.
    # This is an application action, never a shell command assembled from a path.
    try:
        shutil.rmtree(current.path)
    except OSError as exc:
        raise OSError('删除未完成，请刷新检查剩余文件：' + str(exc)) from exc
    return current.size_bytes
