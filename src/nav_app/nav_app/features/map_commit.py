"""Bounded verification of a FAST-LIVO map's durable commit receipt.

Do not infer success from a PCD filename, size, or timestamp alone. The native
writer publishes a receipt only AFTER committing the complete output. No point
payload is loaded here; this is a commit/structure check, not a geometry audit.
"""
import json
import hashlib
import os
from pathlib import Path
import stat


def _regular(path):
    value = path.lstat()
    if not stat.S_ISREG(value.st_mode):
        raise ValueError('not a regular file')
    return value


def _identity(value):
    return value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns


def receipt_snapshot(output_path, legacy_root=None):
    """Freeze existing receipts, without comparing kernel times to wall clock."""
    candidates = [Path(str(output_path) + '.commit.json')]
    if legacy_root is not None:
        candidates.extend(Path(legacy_root).glob('capture-*/exported.txt'))
    snapshot = {}
    for path in candidates:
        try:snapshot[str(path)] = _identity(path.lstat())
        except FileNotFoundError:pass
        except OSError:snapshot[str(path)] = None  # Unknown old receipt cannot prove a new commit.
    return snapshot


def _read_small(path, limit=16384):
    before = _regular(path)
    if before.st_size > limit:
        raise ValueError('receipt too large')
    with path.open('rb') as stream:
        if _identity(os.fstat(stream.fileno())) != _identity(before):
            raise ValueError('receipt changed while opening')
        value = stream.read(limit + 1)
    if len(value) > limit or _identity(path.lstat()) != _identity(before):
        raise ValueError('receipt changed while reading')
    return value


def binary_pcd_dimensions(path):
    """Validate the exact binary layout emitted by MapSpool.exportPcd()."""
    path = Path(path)
    before = _regular(path)
    with path.open('rb') as stream:
        opened = os.fstat(stream.fileno())
        if (opened.st_dev, opened.st_ino, opened.st_size, opened.st_mtime_ns) != (
                before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns):
            raise ValueError('PCD changed while opening')
        header = {}
        for _ in range(32):
            line = stream.readline(1025)
            if len(line) > 1024 or not line.endswith(b'\n'):
                raise ValueError('invalid PCD header')
            fields = line.decode('ascii').strip().split()
            if not fields or fields[0].startswith('#'):
                continue
            key, values = fields[0], fields[1:]
            if key in header:
                raise ValueError('duplicate PCD field')
            header[key] = values
            if key == 'DATA':
                break
        expected = dict(FIELDS=['x', 'y', 'z', 'intensity'], SIZE=['4'] * 4,
                        TYPE=['F'] * 4, COUNT=['1'] * 4, HEIGHT=['1'], DATA=['binary'])
        if any(header.get(key) != value for key, value in expected.items()):
            raise ValueError('not a native binary map')
        points = int(header['POINTS'][0])
        if header['POINTS'] != [str(points)] or header.get('WIDTH') != [str(points)] or points <= 0:
            raise ValueError('invalid map point count')
        if before.st_size != stream.tell() + points * 16:
            raise ValueError('PCD payload size does not match point count')
        after = path.lstat()
        if (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns) != (
                before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns):
            raise ValueError('PCD changed while checking')
    return dict(bytes=before.st_size, points=points, file_fingerprint=list(_identity(before)))


def committed_map_receipt(output_path, *, prior_receipts, legacy_root=None):
    """Return evidence for THIS pending new output, or None (never a glob map)."""
    path = Path(output_path)
    try:
        dimensions = binary_pcd_dimensions(path)
        receipt_path = Path(str(path) + '.commit.json')
        if receipt_path.exists() or receipt_path.is_symlink():
            receipt_before = _identity(_regular(receipt_path))
            raw = _read_small(receipt_path)
            receipt = json.loads(raw)
            if not isinstance(receipt, dict):
                return None
            if (str(receipt_path) in prior_receipts or type(receipt.get('schema_version')) is not int
                    or receipt.get('schema_version') != 1 or receipt.get('output_path') != str(path)
                    or receipt.get('bytes') != dimensions['bytes'] or receipt.get('points') != dimensions['points']
                    or not isinstance(receipt.get('source_spool'), str)
                    or not Path(receipt['source_spool']).is_absolute()
                    or any(type(receipt.get(key)) is not int or receipt[key] < 0
                           for key in ('bytes', 'points', 'checkpoint_chunks', 'archive_generation'))):
                return None
            if (list(_identity(_regular(path))) != dimensions['file_fingerprint']
                    or _identity(_regular(receipt_path)) != receipt_before):
                return None
            return dict(source='native_commit_receipt', receipt=str(receipt_path), **dimensions,
                        source_spool=receipt['source_spool'], checkpoint_chunks=receipt['checkpoint_chunks'],
                        archive_generation=receipt['archive_generation'],
                        receipt_fingerprint=list(receipt_before), receipt_sha256=hashlib.sha256(raw).hexdigest())
        # Legacy receipt belongs to the exact unique requested path. A later
        # checkpoint may remove it; absence is not permission to guess success.
        if legacy_root is not None:
            for directory in Path(legacy_root).glob('capture-*'):
                if directory.is_symlink() or not directory.is_dir():
                    continue
                receipt_path = directory / 'exported.txt'
                try:
                    if str(receipt_path) in prior_receipts and prior_receipts[str(receipt_path)] is None:
                        continue
                    if (_read_small(receipt_path).decode('utf-8').strip() == str(path)
                            and prior_receipts.get(str(receipt_path)) != _identity(receipt_path.stat())):
                        return dict(source='legacy_export_receipt', receipt=str(receipt_path), **dimensions)
                except (OSError, ValueError, UnicodeError):
                    continue
    except (OSError, ValueError, KeyError, IndexError, TypeError, UnicodeError, RecursionError):
        pass
    return None
