"""Compress closed SQLite splits from disk with bounded zstd streaming buffers."""
import ctypes as C
import ctypes.util
import hashlib
import os
from pathlib import Path
import shutil
import time
import yaml
from .bag_cache import release_file_cache

BLOCK=128*1024
class Buffer(C.Structure):
    _fields_=[("data",C.c_void_p),("size",C.c_size_t),("pos",C.c_size_t)]


def transform(source, destination=None, *, decode=False, cancelled=None):
    lib=C.CDLL(ctypes.util.find_library("zstd") or "libzstd.so.1")
    for name in ("ZSTD_createCStream","ZSTD_createDStream"):
        getattr(lib,name).restype=C.c_void_p
    for name in ("ZSTD_freeCStream","ZSTD_freeDStream","ZSTD_initDStream"):
        getattr(lib,name).argtypes=[C.c_void_p];getattr(lib,name).restype=C.c_size_t
    lib.ZSTD_initCStream.argtypes=[C.c_void_p,C.c_int];lib.ZSTD_initCStream.restype=C.c_size_t
    lib.ZSTD_CCtx_setPledgedSrcSize.argtypes=[C.c_void_p,C.c_ulonglong]
    lib.ZSTD_CCtx_setPledgedSrcSize.restype=C.c_size_t
    lib.ZSTD_isError.argtypes=[C.c_size_t];lib.ZSTD_isError.restype=C.c_uint
    lib.ZSTD_compressStream2.argtypes=[C.c_void_p,C.POINTER(Buffer),C.POINTER(Buffer),C.c_int]
    lib.ZSTD_compressStream2.restype=C.c_size_t
    lib.ZSTD_decompressStream.argtypes=[C.c_void_p,C.POINTER(Buffer),C.POINTER(Buffer)]
    lib.ZSTD_decompressStream.restype=C.c_size_t
    def check(code):
        if lib.ZSTD_isError(code):raise ValueError("zstd流处理失败")
        return code
    stream=lib.ZSTD_createDStream() if decode else lib.ZSTD_createCStream()
    if not stream:raise MemoryError("zstd stream allocation failed")
    output=None; digest=hashlib.sha256(); remaining=1
    try:
        check(lib.ZSTD_initDStream(stream) if decode else lib.ZSTD_initCStream(stream,1))
        if not decode:check(lib.ZSTD_CCtx_setPledgedSrcSize(stream,Path(source).stat().st_size))
        if destination:
            output=os.fdopen(os.open(destination,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),"wb")
        out_data=C.create_string_buffer(BLOCK)
        with Path(source).open("rb") as src:
            while True:
                if cancelled is not None and cancelled():
                    raise InterruptedError('录包压缩已暂停，原始分片保留')
                data=src.read(BLOCK)
                if not decode:digest.update(data)
                in_data=C.create_string_buffer(data); inp=Buffer(C.cast(in_data,C.c_void_p),len(data),0)
                if decode and not data:
                    if remaining:raise ValueError("zstd文件截断")
                    break
                last_out=0
                while inp.pos<inp.size or (decode and last_out==BLOCK) or not data:
                    out=Buffer(C.cast(out_data,C.c_void_p),BLOCK,0)
                    remaining=check(lib.ZSTD_decompressStream(stream,C.byref(out),C.byref(inp)) if decode else
                                    lib.ZSTD_compressStream2(stream,C.byref(out),C.byref(inp),2 if not data else 0))
                    block=out_data.raw[:out.pos]
                    last_out=out.pos
                    if decode:digest.update(block)
                    if output:output.write(block)
                    if decode and remaining==0 and inp.pos==inp.size:break
                    if not data and remaining==0:break
                if not data:break
        if output:output.flush();os.fsync(output.fileno())
        return digest.hexdigest()
    finally:
        if output:output.close()
        (lib.ZSTD_freeDStream if decode else lib.ZSTD_freeCStream)(stream)


def compress_closed_bag(root, progress=None, cancelled=None):
    """Call only after uncompressed readback audit passed. Preserve originals
    until all compressed streams decode to the original SHA256 and metadata is
    atomically committed. Existing user bags are never called through this path.
    """
    root=Path(root).resolve(); path=root/"metadata.yaml"
    metadata=yaml.safe_load(path.read_text());info=metadata["rosbag2_bagfile_information"]
    if str(info.get("compression_mode","")).lower() not in ("","none"):
        raise ValueError("拒绝重复压缩已有录包")
    originals=[]; result=[]
    for name in info["relative_file_paths"]:
        if cancelled is not None and cancelled():
            raise InterruptedError('录包压缩已暂停，原始分片保留')
        source=(root/name).resolve()
        if root not in source.parents or source.is_symlink():raise ValueError("录包分片路径无效")
        if shutil.disk_usage(root).free < source.stat().st_size+256*1024**2:
            raise OSError("磁盘不足，原始分片已保留，未完成压缩")
        target=source.with_name(source.name+".zstd")
        partial=target.with_name(target.name+".partial")
        if target.exists():
            digest=hashlib.sha256()
            with source.open("rb") as stream:
                for block in iter(lambda:stream.read(BLOCK),b""):
                    if cancelled is not None and cancelled():
                        raise InterruptedError('录包压缩已暂停，原始分片保留')
                    digest.update(block)
            expected=digest.hexdigest()
            if transform(target,decode=True,cancelled=cancelled)!=expected:raise ValueError("已有压缩分片与原始数据不一致")
        else:
            if partial.exists():partial.rename(partial.with_name(partial.name+f".failed-{time.time_ns()}"))
            try:
                expected=transform(source,partial,cancelled=cancelled)
                if transform(partial,decode=True,cancelled=cancelled)!=expected:raise ValueError("压缩文件读回校验失败，原始分片已保留")
            except InterruptedError:
                partial.unlink(missing_ok=True)
                raise
            os.replace(partial,target)
        originals.append(source)
        result.append(dict(source=name,compressed=name+".zstd",raw_sha256=expected,
                           raw_bytes=source.stat().st_size,compressed_bytes=target.stat().st_size))
        release_file_cache(source)
        release_file_cache(target)
        if progress is not None:
            progress(len(result), len(info["relative_file_paths"]))
    if cancelled is not None and cancelled():
        raise InterruptedError('录包压缩已暂停，原始分片保留')
    # From this point finish the short metadata commit before honoring another
    # cancellation; otherwise readers could reference a removed original.
    info["relative_file_paths"]=[row["compressed"] for row in result]
    for entry in info.get("files",[]):entry["path"] += ".zstd"
    info["compression_format"]="zstd";info["compression_mode"]="FILE"
    temporary=path.with_name("metadata.yaml.compress-tmp")
    with temporary.open("w") as stream:
        yaml.safe_dump(metadata,stream,sort_keys=False);stream.flush();os.fsync(stream.fileno())
    os.replace(temporary,path)
    directory=os.open(root,os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(directory)
    finally:os.close(directory)
    for source in originals:source.unlink()
    return dict(mode="file",format="zstd",stream_buffer_bytes=BLOCK,verified_splits=result)
