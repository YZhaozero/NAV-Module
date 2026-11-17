#!/usr/bin/env python3
"""Extract the manufacturer's ARM64 bundle into a user-owned ROS overlay."""
import argparse
import hashlib
import platform
from pathlib import Path
import subprocess
import tempfile
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--prefix", type=Path, default=Path.home()/"nav_dependencies/woosh-agent-0.0.7")
    args = parser.parse_args()
    if platform.machine() not in ("aarch64", "arm64"):
        parser.error("厂家安装包仅支持 ARM64，不能安装到当前架构")
    checksum = hashlib.sha256(args.bundle.read_bytes()).hexdigest()
    prefix = args.prefix.expanduser().absolute()
    if prefix.exists():
        if (prefix/"bundle.sha256").read_text().strip() != checksum:
            parser.error("目标目录已存在且安装包不同，请使用另一个 --prefix")
        print("已安装：", prefix)
        return
    prefix.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".woosh-install-", dir=prefix.parent) as temporary:
        temporary = Path(temporary)
        packages = temporary/"packages"
        with zipfile.ZipFile(args.bundle) as archive:
            for name in archive.namelist():
                if Path(name).name != name or name in (".", ".."):
                    raise ValueError("安装包包含非预期路径："+name)
            archive.extractall(packages)
        debs = sorted(packages.glob("*.deb"))
        agent = packages/"ros-humble-woosh-robot-agent_0.0.7-0jammy_arm64.deb"
        if agent not in debs:
            raise ValueError("缺少指定版本的 WOOSH Agent")
        staging = temporary/"overlay"
        for deb in debs:
            architecture = subprocess.check_output(["dpkg-deb", "-f", str(deb), "Architecture"], text=True).strip()
            if architecture != "arm64":
                raise ValueError("安装包架构不匹配："+deb.name)
            subprocess.run(["dpkg-deb", "-x", str(deb), str(staging)], check=True)
        (staging/"bundle.sha256").write_text(checksum+"\n")
        staging.rename(prefix)
    print("SDK 用户目录：", prefix)
    print("请重新 source scripts/runtime_environment.sh，再启动 NAV")


if __name__ == "__main__":
    main()
