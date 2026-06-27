# -*- coding: utf-8 -*-
"""FastCAE 编译脚本（毕设环境专用）。

用法示例：
    python scripts/build.py                 # 默认 Release 配置 + configure + build
    python scripts/build.py --debug         # Debug 配置
    python scripts/build.py --reconfigure   # 删 build/CMakeCache.txt 后重新 configure
    python scripts/build.py --clean         # 完全删 build/ 后从零编译
    python scripts/build.py --configure     # 只 configure，不编译
    python scripts/build.py --target FastCAE
    python scripts/build.py --install       # 编译后执行 cmake --install

路径在顶部 CONFIG 区配置，路径变了改这里即可。
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

# Windows 控制台默认 GBK，强制 UTF-8 避免中文/emoji 乱码
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass


# ====================== CONFIG（按本机环境改这里） ======================
PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Qt5：cmake 找的是 Qt5Config.cmake 所在目录
QT5_DIR = r"E:/APPs/QT/5.14.2/msvc2017_64/lib/cmake/Qt5"

# CMake 生成器：本机装的是 VS 2019 BuildTools
CMAKE_GENERATOR = "Visual Studio 16 2019"
CMAKE_ARCH = "x64"

# 构建目录（与 .gitignore 中 /build 一致）
BUILD_DIR = PROJECT_ROOT / "build"

# 默认并行任务数：0 = 让 cmake 自己决定（一般 = 逻辑核心数）
DEFAULT_JOBS = 0
# =====================================================================


def log(msg: str) -> None:
    print(f"[build] {msg}", flush=True)


def run(cmd: list, cwd: Path = None) -> None:
    """执行命令，失败立即退出。"""
    log("$ " + " ".join(str(x) for x in cmd))
    result = subprocess.run(cmd, cwd=str(cwd) if cwd else None)
    if result.returncode != 0:
        log(f"命令失败，退出码 {result.returncode}")
        sys.exit(result.returncode)


def check_paths() -> None:
    """快速校验关键路径，避免跑半天才报错。"""
    qt_cfg = Path(QT5_DIR) / "Qt5Config.cmake"
    if not qt_cfg.is_file():
        log(f"❌ 找不到 Qt5Config.cmake：{qt_cfg}")
        log("   请检查脚本顶部 QT5_DIR 是否正确。")
        sys.exit(1)
    log(f"✅ Qt5Config.cmake 存在：{qt_cfg}")

    extlib = PROJECT_ROOT / "extlib"
    if not extlib.is_dir():
        log("⚠  extlib/ 不存在，首次 configure 会从 gitee 自动 clone")
        log("   （需要可访问 gitee.com，仓库较大）")
    else:
        log(f"✅ extlib/ 已存在：{extlib}")


def cmake_configure(build_type: str) -> None:
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    cmd = [
        "cmake",
        "-S", str(PROJECT_ROOT),
        "-B", str(BUILD_DIR),
        "-G", CMAKE_GENERATOR,
        "-A", CMAKE_ARCH,
        f"-DQt5_DIR={QT5_DIR}",
        f"-DCMAKE_BUILD_TYPE={build_type}",
    ]
    run(cmd)


def cmake_build(build_type: str, target: str, jobs: int) -> None:
    cmd = [
        "cmake",
        "--build", str(BUILD_DIR),
        "--config", build_type,
    ]
    if target:
        cmd += ["--target", target]
    if jobs and jobs > 0:
        cmd += ["--parallel", str(jobs)]
    else:
        cmd += ["--parallel"]
    run(cmd)


def cmake_install(build_type: str) -> None:
    cmd = ["cmake", "--install", str(BUILD_DIR), "--config", build_type]
    run(cmd)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="FastCAE 编译脚本")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--release", action="store_true", help="Release 配置（默认）")
    g.add_argument("--debug", action="store_true", help="Debug 配置")

    p.add_argument("--configure", action="store_true",
                   help="只 configure，不编译")
    p.add_argument("--reconfigure", action="store_true",
                   help="删除 CMakeCache.txt 后重新 configure")
    p.add_argument("--clean", action="store_true",
                   help="完全删除 build/ 目录后重新编译")
    p.add_argument("--target", default="",
                   help="指定 target（如 FastCAE / GearAutoOpt），默认全部")
    p.add_argument("--jobs", type=int, default=DEFAULT_JOBS,
                   help="并行任务数，0=自动")
    p.add_argument("--install", action="store_true",
                   help="编译后执行 cmake --install")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    build_type = "Debug" if args.debug else "Release"

    log(f"项目根：{PROJECT_ROOT}")
    log(f"构建目录：{BUILD_DIR}")
    log(f"配置：{build_type}")
    log(f"生成器：{CMAKE_GENERATOR} ({CMAKE_ARCH})")

    check_paths()

    if args.clean and BUILD_DIR.exists():
        log(f"--clean：删除 {BUILD_DIR}")
        shutil.rmtree(BUILD_DIR)

    cache_file = BUILD_DIR / "CMakeCache.txt"
    if args.reconfigure and cache_file.exists():
        log(f"--reconfigure：删除 {cache_file}")
        cache_file.unlink()

    need_configure = args.configure or args.reconfigure or args.clean \
        or not cache_file.exists()
    if need_configure:
        cmake_configure(build_type)

    if args.configure:
        log("仅 configure 完成，退出。")
        return

    cmake_build(build_type, args.target, args.jobs)

    if args.install:
        cmake_install(build_type)

    log("✅ 完成")


if __name__ == "__main__":
    main()
