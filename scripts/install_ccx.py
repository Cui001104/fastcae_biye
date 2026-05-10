# -*- coding: utf-8 -*-
"""下载并安装 CalculiX (ccx.exe) 到 tools/calculix/。

用法：
    python scripts/install_ccx.py                # 默认 2.20 from dhondt.de
    python scripts/install_ccx.py --version 2.22 # 指定版本
    python scripts/install_ccx.py --url <URL>    # 自定义下载源（zip）
    python scripts/install_ccx.py --local <ZIP>  # 不下载，从本地 zip 安装
    python scripts/install_ccx.py --verify-only  # 跳过下载/安装，只跑 ccx -v
"""

import argparse
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
INSTALL_DIR = PROJECT_ROOT / "tools" / "calculix"

# Windows 控制台默认 GBK，强制 UTF-8 输出
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass


def log(msg: str) -> None:
    print(f"[ccx] {msg}", flush=True)


def default_url(version: str) -> str:
    # 默认源：GeneralElectric/CalculiX GitHub 镜像（预编译 win64 zip）
    # 候选版本：2.9 / 2.10
    # dhondt.de 的最新 2.23 也可以用 --url 自定义：
    #   http://www.dhondt.de/calculix_2.23_4win.zip
    return ("https://raw.githubusercontent.com/GeneralElectric/CalculiX/"
            f"master/releases/CalculiX-GE-OSS-{version}-win-x64.zip")


def download(url: str, dst: Path) -> None:
    """用 curl 下载，自动重试 + 断点续传（dhondt.de 经常断流）。"""
    log(f"下载：{url}")
    log(f"保存到：{dst}")

    if shutil.which("curl") is None:
        log("❌ 找不到 curl.exe（Git for Windows 自带，请确认已加 PATH）")
        sys.exit(2)

    cmd = [
        "curl",
        "-L",                    # 跟随 redirect
        "--fail",                # HTTP 错误码 → 非零退出
        "--retry", "10",
        "--retry-delay", "5",
        "--retry-all-errors",    # 网络/HTTP 错误都重试
        "--connect-timeout", "30",
        "-C", "-",               # 断点续传
        "-o", str(dst),
        url,
    ]
    result = subprocess.run(cmd)
    if result.returncode != 0:
        log(f"❌ 下载失败（curl 退出码 {result.returncode}）")
        log("   可能原因：")
        log("   1) 默认 URL 已变更 → 用 --url 自定义")
        log("   2) 公司/校园网拦截 → 换网络或 --local 用本地 zip")
        log("   3) dhondt.de 临时不可达 → 稍后重试或换 GitHub 镜像")
        sys.exit(2)


def extract_and_install(zip_path: Path) -> Path:
    """解压 zip，把 ccx.exe（含同目录依赖）放到 INSTALL_DIR。"""
    INSTALL_DIR.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        log(f"解压到临时目录：{tmp_path}")
        with zipfile.ZipFile(zip_path, "r") as zf:
            zf.extractall(tmp_path)

        # 在解压结果中找 ccx*.exe
        candidates = list(tmp_path.rglob("ccx*.exe"))
        if not candidates:
            log("❌ 解压结果中未发现 ccx*.exe")
            log("   zip 内容：")
            for p in sorted(tmp_path.rglob("*"))[:20]:
                log(f"     {p.relative_to(tmp_path)}")
            sys.exit(3)

        # 选最深的（通常 ccx_2.20_win/bin/ccx*.exe）
        ccx_src = max(candidates, key=lambda p: len(p.parts))
        log(f"找到 ccx：{ccx_src.relative_to(tmp_path)}")

        # 把 ccx.exe 同目录所有文件复制过去（dll 同行）
        src_dir = ccx_src.parent
        for f in src_dir.iterdir():
            if f.is_file():
                shutil.copy2(f, INSTALL_DIR / f.name)
                log(f"  copy {f.name}")

        # 统一命名为 ccx.exe（如果原名带版本）
        installed = INSTALL_DIR / ccx_src.name
        canonical = INSTALL_DIR / "ccx.exe"
        if installed != canonical and not canonical.exists():
            shutil.copy2(installed, canonical)
            log(f"  alias {installed.name} -> ccx.exe")

        return canonical


def verify(ccx: Path) -> None:
    log(f"验证：{ccx} -v")
    if not ccx.is_file():
        log(f"❌ 找不到 {ccx}")
        sys.exit(4)
    try:
        result = subprocess.run(
            [str(ccx), "-v"],
            capture_output=True, text=True, timeout=30,
            cwd=str(INSTALL_DIR),
        )
    except Exception as e:
        log(f"❌ 启动失败：{e}")
        sys.exit(5)

    out = (result.stdout or "") + (result.stderr or "")
    log(f"输出：{out.strip()}")

    if "Version" in out or "version" in out or out.strip():
        log("✅ ccx.exe 可用")
    else:
        log("⚠  ccx 启动了但输出异常，请人工检查")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="安装 CalculiX (ccx.exe)")
    p.add_argument("--version", default="2.10",
                   help="版本号（GitHub GE 镜像可用：2.9 / 2.10），默认 2.10")
    p.add_argument("--url", default="",
                   help="自定义 zip URL（覆盖 --version）")
    p.add_argument("--local", default="",
                   help="本地 zip 路径，跳过下载")
    p.add_argument("--verify-only", action="store_true",
                   help="只跑 ccx -v 验证现有安装")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    if args.verify_only:
        verify(INSTALL_DIR / "ccx.exe")
        return

    if args.local:
        zip_path = Path(args.local).resolve()
        if not zip_path.is_file():
            log(f"❌ 本地 zip 不存在：{zip_path}")
            sys.exit(1)
        log(f"使用本地 zip：{zip_path}")
    else:
        url = args.url or default_url(args.version)
        with tempfile.NamedTemporaryFile(suffix=".zip", delete=False) as tf:
            zip_path = Path(tf.name)
        try:
            download(url, zip_path)
        except SystemExit:
            zip_path.unlink(missing_ok=True)
            raise

    try:
        ccx = extract_and_install(zip_path)
        verify(ccx)
        log(f"✅ 安装完成：{INSTALL_DIR}")
    finally:
        if not args.local:
            zip_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
