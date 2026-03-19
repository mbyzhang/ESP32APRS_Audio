Import("env")

from pathlib import Path
import gzip
import shutil
import subprocess


PROJECT_DIR = Path(env["PROJECT_DIR"])
STATIC_APP_DIR = PROJECT_DIR / "static" / "app"
DATA_APP_DIR = PROJECT_DIR / "data" / "app"


def _gzip_file(src_path: Path, dst_path: Path) -> None:
    with src_path.open("rb") as src, dst_path.open("wb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, compresslevel=9, mtime=0) as gz:
            shutil.copyfileobj(src, gz)


def _read_git_commit() -> str:
    try:
        sha = subprocess.check_output(
            ["git", "rev-parse", "--short=12", "HEAD"],
            cwd=PROJECT_DIR,
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        return "unknown"

    try:
        dirty = subprocess.check_output(
            ["git", "status", "--porcelain"],
            cwd=PROJECT_DIR,
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
        if dirty:
            sha = f"{sha}-dirty"
    except Exception:
        pass
    return sha


def _prepare_static_assets(*_args, **_kwargs) -> None:
    DATA_APP_DIR.mkdir(parents=True, exist_ok=True)

    if not STATIC_APP_DIR.exists():
        print("[static] static/app not found; skipping asset prep")
        return

    source_files = sorted(p for p in STATIC_APP_DIR.iterdir() if p.is_file())

    for old in DATA_APP_DIR.iterdir():
        if old.is_file():
            old.unlink()

    for src_file in source_files:
        dst_file = DATA_APP_DIR / src_file.name
        shutil.copy2(src_file, dst_file)
        _gzip_file(dst_file, DATA_APP_DIR / f"{src_file.name}.gz")

    commit_text = _read_git_commit() + "\n"
    commit_file = DATA_APP_DIR / "commit.txt"
    commit_file.write_text(commit_text, encoding="utf-8")
    _gzip_file(commit_file, DATA_APP_DIR / "commit.txt.gz")

    print(f"[static] prepared {len(source_files)} assets + commit.txt ({commit_text.strip()})")


env.AddPreAction("buildprog", _prepare_static_assets)
env.AddPreAction("buildfs", _prepare_static_assets)
