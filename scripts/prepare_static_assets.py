Import("env")

from pathlib import Path
import gzip
import shutil
import subprocess


PROJECT_DIR = Path(env["PROJECT_DIR"])
STATIC_DIR = PROJECT_DIR / "static"
DATA_DIR = PROJECT_DIR / "data"


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


def _sync_static_subdir(subdir: str, commit_text: str) -> int:
    src_dir = STATIC_DIR / subdir
    if not src_dir.exists():
        return 0

    dst_dir = DATA_DIR / subdir
    if dst_dir.exists():
        shutil.rmtree(dst_dir)
    dst_dir.mkdir(parents=True, exist_ok=True)

    count = 0
    for src_file in sorted(p for p in src_dir.rglob("*") if p.is_file()):
        rel = src_file.relative_to(src_dir)
        dst_file = dst_dir / rel
        dst_file.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_file, dst_file)
        _gzip_file(dst_file, dst_file.with_name(dst_file.name + ".gz"))
        count += 1

    commit_file = dst_dir / "commit.txt"
    commit_file.write_text(commit_text, encoding="utf-8")
    _gzip_file(commit_file, dst_dir / "commit.txt.gz")
    return count


def _prepare_static_assets(*_args, **_kwargs) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    if not STATIC_DIR.exists():
        print("[static] static directory not found; skipping asset prep")
        return

    commit_text = _read_git_commit() + "\n"
    total_files = 0
    synced = []
    for subdir in ("app", "legacy"):
        copied = _sync_static_subdir(subdir, commit_text)
        if copied > 0:
            synced.append(subdir)
            total_files += copied

    if not synced:
        print("[static] no static subdirectories (app/legacy) found; skipping asset prep")
        return

    print(f"[static] prepared {total_files} assets in {','.join(synced)} + commit.txt ({commit_text.strip()})")


env.AddPreAction("buildprog", _prepare_static_assets)
env.AddPreAction("buildfs", _prepare_static_assets)
