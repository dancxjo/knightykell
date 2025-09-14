#!/usr/bin/env python3
"""Keep the PSYCHE repo evergreen and re‑provision if updated.

Behavior:
- If ``/opt/psyche`` is a Git repo, ``git fetch`` and fast‑forward to
  ``origin/$PSYCHE_BRANCH`` (default ``main``).
- Else, download the GitHub zip for ``$GITHUB_OWNER/$GITHUB_REPO@$GITHUB_BRANCH``
  (defaults: dancxjo/knightykell@main), extract, and replace ``/opt/psyche``.
- On change, run ``psyche-provision`` (or fall back to setup_host.py).

Environment:
- ``PSYCHE_BRANCH`` (default: ``main``)
- ``GITHUB_OWNER`` (default: ``dancxjo``)
- ``GITHUB_REPO`` (default: ``knightykell``)
- ``GITHUB_BRANCH`` (default: ``main``)

Examples:
    Run once (service context)::

        $ python3 git_updater.py  # doctest: +SKIP
"""
from __future__ import annotations

import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import urllib.request
import zipfile

SERVICE_USER = os.getenv("PSYCHE_USER", "root")
REPO_DIR = pathlib.Path("/opt/psyche")


def _run(cmd: list[str], check: bool = True, cwd: str | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, check=check, cwd=cwd, text=True, capture_output=True)


def is_git_repo(path: pathlib.Path) -> bool:
    return (path / ".git").exists()


def git_update(path: pathlib.Path, branch: str) -> bool:
    """Fast-forward ``path`` to origin/branch. Returns True if changed."""
    try:
        before = _run(["git", "rev-parse", "HEAD"], cwd=str(path)).stdout.strip()
    except Exception:
        return False
    try:
        _run(["git", "fetch", "--prune", "origin"], cwd=str(path))
        _run(["git", "checkout", branch], cwd=str(path), check=False)
        _run(["git", "reset", "--hard", f"origin/{branch}"], cwd=str(path))
        after = _run(["git", "rev-parse", "HEAD"], cwd=str(path)).stdout.strip()
        return before != after
    except Exception:
        return False


def download_zip(owner: str, repo: str, branch: str) -> pathlib.Path | None:
    url = f"https://codeload.github.com/{owner}/{repo}/zip/refs/heads/{branch}"
    try:
        fd, zpath = tempfile.mkstemp(suffix=".zip")
        os.close(fd)
        urllib.request.urlretrieve(url, zpath)  # noqa: S310
        return pathlib.Path(zpath)
    except Exception:
        return None


def replace_tree_from_zip(zip_path: pathlib.Path, dest: pathlib.Path) -> bool:
    """Extract a GitHub zip and replace ``dest``. Returns True if changed."""
    try:
        with zipfile.ZipFile(str(zip_path)) as zf:
            tmpdir = pathlib.Path(tempfile.mkdtemp())
            zf.extractall(tmpdir)
            # GitHub zips to {repo}-{branch}/
            sub = next(tmpdir.iterdir())
            # Compare a simple marker (count of files) to decide if changed
            old_count = sum(1 for _ in dest.rglob("*")) if dest.exists() else -1
            new_count = sum(1 for _ in sub.rglob("*"))
            if dest.exists():
                shutil.rmtree(dest)
            shutil.copytree(sub, dest)
            return new_count != old_count
    except Exception:
        return False
    finally:
        try:
            zip_path.unlink(missing_ok=True)  # type: ignore[arg-type]
        except Exception:
            pass


def reprovision() -> None:
    """Run psyche-provision (preferred) or fall back to setup_host.py."""
    prov = shutil.which("psyche-provision")
    if prov:
        os.execv(prov, [prov])
    # Fallback: run setup_host.py
    exe = shutil.which("python3") or sys.executable
    os.execv(exe, [exe, "/opt/psyche/setup_host.py"])  # type: ignore[list-item]


def main() -> None:
    branch = os.getenv("PSYCHE_BRANCH", "main")
    owner = os.getenv("GITHUB_OWNER", "dancxjo")
    repo = os.getenv("GITHUB_REPO", "knightykell")
    gh_branch = os.getenv("GITHUB_BRANCH", "main")
    changed = False
    if REPO_DIR.exists() and is_git_repo(REPO_DIR):
        changed = git_update(REPO_DIR, branch)
    else:
        z = download_zip(owner, repo, gh_branch)
        if z:
            changed = replace_tree_from_zip(z, REPO_DIR)
    if changed:
        reprovision()


if __name__ == "__main__":
    main()
