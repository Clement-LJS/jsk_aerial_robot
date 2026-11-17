#!/usr/bin/env python3
"""Download pinned source archives only; no git metadata, installs or builds.

Deliberate setup utility, not invoked by CMake, ROS launch, or the generator.
Destination must be a NEW directory outside the robot repository.
Only the HPIPM/BLASFEO backend and matching renderer are included; optional QP
backends and acados tests/examples are not configured or executed.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import tarfile
import urllib.request


SOURCES = [
    (".", "acados/acados", "fd231104913cc9c305e0dff736281b91dc1dbc5b"), # v0.5.0
    ("external/blasfeo", "giaf/blasfeo", "9c2a8345f9de37fbb4b3cd84810b0250810de622"),
    ("external/hpipm", "giaf/hpipm", "945645e8c0ac473f3983fb3c7f22dbae50d864cc"),
    ("interfaces/acados_template/tera_renderer", "acados/tera_renderer",
     "c41c5a03a03a42d3ac5084ddcd415502f7721b05"), # v0.0.34, acados TERA_VERSION
]


def extract(data, target, root):
    target.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
        for member in archive:
            # GitHub adds one enclosing directory named repo-commit.
            parts = Path(member.name).parts[1:]
            if not parts:
                continue
            path = target.joinpath(*parts)
            if member.name.startswith("/") or ".." in parts or root not in path.resolve().parents:
                raise ValueError("Archive path escapes destination")
            if member.isdir():
                path.mkdir(parents=True, exist_ok=True)
            elif member.issym():
                link_target = (path.parent/member.linkname).resolve()
                if member.linkname.startswith("/") or (link_target != root and root not in link_target.parents):
                    # Some upstream examples point to files outside the checkout.
                    # Never materialize such links in a prepared dependency tree.
                    print(f"Skipped external symlink: {path.relative_to(root)} -> {member.linkname}")
                    continue
                path.parent.mkdir(parents=True, exist_ok=True)
                if path.is_symlink():
                    if path.readlink() != Path(member.linkname):
                        raise ValueError(f"Refusing to replace existing symlink: {path}")
                else:
                    path.symlink_to(member.linkname)
            elif member.isfile():
                path.parent.mkdir(parents=True, exist_ok=True)
                content = archive.extractfile(member).read()
                if path.exists():
                    if path.is_symlink() or path.read_bytes() != content:
                        raise ValueError(f"Refusing to replace existing file: {path}")
                else:
                    with path.open("xb") as stream:
                        stream.write(content)
                    path.chmod(member.mode & 0o777)
            else:
                raise ValueError("Unsupported archive entry")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", required=True, type=Path)
    parser.add_argument("--resume", action="store_true", help="Fill a partial download; existing files must match byte-for-byte")
    args = parser.parse_args()
    destination = args.destination.expanduser().resolve()
    repository = Path(__file__).resolve().parents[4]
    if destination == repository or repository in destination.parents:
        parser.error("Do not vendor acados into the robot repository")
    if destination.exists() and not args.resume:
        parser.error("Destination already exists; refusing to overwrite it")
    # Download everything before creating the destination, so network failures
    # do not leave a partly populated tree.
    archives, lock = [], []
    for subdir, repo, revision in SOURCES:
        url = f"https://codeload.github.com/{repo}/tar.gz/{revision}"
        print(f"Downloading {repo}@{revision}", flush=True)
        request = urllib.request.Request(url, headers={"User-Agent": "gimbalrotor-source-preparation"})
        with urllib.request.urlopen(request, timeout=60) as response:
            data = response.read()
        archives.append((subdir, data))
        lock.append({"directory": subdir, "repository": repo, "revision": revision,
                     "url": url, "sha256": hashlib.sha256(data).hexdigest()})
    destination.mkdir(parents=True, exist_ok=args.resume)
    for subdir, data in archives:
        extract(data, destination/subdir, destination)
    (destination/"NMPC_SOURCE_LOCK.json").write_text(json.dumps(lock, indent=2)+"\n")
    for subdir in ("build", "include", "lib", "bin"):
        (destination/subdir).mkdir(exist_ok=True)
    print(f"Sources prepared in {destination}; include/lib/build are unbuilt placeholders.")
    print("No Python packages, native libraries, renderer executable, or ROS packages were installed.")


if __name__ == "__main__":
    main()
