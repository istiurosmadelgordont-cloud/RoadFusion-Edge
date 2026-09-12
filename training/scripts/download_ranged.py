#!/usr/bin/env python3
"""Download a large HTTP file with validated parallel byte ranges."""

from __future__ import annotations

import argparse
import hashlib
import math
import os
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import requests


def file_md5(path: Path) -> str:
    digest = hashlib.md5()
    with path.open("rb") as handle:
        while block := handle.read(8 * 1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("url")
    p.add_argument("output", type=Path)
    p.add_argument("--size", type=int, required=True)
    p.add_argument("--md5", default="")
    p.add_argument("--parts", type=int, default=8)
    p.add_argument("--workers", type=int, default=8)
    return p.parse_args()


def download_part(url: str, path: Path, start: int, end: int) -> tuple[Path, int]:
    expected = end - start + 1
    if path.exists() and path.stat().st_size == expected:
        return path, expected
    # Large public archives may see occasional CDN resets. Each valid range is
    # kept on disk, so retry the failed range without restarting the archive.
    for attempt in range(12):
        try:
            existing = path.stat().st_size if path.exists() else 0
            if existing > expected:
                path.unlink()
                existing = 0
            if existing == expected:
                return path, expected
            request_start = start + existing
            with requests.get(
                url,
                headers={"Range": f"bytes={request_start}-{end}", "User-Agent": "RoadFusion-data-audit/1.0"},
                stream=True,
                timeout=(30, 240),
            ) as response:
                response.raise_for_status()
                if response.status_code != 206:
                    raise RuntimeError(f"range request returned HTTP {response.status_code}")
                # Keep bytes received before a connection reset. The next retry
                # requests only the missing tail of this same validated range.
                with path.open("ab") as handle:
                    for chunk in response.iter_content(1024 * 1024):
                        if chunk:
                            handle.write(chunk)
            actual = path.stat().st_size
            if actual != expected:
                raise RuntimeError(f"part size {actual}, expected {expected}")
            return path, actual
        except Exception:
            if attempt == 11:
                raise
            time.sleep(min(30, 2 ** attempt))
    raise AssertionError("unreachable")


def main() -> None:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists() and args.output.stat().st_size == args.size:
        digest = file_md5(args.output)
        if not args.md5 or digest.lower() == args.md5.lower():
            print(f"already complete: {args.output} ({args.size} bytes, md5={digest})")
            return
    chunk_size = math.ceil(args.size / args.parts)
    tasks = []
    with ThreadPoolExecutor(max_workers=min(args.workers, args.parts)) as pool:
        for index in range(args.parts):
            start = index * chunk_size
            if start >= args.size:
                break
            end = min(args.size - 1, start + chunk_size - 1)
            part = args.output.with_suffix(args.output.suffix + f".part{index:02d}")
            tasks.append((index, pool.submit(download_part, args.url, part, start, end)))
        for index, future in tasks:
            part, size = future.result()
            print(f"part {index:02d}: {size} bytes ({part.name})", flush=True)

    temp = args.output.with_suffix(args.output.suffix + ".joining")
    digest = hashlib.md5()
    with temp.open("wb") as output:
        for index, _ in tasks:
            part = args.output.with_suffix(args.output.suffix + f".part{index:02d}")
            with part.open("rb") as source:
                while block := source.read(1024 * 1024):
                    output.write(block)
                    digest.update(block)
    if temp.stat().st_size != args.size:
        raise RuntimeError(f"joined size {temp.stat().st_size}, expected {args.size}")
    actual_md5 = digest.hexdigest()
    if args.md5 and actual_md5.lower() != args.md5.lower():
        raise RuntimeError(f"md5 {actual_md5}, expected {args.md5}")
    os.replace(temp, args.output)
    for index, _ in tasks:
        args.output.with_suffix(args.output.suffix + f".part{index:02d}").unlink()
    print(f"complete: {args.output} ({args.size} bytes, md5={actual_md5})")


if __name__ == "__main__":
    main()
