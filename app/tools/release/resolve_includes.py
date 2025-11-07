#!/usr/bin/env python3
import re
import sys
from argparse import ArgumentParser
from pathlib import Path
from typing import Any, Callable, Iterable

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*(?:"([^"]+)"|<([^>]+)>)')


def resolve_file(path: Path | str, *, include_path: Iterable[Path | str], write: Callable[[str], Any]):
    path = Path(path).resolve()
    initial_dir = path.parent
    include_path = [Path(p).resolve() for p in include_path]
    include_path = [p for p in include_path if p.exists() and p.is_dir()]
    return _resolve_file_inner(path, include_path, initial_dir, [], write=write)


def _resolve_file_inner(
    path: Path,
    include_path: list[Path],
    initial_dir: Path,
    include_stack: list[Path],
    *,
    write: Callable[[str], Any],
):
    path = path.resolve()
    if path in include_stack:
        raise RuntimeError(f"include cycle detected for {path} */")

    include_stack.append(path)
    try:
        with path.open("r", encoding="utf-8") as f:
            lines = f.readlines()
    except Exception:
        raise RuntimeError(f"include not found: {path} */") from None

    cur_dir = path.parent
    for ln in lines:
        m = INCLUDE_RE.match(ln)
        if not m:
            write(ln)
            continue
    
        inc_name = m.group(1) or m.group(2)
        inc_path_preferred = bool(m.group(2))

        resolved = None
        inc_path = Path(str(inc_name))
        # If absolute path, try directly
        if inc_path.is_absolute():
            if inc_path.exists():
                resolved = inc_path.resolve()
        else:
            # search order: directory of current file, initial input directory, cwd
            candidates = [
                cur_dir if cur_dir else None,
                initial_dir if initial_dir else None,
            ]
            if inc_path_preferred:
                candidates[0:0] = include_path
            else:
                candidates.extend(include_path)
            for c in candidates:
                if c is None:
                    continue
                c = c / inc_name
                if not c:
                    continue
                if c.exists():
                    resolved = c.resolve()
                    break
        if resolved is None:
            # leave the original include line and warn
            raise RuntimeError(f"include not found: {inc_name} referenced from {path} */")
        else:
            # inline resolved file recursively
            write("#####\n")
            write(f"# Begin include: {inc_name}\n")
            write("#####\n\n")
            _resolve_file_inner(resolved, include_path, initial_dir, include_stack, write=write)
            write("\n")
            write("#####\n")
            write(f"# End include: {inc_name}\n")
            write("#####\n\n")

    include_stack.pop()


def main():
    parser = ArgumentParser()
    parser.add_argument(
        "input_file", help="Path to the input file to resolve includes for"
    )
    parser.add_argument(
        "-I",
        "--include-dir",
        action="append",
        default=[],
        help="Additional include directories to search",
    )
    args = parser.parse_args()

    input_path = Path(args.input_file)
    if not input_path.exists():
        print(f"Error: input file not found: {input_path}", file=sys.stderr)
        sys.exit(1)

    resolve_file(input_path, include_path=args.include_dir, write=sys.stdout.write)


if __name__ == "__main__":
    main()
