#!/usr/bin/env python3
"""Generate compile_commands.json for clangd from colcon build artifacts."""

from __future__ import annotations

import json
import re
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = WORKSPACE_ROOT / "src"
BUILD_ROOT = WORKSPACE_ROOT / "build"
OUTPUT = SRC_ROOT / "compile_commands.json"
STUBS_DIR = SRC_ROOT / ".vscode/clangd-stubs"
STUBS_HPP = SRC_ROOT / ".vscode/clangd_stubs.hpp"
ROS_INCLUDE = Path("/opt/ros/galactic/include")

SKIP_DIR_MARKERS = ("__rosidl", "__python", "__pyext", "ament_cmake")
MERGE_WITH_NEXT = {"-isystem", "-I", "-include"}


def tokenize_cmake_flags(value: str) -> list[str]:
    tokens = value.split()
    result: list[str] = []
    index = 0
    while index < len(tokens):
        token = tokens[index]
        if token in MERGE_WITH_NEXT and index + 1 < len(tokens):
            result.append(f"{token}{tokens[index + 1]}")
            index += 2
            continue
        result.append(token)
        index += 1
    return result


def parse_flags(flags_make: Path) -> list[str]:
    text = flags_make.read_text(encoding="utf-8")
    flags: list[str] = []
    for key in ("CXX_DEFINES", "CXX_INCLUDES", "CXX_FLAGS"):
        match = re.search(rf"^{key} = (.+)$", text, re.MULTILINE)
        if match:
            flags.extend(tokenize_cmake_flags(match.group(1).strip()))
    return flags


def should_skip_target(target_dir: Path) -> bool:
    path_str = str(target_dir)
    if any(marker in path_str for marker in SKIP_DIR_MARKERS):
        return True

    build_make = target_dir / "build.make"
    if not build_make.exists():
        return True

    return ".cpp.o:" not in build_make.read_text(encoding="utf-8")


def stub_fallback_flags() -> list[str]:
    flags = [
        f"-I{STUBS_DIR}",
        f"-include{STUBS_HPP}",
        "-DDEFAULT_RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        "-DRCUTILS_ENABLE_FAULT_INJECTION",
        "-std=gnu++17",
    ]
    install_root = WORKSPACE_ROOT / "install"
    if install_root.is_dir():
        for include_dir in sorted(install_root.glob("*/include")):
            flags.append(f"-isystem{include_dir}")
    return flags


def adapt_flags_for_clangd(flags: list[str]) -> list[str]:
    unavailable_prefixes = (
        str(ROS_INCLUDE),
        "/usr/include/eigen3",
        "/usr/include/opencv4",
    )

    adapted: list[str] = []
    for flag in flags:
        if flag.startswith(("-I", "-isystem")):
            path = flag[2:] if flag.startswith("-I") else flag[len("-isystem") :]
            if any(path.startswith(prefix) for prefix in unavailable_prefixes):
                continue
        adapted.append(flag)

    if ROS_INCLUDE.is_dir():
        return adapted

    merged = stub_fallback_flags()
    for flag in adapted:
        if flag not in merged:
            merged.append(flag)
    return merged


def extract_from_build() -> list[dict]:
    entries: list[dict] = []
    seen: set[str] = set()

    if not BUILD_ROOT.is_dir():
        return entries

    for flags_make in sorted(BUILD_ROOT.glob("*/CMakeFiles/*/*.dir/flags.make")):
        target_dir = flags_make.parent
        if should_skip_target(target_dir):
            continue

        build_make = target_dir / "build.make"
        flags = adapt_flags_for_clangd(parse_flags(flags_make))
        package_build_dir = flags_make.parents[2]

        for match in re.finditer(
            r"^\t/usr/bin/c\+\+\s+.*?-c (/home/[^\s]+\.cpp)",
            build_make.read_text(encoding="utf-8"),
            re.MULTILINE,
        ):
            source = match.group(1)
            if source in seen:
                continue
            seen.add(source)
            command = " ".join(["/usr/bin/c++", *flags, "-c", source])
            entries.append(
                {
                    "directory": str(package_build_dir),
                    "command": command,
                    "file": source,
                }
            )

    return entries


def generate_fallback() -> list[dict]:
    entries: list[dict] = []
    packages_root = SRC_ROOT / "src"
    for package_dir in sorted(packages_root.iterdir()):
        src_dir = package_dir / "src"
        include_dir = package_dir / "include"
        build_dir = BUILD_ROOT / package_dir.name
        if not src_dir.is_dir():
            continue

        flags = stub_fallback_flags()
        if include_dir.is_dir():
            flags.append(f"-I{include_dir}")

        for source in sorted(src_dir.rglob("*.cpp")):
            command = " ".join(["/usr/bin/c++", *flags, "-c", str(source)])
            entries.append(
                {
                    "directory": str(build_dir),
                    "command": command,
                    "file": str(source),
                }
            )
    return entries


def main() -> None:
    entries = extract_from_build()
    if not entries:
        entries = generate_fallback()

    with OUTPUT.open("w", encoding="utf-8") as handle:
        json.dump(entries, handle, indent=2)
        handle.write("\n")

    print(f"Wrote {len(entries)} compile commands to {OUTPUT}")


if __name__ == "__main__":
    main()
