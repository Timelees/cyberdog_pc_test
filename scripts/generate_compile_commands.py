#!/usr/bin/env python3
"""Generate compile_commands.json for clangd in this ROS2 workspace."""

from __future__ import annotations

import json
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = WORKSPACE_ROOT / "src"
OUTPUT = SRC_ROOT / "compile_commands.json"

COMMON_FLAGS = [
    "-DDEFAULT_RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
    "-DRCUTILS_ENABLE_FAULT_INJECTION",
    f"-I{SRC_ROOT / '.vscode/clangd-stubs'}",
    f"-include{SRC_ROOT / '.vscode/clangd_stubs.hpp'}",
    f"-isystem{WORKSPACE_ROOT / 'install/protocol/include'}",
    "-std=gnu++17",
]

PACKAGES = [
    {
        "build_dir": WORKSPACE_ROOT / "build/mutil_odom_shared",
        "include": WORKSPACE_ROOT / "src/src/mutil_odom_shared/include",
        "sources": [
            WORKSPACE_ROOT / "src/src/mutil_odom_shared/src/odom_shared.cpp",
        ],
    },
    {
        "build_dir": WORKSPACE_ROOT / "build/topic_visualization",
        "include": WORKSPACE_ROOT / "src/src/topic_visualization/include",
        "sources": [
            WORKSPACE_ROOT / "src/src/topic_visualization/src/ros_topic_visual.cpp",
            WORKSPACE_ROOT / "src/src/topic_visualization/src/vins_visual.cpp",
        ],
    },
    {
        "build_dir": WORKSPACE_ROOT / "build/keyboard_input",
        "include": WORKSPACE_ROOT / "src/src/keyboard_input/include",
        "sources": [
            WORKSPACE_ROOT / "src/src/keyboard_input/src/keyBoardInput.cpp",
        ],
    },
]


def merge_from_build() -> list[dict]:
    entries: list[dict] = []
    build_root = WORKSPACE_ROOT / "build"
    if not build_root.exists():
        return entries

    for compile_db in sorted(build_root.glob("*/compile_commands.json")):
        with compile_db.open(encoding="utf-8") as handle:
            data = json.load(handle)
        if isinstance(data, list):
            entries.extend(data)
    return entries


def generate_fallback() -> list[dict]:
    entries: list[dict] = []
    for package in PACKAGES:
        build_dir = package["build_dir"]
        include_flag = f"-I{package['include']}"
        for source in package["sources"]:
            command = " ".join(
                ["/usr/bin/c++", *COMMON_FLAGS, include_flag, "-c", str(source)]
            )
            entries.append(
                {
                    "directory": str(build_dir),
                    "command": command,
                    "file": str(source),
                }
            )
    return entries


def main() -> None:
    entries = merge_from_build()
    if not entries:
        entries = generate_fallback()

    with OUTPUT.open("w", encoding="utf-8") as handle:
        json.dump(entries, handle, indent=2)
        handle.write("\n")

    print(f"Wrote {len(entries)} compile commands to {OUTPUT}")


if __name__ == "__main__":
    main()
