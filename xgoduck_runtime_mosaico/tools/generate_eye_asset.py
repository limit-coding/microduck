"""Generate the 480x480 RGB565 RLE asset used by the one-shot LCD splash."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

from PIL import Image


def rgb565(rgb: tuple[int, int, int]) -> int:
    r, g, b = rgb
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("--asset-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    args.asset_dir.mkdir(parents=True, exist_ok=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    source_bytes = args.source.read_bytes()
    source_copy = args.asset_dir / "duck_eye_source.png"
    source_copy.write_bytes(source_bytes)

    image = Image.open(args.source).convert("RGB")
    image = image.resize((480, 480), Image.Resampling.LANCZOS)
    # Match the physical panel mounting: flip horizontally and vertically
    # (equivalent to a 180-degree rotation) before encoding the one-shot frame.
    image = image.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
    image = image.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
    image.save(args.asset_dir / "duck_eye_480.png", optimize=True)

    values = [rgb565(pixel) for pixel in image.getdata()]
    runs: list[tuple[int, int]] = []
    start = 0
    while start < len(values):
        color = values[start]
        end = start + 1
        while end < len(values) and values[end] == color and end - start < 65535:
            end += 1
        runs.append((end - start, color))
        start = end

    header = """#pragma once
#include <stddef.h>
#include <stdint.h>

#define DUCK_EYE_WIDTH 480
#define DUCK_EYE_HEIGHT 480

typedef struct {
    uint16_t count;
    uint16_t rgb565;
} duck_eye_run_t;

extern const duck_eye_run_t duck_eye_runs[];
extern const size_t duck_eye_run_count;
"""
    (args.output_dir / "duck_eye_asset.h").write_text(header, encoding="utf-8", newline="\n")

    sha = hashlib.sha256(source_bytes).hexdigest()
    rows = []
    for index in range(0, len(runs), 6):
        chunk = runs[index : index + 6]
        rows.append("    " + ", ".join(f"{{{count}, 0x{color:04x}}}" for count, color in chunk) + ",")
    source = (
        '#include "duck_eye_asset.h"\n\n'
        f"/* Generated from duck_eye_source.png, SHA256 {sha}. */\n"
        "const duck_eye_run_t duck_eye_runs[] = {\n"
        + "\n".join(rows)
        + "\n};\n\n"
        "const size_t duck_eye_run_count = sizeof(duck_eye_runs) / sizeof(duck_eye_runs[0]);\n"
    )
    (args.output_dir / "duck_eye_asset.c").write_text(source, encoding="utf-8", newline="\n")
    raw_bytes = 480 * 480 * 2
    print(f"generated {len(runs)} runs, {len(runs) * 4} bytes RLE versus {raw_bytes} bytes raw")


if __name__ == "__main__":
    main()
