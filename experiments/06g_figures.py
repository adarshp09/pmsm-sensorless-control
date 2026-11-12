#!/usr/bin/env python3
"""Phase 06g Step 0d: render Fig. 13 / Fig. 19 at >=300 dpi, crop the speed panels, x3 upscale.

PDF pages are 1-based in the docs convention: printed 827..832 = PDF 5..10, so
Fig. 13 (printed p.830) = PDF page 8 and Fig. 19 (printed p.832) = PDF page 10.

Outputs: docs/figures/*.png (full page 300 dpi + x3-upscaled crops).
"""

from __future__ import annotations

import sys
from pathlib import Path

import fitz
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
PDF = ROOT / "docs" / "act.pdf"
OUT = ROOT / "docs" / "figures"

DPI = 300
ZOOM = DPI / 72.0
UPSCALE = 3


def render(page_1based: int) -> Image.Image:
    doc = fitz.open(PDF)
    page = doc[page_1based - 1]
    pix = page.get_pixmap(matrix=fitz.Matrix(ZOOM, ZOOM), alpha=False)
    img = Image.frombytes("RGB", (pix.width, pix.height), pix.samples)
    doc.close()
    return img


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    jobs = {
        "fig13_p830": (8, (0.02, 0.05, 0.52, 0.55)),   # top-left column, speed panel
        "fig19_p832": (10, (0.02, 0.05, 0.52, 0.55)),
    }
    for name, (page, crop) in jobs.items():
        img = render(page)
        img.save(OUT / f"{name}_full_{DPI}dpi.png")
        w, h = img.size
        box = (int(crop[0] * w), int(crop[1] * h), int(crop[2] * w), int(crop[3] * h))
        panel = img.crop(box)
        panel = panel.resize((panel.width * UPSCALE, panel.height * UPSCALE), Image.LANCZOS)
        panel.save(OUT / f"{name}_speedpanel_x{UPSCALE}.png")
        print(f"{name}: page {page} full {w}x{h}; crop {box} -> {panel.size}")
    print(f"Outputs: {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
