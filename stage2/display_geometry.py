"""Display-only coordinate transforms for Stage 2 visual demos."""

from __future__ import annotations


def display_pixel(
    raw_pixel: tuple[int, int],
    frame_width: int,
    mirror: bool,
) -> tuple[int, int]:
    """Map a raw pixel into preview coordinates without changing source data."""
    if frame_width <= 0:
        raise ValueError("frame_width must be positive.")

    raw_u, raw_v = raw_pixel
    if not mirror:
        return raw_u, raw_v
    return frame_width - 1 - raw_u, raw_v
