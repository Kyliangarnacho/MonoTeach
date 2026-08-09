"""Coordinate helpers independent of cameras and hand-detection libraries."""

from __future__ import annotations


def normalized_to_pixel(
    x_norm: float,
    y_norm: float,
    width: int,
    height: int,
) -> tuple[int, int]:
    """Map normalized coordinates to clipped zero-based image pixel coordinates."""
    if width <= 0 or height <= 0:
        raise ValueError("Image width and height must be positive.")

    clipped_x = min(max(float(x_norm), 0.0), 1.0)
    clipped_y = min(max(float(y_norm), 0.0), 1.0)
    return (
        round(clipped_x * (width - 1)),
        round(clipped_y * (height - 1)),
    )
