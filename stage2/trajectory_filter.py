"""Pure EMA filtering for Stage 2.2 quality-processed trajectories."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, replace

from .fingertip import normalized_to_pixel
from .trajectory import TrajectoryMetadata, TrajectorySample


@dataclass(frozen=True)
class EMAConfig:
    """Exponential moving-average configuration."""

    alpha: float = 0.25

    def __post_init__(self) -> None:
        if not 0.0 < self.alpha <= 1.0:
            raise ValueError("alpha must satisfy 0 < alpha <= 1.")


def apply_ema_filter(
    processed_samples: Iterable[TrajectorySample],
    metadata: TrajectoryMetadata,
    config: EMAConfig = EMAConfig(),
) -> tuple[TrajectorySample, ...]:
    """Return new EMA-filtered samples; invalid gaps reset filter state."""
    filtered_samples: list[TrajectorySample] = []
    filtered_x: float | None = None
    filtered_y: float | None = None

    for sample in processed_samples:
        if not sample.valid:
            filtered_samples.append(replace(sample))
            filtered_x = None
            filtered_y = None
            continue

        assert sample.x_norm is not None and sample.y_norm is not None
        if filtered_x is None or filtered_y is None:
            filtered_x = sample.x_norm
            filtered_y = sample.y_norm
        else:
            filtered_x = (
                config.alpha * sample.x_norm
                + (1.0 - config.alpha) * filtered_x
            )
            filtered_y = (
                config.alpha * sample.y_norm
                + (1.0 - config.alpha) * filtered_y
            )

        u, v = normalized_to_pixel(
            filtered_x,
            filtered_y,
            metadata.frame_width,
            metadata.frame_height,
        )
        filtered_samples.append(
            TrajectorySample(
                t_ms=sample.t_ms,
                valid=True,
                x_norm=filtered_x,
                y_norm=filtered_y,
                u=u,
                v=v,
                invalid_reason=None,
            )
        )

    return tuple(filtered_samples)
