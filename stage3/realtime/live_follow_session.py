"""Explicit two-confirmation live teaching session and minimal event gate.

This replaces automatic ``waiting for stable start`` for the final live demo.
The operator, not a noisy fingertip classifier, decides when P0 is acceptable.
All output points remain derived evidence; raw camera observations stay intact.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from statistics import median

from stage2.workspace_geometry import WorkspaceCalibration

from .live_observation import LiveWorkspaceObservation
from .live_position_filter import FilteredWorkspaceEstimate
from .stream_contract import RealtimeTaskEvent


class LiveFollowPhase(str, Enum):
    WAIT_FOR_START_CONFIRM = "WAIT_FOR_START_CONFIRM"
    WAIT_FOR_ROBOT_READY = "WAIT_FOR_ROBOT_READY"
    WAIT_FOR_TEACH_CONFIRM = "WAIT_FOR_TEACH_CONFIRM"
    TRACKING = "TRACKING"
    # Camera acquisition continues in these phases, but ordinary Cartesian
    # waypoints are not admitted while MATLAB finishes the semantic bridge.
    PEN_TRANSITION = "PEN_TRANSITION"
    VISION_GAP = "VISION_GAP"
    WAIT_FOR_REACQUIRE_READY = "WAIT_FOR_REACQUIRE_READY"
    CLOSED = "CLOSED"


@dataclass(frozen=True)
class StartReadiness:
    """Why the first SPACE may or may not lock a safe P0 right now.

    This is deliberately display-friendly evidence, not another state
    machine.  The camera window can explain a rejected key press without
    terminating the live session or hiding the original observation.
    """

    candidate_count: int
    required_candidate_count: int
    current_point_is_operating: bool
    reason: str
    median_x_mm: float | None
    median_y_mm: float | None
    max_spread_mm: float | None
    allowed_spread_mm: float

    @property
    def ready(self) -> bool:
        return self.reason == "ready"

    @property
    def summary(self) -> str:
        location = "median=--" if self.median_x_mm is None else (
            f"median=({self.median_x_mm:.1f},{self.median_y_mm:.1f}) mm"
        )
        spread = "spread=--" if self.max_spread_mm is None else (
            f"spread={self.max_spread_mm:.2f}/{self.allowed_spread_mm:.2f} mm"
        )
        return (f"{self.reason} | eligible={self.candidate_count}/"
                f"{self.required_candidate_count} | {location} | {spread} | "
                f"operating={self.current_point_is_operating}")


class StartNotReadyError(RuntimeError):
    """Expected first-SPACE rejection; callers should keep the preview alive."""

    def __init__(self, readiness: StartReadiness) -> None:
        self.readiness = readiness
        super().__init__(f"Start is not ready: {readiness.summary}")


@dataclass(frozen=True)
class LiveFollowConfig:
    anchor_window_ms: float = 500.0
    operating_margin_mm: float = 10.0
    minimum_motion_mm: float = 2.0
    heartbeat_ms: float = 200.0
    minimum_start_candidates: int = 2
    maximum_start_spread_mm: float = 5.0
    minimum_reacquire_candidates: int = 2
    maximum_reacquire_spread_mm: float = 5.0
    contact_z_task_mm: float = 0.0
    lifted_z_task_mm: float = 12.0

    def __post_init__(self) -> None:
        for name in ("anchor_window_ms", "operating_margin_mm", "minimum_motion_mm", "heartbeat_ms", "maximum_start_spread_mm", "maximum_reacquire_spread_mm"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")
            object.__setattr__(self, name, value)
        for name in ("minimum_start_candidates", "minimum_reacquire_candidates"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer.")
        contact = float(self.contact_z_task_mm)
        lifted = float(self.lifted_z_task_mm)
        if not math.isfinite(contact) or not math.isfinite(lifted) or lifted <= contact:
            raise ValueError("lifted_z_task_mm must be finite and greater than contact_z_task_mm.")
        object.__setattr__(self, "contact_z_task_mm", contact)
        object.__setattr__(self, "lifted_z_task_mm", lifted)


@dataclass(frozen=True)
class StartTarget:
    """The first SPACE result: a robust target used for HOME_TO_START."""
    x_mm: float
    y_mm: float
    source_frame_indices: tuple[int, ...]


@dataclass(frozen=True)
class ReacquireTarget:
    """One stable current target after a true visual gap, never a gap bridge."""

    x_mm: float
    y_mm: float
    z_task_mm: float
    pen_state: str
    source_frame_indices: tuple[int, ...]


@dataclass(frozen=True)
class _PendingDwell:
    """Latest stationary instant, retained as a temporal planner boundary."""

    source_t_ms: float
    available_t_ms: float


@dataclass(frozen=True)
class MotionRestartBoundary:
    """Derived notice that a long hold split two ordinary motion runs.

    This is deliberately not a ``RealtimeTaskEvent``.  The hand stayed valid,
    inside the workspace, and in the same pen state, so it is neither a visual
    barrier nor a semantic pen transition.  Its only job is to tell the
    rolling planner to close the old run at rest before it accepts the first
    genuinely moved point after the hold.
    """

    source_t_ms: float
    replay_t_ms: float


class LiveFollowSession:
    """Own only live-session state; planner/IK/TCP stay outside this class."""
    def __init__(self, calibration: WorkspaceCalibration, config: LiveFollowConfig = LiveFollowConfig()) -> None:
        if not isinstance(calibration, WorkspaceCalibration):
            raise TypeError("calibration must be a WorkspaceCalibration.")
        if not isinstance(config, LiveFollowConfig):
            raise TypeError("config must be a LiveFollowConfig.")
        if 2.0 * config.operating_margin_mm >= min(calibration.workspace_definition.width_mm, calibration.workspace_definition.height_mm):
            raise ValueError("operating_margin_mm leaves no usable interior workspace.")
        self.calibration, self.config, self.phase = calibration, config, LiveFollowPhase.WAIT_FOR_START_CONFIRM
        self._recent: list[FilteredWorkspaceEstimate] = []
        # The live target is intentionally *relative* after the second SPACE:
        # formal_target = measured_workspace + mapping_offset.  Keeping one
        # offset, rather than separately mutable robot/hand anchors, makes a
        # rebase atomic and prevents a pen transition from updating only half
        # of the mapping.
        self._start_target_xy: tuple[float, float] | None = None
        self._mapping_offset_xy: tuple[float, float] | None = None
        self._epoch_capture_ms: float | None = None
        self._last_event: RealtimeTaskEvent | None = None
        self._events: list[RealtimeTaskEvent] = []
        self._observations: list[LiveWorkspaceObservation] = []
        self._latest_start_reason = "waiting_for_camera"
        self._latest_point_is_operating = False
        self._pending_dwell: _PendingDwell | None = None
        self._pending_motion_restart: MotionRestartBoundary | None = None
        self._reacquire_candidates: list[tuple[FilteredWorkspaceEstimate, tuple[float, float]]] = []
        self._pending_reacquire_target: ReacquireTarget | None = None
        self._active_reacquire_target: ReacquireTarget | None = None
        self._latest_measurement_estimate: FilteredWorkspaceEstimate | None = None
        self._latest_paused_estimate: FilteredWorkspaceEstimate | None = None
        self._pen_transition_complete = False

    @property
    def events(self) -> tuple[RealtimeTaskEvent, ...]:
        return tuple(self._events)

    @property
    def mapping_offset_xy_mm(self) -> tuple[float, float] | None:
        """Current formal-target minus filtered-measurement XY offset."""
        return self._mapping_offset_xy

    @property
    def latest_measured_xy_mm(self) -> tuple[float, float] | None:
        """Latest filtered visual measurement; never the planner's target."""
        estimate = self._latest_measurement_estimate
        if estimate is None or estimate.filtered_x_mm is None or estimate.filtered_y_mm is None:
            return None
        return (estimate.filtered_x_mm, estimate.filtered_y_mm)

    @property
    def latest_formal_target_xy_mm(self) -> tuple[float, float] | None:
        """Latest admitted robot target, retained visibly across a gap."""
        for event in reversed(self._events):
            if event.valid:
                assert event.x_mm is not None and event.y_mm is not None
                return (event.x_mm, event.y_mm)
        return None

    @property
    def start_readiness(self) -> StartReadiness:
        """Read-only reason for the next first-SPACE decision."""
        candidates = self._recent_valid()
        candidate_count = len(candidates)
        reason = self._latest_start_reason
        if reason == "eligible" and candidate_count < self.config.minimum_start_candidates:
            reason = "collecting_recent_samples"
        elif reason == "eligible":
            median_x, median_y, spread = self._start_statistics(candidates)
            if spread > self.config.maximum_start_spread_mm:
                reason = "start_points_too_spread"
            else:
                reason = "ready"
        if candidates:
            median_x, median_y, spread = self._start_statistics(candidates)
        else:
            median_x = median_y = spread = None
        return StartReadiness(
            candidate_count=candidate_count,
            required_candidate_count=self.config.minimum_start_candidates,
            current_point_is_operating=self._latest_point_is_operating,
            reason=reason,
            median_x_mm=median_x,
            median_y_mm=median_y,
            max_spread_mm=spread,
            allowed_spread_mm=self.config.maximum_start_spread_mm,
        )

    def process(self, observation: LiveWorkspaceObservation, estimate: FilteredWorkspaceEstimate) -> RealtimeTaskEvent | None:
        """Compatibility view returning the newest formal event, if any.

        Live code should use :meth:`process_many`, which exposes any preceding
        stationary-dwell boundary separately from ordinary source events.
        """
        events = self.process_many(observation, estimate)
        return events[-1] if events else None

    def process_many(self, observation: LiveWorkspaceObservation, estimate: FilteredWorkspaceEstimate) -> tuple[RealtimeTaskEvent, ...]:
        if estimate.raw_observation is not observation:
            raise ValueError("estimate must refer to the exact observation.")
        if self.phase is LiveFollowPhase.CLOSED:
            return ()
        self._observations.append(observation)
        self._latest_measurement_estimate = estimate
        if self.phase is LiveFollowPhase.VISION_GAP:
            self._remember_reacquire_candidate(estimate)
            return ()
        if self.phase in {LiveFollowPhase.PEN_TRANSITION, LiveFollowPhase.WAIT_FOR_REACQUIRE_READY}:
            if self._is_operating_estimate(estimate):
                self._latest_paused_estimate = estimate
                if self.phase is LiveFollowPhase.PEN_TRANSITION and self._pen_transition_complete:
                    # MATLAB may finish a short lift before the operator has
                    # released L.  The first ordinary POINT frame becomes a
                    # fresh offset rebase rather than a false spatial jump.
                    target_xy = self.latest_formal_target_xy_mm
                    assert target_xy is not None
                    self._rebase(estimate, target_xy)
                    self.phase = LiveFollowPhase.TRACKING
            return ()
        if self.phase is not LiveFollowPhase.TRACKING:
            self._remember_anchor_candidate(estimate)
            return ()
        return self._maybe_publish_many(estimate)

    def confirm_start(self) -> StartTarget:
        """First SPACE: choose a median P0 and lock the future robot target."""
        if self.phase is not LiveFollowPhase.WAIT_FOR_START_CONFIRM:
            raise RuntimeError("Start may be confirmed only while waiting for the first SPACE.")
        readiness = self.start_readiness
        if not readiness.ready:
            raise StartNotReadyError(readiness)
        candidates = self._recent_valid()
        x = float(median(item.filtered_x_mm for item in candidates if item.filtered_x_mm is not None))
        y = float(median(item.filtered_y_mm for item in candidates if item.filtered_y_mm is not None))
        self._start_target_xy = (x, y)
        self.phase = LiveFollowPhase.WAIT_FOR_ROBOT_READY
        return StartTarget(x, y, tuple(item.raw_observation.frame_index for item in candidates))

    def notify_robot_ready(self) -> None:
        if self.phase is not LiveFollowPhase.WAIT_FOR_ROBOT_READY:
            raise RuntimeError("Robot READY is only valid after a start target was confirmed.")
        self.phase = LiveFollowPhase.WAIT_FOR_TEACH_CONFIRM

    def confirm_teaching(self) -> RealtimeTaskEvent:
        """Second SPACE: establish relative hand anchor and formal source epoch."""
        if self.phase is not LiveFollowPhase.WAIT_FOR_TEACH_CONFIRM or self._start_target_xy is None:
            raise RuntimeError("Teaching may begin only after MATLAB reports READY.")
        candidates = self._recent_valid()
        if not candidates:
            raise RuntimeError("Need a current valid interior hand point before teaching confirmation.")
        last = candidates[-1]
        assert last.filtered_x_mm is not None and last.filtered_y_mm is not None
        self._rebase(last, self._start_target_xy)
        raw = last.raw_observation
        self._epoch_capture_ms = raw.capture_t_ms
        self.phase = LiveFollowPhase.TRACKING
        event = RealtimeTaskEvent(
            0, 0.0, raw.available_t_ms - self._epoch_capture_ms, True, True,
            self._start_target_xy[0], self._start_target_xy[1], raw.pen_state,
            raw.stroke_id, None, self._z_for_pen(raw.pen_state),
        )
        self._events.append(event); self._last_event = event
        return event

    def finish_tracking(self) -> tuple[RealtimeTaskEvent, ...]:
        """Discard a quiet-tail marker; pipeline EOF closes the active run."""
        self._pending_dwell = None
        return ()

    def take_motion_restart_boundary(self) -> MotionRestartBoundary | None:
        """Consume one pending stationary-dwell boundary, if motion resumed."""
        boundary = self._pending_motion_restart
        self._pending_motion_restart = None
        return boundary

    def take_reacquire_target(self) -> ReacquireTarget | None:
        """Return one ready target and freeze ordinary planning until MATLAB ACKs.

        This is deliberately automatic for an ordinary short visual gap: raw
        camera frames continue, but no stale post-gap path can pile up behind
        the robot's physical reacquire motion.
        """
        if self.phase is not LiveFollowPhase.VISION_GAP or self._pending_reacquire_target is None:
            return None
        target = self._pending_reacquire_target
        self._pending_reacquire_target = None
        self._active_reacquire_target = target
        self.phase = LiveFollowPhase.WAIT_FOR_REACQUIRE_READY
        return target

    def notify_reacquire_ready(self) -> RealtimeTaskEvent:
        """Seed a new planner run only after MATLAB completed the bridge."""
        if self.phase is not LiveFollowPhase.WAIT_FOR_REACQUIRE_READY:
            raise RuntimeError("REACQUIRE_READY is only valid after a reacquire target was sent.")
        estimate = self._latest_paused_estimate
        if estimate is None or not self._is_operating_estimate(estimate):
            raise RuntimeError("Need a current valid operating-region point to resume after reacquire.")
        target = self._active_reacquire_target
        if target is None:
            raise RuntimeError("Missing the target that MATLAB was asked to reacquire.")
        target_xy = (target.x_mm, target.y_mm)
        assert self._epoch_capture_ms is not None
        self._rebase(estimate, target_xy)
        raw = estimate.raw_observation
        event = self._append_event(
            raw.capture_t_ms - self._epoch_capture_ms,
            raw.available_t_ms - self._epoch_capture_ms,
            True, target_xy[0], target_xy[1], raw.pen_state, raw.stroke_id, None,
        )
        self._active_reacquire_target = None
        self.phase = LiveFollowPhase.TRACKING
        return event

    def notify_pen_transition_ready(self) -> None:
        """Resume after lift/lower is actually at the FIFO tail, not on toggle."""
        if self.phase is not LiveFollowPhase.PEN_TRANSITION:
            raise RuntimeError("PEN_TRANSITION_READY is only valid while pen motion is active.")
        self._pen_transition_complete = True
        estimate = self._latest_paused_estimate
        if estimate is not None and self._is_operating_estimate(estimate):
            # The robot stayed at the old Cartesian target while the hand was
            # forming the command.  Rebase the single offset atomically, so
            # the first released POINT cannot jump back toward P0.
            target_xy = self.latest_formal_target_xy_mm
            assert target_xy is not None
            self._rebase(estimate, target_xy)
            self.phase = LiveFollowPhase.TRACKING

    def close(self) -> None:
        self.phase = LiveFollowPhase.CLOSED

    def _remember_anchor_candidate(self, estimate: FilteredWorkspaceEstimate) -> None:
        raw = estimate.raw_observation
        self._latest_start_reason = self._start_candidate_reason(estimate)
        self._latest_point_is_operating = self._latest_start_reason == "eligible"
        if self._latest_point_is_operating:
            self._recent.append(estimate)
        elif not raw.valid:
            # A true no-hand gap must not borrow an old P0 when the hand
            # returns.  A momentary edge/gesture rejection merely pauses
            # collection, preserving already observed recent candidates.
            self._recent.clear()
        cutoff = raw.capture_t_ms - self.config.anchor_window_ms
        self._recent = [item for item in self._recent if item.raw_observation.capture_t_ms >= cutoff]

    def _start_candidate_reason(self, estimate: FilteredWorkspaceEstimate) -> str:
        raw = estimate.raw_observation
        if not raw.valid:
            return "no_hand"
        if raw.control_gesture_active:
            return "gesture_control_hold"
        if not raw.inside_workspace:
            return "outside_workspace"
        if not self._is_operating_point(raw):
            return "inside_but_near_edge"
        if estimate.filtered_x_mm is None or estimate.filtered_y_mm is None:
            return "filtered_position_unavailable"
        return "eligible"

    def _recent_valid(self) -> list[FilteredWorkspaceEstimate]:
        return [item for item in self._recent if item.filtered_x_mm is not None and item.filtered_y_mm is not None]

    @staticmethod
    def _start_statistics(candidates: list[FilteredWorkspaceEstimate]) -> tuple[float, float, float]:
        """Describe the P0 cloud without changing the raw observations.

        The first SPACE deliberately remains a human decision.  These three
        numbers merely prevent a median from silently combining points that
        actually belong to different hand positions.
        """
        x = float(median(item.filtered_x_mm for item in candidates if item.filtered_x_mm is not None))
        y = float(median(item.filtered_y_mm for item in candidates if item.filtered_y_mm is not None))
        spread = max(math.hypot(item.filtered_x_mm - x, item.filtered_y_mm - y)
                     for item in candidates
                     if item.filtered_x_mm is not None and item.filtered_y_mm is not None)
        return x, y, float(spread)

    def _is_operating_point(self, raw: LiveWorkspaceObservation) -> bool:
        if not (raw.valid and raw.inside_workspace): return False
        assert raw.x_mm is not None and raw.y_mm is not None
        w, h = self.calibration.workspace_definition.width_mm, self.calibration.workspace_definition.height_mm
        m = self.config.operating_margin_mm
        return m <= raw.x_mm <= w - m and m <= raw.y_mm <= h - m

    def _maybe_publish_many(self, estimate: FilteredWorkspaceEstimate) -> tuple[RealtimeTaskEvent, ...]:
        raw = estimate.raw_observation
        assert self._epoch_capture_ms is not None
        source_t = raw.capture_t_ms - self._epoch_capture_ms
        available_t = raw.available_t_ms - self._epoch_capture_ms
        if self._last_event is not None and source_t <= self._last_event.source_t_ms:
            return ()
        if raw.control_gesture_active:
            # A confirmed L is a one-shot pen command, not perception loss.
            # Lock its XY to the last accepted point because forming an L can
            # physically move the fingertip even though the intended command
            # is purely semantic.  Subsequent held frames publish nothing.
            if not raw.control_gesture_toggled or self._last_event is None or not self._last_event.valid:
                return ()
            self._discard_pending_dwell()
            events: list[RealtimeTaskEvent] = []
            event = RealtimeTaskEvent(
                len(self._events), source_t, available_t, True, True,
                self._last_event.x_mm, self._last_event.y_mm, raw.pen_state,
                raw.stroke_id, None, self._z_for_pen(raw.pen_state),
            )
            self._events.append(event); self._last_event = event
            events.append(event)
            self.phase = LiveFollowPhase.PEN_TRANSITION
            self._latest_paused_estimate = estimate
            self._pen_transition_complete = False
            return tuple(events)
        if raw.control_gesture_candidate:
            # Before the L dwell completes, the index fingertip is allowed to
            # rotate, lift, or disappear for the detector's short grace.  The
            # raw observation is retained above, but it must not become an
            # ordinary no-hand/outside trajectory barrier halfway through a
            # deliberate semantic command.
            return ()
        if not self._is_operating_point(raw):
            reason = raw.invalid_reason or "outside_operating_region"
            # Do not flood the planner with the same barrier on every camera
            # frame.  One explicit boundary is enough; health still travels via
            # SOURCE_HEARTBEAT.
            if self._last_event is not None and not self._last_event.valid and self._last_event.invalid_reason == reason:
                return ()
            self._discard_pending_dwell()
            events: list[RealtimeTaskEvent] = []
            event = RealtimeTaskEvent(len(self._events), source_t, available_t, False, False, None, None, raw.pen_state, raw.stroke_id, reason, self._z_for_pen(raw.pen_state))
            self._events.append(event); self._last_event = event
            events.append(event)
            self.phase = LiveFollowPhase.VISION_GAP
            self._reacquire_candidates.clear()
            self._pending_reacquire_target = None
            self._active_reacquire_target = None
            self._latest_paused_estimate = None
            self._pen_transition_complete = False
            return tuple(events)
        point = self._mapped_point(estimate)
        if self._last_event is not None and self._last_event.valid:
            assert self._last_event.x_mm is not None and self._last_event.y_mm is not None
            moved = math.hypot(point[0] - self._last_event.x_mm, point[1] - self._last_event.y_mm) >= self.config.minimum_motion_mm
            due = source_t - self._last_event.source_t_ms >= self.config.heartbeat_ms
            semantic_change = (raw.pen_state, raw.stroke_id) != (self._last_event.pen_state, self._last_event.stroke_id)
            if not (moved or due or semantic_change): return ()
            if due and not moved and not semantic_change:
                # Preserve the latest quiet instant, but postpone materializing
                # it until it actually bounds a motion or EOF.  This turns a
                # long stationary run into one dwell endpoint rather than one
                # pointless IK/quintic chunk per heartbeat.
                self._pending_dwell = _PendingDwell(source_t, available_t)
                return ()
        previous = self._last_event
        self._mark_motion_restart_if_needed()
        events: list[RealtimeTaskEvent] = []
        event = RealtimeTaskEvent(
            len(self._events), source_t, available_t, True, True,
            point[0], point[1], raw.pen_state, raw.stroke_id, None,
            self._z_for_pen(raw.pen_state),
        )
        self._events.append(event); self._last_event = event
        events.append(event)
        if previous is not None and previous.valid and (raw.pen_state, raw.stroke_id) != (previous.pen_state, previous.stroke_id):
            # Keyboard P is a semantic command too.  It must obey exactly the
            # same queue handshake as a confirmed L, rather than slipping a
            # Z change into ordinary tracking behind the user's back.
            self.phase = LiveFollowPhase.PEN_TRANSITION
            self._latest_paused_estimate = estimate
            self._pen_transition_complete = False
        return tuple(events)

    def _discard_pending_dwell(self) -> None:
        """Forget a hold when a real barrier/semantic boundary closes its run."""
        self._pending_dwell = None

    def _mark_motion_restart_if_needed(self) -> None:
        """Turn a compressed hold into a planner-only stop/restart boundary."""
        pending = self._pending_dwell
        self._pending_dwell = None
        if pending is None:
            return
        if self._last_event is None or not self._last_event.valid:
            return
        if pending.source_t_ms <= self._last_event.source_t_ms:
            return
        self._pending_motion_restart = MotionRestartBoundary(
            source_t_ms=pending.source_t_ms,
            replay_t_ms=pending.available_t_ms,
        )

    def _is_operating_estimate(self, estimate: FilteredWorkspaceEstimate) -> bool:
        raw = estimate.raw_observation
        return (
            raw.valid and raw.inside_workspace and not raw.control_gesture_active
            and not raw.control_gesture_candidate and self._is_operating_point(raw)
            and estimate.filtered_x_mm is not None and estimate.filtered_y_mm is not None
        )

    def _mapped_point(self, estimate: FilteredWorkspaceEstimate) -> tuple[float, float]:
        assert self._mapping_offset_xy is not None
        assert estimate.filtered_x_mm is not None and estimate.filtered_y_mm is not None
        return (
            estimate.filtered_x_mm + self._mapping_offset_xy[0],
            estimate.filtered_y_mm + self._mapping_offset_xy[1],
        )

    def _rebase(self, estimate: FilteredWorkspaceEstimate, target_xy: tuple[float, float]) -> None:
        """Atomically make this measured hand pose correspond to ``target_xy``."""
        assert estimate.filtered_x_mm is not None and estimate.filtered_y_mm is not None
        self._mapping_offset_xy = (
            target_xy[0] - estimate.filtered_x_mm,
            target_xy[1] - estimate.filtered_y_mm,
        )

    def _remember_reacquire_candidate(self, estimate: FilteredWorkspaceEstimate) -> None:
        if not self._is_operating_estimate(estimate):
            # A fresh recovery window must contain one uninterrupted,
            # operating-region measurement run.  Retaining candidates across
            # outside frames used to let an old candidate influence a new one.
            self._reacquire_candidates.clear()
            return
        mapped = self._mapped_point(estimate)
        self._latest_paused_estimate = estimate
        self._reacquire_candidates.append((estimate, mapped))
        cutoff = estimate.raw_observation.capture_t_ms - self.config.anchor_window_ms
        self._reacquire_candidates = [item for item in self._reacquire_candidates if item[0].raw_observation.capture_t_ms >= cutoff]
        if len(self._reacquire_candidates) < self.config.minimum_reacquire_candidates:
            return
        xs = [item[1][0] for item in self._reacquire_candidates]
        ys = [item[1][1] for item in self._reacquire_candidates]
        x, y = float(median(xs)), float(median(ys))
        spread = max(math.hypot(px - x, py - y) for px, py in (item[1] for item in self._reacquire_candidates))
        if spread > self.config.maximum_reacquire_spread_mm:
            return
        raw = estimate.raw_observation
        self._pending_reacquire_target = ReacquireTarget(
            x, y, self._z_for_pen(raw.pen_state), raw.pen_state,
            tuple(item[0].raw_observation.frame_index for item in self._reacquire_candidates),
        )

    def _append_event(self, source_t: float, available_t: float, valid: bool, x: float | None, y: float | None, pen_state: str, stroke_id: int | None, reason: str | None) -> RealtimeTaskEvent:
        if self._last_event is not None and source_t <= self._last_event.source_t_ms:
            source_t = self._last_event.source_t_ms + 1e-3
        event = RealtimeTaskEvent(
            len(self._events), source_t, available_t, valid, valid,
            x, y, pen_state, stroke_id, reason, self._z_for_pen(pen_state),
        )
        self._events.append(event)
        self._last_event = event
        return event

    def _z_for_pen(self, pen_state: str) -> float:
        return self.config.contact_z_task_mm if pen_state == "DOWN" else self.config.lifted_z_task_mm
