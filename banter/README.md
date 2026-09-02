# Stage Banter — Tasks 1–7: Gesture Perception → Grammar → Memory → Behavior → Simulated Execution

This independent package converts a camera frame into a `GesturePerception`
(0/1/2 `HandObservation` values plus instantaneous `GestureEvidence`) and
then into one-shot `GestureEvent` values. Task 2 derives Token/Chord/Sequence
facts from those events; Task 3 records those facts in local immutable
interaction memory. Task 4 derives bounded persona affect and one semantic
behavior intent from that memory. Task 5 expands that intent into a nominal
MotionPlan; Task 6 derives a timing-only StyledMotionPlan and compiles it
through the existing Legacy5 quintic path. Task 7 adds a narrow localhost
TCP bridge to a Legacy5 digital-twin executor, with actual completion as the
busy-release authority. It remains a simulated single-action executor: no
hardware command, persistence, queue, preemption, or Stage 2/3 protocol
change is introduced.

## Task 1 vocabulary and contract

The official MediaPipe **HandLandmarker** is configured for two hands. Banter
uses its landmark rows as the only 0/1/2-hand presence fact, then applies a
small intentional landmark-rule vocabulary:

- `POINT_ONE`: only the index finger is clearly extended.
- `VICTORY_TWO`: index and middle are extended; ring/pinky are folded.
- `OPEN_PALM_FIVE`: all five digits are clearly extended.
- `THUMBS_UP`: upright thumb with the other four digits folded.
- `FIST`: index through pinky are folded.
- `MIDDLE_FINGER`: middle extended, with thumb/index/ring/pinky folded.

Thumb openness is measured relative to its own palm, not from thumb-joint
straightness alone. A thumb folded across a fist therefore remains `FIST`,
while `THUMBS_UP` requires that the thumb is both away from the palm and
oriented upward. A natural, close-to-palm thumb does not invalidate
`VICTORY_TWO`. `OPEN_PALM_FIVE` uses a separate lighter splay check, so a
natural slightly bent thumb can still count as five without weakening the
stricter three/Thumbs-Up boundaries.

All remaining/ambiguous patterns are `UNKNOWN`. In particular, 3, 4, and
half-formed poses are rejected instead of being
coerced into a supported command. The rule layer is isolated from the event
and grammar contracts, so a future gesture can be added as another rule (or a
different classifier backend) without changing downstream code.

`GestureEvent` is the only temporal output.  Its JSON-safe `to_dict()` gives
the schema version, event id, gesture, resolved `LEFT`/`RIGHT` side, source
frame, evidence/confirmation/availability timestamps, visible dwell duration,
derived geometry score, observed hand count, and source. Unknown/ambiguous sides
and unsupported/low-confidence labels remain evidence only; they cannot emit
an event.  `capture_t_ms` belongs to the camera frame; the live perceiver
samples `available_t_ms` only after MediaPipe inference returns, so it denotes
when that derived perception is actually available to downstream code.
If MediaPipe emits a transient partial per-hand result (its landmark or
handedness rows disagree), the frame is preserved without stopping the demo.
The hand remains visible as evidence; an unknown/ambiguous hand side cannot
produce an event.

The default temporal policy is per side: score entry/hold hysteresis of
`0.65/0.50`, `400 ms` visible dwell, `180 ms` dropout grace, and `280 ms`
release.  A held pose therefore prints exactly one event; it needs release and
a fresh dwell to emit again.  A brief candidate dropout pauses rather than
counts as dwell time.

## Task 2 grammar contract

`GestureEvent` itself is the canonical Token: Task 2 never rereads landmarks
or changes the input event.  `GestureGrammarEngine.update()` accepts exactly
one Task 1 event tuple and returns `GrammarUpdate(tokens, phrases)`.  Tokens
are preserved verbatim; a derived `GesturePhraseEvent` contains its phrase
form, stable name, source events/ids, capture-time start/completion, and a new
`available_t_ms` sampled after grammar matching completes.

The matcher uses `confirmed_capture_t_ms` for human interaction timing, not
inference availability.  Chord and Sequence matching are non-destructive
annotations: a Token may be provenance for both kinds of phrase; later Memory
or Behavior selection will decide response priority.

The initial deliberately small grammar is:

- `LEFT:FIST + RIGHT:VICTORY_TWO` within `450 ms` → `POWER_VICTORY` Chord.
- Same-hand `OPEN_PALM_FIVE → FIST → VICTORY_TWO`, step gap ≤ `2500 ms` and
  total duration ≤ `6000 ms` → `CHALLENGE_TRIUMPH` Sequence.
- Same-hand `OPEN_PALM_FIVE → POINT_ONE → THUMBS_UP` → `YOU_GOT_IT` Sequence.
- Same-hand `OPEN_PALM_FIVE → VICTORY_TWO → THUMBS_UP` → `HYPE_CONFIRM` Sequence.

Chord means near-simultaneous **confirmed events**.  Task 1 intentionally
does not export hand-release events, so Grammar does not infer whether an old
held hand is still physically present.  Sequences are contiguous per-hand;
other-hand Tokens do not interrupt them.

The live HUD is intentionally compact: `No hand` means no HandLandmarker row
arrived; `LEFT: UNKNOWN` / `RIGHT: UNKNOWN` means a hand was detected but its
pose was rejected; `HAND?: FIST` means the pose is clear but handedness is not.
Unknown pose or side cannot enter a candidate, emit a `GestureEvent`, or
become a Token.

## Task 3 interaction-memory contract

`InteractionMemory.update(grammar_update)` consumes exactly one Task 2
`GrammarUpdate` atomically and returns an immutable
`InteractionMemorySnapshot`. It preserves original Tokens and derived Phrases
in separate bounded chronological histories, while retaining cumulative totals
after old history entries are evicted. Token keys preserve `LEFT`/`RIGHT` plus
gesture; Phrase keys preserve form plus rule name.

The Snapshot has an explicit session id, revision and interaction-turn count;
total Token/Phrase counts; per-hand Token streaks; a separate Phrase streak;
last capture/input-availability timestamps; and JSON-safe `to_dict()` output.
Repeated same-key streaks use only capture time (`5000 ms` Token and `8000 ms`
Phrase default gaps). Right-hand Tokens never reset the left-hand streak, and
ordinary Tokens never reset the Phrase streak.

The reducer is a pure fold of previous Snapshot + GrammarUpdate; only the
stateful `InteractionMemory` wrapper samples its injected monotonic clock
after a successful fold to publish `available_t_ms`. Empty updates return the
same Snapshot by identity without a revision, turn, history, or clock update.
The full batch is validated before state changes: Token/Phrase ids and Token
capture order must advance; phrase source Tokens must be in the same update or
retained history and match exactly; a repeated `(form, name, source_event_ids)`
is rejected even with a fresh phrase id.

`memory.reset(new_session_id)` explicitly clears histories, totals, streaks,
timestamps, ids and phrase fingerprints. Task 3 intentionally does not save a
database, infer user traits, prioritize responses, or cause robot/behavior
actions.

## Task 4 persona and behavior contract

`PersonaBehaviorEngine.update(memory_snapshot)` accepts only the immutable
Task 3 `InteractionMemorySnapshot`; it never reads landmarks, invokes grammar,
or opens a camera. The incoming Snapshot must be from the same session and
exactly one Memory revision after the previous one. Replaying the same revision
returns the previous `Task4Update` by identity, without reading its clock or
emitting another behavior.

`PersonaProfile` contains stable baselines; `PersonaState` contains only live,
bounded `friendliness`, `playfulness`, `energy`, and `annoyance`. Appraisal is
declarative through the per-stimulus `PersonaConfig.response_rules`; there is
no global “positive/negative Token” class. Each supported Token and Phrase has
its own `behavior_name`, affect delta, priority and nominal duration. The
global variant vocabulary is deliberately limited to `DEFAULT`, `FIRM`, and
`FORCEFUL`. `DEFAULT` means the behavior's own Macro needs no extra style;
only same-hand repeated `FIST` events can escalate to `FIRM` and then
`FORCEFUL`. A held FIST remains one Task 1 event and never self-escalates.
Variants never replace the underlying Token/Phrase behavior and never produce
a continuous intensity value. Dynamic affect relaxes exponentially to the profile
baseline using **capture time** between accepted interactions. There is no
autonomous background timer: a later interaction is required to publish a
relaxed state.

`BehaviorEvent` is one immutable semantic intent with a concrete
`behavior_name`, discrete `variant`, capture/availability times, and complete
Token/Phrase provenance. A behavior is emitted only when no earlier nominal
behavior is active; new facts during that interval remain in Task 3 Memory and
update Persona, but receive the explicit `IGNORED_BUSY` admission and are not
queued. `OPEN_PALM_FIVE` is the Sequence wake gesture: its standalone behavior
and its compatible following Tokens are delayed until a Sequence completes or
the relevant prefix expires. Chord-capable Tokens are delayed for their Chord
window for the same reason. Thus a completed Phrase gets its own response
instead of first triggering the response of an early Token. Chord provenance
is sorted by source event id, while its LEFT/RIGHT semantic rule remains intact.
This is a semantic intent only—not a robot command, text response, animation,
motion plan, MATLAB message, or LLM call. `engine.reset(new_session_id)`
returns the new profile baseline with no behavior and restarts behavior ids.

## Task 5 nominal motion-plan contract

Task 5 is an offline boundary, not a second robot controller:

```text
BehaviorEvent -> MotionPlan -> CompiledBanterMotion (MATLAB q(t))
```

`MotionPlanner` maps every configured Task 4 `behavior_name` to exactly one
declarative `MotionMacro`; there is no generic response or silent fallback.
Each `MotionPlan` retains the complete source `BehaviorEvent`, its discrete
`variant`, ordered `MOVE_POSE`/`HOLD`/`RETURN_NEUTRAL` steps, and a planning
availability timestamp sampled after expansion. It never contains joint
coordinates or a continuous intensity.

The initial Macro names are `POINT_JAB`, `SALUTE_SWEEP`, `SIDE_WAVE`,
`UPWARD_ACK`, `COUNTER_JAB`, `RECOIL_DISMISS`, `POWER_POSE`, `TRIUMPH_ARC`,
`PROUD_DOUBLE_NOD`, and `HYPE_DOUBLE_BOUNCE`. Every Macro ends at `NEUTRAL`.
The source behavior determines the action; variants remain explicit discrete
provenance. Task 6 maps them only to timing parameters without changing the
Macro identity, named poses, or joint angles.

MATLAB owns the Legacy5 numerical realization. The Task 5 pose library uses
only named conservative joint poses and calls the existing public
`load_robot_context` plus `generate_quintic_continuous_joint_trajectory`.
The compiler rejects unknown poses, non-neutral endings, joint/velocity/
acceleration failures, and quintic overshoot. It does not change Stage 3's
live TCP executor, does not open a camera, and does not command hardware.
It first groups contiguous movement steps between explicit `HOLD` steps, so
named intermediate poses are genuine quintic waypoints. A `HOLD` is compiled
only as a stationary segment at the current pose. If an otherwise maximal
movement run is rejected by existing Stage 3 overshoot evidence, Banter first
uses a local quintic boundary guard: at an affected reversal waypoint it sets
only that joint's internal qd to zero, then deterministically damps only the
affected qdd if necessary. Every candidate regenerates the complete same-waypoint
quintic and must improve overshoot while satisfying existing v/a/joint checks.
This is not a new trajectory backend and does not alter frozen Stage 3. If the
guard cannot eliminate the evidence, the compiler uses the existing fewest
safe contiguous sub-runs; it never invents a `HOLD`, alters a pose, or bypasses
safety checks. `movement_run_guard_evidence` records the before/after evidence
and the exact qd/qdd boundary edits for each final segment group.

### Low-cost behavior extension point

Add a supported Gesture to `GestureKind`/the landmark recognizer first, then
add one `BehaviorRule` to `PersonaConfig.response_rules`. Add a Chord or
Sequence separately to `GestureGrammarConfig`; a new wake Sequence must start
with `PersonaConfig.sequence_wake_gesture`. No routing `if` branch is needed.
Use this request template when designing one:

```text
Add one Banter behavior rule only. Stimulus: <GestureKind or PhraseForm/name>.
Behavior name: <UPPERCASE semantic intent>. Base discrete variant: <UPPERCASE>.
Optional variants and gates: <variant + persona/streak thresholds>.
Affect delta: <friendliness, playfulness, energy, annoyance>.
Priority: <integer>; nominal duration: <milliseconds>.
If it is a Phrase, include the exact Chord/Sequence and confirm its wake
gesture. Do not introduce global positive/negative categories, continuous
intensity, robot motion, MATLAB, or changes to Stage 2/3.
```

## Test, verify, and C920 manual demo

```powershell
python -m pytest tests/test_banter_task1_contracts.py tests/test_banter_task1_perception.py tests/test_banter_task1_events.py -q
python -m banter.verify_task1
python -m pytest tests/test_banter_task2_grammar_contracts.py tests/test_banter_task2_grammar.py tests/test_banter_task2_integration.py -q
python -m banter.verify_task2
python -m pytest tests/test_banter_task3_memory_contracts.py tests/test_banter_task3_memory.py tests/test_banter_task3_integration.py -q
python -m banter.verify_task3
python -m pytest tests/test_banter_task4_contracts.py tests/test_banter_task4_personality.py tests/test_banter_task4_behavior.py tests/test_banter_task4_integration.py -q
python -m banter.verify_task4
python -m pytest tests/test_banter_task5_contracts.py tests/test_banter_task5_library.py tests/test_banter_task5_integration.py -q
python -m banter.verify_task5
```

The verify is offline: it loads the model on a blank synthetic frame and
checks one-shot dwell behavior with synthetic evidence.  It never opens a
camera.

For hardware acceptance, first identify the currently enumerated C920 index
(Windows can change it), then use ordinary local PowerShell:

```powershell
python -m banter.demo_gesture_events --camera-index <actual-index>
```

Hold one supported pose for roughly 0.4 s and confirm one printed event;
continuing to hold it must not produce repeats. Hold 3 and 4 separately and
confirm they remain `UNKNOWN` without an event. Then hold FIST with the other
hand showing 3: the two skeleton labels must be FIST and UNKNOWN respectively.
Verify `M` mirrors only the preview while `Q` exits cleanly.

For Task 2 hardware acceptance, use the same actual C920 index:

```powershell
python -m banter.demo_gesture_grammar --camera-index <actual-index>
```

Confirm ordinary Tokens first. Then hold left `FIST` and right `VICTORY_TWO`
near-simultaneously for one `POWER_VICTORY`; each Sequence begins with one-hand
`OPEN_PALM_FIVE`, then perform either `FIST → VICTORY_TWO`, `POINT_ONE →
THUMBS_UP`, or `VICTORY_TWO → THUMBS_UP`. Wrong order, wrong hand side, or a
window timeout must not produce a phrase.

Task 3 has its own manual entry point; it does not alter the Task 2 demo:

```powershell
python -m banter.demo_interaction_memory --camera-index <actual-index>
```

Its HUD shows session/revision, Token/Phrase totals, current LEFT/RIGHT/Phrase
streaks, last facts, and recent facts. `M` mirrors, `R` starts a complete new
Banter session (new stabilizer, grammar and memory), and `Q` exits. When Task 2
HUD is correct but these fields are not, it is a Task 3 bug; when Task 2 itself
does not produce the expected Token/Phrase, diagnose Task 1/2 rather than add a
Memory compensation heuristic.

Task 4 adds a separate manual demo and leaves the Task 3 demo unchanged:

```powershell
python -m banter.demo_personality_behavior --camera-index <actual-index>
```

Its compact HUD reports the four persona axes, latest concrete semantic
behavior (`behavior_name:variant`), and admission outcome. Hold thumbs-up for
`THUMBS_UP_ACK`; form the Task 2 `POWER_VICTORY` chord for `POWER_POSE_REPLY`;
try `OPEN → FIST → VICTORY_TWO` for `TRIUMPH_FLOURISH`, and hold middle finger
for `OFFENDED_RETORT`. While a behavior is shown as active, another Token must
be recorded in Memory/Persona but produce no second behavior. `R` starts a
fresh Persona/Memory session, `M` mirrors only the preview, and `Q` exits. The
console prints ordered `[CLEANUP] camera_release`, `windows_destroy`, and
`perceiver_close` markers to identify any backend-specific exit stall. This is
solely a C920 visual/logging acceptance demo: it sends no command to Stage 3,
MATLAB, or a robot.

Task 5 is intentionally a MATLAB-only offline motion gallery; it does not
need a C920. Generate a disposable fixture in PowerShell, then compile and
display it in MATLAB:

```powershell
python -m banter.export_task5_motion_plans outputs/banter_task5_plans.json
```

```matlab
addpath(genpath(fullfile(pwd, 'stage3')))
addpath(fullfile(pwd, 'stage1'))
addpath(fullfile(pwd, 'banter', 'matlab'))
results = runtests('tests/test_banter_task5_motion.m'); disp(table(results))
demo_banter_motion_library('outputs/banter_task5_plans.json')
```

The MATLAB test compiles every named Task 5 pose and verifies the existing
Legacy5 joint, velocity, acceleration, finite-value, overshoot, and neutral
return evidence. The gallery must show recognizably different motions without
joint jumps; tune only Task 5 pose values if a simulated motion is unclear.

## Task 6 discrete timing-style contract

Task 6 is a derived offline boundary:

```text
BehaviorEvent -> MotionPlan -> StyledMotionPlan -> CompiledStyledBanterMotion
```

It never changes a Macro's step count, step kinds, named poses, or joint
angles. `DEFAULT` is an identity style for every normal behavior. Only
`FIST_COUNTER:FIRM` and `FIST_COUNTER:FORCEFUL` have explicit overrides;
they alter `MOVE_POSE`, `HOLD`, and `RETURN_NEUTRAL` durations. The existing
Task 5 compiler remains the numerical and safety authority, so a fast timing
request can still be safely retimed.

For `FIST_COUNTER` specifically, MATLAB uses the existing
`contopptraj` / TOPP-RA diagnostic as the required fastest-safe non-`HOLD`
timing authority under the current Stage 3 limits. The Task 5 quintic output
is retained only as the diagnostic's required piecewise-geometric path input;
its safety-stretched duration is neither selected as a timing baseline nor
used as a fallback trajectory. TOPP-RA must pass joint/velocity/acceleration
and path-deviation evidence or FIST compilation stops with an explicit error.
`FORCEFUL` uses TOPP-RA Tmin, `FIRM` uses 1.15×, and `DEFAULT` uses 1.35×.
`HOLD` keeps the duration requested by its own style profile. This preserves a
visible `DEFAULT > FIRM > FORCEFUL` execution-time ordering without changing
named poses, joint angles, the Macro, or the limits.
The TOPP-RA diagnostic tries `100`, `200`, `500`, then `1000` samples and
accepts the first count that passes the existing safety and path guards.  The
original split FIST path passed at 200; its restored continuous reversal path
requires 500 (100/200 are too coarse for the `1e-5 m` FK-path guard), while
500 and 1000 agree on Tmin.
`requested_styled_duration_s` remains the incoming style request;
`resolved_timing_duration_s` is the safety-aware derived duration used by the
compiler. Current Legacy5 v/a values originate from the explicitly labelled
Stage 3 planning-default context, not verified hardware ratings.
The Task 6 output records `retiming_method`, quintic-geometry
`baseline_duration_s`, `toppra_duration_s`, path-deviation evidence, and an
empty successful `fallback_reason` compatibility field, plus
`toppra_sample_count`; this is a derived
Banter artifact and does not change the frozen Stage 3 formal baseline.

Generate and inspect the compact twelve-plan fixture without a C920:

```powershell
python -m pytest tests/test_banter_task6_contracts.py tests/test_banter_task6_styling.py tests/test_banter_task6_integration.py -q
python -m banter.verify_task6
python -m banter.export_task6_styled_plans outputs/banter_task6_styled_plans.json
```

```matlab
addpath(genpath(fullfile(pwd, 'stage3')))
addpath(fullfile(pwd, 'stage1'))
addpath(fullfile(pwd, 'banter', 'matlab'))
results = runtests('tests/test_banter_task6_motion.m'); disp(table(results))
audit_banter_fist_timing('outputs/banter_task6_styled_plans.json')
demo_banter_motion_styles('outputs/banter_task6_styled_plans.json', 'FIST_COUNTER')
```

The FIST gallery plays `DEFAULT`, `FIRM`, and `FORCEFUL` in order.  It should
show timing/hold escalation while preserving the exact same named pose path.
The audit prints each continuous segment's requested and actual durations,
time-stretch factor, initial velocity/acceleration ratios, dominant joint,
TOPP-RA/quintic-geometry A/B timing evidence and the timing-limit source
before the gallery is used for visual acceptance. The gallery preserves each
trajectory's original time clock (including `HOLD`) but draws a default
25 Hz subset of dense TOPP-RA samples, so rendering overhead cannot extend the
visible action; pass a fourth `renderRateHz` argument to change only draw rate.
Task 6 still does not open a camera, use TCP, change Stage 2/3, or command a
robot.

## Task 7 live interaction gate and simulated execution

Task 7 composes the existing contracts without changing their ownership:

```text
Task 1 GestureEvent
  -> control-only InteractionArmGate
  -> Task 2 / 3 / 4
  -> Task 5 MotionPlan -> Task 6 StyledMotionPlan
  -> banter_execution_v1 TCP
  -> precompiled Legacy5 q(t) digital twin
  -> MOTION_COMPLETED -> release Task 4 execution lock
```

The live demo starts `DISARMED`.  Hold `LEFT:OPEN_PALM_FIVE` and
`RIGHT:OPEN_PALM_FIVE` to toggle ARMED/DISARMED.  The chord is control-only:
it does not create a Token, Phrase, Memory fact, Persona change, Behavior, or
motion.  Task 1 remains active while DISARMED; all ordinary events are simply
dropped before Task 2.  A held chord toggles once and must lose at least one
stable FIVE (using Task 1's existing release/dropout semantics) before it can
toggle again.  A short control-resolution window preserves ordinary single
FIVE behavior and wake-sequences without allowing a later second FIVE to
retroactively contaminate business Grammar.

Task 7 permits exactly one active action.  On a Task 4 `BehaviorEvent`, Python
first acquires the executor lock and enters `DISPATCHED`, then builds and sends
the styled plan.  `MOTION_ACCEPTED` and `MOTION_STARTED` do not create busy;
they only confirm the already-local lock.  Only a matching, provenance-checked
`MOTION_COMPLETED` releases it.  While ARMED and busy, Grammar/Memory/Persona
continue but new Behavior candidates are `IGNORED_BUSY`: no queue and no later
replay.  DISARM never preempts an already accepted motion; MATLAB drains it to
neutral first.

Generate the disposable twelve-plan fixture and run the offline checks:

```powershell
.venv\Scripts\python.exe -m pytest tests/test_banter_task7_interaction_gate.py tests/test_banter_task7_protocol.py tests/test_banter_task7_runtime.py -q
.venv\Scripts\python.exe -m banter.verify_task7
.venv\Scripts\python.exe -m banter.export_task6_styled_plans outputs/banter_task6_styled_plans.json
```

```matlab
addpath(genpath(fullfile(pwd, 'stage3')))
addpath(fullfile(pwd, 'stage1'))
addpath(fullfile(pwd, 'banter', 'matlab'))
results = runtests('tests/test_banter_task7_execution.m'); disp(table(results))
run_banter_tcp_executor('outputs/banter_task6_styled_plans.json', 51012)
```

With that MATLAB server running, this deterministic no-camera rehearsal must
complete one `THUMBS_UP_ACK` and receive its real completion acknowledgement:

```powershell
.venv\Scripts\python.exe -m banter.demo_task7_tcp_replay --port 51012
```

Then, in a normal PowerShell after confirming the current C920 index:

```powershell
.venv\Scripts\python.exe -m banter.demo_banter_live_execution --camera-index <actual-index> --port 51012
```

`Q` releases the camera/UI immediately, requests `END_SESSION`, and lets an
already active simulated action finish at neutral before the TCP side closes.
