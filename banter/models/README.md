# Banter 模型依赖

Banter Task 1 reuses the official MediaPipe HandLandmarker asset already
documented at `stage2/models/README.md`. Its independent Banter instance is
configured for two hands; it does not use `gesture_recognizer.task` at runtime.

An older local `gesture_recognizer.task` may remain ignored in this folder,
but it is neither required nor read by the current Banter demo. Do not put
binary model files into source control.
