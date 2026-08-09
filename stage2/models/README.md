# HandLandmarker 模型文件

`hand_landmarker.task` 是官方 MediaPipe Hand Landmarker 的任务模型文件，供
`stage2.hand_tracker` 进行单手的 21 点关键点检测。

模型二进制体积较大，且可从官方来源恢复，因此不提交到 Git；项目通过
`.gitignore` 忽略该文件，避免把可再下载的运行资产混入代码历史。

在新环境中，进入本目录后执行以下命令恢复模型：

```powershell
curl.exe -L --fail -o hand_landmarker.task `
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

创建 `HandTracker` 或运行相关软件验收前，本地必须存在该文件。
