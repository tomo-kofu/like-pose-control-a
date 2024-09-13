# like pose control a

このプロジェクトは、人の動作でパソコンを操作するツールです。

参考動画
https://youtu.be/cMB1TL8nV14


## 目次
- [概要](#概要)
- [インストール](#インストール)
- [使い方](#使い方)
- [ライセンス](#ライセンス)


## 概要
このツール(src)は、Mediapipe Pose(Hand、FaceMesh)を拡張します。
体のランドマーク(右手首等)の画面上の位置や、右手首の動作(パンチ)等でパソコンを操作することが可能です。
どんな動作で、どんな操作をさせるかは設定ファイルに設定します。
詳しくは、以下のページを参照ください。（作成途中）
  https://healthier-happier-tmkf.blogspot.com/

すでにビルド済みパッケージは、以下に公開しております。
### like pose control a
#### BOOTH
https://healthierhappier.booth.pm/items/4710857
#### Vector
https://www.vector.co.jp/soft/dl/winnt/util/se526558.html

### like hand control a
#### BOOTH
https://healthierhappier.booth.pm/items/5619947

### like facemesh control a
#### BOOTH
https://healthierhappier.booth.pm/items/5612598

## インストール

1. Mediapipeをgit clone ＆ build
以下のページを参考にしてください。
https://kunsen.net/2021/01/30/post-3543/


2. ソースファイル(src\demo_run_graph_main.cc)の更新
以下のファイルを、like pose control a版に更新する。
  <mediapipe-python-sample-main>\windows\mediapipe\mediapipe\mediapipe\examples\desktop\demo_run_graph_main.cc

3. Pose/Hand/Facemeshの切り替え
ソースの81行目付近の定義(以下)を切り替える(使わないのはコメントアウトする)

```
#define DETECT_MODE_POSE        1
//#define DETECT_MODE_HAND        1
//#define DETECT_MODE_FACE_MESH   1
```

ソースの場所は以下のURL
https://github.com/tomo-kofu/like-pose-control-a/blob/bf505cb3e9a2fcd185975acbcc26cf37421ee2dc/src/demo_run_graph_main.cc#L81

4. ビルド

4.1. Pose

参考：https://kunsen.net/2021/01/30/post-3543/#Pose_Tracking

bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 --action_env PYTHON_BIN_PATH=$env:PYTHON_PATH mediapipe/examples/desktop/pose_tracking:pose_tracking_cpu

4.2. Hand

bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 --action_env PYTHON_BIN_PATH=%PYTHON_PATH% mediapipe/examples/desktop/hand_tracking:hand_tracking_cpu

4.3. Facemesh

bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 --action_env PYTHON_BIN_PATH=$env:PYTHON_PATH mediapipe/examples/desktop/face_mesh:face_mesh_cpu


## 使い方

1. 設定ファイルを更新・作成

https://healthier-happier-tmkf.blogspot.com/2024/03/like-pose-control.html

それ以降については、まだ作成途中です。

2. 実行

「https://kunsen.net/2021/01/30/post-3543/#Pose_Tracking」を参考(Hand Trackingと記載されていますが、実際にはPose Trackingのことです。)

実行例：
設定ファイル(config.json)をカレントディレクトリに配置して、以下を実行。
> set GLOG_logtostderr=1 

### like pose control a
> bazel-bin\mediapipe\examples\desktop\pose_tracking\pose_tracking_cpu.exe   --calculator_graph_config_file=mediapipe/graphs/pose_tracking/pose_tracking_cpu.pbtxt

### like hand control a
> bazel-bin\mediapipe\examples\desktop\hand_tracking\hand_tracking_cpu.exe --calculator_graph_config_file=mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt

### like facemesh control a
> bazel-bin\mediapipe\examples\desktop\face_mesh\face_mesh_cpu.exe --calculator_graph_config_file=mediapipe/graphs/face_mesh/face_mesh_desktop_live.pbtxt

### 参考

#### Chrome dinoで遊ぶ動画
https://www.youtube.com/embed/3cf2XfCe3EI

#### KOF ALLSTARで遊ぶ動画
https://www.youtube.com/embed/5c_LNVqxTE8


## ライセンス

Apache License 2.0
(Mediapipeと同様です。)


