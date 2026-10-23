# oumuamua-controller

ロボット oumuamua を Android スマホ(横向き)から操作するコントローラです。スティックでの走行、リフトとグリッパの操作、タスクの開始、ロボットのコンソール(シェル)を 1 つの画面で扱います。スマホ側アプリ(Rust + Slint)と、ロボット側の ROS 2 橋渡しノード(Python)でできています。通信は UDP + MessagePack で、形式は `docs/protocol.md` に書いてあります。

## 構成

| 場所 | 役割 |
|---|---|
| `crates/proto` | メッセージの定義と符号化(`docs/protocol.md` に対応) |
| `crates/link` | UDP 通信、ロボットの発見、再送 |
| `crates/term` | コンソール画面の端末エミュレーション |
| `crates/app` | Slint のアプリ本体。Android(`android_main`)と PC の両方で動く |
| `ros/oumuamua_controller_bridge` | ROS 2 の橋渡しノード。ROS に依存しない部分と、ROS を使う `ros_io.py` / `node.py` に分かれる。ROS なしで試せる `fake_robot` つき |
| `docs/protocol.md` | 通信の仕様 |
| `docs/superpowers/` | 設計書と計画 |

`crates/COLCON_IGNORE` があるので、colcon は Rust 側を拾いません。

## ロボット側の準備

1. oumuamua のワークスペースに、このリポジトリを submodule として `src/` の下に追加します。

   ```sh
   cd <oumuamua のワークスペース>
   git submodule add <このリポジトリの URL> src/oumuamua-controller
   ```

2. ビルドします。

   ```sh
   colcon build --packages-up-to oumuamua_controller_bridge
   source install/setup.bash
   ```

3. 橋渡しノードを起動します。

   ```sh
   ros2 launch oumuamua_controller_bridge bridge.launch.py
   # 設定ファイルを差し替えるとき
   ros2 launch oumuamua_controller_bridge bridge.launch.py config:=/path/to/bridge.yaml
   ```

   launch の引数は `config`(パラメータファイル。省略時はパッケージの `config/bridge.yaml`)だけです。

設定は `ros/oumuamua_controller_bridge/config/bridge.yaml` にあります。リフトの段、速度の段、フィールドの寸法、障害物、棚、開始位置、ロボット半径などで、値は oumuamua の `task_commander.yaml` から写したものです。フィールドやロボットを変えたら、ここを直してください。

## スマホだけで試す(デモ)

ロボットも PC もなしで、アプリの動きを確かめられます。接続画面の紫の「デモ」ボタンを押すと、アプリの中に模擬のロボットが立ち上がり、ふだんと同じ経路でそれにつながります。

- 右端の帯と出力の面の上に、紫の「デモ」が出ている間は本物のロボットにはつながっていません。
- つないだ直後は停止状態です。「解除」を押すと手動で動かせます。手動・自動(回収・配置・経路)・コンソール(`help` など決まったコマンドだけの模擬シェル)が使えます。
- デモは端末の中(127.0.0.1)だけで動き、ネットワークには何も出しません。
- 右端の帯の「接続先」を押して「はい」を選ぶと、デモを止めて接続画面に戻り、本物のロボットを選べます。接続が終わる、アプリを閉じる、のときもデモは止まります。

## ROS なしで試す

ROS 2 がない PC でも、模擬のロボットと PC 版アプリで動作を確かめられます。

```sh
python3 -m venv .venv && .venv/bin/pip install msgpack pytest
cd ros/oumuamua_controller_bridge
../../.venv/bin/python -m oumuamua_controller_bridge.fake_robot   # UDP 47800 で待つ
```

別の端末で、PC 版アプリを起動します。

```sh
cargo run -p app
```

アプリのロボット一覧に出る模擬ロボットを選ぶか、`127.0.0.1:47800` を入力して接続します。`fake_robot` は平面の走行、タスクの進行(各段階 1 秒)、経路の走行(3 秒で終わる)、リフト、LiDAR(外壁のみ)を模擬します。

## APK の作り方と入れ方

必要なもの: Rust のターゲット `aarch64-linux-android`、`cargo-apk`、Android SDK と NDK(動作確認は NDK 29.0.14206865)。

```sh
export ANDROID_HOME=/opt/android-sdk
export ANDROID_NDK_ROOT=$ANDROID_HOME/ndk/29.0.14206865
cargo apk build -p app --release --lib
```

出力は `target/release/apk/app.apk` です。署名は `crates/app/Cargo.toml` に絶対パスで書いたデバッグ用キーストア(`/root/.android/debug.keystore`、パスワード `android`)を使うので、別の環境では自分のキーストアのパスに直してください。パッケージ名は `dev.oumuamua.controller` です。APK 自体は git に入れていません(`dist/` は無視されます)。

スマホへの入れ方は 2 通りあります。

- USB デバッグを有効にして接続し、`adb install -r target/release/apk/app.apk`。
- APK をファイル転送でスマホへコピーし、ファイルアプリから開く。提供元不明のアプリのインストールを許可する必要があります。

## 接続のしかた

1. スマホとロボット(橋渡しノードが動く計算機)を同じ Wi-Fi につなぎます。
2. アプリを開くと、ロボットの名乗り(UDP 47801 への同報)を受けて一覧に出ます。選んで接続します。
3. 一覧に出ないとき(同報が通らない Wi-Fi など)は、`IP アドレス:47800` を入力して接続します。

## 安全について

**認証はありません。** 同じ Wi-Fi にいる人なら誰でも、UDP 47800 に HELLO を送ってロボットの操作を奪えます(`takeover`)。そのままコンソールから bash を開くこともできます。コンソールに入力した内容(パスワードを含む)は、ロボット側の `~/.oumuamua_controller/console/` に記録されます。信頼できる閉じたネットワークだけで使ってください。


- コントローラからの CONTROL が 0.3 秒途切れたら、橋渡しノードはロボットを完全に止めます。吸着もオフにします。Wi-Fi が切れたり、アプリが落ちたり、スマホが画面オフになったりしたときも同じです。
- 停止は保持されます。止まったあとは、画面の「解除」を押すまで動きません。
- 速度の上限は、並進 2.0 m/s、旋回 2.5 rad/s です。
- 初めて実機で動かすときは、タイヤを浮かせるか、広い場所で低速から始めてください。

## ROS 側の人に確かめてほしいこと

ROS 2 はこの開発環境にないため、`ros_io.py` と `node.py` は `py_compile` までしか確認していません。実機または ROS 2 のある環境で確かめてほしい項目は `ros/oumuamua_controller_bridge/README.md` の "Please verify on a machine with ROS 2" の一覧にあります。

## 前提にしていること

- `/gripper{i}/zero_lift`(`std_srvs/Trigger`)があるものとして呼び出します。これは oumuamua 側のグリッパに追加する必要があります。`robomas_plugins` の `/robomas_zero_position`(ブランチ `feature/zero-position`)を使う想定です。ないときは、スマホに「がありません」というエラーが約 1 秒後に出ます。

## マルチタッチ

Slint 1.18.1 の `i-slint-core` を差し替えて(`third_party/i-slint-core`、変更点は `MULTITOUCH.md`)、指ごとにタッチを配るようにしています。スティックと旋回スライダ、上下ボタンなどを別々の指で同時に操作できます。

- 1 つの操作部品には 1 本の指だけが付きます。同じ部品に 2 本目の指を置いても無視されます。
- 地図の上では 2 本の指でピンチ(拡大縮小・回転)します。

## 開発

```sh
cargo build -p app
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
.venv/bin/pytest ros/oumuamua_controller_bridge/test
```
