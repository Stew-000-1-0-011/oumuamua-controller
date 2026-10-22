# oumuamua コントローラ 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Android スマホ(Rust + Slint)と、ロボット側の ROS 2 橋渡しノード(Python)を作り、UDP で oumuamua を操作・監視できるようにする。

**Architecture:** スマホは `proto`(メッセージ)→ `link`(UDP・再送・50 Hz の操縦送信)→ `app`(Slint 画面)の 3 層に、端末の行編集と出力解釈の `term` を足す。橋渡しノードは、ROS に依存しない部分(通信・停止の判断・シェル・要求の処理)と、rclpy を使う `ros_io.py` を分け、ROS の無い PC でも `fake_robot` で全体を動かせるようにする。

**Tech Stack:** Rust 2024、Slint =1.18.1、rmpv、tiny-skia、cargo-apk / Python 3.12+、msgpack、rclpy(ROS 2 Lyrical)、pytest

**Spec:** `docs/superpowers/specs/2026-10-22-oumuamua-controller-design.md` と、バイトの並びを決めた `docs/protocol.md`。どのタスクも両方を読むこと。

## Global Constraints

- メッセージの形・番号・単位・ポート・時間は `docs/protocol.md` のとおり。勝手に変えない。変えるなら protocol.md も直す。
- Slint は `=1.18.1` に固定(petaricon と同じ)。Android は `backend-android-activity-06`、PC は `backend-winit` + `renderer-femtovg`。
- 日本語フォントは petaricon の `crates/app/fonts/NotoSansJP-subset.otf` を `crates/app/fonts/` に複製し、Slint の初期化前に `SLINT_DEFAULT_FONT` で指定する(petaricon `crates/app/src/font.rs` と同じやり方)。
- 画面の文言は日本語。タップの的は 44px 以上。横向き。
- 橋渡しノードの ROS に依存しないモジュールは `rclpy` を import しない(`ros_io.py` と `node.py` だけが import する)。
- 停止の判断: CONTROL が 0.3 秒途切れたら停止。速度の上限は並進 2.0 m/s、旋回 2.5 rad/s。
- ROS 2 はこの開発環境に無い。`ros_io.py` / `node.py` は `python3 -m py_compile` と、ROS がある人の確認に回す。インストールしない。
- コミットは小さく。Rust は `cargo test --workspace` と `cargo clippy --workspace -- -D warnings`、Python は `.venv/bin/pytest ros/oumuamua_controller_bridge/test` が通ってからコミット。
- colcon が Rust 側を拾わないよう、`crates/COLCON_IGNORE` を置く。

## Review Focus

1. **Wi-Fi の一時的な途切れ**: 数秒切れて戻ったら、橋渡しノードは停止のまま、スマホは「停止中(通信断)」を表示し、解除で手動に戻れる。同じ session で CONTROL が再開すれば再接続の手続きは要らない(Task 7・Task 4 のテスト)。
2. **橋渡しノードの再起動**: スマホ側は 2 秒 STATE が来なければ新しい session で HELLO し直し、つながったらタブの中身を SHELL_OPENED で置き換える(Task 4・Task 7)。
3. **2 本指の同時操作**: スティックと旋回スライダーを同時に触る。Slint のマルチタッチが効くか実機で確かめ、効かなければ 1 つの TouchArea で 2 本の指を扱う形に変える(Task 13)。
4. **大量の出力**: `ros2 topic echo /scan` のような出力が続いても、操縦(CONTROL)と STATE が遅れない。橋渡しノードは未確認 64 個で送るのを待ち、出力はタブの 256 KiB の取っておきに溜める(Task 3・Task 6・Task 7)。
5. **UTF-8 の切れ目と制御文字**: 出力のかたまりの境目で多バイト文字が切れても化けない。知らない制御の並びは捨てる(Task 10)。

---

## ファイル構成

```
Cargo.toml                         workspace(crates/*)
crates/COLCON_IGNORE
crates/proto/src/lib.rs            Datagram / Msg / State の型と符号化
crates/proto/tests/golden.rs       testdata/golden の読み書き
crates/link/src/{lib.rs,reliable.rs,client.rs,discovery.rs}
crates/term/src/{lib.rs,keys.rs,history.rs,editor.rs,ansi.rs,scrollback.rs}
crates/app/{Cargo.toml,build.rs,fonts/,ui/*.slint,src/*.rs}
testdata/golden/*.msgpack          Python が作り、両方が確かめる見本
ros/oumuamua_controller_bridge/
  package.xml setup.py setup.cfg resource/oumuamua_controller_bridge
  config/bridge.yaml launch/bridge.launch.py
  oumuamua_controller_bridge/
    protocol.py reliable.py supervisor.py handlers.py task_memory.py
    shells.py completion.py server.py rosio.py fake_robot.py
    ros_io.py node.py
  test/  make_golden.py test_*.py
```

Python のテスト環境: `python3 -m venv .venv && .venv/bin/pip install msgpack pytest`(`.venv/` は .gitignore)。

---

### Task 1: プロトコル(Python)と見本

**Files:**
- Create: `ros/oumuamua_controller_bridge/` の package.xml(exec_depend: rclpy, std_msgs, std_srvs, geometry_msgs, nav_msgs, sensor_msgs, rcl_interfaces, tf2_ros, mini_shirasu_ros, gripper, task_commander, python3-msgpack)、setup.py(entry_points: `bridge = oumuamua_controller_bridge.node:main`, `fake_robot = oumuamua_controller_bridge.fake_robot:main`)、setup.cfg、resource、`__init__.py`
- Create: `oumuamua_controller_bridge/protocol.py`、`test/make_golden.py`、`test/test_protocol.py`、`testdata/golden/*.msgpack`、`.gitignore`(`.venv/`, `target/`, `__pycache__/`)、`crates/COLCON_IGNORE`

**Interfaces:**
- Produces(protocol.py):
  - 定数: `PORT = 47800`、`BEACON_PORT = 47801`、データグラムの番号 `HELLO=0 … BYE=8`、Msg の番号 `RELEASE_STOP=10 … SHELL_CLOSED=63`(protocol.md の名前どおり)、`MAX_POINTS_PER_SCAN = 280`、`MAX_SHELL_CHUNK = 900`
  - `encode(value: list) -> bytes`(`msgpack.packb(value, use_bin_type=True, use_single_float=False)`)
  - `decode(data: bytes) -> list | None`(配列で先頭が int でなければ None。壊れたデータも None)
  - `pack_points(points: Sequence[tuple[float, float]]) -> list[bytes]`(m → mm の i16 LE、範囲外は ±32767 に丸める、280 点ごとに分ける。点が 0 個なら `[b""]`)
  - `unpack_points(data: bytes) -> list[tuple[float, float]]`
- `make_golden.py`: 見本の一覧 `SAMPLES: dict[str, list]` を持ち、`python3 test/make_golden.py <出力先>` で `<名前>.msgpack` を書く。データグラム 9 種すべてと、REL に包んだ Msg 全種(protocol.md 4 章の表の全行)を含める。State は全欄が埋まったもの(`state_full`)と nil だらけのもの(`state_empty`)の 2 つ。実数には 0.5 や -1.25 のような 2 進で割り切れる値を使う。

- [ ] **Step 1:** `test_protocol.py` を書く: `test_golden_files_match_samples`(SAMPLES を encode したものが `testdata/golden/<名前>.msgpack` と一致)、`test_decode_roundtrip`(全見本で `decode(encode(x)) == x`)、`test_decode_rejects_garbage`(`b"\xc1"`、`encode([])`、`encode(["x"])`、`encode({})` が None)、`test_pack_points_splits_and_rounds`(600 点 → 3 つ、280/280/40 点。`(0.0015, -40.0)` → `(2 mm を経て 0.002, -32.767)`。空 → `[b""]`)
- [ ] **Step 2:** `.venv/bin/pytest ros/oumuamua_controller_bridge/test/test_protocol.py` が失敗することを確かめる
- [ ] **Step 3:** protocol.py と make_golden.py を書き、`python3 ros/oumuamua_controller_bridge/test/make_golden.py testdata/golden` で見本を作る
- [ ] **Step 4:** テストが通ることを確かめる
- [ ] **Step 5:** コミット `Add protocol encoding and golden samples (Python)`

### Task 2: プロトコル(Rust の proto クレート)

**Files:**
- Create: `Cargo.toml`(workspace、`members = ["crates/*"]`、resolver 3、edition 2024)、`crates/proto/Cargo.toml`(rmpv、thiserror)、`crates/proto/src/lib.rs`、`crates/proto/tests/golden.rs`

**Interfaces:**
- Consumes: `testdata/golden/*.msgpack`(Task 1)
- Produces:
  - `pub enum Datagram { Hello { session: u32, client_name: String, takeover: bool }, Welcome { session: u32, robot_name: String, accepted: bool, reason: String }, Control(Control), State { session: u32, seq: u32, ack_seq: u32, ack_t_ms: u32, body: State }, Scan { session: u32, scan_id: u32, frag: u8, frag_count: u8, points: Vec<(i16, i16)> }, Rel { session: u32, seq: u32, msg: Msg }, Ack { session: u32, cum: u32 }, Beacon { robot_name: String, port: u16 }, Bye { session: u32, reason: String } }`
  - `pub struct Control { session: u32, seq: u32, t_ms: u32, vx: f64, vy: f64, wz: f64, field_frame: bool, jog_left: i8, jog_right: i8, estop: bool }`
  - `pub struct State { mode: Mode, stop_reason: String, pose: Option<[f64; 3]>, vel: Option<[f64; 3]>, wheels: Vec<Wheel>, grippers: [Option<Gripper>; 2], task: Option<TaskState>, path: Option<(String, bool)>, tracker_enabled: bool }`、`pub enum Mode { Manual = 0, Auto = 1, Stopped = 2 }`、`Wheel { vdc: f64, ok: bool }`、`Gripper { height, goal: f64, moving, arrived, suction, feedback: bool }`、`TaskState { kind: u8, step: String, args: [i32; 4] }`
  - `pub enum Msg`: protocol.md 4 章の全行を 1 対 1 の変種にする(例 `RunTask { req: u32, kind: u8, gripper: i32, block: i32, column: i32, row: i32 }`、`ShellInput { req: u32, tab: String, data: Vec<u8> }`、`Config { robot_name, paths: Vec<String>, lift_presets: Vec<(String, f64)>, lift_max: f64, speed_levels: [(f64, f64); 3], blocks: u8, columns: u8, rows: u8 }`、`Field { walls: Vec<[f64; 4]>, shelf: [f64; 4], start: [f64; 3], robot_radius: f64 }`、`PathShape { name, points: Vec<(i16, i16)> }`、`TaskMemory { holding: [bool; 2], picked: Vec<u8>, placed: Vec<(u8, u8)> }`、`Log { stamp_ms: u64, level: u8, name: String, text: String }`)
  - `impl Datagram { pub fn encode(&self) -> Vec<u8>; pub fn decode(bytes: &[u8]) -> Result<Datagram, DecodeError> }`
  - `pub fn points_to_m(points: &[(i16, i16)]) -> Vec<(f32, f32)>`
  - 定数 `PORT`, `BEACON_PORT`, `MAX_POINTS_PER_SCAN`
- 余分な末尾の要素は読み飛ばす。足りない・型が違うときは `DecodeError`。

- [ ] **Step 1:** `tests/golden.rs`: `every_golden_file_roundtrips`(`testdata/golden` の全ファイルを decode → encode して同じバイト列)、`golden_set_is_complete`(見本のファイル名が 9 + Msg 種の数以上ある)、`extra_trailing_fields_are_ignored`(CONTROL の配列の末尾に 1 要素足したものが読める)、`wrong_types_are_errors`(CONTROL の vx が文字列なら Err)
- [ ] **Step 2:** 失敗を確かめる(`cargo test -p proto`)
- [ ] **Step 3:** lib.rs を書く。rmpv の `Value` を組み立てて `rmpv::encode::write_value` で書く(整数は最短形になる)
- [ ] **Step 4:** `cargo test -p proto` と clippy が通る
- [ ] **Step 5:** コミット `Add proto crate matching the golden samples`

### Task 3: 確実に届ける流れ(Python)

**Files:** Create `oumuamua_controller_bridge/reliable.py`、`test/test_reliable.py`

**Interfaces:**
- Produces:
  - `class ReliableSender: __init__(window=64, rto_ms=100)`、`push(msg: list) -> None`、`poll(now_ms: int) -> list[tuple[int, list]]`(まだ送っていないものと、最後に送ってから rto_ms 経ったものを返す。窓の外は返さない)、`on_ack(cum: int) -> None`、`pending() -> int`(未確認 + 未送信の数)、`reset() -> None`
  - `class ReliableReceiver: on_rel(seq: int, msg: list) -> list[list]`(順番どおりに渡せるようになったものを返す)、`cum: int`、`reset()`
- 番号は u32 で増やす(テストでは桁あふれを扱わなくてよいが、比較は `seq <= cum` のような素直な比較でよい。2^32 個は送らない前提)。

- [ ] **Step 1:** テスト: `test_in_order_delivery`、`test_out_of_order_buffered`(3,2,1 の順で来ても 1,2,3)、`test_duplicates_dropped`、`test_beyond_window_dropped`(cum+65 は捨てる)、`test_retransmit_after_rto`(送って 99 ms では再送しない、100 ms で再送)、`test_window_limits_inflight`(100 個 push → poll は 64 個、ack 10 で次の 10 個)、`test_lossy_link_simulation`(乱数の種を固定し、30% 落とす・入れ替える・重複させる経路で 500 個が順番どおり全部届く)
- [ ] **Step 2:** 失敗を確かめる
- [ ] **Step 3:** 実装
- [ ] **Step 4:** 通ることを確かめる
- [ ] **Step 5:** コミット `Add reliable channel (Python)`

### Task 4: link クレート(Rust の再送・接続・名乗りの受信)

**Files:** Create `crates/link/{Cargo.toml,src/lib.rs,src/reliable.rs,src/client.rs,src/discovery.rs}`、`crates/link/tests/client.rs`

**Interfaces:**
- Consumes: `proto::{Datagram, Msg, Control, State}`
- Produces:
  - `reliable::{Sender<T: Clone>, Receiver<T>}`: Task 3 と同じ振る舞い。時刻は `u64` ms を引数で受ける
  - `pub struct ControlInput { vx: f64, vy: f64, wz: f64, field_frame: bool, jog_left: i8, jog_right: i8, estop: bool }`(Default は全部 0 / false)
  - `pub enum LinkEvent { Connected { robot_name: String }, Rejected { reason: String }, State { body: State, rtt_ms: Option<u32> }, Scan(Vec<(f32, f32)>), Msg(Msg), Lost, Bye { reason: String } }`
  - `pub struct Client`: `Client::start(addr: SocketAddr, client_name: String, takeover: bool) -> (Client, std::sync::mpsc::Receiver<LinkEvent>)`、`set_control(&self, c: ControlInput)`、`next_req(&self) -> u32`、`send(&self, msg: Msg)`、`stop(self)`(BYE を送ってスレッドを終える)
  - `pub struct Discovery`: `Discovery::start() -> (Discovery, Receiver<(String, SocketAddr)>)`(BEACON_PORT で受け、`port` を使った宛先を返す)
- client のスレッド: 20 ms ごとに CONTROL(seq 1 から)と再送の poll。WELCOME までは 0.5 秒ごとに HELLO。STATE が 1 秒来なければ `Lost` を 1 回出す。2 秒来なければ新しい session で HELLO からやり直し、送信待ちの REL は捨てる。SCAN は scan_id ごとにそろったら `Scan`。REL を受けたら ACK を返す。rtt は `now - ack_t_ms`。

- [ ] **Step 1:** `reliable.rs` の単体テスト(Task 3 の 7 つと同じ内容)と、`tests/client.rs`: `handshake_and_control_stream`(テストの中で UdpSocket を橋渡しノード役にし、HELLO に WELCOME を返すと `Connected`、その後 CONTROL の seq が増えて届く)、`reliable_msgs_are_acked_and_delivered`、`state_rtt_is_reported`、`rehello_after_two_seconds_silence`(STATE を止めると `Lost`、その後 session の違う HELLO が来る)、`scan_fragments_are_joined`
- [ ] **Step 2:** 失敗を確かめる
- [ ] **Step 3:** 実装
- [ ] **Step 4:** `cargo test -p link` と clippy
- [ ] **Step 5:** コミット `Add link crate: reliable channel, client and discovery`

### Task 5: 停止の判断と要求の処理(Python、ROS 非依存)

**Files:** Create `oumuamua_controller_bridge/rosio.py`(インターフェース)、`supervisor.py`、`handlers.py`、`task_memory.py`、`test/fake_rosio.py`、`test/test_supervisor.py`、`test/test_handlers.py`、`test/test_task_memory.py`

**Interfaces:**
- Produces:
  - `rosio.py`: `Done = Callable[[bool, str], None]`、`class RosIo(Protocol)`:
    `publish_cmd_vel(vx, vy, wz)`、`stop_path(done)`、`start_path(name, done)`、`set_tracker(on, done)`、`set_pose(x, y, yaw)`、`run_task(kind, gripper, block, column, row, step: Callable[[str], None], done)`、`cancel_task()`、`task_reset(done)`、`set_suction(gripper: int, on, done)`、`lift_to(gripper: int, height, done)`、`lift_cancel(gripper: int)`、`lift_zero(gripper: int, done)`、`snapshot() -> Snapshot`、`path_names() -> list[str]`
    と `@dataclass Snapshot(pose, vel, wheels: list[tuple[float, bool]], grippers: list[tuple|None], scan: list[tuple[float, float]], path: list[tuple[float, float]], task_text: str | None)`(grippers の要素は protocol.md の Gripper の 6 つ組)
  - `supervisor.py`: `class Supervisor: __init__(io: RosIo, timeout_s=0.3, max_v=2.0, max_w=2.5)`、`mode: Mode`、`stop_reason: str`、`on_control(now_s, c: dict)`(c は CONTROL の欄名のキー)、`tick(now_s, yaw: float | None)`(50 Hz で呼ぶ。停止なら速度 0 を出す、手動なら最後の速度を出す、自動なら出さない)、`release() -> tuple[bool, str]`、`enter_auto(label: str)`、`leave_auto()`、`cancel_to_manual()`
    - 停止に入るとき 1 回だけ: `cancel_task`、`stop_path`、`set_tracker(False)`、両方の `lift_cancel`、両方の `set_suction(False)`
    - フィールド基準は `yaw` で機体座標に回す(yaw が None ならロボット基準のまま)。速さは上限で丸める(並進はベクトルの長さで)
    - jog が変わったら: +1 → `lift_to(g, lift_max)`、-1 → `lift_to(g, 0.0)`、0 → `lift_cancel(g)`。停止中は無視する
    - 接続直後(最初の CONTROL の前)は停止、理由 `"link lost"`
  - `handlers.py`: `class Handlers: __init__(io, sup: Supervisor, shells, send: Callable[[list], None], config: dict)`、`handle(msg: list) -> None`(4.1 の全種を処理し、RESULT `[50, req, ok, message]` を送る。RUN_TASK は受け付けたら自動へ、終わったら TASK_DONE と手動へ。作業中の RUN_TASK と START_PATH は `"busy"` で断る。LIFT_TO で `config["lift_max"]` を超える高さは断る)
  - `task_memory.py`: `parse(text: str) -> tuple[list[bool], list[int], list[tuple[int, int]]]`(`/task_commander/state` の文字列 → holding [gripper0, gripper1]、picked、placed)
- `test/fake_rosio.py`: 呼ばれたものを `calls: list[tuple]` に記録し、done をすぐ(または手で)呼ぶ偽物

- [ ] **Step 1:** テスト: `test_starts_stopped_until_release`、`test_timeout_enters_stop_once`(0.29 秒では入らない、0.31 秒で入る、停止の動作は 1 回だけ)、`test_estop_enters_stop_and_blocks_release`、`test_stop_publishes_zero_each_tick`、`test_manual_clamps_speed`(vx=3, vy=0 → 2.0、wz=-4 → -2.5、vx=vy=2 → 長さ 2.0)、`test_field_frame_rotates_by_yaw`(yaw=π/2 で フィールドの +x → 機体の -y)、`test_auto_publishes_nothing`、`test_jog_starts_and_cancels_lift`、`test_jog_ignored_while_stopped`、`test_run_task_enters_auto_and_done_returns_to_manual`、`test_busy_rejects_second_task`、`test_cancel_returns_to_manual`、`test_lift_to_above_max_rejected`、`test_parse_task_memory`(task_commander_node.cpp の `publish_state` の形の例文 3 行 → 期待値)
- [ ] **Step 2:** 失敗を確かめる
- [ ] **Step 3:** 実装
- [ ] **Step 4:** 通る
- [ ] **Step 5:** コミット `Add supervisor and request handlers (ROS-free)`

### Task 6: シェルと補完(Python)

**Files:** Create `oumuamua_controller_bridge/shells.py`、`completion.py`、`test/test_shells.py`、`test/test_completion.py`

**Interfaces:**
- Produces:
  - `class ShellManager: __init__(log_root: Path, on_output: Callable[[str, bytes], None], on_closed: Callable[[str, str], None], replay_lines=2000, replay_bytes=256*1024)`、`open(tab) -> tuple[bool, str]`、`write(tab, data: bytes) -> tuple[bool, str]`、`close(tab) -> tuple[bool, str]`、`rename(tab, new) -> tuple[bool, str]`、`replay(tab) -> bytes`、`tabs() -> list[str]`、`cwd(tab) -> str`、`poll(timeout_s: float) -> None`(読めた出力を on_output へ。bash が終わったタブは on_closed(tab, "exited") して消す)、`close_all()`
    - `pty.fork()` で `bash -i`。端末の大きさは 80×24(`TIOCSWINSZ`)、`TERM=xterm-256color`
    - 名前は `[A-Za-z0-9_-]{1,32}`。重なり・不正は `(False, 理由)`
    - 閉じるときは SIGHUP、0.5 秒で終わらなければ SIGKILL
    - 記録: protocol.md 5.2 の形。入力は write で受けた行(制御文字は `^C` のように書く)
  - `completion.py`: `complete(line: str, cursor: int, cwd: str, setup_files: list[str]) -> tuple[int, list[str]]`
    - 補助の bash を `bash --norc --noprofile -c <script>` で 2 秒の制限付きで動かす。script は setup_files と bash-completion(`/usr/share/bash-completion/bash_completion` があれば)を読み、`complete -p <最初の語>` で関数があれば COMP_WORDS / COMP_CWORD / COMP_LINE / COMP_POINT を作って呼び、COMPREPLY を 1 行ずつ出す。関数が無ければ、最初の語なら `compgen -c`、それ以外は `compgen -f`(フォルダには `/` を付ける)
    - 返す start は置き換える語の始まり(文字位置)。候補は重複を除いて並べ替える

- [ ] **Step 1:** テスト(本物の bash を使う): `test_open_echo_and_output`(`echo hi\n` → 出力に `hi`)、`test_ctrl_c_interrupts`(`sleep 30\n` の後 `\x03` で 1 秒以内にプロンプトが戻る)、`test_close_kills_bash`(close 後に pid が無い)、`test_exit_reports_closed`(`exit\n` → on_closed)、`test_duplicate_and_bad_names_rejected`、`test_replay_keeps_last_lines`(3000 行出す → replay は最後の 2000 行)、`test_log_file_written`(tmp_path にファイルができ、`> echo hi` と `  hi` の行がある)、`test_rename_moves_logging`、`test_complete_command`(`ech` → start 0、`echo` を含む)、`test_complete_file`(tmp に `alpha.txt` と `alps/` を置き、`cat al` → start 4、`alpha.txt` と `alps/`)、`test_complete_respects_cwd`
- [ ] **Step 2:** 失敗を確かめる
- [ ] **Step 3:** 実装
- [ ] **Step 4:** 通る
- [ ] **Step 5:** コミット `Add PTY shells with logging and bash completion`

### Task 7: UDP サーバ(Python)

**Files:** Create `oumuamua_controller_bridge/server.py`、`test/test_server.py`

**Interfaces:**
- Consumes: protocol、reliable、Supervisor、Handlers、ShellManager
- Produces: `class BridgeServer: __init__(robot_name: str, io: RosIo, config: dict, log_root: Path, port=PORT, beacon_port=BEACON_PORT, beacon_addr="255.255.255.255")`、`start()`(受信・50 Hz の tick・20 Hz の STATE・10 Hz の SCAN・1 Hz の BEACON・シェルの poll を 1 本のスレッドで回す)、`stop()`、`notify(msg: list)`(ROS 側からの LOG や PATH_SHAPE を確実に届ける流れに載せる。スレッドをまたいでよい)
- protocol.md 2 章の HELLO / takeover / busy / BYE、WELCOME 直後の初期送信(CONFIG、FIELD、PATH_SHAPE、TASK_MEMORY、SHELL_OPENED)、STATE の組み立て(Snapshot と Supervisor から)、SCAN の分割、SHELL_OUTPUT の 900 バイト分割、未確認 64 個で待つ間は出力を ShellManager の取っておきに任せて捨てる(つなぎ直しで replay される)
- config の欄(bridge.yaml と同じ): `robot_name`、`lift_presets`(`[["下段", 0.01], ["中段", 0.35], ["上段", 0.69], ["0 点", 0.0]]`)、`lift_max`(0.19)、`speed_levels`(`[[0.3, 1.0], [0.8, 1.8], [1.5, 2.5]]`)、`blocks`(6)、`columns`(3)、`rows`(3)、`field_bounds`、`field_obstacles`(task_commander.yaml の `field.bounds` / `field.obstacles` と同じ並び。FIELD の walls は箱の 4 辺に直す)、`shelf`(`[1.5625, -0.30, 2.5025, 0.30]`)、`start_pose`(`[-2.2, 1.3, 0.0]`)、`robot_radius`(0.32)、`completion_setup_files`

- [ ] **Step 1:** テスト(localhost の本物の UDP と FakeRosIo、Rust の代わりに Python で書いたテスト用の相手): `test_hello_welcome_and_initial_burst`、`test_second_client_busy_then_takeover_sends_bye`、`test_control_timeout_stops`、`test_state_stream_has_ack`、`test_reliable_shell_roundtrip`(SHELL_OPEN → SHELL_OPENED、SHELL_INPUT `echo hi\n` → SHELL_OUTPUT に hi)、`test_beacon_sent`(beacon_addr を 127.0.0.1 にして受ける)、`test_output_flood_does_not_delay_state`(`yes | head -c 5000000` の間も STATE が 20 Hz の 8 割以上届く)
- [ ] **Step 2:** 失敗を確かめる
- [ ] **Step 3:** 実装
- [ ] **Step 4:** 通る
- [ ] **Step 5:** コミット `Add UDP bridge server`

### Task 8: ROS とのつなぎ(rclpy)と偽のロボット

**Files:** Create `oumuamua_controller_bridge/ros_io.py`、`node.py`、`fake_robot.py`、`config/bridge.yaml`、`launch/bridge.launch.py`、`test/test_fake_robot.py`、`ros/oumuamua_controller_bridge/README.md`

**Interfaces:**
- `ros_io.py`: `class RosIoNode(rclpy.node.Node, RosIo)`。つなぐ先は設計書 10 章:
  - pub `/cmd_vel`(TwistStamped、frame_id `base_link`)、`/initialpose`(PoseWithCovarianceStamped、frame `field`、共分散 x, y 0.01、yaw 0.01)
  - sub `/state_estimator/odom`、`/scan`(tf2 で `field` へ写して 2 本に 1 本に間引く。TF が無ければ点を出さない)、`/path_follower/path`(transient local)、`/wheel{0,1,2}/status`(`mini_shirasu_ros/msg/Status`。0.5 秒届かないか mode 4 なら ok=false)、`/gripper{0,1}/state`、`/task_commander/state`(transient local)、`/rosout`(LOG にして `notify`)
  - service: `/path_follower/stop`・`/path_follower/start/<name>`・`/task_commander/reset`・`/gripper{i}/zero_lift`(Trigger)、`/tracker_node/enable`・`/gripper{i}/set_suction`(SetBool)。サービスが 1 秒で見つからなければ done(False, "<名前> がありません")
  - action: `/task_commander/run`(RunTask、feedback の step を渡す)、`/gripper{i}/move_lift`(MoveLift、lift_cancel でゴールを取り消す)
  - `path_names()` は `get_service_names_and_types()` から `/path_follower/start/` で始まるもの
  - 呼び出しはスレッドをまたぐので、MultiThreadedExecutor と ReentrantCallbackGroup を使い、done は executor のスレッドから呼んでよい(BridgeServer 側はロックで守る)
- `node.py`: `main()` — パラメータ(bridge.yaml の欄)を読み、RosIoNode と BridgeServer を作って回す
- `fake_robot.py`: `class FakeRosIo(RosIo)`(cmd_vel を積分する平面の模型、追従は作業の段階を 1 秒ずつ進めて成功、昇降は 0.05 m/s で動く、吸引はすぐ切り替わる、LiDAR はフィールドの壁への光線を 1° ごと)と `main()`(`python3 -m oumuamua_controller_bridge.fake_robot [--port]`。ROS なしで BridgeServer を動かす。設定は bridge.yaml を PyYAML 無しで読めるよう、同じ値を `DEFAULT_CONFIG` として持つ)
- bridge.yaml の値は Task 7 の config と同じ

- [ ] **Step 1:** `test_fake_robot.py`: `test_fake_drives_with_cmd_vel`、`test_fake_scan_hits_walls`(原点から +x の光線が 2.786 m 付近)、`test_fake_task_completes`、`test_fake_lift_moves_and_cancels`
- [ ] **Step 2:** 失敗を確かめる
- [ ] **Step 3:** 実装。`python3 -m py_compile` を ros_io.py と node.py にかける(ROS は入れない)
- [ ] **Step 4:** テストが通り、`python3 -m oumuamua_controller_bridge.fake_robot` が起動して BEACON を出す
- [ ] **Step 5:** README に起動方法(`ros2 launch oumuamua_controller_bridge bridge.launch.py`、ROS なしの fake_robot)と、ROS がある人に確かめてほしいこと(colcon build、sim:=true での動作)を書いてコミット `Add ROS adapter, node, launch and fake robot`

### Task 9: term クレート(キーと行編集と履歴)

**Files:** Create `crates/term/{Cargo.toml,src/lib.rs,src/keys.rs,src/history.rs,src/editor.rs}`

**Interfaces:**
- Produces:
  - `keys.rs`: `pub enum Key { Char(char), Enter, Tab, Backspace, Delete, Left, Right, Up, Down, Home, End, Esc }`、`pub enum KeyEvent { Key(Key), Ctrl(char), Alt(char) }`(Ctrl の文字は小文字)、`pub enum Modifier { Ctrl, Alt, Shift }`、`pub struct Modifiers`: `tap(&mut self, m: Modifier)`(1 回目で次の 1 キーだけ、2 回目で固定、3 回目で解除)、`press(&mut self, k: Key) -> KeyEvent`(Shift は文字を大文字・記号に、1 回だけのものは使ったら外す)、`state(&self, m) -> ModState { Off, Once, Locked }`
  - `history.rs`: `pub struct History`: `new(max: usize)`、`push(&mut self, line: &str)`(空行と直前と同じ行は足さない)、`entries(&self) -> &[String]`、`search(&self, query: &str, before: Option<usize>) -> Option<usize>`(新しい方から)、`to_text()` / `from_text(&str)`(保存用、1 行 1 件)
  - `editor.rs`: `pub enum EditAction { None, Submit(String), Complete { line: String, cursor: usize }, SendNow(Vec<u8>), ClearScreen }`、`pub struct LineEditor`: `handle(&mut self, ev: KeyEvent, tab_history: &mut History, global_history: &mut History) -> EditAction`、`display(&self) -> (String, usize)`(検索中は `(reverse-i-search)'q': 行` の形、カーソルの文字位置)、`apply_completion(&mut self, start: usize, candidates: &[String]) -> Vec<String>`(1 つならそれで置き換えて、フォルダでなければ空白を足す。複数なら共通の頭まで入れて候補を返す)
    - Enter: Submit(行) と両方の履歴へ。Tab: Complete。↑↓: タブの履歴。Ctrl+R: 全体の履歴を検索(続けて Ctrl+R で前へ、Enter で確定して Submit、Esc で抜ける)。Ctrl+A/E/W/U/L は手元で処理。Ctrl+C/D/Z/\ と他の Ctrl は `SendNow(制御文字 1 バイト)`(Ctrl+C は手元の行も消す)。Alt+x は `SendNow([0x1b, x])`

- [ ] **Step 1:** テスト: `modifiers_once_lock_off`、`shift_maps_symbols`、`insert_move_delete_utf8`(日本語を含む行でカーソル移動と削除)、`ctrl_a_e_w_u`、`enter_submits_and_records`、`up_down_walk_tab_history`、`ctrl_r_searches_global`、`ctrl_c_sends_etx_and_clears`、`other_ctrl_sends_control_byte`(Ctrl+Z → 0x1a)、`tab_requests_completion`、`completion_single_and_common_prefix`、`history_skips_blank_and_repeat`、`history_text_roundtrip`
- [ ] **Step 2〜4:** 失敗 → 実装 → `cargo test -p term` と clippy
- [ ] **Step 5:** コミット `Add term crate: keys, line editor and history`

### Task 10: term クレート(出力の解釈と折りたたみ)

**Files:** Create `crates/term/src/ansi.rs`、`crates/term/src/scrollback.rs`

**Interfaces:**
- Produces:
  - `pub enum Color { Indexed(u8), Rgb(u8, u8, u8) }`、`pub struct Span { text: String, fg: Option<Color>, bg: Option<Color>, bold: bool }`、`pub struct Line { spans: Vec<Span> }`(`Line::text()` で文字だけ)
  - `pub struct Scrollback`: `new(max_lines: usize)`、`feed(&mut self, bytes: &[u8])`、`replace(&mut self, bytes: &[u8])`(SHELL_OPENED)、`clear()`、`mark_command(&mut self)`(Enter で送ったときに呼ぶ。ここから次の mark までを 1 つのかたまりとする)、`rows(&self) -> Vec<Row>`、`toggle_fold(&mut self, block: usize)`
  - `pub enum Row { Line(Line), Folded { block: usize, hidden: usize } }`。かたまりが 30 行を超えたら、最後の 10 行だけ見せ、その上に Folded を置く(開くと全部)。最後のかたまり(実行中)も同じ
  - 解釈するもの: `\n`、`\r`(行頭へ戻って上書き)、`\x08`(1 文字戻る)、`\t`(8 桁区切りまで空白)、`ESC[...m`(0, 1, 22, 30–37, 39, 40–47, 49, 90–97, 100–107, 38;5;n, 48;5;n, 38;2;r;g;b, 48;2;r;g;b)、`ESC[K` / `ESC[0K` / `ESC[2K`。それ以外の ESC の並び(CSI・OSC・2 文字)は捨てる。BEL は捨てる
  - UTF-8 はかたまりの境目をまたいでも正しく読む。壊れたバイトは U+FFFD

- [ ] **Step 1:** テスト: `plain_lines`、`cr_overwrites`(`abc\rX` → `Xbc`)、`backspace`、`sgr_16_256_truecolor`、`reset_and_bold`、`erase_line`、`unknown_sequences_dropped`(`ESC]0;title BEL`、`ESC[?25l`)、`utf8_split_across_feeds`(`あ` の 3 バイトを 1 バイトずつ)、`invalid_utf8_replaced`、`max_lines_trims_oldest`、`fold_long_block`(40 行 → Folded{hidden: 30} + 10 行、toggle で 40 行)、`replace_resets`
- [ ] **Step 2〜4:** 失敗 → 実装 → テストと clippy
- [ ] **Step 5:** コミット `Add ANSI parsing and scrollback with folding`

### Task 11: アプリの骨組み(接続・右端の帯・面の切り替え・全停止)

**Files:** Create `crates/app/{Cargo.toml,build.rs,src/lib.rs,src/main.rs,src/font.rs,src/storage.rs,src/model.rs,src/glue.rs,ui/main.slint,ui/theme.slint,ui/strip.slint,ui/connect.slint,fonts/*}`

**Interfaces:**
- Consumes: `link::{Client, Discovery, LinkEvent, ControlInput}`、`proto`
- Produces:
  - `model.rs`(Slint に依存しない、テストできる状態): `pub struct AppModel`: `apply(&mut self, ev: LinkEvent)`、`connection(&self) -> ConnState { Searching, Connecting, Connected { rtt_ms }, Lost, Rejected(String) }`、`state(&self) -> Option<&State>`、`config()`、`field()`、`path_shape()`、`task_memory()`、`scan()`、`results: VecDeque<(u32, bool, String)>`(最後の 20 件)、`take_shell_events(&mut self) -> Vec<ShellEvent>`(Task 16 が使う。SHELL_* と LOG をため込む)
  - `storage.rs`: `pub fn data_dir() -> PathBuf`(Android は petaricon の storage と同じく app の内部ファイル置き場、PC は `~/.config/oumuamua-controller`)、`load(name) -> Option<String>` / `save(name, &str)`
  - `glue.rs`: Slint のコールバックと Client をつなぎ、33 ms の Timer で `Receiver<LinkEvent>` を読んで AppModel に入れ、画面の性質を更新する
  - UI: `MainWindow` に `page: int`(0 出力・1 手動・2 自動)、右端の帯(`ui/strip.slint`: 3 つの切り替え、接続状態と遅延 ms、全停止ボタン。停止中は「解除」に変わる。帯は safe-area の内側)、接続画面(`ui/connect.slint`: 名乗りの一覧、IP の入力欄(OS のキーボードで可)、「接続」「乗っ取って接続」、最後の宛先を保存)。各面は Task 12〜16 で中身を入れる空の部品にしておく
  - 全停止ボタン: 押すと `ControlInput.estop = true`(スマホ側の固定)。「解除」は estop を false にして RELEASE_STOP を送る。結果が失敗なら理由を下に出す
  - Android: `Cargo.toml` の `package.metadata.android` は petaricon と同じ形で、package `dev.oumuamua.controller`、label `oumuamua`、orientation `landscape`、`uses_permission` に INTERNET・ACCESS_WIFI_STATE・CHANGE_WIFI_MULTICAST_STATE

- [ ] **Step 1:** `model.rs` のテスト: `connected_then_state_updates_rtt`、`lost_and_recovered`、`rejected_shows_reason`、`results_keep_last_20`、`config_field_path_task_memory_stored`
- [ ] **Step 2〜4:** 失敗 → 実装 → `cargo test -p app`、`cargo build -p app`(PC)、fake_robot に PC 版でつながり、帯の遅延表示と全停止・解除で mode が変わることを見る(スクリーンショットを撮る)
- [ ] **Step 5:** コミット `Add app skeleton: connection, edge strip, stop`

### Task 12: 地図の描画と出力の面の上の帯

**Files:** Create `crates/app/src/map.rs`、`crates/app/ui/map.slint`、`crates/app/ui/output.slint`(地図と数値の帯。コンソールの場所は空けておく)

**Interfaces:**
- Produces: `pub struct MapView { center: (f32, f32), scale: f32 /* px/m */ }`、`fit(field_bounds, width, height) -> MapView`、`to_px` / `to_field`、`pan(dx_px, dy_px)`、`zoom(factor, about_px)`、`pub fn render(view: &MapView, w: u32, h: u32, scene: &Scene) -> slint::SharedPixelBuffer<Rgba8Pixel>`(tiny-skia。`Scene { walls, shelf, robot_pose, robot_radius, scan, path, reference: Option<[f64;3]>, pose_drag: Option<([f64;2], f64)> }`)
- 描く順: 背景 → 壁(線) → 棚(枠) → 経路(線) → LiDAR(点 2px) → ロボット(円と向きの線) → 指定中の姿勢(半透明の矢印)。+y が画面の上
- `map.slint`: `MapPane` に画像、ドラッグで移動、「+」「−」「全体」ボタン。Slint 1.18 にピンチの部品があれば使う(無ければボタンだけ)。長押しドラッグは `pose-pick-enabled` のときだけ有効で、`pose-picked(x, y, yaw)` を出す(Task 15 が使う)
- 数値の帯: 位置 (x, y, yaw°)、速度、各車輪の電圧(ok=false は赤)、モードと停止の理由

- [ ] **Step 1:** テスト: `fit_contains_field`、`to_px_and_back`、`zoom_keeps_anchor`、`render_draws_robot_pixel`(ロボットの中心の画素が背景色でない)
- [ ] **Step 2〜4:** 失敗 → 実装 → テスト、PC 版で fake_robot の地図が動くスクリーンショット
- [ ] **Step 5:** コミット `Add map rendering and output status band`

### Task 13: 手動制御の面

**Files:** Create `crates/app/ui/manual.slint`、`crates/app/src/manual.rs`

**Interfaces:**
- Produces:
  - `manual.rs`: `pub fn stick_to_velocity(dx: f32, dy: f32, level: (f64, f64)) -> (f64, f64)`(dx, dy は −1..1、画面の上 = +x 前、左 = +y。不感帯 0.1、長さ 1 で丸める)、`pub fn slider_to_omega(s: f32, level) -> f64`(右 = 時計回り = 負)
  - UI(設計書 6.2): 左にスティック、右に旋回のスライダー(どちらも離すと 0)、中央に速度の段階 3 つ、ロボット基準/フィールド基準、左右の昇降(「上」「下」は押している間 jog、プリセットのボタンは CONFIG の `lift_presets`、`lift_max` を超えるものは押せず「配線の都合で 0.19 m まで」のように理由を出す、「両方」の切り替え、今の高さと目標)、左右の吸引(入り切り、色)、リセット(1 秒の長押しで LIFT_ZERO、動いている間は押せない)
  - 自動(STATE の mode = 1)の間は、操作部の上にポップアップ「自動走行中」と、作業名/経路名と段階、「手動制御へ」(CANCEL を送る)を重ねる
  - スティックと旋回はマルチタッチで同時に動かせること。Slint の TouchArea で同時に動かないなら、2 本の指を 1 つの領域で扱う(Review Focus 3)

- [ ] **Step 1:** テスト: `stick_deadzone_and_clamp`、`stick_axes`(上 → +vx、左 → +vy)、`slider_sign`
- [ ] **Step 2〜4:** 失敗 → 実装 → テスト、PC 版で fake_robot を動かし、ロボットが地図上で動く・昇降と吸引の表示が変わる・自動中のポップアップが出ることをスクリーンショットで確かめる
- [ ] **Step 5:** コミット `Add manual control page`

### Task 14: 自動制御の面

**Files:** Create `crates/app/ui/auto.slint`、`crates/app/src/auto.rs`

**Interfaces:**
- Produces:
  - `auto.rs`: `pub enum TaskKind { Pick = 0, ToSlalomEntrance = 1, ToStart = 2, SlalomForward = 3, SlalomBackward = 4, Place = 5, ToSlalomExit = 6 }`、`pub fn task_label(kind: u8, args: [i32; 4]) -> String`(例「回収 左 ブロック 2」「配置 右 列 1 段 0」「スタートへ」)、`pub fn place_warning(memory: &TaskMemory, column: u8, row: u8) -> Option<String>`(row ≥ 1 で同じ列の row−1 に自分で置いていなければ「下の段にまだ置いていません」)、`pub fn can_pick(memory, block) -> bool`、`pub fn holding_gripper(memory) -> Vec<u8>`
  - UI(設計書 6.3): 回収(6 個の並び + 左/右 + 「回収」)、配置(3×3 + 左/右 + 「配置」)、移動 5 つ、作業中は名前と段階と「取り消し」、覚えていることの表示と「忘れさせる」(長押し)、下に小さく経路の一覧・経路の停止・追従の入り切り、地図(MapPane)を置き、長押しドラッグで姿勢を決めて確認の「決定」で SET_POSE
  - 作業中は作業のボタンを押せなくする。TASK_DONE の結果を下に出す

- [ ] **Step 1:** テスト: `labels`、`place_warning_rules`、`can_pick_excludes_picked`、`holding_grippers`
- [ ] **Step 2〜4:** 失敗 → 実装 → テスト、PC 版で fake_robot に回収・配置・移動を送り、段階の表示と結果が出るスクリーンショット
- [ ] **Step 5:** コミット `Add automatic control page`

### Task 15: コンソールと専用キーボード

**Files:** Create `crates/app/ui/console.slint`、`crates/app/ui/keyboard.slint`、`crates/app/src/console.rs`

**Interfaces:**
- Consumes: `term::{Modifiers, LineEditor, History, Scrollback, Row}`、AppModel の `take_shell_events`
- Produces:
  - `console.rs`(Slint に依存しない): `pub struct Console`: `tabs()`、`active`、`new_tab(name) -> Option<Msg>`(SHELL_OPEN)、`close_tab`、`rename_tab`、`key(&mut self, cap: &str) -> Vec<Msg>`(キーボードの 1 キー → Modifiers → LineEditor → 必要なら SHELL_INPUT / SHELL_COMPLETE)、`on_event(ShellEvent)`、`candidates()`(補完の候補)、`pick_candidate(i)`、`quick_commands()` / `set_quick_commands`、`log_filter`、`save()` / `load()`(履歴と 1 タップのコマンドを storage へ)
    - キーの名前(`cap`): 1 文字ならその文字、それ以外は `"Enter" "Tab" "Backspace" "Delete" "Left" "Right" "Up" "Down" "Home" "End" "Esc" "Ctrl" "Alt" "Shift" "Sym"`
    - 先頭のタブ `rosout` は LOG を受ける閉じられないタブ。重要度の絞り込み(既定 INFO)は LOG_LEVEL を送る
  - UI: タブの列(スワイプで切り替え、「+」、長押しで名前変更・閉じる)、出力(ListView、Span ごとに色、Folded の行はタップで開く)、入力欄(カーソル付きの表示だけで、OS のキーボードは出さない)、補完の候補の列、1 タップのコマンドの列、専用キーボード(設計書 8 章の配列、高さは画面の 4 割、Backspace と矢印は長押しで 50 ms ごとに繰り返す)
  - 出力の面の右側に置く。キーボードが出ている間は地図を縮めずにコンソールの出力欄を縮める

- [ ] **Step 1:** テスト: `typing_and_enter_sends_line_with_newline`、`ctrl_c_sends_etx`、`tab_sends_complete_and_applies_result`、`rosout_tab_receives_logs_and_cannot_close`、`opened_replaces_content`、`closed_event_removes_tab`、`history_persists`(save → load)
- [ ] **Step 2〜4:** 失敗 → 実装 → テスト、PC 版で fake_robot のシェルに `ls`、`ech`+Tab、`sleep 10` + Ctrl+C を打つスクリーンショット
- [ ] **Step 5:** コミット `Add console with tabs and custom keyboard`

### Task 16: Android の APK と使い方

**Files:** Modify `crates/app/Cargo.toml`(必要なら)、Create `README.md`

- [ ] **Step 1:** `cargo apk build -p app --release --lib` で APK を作る(NDK・SDK は petaricon と同じ `/opt/android-sdk`)。`llvm-nm -D` で `android_main` があることを確かめる
- [ ] **Step 2:** README に、構成、ロボット側の起動(oumuamua に入れて `colcon build` → `ros2 launch oumuamua_controller_bridge bridge.launch.py`)、ROS なしで試す方法(`fake_robot` と PC 版)、APK の入れ方、ROS がある人に確かめてほしいこと(Task 8 の一覧)、前提にしている `~/zero_lift` を書く
- [ ] **Step 3:** コミット `Add README and Android packaging`
