# スマホ ⇔ 橋渡しノードの通信の取り決め

設計書 `docs/superpowers/specs/2026-10-22-oumuamua-controller-design.md` の 3・4 章を、バイトの並びまで決めたもの。
Rust (`crates/proto`) と Python (`ros/oumuamua_controller_bridge/oumuamua_controller_bridge/protocol.py`) はこれに従い、
`testdata/golden/` の見本で一致を確かめる。

## 1. 符号化の決まり

- UDP の 1 データグラム = MessagePack の値 1 つ。値は必ず配列で、先頭が種類の番号。
- 数の書き方は MessagePack の最短形(正の整数は uint、負は int)。実数はすべて float64(0xcb)。文字列は str、バイト列は bin。無いことは nil。
- 単位: 長さ m、角度 rad、速度 m/s・rad/s、時刻 ms(u32、桁あふれで 0 に戻る)。座標はフィールド座標系(原点はフィールド中心、+x がスタート→ビンゴ、+y が左)。
- 1 データグラムは 1200 バイト以下。
- ポート: 橋渡しノードは UDP 47800 で受ける。名乗りは UDP 47801 への同報(255.255.255.255)。
- 知らない種類の番号は捨てる。配列の要素が多すぎるときは、知っている分だけ読む(後から要素を末尾に足せるように)。

## 2. データグラム

| 番号 | 名前 | 向き | 中身 |
| --- | --- | --- | --- |
| 0 | HELLO | スマホ→ | `[0, session: u32, client_name: str, takeover: bool, auth: nil]` |
| 1 | WELCOME | →スマホ | `[1, session: u32, robot_name: str, accepted: bool, reason: str]` |
| 2 | CONTROL | スマホ→ | `[2, session, seq: u32, t_ms: u32, vx: f64, vy: f64, wz: f64, field_frame: bool, jog_left: int, jog_right: int, estop: bool]` |
| 3 | STATE | →スマホ | `[3, session, seq: u32, ack_seq: u32, ack_t_ms: u32, body: State]` |
| 4 | SCAN | →スマホ | `[4, session, scan_id: u32, frag: u8, frag_count: u8, points: bin]` |
| 5 | REL | 両方 | `[5, session, seq: u32, msg: Msg]`(確実に届ける流れの 1 区切り) |
| 6 | ACK | 両方 | `[6, session, cum: u32]`(cum 以下の番号はすべて受け取った。まだ何も受けていなければ 0) |
| 7 | BEACON | →同報 | `[7, robot_name: str, port: u16]` |
| 8 | BYE | 両方 | `[8, session, reason: str]`(乗っ取られた、アプリを閉じた) |

- session はスマホが HELLO のたびに乱数で決める(0 以外)。橋渡しノードは今の操作者の session と違う CONTROL / REL / ACK を捨てる(HELLO を除く)。
- HELLO は WELCOME が来るまで 0.5 秒ごとに送り直す。別のスマホが操作中なら `accepted: false, reason: "busy"`。`takeover: true` なら受け入れ、前のスマホに BYE `"taken over"` を送る。
- 同じ session の HELLO がまた来たら(WELCOME が落ちた)、WELCOME を送り直すだけにする。
- CONTROL は 50 回/秒。seq は 1 から増える。橋渡しノードは今までで最大の seq 以下を捨てる。
  - jog_left / jog_right: 昇降の「上」「下」を押している間 +1 / -1、離すと 0。
  - estop: スマホの全停止ボタンが押されて、まだ解除されていない間 true。
- STATE は 20 回/秒。ack_seq / ack_t_ms は最後に受けた CONTROL の seq と t_ms(往復の遅延を測るため。無ければ 0)。
- SCAN は 10 回/秒。points は、フィールド座標系の点を (x, y) の順に i16 リトルエンディアン [mm] で並べたもの。1 データグラムに 280 点まで。scan_id の frag 0..frag_count-1 がそろったら 1 回分。そろう前に次の scan_id が来たら捨てる。

### 2.1 State

`[mode: u8, stop_reason: str, pose: [x, y, yaw] | nil, vel: [vx, vy, wz] | nil, wheels: [[vdc: f64, ok: bool] ...], grippers: [Gripper | nil, Gripper | nil], task: Task | nil, path: [name: str, active: bool] | nil, tracker_enabled: bool]`

- mode: 0 = 手動、1 = 自動、2 = 停止。stop_reason は停止の理由(`"estop"`、`"link lost"`)、停止でなければ ""。
- pose は自己位置(フィールド座標系)、vel は機体座標系の速度。推定が無ければ nil。
- wheels は `/wheel<i>/status` の電圧と、状態が届いているか。
- grippers は [左, 右] = [gripper0, gripper1]。届いていなければ nil。
  - Gripper: `[height: f64, goal: f64, moving: bool, arrived: bool, suction: bool, feedback: bool]`
- task は作業中なら `[kind: u8, step: str, args: [gripper: i32, block: i32, column: i32, row: i32]]`(kind は RunTask の type)。作業中でなければ nil。
- path は path_follower で走っている経路(~/start で始めたもの)。無ければ nil。

## 3. 確実に届ける流れ

- 向きごとに独立。送る側は seq を 1 から振り、REL を送る。受ける側は順番どおりに渡し、ACK で cum を返す。
- 送る側: 未確認の REL を 100 ms ごとに送り直す。未確認は 64 個まで(超えたら送るのを待つ)。
- 受ける側: 次に欲しい番号より先のものは 64 個先まで取っておく。それより先と、もう渡した番号は捨てる。REL を受けるたびに ACK を返す(重複でも)。
- session が変わったら、両側とも番号を 1 からやり直す。前の session の未送信分は捨てる(シェルの出力は 5.2 の再送で補う)。

## 4. Msg(REL の中身)

`[type: u8, ...]`。スマホからの要求は 2 番目に req: u32(1 から増える)を持ち、橋渡しノードは RESULT で答える。

### 4.1 スマホ → 橋渡しノード

| 番号 | 名前 | 中身 | すること |
| --- | --- | --- | --- |
| 10 | RELEASE_STOP | `[10, req]` | 全停止を解除して手動へ。CONTROL の estop が true の間は失敗 |
| 11 | SET_SUCTION | `[11, req, gripper: u8, on: bool]` | gripper: 0 = 左、1 = 右、2 = 両方 |
| 12 | LIFT_TO | `[12, req, gripper: u8, height: f64]` | `~/move_lift`。gripper は 11 と同じ |
| 13 | LIFT_ZERO | `[13, req, gripper: u8]` | `~/zero_lift` |
| 14 | RUN_TASK | `[14, req, kind: u8, gripper: i32, block: i32, column: i32, row: i32]` | task_commander `~/run`。自動へ |
| 15 | CANCEL | `[15, req]` | 作業を取り消し、経路を止め、追従を切って手動へ(ポップアップの「手動制御へ」も同じ) |
| 16 | START_PATH | `[16, req, name: str]` | 追従を入れて `/path_follower/start/<name>`。自動へ |
| 17 | SET_TRACKER | `[17, req, on: bool]` | `/tracker_node/enable` |
| 18 | SET_POSE | `[18, req, x: f64, y: f64, yaw: f64]` | `/initialpose` |
| 19 | TASK_RESET | `[19, req]` | `/task_commander/reset` |
| 20 | LOG_LEVEL | `[20, req, min_level: u8]` | ROS ログの絞り込み(10 DEBUG、20 INFO、30 WARN、40 ERROR、50 FATAL) |
| 30 | SHELL_OPEN | `[30, req, tab: str]` | タブを開く(同じ名前があれば失敗) |
| 31 | SHELL_INPUT | `[31, req, tab: str, data: bin]` | 疑似端末へ書く(行 + `\n`、または制御文字 1 バイト) |
| 32 | SHELL_COMPLETE | `[32, req, tab: str, line: str, cursor: u32]` | 補完の候補を求める(cursor は line の中の文字位置) |
| 33 | SHELL_CLOSE | `[33, req, tab: str]` | bash を終わらせる |
| 34 | SHELL_RENAME | `[34, req, tab: str, new_name: str]` | 名前を変える(記録のフォルダも新しい名前で続ける) |

### 4.2 橋渡しノード → スマホ

| 番号 | 名前 | 中身 |
| --- | --- | --- |
| 50 | RESULT | `[50, req: u32, ok: bool, message: str]` |
| 51 | CONFIG | `[51, robot_name: str, paths: [str], lift_presets: [[name: str, height: f64]], lift_max: f64, speed_levels: [[v: f64, w: f64] x3], blocks: u8, columns: u8, rows: u8]` |
| 52 | FIELD | `[52, bounds: [xmin, ymin, xmax, ymax], obstacles: [[xmin, ymin, xmax, ymax]], shelf: [xmin, ymin, xmax, ymax], start: [x, y, yaw], robot_radius: f64]`(外周と障害物は箱。1200 バイトに収まるよう、辺ではなく箱で送る) |
| 53 | PATH_SHAPE | `[53, name: str, points: bin]`(SCAN と同じ点の並び。経路が変わるたび。消えたら空。280 点を超える経路は、始点と終点を残して全体から等間隔に間引く) |
| 54 | TASK_MEMORY | `[54, holding: [bool, bool], picked: [u8], placed: [[column: u8, row: u8]]]` |
| 55 | TASK_DONE | `[55, kind: u8, ok: bool, message: str]` |
| 56 | LOG | `[56, stamp_ms: u64, level: u8, name: str, text: str]`(エポックからの ms) |
| 60 | SHELL_OPENED | `[60, tab: str, replay: bin]`(タブの直近の出力。スマホはタブの中身をこれで置き換える) |
| 61 | SHELL_OUTPUT | `[61, tab: str, data: bin]` |
| 62 | SHELL_COMPLETION | `[62, req: u32, tab: str, start: u32, candidates: [str]]`(line の start 文字目から後ろを候補で置き換える) |
| 63 | SHELL_CLOSED | `[63, tab: str, reason: str]`(閉じた、bash が終わった) |

- WELCOME で受け入れた直後に、CONFIG、FIELD、PATH_SHAPE、TASK_MEMORY、開いているタブごとの SHELL_OPENED を送る。
- SHELL_OUTPUT は 1 つ 900 バイトまで(超える分は分ける)。

## 5. 決まった振る舞い

### 5.1 停止

- 停止に入る条件: CONTROL の estop が true、または今の操作者の CONTROL が 0.3 秒届かない。接続した直後(最初の CONTROL の前)も停止とする。
- 停止に入るとき 1 回: 作業の取り消し、`/path_follower/stop`、追従を切る、昇降の動きを取り消す、左右の吸引を切る。
- 停止の間: 速度 0 の `/cmd_vel` を 50 回/秒。
- RELEASE_STOP で手動へ(estop が false で、CONTROL が届いているときだけ)。

### 5.2 シェル

- 疑似端末は 80 列 × 24 行、`TERM=xterm-256color`。
- タブごとに直近の出力を 2000 行(多くても 256 KiB)取っておき、SHELL_OPENED の replay にする。
- 記録: `~/.oumuamua_controller/console/<タブ名>/<開いた日時 YYYYmmdd-HHMMSS>.log`。行の頭に `HH:MM:SS.mmm` と、入力なら `> `、出力なら `  ` を付ける。
- タブ名: `[A-Za-z0-9_-]{1,32}`。
