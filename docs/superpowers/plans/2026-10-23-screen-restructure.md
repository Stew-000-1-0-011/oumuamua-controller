# コントローラの画面の組み直し 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 画面を Window 直下の名前付き部品にして `visible` で切り替え、手動・自動を出力画面に半透明で重ね、800×360 で見切れないようにする。

**Architecture:** `main.slint` は部品の利用とプロパティのつなぎだけにし、各画面の見た目と振る舞いは各ファイルの部品の中に置く。Rust 側(glue.rs)は、プロパティ名やコールバック名の変更に合わせるだけにとどめ、ロジックは変えない。

**Tech Stack:** Slint =1.18.1(third_party で差し替え済み)、Rust

**Spec:** `docs/superpowers/specs/2026-10-23-screen-restructure-design.md`

## Global Constraints

- 設計書 2〜4 章のとおり。基準の大きさは 800×360、ほかに 914×411 と 1280×720。
- 画面の切り替えに `if` を使わない。
- タップの的は 44px 以上(専用キーボードのキーだけ最小 32px)。
- 安全の決まり(全停止、通信断、重ね表示中は出力画面に触れない、手動の入力はページを離れると 0)を変えない。既存のテストはすべて通す。
- コミットの形は SDD の common.md のとおり。

## Review Focus

1. 重ね表示が出ている間に、後ろの地図やコンソールへタッチが抜けない。
2. 手動の重ね表示から別の面へ切り替えたとき、スティック・スライダー・昇降の入力が必ず 0 になる(今は `if` で部品が消えていたので自然に離されていた。`visible: false` では部品が残るので、押したまま隠れる場合を明示的に扱う)。
3. 800×360 で、全停止ボタンと「解除」が常に見えて押せる。
4. 専用キーボードを出したまま面を切り替えても、入力欄やキーボードの状態が崩れない。

---

### Task 1: 骨組み(Window 直下の部品と visible の切り替え、右端の帯、接続画面、確認、通知、フォントの移動)

**Files:** Modify `crates/app/ui/{main,strip,connect,output,manual,auto}.slint`、Create `crates/app/ui/{leave,notice}.slint`、Move `crates/app/fonts/DejaVu*` → `crates/app/ui/fonts/`(font.rs が参照する Noto は今の場所のままでよい)、Modify `crates/app/src/glue.rs`(名前の変更に追従)

- [ ] **Step 1:** `MainWindow` の直下を設計書 2 章の表のとおりにする。中身はまだ今の各ページをそのまま包むだけでよい(手動・自動は一時的に不透明のまま)。Review Focus 2 のとおり、手動の面が隠れたら押している入力を離す処理を、部品側(`changed visible` など)と Rust 側(ページの値で判定済みの `manual_allowed`)の両方で確かめる
- [ ] **Step 2:** 右端の帯と接続画面と確認と通知を 800×360 に収める
- [ ] **Step 3:** testing backend のテスト: 手動の面でスティックを押したまま `page` を 0 にすると速度が 0 になる
- [ ] **Step 4:** 3 つの大きさで、接続画面と右端の帯のスクリーンショット(/tmp/claude-0/shots/rs1-*.png)
- [ ] **Step 5:** コミット `Put each screen under the window and switch by visibility`

### Task 2: 手動・自動の重ね表示と姿勢の指定

**Files:** Modify `crates/app/ui/{manual,auto,output,map,main}.slint`、`crates/app/src/glue.rs`

- [ ] **Step 1:** `ManualPanel` を設計書 3・4 章のとおり半透明の重ね表示にする(全面のタッチ吸収を含む)
- [ ] **Step 2:** `AutoPanel` も同様にし、自動の専用の地図をやめて「地図で姿勢を指定」の流れ(出力画面へ切り替え、案内、長押しドラッグ、確認、SET_POSE、やめる)にする
- [ ] **Step 3:** testing backend のテスト: 重ね表示中に地図の上をタッチしても地図が動かない/姿勢の指定の流れで SET_POSE が 1 回送られる
- [ ] **Step 4:** 3 つの大きさで、手動・自動・姿勢の指定のスクリーンショット(/tmp/claude-0/shots/rs2-*.png)
- [ ] **Step 5:** コミット `Overlay manual and auto controls on the output screen`

### Task 3: 出力画面とコンソールとキーボードの大きさ、APK

**Files:** Modify `crates/app/ui/{output,console,keyboard}.slint`、必要なら `crates/app/src/glue.rs`、`README.md`

- [ ] **Step 1:** 上部の数値の帯を 1 行にし、地図とコンソールを 800×360 に収める
- [ ] **Step 2:** キーボードを出している間はコンソールを出力画面の全体に広げ、キーボードは高さの 55% 前後、キーは最小 32px
- [ ] **Step 3:** 3 つの大きさで、出力・コンソール(キーボードあり・なし)のスクリーンショット(/tmp/claude-0/shots/rs3-*.png)
- [ ] **Step 4:** `cargo test --workspace`、clippy、APK を作り直して `dist/` へ、README の画面の説明を更新
- [ ] **Step 5:** コミット `Fit the output screen and console to small phones`
