# Slint マルチタッチ対応 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Slint 1.18.1 の中核を手元で直し、別々の部品なら複数の指を同時に受けられるようにする。

**Architecture:** i-slint-core を `third_party/` に置いて `[patch.crates-io]` で差し替え、`TouchState` と窓の入力の配り方だけを指ごとにする。テストは testing backend に内部の Touch イベントを送る専用クレートで行う。

**Tech Stack:** Rust 2024、Slint =1.18.1、i-slint-backend-testing

**Spec:** `docs/superpowers/specs/2026-10-23-slint-multitouch-design.md`

## Global Constraints

- 設計書 3 章の 4 つの扱いを守る。指 1 本のときの動きは変えない。
- 変更は `third_party/i-slint-core` の `input.rs` と `window.rs`(と必要最小限の周辺)だけ。変えた箇所には `// MULTITOUCH:` の注を付け、`third_party/i-slint-core/MULTITOUCH.md` に一覧を書く。
- 版は `=1.18.1` のまま。
- コミットの形は SDD の common.md のとおり。ROS は入れない。

## Review Focus

1. 指が離れる・取り消される(Cancelled)・画面外へ出るときに、指ごとの状態が必ず片付く(押されたままの部品が残らない)。
2. ポップアップ(PopupWindow)やメニューが開いた状態で 2 本目が触れたとき、落ちたり閉じ方が壊れたりしない。
3. 地図のピンチ(ScaleRotateGestureHandler)が今と同じように動く。
4. 主の指が離れて別の指が残ったとき、ホバーやカーソルの状態が壊れない。

---

### Task 1: i-slint-core を手元に置き、差し替えの土台とテスト用クレートを作る

**Files:**
- Create: `third_party/i-slint-core/`(`~/.cargo/registry/src/*/i-slint-core-1.18.1` の中身。`Cargo.toml.orig`、`.cargo_vcs_info.json`、`benches/` など不要なものは除いてよいが LICENSES は残す)、`third_party/i-slint-core/MULTITOUCH.md`、`crates/multitouch-tests/{Cargo.toml,src/lib.rs,tests/multitouch.rs}`
- Modify: ルート `Cargo.toml`(`[patch.crates-io] i-slint-core = { path = "third_party/i-slint-core" }`、members に `crates/multitouch-tests`。`third_party` はワークスペースから外す)

**Interfaces:**
- Produces: テストの補助 `fn touch(window: &slint::Window, id: i32, x: f32, y: f32, phase: TouchPhase)`(`WindowEvent::internal(InternalEvent::Touch {..})` を送る)

- [ ] **Step 1:** 差し替えただけ(ソースは無変更)で `cargo build --workspace` と `cargo test --workspace` が通ることを確かめる
- [ ] **Step 2:** `tests/multitouch.rs` に設計書 5 章の 6 つのテストを書く(画面は `slint::slint!` で、左右に TouchArea 2 つ、Flickable と中のボタン、ScaleRotateGestureHandler を置く)。指 1 本のテストと、今の動きのままでも通るテスト(ピンチ)は通り、複数指のテストは失敗することを確かめる
- [ ] **Step 3:** コミット `Vendor i-slint-core 1.18.1 and add multitouch tests`

### Task 2: 中核の入力を指ごとにする

**Files:** Modify `third_party/i-slint-core/input.rs`、`third_party/i-slint-core/window.rs`、`third_party/i-slint-core/MULTITOUCH.md`

- [ ] **Step 1:** 設計書 4 章のとおりに変える(主でない指ごとの `MouseInputState`、押下前の通り道の重なり判定、ピンチは同じ ScaleRotateGestureHandler のときだけ、引き継ぎなし)
- [ ] **Step 2:** Task 1 のテストがすべて通り、i-slint-core の既存テスト(`cargo test -p i-slint-core` が動けば)も通ることを確かめる。Review Focus の 4 項目のテストも足す
- [ ] **Step 3:** MULTITOUCH.md に変更点と理由を書いてコミット `Dispatch touches per finger in i-slint-core`

### Task 3: アプリで使い、APK を作る

**Files:** Modify `crates/app`(必要なら)、`README.md`(既知の制限から 2 本指の項目を外す)

- [ ] **Step 1:** 手動制御の面のスティックとスライダーが別々の TouchArea であることを確かめ、2 本目の指で 1 本目が離される前提の処理(あれば)を見直す
- [ ] **Step 2:** `cargo test --workspace`、clippy、APK の作り直し(`dist/oumuamua-controller.apk`)
- [ ] **Step 3:** コミット `Use per-finger touches on the manual page`
