# MULTITOUCH changes to i-slint-core 1.18.1

This directory is a copy of the crates.io `i-slint-core` 1.18.1 (benches/ and
packaging metadata removed; `[[bench]]` dropped from Cargo.toml). It replaces
the registry crate via `[patch.crates-io]` in the root Cargo.toml.

Design: docs/superpowers/specs/2026-10-23-slint-multitouch-design.md

Every source change is marked `// MULTITOUCH:` and listed below.

## Changes

Re-apply on a newer Slint: copy the new i-slint-core over this directory, then
`grep -rn MULTITOUCH` the old tree and redo each item below. Everything is additive
except the two small edits marked "edit" (the delayed-press timer and the body of
`process_touch_input`). `TouchState::process` itself is unchanged.

Behaviour (spec section 3): an item belongs to the first finger that pressed it; items on a
finger's path, including intercepting ancestors such as Flickable, are in use and other fingers
that land on them are ignored until they lift; two fingers inside the same
ScaleRotateGestureHandler still pinch; ignored fingers are never handed over to when the owner
lifts. A single finger behaves exactly as before.

### window.rs

- `WindowInner` new fields `secondary_touches`, `ignored_touches`, `pinch_handler` (reset in
  `set_component`, constructor). Secondary fingers get their own `MouseInputState`; the first
  finger (and a pinch pair) stays on `mouse_input_state` + `touch_state`, so hover, cursor,
  drag and drop, popups and `click_state` are primary-only, as before.
- edit: `process_touch_input` first calls new `route_touch` -> `TouchRoute::{Primary, Secondary,
  Ignored}`; `Primary` runs the original body unchanged.
- `route_touch` (decides on `Started`, later phases follow the route):
  1. finger already tracked by `touch_state` / `secondary_touches` / `ignored_touches` -> same route
     (ignored ids are forgotten on Ended/Cancelled);
  2. pinch partner: `touch_state` holds exactly one idle finger, and the new finger and that
     finger are both inside the same enabled `ScaleRotateGestureHandler` (hit test of both
     positions) -> `Primary` (stock two-finger gesture, rule 3). Checked against secondaries'
     items first;
  3. overlap: hit-test items of the new finger vs. items owned by the other fingers (each
     secondary state, the primary state while a plain finger or a held mouse button, the
     running pinch handler) -> `Ignored` (rules 1, 2). The mouse counts as a finger only while a
     button is held (grab or delayed press), a hovering mouse owns nothing;
  4. no overlap and `touch_state` empty and no mouse button held -> `Primary`;
  5. otherwise -> `Secondary` (a fresh `MouseInputState`), except that with any popup open or a
     `DragArea` under the finger it is `Ignored` (popups, drag and drop are primary-only).
- `process_secondary_touch` / `dispatch_secondary_event`: Started -> Pressed, Moved -> Moved,
  Ended -> Released + Exit, Cancelled -> Exit only (not a click); events carry `id + 1` as
  `touch_finger_id`. Runs `handle_mouse_grab` + `input::process_mouse_input` on the finger's own
  state; the cursor is discarded, no click-count/drag/popup logic runs.
- `reset_secondary_touches`: called from `hide()`, sends Exit to every secondary stack so
  nothing stays pressed. `set_component` just drops the state (old items are gone).
- `process_delayed_event_for_finger`: the Flickable press-delay timer now names the finger that
  started it, so a delayed press of a secondary finger is released on that finger's state.
- `any_popup_open`: helper (same parent-adapter lookup as `process_mouse_input`).

### input.rs

- `TouchState`: query helpers `tracks`, `is_empty`, `lone_primary_position`,
  `primary_is_plain_finger` (no change to the state machine).
- edit in `send_mouse_event_to_item`: the `DelayForwarding` timer closure calls
  `process_delayed_event_for_finger(mouse_event.touch_finger_id())` instead of
  `process_delayed_event()` (the latter is still used for the primary / mouse).
- `multitouch_hit_claims`: side-effect free hit test (structural: geometry + item type, no event
  filters, which are stateful). Returns the frontmost handler under the point (children before
  parent, front to back, like `send_mouse_event_to_item`) plus every intercepting ancestor.
  Handlers: enabled TouchArea / TextInput / WindowMoveArea, DragArea, and the interceptors.
  Interceptors: interactive Flickable, enabled SwipeGestureHandler, enabled
  ScaleRotateGestureHandler. Other ancestors (Rectangle, FocusScope, a TouchArea around
  children, ...) are deliberately not "in use". Update this list when Slint adds input items.
- `multitouch_state_claims`: the same notion derived from a finger's `MouseInputState` item
  stack (interceptors on the stack + the top item), only while it holds a grab / delayed press
  (or always, for a touch finger, where hover == pressed).
- `MouseInputState::multitouch_exit_all`: Exit to a stack's items (window hide).

### Known limits

- Touch/mouse: a mouse press while a touch is already the primary shares the primary state (no
  overlap check in that direction; touch arriving while a mouse button is held is handled).
- A finger that lifts out of a pinch re-presses the remaining finger as primary as before; it is
  not overlap-checked against secondaries.
- Hit test is structural; an item that ignores presses at runtime although of a handler type
  (e.g. TextInput that is read-only) still counts as owning the point.
