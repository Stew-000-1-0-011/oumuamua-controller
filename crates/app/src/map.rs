//! Top-down 2D map: view transform, pane state and the tiny-skia renderer. Slint-free except
//! for the pixel buffer type.

use slint::{Rgba8Pixel, SharedPixelBuffer};
use tiny_skia::{Color, FillRule, LineCap, LineJoin, Paint, PathBuilder, Pixmap, Rect, Stroke, Transform};

/// Pixel size of the map and where it looks. +y is up on screen.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapView {
    /// Field point (m) shown at the middle of the pane.
    pub center: (f32, f32),
    /// Pixels per metre.
    pub scale: f32,
    /// Pane size in pixels.
    pub size: (f32, f32),
}

pub const MIN_SCALE: f32 = 2.0;
pub const MAX_SCALE: f32 = 2000.0;

impl MapView {
    /// The whole `bounds` (`[xmin, ymin, xmax, ymax]`, m) inside a `width` x `height` pane.
    pub fn fit(bounds: [f64; 4], width: f32, height: f32) -> MapView {
        let bw = ((bounds[2] - bounds[0]) as f32).max(0.1);
        let bh = ((bounds[3] - bounds[1]) as f32).max(0.1);
        let margin = 0.94;
        let scale = (width.max(1.0) / bw).min(height.max(1.0) / bh) * margin;
        MapView {
            center: (((bounds[0] + bounds[2]) / 2.0) as f32, ((bounds[1] + bounds[3]) / 2.0) as f32),
            scale: scale.clamp(MIN_SCALE, MAX_SCALE),
            size: (width, height),
        }
    }
    pub fn to_px(&self, p: (f32, f32)) -> (f32, f32) {
        (
            self.size.0 / 2.0 + (p.0 - self.center.0) * self.scale,
            self.size.1 / 2.0 - (p.1 - self.center.1) * self.scale,
        )
    }
    pub fn to_field(&self, px: (f32, f32)) -> (f32, f32) {
        (
            self.center.0 + (px.0 - self.size.0 / 2.0) / self.scale,
            self.center.1 - (px.1 - self.size.1 / 2.0) / self.scale,
        )
    }
    /// Drag the content by `(dx, dy)` pixels.
    pub fn pan(&mut self, dx_px: f32, dy_px: f32) {
        self.center.0 -= dx_px / self.scale;
        self.center.1 += dy_px / self.scale;
    }
    /// Scale by `factor` keeping the field point under `about_px` fixed.
    pub fn zoom(&mut self, factor: f32, about_px: (f32, f32)) {
        if !factor.is_finite() || factor <= 0.0 {
            return;
        }
        let anchor = self.to_field(about_px);
        self.scale = (self.scale * factor).clamp(MIN_SCALE, MAX_SCALE);
        // Re-centre so `anchor` is back under `about_px`.
        self.center.0 = anchor.0 - (about_px.0 - self.size.0 / 2.0) / self.scale;
        self.center.1 = anchor.1 + (about_px.1 - self.size.1 / 2.0) / self.scale;
    }
}

/// The bounds to fit for a field: all walls and the shelf, or a default when unknown.
pub fn field_bounds(walls: &[[f64; 4]], shelf: Option<[f64; 4]>) -> [f64; 4] {
    let mut b = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
    let mut add = |x: f64, y: f64| {
        b[0] = b[0].min(x);
        b[1] = b[1].min(y);
        b[2] = b[2].max(x);
        b[3] = b[3].max(y);
    };
    for w in walls {
        add(w[0], w[1]);
        add(w[2], w[3]);
    }
    if let Some(s) = shelf {
        add(s[0], s[1]);
        add(s[2], s[3]);
    }
    if b[0] > b[2] { [-1.0, -1.0, 5.0, 5.0] } else { b }
}

/// One map pane's own view; panes zoom and pan independently.
#[derive(Debug, Clone)]
pub struct PaneState {
    pub view: MapView,
    /// Pane size in physical pixels (0 until the UI reports it).
    pub size: (u32, u32),
    /// True once the user zoomed or panned: stop re-fitting on field changes.
    pub user_moved: bool,
    pub dirty: bool,
    /// Pose being picked on this pane (drawn as an arrow); set by the page that enables picking.
    pub pose_drag: Option<([f64; 2], f64)>,
}

impl Default for PaneState {
    fn default() -> Self {
        PaneState {
            view: MapView { center: (0.0, 0.0), scale: 50.0, size: (1.0, 1.0) },
            size: (0, 0),
            user_moved: false,
            dirty: true,
            pose_drag: None,
        }
    }
}

impl PaneState {
    pub fn set_size(&mut self, w: u32, h: u32, bounds: [f64; 4]) {
        if self.size == (w, h) {
            return;
        }
        let old = self.size;
        self.size = (w, h);
        self.dirty = true;
        if self.user_moved && old.0 > 0 && old.1 > 0 {
            self.view.size = (w as f32, h as f32);
        } else {
            self.fit(bounds);
        }
    }
    pub fn fit(&mut self, bounds: [f64; 4]) {
        self.view = MapView::fit(bounds, self.size.0 as f32, self.size.1 as f32);
        self.user_moved = false;
        self.dirty = true;
    }
    pub fn pan(&mut self, dx: f32, dy: f32) {
        self.view.pan(dx, dy);
        self.user_moved = true;
        self.dirty = true;
    }
    pub fn zoom(&mut self, factor: f32, about: (f32, f32)) {
        self.view.zoom(factor, about);
        self.user_moved = true;
        self.dirty = true;
    }
}

/// Everything drawn on the map, in field coordinates (m).
#[derive(Debug, Clone, Default)]
pub struct Scene {
    pub walls: Vec<[f64; 4]>,
    pub shelf: Option<[f64; 4]>,
    pub robot_pose: Option<[f64; 3]>,
    pub robot_radius: f64,
    pub scan: Vec<(f32, f32)>,
    pub path: Vec<(f32, f32)>,
    /// Follow target.
    pub reference: Option<[f64; 3]>,
    /// Pose being picked: `([x, y], yaw)`.
    pub pose_drag: Option<([f64; 2], f64)>,
}

pub const BACKGROUND: [u8; 3] = [0x0b, 0x0e, 0x12];

const WALL: [u8; 4] = [0xe8, 0xed, 0xf3, 255];
const SHELF: [u8; 4] = [0xd9, 0xa2, 0x1b, 255];
const PATH: [u8; 4] = [0x3d, 0x8b, 0xfd, 255];
const LIDAR: [u8; 4] = [0x3f, 0xb9, 0x50, 255];
const ROBOT: [u8; 4] = [0xf0, 0x6a, 0x3a, 255];
const GRID: [u8; 4] = [0x1c, 0x22, 0x2b, 255];
const PICK: [u8; 4] = [0xff, 0x4d, 0xb8, 150];
const TARGET: [u8; 4] = [0xb3, 0x8c, 0xff, 255];

fn paint(c: [u8; 4]) -> Paint<'static> {
    let mut p = Paint::default();
    p.set_color_rgba8(c[0], c[1], c[2], c[3]);
    p.anti_alias = true;
    p
}

fn line(pm: &mut Pixmap, a: (f32, f32), b: (f32, f32), c: [u8; 4], width: f32) {
    let mut pb = PathBuilder::new();
    pb.move_to(a.0, a.1);
    pb.line_to(b.0, b.1);
    if let Some(path) = pb.finish() {
        let stroke = Stroke { width, line_cap: LineCap::Round, line_join: LineJoin::Round, ..Stroke::default() };
        pm.stroke_path(&path, &paint(c), &stroke, Transform::identity(), None);
    }
}

/// An arrow from `at` pointing along `yaw`, `len` pixels long.
fn arrow(pm: &mut Pixmap, at: (f32, f32), yaw: f64, len: f32, c: [u8; 4], width: f32) {
    let (dx, dy) = (yaw.cos() as f32, -(yaw.sin() as f32));
    let tip = (at.0 + dx * len, at.1 + dy * len);
    line(pm, at, tip, c, width);
    let head = (len * 0.3).max(8.0);
    for side in [-1.0f32, 1.0] {
        let a = 2.6f32;
        let (s, co) = (a.sin() * side, a.cos());
        let (hx, hy) = (dx * co - dy * s, dx * s + dy * co);
        line(pm, tip, (tip.0 + hx * head, tip.1 + hy * head), c, width);
    }
}

pub fn render(view: &MapView, w: u32, h: u32, scene: &Scene) -> SharedPixelBuffer<Rgba8Pixel> {
    let mut buf = SharedPixelBuffer::<Rgba8Pixel>::new(w.max(1), h.max(1));
    let Some(mut pm) = Pixmap::new(w.max(1), h.max(1)) else { return buf };
    let view = MapView { size: (w as f32, h as f32), ..*view };
    pm.fill(Color::from_rgba8(BACKGROUND[0], BACKGROUND[1], BACKGROUND[2], 255));
    let px = |p: (f64, f64)| view.to_px((p.0 as f32, p.1 as f32));

    // 1 m grid, only when it is not too dense.
    if view.scale >= 12.0 {
        let (tl, br) = (view.to_field((0.0, 0.0)), view.to_field((w as f32, h as f32)));
        let (x0, x1) = (tl.0.floor() as i32, br.0.ceil() as i32);
        let (y0, y1) = (br.1.floor() as i32, tl.1.ceil() as i32);
        if x1 - x0 < 400 && y1 - y0 < 400 {
            for x in x0..=x1 {
                line(&mut pm, view.to_px((x as f32, tl.1)), view.to_px((x as f32, br.1)), GRID, 1.0);
            }
            for y in y0..=y1 {
                line(&mut pm, view.to_px((tl.0, y as f32)), view.to_px((br.0, y as f32)), GRID, 1.0);
            }
        }
    }
    for wl in &scene.walls {
        line(&mut pm, px((wl[0], wl[1])), px((wl[2], wl[3])), WALL, 3.0);
    }
    if let Some(s) = &scene.shelf {
        let (a, b) = (px((s[0], s[1])), px((s[2], s[3])));
        let mut pb = PathBuilder::new();
        pb.move_to(a.0, a.1);
        pb.line_to(b.0, a.1);
        pb.line_to(b.0, b.1);
        pb.line_to(a.0, b.1);
        pb.close();
        if let Some(path) = pb.finish() {
            let stroke = Stroke { width: 2.5, ..Stroke::default() };
            pm.stroke_path(&path, &paint(SHELF), &stroke, Transform::identity(), None);
        }
    }
    for pair in scene.path.windows(2) {
        let (a, b) = (view.to_px(pair[0]), view.to_px(pair[1]));
        line(&mut pm, a, b, PATH, 2.5);
    }
    let dot = paint(LIDAR);
    for &p in &scene.scan {
        let (x, y) = view.to_px(p);
        if x >= -2.0 && y >= -2.0 && x < w as f32 + 2.0 && y < h as f32 + 2.0
            && let Some(r) = Rect::from_xywh(x - 1.0, y - 1.0, 2.0, 2.0)
        {
            pm.fill_rect(r, &dot, Transform::identity(), None);
        }
    }
    if let Some(t) = &scene.reference {
        let at = px((t[0], t[1]));
        if let Some(c) = PathBuilder::from_circle(at.0, at.1, 6.0) {
            let stroke = Stroke { width: 2.0, ..Stroke::default() };
            pm.stroke_path(&c, &paint(TARGET), &stroke, Transform::identity(), None);
        }
    }
    if let Some(p) = &scene.robot_pose {
        let at = px((p[0], p[1]));
        let r = (scene.robot_radius as f32 * view.scale).max(7.0);
        if let Some(c) = PathBuilder::from_circle(at.0, at.1, r) {
            pm.fill_path(&c, &paint(ROBOT), FillRule::Winding, Transform::identity(), None);
            let stroke = Stroke { width: 2.0, ..Stroke::default() };
            pm.stroke_path(&c, &paint(WALL), &stroke, Transform::identity(), None);
        }
        let (dx, dy) = (p[2].cos() as f32, -(p[2].sin() as f32));
        line(&mut pm, at, (at.0 + dx * r, at.1 + dy * r), [0xff, 0xff, 0xff, 255], 3.0);
    }
    if let Some((p, yaw)) = &scene.pose_drag {
        arrow(&mut pm, px((p[0], p[1])), *yaw, 56.0, PICK, 6.0);
    }

    // Premultiplied == straight here: the background is opaque, so every pixel is.
    buf.make_mut_bytes().copy_from_slice(pm.data());
    buf
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIELD: [f64; 4] = [0.0, 0.0, 6.0, 4.0];

    #[test]
    fn fit_contains_field() {
        let v = MapView::fit(FIELD, 800.0, 600.0);
        for p in [(0.0, 0.0), (6.0, 0.0), (0.0, 4.0), (6.0, 4.0)] {
            let (x, y) = v.to_px(p);
            assert!((0.0..=800.0).contains(&x) && (0.0..=600.0).contains(&y), "{p:?} -> {x},{y}");
        }
        // +y is up: the top of the field has the smaller pixel y.
        assert!(v.to_px((0.0, 4.0)).1 < v.to_px((0.0, 0.0)).1);
        // Degenerate pane does not panic.
        let _ = MapView::fit(FIELD, 0.0, 0.0);
    }

    #[test]
    fn to_px_and_back() {
        let v = MapView { center: (1.5, -2.0), scale: 37.0, size: (640.0, 480.0) };
        assert_eq!(v.to_px((1.5, -2.0)), (320.0, 240.0));
        let p = (3.2, 0.7);
        let q = v.to_field(v.to_px(p));
        assert!((p.0 - q.0).abs() < 1e-4 && (p.1 - q.1).abs() < 1e-4);
    }

    #[test]
    fn zoom_keeps_anchor() {
        let mut v = MapView::fit(FIELD, 800.0, 600.0);
        let about = (123.0, 456.0);
        let before = v.to_field(about);
        v.zoom(1.7, about);
        let after = v.to_field(about);
        assert!((before.0 - after.0).abs() < 1e-3 && (before.1 - after.1).abs() < 1e-3);
        let s = v.scale;
        v.zoom(1e9, about);
        assert!(v.scale <= MAX_SCALE && v.scale >= s);
    }

    #[test]
    fn pan_drags_content() {
        let mut v = MapView::fit(FIELD, 800.0, 600.0);
        let p = v.to_px((1.0, 1.0));
        v.pan(30.0, -10.0);
        let q = v.to_px((1.0, 1.0));
        assert!((q.0 - p.0 - 30.0).abs() < 1e-3 && (q.1 - p.1 + 10.0).abs() < 1e-3);
    }

    fn px(buf: &SharedPixelBuffer<Rgba8Pixel>, x: u32, y: u32) -> [u8; 4] {
        let p = buf.as_slice()[(y * buf.width() + x) as usize];
        [p.r, p.g, p.b, p.a]
    }

    #[test]
    fn render_draws_robot_pixel() {
        let view = MapView::fit(FIELD, 400.0, 300.0);
        let empty = render(&view, 400, 300, &Scene::default());
        assert_eq!(empty.width(), 400);
        assert_eq!(px(&empty, 170, 120), [BACKGROUND[0], BACKGROUND[1], BACKGROUND[2], 255]);
        let scene = Scene { robot_pose: Some([3.0, 2.0, 0.5]), robot_radius: 0.3, ..Scene::default() };
        let buf = render(&view, 400, 300, &scene);
        let (cx, cy) = view.to_px((3.0, 2.0));
        let c = px(&buf, cx as u32, cy as u32);
        assert_ne!(&c[..3], &BACKGROUND[..]);
        assert_eq!(c[3], 255);
    }

    #[test]
    fn render_draws_walls_scan_and_overlay() {
        let view = MapView::fit(FIELD, 400.0, 300.0);
        let scene = Scene {
            walls: vec![[0.0, 0.0, 6.0, 0.0]],
            shelf: Some([1.0, 1.0, 2.0, 2.0]),
            scan: vec![(3.0, 3.0)],
            path: vec![(1.0, 3.0), (5.0, 3.5)],
            pose_drag: Some(([4.0, 2.0], 1.0)),
            ..Scene::default()
        };
        let buf = render(&view, 400, 300, &scene);
        let (sx, sy) = view.to_px((3.0, 3.0));
        assert_ne!(&px(&buf, sx as u32, sy as u32)[..3], &BACKGROUND[..]);
        let (wx, wy) = view.to_px((3.0, 0.0));
        let near_wall = (-2..=2).any(|d| px(&buf, wx as u32, (wy as i32 + d) as u32)[..3] != BACKGROUND);
        assert!(near_wall);
    }
}
