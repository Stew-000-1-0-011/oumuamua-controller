//! Reliable in-order channel (one direction). Pure, time-injected (ms), no sockets.
//! Mirrors `ros/.../reliable.py`.

use std::collections::BTreeMap;

pub const WINDOW: u32 = 64;
pub const RTO_MS: u64 = 100;

#[derive(Debug)]
pub struct Sender<T: Clone> {
    window: u32,
    rto_ms: u64,
    acked: u32,
    next_seq: u32,
    msgs: BTreeMap<u32, T>,
    last_sent: BTreeMap<u32, u64>,
}

impl<T: Clone> Default for Sender<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Clone> Sender<T> {
    pub fn new() -> Self {
        Self::with_params(WINDOW, RTO_MS)
    }

    pub fn with_params(window: u32, rto_ms: u64) -> Self {
        Sender { window, rto_ms, acked: 0, next_seq: 1, msgs: BTreeMap::new(), last_sent: BTreeMap::new() }
    }

    pub fn reset(&mut self) {
        self.acked = 0;
        self.next_seq = 1;
        self.msgs.clear();
        self.last_sent.clear();
    }

    pub fn push(&mut self, msg: T) {
        self.msgs.insert(self.next_seq, msg);
        self.next_seq += 1;
    }

    /// Messages that should be (re)sent now: new ones inside the window, and
    /// unacked ones last sent at least `rto_ms` ago.
    pub fn poll(&mut self, now_ms: u64) -> Vec<(u32, T)> {
        let mut out = Vec::new();
        let top = (self.next_seq - 1).min(self.acked.saturating_add(self.window));
        for seq in self.acked + 1..=top {
            let due = match self.last_sent.get(&seq) {
                None => true,
                Some(&last) => now_ms.saturating_sub(last) >= self.rto_ms,
            };
            if due {
                self.last_sent.insert(seq, now_ms);
                out.push((seq, self.msgs[&seq].clone()));
            }
        }
        out
    }

    pub fn on_ack(&mut self, cum: u32) {
        let cum = cum.min(self.next_seq - 1);
        while self.acked < cum {
            self.acked += 1;
            self.msgs.remove(&self.acked);
            self.last_sent.remove(&self.acked);
        }
    }

    pub fn pending(&self) -> usize {
        (self.next_seq - 1 - self.acked) as usize
    }
}

#[derive(Debug)]
pub struct Receiver<T> {
    window: u32,
    /// Highest sequence number delivered in order (0 if none).
    pub cum: u32,
    buf: BTreeMap<u32, T>,
}

impl<T> Default for Receiver<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Receiver<T> {
    pub fn new() -> Self {
        Self::with_window(WINDOW)
    }

    pub fn with_window(window: u32) -> Self {
        Receiver { window, cum: 0, buf: BTreeMap::new() }
    }

    pub fn reset(&mut self) {
        self.cum = 0;
        self.buf.clear();
    }

    /// Returns the messages now deliverable in order (possibly none).
    pub fn on_rel(&mut self, seq: u32, msg: T) -> Vec<T> {
        if seq <= self.cum || seq > self.cum.saturating_add(self.window) {
            return Vec::new();
        }
        self.buf.entry(seq).or_insert(msg);
        let mut out = Vec::new();
        while let Some(m) = self.buf.remove(&(self.cum + 1)) {
            self.cum += 1;
            out.push(m);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn in_order_delivery() {
        let mut r = Receiver::new();
        assert_eq!(r.on_rel(1, "a"), vec!["a"]);
        assert_eq!(r.on_rel(2, "b"), vec!["b"]);
        assert_eq!(r.cum, 2);
    }

    #[test]
    fn out_of_order_buffered() {
        let mut r = Receiver::new();
        assert!(r.on_rel(3, "c").is_empty());
        assert!(r.on_rel(2, "b").is_empty());
        assert_eq!(r.cum, 0);
        assert_eq!(r.on_rel(1, "a"), vec!["a", "b", "c"]);
        assert_eq!(r.cum, 3);
    }

    #[test]
    fn duplicates_dropped() {
        let mut r = Receiver::new();
        assert_eq!(r.on_rel(1, "a"), vec!["a"]);
        assert!(r.on_rel(1, "a").is_empty());
        assert!(r.on_rel(3, "c").is_empty());
        assert!(r.on_rel(3, "c").is_empty());
        assert_eq!(r.on_rel(2, "b"), vec!["b", "c"]);
    }

    #[test]
    fn beyond_window_dropped() {
        let mut r = Receiver::new();
        assert!(r.on_rel(65, 65).is_empty());
        assert!(r.on_rel(64, 64).is_empty());
        for i in 1..63 {
            assert_eq!(r.on_rel(i, i as i32), vec![i as i32]);
        }
        // 64 was buffered; 63 releases 63 and 64. 65 was dropped.
        assert_eq!(r.on_rel(63, 63), vec![63, 64]);
        assert_eq!(r.cum, 64);
        assert_eq!(r.on_rel(65, 65), vec![65]);
    }

    #[test]
    fn retransmit_after_rto() {
        let mut s = Sender::new();
        s.push("a");
        assert_eq!(s.poll(0), vec![(1, "a")]);
        assert!(s.poll(99).is_empty());
        assert_eq!(s.poll(100), vec![(1, "a")]);
        assert!(s.poll(199).is_empty());
        assert_eq!(s.poll(200), vec![(1, "a")]);
        s.on_ack(1);
        assert!(s.poll(400).is_empty());
        assert_eq!(s.pending(), 0);
    }

    #[test]
    fn window_limits_inflight() {
        let mut s = Sender::new();
        for i in 0..100 {
            s.push(i);
        }
        let out = s.poll(0);
        assert_eq!(out.iter().map(|x| x.0).collect::<Vec<_>>(), (1..=64).collect::<Vec<_>>());
        assert_eq!(s.pending(), 100);
        assert!(s.poll(1).is_empty());
        s.on_ack(10);
        assert_eq!(s.pending(), 90);
        let out = s.poll(2);
        assert_eq!(out.iter().map(|x| x.0).collect::<Vec<_>>(), (65..=74).collect::<Vec<_>>());
    }

    #[test]
    fn stale_ack_ignored_and_reset() {
        let mut s = Sender::new();
        s.push(1);
        s.push(2);
        s.poll(0);
        s.on_ack(2);
        s.on_ack(1);
        assert_eq!(s.pending(), 0);
        s.on_ack(99);
        assert_eq!(s.pending(), 0);
        s.push(3);
        assert_eq!(s.poll(5), vec![(3, 3)]);
        s.reset();
        assert_eq!(s.pending(), 0);
        s.push(7);
        assert_eq!(s.poll(6), vec![(1, 7)]);
        let mut r = Receiver::new();
        r.on_rel(1, "a");
        r.on_rel(3, "c");
        r.reset();
        assert_eq!(r.cum, 0);
        assert_eq!(r.on_rel(1, "z"), vec!["z"]);
        assert_eq!(r.on_rel(2, "y"), vec!["y"]);
    }

    /// Small deterministic xorshift so the test needs no rand crate.
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
    }

    fn transmit<X: Clone>(rng: &mut Rng, q: &mut Vec<X>, item: X) {
        if rng.next() < 0.3 {
            return;
        }
        q.push(item.clone());
        if rng.next() < 0.2 {
            q.push(item);
        }
        let n = q.len();
        if n > 1 && rng.next() < 0.3 {
            q.swap(n - 1, n - 2);
        }
    }

    #[test]
    fn lossy_link_simulation() {
        let mut rng = Rng(0x9E3779B97F4A7C15);
        let n = 500;
        let mut s = Sender::new();
        let mut r = Receiver::new();
        for i in 0..n {
            s.push(i);
        }
        let mut got = Vec::new();
        let mut to_recv: Vec<(u32, i32)> = Vec::new();
        let mut to_send: Vec<u32> = Vec::new();
        let mut now = 0u64;
        while got.len() < n as usize && now < 200_000 {
            for (seq, msg) in s.poll(now) {
                transmit(&mut rng, &mut to_recv, (seq, msg));
            }
            for (seq, msg) in std::mem::take(&mut to_recv) {
                got.extend(r.on_rel(seq, msg));
                transmit(&mut rng, &mut to_send, r.cum);
            }
            for c in std::mem::take(&mut to_send) {
                s.on_ack(c);
            }
            now += 10;
        }
        assert_eq!(got, (0..n).collect::<Vec<_>>());
    }
}
