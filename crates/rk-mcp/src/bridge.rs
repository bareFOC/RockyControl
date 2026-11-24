//! Native-only bridge between HTTP request threads and the non-`Send` egui thread.
//!
//! The GUI owns the application (`McpHost`) and must dispatch JSON-RPC messages on
//! its own thread. HTTP worker threads submit a message through a bounded queue,
//! wake the GUI, and block on a oneshot reply channel.

use std::marker::PhantomData;
use std::rc::Rc;
use std::sync::mpsc::{
    self, Receiver, RecvTimeoutError, Sender, SyncSender, TrySendError,
};
use std::sync::Arc;
use std::time::Duration;

use serde_json::Value;

/// Bounded number of in-flight MCP requests awaiting GUI dispatch.
pub(crate) const BRIDGE_QUEUE_CAPACITY: usize = 16;

/// Default maximum time an HTTP worker waits for the GUI to reply.
pub const DEFAULT_BRIDGE_REPLY_TIMEOUT: Duration = Duration::from_secs(15);

/// Why a bridge submission could not be completed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BridgeError {
    /// The bounded request queue is full; the client should retry.
    Busy,
    /// The GUI did not reply before the deadline.
    Timeout,
    /// The receiving (GUI) side has been dropped.
    GuiClosed,
}

impl std::fmt::Display for BridgeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Busy => f.write_str("MCP bridge queue is full"),
            Self::Timeout => f.write_str("timed out waiting for the GUI to reply"),
            Self::GuiClosed => f.write_str("MCP GUI bridge is closed"),
        }
    }
}

impl std::error::Error for BridgeError {}

/// One JSON-RPC message plus its oneshot reply channel.
pub struct McpBridgeRequest {
    pub message: Value,
    pub reply: Sender<Option<Value>>,
}

/// Clonable producer half handed to the HTTP server.
#[derive(Clone)]
pub struct McpBridgeTx {
    requests: SyncSender<McpBridgeRequest>,
    wake: Arc<dyn Fn() + Send + Sync>,
}

impl McpBridgeTx {
    /// Queues `message`, wakes the GUI, and waits up to `timeout` for a reply.
    ///
    /// A `None` reply means the message was a JSON-RPC notification (no response body).
    pub fn submit(
        &self,
        message: Value,
        timeout: Duration,
    ) -> Result<Option<Value>, BridgeError> {
        let (reply_tx, reply_rx) = mpsc::channel();
        match self.requests.try_send(McpBridgeRequest {
            message,
            reply: reply_tx,
        }) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => return Err(BridgeError::Busy),
            Err(TrySendError::Disconnected(_)) => return Err(BridgeError::GuiClosed),
        }
        // Wake after enqueueing so a repaint is always requested for queued work.
        (self.wake)();
        match reply_rx.recv_timeout(timeout) {
            Ok(reply) => Ok(reply),
            Err(RecvTimeoutError::Timeout) => Err(BridgeError::Timeout),
            Err(RecvTimeoutError::Disconnected) => Err(BridgeError::GuiClosed),
        }
    }
}

/// Consumer half. Deliberately `!Send`: it must live on the egui thread.
pub struct McpBridgeRx {
    requests: Receiver<McpBridgeRequest>,
    /// `Rc` is `!Send`, so this marker keeps the receiver on the creating thread.
    _not_send: PhantomData<Rc<()>>,
}

impl McpBridgeRx {
    /// Drains every currently queued request without blocking.
    pub fn try_recv_all(&mut self) -> Vec<McpBridgeRequest> {
        let mut out = Vec::new();
        while let Ok(req) = self.requests.try_recv() {
            out.push(req);
        }
        out
    }
}

/// Creates a bounded bridge. `wake` is invoked after each successful enqueue
/// (e.g. `egui::Context::request_repaint`) and must be thread-safe.
pub fn mcp_bridge_channel<F>(wake: F) -> (McpBridgeTx, McpBridgeRx)
where
    F: Fn() + Send + Sync + 'static,
{
    let (requests, receiver) = mpsc::sync_channel(BRIDGE_QUEUE_CAPACITY);
    (
        McpBridgeTx {
            requests,
            wake: Arc::new(wake),
        },
        McpBridgeRx {
            requests: receiver,
            _not_send: PhantomData,
        },
    )
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::thread;
    use std::time::Duration;

    use serde_json::json;

    use super::*;

    #[test]
    fn submit_drain_and_respond() {
        let (tx, mut rx) = mcp_bridge_channel(|| {});
        let worker = thread::spawn(move || {
            tx.submit(json!({"method": "tools/list"}), Duration::from_secs(2))
                .unwrap()
        });

        let requests = loop {
            let requests = rx.try_recv_all();
            if !requests.is_empty() {
                break requests;
            }
            thread::sleep(Duration::from_millis(1));
        };
        assert_eq!(requests.len(), 1);
        assert_eq!(requests[0].message, json!({"method": "tools/list"}));
        requests[0]
            .reply
            .send(Some(json!({"result": {"tools": []}})))
            .unwrap();

        assert_eq!(
            worker.join().unwrap(),
            Some(json!({"result": {"tools": []}}))
        );
    }

    #[test]
    fn notification_reply_is_none() {
        let (tx, mut rx) = mcp_bridge_channel(|| {});
        let worker = thread::spawn(move || {
            tx.submit(json!({"method": "notifications/initialized"}), Duration::from_secs(2))
                .unwrap()
        });

        let requests = loop {
            let requests = rx.try_recv_all();
            if !requests.is_empty() {
                break requests;
            }
            thread::sleep(Duration::from_millis(1));
        };
        requests[0].reply.send(None).unwrap();
        assert_eq!(worker.join().unwrap(), None);
    }

    #[test]
    fn wake_is_called_on_enqueue() {
        let wakes = Arc::new(AtomicUsize::new(0));
        let wakes_clone = Arc::clone(&wakes);
        let (tx, mut rx) = mcp_bridge_channel(move || {
            wakes_clone.fetch_add(1, Ordering::SeqCst);
        });
        let worker = thread::spawn(move || {
            let _ = tx.submit(json!({"method": "ping"}), Duration::from_secs(2));
        });
        let requests = loop {
            let requests = rx.try_recv_all();
            if !requests.is_empty() {
                break requests;
            }
            thread::sleep(Duration::from_millis(1));
        };
        requests[0].reply.send(None).unwrap();
        worker.join().unwrap();
        assert_eq!(wakes.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn full_queue_reports_busy() {
        let (tx, mut rx) = mcp_bridge_channel(|| {});
        // Fill the bounded queue directly so no test thread blocks waiting for a reply.
        for i in 0..BRIDGE_QUEUE_CAPACITY {
            let (reply, _receiver) = mpsc::channel();
            tx.requests
                .try_send(McpBridgeRequest {
                    message: json!({"n": i}),
                    reply,
                })
                .unwrap();
        }
        assert_eq!(
            tx.submit(json!({"n": "overflow"}), Duration::from_millis(10)),
            Err(BridgeError::Busy)
        );
        assert_eq!(rx.try_recv_all().len(), BRIDGE_QUEUE_CAPACITY);
    }

    #[test]
    fn unanswered_request_times_out() {
        let (tx, _rx) = mcp_bridge_channel(|| {});
        assert_eq!(
            tx.submit(json!({"method": "ping"}), Duration::from_millis(20)),
            Err(BridgeError::Timeout)
        );
    }

    #[test]
    fn closed_gui_reports_gui_closed() {
        let (tx, rx) = mcp_bridge_channel(|| {});
        drop(rx);
        assert_eq!(
            tx.submit(json!({"method": "ping"}), Duration::from_millis(20)),
            Err(BridgeError::GuiClosed)
        );
    }
}
