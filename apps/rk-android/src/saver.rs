//! Android implementation of [`rk_ui::PlatformSaver`].
//!
//! All files live under the app-internal data directory reported by
//! [`AndroidApp::internal_data_path`](winit::platform::android::activity::AndroidApp::internal_data_path).
//! On the host (tests) the same code is driven with an explicit temp directory,
//! so none of the test paths touch Android APIs.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use rk_ui::PlatformSaver;

/// Queue of documents selected by a future SAF/JNI picker. Shared via `Arc` so
/// a JNI callback thread can enqueue bytes while the UI thread drains them in
/// [`PlatformSaver::poll_opened_files`].
type OpenedQueue = Arc<Mutex<Vec<(String, Vec<u8>)>>>;

pub struct AndroidSaver {
    /// App-internal data path on Android; `None` if the activity is gone.
    /// On the host this is supplied explicitly (e.g. a temp dir in unit tests).
    base_dir: Option<PathBuf>,
    opened_queue: OpenedQueue,
}

impl AndroidSaver {
    /// Build a saver rooted at `base_dir` (the value of
    /// `AndroidApp::internal_data_path()`, already an app-private directory).
    pub fn new(base_dir: Option<PathBuf>) -> Self {
        Self {
            base_dir,
            opened_queue: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Directory that receives exported logs and imported URDFs.
    fn files_dir(&self) -> Result<PathBuf, String> {
        let root = self.base_dir.as_ref().ok_or_else(|| {
            "Android internal data directory is unavailable (activity destroyed?)".to_string()
        })?;
        Ok(root.join("files"))
    }

    /// Enqueue a document chosen outside the save path (e.g. by a SAF picker).
    /// Kept separate from `PlatformSaver` so the picker can be added later
    /// without changing the UI crate. Currently only the test path feeds it;
    /// a future JNI document-picker callback will use the same hook.
    #[allow(dead_code)]
    pub fn enqueue_opened_file(&self, filename: String, bytes: Vec<u8>) {
        if let Ok(mut queue) = self.opened_queue.lock() {
            queue.push((sanitize_filename(&filename), bytes));
        }
    }
}

impl PlatformSaver for AndroidSaver {
    fn save_file(&mut self, filename: &str, _mime: &str, bytes: &[u8]) -> Result<String, String> {
        let dir = self.files_dir()?;
        std::fs::create_dir_all(&dir)
            .map_err(|e| format!("Failed to create {}: {e}", dir.display()))?;
        let dest = dir.join(sanitize_filename(filename));
        std::fs::write(&dest, bytes)
            .map_err(|e| format!("Failed to write {}: {e}", dest.display()))?;
        Ok(format!("Saved to {}", dest.display()))
    }

    fn request_open_urdf_file(&mut self) -> Result<(), String> {
        // A Storage Access Framework / JNI picker is deferred (no device to
        // validate it against). Until then the documented fallback is a URDF
        // bundled with the app or supplied by the offline tooling workflow.
        // Deliberately do NOT spawn `zenity` or use a cwd-relative path here.
        Err("URDF picker is not configured in the Android APK — use a bundled URDF instead.".to_string())
    }

    fn poll_opened_files(&mut self) -> Vec<(String, Vec<u8>)> {
        self.opened_queue
            .lock()
            .map(|mut queue| std::mem::take(&mut *queue))
            .unwrap_or_default()
    }

    fn is_fullscreen(&self, ctx: &egui::Context) -> bool {
        ctx.input(|i| i.viewport().fullscreen.unwrap_or(false))
    }

    fn toggle_fullscreen(&mut self, ctx: &egui::Context) -> Result<bool, String> {
        let next = !self.is_fullscreen(ctx);
        ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(next));
        Ok(next)
    }
}

/// Reduce an untrusted filename to a single, safe path component.
///
/// Runs of characters outside `[A-Za-z0-9._-]` collapse to a single `_`, and
/// leading/trailing dots and underscores are stripped so `..` can never form
/// a parent-directory component. An empty result falls back to `file`.
fn sanitize_filename(filename: &str) -> String {
    let mut out = String::new();
    let mut last_replaced = false;
    for c in filename.chars() {
        if c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | '.') {
            out.push(c);
            last_replaced = false;
        } else if !last_replaced {
            out.push('_');
            last_replaced = true;
        }
    }

    let trimmed = out.trim_matches(|c| c == '.' || c == '_');
    if trimmed.is_empty() {
        return "file".to_string();
    }
    let mut result = trimmed.to_string();
    result.truncate(120);
    let result = result.trim_end_matches(|c| c == '.' || c == '_');
    if result.is_empty() {
        "file".to_string()
    } else {
        result.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sanitizes_path_traversal_and_odd_characters() {
        assert_eq!(sanitize_filename("walk.urdf"), "walk.urdf");
        assert_eq!(sanitize_filename("../../etc/passwd"), "etc_passwd");
        assert_eq!(sanitize_filename(".."), "file");
        assert_eq!(sanitize_filename(""), "file");
        assert_eq!(sanitize_filename("my model (v2).zip"), "my_model_v2_.zip");
        assert_eq!(sanitize_filename("a/b\\c"), "a_b_c");
    }

    #[test]
    fn saves_under_internal_files_dir() {
        let temp = std::env::temp_dir().join(format!(
            "rk-android-saver-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let mut saver = AndroidSaver::new(Some(temp.clone()));
        let msg = saver
            .save_file("../export.copper", "application/octet-stream", b"log-bytes")
            .expect("save");
        let expected = temp.join("files").join("export.copper");
        assert!(msg.contains(&expected.display().to_string()));
        assert_eq!(std::fs::read(&expected).unwrap(), b"log-bytes");
        std::fs::remove_dir_all(&temp).unwrap();
    }

    #[test]
    fn polls_enqueued_files_once() {
        let mut saver = AndroidSaver::new(None);
        assert!(saver.poll_opened_files().is_empty());
        saver.enqueue_opened_file("robot.urdf".to_string(), vec![1, 2, 3]);
        assert_eq!(
            saver.poll_opened_files(),
            vec![("robot.urdf".to_string(), vec![1, 2, 3])]
        );
        assert!(saver.poll_opened_files().is_empty());
    }

    #[test]
    fn reports_missing_internal_dir() {
        let mut saver = AndroidSaver::new(None);
        let err = saver.save_file("x", "", b"").unwrap_err();
        assert!(err.contains("unavailable"));
    }
}
