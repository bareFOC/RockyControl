//! In-memory URDF + CAD mesh asset bundle (`AssetBundle`) and MCU flash serialization helpers.

use std::collections::HashMap;
use std::io::{Read, Write};

use flate2::Compression;
use flate2::read::ZlibDecoder;
use flate2::write::ZlibEncoder;
use humanoid_proto::{ModelFormat, ModelInfo, crc32};

use crate::kinematics::{KinematicRig, VisualShape};

pub const BUNDLED_URDF_XML: &str = include_str!("../../../assets/sample_humanoid/rocky_mk1.urdf");

/// Provenance of the currently loaded robot URDF bundle.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ModelSource {
    Bundled,
    SamplePreset(String),
    UploadedFile(String),
    PastedXml,
    McuFlash { crc32: u32, format: ModelFormat },
}

impl ModelSource {
    pub fn label(&self) -> String {
        match self {
            Self::Bundled => "Bundled (rocky_mk1)".to_string(),
            Self::SamplePreset(name) => format!("Sample ({name})"),
            Self::UploadedFile(name) => format!("Uploaded ({name})"),
            Self::PastedXml => "Pasted URDF XML".to_string(),
            Self::McuFlash { crc32, format } => {
                format!("MCU Flash ({}, {crc32:#010x})", format.label())
            }
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct BundleDiagnostics {
    pub urdf_bytes: usize,
    pub crc32: u32,
    pub total_links: usize,
    pub movable_joints: usize,
    pub visual_count: usize,
    pub mesh_visuals: usize,
    pub resolved_meshes: Vec<String>,
    pub missing_meshes: Vec<String>,
}

/// In-memory URDF + mesh asset bundle (zero direct filesystem dependency for WASM compatibility).
#[derive(Clone, Debug)]
pub struct AssetBundle {
    pub robot_name: String,
    pub urdf_xml: String,
    pub files: HashMap<String, Vec<u8>>,
    pub source: ModelSource,
}

impl Default for AssetBundle {
    fn default() -> Self {
        Self::bundled_rocky()
    }
}

impl AssetBundle {
    pub fn bundled_rocky() -> Self {
        Self {
            robot_name: "rocky_mk1".to_string(),
            urdf_xml: BUNDLED_URDF_XML.to_string(),
            files: HashMap::new(),
            source: ModelSource::Bundled,
        }
    }

    pub fn from_sample(name: &str, urdf_xml: &str) -> Result<Self, String> {
        let parsed =
            urdf_rs::read_from_string(urdf_xml).map_err(|e| format!("URDF parse failed: {e}"))?;
        Ok(Self {
            robot_name: parsed.name,
            urdf_xml: urdf_xml.to_string(),
            files: HashMap::new(),
            source: ModelSource::SamplePreset(name.to_string()),
        })
    }

    pub fn from_pasted_xml(urdf_xml: &str) -> Result<Self, String> {
        let parsed =
            urdf_rs::read_from_string(urdf_xml).map_err(|e| format!("URDF parse failed: {e}"))?;
        Ok(Self {
            robot_name: parsed.name,
            urdf_xml: urdf_xml.to_string(),
            files: HashMap::new(),
            source: ModelSource::PastedXml,
        })
    }

    /// Loads either a raw `.urdf`/`.xml` file or a `.zip` bundle containing a URDF + STL/OBJ meshes.
    pub fn from_bytes(filename: &str, bytes: &[u8]) -> Result<Self, String> {
        let lower = filename.to_ascii_lowercase();
        if lower.ends_with(".zip") {
            let cursor = std::io::Cursor::new(bytes);
            let mut archive =
                zip::ZipArchive::new(cursor).map_err(|e| format!("Invalid zip archive: {e}"))?;
            let mut files = HashMap::new();
            let mut urdf_xml = None;
            for i in 0..archive.len() {
                let mut entry = archive
                    .by_index(i)
                    .map_err(|e| format!("Zip read error: {e}"))?;
                if entry.is_dir() {
                    continue;
                }
                let raw_name = entry.name().replace('\\', "/");
                if raw_name.starts_with("__MACOSX/") {
                    continue;
                }
                let mut data = Vec::new();
                entry
                    .read_to_end(&mut data)
                    .map_err(|e| format!("Zip entry read error: {e}"))?;
                let entry_lower = raw_name.to_ascii_lowercase();
                if entry_lower.ends_with(".urdf") || entry_lower.ends_with(".xml") {
                    if let Ok(s) = String::from_utf8(data.clone()) {
                        if s.contains("<robot") && urdf_xml.is_none() {
                            urdf_xml = Some(s);
                        }
                    }
                }
                files.insert(raw_name, data);
            }
            let urdf_xml =
                urdf_xml.ok_or_else(|| "Zip bundle does not contain a .urdf file".to_string())?;
            let parsed = urdf_rs::read_from_string(&urdf_xml)
                .map_err(|e| format!("URDF parse failed: {e}"))?;
            Ok(Self {
                robot_name: parsed.name,
                urdf_xml,
                files,
                source: ModelSource::UploadedFile(filename.to_string()),
            })
        } else {
            let urdf_xml = String::from_utf8(bytes.to_vec())
                .map_err(|_| "URDF file is not valid UTF-8".to_string())?;
            let parsed = urdf_rs::read_from_string(&urdf_xml)
                .map_err(|e| format!("URDF parse failed: {e}"))?;
            Ok(Self {
                robot_name: parsed.name,
                urdf_xml,
                files: HashMap::new(),
                source: ModelSource::UploadedFile(filename.to_string()),
            })
        }
    }

    /// Reconstructs an `AssetBundle` from a CRC32-verified MCU flash blob.
    pub fn from_mcu_blob(info: &ModelInfo, bytes: &[u8]) -> Result<Self, String> {
        let actual_crc = crc32(bytes);
        if actual_crc != info.crc32 {
            return Err(format!(
                "MCU model CRC32 mismatch: expected {:#010x}, got {:#010x}",
                info.crc32, actual_crc
            ));
        }

        let urdf_xml = match info.format {
            ModelFormat::UrdfXml => String::from_utf8(bytes.to_vec())
                .map_err(|_| "MCU UrdfXml blob is not valid UTF-8".to_string())?,
            ModelFormat::UrdfDeflate => {
                let mut decoder = ZlibDecoder::new(bytes);
                let mut out = String::new();
                decoder
                    .read_to_string(&mut out)
                    .map_err(|e| format!("Failed to decompress MCU UrdfDeflate blob: {e}"))?;
                out
            }
            ModelFormat::CompactKinematics => {
                return Err(
                    "CompactKinematics binary decoding requires schema v2 descriptor".to_string(),
                );
            }
        };

        let parsed = urdf_rs::read_from_string(&urdf_xml)
            .map_err(|e| format!("MCU URDF parse failed: {e}"))?;
        Ok(Self {
            robot_name: parsed.name,
            urdf_xml,
            files: HashMap::new(),
            source: ModelSource::McuFlash {
                crc32: info.crc32,
                format: info.format,
            },
        })
    }

    /// Compresses the active URDF XML with zlib/deflate for compact MCU flash staging.
    pub fn to_deflated_urdf_bytes(&self) -> Result<Vec<u8>, String> {
        let mut enc = ZlibEncoder::new(Vec::new(), Compression::best());
        enc.write_all(self.urdf_xml.as_bytes())
            .map_err(|e| format!("Deflate write error: {e}"))?;
        enc.finish()
            .map_err(|e| format!("Deflate finish error: {e}"))
    }

    /// Resolves a URDF `<mesh filename="..."/>` URI against the in-memory bundle files.
    /// Supports `package://<pkg>/...`, `file://`, relative `./`, Windows `\`, and basename fallback.
    pub fn resolve_mesh_bytes(&self, urdf_filename: &str) -> Option<(&str, &[u8])> {
        if self.files.is_empty() {
            return None;
        }
        let normalized = urdf_filename
            .trim()
            .replace('\\', "/")
            .trim_start_matches("package://")
            .trim_start_matches("file://")
            .trim_start_matches("./")
            .trim_start_matches('/')
            .to_string();
        let norm_lower = normalized.to_ascii_lowercase();

        // 1. Exact or normalized suffix match
        if let Some((k, v)) = self.files.iter().find(|(k, _)| {
            let kl = k.replace('\\', "/").to_ascii_lowercase();
            kl == norm_lower || kl.ends_with(&format!("/{norm_lower}")) || kl.ends_with(&norm_lower)
        }) {
            return Some((k.as_str(), v.as_slice()));
        }

        // 2. Strip first path component (ROS package name in package://pkg_name/meshes/foo.stl)
        if let Some((_, without_pkg)) = norm_lower.split_once('/') {
            if let Some((k, v)) = self.files.iter().find(|(k, _)| {
                let kl = k.replace('\\', "/").to_ascii_lowercase();
                kl == without_pkg || kl.ends_with(&format!("/{without_pkg}"))
            }) {
                return Some((k.as_str(), v.as_slice()));
            }
        }

        // 3. Basename fallback
        let basename = norm_lower.rsplit('/').next().unwrap_or(&norm_lower);
        self.files
            .iter()
            .find(|(k, _)| {
                let kl = k.replace('\\', "/").to_ascii_lowercase();
                kl.rsplit('/').next() == Some(basename)
            })
            .map(|(k, v)| (k.as_str(), v.as_slice()))
    }

    pub fn diagnostics(&self, rig: &KinematicRig) -> BundleDiagnostics {
        let mut resolved_meshes = Vec::new();
        let mut missing_meshes = Vec::new();
        let mut mesh_visuals = 0usize;
        for vis in &rig.visuals {
            if let VisualShape::Mesh { filename, .. } = &vis.shape {
                mesh_visuals += 1;
                if self.resolve_mesh_bytes(filename).is_some() {
                    if !resolved_meshes.contains(filename) {
                        resolved_meshes.push(filename.clone());
                    }
                } else if !missing_meshes.contains(filename) {
                    missing_meshes.push(filename.clone());
                }
            }
        }
        BundleDiagnostics {
            urdf_bytes: self.urdf_xml.len(),
            crc32: crc32(self.urdf_xml.as_bytes()),
            total_links: rig.links.len(),
            movable_joints: rig.joints.len(),
            visual_count: rig.visuals.len(),
            mesh_visuals,
            resolved_meshes,
            missing_meshes,
        }
    }
}
