//! Unified touch-first ergonomics & responsive sizing for Desktop, Web, and Tablet.
//!
//! All platforms share the exact same 3-screen app layout (`Dashboard`, `Control`, `Config & Calibration`)
//! with generous tap targets, clean spacing, and automatic landscape/portrait adaptation.

use egui::{FontId, TextStyle, Vec2};

/// Active viewport orientation form factor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FormFactor {
    Landscape,
    Portrait,
}

impl FormFactor {
    pub fn is_portrait(self) -> bool {
        matches!(self, Self::Portrait)
    }

    pub fn badge(self) -> &'static str {
        match self {
            Self::Landscape => "LANDSCAPE",
            Self::Portrait => "PORTRAIT",
        }
    }
}

/// Unified touch-first UI metrics resolved from viewport dimensions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UiMetrics {
    pub form_factor: FormFactor,
    pub top_bar_height: f32,
    pub status_bar_height: f32,
    pub min_button_height: f32,
    pub estop_size: Vec2,
    pub authority_btn_height: f32,
    pub slider_rail_height: f32,
    pub slider_width: f32,
    pub button_padding: Vec2,
    pub item_spacing: Vec2,
    pub scroll_bar_width: f32,
    pub compact_toolbar: bool,
    pub viewport_pick_scale: f32,
}

impl UiMetrics {
    pub fn resolve(viewport_size: Vec2) -> Self {
        let w = viewport_size.x.max(320.0);
        let h = viewport_size.y.max(320.0);
        let portrait = h > w * 1.02;
        let compact = w < 1040.0 || portrait;

        if portrait {
            Self {
                form_factor: FormFactor::Portrait,
                top_bar_height: 52.0,
                status_bar_height: 26.0,
                min_button_height: 36.0,
                estop_size: Vec2::new(98.0, 36.0),
                authority_btn_height: 36.0,
                slider_rail_height: 14.0,
                slider_width: 170.0,
                button_padding: Vec2::new(11.0, 7.0),
                item_spacing: Vec2::new(8.0, 8.0),
                scroll_bar_width: 12.0,
                compact_toolbar: true,
                viewport_pick_scale: 1.45,
            }
        } else {
            Self {
                form_factor: FormFactor::Landscape,
                top_bar_height: 48.0,
                status_bar_height: 24.0,
                min_button_height: 32.0,
                estop_size: Vec2::new(if compact { 102.0 } else { 132.0 }, 34.0),
                authority_btn_height: 32.0,
                slider_rail_height: 12.0,
                slider_width: 155.0,
                button_padding: Vec2::new(10.0, 6.0),
                item_spacing: Vec2::new(8.0, 7.0),
                scroll_bar_width: 11.0,
                compact_toolbar: compact,
                viewport_pick_scale: 1.25,
            }
        }
    }
}

/// Applies unified touch-first spacing, slider rails, and typography across Desktop, Web, and Tablet.
pub fn apply_responsive_style(ctx: &egui::Context, metrics: &UiMetrics) {
    ctx.all_styles_mut(|style| {
        style.spacing.item_spacing = metrics.item_spacing;
        style.spacing.button_padding = metrics.button_padding;
        style.spacing.interact_size = Vec2::new(40.0, metrics.min_button_height);
        style.spacing.slider_width = metrics.slider_width;
        style.spacing.slider_rail_height = metrics.slider_rail_height;
        style.spacing.combo_height = 260.0;
        style.spacing.scroll.bar_width = metrics.scroll_bar_width;
        style.spacing.scroll.floating = false;
        style.spacing.icon_width = 18.0;
        style.spacing.icon_width_inner = 10.0;
        style.spacing.icon_spacing = 7.0;
        style.interaction.selectable_labels = false;

        style
            .text_styles
            .insert(TextStyle::Body, FontId::proportional(13.5));
        style
            .text_styles
            .insert(TextStyle::Button, FontId::proportional(13.0));
        style
            .text_styles
            .insert(TextStyle::Monospace, FontId::monospace(12.5));
        style
            .text_styles
            .insert(TextStyle::Small, FontId::proportional(11.5));
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unified_metrics_adapt_between_landscape_and_portrait() {
        let land = UiMetrics::resolve(Vec2::new(1440.0, 900.0));
        assert_eq!(land.form_factor, FormFactor::Landscape);
        assert!(land.min_button_height >= 32.0);
        assert!(!land.compact_toolbar);

        let port = UiMetrics::resolve(Vec2::new(800.0, 1280.0));
        assert_eq!(port.form_factor, FormFactor::Portrait);
        assert!(port.form_factor.is_portrait());
        assert!(port.compact_toolbar);
        assert!(port.min_button_height >= 36.0);
    }
}
