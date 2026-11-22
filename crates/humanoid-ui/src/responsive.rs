//! Adaptive ergonomics & layout engine for Desktop Computers and Android Tablets (Touchscreens).
//!
//! Pure, deterministic profile resolution allows testing desktop, tablet-landscape, and
//! tablet-portrait thresholds without a windowing context.

use egui::{FontId, TextStyle, Vec2};

/// User-selectable ergonomics mode in the top toolbar.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UiProfileMode {
    /// Automatically adapts based on touch events, viewport width, and aspect ratio.
    Auto,
    /// High-density mouse + keyboard workstation layout.
    Desktop,
    /// Touch-first Android tablet ergonomics (larger tap targets, thick sliders, touch tab bar).
    Tablet,
}

impl UiProfileMode {
    pub fn label(self) -> &'static str {
        match self {
            Self::Auto => "Auto",
            Self::Desktop => "Desktop",
            Self::Tablet => "Tablet Touch",
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Auto => Self::Tablet,
            Self::Tablet => Self::Desktop,
            Self::Desktop => Self::Auto,
        }
    }
}

/// Active resolved form factor for the current frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FormFactor {
    /// Wide desktop display with mouse/keyboard density and full craft-ui docking split.
    Desktop,
    /// Horizontal tablet (e.g. 1280x800 or 1024x768) with touch-sized controls and optional Full-3D toggle.
    TabletLandscape,
    /// Vertical tablet (e.g. 800x1280) with top/bottom stacked 3D canvas + touch inspector.
    TabletPortrait,
}

impl FormFactor {
    pub fn is_touch(self) -> bool {
        !matches!(self, Self::Desktop)
    }

    pub fn is_portrait(self) -> bool {
        matches!(self, Self::TabletPortrait)
    }

    pub fn badge(self) -> &'static str {
        match self {
            Self::Desktop => "DESKTOP",
            Self::TabletLandscape => "TABLET LANDSCAPE",
            Self::TabletPortrait => "TABLET PORTRAIT",
        }
    }
}

/// Concrete spacing and hit-target metrics derived from [`FormFactor`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UiMetrics {
    pub form_factor: FormFactor,
    pub touch_mode: bool,
    pub top_bar_height: f32,
    pub status_bar_height: f32,
    pub touch_tab_bar_height: f32,
    pub min_button_height: f32,
    pub estop_size: Vec2,
    pub authority_btn_height: f32,
    pub slider_rail_height: f32,
    pub slider_width: f32,
    pub button_padding: Vec2,
    pub item_spacing: Vec2,
    pub scroll_bar_width: f32,
    pub panel_margin: i8,
    pub split_ratio_3d: f32,
    pub compact_toolbar: bool,
    pub show_full_telemetry_inline: bool,
    pub viewport_pick_scale: f32,
}

impl UiMetrics {
    /// Resolves the active form factor and ergonomic metrics from viewport size and input hints.
    pub fn resolve(mode: UiProfileMode, viewport_size: Vec2, seen_touch: bool) -> Self {
        let w = viewport_size.x.max(320.0);
        let h = viewport_size.y.max(320.0);
        let portrait = h > w * 1.02;

        let form_factor = match mode {
            UiProfileMode::Desktop => FormFactor::Desktop,
            UiProfileMode::Tablet => {
                if portrait {
                    FormFactor::TabletPortrait
                } else {
                    FormFactor::TabletLandscape
                }
            }
            UiProfileMode::Auto => {
                if portrait {
                    FormFactor::TabletPortrait
                } else if seen_touch || w <= 1180.0 {
                    FormFactor::TabletLandscape
                } else {
                    FormFactor::Desktop
                }
            }
        };

        let compact_toolbar = w < 1260.0 || form_factor.is_touch();

        match form_factor {
            FormFactor::Desktop => Self {
                form_factor,
                touch_mode: false,
                top_bar_height: 46.0,
                status_bar_height: 26.0,
                touch_tab_bar_height: 0.0,
                min_button_height: 24.0,
                estop_size: Vec2::new(124.0, 30.0),
                authority_btn_height: 28.0,
                slider_rail_height: 7.0,
                slider_width: 140.0,
                button_padding: Vec2::new(8.0, 4.0),
                item_spacing: Vec2::new(8.0, 6.0),
                scroll_bar_width: 10.0,
                panel_margin: 10,
                split_ratio_3d: 0.62,
                compact_toolbar,
                show_full_telemetry_inline: w >= 1220.0,
                viewport_pick_scale: 1.0,
            },
            FormFactor::TabletLandscape => Self {
                form_factor,
                touch_mode: true,
                top_bar_height: 54.0,
                status_bar_height: 30.0,
                touch_tab_bar_height: 46.0,
                min_button_height: 36.0,
                estop_size: Vec2::new(108.0, 40.0),
                authority_btn_height: 38.0,
                slider_rail_height: 15.0,
                slider_width: 175.0,
                button_padding: Vec2::new(12.0, 8.0),
                item_spacing: Vec2::new(10.0, 8.0),
                scroll_bar_width: 14.0,
                panel_margin: 12,
                split_ratio_3d: 0.56,
                compact_toolbar: true,
                show_full_telemetry_inline: w >= 1260.0,
                viewport_pick_scale: 1.45,
            },
            FormFactor::TabletPortrait => Self {
                form_factor,
                touch_mode: true,
                top_bar_height: 56.0,
                status_bar_height: 30.0,
                touch_tab_bar_height: 48.0,
                min_button_height: 40.0,
                estop_size: Vec2::new(104.0, 42.0),
                authority_btn_height: 40.0,
                slider_rail_height: 16.0,
                slider_width: 210.0,
                button_padding: Vec2::new(12.0, 9.0),
                item_spacing: Vec2::new(10.0, 9.0),
                scroll_bar_width: 14.0,
                panel_margin: 12,
                split_ratio_3d: 0.50,
                compact_toolbar: true,
                show_full_telemetry_inline: false,
                viewport_pick_scale: 1.55,
            },
        }
    }
}

/// Applies Desktop or Android Tablet touch spacing, slider rails, scrollbars, and typography to `egui`.
pub fn apply_responsive_style(ctx: &egui::Context, metrics: &UiMetrics) {
    ctx.all_styles_mut(|style| {
        style.spacing.item_spacing = metrics.item_spacing;
        style.spacing.button_padding = metrics.button_padding;
        style.spacing.interact_size = Vec2::new(
            if metrics.touch_mode { 44.0 } else { 40.0 },
            metrics.min_button_height,
        );
        style.spacing.slider_width = metrics.slider_width;
        style.spacing.slider_rail_height = metrics.slider_rail_height;
        style.spacing.combo_height = if metrics.touch_mode { 280.0 } else { 220.0 };
        style.spacing.scroll.bar_width = metrics.scroll_bar_width;
        style.spacing.scroll.floating = !metrics.touch_mode;
        style.spacing.icon_width = if metrics.touch_mode { 20.0 } else { 15.0 };
        style.spacing.icon_width_inner = if metrics.touch_mode { 11.0 } else { 8.0 };
        style.spacing.icon_spacing = if metrics.touch_mode { 8.0 } else { 5.0 };

        // Make touch scrolling forgiving and prevent accidental clicks when dragging
        style.interaction.selectable_labels = !metrics.touch_mode;

        if metrics.touch_mode {
            style
                .text_styles
                .insert(TextStyle::Body, FontId::proportional(14.0));
            style
                .text_styles
                .insert(TextStyle::Button, FontId::proportional(13.5));
            style
                .text_styles
                .insert(TextStyle::Monospace, FontId::monospace(13.0));
            style
                .text_styles
                .insert(TextStyle::Small, FontId::proportional(12.0));
        } else {
            style
                .text_styles
                .insert(TextStyle::Body, FontId::proportional(13.0));
            style
                .text_styles
                .insert(TextStyle::Button, FontId::proportional(12.5));
            style
                .text_styles
                .insert(TextStyle::Monospace, FontId::monospace(12.0));
            style
                .text_styles
                .insert(TextStyle::Small, FontId::proportional(11.0));
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auto_profile_selects_desktop_on_wide_non_touch() {
        let m = UiMetrics::resolve(UiProfileMode::Auto, Vec2::new(1440.0, 900.0), false);
        assert_eq!(m.form_factor, FormFactor::Desktop);
        assert!(!m.touch_mode);
        assert!(m.min_button_height <= 26.0);
        assert!(m.show_full_telemetry_inline);
    }

    #[test]
    fn auto_profile_selects_tablet_landscape_on_touch_or_compact_width() {
        // 1280x800 Android tablet with touch input detected
        let m_touch = UiMetrics::resolve(UiProfileMode::Auto, Vec2::new(1280.0, 800.0), true);
        assert_eq!(m_touch.form_factor, FormFactor::TabletLandscape);
        assert!(m_touch.touch_mode);
        assert!(m_touch.min_button_height >= 36.0);
        assert!(m_touch.estop_size.y >= 40.0);
        assert!(m_touch.slider_rail_height >= 14.0);

        // 1024x768 landscape tablet even before first touch
        let m_width = UiMetrics::resolve(UiProfileMode::Auto, Vec2::new(1024.0, 768.0), false);
        assert_eq!(m_width.form_factor, FormFactor::TabletLandscape);
        assert!(m_width.touch_mode);
    }

    #[test]
    fn auto_profile_selects_tablet_portrait_when_tall() {
        let m = UiMetrics::resolve(UiProfileMode::Auto, Vec2::new(800.0, 1280.0), false);
        assert_eq!(m.form_factor, FormFactor::TabletPortrait);
        assert!(m.touch_mode);
        assert!(m.form_factor.is_portrait());
        assert!(m.min_button_height >= 40.0);
        assert!(!m.show_full_telemetry_inline);
    }

    #[test]
    fn manual_override_honors_desktop_and_tablet() {
        let forced_tablet =
            UiMetrics::resolve(UiProfileMode::Tablet, Vec2::new(1920.0, 1080.0), false);
        assert_eq!(forced_tablet.form_factor, FormFactor::TabletLandscape);
        assert!(forced_tablet.touch_mode);

        let forced_desktop =
            UiMetrics::resolve(UiProfileMode::Desktop, Vec2::new(900.0, 1200.0), true);
        assert_eq!(forced_desktop.form_factor, FormFactor::Desktop);
        assert!(!forced_desktop.touch_mode);
    }
}
