use vexide::startup::banner::themes::BannerTheme;

macro_rules! ansi_rgb_bold {
    ($r:expr, $g:expr, $b:expr) => {
        concat!("\x1b[1;38;2", $r, ";", $g, ";", $b, "m")
    };
}

pub const THEME_RAINBOTS: BannerTheme = BannerTheme {
    emoji: "🌧️",
    logo_primary: [
        ansi_rgb_bold!(27, 140, 238),
        ansi_rgb_bold!(0, 154, 243),
        ansi_rgb_bold!(0, 167, 245),
        ansi_rgb_bold!(0, 180, 246),
        ansi_rgb_bold!(0, 193, 245),
        ansi_rgb_bold!(0, 205, 242),
        ansi_rgb_bold!(0, 216, 239),
    ],
    logo_secondary: ansi_rgb_bold!(93, 98, 113),
    crate_version: "[1;33m",
    metadata_key: "[1;33m",
};
