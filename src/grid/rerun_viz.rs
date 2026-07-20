//! Rerun logging for the occupancy grid (behind the `viz` feature).

use crate::convert::floor_to_i64;
use crate::grid::OccupancyGrid;

impl OccupancyGrid {
    /// Log the grid to rerun as a grayscale image at `entity`.
    ///
    /// Pixel value is `255 × (1 - p)`: occupied cells dark, free cells light,
    /// unknown mid-grey — matching the PGM export convention. The image is
    /// top-row-first, so the grid's maximum-y row comes first (+y up in the
    /// viewer's 2D view when the image is displayed conventionally).
    ///
    /// This is a debugging instrument, not a hot path — it allocates one byte
    /// buffer per call.
    ///
    /// # Errors
    ///
    /// Propagates any [`rerun::RecordingStreamError`] from logging.
    pub fn log_to_rerun(
        &self,
        rec: &rerun::RecordingStream,
        entity: &str,
    ) -> Result<(), rerun::RecordingStreamError> {
        let width = self.width();
        let height = self.height();
        let mut bytes = Vec::with_capacity(width * height);
        for y in (0..height).rev() {
            let start = y * width;
            let row = self.cells().get(start..start + width).unwrap_or(&[]);
            bytes.extend(row.iter().map(|&l| gray_from_log_odds(l)));
        }
        // Dimensions were validated to fit u32 at grid construction; the
        // fallback is unreachable.
        let dims = [
            u32::try_from(width).unwrap_or(u32::MAX),
            u32::try_from(height).unwrap_or(u32::MAX),
        ];
        let image = rerun::Image::from_color_model_and_bytes(
            bytes,
            dims,
            rerun::ColorModel::L,
            rerun::ChannelDatatype::U8,
        );
        rec.log(entity, &image)
    }
}

/// Map cell log-odds to a display gray value: `255 × (1 - p)`, rounded.
/// Occupied (p→1) → 0, free (p→0) → 255, unknown (p = 0.5) → 128.
fn gray_from_log_odds(l: f32) -> u8 {
    let p = 1.0 - 1.0 / (1.0 + l.exp());
    let scaled = f64::from((1.0 - p).clamp(0.0, 1.0)) * 255.0;
    // Round via the audited floor conversion; the value is in [0, 255] so the
    // fallbacks are unreachable.
    floor_to_i64(scaled + 0.5)
        .and_then(|v| u8::try_from(v).ok())
        .unwrap_or(205)
}
