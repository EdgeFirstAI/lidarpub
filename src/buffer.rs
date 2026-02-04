// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Zero-allocation point cloud buffer infrastructure.
// Allow dead_code: This module provides public API for library consumers.
// The binary doesn't use all methods yet, but they are intentionally exposed.
#![allow(dead_code)]
//!
//! This module provides pre-allocated buffers for point cloud data to eliminate
//! runtime allocations during steady-state operation. The [`DoubleBuffer`] type
//! enables zero-copy frame handoff between producers (LiDAR drivers) and
//! consumers (publishers, clustering).
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                      DoubleBuffer                            │
//! │  ┌──────────────────┐     ┌──────────────────┐              │
//! │  │   Buffer A       │     │   Buffer B       │              │
//! │  │  (filling)       │ ←→  │  (ready)         │              │
//! │  │   x: [f32]       │     │   x: [f32]       │              │
//! │  │   y: [f32]       │ swap│   y: [f32]       │              │
//! │  │   z: [f32]       │     │   z: [f32]       │              │
//! │  │   intensity: [u8]│     │   intensity: [u8]│              │
//! │  │   range: [f32]   │     │   range: [f32]   │              │
//! │  └──────────────────┘     └──────────────────┘              │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Example
//!
//! ```
//! use edgefirst_lidarpub::buffer::{DoubleBuffer, PointBuffer};
//!
//! // Create a double buffer with capacity for 30,000 points
//! let mut db = DoubleBuffer::new(30_000);
//!
//! // Fill the current buffer
//! let filling = db.filling_mut();
//! filling.push(1.0, 2.0, 3.0, 128, 3.74);
//! filling.push(4.0, 5.0, 6.0, 255, 8.77);
//!
//! // Swap and get the completed buffer
//! let completed = db.swap();
//! assert_eq!(completed.len(), 2);
//! assert_eq!(completed.x()[0], 1.0);
//!
//! // The new filling buffer is empty and ready
//! assert_eq!(db.filling().len(), 0);
//! ```

/// Pre-allocated point cloud buffer with zero-copy access.
///
/// This buffer stores point cloud data in a structure-of-arrays (SoA) layout
/// for efficient SIMD processing. The `range` field stores pre-computed
/// distances to avoid redundant sqrt calculations during clustering.
///
/// # Memory Layout
///
/// Points are stored in separate vectors for optimal cache utilization
/// during SIMD operations:
/// - `x`, `y`, `z`: 3D coordinates as f32
/// - `intensity`: Reflectivity/signal strength as u8
/// - `range`: Pre-computed distance (sqrt(x² + y² + z²)) as f32
#[derive(Debug, Clone)]
pub struct PointBuffer {
    x: Vec<f32>,
    y: Vec<f32>,
    z: Vec<f32>,
    intensity: Vec<u8>,
    range: Vec<f32>,
    len: usize,
}

impl PointBuffer {
    /// Create a new buffer with the specified capacity.
    ///
    /// Memory is allocated once at construction; no allocations occur during
    /// normal operation (push/clear).
    ///
    /// # Arguments
    ///
    /// * `capacity` - Maximum number of points this buffer can hold
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            x: vec![0.0; capacity],
            y: vec![0.0; capacity],
            z: vec![0.0; capacity],
            intensity: vec![0; capacity],
            range: vec![0.0; capacity],
            len: 0,
        }
    }

    /// Returns the number of valid points in the buffer.
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Returns true if the buffer contains no points.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns the maximum capacity of the buffer.
    #[inline]
    pub fn capacity(&self) -> usize {
        self.x.capacity()
    }

    /// Clear all points, resetting length to zero.
    ///
    /// **Important**: This does NOT zero the underlying memory for performance.
    /// The buffer retains its capacity and no allocations occur.
    #[inline]
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Add a point to the buffer.
    ///
    /// # Arguments
    ///
    /// * `x`, `y`, `z` - 3D coordinates
    /// * `intensity` - Reflectivity value
    /// * `range` - Pre-computed distance from sensor
    ///
    /// # Panics
    ///
    /// Panics in debug mode if the buffer is full. In release mode,
    /// points beyond capacity are silently ignored.
    #[inline]
    pub fn push(&mut self, x: f32, y: f32, z: f32, intensity: u8, range: f32) {
        debug_assert!(
            self.len < self.capacity(),
            "PointBuffer overflow: {} >= {}",
            self.len,
            self.capacity()
        );

        if self.len < self.capacity() {
            self.x[self.len] = x;
            self.y[self.len] = y;
            self.z[self.len] = z;
            self.intensity[self.len] = intensity;
            self.range[self.len] = range;
            self.len += 1;
        }
    }

    /// Returns a slice of valid X coordinates.
    #[inline]
    pub fn x(&self) -> &[f32] {
        &self.x[..self.len]
    }

    /// Returns a slice of valid Y coordinates.
    #[inline]
    pub fn y(&self) -> &[f32] {
        &self.y[..self.len]
    }

    /// Returns a slice of valid Z coordinates.
    #[inline]
    pub fn z(&self) -> &[f32] {
        &self.z[..self.len]
    }

    /// Returns a slice of valid intensity values.
    #[inline]
    pub fn intensity(&self) -> &[u8] {
        &self.intensity[..self.len]
    }

    /// Returns a slice of valid range (distance) values.
    ///
    /// Range is pre-computed during point insertion to avoid
    /// redundant sqrt calculations in downstream processing.
    #[inline]
    pub fn range(&self) -> &[f32] {
        &self.range[..self.len]
    }

    /// Returns mutable access to the X coordinate array.
    ///
    /// Use with caution - prefer `push()` for normal operation.
    #[inline]
    pub fn x_mut(&mut self) -> &mut [f32] {
        &mut self.x[..self.len]
    }

    /// Returns mutable access to the Y coordinate array.
    #[inline]
    pub fn y_mut(&mut self) -> &mut [f32] {
        &mut self.y[..self.len]
    }

    /// Returns mutable access to the Z coordinate array.
    #[inline]
    pub fn z_mut(&mut self) -> &mut [f32] {
        &mut self.z[..self.len]
    }

    /// Returns mutable access to the intensity array.
    #[inline]
    pub fn intensity_mut(&mut self) -> &mut [u8] {
        &mut self.intensity[..self.len]
    }

    /// Returns mutable access to the range array.
    #[inline]
    pub fn range_mut(&mut self) -> &mut [f32] {
        &mut self.range[..self.len]
    }

    /// Set the valid length of the buffer.
    ///
    /// Used when populating the buffer through direct array access
    /// rather than `push()`.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `len` does not exceed capacity
    /// and that all data up to `len` has been initialized.
    #[inline]
    pub fn set_len(&mut self, len: usize) {
        debug_assert!(len <= self.capacity());
        self.len = len.min(self.capacity());
    }

    /// Get direct access to the underlying X array for bulk writes.
    ///
    /// Returns the full capacity array. Use `set_len()` after populating.
    #[inline]
    pub fn x_raw(&mut self) -> &mut Vec<f32> {
        &mut self.x
    }

    /// Get direct access to the underlying Y array for bulk writes.
    #[inline]
    pub fn y_raw(&mut self) -> &mut Vec<f32> {
        &mut self.y
    }

    /// Get direct access to the underlying Z array for bulk writes.
    #[inline]
    pub fn z_raw(&mut self) -> &mut Vec<f32> {
        &mut self.z
    }

    /// Get direct access to the underlying intensity array for bulk writes.
    #[inline]
    pub fn intensity_raw(&mut self) -> &mut Vec<u8> {
        &mut self.intensity
    }

    /// Get direct access to the underlying range array for bulk writes.
    #[inline]
    pub fn range_raw(&mut self) -> &mut Vec<f32> {
        &mut self.range
    }
}

impl Default for PointBuffer {
    fn default() -> Self {
        Self::with_capacity(0)
    }
}

/// Double-buffer for zero-allocation frame swapping.
///
/// Maintains two [`PointBuffer`]s: one being filled by the driver,
/// and one holding the last complete frame for consumption. When a frame
/// is complete, `swap()` exchanges them atomically, providing the consumer
/// with the completed buffer while the driver continues filling the other.
///
/// This pattern eliminates allocations during steady-state operation since
/// the consumer receives a reference to the completed buffer rather than
/// a clone.
///
/// # Thread Safety
///
/// `DoubleBuffer` is not thread-safe. It is designed to be owned by a single
/// driver and used within a single async task. For multi-threaded scenarios,
/// wrap in appropriate synchronization primitives.
#[derive(Debug)]
pub struct DoubleBuffer {
    buffers: [PointBuffer; 2],
    filling_idx: usize,
}

impl DoubleBuffer {
    /// Create a new double buffer with specified capacity per buffer.
    ///
    /// Allocates two buffers, each capable of holding `capacity` points.
    pub fn new(capacity: usize) -> Self {
        Self {
            buffers: [
                PointBuffer::with_capacity(capacity),
                PointBuffer::with_capacity(capacity),
            ],
            filling_idx: 0,
        }
    }

    /// Returns a reference to the buffer currently being filled.
    #[inline]
    pub fn filling(&self) -> &PointBuffer {
        &self.buffers[self.filling_idx]
    }

    /// Returns a mutable reference to the buffer currently being filled.
    #[inline]
    pub fn filling_mut(&mut self) -> &mut PointBuffer {
        &mut self.buffers[self.filling_idx]
    }

    /// Returns a reference to the buffer holding the last completed frame.
    ///
    /// Before any `swap()` call, this returns an empty buffer.
    #[inline]
    pub fn ready(&self) -> &PointBuffer {
        &self.buffers[1 - self.filling_idx]
    }

    /// Swap buffers and return a reference to the newly-ready (completed)
    /// buffer.
    ///
    /// After calling:
    /// - The returned buffer contains the completed frame data
    /// - The filling buffer is cleared and ready for new data
    ///
    /// This operation is O(1) - only indices change, no data is copied.
    #[inline]
    pub fn swap(&mut self) -> &PointBuffer {
        // Swap indices
        self.filling_idx = 1 - self.filling_idx;

        // Clear the new filling buffer for the next frame
        self.buffers[self.filling_idx].clear();

        // Return reference to the newly-ready buffer (old filling buffer)
        &self.buffers[1 - self.filling_idx]
    }

    /// Returns the capacity of each buffer.
    #[inline]
    pub fn capacity(&self) -> usize {
        self.buffers[0].capacity()
    }
}

impl Default for DoubleBuffer {
    fn default() -> Self {
        Self::new(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_buffer_basic() {
        let mut buf = PointBuffer::with_capacity(100);
        assert_eq!(buf.len(), 0);
        assert!(buf.is_empty());
        assert_eq!(buf.capacity(), 100);

        buf.push(1.0, 2.0, 3.0, 128, 3.74);
        assert_eq!(buf.len(), 1);
        assert!(!buf.is_empty());
        assert_eq!(buf.x()[0], 1.0);
        assert_eq!(buf.y()[0], 2.0);
        assert_eq!(buf.z()[0], 3.0);
        assert_eq!(buf.intensity()[0], 128);
        assert_eq!(buf.range()[0], 3.74);

        buf.push(4.0, 5.0, 6.0, 255, 8.77);
        assert_eq!(buf.len(), 2);

        buf.clear();
        assert_eq!(buf.len(), 0);
        assert!(buf.is_empty());
        // Capacity unchanged
        assert_eq!(buf.capacity(), 100);
    }

    #[test]
    fn test_point_buffer_slices() {
        let mut buf = PointBuffer::with_capacity(10);
        for i in 0..5 {
            buf.push(i as f32, (i * 2) as f32, (i * 3) as f32, i as u8, i as f32);
        }

        assert_eq!(buf.x().len(), 5);
        assert_eq!(buf.y().len(), 5);
        assert_eq!(buf.z().len(), 5);
        assert_eq!(buf.intensity().len(), 5);
        assert_eq!(buf.range().len(), 5);

        // Verify values
        assert_eq!(buf.x(), &[0.0, 1.0, 2.0, 3.0, 4.0]);
        assert_eq!(buf.intensity(), &[0, 1, 2, 3, 4]);
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore)]
    fn test_point_buffer_overflow_ignored() {
        // This test only runs in release mode since debug_assert! panics in debug
        let mut buf = PointBuffer::with_capacity(2);
        buf.push(1.0, 1.0, 1.0, 1, 1.0);
        buf.push(2.0, 2.0, 2.0, 2, 2.0);
        // Third push exceeds capacity - silently ignored in release
        buf.push(3.0, 3.0, 3.0, 3, 3.0);

        assert_eq!(buf.len(), 2);
        assert_eq!(buf.x(), &[1.0, 2.0]);
    }

    #[test]
    fn test_double_buffer_swap() {
        let mut db = DoubleBuffer::new(100);

        // Fill first buffer
        db.filling_mut().push(1.0, 2.0, 3.0, 100, 3.74);
        db.filling_mut().push(4.0, 5.0, 6.0, 200, 8.77);
        assert_eq!(db.filling().len(), 2);

        // Swap - get completed, filling is now empty
        let completed = db.swap();
        assert_eq!(completed.len(), 2);
        assert_eq!(completed.x()[0], 1.0);
        assert_eq!(completed.x()[1], 4.0);
        assert_eq!(db.filling().len(), 0);

        // Fill second buffer
        db.filling_mut().push(7.0, 8.0, 9.0, 50, 13.93);
        assert_eq!(db.filling().len(), 1);

        // Previous ready buffer (completed from first swap) unchanged
        assert_eq!(db.ready().len(), 2);

        // Swap again
        let completed = db.swap();
        assert_eq!(completed.len(), 1);
        assert_eq!(completed.x()[0], 7.0);
        assert_eq!(db.filling().len(), 0);
    }

    #[test]
    fn test_double_buffer_capacity() {
        let db = DoubleBuffer::new(500);
        assert_eq!(db.capacity(), 500);
        assert_eq!(db.filling().capacity(), 500);
        assert_eq!(db.ready().capacity(), 500);
    }

    #[test]
    fn test_point_buffer_raw_access() {
        let mut buf = PointBuffer::with_capacity(5);

        // Direct array access for bulk population
        buf.x_raw()[0] = 10.0;
        buf.x_raw()[1] = 20.0;
        buf.y_raw()[0] = 11.0;
        buf.y_raw()[1] = 21.0;
        buf.z_raw()[0] = 12.0;
        buf.z_raw()[1] = 22.0;
        buf.intensity_raw()[0] = 100;
        buf.intensity_raw()[1] = 200;
        buf.range_raw()[0] = 19.0;
        buf.range_raw()[1] = 36.0;
        buf.set_len(2);

        assert_eq!(buf.len(), 2);
        assert_eq!(buf.x(), &[10.0, 20.0]);
        assert_eq!(buf.intensity(), &[100, 200]);
    }
}
