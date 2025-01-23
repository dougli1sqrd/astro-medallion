#![cfg_attr(not(test), no_std)]

extern crate alloc;

use core::f64;

pub mod astronomy;
pub mod display;
pub mod rgb;
pub mod rtc;
pub use astro;

struct Interval {
    start: f64,
    end: f64,
}

impl Interval {
    pub fn includes(&self, x: f64) -> bool {
        x >= self.start && x < self.end
    }
}

impl From<(f64, f64)> for Interval {
    fn from(value: (f64, f64)) -> Self {
        Interval { start: value.0, end: value.1 }
    }
}



#[cfg(test)]
mod tests {

}
