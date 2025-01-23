//!
//! # Horizon LEDs
//! The horizon display consists of 3 white LEDs per 30 degree sector for a star sign.
//! A sign occupies 30 degrees, starting at "0", up to 30. Each LED should represent
//! an angle spread evenly across the 30, but centered.
//! 3 LEDs across 30 degrees is 10 degrees per LED. To Center the LED in its 10 degree spread
//! we can offset the critical angle by 5 degrees (half the spread)
//! | o x o o x o o x o | o x o ...
//!
//! So each LED is 5, 15, and 25 degrees from the start of the star sign spread
//!
//! The primary way we will interact with the horizon display is to ask some set of angles to be lit or unlit:
//! starting angle and a delta theta.
//!
//! Each LED is indexed starting with the right-most LED in Aries and increasing going counter-clockwise,
//! from 0 to 35.
//!
//! Angle around the horizon disc, however, increases going clockwise. 5 degrees starts with the left-most LED
//! of Aries, or index 0.
//!
//! Angle: 5,   15,   25,   35,   45,   55,   65,   75,   85 ...
//! Index: 2,   1,    0,    35,   34,   33,   32,   31,   30 ...
//!
//! `angle = (5 - 10*(I - 2)) % 360`
//! `angle = (25 - 10*I) % 360`
//!
//! OR
//!
//! `I = ((25 - angle) / 10) % 36`
//!
//!
//! # Planet LEDs
//! The Planet LEDs are formed by 12 spokes of length 7, each rung of the ring representing a planetary position.
//! A planet will be represented by a color. Lastly earth is at the center which is always blue.
//!
//! LEDs are ordered in the following way:
//!
//! The LED's data lines are connected by starting in Aries and the outer-most rung and continue going inward and then
//! counter-clockwise. There are 7 rungs and 12 spokes, so a natural way to refer to an LED is by `(spoke, rung)`,
//! where `spoke`, starting at 0 at Aries and ascends counter-clockwise. And `rung` will start at 0 on the outside ascending
//! going inward towards Earth.
//!
//! LED index is then `7*spoke + rung` with Earth being one more past this.
//!
//!

use core::{f64, fmt::Debug};

use alloc;
use alloc::vec;
use alloc::vec::Vec;
use astro::{
    angle::TWO_PI,
    coords::EclPoint,
};
use core_maths::CoreFloat;
use embedded_hal::{
    digital::{OutputPin, PinState},
    spi::SpiDevice,
};


use crate::{
    astronomy::{
        angular_diameter, diameter, jupiter, mars, mercury, moon, neptune, saturn, sun,
        uranus, venus, within_2pi,
    },
    rgb::{RGBWrite, RGB},
    Interval,
};

pub const NUMBER_LEDS: usize = 36;

pub const ANGLE_BETWEEN_LEDS: f64 = 2.0 * f64::consts::PI / NUMBER_LEDS as f64;

pub fn index_to_angle(index: usize) -> f64 {
    within_2pi(25.0_f64.to_radians() - ANGLE_BETWEEN_LEDS * index as f64)
}

pub fn angle_to_indexf(angle: f64) -> f64 {
    (36.0 + (25.0_f64.to_radians() - angle) / ANGLE_BETWEEN_LEDS) % 36.0
}

/// Implementation of actually lighting the Horizon LEDs given the bit pattern in `data`.
/// For example, an Impl for the actual PCB could be done with GPIOs or perhaps SPI using
/// the given bytes in the update function's `data` input.
pub trait HorizonLEDs {
    /// Update the Horizon LEDs given the bits in `data`. Each bit index corresponds to
    /// and LED. Nominally, the first byte corresponds to the first 8 LEDs starting
    /// at 0 degrees, going in an easterly direction. The higher order bits correspond
    /// to the lower LED number: for example bit index 7 of the first byte maps to
    /// the first LED, and bit 0 of the first byte maps to the 8th LED. Lastly bit 4
    /// of byte index 4 (the 5th byte) is the last LED.
    ///
    /// A 1 corresponds to the LED being ON, and a 0 is OFF.
    fn write(&mut self, data: &[u8]);

    fn update(&mut self);

    fn reset(&mut self);
}

pub struct DirectHorizonControl<CLK, D, FL, RS> {
    clock: CLK,
    data: D,
    flush: FL,
    reset: RS,
}

impl<CLK: OutputPin, D: OutputPin, FL: OutputPin, RS: OutputPin>
    DirectHorizonControl<CLK, D, FL, RS>
{
    pub fn new(clock: CLK, data: D, flush: FL, reset: RS) -> Self {
        DirectHorizonControl {
            clock,
            data,
            flush,
            reset,
        }
    }

    /// We'd like the bits to line up so that byte 0 lines up with the first 8 LEDs,
    /// with each LED going in reverse bit order (diode 1 corresponds to bit 7, and
    /// diode 8 corresponds with bit 0)
    ///
    /// To do this we need to send the sequence bytes in reverse, as the bits are being
    /// placed in essentially a queue in the shift registers
    pub fn send_sequence(&mut self, seq: &[u8]) {
        for (i, b) in seq.iter().rev().enumerate() {
            let x = if i == 0 { 4 } else { 0 };
            for bi in x..8 {
                self.clock_bit(*b & 1 << bi != 0);
            }
        }
    }

    pub fn write(&mut self, seq: &[u8]) {
        self.send_sequence(seq);
        self.update();
    }

    ///
    pub fn on_from_to(&mut self, start: usize, end: usize) {
        let mut s = [0; 5];
        let mut d = start;
        loop {
            let b = d / 8;
            let bi = 7 - d % 8;
            s[b] |= 1 << bi;
            if d == end {
                break;
            }
            d = (d + 1) % NUMBER_LEDS;
        }
        self.send_sequence(&s);
        self.update();
    }

    pub fn clock_bit(&mut self, bit: bool) {
        let _ = self.data.set_state(PinState::from(bit));
        let _ = self.clock.set_high();
        let _ = self.clock.set_low();
    }

    pub fn update(&mut self) {
        let _ = self.flush.set_high();
        let _ = self.flush.set_low();
    }

    pub fn reset(&mut self) {
        let _ = self.reset.set_low();
        let _ = self.reset.set_high();
    }
}

/// Interface to interacting with Horizon LEDs using angles
pub struct HorizonDisplay<CLK, D, FL, RS> {
    horizon: DirectHorizonControl<CLK, D, FL, RS>,
}

impl<CLK, D, FL, RS> HorizonDisplay<CLK, D, FL, RS>
where
    CLK: OutputPin,
    D: OutputPin,
    FL: OutputPin,
    RS: OutputPin,
{
    pub fn new(horizon: DirectHorizonControl<CLK, D, FL, RS>) -> HorizonDisplay<CLK, D, FL, RS> {
        HorizonDisplay { horizon }
    }

    /// Light up all LEDs between the `start` angle and sweeping through the `delta` angle.
    /// Positive angles sweep through going clockwise.
    pub fn light_delta(&mut self, start: f64, delta: f64) {
        let amount = (delta / ANGLE_BETWEEN_LEDS) as usize;
        let start_index = (start / ANGLE_BETWEEN_LEDS) as usize;
        let end_index = start_index + amount;
        self.horizon.on_from_to(start_index, end_index);
    }

    /// Light up all LEDs between the `start` angle and sweeping through to the `end` angle going counter clockwise
    /// LED angle = 5 + index * 10
    /// index = (angle - 5)/10
    pub fn light_start_end(&mut self, start: f64, end: f64) {
        let start_index = angle_to_indexf(start).ceil() as usize % NUMBER_LEDS;
        let end_index = angle_to_indexf(end) as isize % NUMBER_LEDS as isize;
        // Distance from start_index to next integer
        // First, we're making sure the subtraction is within the 36 LEDs range.
        // Then, if the original angles were in between two indexes then the start and
        // end will pass each other, start will go to past end, and end will sink
        // to before start when we match the float angle to integer indexes. This makes
        // the difference -1, which in modulo is one less than the size.
        // if (end_index - start_index + NUMBER_LEDS as isize) % NUMBER_LEDS as isize == NUMBER_LEDS as isize - 1  {
        //     self.horizon.write(&[0, 0, 0, 0, 0]);
        // } else {
        // }
        self.horizon.on_from_to(start_index, end_index as usize);
    }

    pub fn all_off(&mut self) {
        self.horizon.reset();
        self.horizon.update();
    }

    pub fn all_on(&mut self) {
        self.light_delta(0.0, 2.0 * f64::consts::PI);
    }
}

const RED: RGB = RGB::new(36, 0, 0);
const GREEN: RGB = RGB::new(0, 32, 0);
const BLUE: RGB = RGB::new(0, 0, 32);
const YELLOW: RGB = RGB::new(24, 18, 0);
const PURPLE: RGB = RGB::new(24, 0, 24);
const ORANGE: RGB = RGB::new(32, 10, 0);
#[allow(unused)]
const CYAN: RGB = RGB::new(0, 24, 24);
const TEAL: RGB = RGB::new(0, 24, 12);
#[allow(unused)]
const BRIGHT_YELLOW: RGB = RGB::new(24, 24, 6);
const YELLOW_GREEN: RGB = RGB::new(16, 24, 0);
const LOW_WHITE: RGB = RGB::new(20, 16, 24);
const BLUE_GREEN: RGB = RGB::new(0, 12, 24);

pub const MOON: RGB = LOW_WHITE; // RGB::new(160, 160, 160);
pub const MERCURY: RGB = GREEN; // RGB::new(0, 224, 32);
pub const VENUS: RGB = PURPLE; // RGB::new(160, 32, 128);
pub const SUN: RGB = YELLOW; // RGB::new(192, 160, 96);
pub const MARS: RGB = RED; // RGB::new(240, 0, 0);
pub const JUPITER: RGB = ORANGE; // RGB::new(192, 128, 0);
pub const SATURN: RGB = YELLOW_GREEN; // RGB::new(160, 160, 0);
pub const URANUS: RGB = BLUE; // RGB::new(32, 96, 224);
pub const NEPTUNE: RGB = TEAL; // RGB::new(32, 160, 128);
pub const EARTH: RGB = BLUE_GREEN; // RGB::new(0, 96, 192);
pub const BLACK: RGB = RGB::new(0, 0, 0);

#[derive(Debug, Clone, Copy)]
pub struct Planet {
    pub pos: (u8, u8),
    pub t: isize,
    pub t_max: Option<isize>,
    pub color: RGB,
    pub base: RGB,
}

pub struct Planets<SPI> {
    pub rgb_write: RGBWrite<SPI>,
    pub moon: Planet,
    pub mercury: Planet,
    pub venus: Planet,
    pub sun: Planet,
    pub mars: Planet,
    pub jupiter: Planet,
    pub saturn: Planet,
    pub uranus: Planet,
    pub neptune: Planet,
}

impl<SPI> Planets<SPI> {
    pub fn new(write: RGBWrite<SPI>) -> Planets<SPI> {
        Planets {
            rgb_write: write,
            moon: Planet {
                pos: (0, 0),
                t: 0,
                t_max: None,
                color: MOON,
                base: MOON,
            },
            mercury: Planet {
                pos: (0, 0),
                t: 0,
                t_max: None,
                color: MERCURY,
                base: MERCURY,
            },
            venus: Planet {
                pos: (0, 0),
                t: 0,
                t_max: None,
                color: VENUS,
                base: MERCURY,
            },
            sun: Planet {
                pos: (0, 0),
                t: 0,
                t_max: None,
                color: SUN,
                base: SUN,
            },
            mars: Planet {
                pos: (0, 0),
                t: 0,
                t_max: None,
                color: MARS,
                base: MARS,
            },
            jupiter: Planet {
                pos: (0, 0),
                t: 0,
                t_max: None,
                color: JUPITER,
                base: JUPITER,
            },
            saturn: Planet {
                pos: (0, 0),
                t: 0,
                t_max: None,
                color: SATURN,
                base: SATURN,
            },
            uranus: Planet {
                pos: (0, 0),
                t: 0,
                t_max: None,
                color: URANUS,
                base: URANUS,
            },
            neptune: Planet {
                pos: (0, 0),
                t: 0,
                t_max: None,
                color: NEPTUNE,
                base: NEPTUNE,
            },
        }
    }

    pub fn slice(&self) -> [&Planet; 9] {
        [
            &self.moon,
            &self.mercury,
            &self.venus,
            &self.sun,
            &self.mars,
            &self.jupiter,
            &self.saturn,
            &self.uranus,
            &self.neptune,
        ]
    }

    pub fn dupes(&mut self) -> Vec<Vec<&mut Planet>> {
        let mut m = alloc::collections::BTreeMap::new();
        for p in self.slice_mut() {
            let pos = p.pos;
            if m.contains_key(&pos) {
                m.entry(p.pos)
                    .and_modify(|x: &mut Vec<&mut Planet>| x.push(p));
            } else {
                m.insert(p.pos, vec![p]);
            }
        }
        m.into_iter()
            .filter(|(_, v)| v.len() > 1)
            .map(|(_, v)| v)
            .collect()
    }

    pub fn slice_mut(&mut self) -> [&mut Planet; 9] {
        [
            &mut self.moon,
            &mut self.mercury,
            &mut self.venus,
            &mut self.sun,
            &mut self.mars,
            &mut self.jupiter,
            &mut self.saturn,
            &mut self.uranus,
            &mut self.neptune,
        ]
    }

    pub fn sequence(&self) -> [RGB; 85] {
        let mut seq = [RGB::new(0, 0, 0); 85];
        for s in 0..12 {
            for r in 0..7 {
                let i = 7 * s + r;
                let c = if (s, r) == self.moon.pos {
                    self.moon.color
                } else if (s, r) == self.mercury.pos {
                    self.mercury.color
                } else if (s, r) == self.venus.pos {
                    self.venus.color
                } else if (s, r) == self.sun.pos {
                    self.sun.color
                } else if (s, r) == self.mars.pos {
                    self.mars.color
                } else if (s, r) == self.jupiter.pos {
                    self.jupiter.color
                } else if (s, r) == self.saturn.pos {
                    self.saturn.color
                } else if (s, r) == self.uranus.pos {
                    self.uranus.color
                } else if (s, r) == self.neptune.pos {
                    self.neptune.color
                } else {
                    BLACK
                };
                seq[i as usize] = c;
            }
        }
        seq[84] = EARTH;
        seq
    }

    fn near_new_sign(&self, dist: f64, diameter: f64, lon: f64) -> Option<isize> {
        const SIGN_ANGLE: f64 = TWO_PI / 12.0;
        let a = angular_diameter(dist, diameter);
        let an = lon - a / 2.0;
        

        if SIGN_ANGLE - an.rem_euclid(SIGN_ANGLE) < a {
            // Find out what percent of the angular diameter we have left
            let p = (SIGN_ANGLE - an.rem_euclid(SIGN_ANGLE)) / a;
            Some(((512.0 * p).max(8.0)) as isize)
        } else {
            None
        }
    }

    pub fn update_date(&mut self, jd: f64) -> alloc::vec::Vec<(&str, f64, f64)> {
        let mut v = alloc::vec::Vec::new();

        let (EclPoint { long: lon, lat: _ }, dist) = moon(jd);
        v.push(("moon", lon, dist));
        self.moon.t_max = self.near_new_sign(dist, diameter::MOON, lon);
        self.moon.pos = position(lon, dist);

        let (EclPoint { long: lon, lat: _ }, dist) = sun(jd);
        v.push(("sun", lon, dist));
        self.sun.t_max = self.near_new_sign(dist, diameter::SUN, lon);
        self.sun.pos = position(lon, dist);

        let (EclPoint { long: lon, lat: _ }, dist) = mercury(jd);
        v.push(("mercury", lon, dist));
        self.mercury.t_max = self.near_new_sign(dist, diameter::MERCURY, lon);
        self.mercury.pos = position(lon, dist);

        let (EclPoint { long: lon, lat: _ }, dist) = venus(jd);
        v.push(("venus", lon, dist));
        self.venus.t_max = self.near_new_sign(dist, diameter::VENUS, lon);
        self.venus.pos = position(lon, dist);

        let (EclPoint { long: lon, lat: _ }, dist) = mars(jd);
        v.push(("mars", lon, dist));
        self.mars.t_max = self.near_new_sign(dist, diameter::MARS, lon);
        self.mars.pos = position(lon, dist);

        let (EclPoint { long: lon, lat: _ }, dist) = jupiter(jd);
        v.push(("jupiter", lon, dist));
        self.jupiter.t_max = self.near_new_sign(dist, diameter::JUPITER, lon);
        self.jupiter.pos = position(lon, dist);

        let (EclPoint { long: lon, lat: _ }, dist) = saturn(jd);
        v.push(("saturn", lon, dist));
        self.saturn.t_max = self.near_new_sign(dist, diameter::SATURN, lon);
        self.saturn.pos = position(lon, dist);

        let (EclPoint { long: lon, lat: _ }, dist) = uranus(jd);
        v.push(("uranus", lon, dist));
        self.uranus.t_max = self.near_new_sign(dist, diameter::URANUS, lon);
        self.uranus.pos = position(lon, dist);

        let (EclPoint { long: lon, lat: _ }, dist) = neptune(jd);
        v.push(("neptune", lon, dist));
        self.neptune.t_max = self.near_new_sign(dist, diameter::NEPTUNE, lon);
        self.neptune.pos = position(lon, dist);

        v
    }

    pub fn release(self) -> RGBWrite<SPI> {
        self.rgb_write
    }
}

// struct PlanetIter<'a, SPI> {
//     planet: &'a Planet,
//     planets: &'a Planets<SPI>
// }

// impl<'a, SPI> Iterator for PlanetIter<'_, SPI> {
//     type Item = Planet;

//     fn next(&mut self) -> Option<Self::Item> {
//         if self.planet == self.planets.moon {

//         }
//         None
//     }
// }

impl<SPI> Debug for Planets<SPI> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "moon: {:?}", self.moon.pos)?;
        writeln!(f, "mercury: {:?}", self.mercury.pos)?;
        writeln!(f, "venus: {:?}", self.venus.pos)?;
        writeln!(f, "sun: {:?}", self.sun.pos)?;
        writeln!(f, "mars: {:?}", self.mars.pos)?;
        writeln!(f, "jupiter: {:?}", self.jupiter.pos)?;
        writeln!(f, "saturn: {:?}", self.saturn.pos)?;
        writeln!(f, "uranus: {:?}", self.uranus.pos)?;
        writeln!(f, "neptune: {:?}", self.neptune.pos)?;
        Ok(())
    }
}

impl<SPI: SpiDevice> Planets<SPI> {
    pub fn update(&mut self) {
        let s = self.sequence();
        self.rgb_write.write(&s);
    }

    pub fn clear(&mut self) {
        let s = [(0, 0, 0).into(); 85];
        self.rgb_write.write(&s);
    }
}

fn distance(r: u8) -> Interval {
    match r {
        0 => (0.001, 0.01).into(),
        1 => (0.2, 0.4).into(),
        2 => (0.4, 1.1).into(),
        3 => (1.1, 2.7).into(),
        4 => (2.7, 8.1).into(),
        5 => (8.1, 16.0).into(),
        6 => (16.0, 32.0).into(),
        _ => (0.0, 0.0).into(),
    }
}

pub fn position(lon: f64, dist: f64) -> (u8, u8) {
    let lon = within_2pi(lon);
    let s = (12 - ((lon / 30.0_f64.to_radians()) as u8)) % 12;
    let mut r: u8 = 6;
    for i in 0..7 {
        let dr = distance(i);
        if dr.includes(dist) {
            r = i;
            break;
        }
    }
    (s, 6 - r)
}

/// v0 + t * (v1 - v0);
pub fn lerp_int(v0: isize, v1: isize, t: isize, max_t: isize) -> isize {
    let t = t.clamp(0, max_t);
    v0 + t * (v1 - v0) / max_t
}

pub fn lerp_rgb(c0: RGB, c1: RGB, t: isize, max_t: isize) -> RGB {
    let r = lerp_int(c0.r as isize, c1.r as isize, t, max_t);
    let g = lerp_int(c0.g as isize, c1.g as isize, t, max_t);
    let b = lerp_int(c0.b as isize, c1.b as isize, t, max_t);
    RGB::new(r as u8, g as u8, b as u8)
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_name() {}
}
