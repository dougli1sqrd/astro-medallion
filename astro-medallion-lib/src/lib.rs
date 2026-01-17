#![cfg_attr(not(test), no_std)]

extern crate alloc;

use core::f64;

pub mod astronomy;
pub mod display;
pub mod rgb;
pub mod rtc;
pub mod input;
pub use astro;
pub use nom;
use chrono::{Datelike, NaiveDate, NaiveTime, Timelike};
use serde::{Deserialize, Serialize};
use alloc::vec::Vec;
use alloc::vec;
use alloc::string::ToString;

use crate::input::{Command, Value};

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

#[derive(Serialize, Deserialize, Debug)]
pub struct Config {
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub date: Option<NaiveDate>,
    pub time: Option<NaiveTime>,
    pub tz: i32,
}

impl Config {
    pub fn well_defined(&self) -> bool {
        self.lat.is_some() && self.lon.is_some() && self.date.is_some() && self.time.is_some()
    }

    pub fn commands(&self) -> Vec<Command> {
        if self.well_defined() {
            let date = self.date.unwrap();
            let time = self.time.unwrap();
            // We can unwrap because we know that all fields are Some()
            return vec![
                Command::KeyValue { key: "lat".to_string(), value: Value::Float(self.lat.unwrap()) },
                Command::KeyValue { key: "lon".to_string(), value: Value::Float(self.lon.unwrap()) },
                Command::KeyValue { key: "date".to_string(), value: Value::Date { year: date.year(), m: date.month0() + 1, d: date.day0() + 1 } },
                Command::KeyValue { key: "time".to_string(), value: Value::Time { hour: time.hour(), min: time.minute(), sec: time.second() } },
                Command::KeyValue { key: "tz".to_string(), value: Value::Int(self.tz) },
            ]
        }
        vec![]
    }
}

impl Default for Config {
    fn default() -> Self {
        Config { lat: None, lon: None, date: None, time: None, tz: -7 }
    }
}



#[cfg(test)]
mod tests {

}
