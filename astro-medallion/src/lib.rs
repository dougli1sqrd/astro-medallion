#![no_std]

extern crate alloc;

use core::num::NonZero;

use alloc::vec::Vec;
use astro_medallion_lib::astro::time::Month;
use chrono::{Datelike, NaiveDate, NaiveTime, Timelike};
use embedded_hal::{
    digital::OutputPin,
    spi::{Error, ErrorKind, ErrorType, Operation, SpiBus, SpiDevice},
};
use esp_hal::{spi::master::Spi, Blocking};
use esp_storage::FlashStorage;
use embedded_storage::{ReadStorage, Storage};
use log::{info, log};
use serde::{Deserialize, Serialize};
use esp_hal::prelude::*;

pub mod serial;
pub mod timing;
pub mod cmd;

pub const CONFIG_OFFSET: u32 = 0x28_0000;


pub fn int_to_month(num: u32, default: Month) -> Month {
    match num {
        1 => Month::Jan,
        2 => Month::Feb,
        3 => Month::Mar,
        4 => Month::Apr,
        5 => Month::May,
        6 => Month::June,
        7 => Month::July,
        8 => Month::Aug,
        9 => Month::Sept,
        10 => Month::Oct,
        11 => Month::Nov,
        12 => Month::Dec,
        _ => default
    }
}

pub fn read_config(storage: &mut FlashStorage) -> astro_medallion_lib::Config {
    // Read number of stored bytes
    // If 0 this is the first start, and write a default config. Store the length and the default config.
    // Read the config out and return
    info!("Read config");
    let mut stored = [0; 4];
    storage.read(CONFIG_OFFSET, &mut stored).unwrap();
    info!("Read length as {:?}", stored);
    let length = u32::from_le_bytes(stored);
    if length == 0 || length == u32::MAX {
        // First time ever starting
        info!("First Read");
        let config = astro_medallion_lib::Config::default();

        let x = postcard::to_allocvec(&config).unwrap();
        storage
            .write(CONFIG_OFFSET, &x.len().to_le_bytes())
            .unwrap();
        storage.write(CONFIG_OFFSET + 4, x.as_slice()).unwrap();
        return config;
    }
    let mut rx = Vec::new();
    rx.resize_with(length as usize, || 0);
    storage.read(CONFIG_OFFSET + 4, &mut rx).unwrap();
    postcard::from_bytes::<astro_medallion_lib::Config>(&rx).unwrap()
}

pub fn write_config(storage: &mut FlashStorage, config: &astro_medallion_lib::Config) {
    info!("Write config");
    let x = postcard::to_allocvec(&config).unwrap();
    storage
        .write(CONFIG_OFFSET, &x.len().to_le_bytes())
        .unwrap();
    storage.write(CONFIG_OFFSET + 4, &x).unwrap();
}

#[derive(Debug)]
pub struct MyError(ErrorKind);

impl Error for MyError {
    fn kind(&self) -> ErrorKind {
        self.0
    }
}

pub struct SpiFesOnlyWrite<'d>(pub Spi<'d, Blocking>);

impl ErrorType for SpiFesOnlyWrite<'_> {
    type Error = MyError;
}

impl SpiDevice for SpiFesOnlyWrite<'_> {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        for x in operations {
            match x {
                Operation::Write(d) => {
                    let _ = self.0.write(d);
                }
                _ => unreachable!(),
            }
        }
        Ok(())
    }
}

pub struct SoftSPIWrite<CLK, D> {
    clk: CLK,
    data: D,
}

impl<CLK, D> SoftSPIWrite<CLK, D> {
    pub fn new(clk: CLK, data: D) -> SoftSPIWrite<CLK, D> {
        SoftSPIWrite { clk, data }
    }
}

impl<CLK, D> ErrorType for SoftSPIWrite<CLK, D> {
    type Error = MyError;
}

impl<CLK: OutputPin, D: OutputPin> SpiDevice for SoftSPIWrite<CLK, D> {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        for x in operations {
            match x {
                Operation::Write(data) => {
                    for d in data.iter().rev() {
                        for b in 0..8 {
                            let i = b;
                            let _ = self.data.set_state((d & (i << 1) != 0).into());
                            let _ = self.clk.set_high();
                            let _ = self.clk.set_low();
                        }
                    }
                }
                _ => unimplemented!(),
            }
        }
        Ok(())
    }
}


// Read storages for Config information
// Create config variable, which will be used throughout which has the relevant info
// If Config is not fully defined, do some default behavior and periodically re-check storage
//  and attempt to rebuild the config
// If Config is fully defined, proceed in the main loop displaying astronomy info
