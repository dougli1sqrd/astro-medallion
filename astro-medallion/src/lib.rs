#![no_std]

extern crate alloc;

use astro_medallion_lib::astro::time::Month;
use embedded_hal::{
    digital::OutputPin,
    spi::{Error, ErrorKind, ErrorType, Operation, SpiBus, SpiDevice},
};
use esp_hal::{spi::master::Spi, Blocking};

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