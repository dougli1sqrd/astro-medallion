#![no_std]
#![no_main]

use core::cell::{RefCell};
use core::f64;
use core::ops::{AddAssign, Deref};

use alloc::format;
use astro_medallion::{int_to_month, SpiFesOnlyWrite};
use astro_medallion_lib::astro::coords::EclPoint;
use astro_medallion_lib::astro::time::Month;
use astro_medallion_lib::astronomy::{
    east_horizon_alt, horizon_longitudes, jd, within_2pi, zenith
};
use astro_medallion_lib::display::{
    self, lerp_rgb, position, DirectHorizonControl, HorizonDisplay, HorizonLEDs, Planets, BLACK, EARTH, JUPITER, MARS, MERCURY, MOON, NEPTUNE, SATURN, SUN, URANUS, VENUS
};
use astro_medallion_lib::rgb::{RGBWrite, RGB};
use chrono::TimeDelta;
use critical_section::Mutex;
use embedded_hal::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output};
use esp_hal::i2c::master::I2c;
use esp_hal::interrupt::Priority;
use esp_hal::peripheral::Peripheral;
use esp_hal::peripherals::{Interrupt, SPI2, TIMG0};
use esp_hal::riscv::asm::wfi;
use esp_hal::spi::master::{Config, Spi};
use esp_hal::spi::SpiMode;
use esp_hal::timer::timg::{Timer, TimerGroup, TimerX};
use esp_hal::uart::{self, Uart, UartRx, UartTx};
use esp_hal::{i2c, interrupt, peripheral, prelude::*, Blocking};
use esp_println::println;
use log::info;
use mcp794xx::ic::Mcp7940n;
use mcp794xx::interface::I2cInterface;
use mcp794xx::{DateTimeAccess, Datelike, Hours, Mcp794xx, NaiveDate, NaiveDateTime, NaiveTime, Timelike};
use nb::block;
use noline::builder::EditorBuilder;

extern crate alloc;

const TIME_SET: u8 = 0x54;
const DEFAULT_YMD: (i32, u32, u32) = (2025, 1, 17);
const DEFAULT_HMS: (u32, u32, u32) = (23, 0, 0);
const UPDATE_INTERVAL: u64 = 1000 * 60 * 20;
// const UPDATE_INTERVAL: u64 = 1000;
const WAKEUP_INTERVAL: u64 = 10;

static TIMER0: Mutex<RefCell<Option<Timer<TimerX<TIMG0>, Blocking>>>> = Mutex::new(RefCell::new(None));

static CURRENT_DURATION: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));


#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_println::logger::init_logger_from_env();

    esp_alloc::heap_allocator!(92 * 1024);

    println!();
    let uart0 = Uart::new_with_config(
        peripherals.UART0,
        uart::Config {
            ..Default::default()
        },
        peripherals.GPIO20,
        peripherals.GPIO21,
    )
    .unwrap();

    let scl = peripherals.GPIO2;
    let sda = peripherals.GPIO3;
    let i2c = I2c::new(
        peripherals.I2C0,
        i2c::master::Config {
            ..Default::default()
        },
    )
    .with_scl(scl)
    .with_sda(sda);
    let mut rtc = mcp794xx::Mcp794xx::new_mcp7940n(i2c);

    rtc.enable().unwrap();
    rtc.enable_backup_battery_power().unwrap();

    // Set the time if it's not been set yet
    if rtc.read_sram_byte(0x20).unwrap() != TIME_SET {
        // READ the first byte in SRAM, and look for the TIME_SET byte.
        // if it's not there it means we need to set the time
        rtc.disable().unwrap();
        // Wait for oscillator to stop
        while rtc.is_oscillator_running().unwrap() {}

        rtc.set_datetime(
            &NaiveDate::from_ymd_opt(DEFAULT_YMD.0, DEFAULT_YMD.1, DEFAULT_YMD.2)
                .unwrap()
                .and_hms_opt(DEFAULT_HMS.0, DEFAULT_HMS.1, DEFAULT_HMS.2)
                .unwrap(),
        ).unwrap();
        rtc.enable().unwrap();
        rtc.write_sram_byte(0x20, TIME_SET).unwrap();
    }


    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: Timer<esp_hal::timer::timg::TimerX<<esp_hal::peripherals::TIMG0 as Peripheral>::P>, Blocking> = timg0.timer0;
    timer0.set_interrupt_handler(timer_interrupt);

    interrupt::enable(Interrupt::TG0_T0_LEVEL, Priority::Priority1).unwrap();
    timer0.load_value(WAKEUP_INTERVAL.millis()).unwrap();
    timer0.start();
    timer0.listen();
    critical_section::with(|cs| {
        TIMER0.borrow(cs).replace(Some(timer0));
    });

    let gpio07 = peripherals.GPIO7;

    let config = Config {
        frequency: 6667.kHz(),
        mode: SpiMode::Mode0,
        ..Default::default()
    };
    let mut spi = Spi::new(peripherals.SPI2).with_mosi(gpio07);

    spi.apply_config(&config).unwrap();

    let rgb_spi = SpiFesOnlyWrite(spi);
    let rgbWrite = RGBWrite::new(rgb_spi);
    let mut planets = Planets::new(rgbWrite);

    let clock = Output::new(peripherals.GPIO0, Level::Low);
    let data = Output::new(peripherals.GPIO1, Level::Low);
    let flush = Output::new(peripherals.GPIO4, Level::Low);
    let reset = Output::new(peripherals.GPIO9, Level::High);

    let mut hc = DirectHorizonControl::new(clock, data, flush, reset);
    hc.reset();
    hc.update();

    let mut horizon = HorizonDisplay::new(hc);

    let lat = 37.77_f64.to_radians();
    let lon = -122.0_f64.to_radians();
    let mut i = 0;

    critical_section::with(|_| {
        update_display(&mut rtc, lat, lon, &mut horizon, &mut planets);
    });

    let mut lerp_t = 0;
    let max_lerp = 200;
    

    loop {
        // Read the current duration, and if we're greater than the update UPDATE_INTERVAL, 
        // then reset the duration and and read the real time clock and update as normal
        let update: bool = critical_section::with(|cs| {
            let mut d = CURRENT_DURATION.borrow(cs).borrow_mut();
            let update = *d >= UPDATE_INTERVAL;
            if update {
                *d = 0;
            } 
            update
        });
        if update {
            critical_section::with(|_| {
                update_display(&mut rtc, lat, lon, &mut horizon, &mut planets);
                // update_display_demo(&mut rtc, lat, lon, &mut horizon, &mut planets, time_delta);
                println!("Update {i}");
                i += 1;
                // let tt = 15 + 15*i + i*i*i / 6;
                // println!("Delta T minutes = {tt}, dt = {}", tt - time_delta);
                // time_delta = tt;
            });
        }
        for mut d in planets.dupes() {
            // println!("dupes = {:?}", d);
            for i in 0..d.len() {
                let n = (i + 1) % d.len();
                // If we have two planets overlapping, that takes precedent over a planet transiting
                d[n].t_max = None;
                if lerp_t < max_lerp {
                    d[n].color = lerp_rgb(d[n].base, d[i].base, lerp_t, max_lerp);
                } else {
                    d[n].color = lerp_rgb(d[i].base, d[n].base, lerp_t - max_lerp, max_lerp);
                }
            }
        }
        
        for p in planets.slice_mut() {
            if let Some(t_max) = p.t_max {
                p.t = (p.t + 1) % (2*t_max);
                // println!("t_max = {t_max}, t = {}", p.t);
                if p.t < t_max {
                    p.color = lerp_rgb(p.base, BLACK, p.t, t_max);
                } else {
                    p.color = lerp_rgb(BLACK, p.base, p.t - t_max, t_max);
                }
            }
        }
        let s = planets.sequence();
        planets.rgb_write.write(&s);

        lerp_t = (lerp_t + 1) % (2 * max_lerp);

        wfi();
    }

}

#[handler]
fn timer_interrupt() {
    critical_section::with(|cs| {
        if let Some(t) = TIMER0.borrow(cs).borrow().deref() {
            t.clear_interrupt();
            t.load_value(WAKEUP_INTERVAL.millis()).unwrap();
            t.start();
        }
        // Update the wakeup duration
        CURRENT_DURATION.borrow(cs).borrow_mut().add_assign(WAKEUP_INTERVAL);
    });
}

fn update_display(rtc: &mut Mcp794xx<I2cInterface<I2c<'_, Blocking>>, Mcp7940n>, lat: f64, lon: f64, horizon: &mut HorizonDisplay<Output<'_>, Output<'_>, Output<'_>, Output<'_>>, planets: &mut Planets<SpiFesOnlyWrite<'_>>) {
    let current_time = rtc.datetime().unwrap();
    // let current_time = NaiveDate::from_ymd_opt(2025, 4, 18).unwrap().and_time(NaiveTime::from_hms_opt(16, 45, 0).unwrap());
    println!("Reading time as {}", current_time);
    let jd = jd(current_time.year() as i16, int_to_month(current_time.month(), Month::Jan), current_time.day() as u8, current_time.hour() as u8, current_time.minute() as u8, current_time.second() as f64, -8.0);
    let (east, west) = horizon_longitudes(jd, lon, lat);
    let alt_east = east_horizon_alt(jd, lon, lat);
    let alt_west = within_2pi(alt_east + f64::consts::PI);
    let z = zenith(jd, lon, lat);
    println!("East: {}, Z: {}, West: {}", east.to_degrees(), z.long.to_degrees(), west.to_degrees());
    println!("AltEast: {}, AltWest: {}", alt_east.to_degrees(), alt_west.to_degrees());
    horizon.light_start_end(alt_east, alt_west);

    let _ = planets.update_date(jd);
    
    let s = planets.sequence();
    planets.rgb_write.reset();
    planets.rgb_write.write(&s);
}

fn update_display_demo(rtc: &mut Mcp794xx<I2cInterface<I2c<'_, Blocking>>, Mcp7940n>, lat: f64, lon: f64, horizon: &mut HorizonDisplay<Output<'_>, Output<'_>, Output<'_>, Output<'_>>, planets: &mut Planets<SpiFesOnlyWrite<'_>>, add_min: i64) {
    let current_time = rtc.datetime().unwrap().checked_add_signed(TimeDelta::minutes(add_min)).unwrap();
    // let current_time = NaiveDate::from_ymd_opt(2025, 4, 18).unwrap().and_time(NaiveTime::from_hms_opt(16, 45, 0).unwrap());
    println!("Reading time as {}", current_time);
    let jd = jd(current_time.year() as i16, int_to_month(current_time.month(), Month::Jan), current_time.day() as u8, current_time.hour() as u8, current_time.minute() as u8, current_time.second() as f64, -8.0);
    let (east, west) = horizon_longitudes(jd, lon, lat);
    let alt_east = east_horizon_alt(jd, lon, lat);
    let alt_west = within_2pi(alt_east + f64::consts::PI);
    let z = zenith(jd, lon, lat);
    println!("East: {}, Z: {}, West: {}", east.to_degrees(), z.long.to_degrees(), west.to_degrees());
    println!("AltEast: {}, AltWest: {}", alt_east.to_degrees(), alt_west.to_degrees());
    horizon.light_start_end(alt_east, alt_west);

    let _ = planets.update_date(jd);
    
    let s = planets.sequence();
    planets.rgb_write.reset();
    planets.rgb_write.write(&s);
}

fn print_pos(name: &str, pos: (EclPoint, f64)) -> alloc::string::String {
    let (p, d) = pos;
    format!(
        "{}: (Lon {}, Lat {}), distance: {}",
        name,
        p.long.to_degrees(),
        p.lat.to_degrees(),
        d
    )
}

struct UartWrapper<'d, M> {
    pub rx: UartRx<'d, M>,
    pub tx: UartTx<'d, M>,
}

impl<'a, M> From<Uart<'a, M>> for UartWrapper<'a, M>
where
    M: esp_hal::Mode,
{
    fn from(value: Uart<'a, M>) -> Self {
        let (rx, tx) = value.split();

        Self { rx, tx }
    }
}

#[derive(Debug)]
struct Error(esp_hal::uart::Error);

impl From<esp_hal::uart::Error> for Error {
    fn from(value: esp_hal::uart::Error) -> Self {
        Self(value)
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl<'a, M> embedded_io::ErrorType for UartWrapper<'a, M> {
    type Error = Error;
}

impl<'a, M: esp_hal::Mode> embedded_io::Read for UartWrapper<'a, M> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        buf[0] = block!(self.rx.read_byte())?;

        Ok(1 + self.rx.drain_fifo(&mut buf[1..]))
    }
}

impl<'a, M: esp_hal::Mode> embedded_io::Write for UartWrapper<'a, M> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write_bytes(buf).map_err(Self::Error::from)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        block!(self.tx.flush_tx()).map_err(Self::Error::from)
    }
}
