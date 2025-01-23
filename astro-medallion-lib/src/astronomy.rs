use core::f64;

use astro::{
    coords::{dec_frm_hz, hr_angl_frm_hz, EclPoint, EqPoint},
    ecl_frm_eq,
    ecliptic::mn_oblq_IAU,
    planet::geocent_geomet_ecl_coords,
    time::{mn_sidr, Month},
};
use core_maths::CoreFloat;
use vsop87::{vsop87b, SphericalCoordinates};

const TWO_PI: f64 = 2.0 * f64::consts::PI;

pub fn within_2pi(theta: f64) -> f64 {
    (theta + TWO_PI) % TWO_PI
}

pub fn zenith(jd: f64, lon: f64, lat: f64) -> EclPoint {
    let tilt = mn_oblq_IAU(jd);
    let gst = mn_sidr(jd);
    let lst = gst + lon;

    let z = eq_from_hz(90.0_f64.to_radians(), 90.0_f64.to_radians(), lst, lat);
    let (lon, lat) = ecl_frm_eq!(z.asc, z.dec, tilt);

    EclPoint { long: within_2pi(lon), lat }
}

pub fn east_horizon(jd: f64, lon: f64, lat: f64) -> EclPoint {
    let tilt = mn_oblq_IAU(jd);
    let gst = mn_sidr(jd);
    let lst = gst + lon;

    let east = eq_from_hz(90.0_f64.to_radians(), 0.0_f64.to_radians(), lst, lat);
    let (lon, lat) = ecl_frm_eq!(east.asc, east.dec, tilt);

    EclPoint { long: within_2pi(lon), lat }
}

pub fn west_horizon(jd: f64, lon: f64, lat: f64) -> EclPoint {
    let tilt = mn_oblq_IAU(jd);
    let gst = mn_sidr(jd);
    let lst = gst + lon;

    let west = eq_from_hz(270.0_f64.to_radians(), 0.0_f64.to_radians(), lst, lat);
    let (lon, lat) = ecl_frm_eq!(west.asc, west.dec, tilt);

    EclPoint { long: within_2pi(lon), lat }
}

pub fn horizon_longitudes(jd: f64, lon: f64, lat: f64) -> (f64, f64) {
    let EclPoint { long: west, lat: _ } = west_horizon(jd, lon, lat);
    let EclPoint { long: east, lat: _ } = east_horizon(jd, lon, lat);
    (east, west)
}

pub fn east_horizon_alt(jd: f64, lon: f64, lat: f64) -> f64 {
    let tilt = mn_oblq_IAU(jd);
    let gst = mn_sidr(jd);
    let lst = gst + lon;

    let x = lst.sin() * tilt.cos() + lat.tan() * tilt.sin();
    let asc = (-lst.cos() / x).atan();
    let asc = if x < 0.0 {
        asc + f64::consts::PI
    } else {
        asc + TWO_PI
    };
    if asc < f64::consts::PI {
        asc + f64::consts::PI
    } else {
        asc - f64::consts::PI
    }
}

/// Given RA α and declination δ, we have
/// Local Hour Angle H = LST - RA, in hours;
/// φ is the latitude
/// azimuth A and altitude a
/// Local Sideral Time t
///
/// sin(δ) = sin(a)sin(φ) + cos(a) cos(φ) cos(A)
/// sin(H) = - sin(A) cos(a) / cos(δ)
/// α = t – H
pub fn eq_from_hz(az: f64, alt: f64, lst: f64, lat: f64) -> EqPoint {
    // let dec = (alt.sin()*lat.sin() + alt.cos()*lat.cos()*az.cos()).asin();
    // let lh = (-az.sin()*alt.cos() / dec.cos()).asin();
    // EqPoint { asc: lst - lh, dec }
    let dec = dec_frm_hz(az, alt, lat);
    let ha = hr_angl_frm_hz(az, alt, lat);
    return EqPoint { asc: lst - ha, dec };
}

pub fn jd(year: i16, month: Month, day: u8, hr: u8, min: u8, sec: f64, time_zone: f64) -> f64 {
    let day_start = astro::time::DayOfMonth {
        day,
        hr,
        min,
        sec,
        time_zone,
    };
    let date = astro::time::Date {
        year,
        month,
        decimal_day: astro::time::decimal_day(&day_start),
        cal_type: astro::time::CalType::Gregorian,
    };
    let julian_day = astro::time::julian_day(&date);
    let delta_t = astro::time::delta_t(date.year.into(), date.month as u8);

    let jd = astro::time::julian_ephemeris_day(julian_day, delta_t);
    jd
}

fn convert_to_geo(jd: f64, pos: SphericalCoordinates) -> (EclPoint, f64) {
    let earthpos = vsop87b::earth(jd);
    let (lon, lat, d, _) = geocent_geomet_ecl_coords(
        earthpos.longitude(),
        earthpos.latitude(),
        earthpos.distance(),
        pos.longitude(),
        pos.latitude(),
        pos.distance(),
    );

    (
        EclPoint {
            long: within_2pi(lon),
            lat: lat,
        },
        d,
    )
}

const AUKM: f64 = 1.496E+8;

pub mod diameter {
    pub const MOON: f64 = 3_475.0;
    pub const MERCURY: f64 = 4_879.0;
    pub const VENUS: f64 = 12_104.0;
    pub const SUN: f64 = 1_391_400.0;
    pub const MARS: f64 = 6_792.0;
    pub const JUPITER: f64 = 142_984.0;
    pub const SATURN: f64 = 120_536.0;
    pub const URANUS: f64 = 51_118.0;
    pub const NEPTUNE: f64 = 49_528.0;
}

/// Distance in AU, diameter in km
pub fn angular_diameter(distance: f64, diameter: f64) -> f64 {
    diameter / (distance * AUKM)
}

pub fn mars(jd: f64) -> (EclPoint, f64) {
    let pos = vsop87b::mars(jd);
    convert_to_geo(jd, pos)
}

pub fn moon(jd: f64) -> (EclPoint, f64) {
    let (p, d) = astro::lunar::geocent_ecl_pos(jd);
    (EclPoint { long: within_2pi(p.long), lat: p.lat }, d / AUKM )
}

pub fn sun(jd: f64) -> (EclPoint, f64) {
    let (EclPoint { long: lon, lat }, d) = astro::sun::geocent_ecl_pos(jd);
    (EclPoint { long: within_2pi(lon), lat }, d)
}

pub fn mercury(jd: f64) -> (EclPoint, f64) {
    let pos = vsop87b::mercury(jd);
    convert_to_geo(jd, pos)
}

pub fn venus(jd: f64) -> (EclPoint, f64) {
    let pos = vsop87b::venus(jd);
    convert_to_geo(jd, pos)
}

pub fn jupiter(jd: f64) -> (EclPoint, f64) {
    let pos = vsop87b::jupiter(jd);
    convert_to_geo(jd, pos)
}

pub fn saturn(jd: f64) -> (EclPoint, f64) {
    let pos = vsop87b::saturn(jd);
    convert_to_geo(jd, pos)
}

pub fn uranus(jd: f64) -> (EclPoint, f64) {
    let pos = vsop87b::uranus(jd);
    convert_to_geo(jd, pos)
}

pub fn neptune(jd: f64) -> (EclPoint, f64) {
    let pos = vsop87b::neptune(jd);
    convert_to_geo(jd, pos)
}

pub trait Within2Pi {
    fn in2pi(&self) -> Self;
}

impl Within2Pi for f64 {
    fn in2pi(&self) -> Self {
        (self + 2.0 * f64::consts::PI) % (2.0 * f64::consts::PI)
    }
}
