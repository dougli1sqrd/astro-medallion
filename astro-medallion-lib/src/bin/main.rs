use core::f64;

use astro::{
    angle::{hms_frm_deg, limit_to_360},
    coords::{self, EclPoint},
    ecl_frm_eq,
    ecliptic::mn_oblq_IAU,
    eq_frm_ecl,
    planet::geocent_geomet_ecl_coords,
    time::mn_sidr,
};
use astro_medallion_lib::astronomy::{east_horizon, eq_from_hz, west_horizon, zenith, Within2Pi};
use vsop87::vsop87b;

fn main() {
    println!("hello");

    let day_of_month = astro::time::DayOfMonth {
        day: 5,
        hr: 13,
        min: 28,
        sec: 4.0,
        time_zone: -8.0,
    };

    let date = astro::time::Date {
        year: 2025,
        month: astro::time::Month::Jan,
        decimal_day: astro::time::decimal_day(&day_of_month),
        cal_type: astro::time::CalType::Gregorian,
    };

    let julian_day = astro::time::julian_day(&date);

    // for higher accuracy in specifying the time of interest,
    // find the Julian Ephemeris day; this slightly differs from
    // the Julian day by ΔT, which is usually a few seconds. you
    // can get a reported value of it from the Astronomical
    // Almanac, or calculate it using the built-in function

    let delta_t = astro::time::delta_t(date.year.into(), date.month as u8);

    let julian_ephm_day = astro::time::julian_ephemeris_day(julian_day, delta_t);

    // let (sun_ecl_point, rad_vec_sun) = astro::sun::geocent_ecl_pos(julian_ephm_day);

    let tilt = astro::ecliptic::mn_oblq_IAU(julian_ephm_day);
    // let (asc, dec) = eq_frm_ecl!(sun_ecl_point.long, sun_ecl_point.lat, tilt);

    // println!(
    //     "Sun pos (equatorial): ({}, {})",
    //     asc * 180.0 / core::f64::consts::PI,
    //     dec * 180.0 / core::f64::consts::PI
    // );
    // println!("Sun distance: {rad_vec_sun}");

    // let (mars_pos, mars_r) =
    //     astro::planet::geocent_apprnt_ecl_coords(&astro::planet::Planet::Mars, julian_ephm_day);
    // let (mars_asc, mars_dec) = eq_frm_ecl!(mars_pos.long, mars_pos.lat, tilt);
    // println!(
    //     "Mars pos (equatorial): ({}, {})",
    //     mars_asc * 180.0 / core::f64::consts::PI,
    //     mars_dec * 180.0 / core::f64::consts::PI
    // );
    // println!("Mars distance: {mars_r}");

    let gsd = astro::time::mn_sidr(julian_ephm_day); //radians
    println!("gsd = {}", gsd);
    let local_long = -122.4;
    let local_sd = gsd + local_long * f64::consts::PI / 180.0;
    println!("local sd = {}", local_sd);
    let lat = 37.77_f64.to_radians();
    println!("obs_dec = {}", lat);

    let (ra, _) = eq_frm_ecl!(30.0 * f64::consts::PI / 180.0, 0.0, tilt);
    let ecl_long = coords::ecl_long_frm_eq(ra, 0.0, tilt);
    println!("ecl_long = {}", ecl_long * 180.0 / f64::consts::PI);
    println!("ra for 30 is {}", ra * 180.0 / f64::consts::PI);
    println!(
        "ra hh:mm:ss: {:?}",
        hms_frm_deg(ra * 180.0 / f64::consts::PI)
    );

    let day_start = astro::time::DayOfMonth {
        day: 5,
        hr: 0,
        min: 0,
        sec: 0.0,
        time_zone: -7.0,
    };
    let d = astro::time::decimal_day(&day_start);

    let date = astro::time::Date {
        year: 2025,
        month: astro::time::Month::Jan,
        decimal_day: d,
        cal_type: astro::time::CalType::Gregorian,
    };
    let julian_day = astro::time::julian_day(&date);
    let delta_t = astro::time::delta_t(date.year.into(), date.month as u8);

    let jd = astro::time::julian_ephemeris_day(julian_day, delta_t);
    let foo = zenith2(jd, -122.0_f64.to_radians(), 37.77_f64.to_radians());
    println!(
        "Zenith at midnight jan 5 (Lon {}, Lat {})",
        foo.long.to_degrees(),
        foo.lat.to_degrees()
    );
    // println!("free_eq (RA {}, DEC {})", foo_asc.to_degrees(), foo_dec.to_degrees());
    for a in 0..36 {
        let dec_day = astro::time::decimal_day(&day_start) + a as f64 * 40.0 / (60.0 * 24.0);

        let date = astro::time::Date {
            year: 2025,
            month: astro::time::Month::Jan,
            decimal_day: dec_day,
            cal_type: astro::time::CalType::Gregorian,
        };
        let julian_day = astro::time::julian_day(&date);
        let delta_t = astro::time::delta_t(date.year.into(), date.month as u8);

        let jd = astro::time::julian_ephemeris_day(julian_day, delta_t);

        let east = east_horizon(jd, local_long.to_radians(), lat);
        let z = zenith(jd, local_long.to_radians(), lat);
        let west = west_horizon(jd, local_long.to_radians(), lat);

        println!("Time: {:?} East: {}, Zenith: {}, West: {}", hms_from_decimal_day(dec_day, -8.0), east.long.in2pi().to_degrees(), z.long.in2pi().to_degrees(), west.long.in2pi().to_degrees());
    }
    println!("Experiment to go from ecliptic to equatorial coordinates and back again");

    let (asc, dec) = eq_frm_ecl!(30.0_f64.to_radians(), 45.0_f64.to_radians(), tilt);
    println!(
        "Lon/Lat (30, 45) in ecliptic is ({} RA, {} Dec)",
        asc.to_degrees(),
        dec.to_degrees()
    );

    let (lon, lat) = ecl_frm_eq!(asc, dec, tilt);
    println!(
        "RA/Dec ({}, {}) in equatorial is ({} lon, {} lat)",
        asc.to_degrees(),
        dec.to_degrees(),
        lon.to_degrees(),
        lat.to_degrees()
    );

    let z = zenith(
        julian_ephm_day,
        -122.4_f64.to_radians(),
        37.77_f64.to_radians(),
    );
    println!("Zenith longitude: {} degrees", z.long.in2pi().to_degrees());
    println!(
        "Zenith equitorial: {}, {}",
        eq_frm_ecl!(z.long, z.lat, tilt).0.to_degrees(),
        eq_frm_ecl!(z.long, z.lat, tilt).1.to_degrees()
    );

    let east_horizon_az = 90.0_f64.to_radians();
    let east_horizon = eq_from_hz(east_horizon_az, 0.0, local_sd, 37.77_f64.to_radians());
    println!(
        "East horizon position: (RA {}, Dec: {})",
        east_horizon.asc.to_degrees(),
        east_horizon.dec.to_degrees()
    );
    let (east_lon, east_lat) = ecl_frm_eq!(east_horizon.asc, east_horizon.dec, tilt);
    println!(
        "East horizon ecliptic: (Lat {}, Lon {})",
        east_lat.to_degrees(),
        east_lon.to_degrees()
    );

    let west_horizon_az = 270.0_f64.to_radians();
    let west_horizon = eq_from_hz(west_horizon_az, 0.0, local_sd, 37.77_f64.to_radians());
    println!(
        "West horizon position: (RA {}, Dec: {})",
        west_horizon.asc.to_degrees(),
        west_horizon.dec.to_degrees()
    );
    let (west_lon, west_lat) = ecl_frm_eq!(west_horizon.asc, west_horizon.dec, tilt);
    println!(
        "East horizon ecliptic: (Lat {}, Lon {})",
        west_lat.to_degrees(),
        limit_to_360(west_lon.to_degrees())
    );

    println!("VSOP87");
    let marspos = vsop87b::mars(jd);
    let earthpos = vsop87b::earth(jd);
    let (mlon, mlat, md, t) = geocent_geomet_ecl_coords(
        earthpos.longitude(),
        earthpos.latitude(),
        earthpos.distance(),
        marspos.longitude(),
        marspos.latitude(),
        marspos.distance(),
    );
    println!(
        "Helio Mars lon: {}, lat: {}, dist: {}",
        marspos.longitude().to_degrees(),
        marspos.latitude().to_degrees(),
        marspos.distance()
    );
    println!(
        "Geo Mars lon: {}, lat: {}, dist: {}, t: {}",
        mlon.to_degrees(),
        mlat.to_degrees(),
        md,
        t
    );
}

pub fn hms_from_decimal_day(decimal_day: f64, timezone: f64) -> (i32, i32, f64) {
    let fract_day = decimal_day.fract();
    let dec_hour = (fract_day * 24.0 + timezone + 24.0) % 24.0;
    let dec_minute = (dec_hour.fract() * 60.0 * 6000000.0).round() / 6000000.0;
    let dec_second = dec_minute.fract() * 60.0;
    (dec_hour as i32, dec_minute as i32, dec_second)
}

pub fn zenith2(jd: f64, lon: f64, lat: f64) -> EclPoint {
    let tilt = mn_oblq_IAU(jd);
    let gst = mn_sidr(jd);
    let lst = gst + lon;

    let z = eq_from_hz(90.0_f64.to_radians(), 90.0_f64.to_radians(), lst, lat);
    println!("z (RA {}, DEC {})", z.asc.to_degrees(), z.dec.to_degrees());
    let (lon, lat) = ecl_frm_eq!(z.asc, z.dec, tilt);

    EclPoint { long: lon, lat }
}
