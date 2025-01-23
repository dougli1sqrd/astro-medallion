use embedded_hal::spi::SpiDevice;


const ZERO_SYMBOL: u8 = 0b11000000;
const ONE_SYMBOL: u8 = 0b11111100;

#[derive(Debug, Clone, Copy)]
pub struct RGB {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl RGB {
    pub const fn new(r: u8, g: u8, b: u8) -> RGB {
        RGB { r, g, b }
    }

    pub fn bytes(&self) -> [u8; 3*8] {
        let mut x = [0; 3*8];
        let start = 0;
        for g in 0..8 {
            let i = 7 - g;
            if self.g & (1 << i) == 0 {
                x[start + g] = ZERO_SYMBOL;
            } else {
                x[start + g] = ONE_SYMBOL;
            }
        }
        let start = 8;
        for r in 0..8 {
            let i = 7 - r;
            if self.r & (1 << i) == 0 {
                x[start + r] = ZERO_SYMBOL;
            } else {
                x[start + r] = ONE_SYMBOL;
            }
        }
        let start = 16;
        for b in 0..8 {
            let i = 7 - b;
            if self.b & (1 << i) == 0 {
                x[start + b] = ZERO_SYMBOL;
            } else {
                x[start + b] = ONE_SYMBOL;
            }
        }
        x
    }
}

impl From<(u8, u8, u8)> for RGB {
    fn from(value: (u8, u8, u8)) -> Self {
        let (r, g, b) = value;
        RGB { r, g, b }
    }
}

pub struct RGBWrite<SPI> {
    spi: SPI,
}

impl<SPI> RGBWrite<SPI> {

    pub fn new(spi: SPI) -> RGBWrite<SPI> {
        RGBWrite { spi }
    }

    pub fn release(self) -> SPI {
        self.spi
    }
}

impl<SPI: SpiDevice> RGBWrite<SPI> {

    pub fn write(&mut self, data: &[RGB]) {
        for d in data {
            let _ = self.spi.write(&d.bytes());
        }
    }

    pub fn reset(&mut self) {

        let low = [0; 200];
        let _ = self.spi.write(&low);
    }
}


