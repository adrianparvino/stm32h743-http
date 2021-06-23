use embedded_hal::timer::CountDown;
use stm32f4xx_hal as hal;
use hal::{stm32::TIM5, timer::Timer, prelude::*, i2c::*, block};

pub struct Hd44780I2c<I2C: Instance, PINS> {
    handle: I2c<I2C, PINS>,
    address: u8,
    backlight_enabled : bool,
}

pub trait Delay {
    fn delay_ms(&mut self, ms: u32);
    fn delay_us(&mut self, us: u32);
}

impl Delay for Timer<TIM5> {
    fn delay_ms(&mut self, ms: u32) {
        self.start((1_000/ms).hz());
        block!(self.wait()).ok();
    }

    fn delay_us(&mut self, us: u32) {
        self.start((1_000_000/us).hz());
        block!(self.wait()).ok();
    }
}

// pub fn probe_address<I2C: Instance, PINS>(handle: &mut I2c<I2C, PINS>) {
//     for addr in 0..=0xff {
//         if let Ok(_) = handle.write(addr, &[1]) {
//             cortex_m_semihosting::hprintln!("{}", addr);
//         }
//     }
// }

fn write_nibble<I2C: Instance, PINS>(handle: &mut I2c<I2C, PINS>, address: u8, value: u8, delay: &mut Timer<TIM5>) -> Result<(), Error> {
    handle.write(address, &[value |  0b00000100])?;
    delay.delay_us(1_u32);
    handle.write(address, &[value & !0b00000100])?;
    delay.delay_us(37_u32);

    Ok(())
}

impl<I2C: Instance, PINS> Hd44780I2c<I2C, PINS> {
    pub fn new(mut handle: I2c<I2C, PINS>, address: u8, backlight_enabled: bool, delay: &mut Timer<TIM5>) -> Result<Self, Error> {
        delay.delay_ms(200_u32);

        write_nibble(&mut handle, address, 0x30, delay)?;
        delay.delay_us(4100_u32);

        write_nibble(&mut handle, address, 0x30, delay)?;
        delay.delay_us(100_u32);

        write_nibble(&mut handle, address, 0x30, delay)?;
        delay.delay_us(100_u32);

        write_nibble(&mut handle, address, 0x20, delay)?;
        delay.delay_us(100_u32);

        Ok(Hd44780I2c {handle, address, backlight_enabled})
    }

    pub fn write_byte(&mut self, delay: &mut Timer<TIM5>, value: u8, as_data: bool) -> Result<(), Error> {
        let mut mask = 0u8;

        mask |= 0b00001000 * (self.backlight_enabled as u8);
        mask |= 0b00000001 * (as_data as u8);

        write_nibble(&mut self.handle, self.address,  value       & 0xF0 | mask, delay)?;
        write_nibble(&mut self.handle, self.address, (value << 4) & 0xF0 | mask, delay)?;

        Ok(())
    }
}
