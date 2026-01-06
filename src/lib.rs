// SPDX-FileCopyrightText: Copyright The arm-pl061 Contributors.
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![doc = include_str!("../README.md")]
#![deny(clippy::undocumented_unsafe_blocks)]
#![deny(unsafe_op_in_unsafe_fn)]

#[cfg(feature = "embedded-hal")]
mod embedded_hal;

use core::array;
pub use safe_mmio::{SharedMmioPointer, UniqueMmioPointer};
use safe_mmio::{
    field, field_shared,
    fields::{ReadPure, ReadPureWrite, WriteOnly},
    split_fields,
};
use thiserror::Error;
use zerocopy::{FromBytes, IntoBytes};

const PIN_COUNT: usize = 8;

/// PL061 register map
#[repr(C, align(4))]
#[derive(Clone, Eq, FromBytes, IntoBytes, PartialEq)]
pub struct PL061Registers {
    /// 0x000 - 0x3FC: Data Register region.
    /// Access is handled dynamically by calculating an address offset.
    gpiodata_region: [ReadPureWrite<u8>; 0x400],
    /// 0x400: Data Direction Register
    gpiodir: ReadPureWrite<u8>,
    _padding0: [u8; 3],
    /// 0x404: Interrupt Sense Register
    gpiois: ReadPureWrite<u8>,
    _padding1: [u8; 3],
    /// 0x408: Interrupt Both Edges Register
    gpioibe: ReadPureWrite<u8>,
    _padding2: [u8; 3],
    /// 0x40C: Interrupt Event Register
    gpioiev: ReadPureWrite<u8>,
    _padding3: [u8; 3],
    /// 0x410: Interrupt Mask Register
    gpioie: ReadPureWrite<u8>,
    _padding4: [u8; 3],
    /// 0x414: Raw Interrupt Status Register
    gpioris: ReadPure<u8>,
    _padding5: [u8; 3],
    /// 0x418: Masked Interrupt Status Register
    gpiomis: ReadPure<u8>,
    _padding6: [u8; 3],
    /// 0x41C: Interrupt Clear Register
    gpioic: WriteOnly<u8>,
    _padding7: [u8; 3],
    /// 0x420: Mode Control Select Register
    gpioafsel: ReadPureWrite<u8>,
    _padding8: [u8; 3],
    /// 0x424 - 0xFDF: Reserved
    reserved_424: [u32; 751],
    /// 0xFE0: UARTPeriphID0 Register
    gpioperiphid0: ReadPure<u32>,
    /// 0xFE4: UARTPeriphID1 Register
    gpioperiphid1: ReadPure<u32>,
    /// 0xFE8: UARTPeriphID2 Register
    gpioperiphid2: ReadPure<u32>,
    /// 0xFEC: UARTPeriphID3 Register
    gpioperiphid3: ReadPure<u32>,
}

/// PL061 GPIO error type
#[derive(Clone, Copy, Debug, Error, Eq, PartialEq)]
pub enum Error {
    #[error("Invalid pin id")]
    InvalidPinId,
}

/// PL061 GPIO implementation
pub struct PL061<'a> {
    regs: UniqueMmioPointer<'a, PL061Registers>,
}

impl<'a> PL061<'a> {
    /// Creates a new PL061 driver instance from a pointer to the peripheral's
    /// memory-mapped registers.
    pub fn new(regs: UniqueMmioPointer<'a, PL061Registers>) -> Self {
        Self { regs }
    }

    /// The pin's mask (e.g., 1 << id).
    fn mask(&self, pin_id: usize) -> Result<u8, Error> {
        if pin_id >= PIN_COUNT {
            return Err(Error::InvalidPinId);
        };
        Ok(1 << pin_id)
    }

    /// Configures the pin as an input.
    pub fn into_input(&mut self, pin_id: usize) -> Result<(), Error> {
        let mask = self.mask(pin_id)?;
        let val = field_shared!(self.regs, gpiodir).read();
        field!(self.regs, gpiodir).write(val & !mask);
        Ok(())
    }

    /// Configures the pin as an output.
    pub fn into_output(&mut self, pin_id: usize) -> Result<(), Error> {
        let mask = self.mask(pin_id)?;
        let mut gpiodir_ptr = field!(self.regs, gpiodir);
        let val = gpiodir_ptr.read();
        gpiodir_ptr.write(val | mask);
        Ok(())
    }

    // --- Interrupt Configuration ---

    /// Enables the interrupt for this pin.
    pub fn enable_interrupt(&mut self, pin_id: usize) -> Result<(), Error> {
        let mask = self.mask(pin_id)?;
        let mut ptr = field!(self.regs, gpioie);
        let val = ptr.read();
        ptr.write(val | mask);
        Ok(())
    }

    /// Disables the interrupt for this pin.
    pub fn disable_interrupt(&mut self, pin_id: usize) -> Result<(), Error> {
        let mask = self.mask(pin_id)?;
        let mut ptr = field!(self.regs, gpioie);
        let val = ptr.read();
        ptr.write(val & !mask);
        Ok(())
    }

    /// Checks if an interrupt is pending for this pin.
    /// This checks the masked interrupt status.
    pub fn is_interrupt_pending(&self, pin_id: usize) -> Result<bool, Error> {
        Ok(field_shared!(self.regs, gpiomis).read() & self.mask(pin_id)? != 0)
    }

    /// Clears edge detection logic for this pin.
    pub fn clear_interrupt_flag(&mut self, pin_id: usize) -> Result<(), Error> {
        let mask = self.mask(pin_id)?;
        field!(self.regs, gpioic).write(mask);
        Ok(())
    }

    /// Configures the interrupt to be edge-sensitive.
    pub fn set_interrupt_edge_sensitive(&mut self, pin_id: usize) -> Result<(), Error> {
        let mask = self.mask(pin_id)?;
        let mut ptr = field!(self.regs, gpiois);
        let val = ptr.read();
        ptr.write(val & !mask);
        Ok(())
    }

    /// Configures the interrupt to be level-sensitive.
    pub fn set_interrupt_level_sensitive(&mut self, pin_id: usize) -> Result<(), Error> {
        let mask = self.mask(pin_id)?;
        let mut ptr = field!(self.regs, gpiois);
        let val = ptr.read();
        ptr.write(val | mask);
        Ok(())
    }

    /// Configures the interrupt to trigger on a single edge (rising or falling,
    /// as determined by `set_interrupt_event`).
    pub fn set_interrupt_single_edge(&mut self, pin_id: usize) -> Result<(), Error> {
        let mask = self.mask(pin_id)?;
        let mut ptr = field!(self.regs, gpioibe);
        let val = ptr.read();
        ptr.write(val & !mask);
        Ok(())
    }

    /// Configures the interrupt to trigger on both rising and falling edges.
    pub fn set_interrupt_both_edges(&mut self, pin_id: usize) -> Result<(), Error> {
        let mask = self.mask(pin_id)?;
        let mut ptr = field!(self.regs, gpioibe);
        let val = ptr.read();
        ptr.write(val | mask);
        Ok(())
    }

    /// Configures the interrupt event to be a rising edge or a high level.
    pub fn set_interrupt_event_rising_high(&mut self, pin_id: usize) -> Result<(), Error> {
        let mask = self.mask(pin_id)?;
        let mut ptr = field!(self.regs, gpioiev);
        let val = ptr.read();
        ptr.write(val | mask);
        Ok(())
    }

    /// Configures the interrupt event to be a falling edge or a low level.
    pub fn set_interrupt_event_falling_low(&mut self, pin_id: usize) -> Result<(), Error> {
        let mask = self.mask(pin_id)?;
        let mut ptr = field!(self.regs, gpioiev);
        let val = ptr.read();
        ptr.write(val & !mask);
        Ok(())
    }

    /// Returns a temporary, safe, mutable pointer to this pin's data register.
    fn mut_data_ptr(
        &'a mut self,
        pin_id: usize,
    ) -> Option<UniqueMmioPointer<'a, ReadPureWrite<u8>>> {
        let offset = 1 << (pin_id + 2);
        field!(self.regs, gpiodata_region).take(offset)
    }

    /// Returns a temporary, safe pointer to this pin's data register.
    fn data_ptr(&'a self, pin_id: usize) -> Option<SharedMmioPointer<'a, ReadPureWrite<u8>>> {
        let offset = 1 << (pin_id + 2);
        field_shared!(self.regs, gpiodata_region).get(offset)
    }

    /// Drives the pin high.
    pub fn pin_set_high(&'a mut self, pin_id: usize) -> Result<(), Error> {
        // Writing any value with the pin's bit set to 1 will drive it high.
        // Writing all 1s is the most robust way to do this.
        let Some(mut ptr) = self.mut_data_ptr(pin_id) else {
            return Err(Error::InvalidPinId);
        };
        ptr.write(0xFF);
        Ok(())
    }

    /// Drives the pin low.
    pub fn pin_set_low(&'a mut self, pin_id: usize) -> Result<(), Error> {
        // Writing any value with the pin's bit set to 0 will drive it low.
        let Some(mut ptr) = self.mut_data_ptr(pin_id) else {
            return Err(Error::InvalidPinId);
        };
        ptr.write(0x00);
        Ok(())
    }

    /// Returns `true` if the pin's input level is high.
    pub fn pin_is_high(&'a self, pin_id: usize) -> Result<bool, Error> {
        let Some(ptr) = self.data_ptr(pin_id) else {
            return Err(Error::InvalidPinId);
        };
        // A masked read will return a non-zero value if the pin is high.
        Ok(ptr.read() != 0)
    }

    /// Returns `true` if the pin's input level is low.
    pub fn pin_is_low(&'a self, pin_id: usize) -> Result<bool, Error> {
        Ok(!self.pin_is_high(pin_id)?)
    }

    /// Returns the values of all the pins.
    pub fn read_all(&self) -> u8 {
        field_shared!(self.regs, gpiodata_region)
            .get(0xff << 2)
            .unwrap()
            .read()
    }

    /// Writes the the given `bits` to all the pins which are included in the given `mask`.
    pub fn write_all(&mut self, bits: u8, mask: u8) {
        field!(self.regs, gpiodata_region)
            .get(usize::from(mask) << 2)
            .unwrap()
            .write(bits);
    }

    /// Splits out the individual pins, so they can be owned separately.
    pub fn split(self) -> [Pin<'a>; PIN_COUNT] {
        // SAFETY: We only pass a single field name.
        let gpiodata_region = unsafe { split_fields!(self.regs, gpiodata_region) };
        gpiodata_region
            .split_some(array::from_fn(|pin_id| 1 << (pin_id + 2)))
            .map(|register| Pin { register })
    }
}

/// A single GPIO pin from a PL061 device.
pub struct Pin<'a> {
    register: UniqueMmioPointer<'a, ReadPureWrite<u8>>,
}

impl Pin<'_> {
    /// Sets the pin high if `value` is true, or low if it is false.
    pub fn set(&mut self, value: bool) {
        self.register.write(if value { 0xff } else { 0 });
    }

    /// Returns `true` if the pin's input level is high.
    pub fn is_high(&self) -> bool {
        self.register.read() != 0
    }
}

/// Peripheral identification structure
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Identification {
    pub part_number: u16,
    pub designer: u8,
    pub revision: u8,
    pub configuration: u8,
}

impl Identification {
    const PART_NUMBER: u16 = 0x061;
    const DESIGNER_ARM: u8 = 0x41; // ASCII 'A'
    const REVISION: u8 = 0x0;
    const CONFIGURATION: u8 = 0x00;

    /// Check if the identification block describes a valid PL061 peripheral
    pub fn is_valid(&self) -> bool {
        self.part_number == Self::PART_NUMBER
            && self.designer == Self::DESIGNER_ARM
            && self.revision == Self::REVISION
            && self.configuration == Self::CONFIGURATION
    }
}

impl<'a> PL061<'a> {
    /// Read GPIO peripheral identification structure
    pub fn read_identification(&self) -> Identification {
        let id0 = field_shared!(self.regs, gpioperiphid0).read();
        let id1 = field_shared!(self.regs, gpioperiphid1).read();
        let id2 = field_shared!(self.regs, gpioperiphid2).read();
        let id3 = field_shared!(self.regs, gpioperiphid3).read();

        Identification {
            part_number: ((id1 & 0x0F) << 8) as u16 | (id0 & 0xFF) as u16,
            designer: ((id2 & 0x0F) << 4) as u8 | ((id1 & 0xF0) >> 4) as u8,
            revision: ((id2 & 0xF0) >> 4) as u8,
            configuration: (id3 & 0xFF) as u8,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zerocopy::transmute_mut;

    #[derive(Clone, Eq, FromBytes, IntoBytes, PartialEq)]
    pub struct FakePL061Registers {
        regs: [u32; 0x3FC],
    }

    impl FakePL061Registers {
        pub fn new() -> Self {
            Self {
                regs: [0u32; 0x3FC],
            }
        }

        pub fn clear(&mut self) {
            self.regs.fill(0);
        }

        // Provide offset in bytes.
        pub fn reg_read(&self, offset: usize) -> u32 {
            self.regs[offset / 4]
        }

        pub fn reg_write(&mut self, offset: usize, value: u32) {
            self.regs[offset / 4] = value;
        }
    }

    fn pl061_for_testing(regs: &mut FakePL061Registers) -> PL061<'_> {
        PL061::new(UniqueMmioPointer::from(transmute_mut!(regs)))
    }

    #[test]
    fn regs_size() {
        assert_eq!(core::mem::size_of::<PL061Registers>(), 0xFF0);
    }

    #[test]
    fn mask_checks() {
        let mut regs = FakePL061Registers::new();
        let pl061 = pl061_for_testing(&mut regs);

        assert_eq!(pl061.mask(0), Ok(1));
        assert_eq!(pl061.mask(1), Ok(2));
        assert_eq!(pl061.mask(2), Ok(4));
        assert_eq!(pl061.mask(8), Err(Error::InvalidPinId));
    }

    #[test]
    fn pin_id_checks() {
        let mut regs = FakePL061Registers::new();
        let pl061 = pl061_for_testing(&mut regs);

        assert_eq!(pl061.pin_is_high(0), Ok(false));
        assert_eq!(pl061.pin_is_low(0), Ok(true));
        assert_eq!(pl061.pin_is_high(8), Err(Error::InvalidPinId));
        assert_eq!(pl061.pin_is_low(8), Err(Error::InvalidPinId));
    }

    #[test]
    fn pin_to_output() {
        const GPIODIR: usize = 0x400;
        let mut regs = FakePL061Registers::new();

        // Pin 0 is configured as input at first.
        assert_eq!(0u32, regs.reg_read(GPIODIR));
        let mut pl061 = pl061_for_testing(&mut regs);
        pl061.into_output(0).unwrap();

        // Check that pin 0 is now an output.
        assert_eq!(1u32, regs.reg_read(GPIODIR));
    }

    #[test]
    fn pin_to_output_to_input() {
        const GPIODIR: usize = 0x400;
        let mut regs = FakePL061Registers::new();

        // Pin 0 is configured as input at first.
        assert_eq!(0u32, regs.reg_read(GPIODIR));
        {
            let mut pl061 = pl061_for_testing(&mut regs);
            pl061.into_output(0).unwrap();
        }

        // Check that pin 0 is now an output.
        assert_eq!(1u32, regs.reg_read(GPIODIR));

        {
            let mut pl061 = pl061_for_testing(&mut regs);
            pl061.into_input(0).unwrap();
        }

        // Check that pin 0 is now an input again.
        assert_eq!(0u32, regs.reg_read(GPIODIR));
    }

    #[test]
    fn enable_interrupt() {
        const GPIOIE: usize = 0x410;
        let mut regs = FakePL061Registers::new();
        assert_eq!(0u32, regs.reg_read(GPIOIE));
        let mut pl061 = pl061_for_testing(&mut regs);

        pl061.enable_interrupt(0).unwrap();
        assert_eq!(1u32, regs.reg_read(GPIOIE));
    }

    #[test]
    fn disable_interrupt() {
        const GPIOIE: usize = 0x410;
        let mut regs = FakePL061Registers::new();
        assert_eq!(0u32, regs.reg_read(GPIOIE));

        {
            let mut pl061 = pl061_for_testing(&mut regs);
            pl061.enable_interrupt(0).unwrap();
        }

        // Check interrupts are enabled.
        assert_eq!(1u32, regs.reg_read(GPIOIE));

        {
            let mut pl061 = pl061_for_testing(&mut regs);
            pl061.disable_interrupt(0).unwrap();
        }

        // Check interrupts are disabled.
        assert_eq!(0u32, regs.reg_read(GPIOIE));
    }

    #[test]
    fn check_pending_interrupt() {
        const GPIOMIS: usize = 0x418;
        let mut regs = FakePL061Registers::new();

        // Check there's no interrupts pending initially.
        assert_eq!(0u32, regs.reg_read(GPIOMIS));

        // Write to GPIOMIS to simulate an interrupt on pin 0.
        regs.reg_write(GPIOMIS, 0x1);

        let pl061 = pl061_for_testing(&mut regs);

        assert_eq!(true, pl061.is_interrupt_pending(0).unwrap());
    }

    #[test]
    fn check_clear_interrupt_flag() {
        const GPIOIC: usize = 0x41C;
        let mut regs = FakePL061Registers::new();
        let mut pl061 = pl061_for_testing(&mut regs);

        pl061.clear_interrupt_flag(0).unwrap();

        assert_eq!(1u32, regs.reg_read(GPIOIC));
    }

    #[test]
    fn check_setting_interrupt_config() {
        // Interrupt sense register.
        const GPIOIS: usize = 0x404;
        // Interrupt both edges register.
        const GPIOIBE: usize = 0x408;
        // Interrupt event register.
        const GPIOIEV: usize = 0x40C;

        let mut regs = FakePL061Registers::new();
        {
            assert_eq!(0u32, regs.reg_read(GPIOIS));
            let mut pl061 = pl061_for_testing(&mut regs);
            pl061.set_interrupt_level_sensitive(0).unwrap();
            assert_eq!(1u32, regs.reg_read(GPIOIS));
        }
        {
            assert_eq!(1u32, regs.reg_read(GPIOIS));
            let mut pl061 = pl061_for_testing(&mut regs);
            pl061.set_interrupt_edge_sensitive(0).unwrap();
            assert_eq!(0u32, regs.reg_read(GPIOIS));
        }
        regs.clear();
        {
            assert_eq!(0u32, regs.reg_read(GPIOIBE));
            let mut pl061 = pl061_for_testing(&mut regs);
            pl061.set_interrupt_both_edges(0).unwrap();
            assert_eq!(1u32, regs.reg_read(GPIOIBE));
        }
        {
            assert_eq!(1u32, regs.reg_read(GPIOIBE));
            let mut pl061 = pl061_for_testing(&mut regs);
            pl061.set_interrupt_single_edge(0).unwrap();
            assert_eq!(0u32, regs.reg_read(GPIOIBE));
        }
        regs.clear();
        {
            assert_eq!(0u32, regs.reg_read(GPIOIEV));
            let mut pl061 = pl061_for_testing(&mut regs);
            pl061.set_interrupt_event_rising_high(0).unwrap();
            assert_eq!(1u32, regs.reg_read(GPIOIEV));
        }
        {
            assert_eq!(1u32, regs.reg_read(GPIOIEV));
            let mut pl061 = pl061_for_testing(&mut regs);
            pl061.set_interrupt_event_falling_low(0).unwrap();
            assert_eq!(0u32, regs.reg_read(GPIOIEV));
        }
    }

    #[test]
    fn check_pin_operations() {
        let mut regs = FakePL061Registers::new();
        {
            // To set pin 0 high, bits [9:2] of the address we write to (PADDR) have to be
            // 0b0000_0001. Any value other than 0 means the pin is high.
            assert_eq!(0u32, regs.reg_read(0x4));
            let mut pl061 = pl061_for_testing(&mut regs);
            assert!(pl061.pin_is_low(0).unwrap());
            pl061.pin_set_high(0).unwrap();
            assert!(0u32 != regs.reg_read(0x4));
        }
        {
            assert!(0u32 != regs.reg_read(0x4));
            let mut pl061 = pl061_for_testing(&mut regs);
            assert!(pl061.pin_is_high(0).unwrap());
            pl061.pin_set_low(0).unwrap();
            assert_eq!(0u32, regs.reg_read(0x4));
        }
        regs.clear();
        {
            // To set pin 2 high, bits [9:2] of the address we write to (PADDR) have to be
            // 0b0000_0100. Any value other than 0 means the pin is high.
            assert_eq!(0u32, regs.reg_read(0x10));
            let mut pl061 = pl061_for_testing(&mut regs);
            assert!(pl061.pin_is_low(2).unwrap());
            pl061.pin_set_high(2).unwrap();
            assert!(0u32 != regs.reg_read(0x10));
        }
        {
            assert!(0u32 != regs.reg_read(0x10));
            let mut pl061 = pl061_for_testing(&mut regs);
            assert!(pl061.pin_is_high(2).unwrap());
            pl061.pin_set_low(2).unwrap();
        }
    }

    #[test]
    fn identification() {
        const GPIO_PERIPH_ID_0: usize = 0xFE0;
        const GPIO_PERIPH_ID_1: usize = 0xFE4;
        const GPIO_PERIPH_ID_2: usize = 0xFE8;
        const GPIO_PERIPH_ID_3: usize = 0xFEC;

        let mut regs = FakePL061Registers::new();

        // Write reset values defined in 3.2 of the PL061 Technical Reference Manual.
        regs.reg_write(GPIO_PERIPH_ID_0, 0x61);
        regs.reg_write(GPIO_PERIPH_ID_1, 0x10);
        regs.reg_write(GPIO_PERIPH_ID_2, 0x04);
        regs.reg_write(GPIO_PERIPH_ID_3, 0x00);

        let expected_identification = Identification {
            part_number: 0x061,
            designer: 0x41,
            revision: 0x00,
            configuration: 0x00,
        };
        {
            let pl061 = pl061_for_testing(&mut regs);
            let identification = pl061.read_identification();
            assert_eq!(expected_identification, identification);
            assert_eq!(true, identification.is_valid());
        }
        // Invalidate the peripheral identification
        regs.reg_write(GPIO_PERIPH_ID_0, 0xFF);
        {
            let pl061 = pl061_for_testing(&mut regs);
            let identification = pl061.read_identification();
            assert_eq!(false, identification.is_valid());
        }
    }
}
