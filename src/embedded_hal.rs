// SPDX-FileCopyrightText: Copyright The arm-pl061 Contributors.
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::Pin;
use core::convert::Infallible;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

impl ErrorType for Pin<'_> {
    type Error = Infallible;
}

impl InputPin for Pin<'_> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!Self::is_high(self))
    }
}

impl OutputPin for Pin<'_> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set(false);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set(true);
        Ok(())
    }
}

impl StatefulOutputPin for Pin<'_> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        self.is_high()
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        self.is_low()
    }
}
