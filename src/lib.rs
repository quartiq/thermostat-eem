#![no_std]

#[cfg(target_os = "none")]
pub mod hardware;

pub mod convert;
pub mod dt670;
pub mod output_channel;
pub mod statistics;
use bitbybit::bitenum;

#[derive(Debug, PartialEq, PartialOrd)]
#[bitenum(u2, exhaustive = true)]
pub enum OutputChannelIdx {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}

impl OutputChannelIdx {
    pub const ALL: [Self; 4] = [Self::Zero, Self::One, Self::Two, Self::Three];
}
