#![no_std]

#[cfg(target_os = "none")]
pub mod hardware;

pub mod convert;
pub mod dt670;
pub mod output_channel;
pub mod statistics;

#[derive(Clone, Copy, strum::EnumIter, Debug)]
#[repr(u8)]
pub enum OutputChannelIdx {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}
