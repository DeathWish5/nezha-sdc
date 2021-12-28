use core::fmt;

pub type SdResult<T = ()> = Result<T, SdError>;

#[repr(isize)]
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum SdError {
    // Hardware Error
    RintTimeout = 0,
    RintError,
    MMCSendTimeout,
    TranslateDataTimeout,
    WrongDataFlag,
    SdcardBusy,
    UpdateClockFail,
    MMCStatusError,
    DmaTimeout,
    DmaError,
    // Software Error
    Unsupported,
    InvalidParam,
    Unknown,
}

impl fmt::Display for SdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use self::SdError::*;
        let explain = match self {
            RintTimeout => "Rint timeout",
            RintError => "Rint error",
            MMCSendTimeout => "MMC send timeout",
            TranslateDataTimeout => "Translate data timeout",
            WrongDataFlag => "Wrong data flag",
            SdcardBusy => "Sdcard busy",
            UpdateClockFail => "Update Clock fail",
            MMCStatusError => "MMC status error",
            DmaTimeout => "Dma timeout",
            DmaError => "Dma error",
            Unsupported => "Not supported yet",
            InvalidParam => "Invalid parameters",
            Unknown => "Unknown error",
        };
        write!(f, "{}", explain)
    }
}
