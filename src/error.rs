use core::fmt;

pub type SdResult<T = ()> = Result<T, SdError>;

#[repr(isize)]
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum SdError {
    RintTimeout = 0,
    RintError = 1,
    MMCSendTimeout = 2,
    TranslateDataTimeout = 3,
    WrongDataFlag = 4,
    SdcardBusy = 5,
    UpdateClockFail = 6,
    MMCStatusError = 7,
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
        };
        write!(f, "{}", explain)
    }
}
