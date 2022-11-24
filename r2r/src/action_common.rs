/// The status of a goal.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GoalStatus {
    Unknown,
    Accepted,
    Executing,
    Canceling,
    Succeeded,
    Canceled,
    Aborted,
}

impl GoalStatus {
    #[allow(dead_code)]
    pub fn to_rcl(&self) -> i8 {
        match self {
            GoalStatus::Unknown => 0,
            GoalStatus::Accepted => 1,
            GoalStatus::Executing => 2,
            GoalStatus::Canceling => 3,
            GoalStatus::Succeeded => 4,
            GoalStatus::Canceled => 5,
            GoalStatus::Aborted => 6,
        }
    }

    pub fn from_rcl(s: i8) -> Self {
        match s {
            0 => GoalStatus::Unknown,
            1 => GoalStatus::Accepted,
            2 => GoalStatus::Executing,
            3 => GoalStatus::Canceling,
            4 => GoalStatus::Succeeded,
            5 => GoalStatus::Canceled,
            6 => GoalStatus::Aborted,
            _ => panic!("unknown action status: {}", s),
        }
    }
}

impl std::fmt::Display for GoalStatus {
    fn fmt(&self, fmtr: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            GoalStatus::Unknown => "unknown",
            GoalStatus::Accepted => "accepted",
            GoalStatus::Executing => "executing",
            GoalStatus::Canceling => "canceling",
            GoalStatus::Succeeded => "succeeded",
            GoalStatus::Canceled => "canceled",
            GoalStatus::Aborted => "aborted",
        };

        write!(fmtr, "{}", s)
    }
}
