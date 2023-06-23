/// The status of a goal.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
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
            GoalStatus::Unknown => crate::action_msgs::msg::GoalStatus::STATUS_UNKNOWN as i8,
            GoalStatus::Accepted => crate::action_msgs::msg::GoalStatus::STATUS_ACCEPTED as i8,
            GoalStatus::Executing => crate::action_msgs::msg::GoalStatus::STATUS_EXECUTING as i8,
            GoalStatus::Canceling => crate::action_msgs::msg::GoalStatus::STATUS_CANCELING as i8,
            GoalStatus::Succeeded => crate::action_msgs::msg::GoalStatus::STATUS_SUCCEEDED as i8,
            GoalStatus::Canceled => crate::action_msgs::msg::GoalStatus::STATUS_CANCELED as i8,
            GoalStatus::Aborted => crate::action_msgs::msg::GoalStatus::STATUS_ABORTED as i8,
        }
    }

    pub fn from_rcl(s: i8) -> Self {
        match s {
            s if s == crate::action_msgs::msg::GoalStatus::STATUS_UNKNOWN as i8 => {
                GoalStatus::Unknown
            }
            s if s == crate::action_msgs::msg::GoalStatus::STATUS_ACCEPTED as i8 => {
                GoalStatus::Accepted
            }
            s if s == crate::action_msgs::msg::GoalStatus::STATUS_EXECUTING as i8 => {
                GoalStatus::Executing
            }
            s if s == crate::action_msgs::msg::GoalStatus::STATUS_CANCELING as i8 => {
                GoalStatus::Canceling
            }
            s if s == crate::action_msgs::msg::GoalStatus::STATUS_SUCCEEDED as i8 => {
                GoalStatus::Succeeded
            }
            s if s == crate::action_msgs::msg::GoalStatus::STATUS_CANCELED as i8 => {
                GoalStatus::Canceled
            }
            s if s == crate::action_msgs::msg::GoalStatus::STATUS_ABORTED as i8 => {
                GoalStatus::Aborted
            }
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
