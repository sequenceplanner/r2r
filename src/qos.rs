//! QoS (Quality of Service)
//! full credit goes to https://github.com/rclrust/rclrust/blob/main/rclrust/src/qos.rs

use std::time::Duration;

use r2r_rcl::{
    rmw_qos_durability_policy_t, rmw_qos_history_policy_t, rmw_qos_liveliness_policy_t,
    rmw_qos_reliability_policy_t, rmw_time_t,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HistoryPolicy {
    KeepAll,
    KeepLast,
    SystemDefault,
    Unknown,
}

impl From<HistoryPolicy> for rmw_qos_history_policy_t {
    fn from(history_policy: HistoryPolicy) -> Self {
        match history_policy {
            HistoryPolicy::KeepAll => rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            HistoryPolicy::KeepLast => rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            HistoryPolicy::SystemDefault => {
                rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT
            }
            HistoryPolicy::Unknown => rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_UNKNOWN,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReliabilityPolicy {
    BestEffort,
    Reliable,
    SystemDefault,
    Unknown,
}
impl From<ReliabilityPolicy> for rmw_qos_reliability_policy_t {
    fn from(reliability_policy: ReliabilityPolicy) -> Self {
        match reliability_policy {
            ReliabilityPolicy::BestEffort => {
                rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
            }
            ReliabilityPolicy::Reliable => {
                rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE
            }
            ReliabilityPolicy::SystemDefault => {
                rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT
            }
            ReliabilityPolicy::Unknown => {
                rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_UNKNOWN
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DurabilityPolicy {
    TransientLocal,
    Volatile,
    SystemDefault,
    Unknown,
}

impl From<DurabilityPolicy> for rmw_qos_durability_policy_t {
    fn from(durability_policy: DurabilityPolicy) -> Self {
        match durability_policy {
            DurabilityPolicy::TransientLocal => {
                rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
            }
            DurabilityPolicy::Volatile => {
                rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE
            }
            DurabilityPolicy::SystemDefault => {
                rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT
            }
            DurabilityPolicy::Unknown => {
                rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_UNKNOWN
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LivelinessPolicy {
    Automatic,
    ManualByNode,
    ManualByTopic,
    SystemDefault,
    Unknown,
}

impl From<LivelinessPolicy> for rmw_qos_liveliness_policy_t {
    fn from(liveliness_policy: LivelinessPolicy) -> Self {
        match liveliness_policy {
            LivelinessPolicy::Automatic => {
                rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
            }
            LivelinessPolicy::ManualByNode => {
                rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE
            }
            LivelinessPolicy::ManualByTopic => {
                rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC
            }
            LivelinessPolicy::SystemDefault => {
                rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT
            }
            LivelinessPolicy::Unknown => {
                rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_UNKNOWN
            }
        }
    }
}

/// QoS profile
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QosProfile {
    /// History QoS policy setting.
    pub history: HistoryPolicy,
    /// Size of the message queue.
    pub depth: usize,
    /// Reliabiilty QoS policy setting.
    pub reliability: ReliabilityPolicy,
    /// Durability QoS policy setting.
    pub durability: DurabilityPolicy,
    /// The period at which messages are expected to be sent/received.
    pub deadline: Duration,
    /// The age at which messages are considered expired and no longer valid.
    pub lifespan: Duration,
    /// Liveliness QoS policy setting.
    pub liveliness: LivelinessPolicy,
    /// The time within which the RMW node or publisher must show that it is alive.
    pub liveliness_lease_duration: Duration,
    /// If true, any ROS specific namespacing conventions will be circumvented.
    ///
    /// In the case of DDS and topics, for example, this means the typical ROS specific prefix of rt
    /// would not be applied as described here:
    ///
    /// <http://design.ros2.org/articles/topic_and_service_names.html#ros-specific-namespace-prefix>
    ///
    /// This might be useful when trying to directly connect a native DDS topic with a ROS 2 topic.
    pub avoid_ros_namespace_conventions: bool,
}

impl QosProfile {
    /// Sensor Data QoS class
    ///    - History: Keep last,
    ///    - Depth: 5,
    ///    - Reliability: Best effort,
    ///    - Durability: Volatile,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - avoid ros namespace conventions: false
    pub const fn sensor_data() -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth: 5,
            reliability: ReliabilityPolicy::BestEffort,
            durability: DurabilityPolicy::Volatile,
            ..Self::common()
        }
    }

    /// Parameters QoS class
    ///    - History: Keep last,
    ///    - Depth: 1000,
    ///    - Reliability: Reliable,
    ///    - Durability: Volatile,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn parameters() -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth: 1000,
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            ..Self::common()
        }
    }

    /// Default QoS class
    ///    - History: Keep last,
    ///    - Depth: 10,
    ///    - Reliability: Reliable,
    ///    - Durability: Volatile,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn default() -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth: 10,
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            ..Self::common()
        }
    }

    /// Services QoS class
    ///    - History: Keep last,
    ///    - Depth: 10,
    ///    - Reliability: Reliable,
    ///    - Durability: Volatile,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn services_default() -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth: 10,
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            ..Self::common()
        }
    }

    /// Parameter events QoS class
    ///    - History: Keep last,
    ///    - Depth: 1000,
    ///    - Reliability: Reliable,
    ///    - Durability: Volatile,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn parameter_events() -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth: 1000,
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            ..Self::common()
        }
    }

    /// System defaults QoS class
    ///    - History: System default,
    ///    - Depth: System default,
    ///    - Reliability: System default,
    ///    - Durability: System default,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn system_default() -> Self {
        Self::common()
    }

    /// Unknow QoS class
    ///    - History: Unknown,
    ///    - Depth: System default,
    ///    - Reliability: Unknown,
    ///    - Durability: Unknown,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: Unknown,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn unknown() -> Self {
        Self {
            history: HistoryPolicy::Unknown,
            reliability: ReliabilityPolicy::Unknown,
            durability: DurabilityPolicy::Unknown,
            liveliness: LivelinessPolicy::Unknown,
            ..Self::common()
        }
    }

    const fn common() -> Self {
        Self {
            history: HistoryPolicy::SystemDefault,
            depth: r2r_rcl::RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT as usize,
            reliability: ReliabilityPolicy::SystemDefault,
            durability: DurabilityPolicy::SystemDefault,
            deadline: Duration::ZERO,
            lifespan: Duration::ZERO,
            liveliness: LivelinessPolicy::SystemDefault,
            liveliness_lease_duration: Duration::ZERO,
            avoid_ros_namespace_conventions: false,
        }
    }

    /// Set the history policy.
    ///
    /// # Examples
    ///
    /// ```
    /// # use r2r::qos::{HistoryPolicy, QosProfile};
    /// #
    /// let qos = QosProfile::default().history(HistoryPolicy::KeepAll);
    /// ```
    pub const fn history(self, history: HistoryPolicy) -> Self {
        Self { history, ..self }
    }

    /// Set the history to keep last.
    ///
    /// # Examples
    ///
    /// ```
    /// # use r2r::qos::QosProfile;
    /// #
    /// let qos = QosProfile::default().keep_last(10);
    /// ```
    pub const fn keep_last(self, depth: usize) -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth,
            ..self
        }
    }

    /// Set the history to keep all.
    ///
    /// # Examples
    ///
    /// ```
    /// # use r2r::qos::QosProfile;
    /// #
    /// let qos = QosProfile::default().keep_all();
    /// ```
    pub const fn keep_all(self) -> Self {
        Self {
            history: HistoryPolicy::KeepAll,
            depth: 0,
            ..self
        }
    }

    /// Set the reliability setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use r2r::qos::{QosProfile, ReliabilityPolicy};
    /// #
    /// let qos = QosProfile::default().reliability(ReliabilityPolicy::Reliable);
    /// ```
    pub const fn reliability(self, reliability: ReliabilityPolicy) -> Self {
        Self {
            reliability,
            ..self
        }
    }

    /// Set the reliability setting to reliable.
    ///
    /// # Examples
    ///
    /// ```
    /// # use r2r::qos::QosProfile;
    /// #
    /// let qos = QosProfile::default().reliable();
    /// ```
    pub const fn reliable(self) -> Self {
        self.reliability(ReliabilityPolicy::Reliable)
    }

    /// Set the reliability setting to best effort.
    ///
    /// # Examples
    ///
    /// ```
    /// # use r2r::qos::QosProfile;
    /// #
    /// let qos = QosProfile::default().best_effort();
    /// ```
    pub const fn best_effort(self) -> Self {
        self.reliability(ReliabilityPolicy::BestEffort)
    }

    /// Set the durability setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use r2r::qos::{DurabilityPolicy, QosProfile};
    /// #
    /// let qos = QosProfile::default().durability(DurabilityPolicy::Volatile);
    /// ```
    pub const fn durability(self, durability: DurabilityPolicy) -> Self {
        Self { durability, ..self }
    }

    /// Set the durability setting to volatile.
    ///
    /// # Examples
    ///
    /// ```
    /// # use r2r::qos::QosProfile;
    /// #
    /// let qos = QosProfile::default().volatile();
    /// ```
    pub const fn volatile(self) -> Self {
        self.durability(DurabilityPolicy::Volatile)
    }

    /// Set the durability setting to transient local.
    ///
    /// # Examples
    ///
    /// ```
    /// # use r2r::qos::QosProfile;
    /// #
    /// let qos = QosProfile::default().transient_local();
    /// ```
    pub const fn transient_local(self) -> Self {
        self.durability(DurabilityPolicy::TransientLocal)
    }

    /// Set the deadline setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use std::time::Duration;
    /// #
    /// # use r2r::qos::QosProfile;
    /// #
    /// let qos = QosProfile::default().deadline(Duration::from_secs(5));
    /// ```
    pub const fn deadline(self, deadline: Duration) -> Self {
        Self { deadline, ..self }
    }

    /// Set the lifespan setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use std::time::Duration;
    /// #
    /// # use r2r::qos::QosProfile;
    /// #
    /// let qos = QosProfile::default().lifespan(Duration::from_secs(5));
    /// ```
    pub const fn lifespan(self, lifespan: Duration) -> Self {
        Self { lifespan, ..self }
    }

    /// Set the liveliness setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use r2r::qos::{LivelinessPolicy, QosProfile};
    /// #
    /// let qos = QosProfile::default().liveliness(LivelinessPolicy::Automatic);
    /// ```
    pub const fn liveliness(self, liveliness: LivelinessPolicy) -> Self {
        Self { liveliness, ..self }
    }

    /// Set the liveliness_lease_duration setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use std::time::Duration;
    /// #
    /// # use r2r::qos::QosProfile;
    /// #
    /// let qos = QosProfile::default().liveliness_lease_duration(Duration::from_secs(5));
    /// ```
    pub const fn liveliness_lease_duration(self, liveliness_lease_duration: Duration) -> Self {
        Self {
            liveliness_lease_duration,
            ..self
        }
    }

    /// Set the avoid_ros_namespace_conventions setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use r2r::qos::QosProfile;
    /// #
    /// let qos = QosProfile::default().avoid_ros_namespace_conventions(true);
    /// ```
    pub const fn avoid_ros_namespace_conventions(
        self,
        avoid_ros_namespace_conventions: bool,
    ) -> Self {
        Self {
            avoid_ros_namespace_conventions,
            ..self
        }
    }
}

impl From<QosProfile> for r2r_rcl::rmw_qos_profile_t {
    fn from(qos: QosProfile) -> Self {
        Self {
            history: qos.history.into(),
            depth: qos.depth,
            reliability: qos.reliability.into(),
            durability: qos.durability.into(),
            deadline: qos.deadline.to_rmw_time_t(),
            lifespan: qos.lifespan.to_rmw_time_t(),
            liveliness: qos.liveliness.into(),
            liveliness_lease_duration: qos.liveliness_lease_duration.to_rmw_time_t(),
            avoid_ros_namespace_conventions: qos.avoid_ros_namespace_conventions,
        }
    }
}
pub(crate) trait RclDurationT {
    fn to_rmw_time_t(&self) -> rmw_time_t;
}

impl RclDurationT for Duration {
    fn to_rmw_time_t(&self) -> rmw_time_t {
        rmw_time_t {
            sec: self.as_secs(),
            nsec: self.subsec_nanos().into(),
        }
    }
}
