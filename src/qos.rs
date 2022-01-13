// The content of this file has the Apache 2.0 licence due to its origin in the rclrust repo (https://github.com/rclrust/rclrust). The licence of that repo is included below.
//
//                                 Apache License
//                           Version 2.0, January 2004
//                        http://www.apache.org/licenses/
//
//   TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION
//
//   1. Definitions.
//
//      "License" shall mean the terms and conditions for use, reproduction,
//      and distribution as defined by Sections 1 through 9 of this document.
//
//      "Licensor" shall mean the copyright owner or entity authorized by
//      the copyright owner that is granting the License.
//
//      "Legal Entity" shall mean the union of the acting entity and all
//      other entities that control, are controlled by, or are under common
//      control with that entity. For the purposes of this definition,
//      "control" means (i) the power, direct or indirect, to cause the
//      direction or management of such entity, whether by contract or
//      otherwise, or (ii) ownership of fifty percent (50%) or more of the
//      outstanding shares, or (iii) beneficial ownership of such entity.
//
//      "You" (or "Your") shall mean an individual or Legal Entity
//      exercising permissions granted by this License.
//
//      "Source" form shall mean the preferred form for making modifications,
//      including but not limited to software source code, documentation
//      source, and configuration files.
//
//      "Object" form shall mean any form resulting from mechanical
//      transformation or translation of a Source form, including but
//      not limited to compiled object code, generated documentation,
//      and conversions to other media types.
//
//      "Work" shall mean the work of authorship, whether in Source or
//      Object form, made available under the License, as indicated by a
//      copyright notice that is included in or attached to the work
//      (an example is provided in the Appendix below).
//
//      "Derivative Works" shall mean any work, whether in Source or Object
//      form, that is based on (or derived from) the Work and for which the
//      editorial revisions, annotations, elaborations, or other modifications
//      represent, as a whole, an original work of authorship. For the purposes
//      of this License, Derivative Works shall not include works that remain
//      separable from, or merely link (or bind by name) to the interfaces of,
//      the Work and Derivative Works thereof.
//
//      "Contribution" shall mean any work of authorship, including
//      the original version of the Work and any modifications or additions
//      to that Work or Derivative Works thereof, that is intentionally
//      submitted to Licensor for inclusion in the Work by the copyright owner
//      or by an individual or Legal Entity authorized to submit on behalf of
//      the copyright owner. For the purposes of this definition, "submitted"
//      means any form of electronic, verbal, or written communication sent
//      to the Licensor or its representatives, including but not limited to
//      communication on electronic mailing lists, source code control systems,
//      and issue tracking systems that are managed by, or on behalf of, the
//      Licensor for the purpose of discussing and improving the Work, but
//      excluding communication that is conspicuously marked or otherwise
//      designated in writing by the copyright owner as "Not a Contribution."
//
//      "Contributor" shall mean Licensor and any individual or Legal Entity
//      on behalf of whom a Contribution has been received by Licensor and
//      subsequently incorporated within the Work.
//
//   2. Grant of Copyright License. Subject to the terms and conditions of
//      this License, each Contributor hereby grants to You a perpetual,
//      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
//      copyright license to reproduce, prepare Derivative Works of,
//      publicly display, publicly perform, sublicense, and distribute the
//      Work and such Derivative Works in Source or Object form.
//
//   3. Grant of Patent License. Subject to the terms and conditions of
//      this License, each Contributor hereby grants to You a perpetual,
//      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
//      (except as stated in this section) patent license to make, have made,
//      use, offer to sell, sell, import, and otherwise transfer the Work,
//      where such license applies only to those patent claims licensable
//      by such Contributor that are necessarily infringed by their
//      Contribution(s) alone or by combination of their Contribution(s)
//      with the Work to which such Contribution(s) was submitted. If You
//      institute patent litigation against any entity (including a
//      cross-claim or counterclaim in a lawsuit) alleging that the Work
//      or a Contribution incorporated within the Work constitutes direct
//      or contributory patent infringement, then any patent licenses
//      granted to You under this License for that Work shall terminate
//      as of the date such litigation is filed.
//
//   4. Redistribution. You may reproduce and distribute copies of the
//      Work or Derivative Works thereof in any medium, with or without
//      modifications, and in Source or Object form, provided that You
//      meet the following conditions:
//
//      (a) You must give any other recipients of the Work or
//          Derivative Works a copy of this License; and
//
//      (b) You must cause any modified files to carry prominent notices
//          stating that You changed the files; and
//
//      (c) You must retain, in the Source form of any Derivative Works
//          that You distribute, all copyright, patent, trademark, and
//          attribution notices from the Source form of the Work,
//          excluding those notices that do not pertain to any part of
//          the Derivative Works; and
//
//      (d) If the Work includes a "NOTICE" text file as part of its
//          distribution, then any Derivative Works that You distribute must
//          include a readable copy of the attribution notices contained
//          within such NOTICE file, excluding those notices that do not
//          pertain to any part of the Derivative Works, in at least one
//          of the following places: within a NOTICE text file distributed
//          as part of the Derivative Works; within the Source form or
//          documentation, if provided along with the Derivative Works; or,
//          within a display generated by the Derivative Works, if and
//          wherever such third-party notices normally appear. The contents
//          of the NOTICE file are for informational purposes only and
//          do not modify the License. You may add Your own attribution
//          notices within Derivative Works that You distribute, alongside
//          or as an addendum to the NOTICE text from the Work, provided
//          that such additional attribution notices cannot be construed
//          as modifying the License.
//
//      You may add Your own copyright statement to Your modifications and
//      may provide additional or different license terms and conditions
//      for use, reproduction, or distribution of Your modifications, or
//      for any such Derivative Works as a whole, provided Your use,
//      reproduction, and distribution of the Work otherwise complies with
//      the conditions stated in this License.
//
//   5. Submission of Contributions. Unless You explicitly state otherwise,
//      any Contribution intentionally submitted for inclusion in the Work
//      by You to the Licensor shall be under the terms and conditions of
//      this License, without any additional terms or conditions.
//      Notwithstanding the above, nothing herein shall supersede or modify
//      the terms of any separate license agreement you may have executed
//      with Licensor regarding such Contributions.
//
//   6. Trademarks. This License does not grant permission to use the trade
//      names, trademarks, service marks, or product names of the Licensor,
//      except as required for reasonable and customary use in describing the
//      origin of the Work and reproducing the content of the NOTICE file.
//
//   7. Disclaimer of Warranty. Unless required by applicable law or
//      agreed to in writing, Licensor provides the Work (and each
//      Contributor provides its Contributions) on an "AS IS" BASIS,
//      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
//      implied, including, without limitation, any warranties or conditions
//      of TITLE, NON-INFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A
//      PARTICULAR PURPOSE. You are solely responsible for determining the
//      appropriateness of using or redistributing the Work and assume any
//      risks associated with Your exercise of permissions under this License.
//
//   8. Limitation of Liability. In no event and under no legal theory,
//      whether in tort (including negligence), contract, or otherwise,
//      unless required by applicable law (such as deliberate and grossly
//      negligent acts) or agreed to in writing, shall any Contributor be
//      liable to You for damages, including any direct, indirect, special,
//      incidental, or consequential damages of any character arising as a
//      result of this License or out of the use or inability to use the
//      Work (including but not limited to damages for loss of goodwill,
//      work stoppage, computer failure or malfunction, or any and all
//      other commercial damages or losses), even if such Contributor
//      has been advised of the possibility of such damages.
//
//   9. Accepting Warranty or Additional Liability. While redistributing
//      the Work or Derivative Works thereof, You may choose to offer,
//      and charge a fee for, acceptance of support, warranty, indemnity,
//      or other liability obligations and/or rights consistent with this
//      License. However, in accepting such obligations, You may act only
//      on Your own behalf and on Your sole responsibility, not on behalf
//      of any other Contributor, and only if You agree to indemnify,
//      defend, and hold each Contributor harmless for any liability
//      incurred by, or claims asserted against, such Contributor by reason
//      of your accepting any such warranty or additional liability.
//
//   END OF TERMS AND CONDITIONS
//
//   APPENDIX: How to apply the Apache License to your work.
//
//      To apply the Apache License to your work, attach the following
//      boilerplate notice, with the fields enclosed by brackets "[]"
//      replaced with your own identifying information. (Don't include
//      the brackets!)  The text should be enclosed in the appropriate
//      comment syntax for the file format. We also recommend that a
//      file or class name and description of purpose be included on the
//      same "printed page" as the copyright notice for easier
//      identification within third-party archives.
//
//   Copyright [yyyy] [name of copyright owner]
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.

//! QoS (Quality of Service)
//! full credit goes to <https://github.com/rclrust/rclrust/blob/main/rclrust/src/qos.rs>

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
