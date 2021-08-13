use super::*;

pub trait ActionServer_ {
    fn handle(&self) -> &rcl_action_server_t;
    fn handle_goal_request(&mut self, server: Arc<Mutex<dyn ActionServer_>>) -> ();
    fn handle_cancel_request(&mut self) -> ();
    fn handle_result_request(&mut self) -> ();
    fn handle_goal_expired(&mut self) -> ();
    fn publish_status(&self) -> ();
    fn add_result(&mut self, uuid: uuid::Uuid, msg: Box<dyn VoidPtr>) -> ();
    fn destroy(&mut self, node: &mut rcl_node_t);
}

pub struct WrappedActionServer<T>
where
    T: WrappedActionTypeSupport,
{
    pub rcl_handle: rcl_action_server_t,
    pub clock_handle: Box<rcl_clock_t>,
    pub accept_goal_cb: Box<dyn FnMut(&uuid::Uuid, &T::Goal) -> bool>,
    pub accept_cancel_cb: Box<dyn FnMut(&ServerGoal<T>) -> bool>,
    pub goal_cb: Box<dyn FnMut(ServerGoal<T>)>,
    pub goals: HashMap<uuid::Uuid, ServerGoal<T>>,
    pub result_msgs: HashMap<uuid::Uuid, Box<dyn VoidPtr>>,
    pub result_requests: HashMap<uuid::Uuid, Vec<rmw_request_id_t>>,
}

impl<T: 'static> ActionServer_ for WrappedActionServer<T>
where
    T: WrappedActionTypeSupport,
{
    fn handle(&self) -> &rcl_action_server_t {
        &self.rcl_handle
    }

    fn handle_goal_request(&mut self, server: Arc<Mutex<dyn ActionServer_>>) -> () {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut request_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Request,
        >::new();
        let ret = unsafe {
            rcl_action_take_goal_request(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                request_msg.void_ptr_mut(),
            )
        };

        if ret != RCL_RET_OK as i32 {
            // this seems normal if client dies.
            return;
        }
        let msg = <<<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Request>::from_native(&request_msg);
        let (uuid_msg, goal) = T::destructure_goal_request_msg(msg);
        let uuid = uuid::Uuid::from_bytes(vec_to_uuid_bytes(uuid_msg.uuid.clone()));
        let goal_accepted = (self.accept_goal_cb)(&uuid, &goal);
        let time = builtin_interfaces::msg::Time::default();

        let goal_info = action_msgs::msg::GoalInfo {
            goal_id: uuid_msg,
            stamp: time.clone(),
        };

        let native_goal_info = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::from(&goal_info);

        let goal_handle: Option<*mut rcl_action_goal_handle_t> = if goal_accepted {
            unsafe {
                Some(rcl_action_accept_new_goal(
                    &mut self.rcl_handle,
                    &*native_goal_info,
                ))
            }
        } else {
            None
        };

        // send response
        let response_msg = T::make_goal_response_msg(goal_accepted, time);
        let mut response_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Response,
        >::from(&response_msg);

        let ret = unsafe {
            let mut request_id = request_id.assume_init();
            rcl_action_send_goal_response(
                &self.rcl_handle,
                &mut request_id,
                response_msg.void_ptr_mut(),
            )
        };
        if ret != RCL_RET_OK as i32 {
            println!("action server: failed to send goal response");
            return;
        }

        // if we accepted the goal, update its state machine and publish all goal statuses
        if let Some(goal_handle) = goal_handle {
            unsafe {
                rcl_action_update_goal_state(
                    goal_handle,
                    rcl_action_goal_event_t::GOAL_EVENT_EXECUTE,
                );
            }

            self.publish_status();

            // run the user supplied cb with newly created goal handle object
            let g: ServerGoal<T> = ServerGoal {
                uuid,
                goal,
                handle: Arc::new(Mutex::new(goal_handle)),
                server: Arc::downgrade(&server),
            };

            self.goals.insert(uuid, g.clone());

            // start goal callback
            (self.goal_cb)(g);
        }
    }

    fn handle_cancel_request(&mut self) -> () {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut request_msg = WrappedNativeMsg::<action_msgs::srv::CancelGoal::Request>::new();
        let ret = unsafe {
            rcl_action_take_cancel_request(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                request_msg.void_ptr_mut(),
            )
        };

        if ret != RCL_RET_OK as i32 {
            // this seems normal if client dies.
            return;
        }

        let mut cancel_response = unsafe { rcl_action_get_zero_initialized_cancel_response() };
        let ret = unsafe {
            rcl_action_process_cancel_request(&self.rcl_handle, &*request_msg, &mut cancel_response)
        };

        if ret != RCL_RET_OK as i32 {
            println!("action server: could not process cancel request. {}", ret);
            return;
        }

        let mut response_msg =
            action_msgs::srv::CancelGoal::Response::from_native(&cancel_response.msg);

        // let user filter cancelled goals.
        let requested_cancels = response_msg.goals_canceling.len();
        response_msg.goals_canceling.retain(|goal_info| {
            let uuid = uuid::Uuid::from_bytes(vec_to_uuid_bytes(goal_info.goal_id.uuid.clone()));
            if let Some(goal) = self.goals.get(&uuid) {
                (self.accept_cancel_cb)(goal)
            } else {
                true
            }
        });

        response_msg.goals_canceling.iter().for_each(|goal_info| {
            let uuid = uuid::Uuid::from_bytes(vec_to_uuid_bytes(goal_info.goal_id.uuid.clone()));
            if let Some(goal) = self.goals.get_mut(&uuid) {
                goal.set_cancel();
            }
        });

        // check if all cancels were rejected.
        if requested_cancels >= 1 && response_msg.goals_canceling.is_empty() {
            response_msg.return_code = 1; // TODO: auto generate these (int8 ERROR_REJECTED=1)
        }

        if !response_msg.goals_canceling.is_empty() {
            // at least one goal state changed, publish a new status message
            self.publish_status();
        }

        let mut native_msg =
            WrappedNativeMsg::<action_msgs::srv::CancelGoal::Response>::from(&response_msg);
        let ret = unsafe {
            let mut request_id = request_id.assume_init();
            rcl_action_send_cancel_response(
                &self.rcl_handle,
                &mut request_id,
                native_msg.void_ptr_mut(),
            )
        };

        if ret != RCL_RET_OK as i32 {
            println!("action server: could send cancel response. {}", ret);
            return;
        }
    }

    fn handle_goal_expired(&mut self) {
        let mut goal_info = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::new();
        let mut num_expired = 1;

        while num_expired > 1 {
            let ret = unsafe {
                rcl_action_expire_goals(&self.rcl_handle, &mut *goal_info, 1, &mut num_expired)
            };
            if ret != RCL_RET_OK as i32 {
                println!("action server: could not expire goal.");
                return;
            }
            let gi = action_msgs::msg::GoalInfo::from_native(&goal_info);
            let uuid = uuid::Uuid::from_bytes(vec_to_uuid_bytes(gi.goal_id.uuid.clone()));
            println!("goal expired: {} - {}", uuid, num_expired);
            self.goals.remove(&uuid);
            self.result_msgs.remove(&uuid);
            self.result_requests.remove(&uuid);
        }
    }

    fn publish_status(&self) {
        unsafe {
            let mut status = rcl_action_get_zero_initialized_goal_status_array();
            let ret = rcl_action_get_goal_status_array(&self.rcl_handle, &mut status);
            if ret != RCL_RET_OK as i32 {
                println!(
                    "action server: failed to get goal status array: {}",
                    Error::from_rcl_error(ret)
                );
                return;
            }
            let ret = rcl_action_publish_status(
                &self.rcl_handle,
                &status as *const _ as *const std::os::raw::c_void,
            );
            if ret != RCL_RET_OK as i32 {
                println!(
                    "action server: failed to publish status: {}",
                    Error::from_rcl_error(ret)
                );
                return;
            }
            rcl_action_goal_status_array_fini(&mut status);
        }
    }

    // bit of a hack...
    fn add_result(&mut self, uuid: uuid::Uuid, mut msg: Box<dyn VoidPtr>) -> () {
        // if there are already requests for this goal, send the result immediately.
        if let Some(rr) = self.result_requests.remove(&uuid) {
            for mut req in rr {
                let ret = unsafe {
                    rcl_action_send_result_response(&self.rcl_handle, &mut req, msg.void_ptr_mut())
                };
                if ret != RCL_RET_OK as i32 {
                    println!(
                        "action server: could send result request response. {}",
                        Error::from_rcl_error(ret)
                    );
                }
            }
        }
        self.result_msgs.insert(uuid, msg);
    }

    fn handle_result_request(&mut self) -> () {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut request_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Request,
        >::new();
        let ret = unsafe {
            rcl_action_take_result_request(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                request_msg.void_ptr_mut(),
            )
        };

        if ret != RCL_RET_OK as i32 {
            // this seems normal if client dies.
            return;
        }

        let msg = <<<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Request>::from_native(&request_msg);
        let goal_info = action_msgs::msg::GoalInfo {
            goal_id: T::destructure_result_request_msg(msg),
            ..action_msgs::msg::GoalInfo::default()
        };
        let goal_info_native = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::from(&goal_info);

        // does this goal exist?
        let goal_exists =
            unsafe { rcl_action_server_goal_exists(&self.rcl_handle, &*goal_info_native) };

        let uuid = uuid::Uuid::from_bytes(vec_to_uuid_bytes(goal_info.goal_id.uuid.clone()));

        let response_msg = if !goal_exists {
            // Goal does not exists
            println!("goal does not exist :(");
            let status = GoalStatus::Unknown;
            let msg = T::make_result_response_msg(status.to_rcl(), T::Result::default());
            let mut response_msg = WrappedNativeMsg::<
                <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
            >::from(&msg);
            Some(response_msg.void_ptr_mut())
        } else {
            self.result_msgs
                .get_mut(&uuid)
                .map(|msg| msg.void_ptr_mut())
        };

        let mut request_id = unsafe { request_id.assume_init() };
        if let Some(response_msg) = response_msg {
            let ret = unsafe {
                rcl_action_send_result_response(&self.rcl_handle, &mut request_id, response_msg)
            };

            if ret != RCL_RET_OK as i32 {
                println!(
                    "action server: could send result request response. {}",
                    Error::from_rcl_error(ret)
                );
                return;
            }
        } else {
            // keep request for later when result comes in
            // todo: add logic that replies to the requests
            self.result_requests
                .entry(uuid)
                .or_insert(vec![])
                .push(request_id);
        }
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_action_server_fini(&mut self.rcl_handle, node);
            rcl_ros_clock_fini(self.clock_handle.as_mut());
        }
    }
}

#[derive(Clone)]
pub struct ServerGoal<T>
where
    T: WrappedActionTypeSupport,
{
    pub uuid: uuid::Uuid,
    pub goal: T::Goal,
    handle: Arc<Mutex<*mut rcl_action_goal_handle_t>>,
    server: Weak<Mutex<dyn ActionServer_>>,
}

unsafe impl<T> Send for ServerGoal<T> where T: WrappedActionTypeSupport {}

impl<T: 'static> ServerGoal<T>
where
    T: WrappedActionTypeSupport,
{
    pub fn is_cancelling(&self) -> bool {
        let mut state = 0u8; // TODO: int8 STATUS_UNKNOWN   = 0;
        let ret = unsafe {
            let handle = self.handle.lock().unwrap();
            rcl_action_goal_handle_get_status(*handle, &mut state)
        };

        if ret != RCL_RET_OK as i32 {
            println!("action server: Failed to get goal handle state: {}", ret);
        }
        return state == 3u8; // TODO: int8 STATUS_CANCELING
    }

    pub fn publish_feedback(&self, msg: T::Feedback) -> Result<()>
    where
        T: WrappedActionTypeSupport,
    {
        // upgrade to actual ref. if still alive
        let action_server = self
            .server
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_SERVER_INVALID)?;

        let uuid_msg = unique_identifier_msgs::msg::UUID {
            uuid: self.uuid.as_bytes().to_vec(),
        };
        let feedback_msg = T::make_feedback_msg(uuid_msg, msg);
        let mut native_msg = WrappedNativeMsg::<T::FeedbackMessage>::from(&feedback_msg);
        let ret = unsafe {
            rcl_action_publish_feedback(
                action_server.lock().unwrap().handle(),
                native_msg.void_ptr_mut(),
            )
        };

        if ret == RCL_RET_OK as i32 {
            Ok(())
        } else {
            eprintln!("coult not publish {}", Error::from_rcl_error(ret));
            Ok(()) // todo: error codes
        }
    }

    fn set_cancel(&mut self) {
        let ret = unsafe {
            let handle = self.handle.lock().unwrap();
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_t::GOAL_EVENT_CANCEL_GOAL)
        };

        if ret != RCL_RET_OK as i32 {
            println!(
                "action server: could not cancel goal: {}",
                Error::from_rcl_error(ret)
            );
        }
    }

    pub fn cancel(&mut self, msg: T::Result) -> Result<()> {
        // upgrade to actual ref. if still alive
        let action_server = self
            .server
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_SERVER_INVALID)?;
        let mut action_server = action_server.lock().unwrap();

        // todo: check that the goal exists
        let goal_info = action_msgs::msg::GoalInfo {
            goal_id: unique_identifier_msgs::msg::UUID {
                uuid: self.uuid.as_bytes().to_vec(),
            },
            ..action_msgs::msg::GoalInfo::default()
        };
        let goal_info_native = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::from(&goal_info);

        // does this goal exist?
        let goal_exists =
            unsafe { rcl_action_server_goal_exists(action_server.handle(), &*goal_info_native) };

        if !goal_exists {
            println!("tried to publish result without a goal");
            return Err(Error::RCL_RET_ACTION_GOAL_HANDLE_INVALID);
        }

        // todo: error handling
        unsafe {
            rcl_action_notify_goal_done(action_server.handle());
        }

        // send out updated statues
        action_server.publish_status();

        // create result message
        let result_msg = T::make_result_response_msg(5, msg); // todo: int8 STATUS_CANCELED  = 5
        let native_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
        >::from(&result_msg);
        action_server.add_result(self.uuid.clone(), Box::new(native_msg));

        Ok(())
    }

    pub fn abort(&mut self, msg: T::Result) -> Result<()> {
        // todo: error handling
        let ret = unsafe {
            let handle = self.handle.lock().unwrap();
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_t::GOAL_EVENT_ABORT)
        };

        if ret != RCL_RET_OK as i32 {
            println!(
                "action server: could not cancel goal: {}",
                Error::from_rcl_error(ret)
            );
        }

        // upgrade to actual ref. if still alive
        let action_server = self
            .server
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_SERVER_INVALID)?;
        let mut action_server = action_server.lock().unwrap();

        // todo: check that the goal exists
        let goal_info = action_msgs::msg::GoalInfo {
            goal_id: unique_identifier_msgs::msg::UUID {
                uuid: self.uuid.as_bytes().to_vec(),
            },
            ..action_msgs::msg::GoalInfo::default()
        };
        let goal_info_native = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::from(&goal_info);

        // does this goal exist?
        let goal_exists =
            unsafe { rcl_action_server_goal_exists(action_server.handle(), &*goal_info_native) };

        if !goal_exists {
            println!("tried to abort without a goal");
            return Err(Error::RCL_RET_ACTION_GOAL_HANDLE_INVALID);
        }

        // todo: error handling
        unsafe {
            rcl_action_notify_goal_done(action_server.handle());
        }

        // send out updated statues
        action_server.publish_status();

        // create result message
        let result_msg = T::make_result_response_msg(6, msg); // todo: int8 STATUS_ABORTED   = 6
        let native_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
        >::from(&result_msg);
        action_server.add_result(self.uuid.clone(), Box::new(native_msg));

        Ok(())
    }

    pub fn succeed(&mut self, msg: T::Result) -> Result<()>
    where
        T: WrappedActionTypeSupport,
    {
        // upgrade to actual ref. if still alive
        let action_server = self
            .server
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_SERVER_INVALID)?;
        let mut action_server = action_server.lock().unwrap();

        // todo: check that the goal exists
        let goal_info = action_msgs::msg::GoalInfo {
            goal_id: unique_identifier_msgs::msg::UUID {
                uuid: self.uuid.as_bytes().to_vec(),
            },
            ..action_msgs::msg::GoalInfo::default()
        };
        let goal_info_native = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::from(&goal_info);

        // does this goal exist?
        let goal_exists =
            unsafe { rcl_action_server_goal_exists(action_server.handle(), &*goal_info_native) };

        if !goal_exists {
            println!("tried to publish result without a goal");
            return Err(Error::RCL_RET_ACTION_GOAL_HANDLE_INVALID);
        }

        // todo: error handling
        unsafe {
            let handle = self.handle.lock().unwrap();
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_t::GOAL_EVENT_SUCCEED);
        }

        // todo: error handling
        unsafe {
            rcl_action_notify_goal_done(action_server.handle());
        }

        // send out updated statues
        action_server.publish_status();

        // create result message
        let result_msg = T::make_result_response_msg(4, msg); // todo: int8 STATUS_SUCCEEDED = 4
        let native_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
        >::from(&result_msg);
        action_server.add_result(self.uuid.clone(), Box::new(native_msg));

        Ok(())
    }
}
