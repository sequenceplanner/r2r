use super::*;

pub trait Service {
    fn handle(&self) -> &rcl_service_t;
    fn send_completed_responses(&mut self) -> ();
    fn handle_request(&mut self) -> ();
    fn destroy(&mut self, node: &mut rcl_node_t) -> ();
}

pub struct TypedService<T>
where
    T: WrappedServiceTypeSupport,
{
    pub rcl_handle: rcl_service_t,
    pub sender: mpsc::Sender<ServiceRequest<T>>,
    pub outstanding_requests: Vec<oneshot::Receiver<(rmw_request_id_t, T::Response)>>,
}

impl<T: 'static> Service for TypedService<T>
where
    T: WrappedServiceTypeSupport,
{
    fn handle(&self) -> &rcl_service_t {
        &self.rcl_handle
    }

    fn send_completed_responses(&mut self) -> () {
        let mut to_send = vec![];
        self.outstanding_requests.retain_mut(|r| {
            match r.try_recv() {
                Ok(Some(resp)) => {
                    to_send.push(resp);
                    false // done with this.
                }
                Ok(None) => true, // keep message, waiting for service
                Err(_) => false,  // channel canceled
            }
        });

        for (mut req_id, msg) in to_send {
            let mut native_response = WrappedNativeMsg::<T::Response>::from(&msg);
            let res = unsafe {
                rcl_send_response(
                    &self.rcl_handle,
                    &mut req_id,
                    native_response.void_ptr_mut(),
                )
            };

            // TODO
            if res != RCL_RET_OK as i32 {
                eprintln!("could not send service response {}", res);
            }
        }
    }

    fn handle_request(&mut self) -> () {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut request_msg = WrappedNativeMsg::<T::Request>::new();

        let ret = unsafe {
            rcl_take_request(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                request_msg.void_ptr_mut(),
            )
        };
        if ret == RCL_RET_OK as i32 {
            let request_id = unsafe { request_id.assume_init() };
            let request_msg = T::Request::from_native(&request_msg);
            let (response_sender, response_receiver) =
                oneshot::channel::<(rmw_request_id_t, T::Response)>();
            self.outstanding_requests.push(response_receiver);
            let request = ServiceRequest::<T> {
                message: request_msg,
                request_id,
                response_sender,
            };
            match self.sender.try_send(request) {
                Err(e) => eprintln!("warning: could not send service request ({})", e),
                _ => (),
            }
        } // TODO handle failure.
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_service_fini(&mut self.rcl_handle, node);
        }
    }
}
