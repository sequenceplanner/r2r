use futures::channel::{mpsc, oneshot};
use std::ffi::CString;
use std::mem::MaybeUninit;
use std::sync::{Arc, Mutex, Weak};

use crate::error::*;
use crate::msg_types::*;
use r2r_rcl::*;

/// Encapsulates a service request.
///
/// In contrast to having a callback from Request -> Response
/// types that is called synchronously, the service request can be
/// moved around and completed asynchronously.
///
/// To complete the request, call the `respond` function.
#[derive(Clone)]
pub struct ServiceRequest<T>
where
    T: WrappedServiceTypeSupport,
{
    pub message: T::Request,
    request_id: rmw_request_id_t,
    service: Weak<Mutex<dyn Service_>>,
}

unsafe impl<T> Send for ServiceRequest<T> where T: WrappedServiceTypeSupport {}

impl<T> ServiceRequest<T>
where
    T: 'static + WrappedServiceTypeSupport,
{
    /// Complete the service request, consuming the request in the process.
    pub fn respond(self, msg: T::Response) -> Result<()> {
        let service = self
            .service
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_SERVER_INVALID)?;
        let mut service = service.lock().unwrap();
        let native_msg = WrappedNativeMsg::<T::Response>::from(&msg);
        service.send_response(self.request_id, Box::new(native_msg))
    }
}

pub trait Service_ {
    fn handle(&self) -> &rcl_service_t;
    fn send_response(&mut self, request_id: rmw_request_id_t, msg: Box<dyn VoidPtr>) -> Result<()>;
    /// Returns true if the service stream has been dropped.
    fn handle_request(&mut self, service: Arc<Mutex<dyn Service_>>) -> bool;
    fn destroy(&mut self, node: &mut rcl_node_t);
}

pub struct TypedService<T>
where
    T: WrappedServiceTypeSupport,
{
    pub rcl_handle: rcl_service_t,
    pub sender: mpsc::Sender<ServiceRequest<T>>,
    pub outstanding_requests: Vec<oneshot::Receiver<(rmw_request_id_t, T::Response)>>,
}

impl<T: 'static> Service_ for TypedService<T>
where
    T: WrappedServiceTypeSupport,
{
    fn handle(&self) -> &rcl_service_t {
        &self.rcl_handle
    }

    fn send_response(
        &mut self,
        mut request_id: rmw_request_id_t,
        mut msg: Box<dyn VoidPtr>,
    ) -> Result<()> {
        let res =
            unsafe { rcl_send_response(&self.rcl_handle, &mut request_id, msg.void_ptr_mut()) };
        if res == RCL_RET_OK as i32 {
            Ok(())
        } else {
            Err(Error::from_rcl_error(res))
        }
    }

    fn handle_request(&mut self, service: Arc<Mutex<dyn Service_>>) -> bool {
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
            let request = ServiceRequest::<T> {
                message: request_msg,
                request_id,
                service: Arc::downgrade(&service),
            };
            if let Err(e) = self.sender.try_send(request) {
                if e.is_disconnected() {
                    return true;
                }
                eprintln!("warning: could not send service request ({})", e)
            }
        } // TODO handle failure.
        false
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_service_fini(&mut self.rcl_handle, node);
        }
    }
}

pub fn create_service_helper(
    node: &mut rcl_node_t,
    service_name: &str,
    service_ts: *const rosidl_service_type_support_t,
) -> Result<rcl_service_t> {
    let mut service_handle = unsafe { rcl_get_zero_initialized_service() };
    let service_name_c_string =
        CString::new(service_name).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

    let result = unsafe {
        let service_options = rcl_service_get_default_options();
        rcl_service_init(
            &mut service_handle,
            node,
            service_ts,
            service_name_c_string.as_ptr(),
            &service_options,
        )
    };
    if result == RCL_RET_OK as i32 {
        Ok(service_handle)
    } else {
        Err(Error::from_rcl_error(result))
    }
}
