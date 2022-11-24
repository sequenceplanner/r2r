use futures::channel::oneshot;
use futures::TryFutureExt;
use std::ffi::CString;
use std::future::Future;
use std::mem::MaybeUninit;
use std::sync::{Mutex, Weak};

use crate::error::*;
use crate::msg_types::*;
use r2r_rcl::*;

/// ROS service client.
///
/// This is a handle to a service client wrapped in a `Mutex` inside a
/// `Weak` `Arc`. As such you can pass it between threads safely.
pub struct Client<T>
where
    T: WrappedServiceTypeSupport,
{
    client: Weak<Mutex<TypedClient<T>>>,
}

impl<T: 'static> Client<T>
where
    T: WrappedServiceTypeSupport,
{
    /// Make a service request.
    ///
    /// Returns a `Future` of the `Response` type.
    pub fn request(&self, msg: &T::Request) -> Result<impl Future<Output = Result<T::Response>>>
    where
        T: WrappedServiceTypeSupport,
    {
        // upgrade to actual ref. if still alive
        let client = self.client.upgrade().ok_or(Error::RCL_RET_CLIENT_INVALID)?;
        let mut client = client.lock().unwrap();
        client.request(msg)
    }
}

/// ROS "untyped" service client.
///
/// The untyped client is useful when you don't know the concrete type
/// at compile time. Messages are represented by `serde_json::Value`.
///
/// This is a handle to a service client wrapped in a `Mutex` inside a
/// `Weak` `Arc`. As such you can pass it between threads safely.
pub struct ClientUntyped {
    client: Weak<Mutex<UntypedClient_>>,
}

impl ClientUntyped {
    /// Make an "untyped" service request.
    ///
    /// The request is a `serde_json::Value`. It is up to the user to
    /// make sure the fields in the json object are correct.
    ///
    /// Returns a `Future` of Result<serde_json::Value>.
    pub fn request(
        &self,
        msg: serde_json::Value,
    ) -> Result<impl Future<Output = Result<Result<serde_json::Value>>>> {
        // upgrade to actual ref. if still alive
        let client = self.client.upgrade().ok_or(Error::RCL_RET_CLIENT_INVALID)?;
        let mut client = client.lock().unwrap();
        client.request(msg)
    }
}

pub fn make_client<T>(client: Weak<Mutex<TypedClient<T>>>) -> Client<T>
where
    T: WrappedServiceTypeSupport,
{
    Client { client }
}

pub fn make_untyped_client(client: Weak<Mutex<UntypedClient_>>) -> ClientUntyped {
    ClientUntyped { client }
}

unsafe impl<T> Send for TypedClient<T> where T: WrappedServiceTypeSupport {}

impl<T: 'static> TypedClient<T>
where
    T: WrappedServiceTypeSupport,
{
    pub fn request(&mut self, msg: &T::Request) -> Result<impl Future<Output = Result<T::Response>>>
    where
        T: WrappedServiceTypeSupport,
    {
        let native_msg: WrappedNativeMsg<T::Request> = WrappedNativeMsg::<T::Request>::from(msg);
        let mut seq_no = 0i64;
        let result =
            unsafe { rcl_send_request(&self.rcl_handle, native_msg.void_ptr(), &mut seq_no) };

        let (sender, receiver) = oneshot::channel::<T::Response>();

        if result == RCL_RET_OK as i32 {
            self.response_channels.push((seq_no, sender));
            // instead of "canceled" we return invalid client.
            Ok(receiver.map_err(|_| Error::RCL_RET_CLIENT_INVALID))
        } else {
            eprintln!("coult not send request {}", result);
            Err(Error::from_rcl_error(result))
        }
    }
}

unsafe impl Send for UntypedClient_ {}

impl UntypedClient_ {
    pub fn request(
        &mut self,
        msg: serde_json::Value,
    ) -> Result<impl Future<Output = Result<Result<serde_json::Value>>>> {
        let mut native_msg = (self.service_type.make_request_msg)();
        native_msg.from_json(msg)?;

        let mut seq_no = 0i64;
        let result =
            unsafe { rcl_send_request(&self.rcl_handle, native_msg.void_ptr(), &mut seq_no) };

        let (sender, receiver) = oneshot::channel::<Result<serde_json::Value>>();

        if result == RCL_RET_OK as i32 {
            self.response_channels.push((seq_no, sender));
            // instead of "canceled" we return invalid client.
            Ok(receiver.map_err(|_| Error::RCL_RET_CLIENT_INVALID))
        } else {
            eprintln!("coult not send request {}", result);
            Err(Error::from_rcl_error(result))
        }
    }
}

pub trait Client_ {
    fn handle(&self) -> &rcl_client_t;
    fn handle_response(&mut self);
    fn register_poll_available(&mut self, s: oneshot::Sender<()>);
    fn poll_available(&mut self, node: &mut rcl_node_t);
    fn destroy(&mut self, node: &mut rcl_node_t);
}

pub struct TypedClient<T>
where
    T: WrappedServiceTypeSupport,
{
    pub rcl_handle: rcl_client_t,
    pub response_channels: Vec<(i64, oneshot::Sender<T::Response>)>,
    pub poll_available_channels: Vec<oneshot::Sender<()>>,
}

impl<T: 'static> Client_ for TypedClient<T>
where
    T: WrappedServiceTypeSupport,
{
    fn handle(&self) -> &rcl_client_t {
        &self.rcl_handle
    }

    fn handle_response(&mut self) {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut response_msg = WrappedNativeMsg::<T::Response>::new();

        let ret = unsafe {
            rcl_take_response(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                response_msg.void_ptr_mut(),
            )
        };
        if ret == RCL_RET_OK as i32 {
            let request_id = unsafe { request_id.assume_init() };
            if let Some(idx) = self
                .response_channels
                .iter()
                .position(|(id, _)| id == &request_id.sequence_number)
            {
                let (_, sender) = self.response_channels.swap_remove(idx);
                let response = T::Response::from_native(&response_msg);
                match sender.send(response) {
                    Ok(()) => {}
                    Err(e) => {
                        println!("error sending to client: {:?}", e);
                    }
                }
            } else {
                let we_have: String = self
                    .response_channels
                    .iter()
                    .map(|(id, _)| id.to_string())
                    .collect::<Vec<_>>()
                    .join(",");
                eprintln!(
                    "no such req id: {}, we have [{}], ignoring",
                    request_id.sequence_number, we_have
                );
            }
        } // TODO handle failure.
    }

    fn register_poll_available(&mut self, s: oneshot::Sender<()>) {
        self.poll_available_channels.push(s);
    }

    fn poll_available(&mut self, node: &mut rcl_node_t) {
        if self.poll_available_channels.is_empty() {
            return;
        }
        let available = service_available_helper(node, self.handle());
        match available {
            Ok(true) => {
                // send ok and close channels
                while let Some(sender) = self.poll_available_channels.pop() {
                    let _res = sender.send(()); // we ignore if receiver dropped.
                }
            }
            Ok(false) => {
                // not available...
            }
            Err(_) => {
                // error, close all channels
                self.poll_available_channels.clear();
            }
        }
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_client_fini(&mut self.rcl_handle, node);
        }
    }
}

pub struct UntypedClient_ {
    pub service_type: UntypedServiceSupport,
    pub rcl_handle: rcl_client_t,
    pub response_channels: Vec<(i64, oneshot::Sender<Result<serde_json::Value>>)>,
    pub poll_available_channels: Vec<oneshot::Sender<()>>,
}

impl Client_ for UntypedClient_ {
    fn handle(&self) -> &rcl_client_t {
        &self.rcl_handle
    }

    fn handle_response(&mut self) {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut response_msg = (self.service_type.make_response_msg)();

        let ret = unsafe {
            rcl_take_response(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                response_msg.void_ptr_mut(),
            )
        };
        if ret == RCL_RET_OK as i32 {
            let request_id = unsafe { request_id.assume_init() };
            if let Some(idx) = self
                .response_channels
                .iter()
                .position(|(id, _)| id == &request_id.sequence_number)
            {
                let (_, sender) = self.response_channels.swap_remove(idx);
                let response = response_msg.to_json();
                match sender.send(response) {
                    Ok(()) => {}
                    Err(e) => {
                        println!("error sending to client: {:?}", e);
                    }
                }
            } else {
                let we_have: String = self
                    .response_channels
                    .iter()
                    .map(|(id, _)| id.to_string())
                    .collect::<Vec<_>>()
                    .join(",");
                eprintln!(
                    "no such req id: {}, we have [{}], ignoring",
                    request_id.sequence_number, we_have
                );
            }
        } // TODO handle failure.
    }

    fn register_poll_available(&mut self, s: oneshot::Sender<()>) {
        self.poll_available_channels.push(s);
    }

    fn poll_available(&mut self, node: &mut rcl_node_t) {
        if self.poll_available_channels.is_empty() {
            return;
        }
        let available = service_available_helper(node, self.handle());
        match available {
            Ok(true) => {
                // send ok and close channels
                while let Some(sender) = self.poll_available_channels.pop() {
                    let _res = sender.send(()); // we ignore if receiver dropped.
                }
            }
            Ok(false) => {
                // not available...
            }
            Err(_) => {
                // error, close all channels
                self.poll_available_channels.clear();
            }
        }
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_client_fini(&mut self.rcl_handle, node);
        }
    }
}

pub fn create_client_helper(
    node: *mut rcl_node_t,
    service_name: &str,
    service_ts: *const rosidl_service_type_support_t,
) -> Result<rcl_client_t> {
    let mut client_handle = unsafe { rcl_get_zero_initialized_client() };
    let service_name_c_string =
        CString::new(service_name).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

    let result = unsafe {
        let client_options = rcl_client_get_default_options();
        rcl_client_init(
            &mut client_handle,
            node,
            service_ts,
            service_name_c_string.as_ptr(),
            &client_options,
        )
    };
    if result == RCL_RET_OK as i32 {
        Ok(client_handle)
    } else {
        Err(Error::from_rcl_error(result))
    }
}

pub fn service_available_helper(node: &mut rcl_node_t, client: &rcl_client_t) -> Result<bool> {
    let mut avail = false;
    let result = unsafe { rcl_service_server_is_available(node, client, &mut avail) };

    if result == RCL_RET_OK as i32 {
        Ok(avail)
    } else {
        Err(Error::from_rcl_error(result))
    }
}

use crate::nodes::IsAvailablePollable;

impl<T: 'static> IsAvailablePollable for Client<T>
where
    T: WrappedServiceTypeSupport,
{
    fn register_poll_available(&self, sender: oneshot::Sender<()>) -> Result<()> {
        let client = self.client.upgrade().ok_or(Error::RCL_RET_CLIENT_INVALID)?;
        let mut client = client.lock().unwrap();
        client.register_poll_available(sender);
        Ok(())
    }
}

impl IsAvailablePollable for ClientUntyped {
    fn register_poll_available(&self, sender: oneshot::Sender<()>) -> Result<()> {
        let client = self.client.upgrade().ok_or(Error::RCL_RET_CLIENT_INVALID)?;
        let mut client = client.lock().unwrap();
        client.register_poll_available(sender);
        Ok(())
    }
}
