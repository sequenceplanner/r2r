use std::ffi::CString;
use std::fmt::Debug;
use std::ops::{Deref, DerefMut};
use std::sync::{Arc, Mutex};

use crate::error::*;
use crate::log_guard;
use r2r_rcl::*;

/// A ROS context. Needed to create nodes etc.
#[derive(Debug, Clone)]
pub struct Context {
    pub(crate) context_handle: Arc<Mutex<ContextHandle>>,
}

unsafe impl Send for Context {}

impl Context {
    /// Create a ROS context.
    pub fn create() -> Result<Context> {
        let mut ctx: Box<rcl_context_t> = unsafe { Box::new(rcl_get_zero_initialized_context()) };
        // argc/v
        let args = std::env::args()
            .map(|arg| CString::new(arg).unwrap())
            .collect::<Vec<CString>>();
        let mut c_args = args
            .iter()
            .map(|arg| arg.as_ptr())
            .collect::<Vec<*const ::std::os::raw::c_char>>();
        c_args.push(std::ptr::null());

        let is_valid = unsafe {
            let allocator = rcutils_get_default_allocator();
            let mut init_options = rcl_get_zero_initialized_init_options();
            rcl_init_options_init(&mut init_options, allocator);
            rcl_init(
                (c_args.len() - 1) as ::std::os::raw::c_int,
                c_args.as_ptr(),
                &init_options,
                ctx.as_mut(),
            );
            rcl_init_options_fini(&mut init_options as *mut _);
            rcl_context_is_valid(ctx.as_mut())
        };

        let logging_ok = unsafe {
            let _guard = log_guard();
            let ret = rcl_logging_configure(
                &ctx.as_ref().global_arguments,
                &rcutils_get_default_allocator(),
            );
            ret == RCL_RET_OK as i32
        };

        if is_valid && logging_ok {
            Ok(Context {
                context_handle: Arc::new(Mutex::new(ContextHandle(ctx))),
            })
        } else {
            Err(Error::RCL_RET_ERROR) // TODO
        }
    }

    /// Check if the ROS context is valid.
    ///
    /// (This is abbreviated to rcl_ok() in the other bindings.)
    pub fn is_valid(&self) -> bool {
        let mut ctx = self.context_handle.lock().unwrap();
        unsafe { rcl_context_is_valid(ctx.as_mut()) }
    }
}

#[derive(Debug)]
pub struct ContextHandle(Box<rcl_context_t>);

impl Deref for ContextHandle {
    type Target = Box<rcl_context_t>;

    fn deref(&self) -> &Box<rcl_context_t> {
        &self.0
    }
}

impl DerefMut for ContextHandle {
    fn deref_mut(&mut self) -> &mut Box<rcl_context_t> {
        &mut self.0
    }
}

impl Drop for ContextHandle {
    fn drop(&mut self) {
        // TODO: error handling? atleast probably need rcl_reset_error
        unsafe {
            rcl_shutdown(self.0.as_mut());
            rcl_context_fini(self.0.as_mut());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_context_drop() -> () {
        {
            let ctx = Context::create().unwrap();
            assert!(ctx.is_valid());
        }
        {
            let ctx = Context::create().unwrap();
            assert!(ctx.is_valid());
        }
    }
}
