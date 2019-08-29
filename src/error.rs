#![allow(non_camel_case_types)]
use rcl::*;

// TODO

#[derive(Debug, Fail)]
pub enum Error {
    // Copied from the generated bindgen
    #[fail(display = "RCL_RET_OK")]
    RCL_RET_OK,
    #[fail(display = "RCL_RET_ERROR")]
    RCL_RET_ERROR,
    #[fail(display = "RCL_RET_TIMEOUT")]
    RCL_RET_TIMEOUT,
    #[fail(display = "RCL_RET_BAD_ALLOC")]
    RCL_RET_BAD_ALLOC,
    #[fail(display = "RCL_RET_INVALID_ARGUMENT")]
    RCL_RET_INVALID_ARGUMENT,
    #[fail(display = "RCL_RET_UNSUPPORTED")]
    RCL_RET_UNSUPPORTED,
    #[fail(display = "RCL_RET_ALREADY_INIT")]
    RCL_RET_ALREADY_INIT,
    #[fail(display = "RCL_RET_NOT_INIT")]
    RCL_RET_NOT_INIT,
    #[fail(display = "RCL_RET_MISMATCHED_RMW_ID")]
    RCL_RET_MISMATCHED_RMW_ID,
    #[fail(display = "RCL_RET_TOPIC_NAME_INVALID")]
    RCL_RET_TOPIC_NAME_INVALID,
    #[fail(display = "RCL_RET_SERVICE_NAME_INVALID")]
    RCL_RET_SERVICE_NAME_INVALID,
    #[fail(display = "RCL_RET_UNKNOWN_SUBSTITUTION")]
    RCL_RET_UNKNOWN_SUBSTITUTION,
    #[fail(display = "RCL_RET_ALREADY_SHUTDOWN")]
    RCL_RET_ALREADY_SHUTDOWN,
    #[fail(display = "RCL_RET_NODE_INVALID")]
    RCL_RET_NODE_INVALID,
    #[fail(display = "RCL_RET_NODE_INVALID_NAME")]
    RCL_RET_NODE_INVALID_NAME,
    #[fail(display = "RCL_RET_NODE_INVALID_NAMESPACE")]
    RCL_RET_NODE_INVALID_NAMESPACE,
    #[fail(display = "RCL_RET_PUBLISHER_INVALID")]
    RCL_RET_PUBLISHER_INVALID,
    #[fail(display = "RCL_RET_SUBSCRIPTION_INVALID")]
    RCL_RET_SUBSCRIPTION_INVALID,
    #[fail(display = "RCL_RET_SUBSCRIPTION_TAKE_FAILED")]
    RCL_RET_SUBSCRIPTION_TAKE_FAILED,
    #[fail(display = "RCL_RET_CLIENT_INVALID")]
    RCL_RET_CLIENT_INVALID,
    #[fail(display = "RCL_RET_CLIENT_TAKE_FAILED")]
    RCL_RET_CLIENT_TAKE_FAILED,
    #[fail(display = "RCL_RET_SERVICE_INVALID")]
    RCL_RET_SERVICE_INVALID,
    #[fail(display = "RCL_RET_SERVICE_TAKE_FAILED")]
    RCL_RET_SERVICE_TAKE_FAILED,
    #[fail(display = "RCL_RET_TIMER_INVALID")]
    RCL_RET_TIMER_INVALID,
    #[fail(display = "RCL_RET_TIMER_CANCELED")]
    RCL_RET_TIMER_CANCELED,
    #[fail(display = "RCL_RET_WAIT_SET_INVALID")]
    RCL_RET_WAIT_SET_INVALID,
    #[fail(display = "RCL_RET_WAIT_SET_EMPTY")]
    RCL_RET_WAIT_SET_EMPTY,
    #[fail(display = "RCL_RET_WAIT_SET_FULL")]
    RCL_RET_WAIT_SET_FULL,
    #[fail(display = "RCL_RET_INVALID_REMAP_RULE")]
    RCL_RET_INVALID_REMAP_RULE,
    #[fail(display = "RCL_RET_WRONG_LEXEME")]
    RCL_RET_WRONG_LEXEME,
    #[fail(display = "RCL_RET_INVALID_PARAM_RULE")]
    RCL_RET_INVALID_PARAM_RULE,
    #[fail(display = "RCL_RET_INVALID_LOG_LEVEL_RULE")]
    RCL_RET_INVALID_LOG_LEVEL_RULE,
    #[fail(display = "RCL_RET_EVENT_INVALID")]
    RCL_RET_EVENT_INVALID,
    #[fail(display = "RCL_RET_EVENT_TAKE_FAILED")]
    RCL_RET_EVENT_TAKE_FAILED,

    // Our own errors
    #[fail(display = "No typesupport build for the message type: {}", msgtype)]
    InvalidMessageType { msgtype: String },
    #[fail(display = "Serde error: {}", err)]
    SerdeError { err: String },
}

impl Error {
    pub fn from_rcl_error(e: i32) -> Self {
        let e = e as u32;
        match e {
            _ if e == RCL_RET_OK => Error::RCL_RET_OK,
            _ if e == RCL_RET_ERROR => Error::RCL_RET_ERROR,
            _ if e == RCL_RET_TIMEOUT => Error::RCL_RET_TIMEOUT,
            _ if e == RCL_RET_BAD_ALLOC => Error::RCL_RET_BAD_ALLOC,
            _ if e == RCL_RET_INVALID_ARGUMENT => Error::RCL_RET_INVALID_ARGUMENT,
            _ if e == RCL_RET_UNSUPPORTED => Error::RCL_RET_UNSUPPORTED,
            _ if e == RCL_RET_ALREADY_INIT => Error::RCL_RET_ALREADY_INIT,
            _ if e == RCL_RET_NOT_INIT => Error::RCL_RET_NOT_INIT,
            _ if e == RCL_RET_MISMATCHED_RMW_ID => Error::RCL_RET_MISMATCHED_RMW_ID,
            _ if e == RCL_RET_TOPIC_NAME_INVALID => Error::RCL_RET_TOPIC_NAME_INVALID,
            _ if e == RCL_RET_SERVICE_NAME_INVALID => Error::RCL_RET_SERVICE_NAME_INVALID,
            _ if e == RCL_RET_UNKNOWN_SUBSTITUTION => Error::RCL_RET_UNKNOWN_SUBSTITUTION,
            _ if e == RCL_RET_ALREADY_SHUTDOWN => Error::RCL_RET_ALREADY_SHUTDOWN,
            _ if e == RCL_RET_NODE_INVALID => Error::RCL_RET_NODE_INVALID,
            _ if e == RCL_RET_NODE_INVALID_NAME => Error::RCL_RET_NODE_INVALID_NAME,
            _ if e == RCL_RET_NODE_INVALID_NAMESPACE => Error::RCL_RET_NODE_INVALID_NAMESPACE,
            _ if e == RCL_RET_PUBLISHER_INVALID => Error::RCL_RET_PUBLISHER_INVALID,
            _ if e == RCL_RET_SUBSCRIPTION_INVALID => Error::RCL_RET_SUBSCRIPTION_INVALID,
            _ if e == RCL_RET_SUBSCRIPTION_TAKE_FAILED => Error::RCL_RET_SUBSCRIPTION_TAKE_FAILED,
            _ if e == RCL_RET_CLIENT_INVALID => Error::RCL_RET_CLIENT_INVALID,
            _ if e == RCL_RET_CLIENT_TAKE_FAILED => Error::RCL_RET_CLIENT_TAKE_FAILED,
            _ if e == RCL_RET_SERVICE_INVALID => Error::RCL_RET_SERVICE_INVALID,
            _ if e == RCL_RET_SERVICE_TAKE_FAILED => Error::RCL_RET_SERVICE_TAKE_FAILED,
            _ if e == RCL_RET_TIMER_INVALID => Error::RCL_RET_TIMER_INVALID,
            _ if e == RCL_RET_TIMER_CANCELED => Error::RCL_RET_TIMER_CANCELED,
            _ if e == RCL_RET_WAIT_SET_INVALID => Error::RCL_RET_WAIT_SET_INVALID,
            _ if e == RCL_RET_WAIT_SET_EMPTY => Error::RCL_RET_WAIT_SET_EMPTY,
            _ if e == RCL_RET_WAIT_SET_FULL => Error::RCL_RET_WAIT_SET_FULL,
            _ if e == RCL_RET_INVALID_REMAP_RULE => Error::RCL_RET_INVALID_REMAP_RULE,
            _ if e == RCL_RET_WRONG_LEXEME => Error::RCL_RET_WRONG_LEXEME,
            _ if e == RCL_RET_INVALID_PARAM_RULE => Error::RCL_RET_INVALID_PARAM_RULE,
            _ if e == RCL_RET_INVALID_LOG_LEVEL_RULE => Error::RCL_RET_INVALID_LOG_LEVEL_RULE,
            _ if e == RCL_RET_EVENT_INVALID => Error::RCL_RET_EVENT_INVALID,
            _ if e == RCL_RET_EVENT_TAKE_FAILED => Error::RCL_RET_EVENT_TAKE_FAILED,
            _ => panic!("TODO: add error code {}", e),
        }
    }
}
