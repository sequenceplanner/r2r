/// Generates two versions of each function definition wrapped via this macro:
/// - One with tracing enabled: Adds a feature flag and copies the function signature and body as is.
/// - One with tracing disabled: Adds a feature flag and replaces the function body to be empty.
macro_rules! tracepoint_fn {
    ($($(#[$atts:meta])* $vi:vis fn $name:ident$(<$generic:tt>)?($($arg_name:ident : $arg_ty:ty),* $(,)?) $(-> $ret_ty:ty)? $body:block)*) => {
        $(
            $(#[$atts])*
            #[inline]
            #[cfg(feature = "tracing")]
            $vi fn $name$(<$generic>)?($($arg_name : $arg_ty),*) $(-> $ret_ty)? $body

            $(#[$atts])*
            #[doc = ""] // Empty doc line to start a new paragraph.
            #[doc = "Tracing is currently disabled so this function is a no-op. Enable `tracing` feature to enable tracing."]
            #[inline(always)]
            #[allow(unused_variables)]
            #[cfg(not(feature = "tracing"))]
            $vi fn $name$(<$generic>)?($($arg_name : $arg_ty),*) $(-> $ret_ty)? {
                // Do nothing
            }
        )*
    };
}

// set macro visibility to public in crate only
pub(crate) use tracepoint_fn;
