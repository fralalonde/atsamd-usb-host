#[macro_export]
macro_rules! logln_now {
    ($($arg:tt)*) => {
        unsafe {crate::logger::write_fmt_now(format_args!($($arg)*), true);}
    };
    (_) => {};
}

#[macro_export]
macro_rules! log_now {
    ($($arg:tt)*) => {
        unsafe {crate::logger::write_fmt_now(format_args!($($arg)*), false);}
    };
    (_) => {};
}
