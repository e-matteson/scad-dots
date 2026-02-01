use std::error::Error;
use std::fmt;
use std::io;

#[derive(Debug)]
pub enum ScadDotsError {
    Rotation,
    Chain,
    Snake,
    Midpoint,
    Dimension,
    Args,
    TestView,
    TestCreate,
    Parse,
    /// For errors originating in some other crate that depends on this one
    /// (probably when using the scad-dots test harness).
    External(Box<dyn Error>),
    Ratio(f32),
    Io(io::Error),
    Context {
        message: String,
        cause: Box<ScadDotsError>,
    },
}

pub trait ResultExt<T> {
    /// Convert the error type to a ScadDotsError, and add the context message around it.
    fn context(self, message: &str) -> Result<T, ScadDotsError>;

    /// Like `context()` but take a closure containing a potentionally costly
    /// operation that will only be executed if there was an error.
    fn with_context<F>(self, message_creator: F) -> Result<T, ScadDotsError>
    where
        F: Fn() -> String;
}

impl<T, E> ResultExt<T> for Result<T, E>
where
    ScadDotsError: From<E>,
{
    fn context(self, message: &str) -> Result<T, ScadDotsError> {
        self.map_err(|err| ScadDotsError::from(err).context(message))
    }

    fn with_context<F>(self, message_creator: F) -> Result<T, ScadDotsError>
    where
        F: Fn() -> String,
    {
        self.context(&message_creator())
    }
}

impl ScadDotsError {
    /// Wrap the error with a message providing more context about what went wrong.
    pub fn context(self, message: &str) -> Self {
        ScadDotsError::Context {
            message: message.to_owned(),
            cause: Box::new(self),
        }
    }

    /// Like `context()` but take a closure containing a potentionally costly operation that will only be executed if there was an error.
    pub fn with_context<T>(self, message_creator: T) -> Self
    where
        T: Fn() -> String,
    {
        self.context(&message_creator())
    }
}

impl Error for ScadDotsError {
    fn cause(&self) -> Option<&dyn Error> {
        match self {
            ScadDotsError::Context { ref cause, .. } => Some(cause),
            ScadDotsError::Io(ref cause) => Some(cause),
            _ => None,
        }
    }
}

impl From<io::Error> for ScadDotsError {
    fn from(io_err: io::Error) -> ScadDotsError {
        ScadDotsError::Io(io_err)
    }
}

impl fmt::Display for ScadDotsError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            ScadDotsError::Ratio(x) => write!(f, "Invalid ratio: {}", x),
            ScadDotsError::Io(err) => write!(f, "Input/output error: {}", err),
            ScadDotsError::Rotation => write!(f, "Failed to compute rotation"),
            ScadDotsError::Chain => {
                write!(f, "Need at least 2 elements to chain")
            }
            ScadDotsError::Snake => write!(f, "Invalid snake axis order"),
            ScadDotsError::Midpoint => {
                write!(f, "A Midpoint can only be made from 2 Corners.")
            }
            ScadDotsError::Dimension => write!(f, "Invalid dimensions"),
            ScadDotsError::Args => write!(f, "Invalid argument(s)"),
            ScadDotsError::TestView => write!(
                f,
                "Viewed testcase instead of checking it, remember to change \
                 action back to Test."
            ),
            ScadDotsError::TestCreate => write!(
                f,
                "Created new expected testcase output instead of checking it, \
                 remember to change action back to Test."
            ),
            ScadDotsError::External(err) => {
                write!(f, "External error:\n{}", err)
            }
            ScadDotsError::Parse => write!(f, "Failed to parse openscad code."),
            ScadDotsError::Context { message, cause } => {
                write!(f, "{}\n  caused by: {}", message, cause)
            }
        }
    }
}

// "Hole fillet is too small, won't punch through wall"
// pub struct FilletError;

// pub fn panic_error(e: ScadDotsError) {
//     print_error(e);
//     panic!("returned error")
// }
