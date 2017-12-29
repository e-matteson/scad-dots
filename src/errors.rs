use failure::Error;

#[derive(Debug, Fail)]
#[fail(display = "Failed to compute rotation")]
pub struct RotationError;

#[derive(Debug, Fail)]
#[fail(display = "Need at least 2 elements to chain")]
pub struct ChainError;

#[derive(Debug, Fail)]
#[fail(display = "Invalid snake axis order")]
pub struct SnakeError;

#[derive(Debug, Fail)]
#[fail(display = "Invalid ratio: {}", _0)]
pub struct RatioError(pub f32);

#[derive(Debug, Fail)]
#[fail(display = "A Midpoint can only be made from 2 Corners.")]
pub struct MidpointError;

#[derive(Debug, Fail)]
#[fail(display = "Invalid dimensions")]
pub struct DimensionError;

// #[derive(Debug, Fail)]
// #[fail(display = "Hole fillet is too small, won't punch through wall")]
// pub struct FilletError;

// #[derive(Debug, Fail)]
// #[fail(display = "Failed to parse openscad code")]
// pub struct ParseError;

#[derive(Debug, Fail)]
#[fail(display = "Invalid argument(s)")]
pub struct ArgError;

#[derive(Debug, Fail)]
pub enum TestError {
    #[fail(display = "Change action from View to Run.")] View,
    #[fail(display = "created new test case, change action from Create to \
                      Run.")]
    Create,
}

#[derive(Debug, Fail)]
#[fail(display = "Failed to parse openscad code")]
pub struct ParseError;

pub fn panic_error(e: Error) {
    print_error(e);
    panic!("returned error")
}

pub fn print_error(e: Error) {
    let mut causes = e.causes();
    if let Some(first) = causes.next() {
        println!("error: {}", first);
    }
    for cause in causes {
        println!("cause: {}", cause);
    }
}
