use failure::{Error, ResultExt};

use std::fs::File;
use std::io::{self, BufReader, Read, Write};
use std::os::unix::process::CommandExt;
use std::path::PathBuf;
use std::process::Command;

use libc;

use core::Tree;
use errors::{panic_error, TestError};
use render::{to_code, RenderQuality};

use parse::scad_relative_eq;

// static RENDER_OPTIONS: RenderQuality = RenderQuality::Test;
pub static MAX_RELATIVE: f32 = 0.00001;

/// What action to perform on this test case.
/// Normally, only `Test` will be used. Others are for temporary use.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub enum Action {
    Test,
    Create,
    ViewBoth,
    Preview,
    UpdatePreview,
    PrintMedium,
    PrintHigh,
}

#[derive(Debug, Clone, Copy)]
enum GoodOrBad {
    Good,
    Bad,
}

////////////////////////////////////////////////////////////////////////////////

pub fn preview_model(tree: &Tree) -> Result<(), Error> {
    let scad = render_model(tree, RenderQuality::Low)?;
    let path = save_temp_file("preview", "", &scad)?;
    view_in_openscad(&[path])
}

pub fn check_model<F>(name: &str, action: Action, f: F)
where
    F: Fn() -> Result<Tree, Error>,
{
    if let Err(e) = test_helper(name, action, &f) {
        panic_error(e);
    }
}

// TODO let lib user control paths, somehow
fn test_helper<F>(name: &str, action: Action, f: F) -> Result<(), Error>
where
    F: Fn() -> Result<Tree, Error>,
{
    let tree = f()?;
    match action {
        Action::PrintMedium => {
            let actual = render_model(&tree, RenderQuality::Medium)?;
            let path = save_temp_file("print-medium", name, &actual)?;
            view_in_openscad(&[path])?;
        }
        Action::PrintHigh => {
            let actual = render_model(&tree, RenderQuality::High)?;
            let path = save_temp_file("print-high", name, &actual)?;
            view_in_openscad(&[path])?;
        }
        Action::ViewBoth => {
            let actual = render_model(&tree, RenderQuality::Low)?;
            let mut paths = Vec::new();
            paths.push(save_temp_file("actual", name, &actual)?);
            if let Ok(expected) = load_model(name) {
                paths.push(save_temp_file("expected", name, &expected)?);
            }
            view_in_openscad(&paths)?;
            return Err(TestError::ViewBoth.into());
        }
        Action::Preview => {
            let actual = render_model(&tree, RenderQuality::Low)?;
            let path = save_temp_file("actual", name, &actual)?;
            view_in_openscad(&[path])?;
            // Don't check if there's a matching expected model
        }
        Action::UpdatePreview => {
            // Overwrite the Previewed file, but don't open a new instance of openscad.
            // If there's an old instance it can reload it instead.
            let actual = render_model(&tree, RenderQuality::Low)?;
            save_temp_file("actual", name, &actual)?;
        }
        Action::Create => {
            let actual = render_model(&tree, RenderQuality::Low)?;
            save_file(&name_to_path(name, GoodOrBad::Good), &actual)?;
            return Err(TestError::Create.into());
        }
        Action::Test => {
            let actual = render_model(&tree, RenderQuality::Low)?;
            let expected = load_model(name)?;
            if !scad_relative_eq(&actual, &expected, MAX_RELATIVE)? {
                save_incorrect(name, &actual)?;
                panic!("Models don't match")
            }
        }
    };
    Ok(())
}

/// This lets the child process (openscad) not get killed when the parent does.
fn change_process_group() -> Result<(), io::Error> {
    // First zero means affect current process, second zero means change pgid to own pid.
    if 0 == unsafe { libc::setpgid(0, 0) } {
        Ok(())
    } else {
        Err(io::Error::last_os_error())
    }
}

fn view_in_openscad(paths: &[String]) -> Result<(), Error> {
    //  TODO only do before_exec for linux
    // https://doc.rust-lang.org/reference/attributes.html#conditional-compilation
    Command::new("openscad")
        .args(paths)
        .before_exec(change_process_group)
        .spawn()
        .context("failed to run openscad viewer")?;
    Ok(())
}

fn load_model(name: &str) -> Result<String, Error> {
    let file = File::open(name_to_path(name, GoodOrBad::Good))?;
    let mut buf = BufReader::new(file);
    let mut s = String::new();
    buf.read_to_string(&mut s)?;
    Ok(s)
}

fn render_model(
    tree: &Tree,
    render_options: RenderQuality,
) -> Result<String, Error> {
    to_code(tree, render_options)
}

fn save_file(path: &str, data: &str) -> Result<(), Error> {
    println!("Writing to: {}", path);
    let mut f = File::create(path)?;
    f.write_all(data.as_bytes())?;
    Ok(())
}

fn save_temp_file(
    id: &str,
    test_name: &str,
    code: &str,
) -> Result<(String), Error> {
    let path = format!("tests/tmp/{}_{}.scad", id, test_name);
    save_file(&path, code).context("failed to save temporary .scad file")?;
    Ok(path)
}

fn save_incorrect(name: &str, code: &str) -> Result<(), Error> {
    let path = name_to_path(name, GoodOrBad::Bad);
    println!("Saving incorrect model as: '{}'", path);
    println!(
        "****************************************************************"
    );
    save_file(&path, code)
}

fn name_to_path(name: &str, status: GoodOrBad) -> String {
    let mut p = PathBuf::new();
    p.push("tests");
    p.push(format!("{}_models", &status.to_string()));
    p.push(format!("{}.scad", name));
    p.to_str().expect("failed to make path").to_owned()
}

impl GoodOrBad {
    fn to_string(&self) -> String {
        match *self {
            GoodOrBad::Good => "good".into(),
            GoodOrBad::Bad => "bad".into(),
        }
    }
}
