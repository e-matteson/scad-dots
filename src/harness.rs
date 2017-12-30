use failure::{Error, ResultExt};

use std::process::Command;
use std::fs::File;
use std::io::{BufReader, Read, Write};
use std::path::PathBuf;

use core::Tree;
use render::{to_code, RenderQuality};
use errors::{panic_error, TestError};


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
    if let Err(e) = test_helper(action, name, &f)
    // .context(format!("test '{}':", name))
    {
        // print_error_chain(e);
        // println!("ERROR: {}", e);
        // print_error(e);
        panic_error(e);

        // ::std::process::exit(1);
    }
}

fn test_helper<F>(action: Action, name: &str, f: F) -> Result<(), Error>
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

fn view_in_openscad(paths: &[String]) -> Result<(), Error> {
    let mut child = Command::new("openscad")
        .args(paths)
        .spawn()
        .context("failed to run openscad viewer")?;
    child.wait()
        .map(|_| ()) // Don't care about Ok value
        .context("failed to wait for openscad")?;
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
