use std::fs::File;
use std::io::{self, BufReader, Read, Write};
use std::os::unix::process::CommandExt;
use std::path::PathBuf;
use std::process::Command;

use libc;

use core::Tree;
use errors::{ResultExt, ScadDotsError};
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
    PrintMedium,
    PrintHigh,
    UpdatePreview,
}

#[derive(Debug, Clone, Copy)]
enum GoodOrBad {
    Good,
    Bad,
}

////////////////////////////////////////////////////////////////////////////////

pub fn preview_model(tree: &Tree) -> Result<(), ScadDotsError> {
    let scad = render_model(tree, RenderQuality::Low)?;
    let path = save_temp_file("preview", "", &scad)?;
    view_in_openscad(&[path])
}

#[track_caller]
pub fn check_model<F>(name: &str, action: Action, f: F)
where
    F: Fn() -> Result<Tree, ScadDotsError>,
{
    if let Err(e) = test_helper(name, action, &f) {
        println!("error: {}", e);
        panic!("returned error")
    }
}

// TODO let lib user control paths, somehow
#[track_caller]
fn test_helper<F>(
    name: &str,
    action: Action,
    model_creator: F,
) -> Result<(), ScadDotsError>
where
    F: Fn() -> Result<Tree, ScadDotsError>,
{
    let tree =
        model_creator().context("failed to construct test case's model")?;
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
            return Err(ScadDotsError::TestView);
        }
        Action::UpdatePreview => {
            let actual = render_model(&tree, RenderQuality::Low)?;
            save_temp_file("actual", name, &actual)?;
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
            return Err(ScadDotsError::TestCreate);
        }
        Action::Test => {
            let actual = render_model(&tree, RenderQuality::Low)?;
            let expected = load_model(name)
                .context("failed to load the expected model")?;
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

fn view_in_openscad(paths: &[String]) -> Result<(), ScadDotsError> {
    if cfg!(target_os = "linux") {
        unsafe {
            Command::new("openscad")
                .args(paths)
                .pre_exec(change_process_group)
                .spawn()
                .context("failed to run openscad viewer")?;
        }
    } else {
        Command::new("openscad")
            .args(paths)
            .spawn()
            .context("failed to run openscad viewer")?;
    };
    Ok(())
}

fn load_model(name: &str) -> Result<String, ScadDotsError> {
    let file = File::open(name_to_path(name, GoodOrBad::Good))
        .context("failed to open openscad file")?;
    let mut buf = BufReader::new(file);
    let mut s = String::new();
    buf.read_to_string(&mut s)
        .context("failed to read openscad file")?;
    Ok(s)
}

fn render_model(
    tree: &Tree,
    render_options: RenderQuality,
) -> Result<String, ScadDotsError> {
    to_code(tree, render_options)
}

fn save_file(path: &str, data: &str) -> Result<(), ScadDotsError> {
    println!("Writing to: {}", path);

    // Ensure parent directory exists
    let path_buf: PathBuf = path.into();
    let prefix = path_buf.parent().unwrap();
    std::fs::create_dir_all(prefix).unwrap();

    let mut f = File::create(path_buf)?;
    f.write_all(data.as_bytes())?;
    Ok(())
}

fn save_temp_file(
    id: &str,
    test_name: &str,
    code: &str,
) -> Result<String, ScadDotsError> {
    let path = format!("tests/tmp/{}_{}.scad", id, test_name);
    save_file(&path, code).context("failed to save temporary .scad file")?;
    Ok(path)
}

fn save_incorrect(name: &str, code: &str) -> Result<(), ScadDotsError> {
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
    fn to_string(self) -> String {
        match self {
            GoodOrBad::Good => "good".into(),
            GoodOrBad::Bad => "bad".into(),
        }
    }
}
