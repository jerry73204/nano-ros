//! Build script for qemu-rs-test
//!
//! This ensures the linker can find memory.x

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Get the output directory
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // Copy memory.x to the output directory
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    // Tell the linker where to find memory.x
    println!("cargo:rustc-link-search={}", out.display());

    // Rebuild if memory.x changes
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
