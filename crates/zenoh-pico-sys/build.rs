//! Build script for zenoh-pico-sys
//!
//! This compiles zenoh-pico as a static library and generates Rust FFI bindings.
//!
//! The zenoh-pico library is built using CMake and linked statically.
//! No external library path configuration is required.

use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let zenoh_pico_src = manifest_dir.join("zenoh-pico");
    let include_dir = zenoh_pico_src.join("include");

    // Check if zenoh-pico submodule is present
    if !include_dir.exists() {
        panic!(
            "zenoh-pico submodule not found at {:?}. Run: git submodule update --init",
            zenoh_pico_src
        );
    }

    // Determine target platform
    let target = env::var("TARGET").unwrap_or_default();

    // For native builds, copy source to OUT_DIR and build there to avoid modifying source tree
    let (_build_dir, build_include_dir) = if !is_embedded_target(&target) {
        let zenoh_pico_build = out_dir.join("zenoh-pico-src");
        copy_source_tree(&zenoh_pico_src, &zenoh_pico_build);

        // Generate version header
        generate_version_header(&zenoh_pico_build);

        // Build
        build_zenoh_pico_native(&zenoh_pico_build);

        (zenoh_pico_build.clone(), zenoh_pico_build.join("include"))
    } else {
        // For embedded, just generate the header
        let generated_include = out_dir.join("include");
        std::fs::create_dir_all(&generated_include).unwrap();
        generate_version_header_to(&zenoh_pico_src, &generated_include);
        (zenoh_pico_src.clone(), include_dir.clone())
    };

    // Generate bindings
    generate_bindings(&manifest_dir, &include_dir, &build_include_dir, &target);

    // Rebuild triggers
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=zenoh-pico/include/zenoh-pico.h.in");
    println!("cargo:rerun-if-changed=zenoh-pico/version.txt");
    println!("cargo:rerun-if-changed=zenoh-pico/CMakeLists.txt");
}

/// Check if we're building for an embedded target
fn is_embedded_target(target: &str) -> bool {
    target.contains("zephyr")
        || target.contains("none")
        || target.contains("thumbv")
        || target.contains("riscv")
}

/// Copy the zenoh-pico source tree to the build directory
fn copy_source_tree(src: &Path, dst: &Path) {
    // Only copy if destination doesn't exist or is older
    if dst.exists() {
        // Check if we need to recopy by comparing a key file
        let src_cmake = src.join("CMakeLists.txt");
        let dst_cmake = dst.join("CMakeLists.txt");
        if dst_cmake.exists() {
            let src_meta = std::fs::metadata(&src_cmake).ok();
            let dst_meta = std::fs::metadata(&dst_cmake).ok();
            if let (Some(s), Some(d)) = (src_meta, dst_meta) {
                if let (Ok(st), Ok(dt)) = (s.modified(), d.modified()) {
                    if dt >= st {
                        return; // Already up to date
                    }
                }
            }
        }
        // Remove old copy
        let _ = std::fs::remove_dir_all(dst);
    }

    // Use cp -rL to follow symlinks (zenoh-pico may be a symlink)
    let status = Command::new("cp")
        .args(["-rL", src.to_str().unwrap(), dst.to_str().unwrap()])
        .status()
        .expect("Failed to copy zenoh-pico source");

    if !status.success() {
        panic!("Failed to copy zenoh-pico source to build directory");
    }
}

/// Generate zenoh-pico.h in the build directory
fn generate_version_header(build_dir: &Path) {
    let include_dir = build_dir.join("include");
    generate_version_header_to(build_dir, &include_dir);
}

/// Generate zenoh-pico.h to a specific directory
fn generate_version_header_to(zenoh_pico_dir: &Path, include_dir: &Path) {
    std::fs::create_dir_all(include_dir).unwrap();

    let version_header = include_dir.join("zenoh-pico.h");

    // Read version from version.txt
    let version_file = zenoh_pico_dir.join("version.txt");
    let version = std::fs::read_to_string(&version_file)
        .unwrap_or_else(|_| "0.0.0".to_string())
        .trim()
        .to_string();

    let parts: Vec<&str> = version.split('.').collect();
    let major = parts.first().unwrap_or(&"0");
    let minor = parts.get(1).unwrap_or(&"0");
    let patch = parts.get(2).unwrap_or(&"0");

    // Read template and substitute
    let template_path = include_dir.join("zenoh-pico.h.in");
    if template_path.exists() {
        let template = std::fs::read_to_string(&template_path).unwrap();
        let generated = template
            .replace("@ZENOH_PICO@", &version)
            .replace("@ZENOH_PICO_MAJOR@", major)
            .replace("@ZENOH_PICO_MINOR@", minor)
            .replace("@ZENOH_PICO_PATCH@", patch)
            .replace("@ZENOH_PICO_TWEAK@", "0");
        std::fs::write(&version_header, generated).unwrap();
    }
}

/// Build zenoh-pico as a static library using CMake
fn build_zenoh_pico_native(zenoh_pico_dir: &Path) {
    let dst = cmake::Config::new(zenoh_pico_dir)
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BUILD_EXAMPLES", "OFF")
        .define("BUILD_TESTING", "OFF")
        .define("BUILD_TOOLS", "OFF")
        .define("ZENOH_DEBUG", "0")
        // Enable local subscriber feature for same-session pub/sub
        .define("Z_FEATURE_LOCAL_SUBSCRIBER", "1")
        .build();

    // Link the static library
    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=zenohpico");

    // Link system libraries required by zenoh-pico
    let target = env::var("TARGET").unwrap_or_default();
    if target.contains("linux") || target.contains("darwin") || target.contains("macos") {
        println!("cargo:rustc-link-lib=pthread");
    } else if target.contains("windows") {
        println!("cargo:rustc-link-lib=ws2_32");
    }
}

/// Generate Rust FFI bindings using bindgen
fn generate_bindings(
    manifest_dir: &Path,
    _source_include_dir: &Path,
    build_include_dir: &Path,
    target: &str,
) {
    let platform_define = if target.contains("linux") {
        "-DZENOH_LINUX"
    } else if target.contains("darwin") || target.contains("macos") {
        "-DZENOH_MACOS"
    } else if target.contains("windows") {
        "-DZENOH_WINDOWS"
    } else if target.contains("zephyr") {
        "-DZENOH_ZEPHYR"
    } else {
        // Default to Linux for native builds
        "-DZENOH_LINUX"
    };

    let bindings = bindgen::Builder::default()
        .header(manifest_dir.join("wrapper.h").to_str().unwrap())
        .clang_arg(format!("-I{}", manifest_dir.display()))
        // Use build include dir (has generated zenoh-pico.h)
        .clang_arg(format!("-I{}", build_include_dir.display()))
        .clang_arg(platform_define)
        // no_std compatibility
        .use_core()
        .ctypes_prefix("cty")
        // Only generate bindings for zenoh-pico API
        .allowlist_function("z_.*")
        .allowlist_function("zp_.*")
        .allowlist_function("ze_.*")
        .allowlist_type("z_.*")
        .allowlist_type("zp_.*")
        .allowlist_type("ze_.*")
        .allowlist_var("Z_.*")
        // Block system types that cause issues
        .blocklist_type("max_align_t")
        // Generate layout tests only with std
        .layout_tests(cfg!(feature = "std"))
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
