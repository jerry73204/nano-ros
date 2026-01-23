use std::env;
use std::path::PathBuf;

fn main() {
    // Get paths
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    // Generate C header from Rust FFI declarations using cbindgen
    generate_header(&manifest_dir);
    let zenoh_pico_sys_path = manifest_dir.parent().unwrap().join("zenoh-pico-sys");

    // Check which platform backend to use
    let use_posix = env::var("CARGO_FEATURE_POSIX").is_ok();
    let use_zephyr = env::var("CARGO_FEATURE_ZEPHYR").is_ok();
    let use_smoltcp = env::var("CARGO_FEATURE_SMOLTCP").is_ok();

    // Count enabled backends
    let backend_count = [use_posix, use_zephyr, use_smoltcp]
        .iter()
        .filter(|&&b| b)
        .count();

    if backend_count == 0 {
        // No backend selected - just build the Rust wrapper without C code
        // This allows using zenoh-pico-shim as a pure Rust crate for FFI declarations
        println!("cargo:warning=No platform backend selected. Building without C shim.");
        return;
    }

    if backend_count > 1 {
        panic!("Only one platform backend can be selected at a time (posix, zephyr, or smoltcp)");
    }

    // Build the C shim
    let mut build = cc::Build::new();

    // Add our include paths
    build.include("include");

    // Find zenoh-pico headers from zenoh-pico-sys
    // The zenoh-pico source is a submodule in zenoh-pico-sys
    let zenoh_pico_include = zenoh_pico_sys_path.join("zenoh-pico").join("include");
    if zenoh_pico_include.exists() {
        build.include(&zenoh_pico_include);
    } else {
        println!(
            "cargo:warning=zenoh-pico include not found at {:?}",
            zenoh_pico_include
        );
    }

    // Add core shim source
    build.file("shim/zenoh_shim.c");

    // Add platform-specific backend and defines
    if use_posix {
        println!("cargo:rustc-cfg=shim_backend=\"posix\"");
        build.file("shim/backend_posix.c");

        // zenoh-pico platform defines for POSIX/Linux
        // Only set the platform macro - feature macros come from zenoh-pico's config.h
        #[cfg(target_os = "linux")]
        {
            build.define("ZENOH_LINUX", None);
        }
        #[cfg(target_os = "macos")]
        {
            build.define("ZENOH_MACOS", None);
        }
    } else if use_zephyr {
        println!("cargo:rustc-cfg=shim_backend=\"zephyr\"");
        build.file("shim/backend_zephyr.c");
        // Zephyr platform macros - must be defined before including zenoh-pico headers
        build.define("__ZEPHYR__", None);
        build.define("ZENOH_ZEPHYR", None);
    } else if use_smoltcp {
        println!("cargo:rustc-cfg=shim_backend=\"smoltcp\"");
        build.file("shim/backend_smoltcp.c");
        // Our custom define for the smoltcp backend
        build.define("ZENOH_SHIM_SMOLTCP", None);
        // Note: For actual smoltcp builds, we'd need to define platform macros
        // and potentially provide a custom platform implementation.
        // This is handled in Phase 8.4.
    }

    // Set optimization level
    build.opt_level(2);

    // Compile
    build.compile("zenoh_shim");

    // Note: We don't need to link zenohpico here because zenoh-pico-sys
    // handles that. The dependency on zenoh-pico-sys (via Cargo.toml features)
    // ensures the library is linked.

    // Re-run if C sources change
    println!("cargo:rerun-if-changed=include/zenoh_shim_platform.h");
    println!("cargo:rerun-if-changed=shim/zenoh_shim.c");
    println!("cargo:rerun-if-changed=shim/backend_posix.c");
    println!("cargo:rerun-if-changed=shim/backend_zephyr.c");
    println!("cargo:rerun-if-changed=shim/backend_smoltcp.c");
}

/// Generate the C header file from Rust FFI declarations using cbindgen.
///
/// cbindgen generates declarations from both:
/// 1. The extern "C" block in lib.rs (undocumented, with `extern` keyword)
/// 2. The stub functions in ffi.rs (documented, without `extern` keyword)
///
/// We post-process to keep only the documented declarations.
fn generate_header(manifest_dir: &PathBuf) {
    let crate_dir = manifest_dir;
    let include_dir = manifest_dir.join("include");
    let output_file = include_dir.join("zenoh_shim.h");
    let config_file = manifest_dir.join("cbindgen.toml");

    // Create include directory if it doesn't exist
    if !include_dir.exists() {
        std::fs::create_dir_all(&include_dir).unwrap_or_else(|e| {
            println!("cargo:warning=Failed to create include directory: {e}");
        });
    }

    // Re-run if FFI sources change
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=cbindgen.toml");

    // Load cbindgen config
    let config = cbindgen::Config::from_file(&config_file).unwrap_or_else(|e| {
        println!("cargo:warning=Failed to load cbindgen config: {e}");
        cbindgen::Config::default()
    });

    // Generate header to a string first
    let mut header = Vec::new();
    let result = cbindgen::Builder::new()
        .with_crate(crate_dir)
        .with_config(config)
        .generate();

    match result {
        Ok(bindings) => {
            bindings.write(&mut header);
            let header_str = String::from_utf8_lossy(&header);

            // Post-process: remove lines starting with "extern " (duplicate declarations)
            // and collapse multiple blank lines
            let processed = post_process_header(&header_str);

            // Write the processed header
            std::fs::write(&output_file, processed).unwrap_or_else(|e| {
                println!("cargo:warning=Failed to write header: {e}");
            });
        }
        Err(e) => {
            println!("cargo:warning=cbindgen failed: {e}");
        }
    }
}

/// Post-process the generated header to remove duplicate declarations.
fn post_process_header(header: &str) -> String {
    let mut result = String::new();
    let mut prev_blank = false;

    for line in header.lines() {
        // Skip lines starting with "extern " (duplicate undocumented declarations)
        if line.starts_with("extern ") {
            continue;
        }

        // Collapse multiple blank lines into one
        let is_blank = line.trim().is_empty();
        if is_blank && prev_blank {
            continue;
        }
        prev_blank = is_blank;

        result.push_str(line);
        result.push('\n');
    }

    result
}
