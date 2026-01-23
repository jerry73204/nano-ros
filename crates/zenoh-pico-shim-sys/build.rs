//! Build script for zenoh-pico-shim-sys
//!
//! This builds:
//! 1. zenoh-pico C library (via CMake for native, sources for embedded)
//! 2. The C shim layer (zenoh_shim.c)
//! 3. Platform-specific C code (system.c, network.c for smoltcp)
//! 4. Generates C header from Rust FFI declarations (cbindgen)

use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let target = env::var("TARGET").unwrap_or_default();

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
        // No backend selected - just build zenoh-pico and generate bindings
        // This allows building for testing header generation
        println!("cargo:warning=No platform backend selected. Building minimal configuration.");
    }

    if backend_count > 1 {
        panic!("Only one platform backend can be selected at a time (posix, zephyr, or smoltcp)");
    }

    // Paths
    let zenoh_pico_src = manifest_dir.join("zenoh-pico");
    let c_dir = manifest_dir.join("c");
    let include_dir = c_dir.join("include");

    // Check if zenoh-pico submodule is present
    if !zenoh_pico_src.join("include").exists() {
        panic!(
            "zenoh-pico submodule not found at {:?}. Run: git submodule update --init",
            zenoh_pico_src
        );
    }

    // Generate C header from Rust FFI declarations
    generate_header(&manifest_dir, &include_dir);

    // Build zenoh-pico
    let zenoh_pico_include = if !is_embedded_target(&target) {
        // For native builds, build via CMake
        build_zenoh_pico_native(&zenoh_pico_src, &out_dir)
    } else {
        // For embedded, just use headers (zenoh-pico is built separately)
        zenoh_pico_src.join("include")
    };

    // Build C shim
    if backend_count > 0 {
        build_c_shim(
            &c_dir,
            &include_dir,
            &zenoh_pico_include,
            use_posix,
            use_zephyr,
            use_smoltcp,
            &target,
        );
    }

    // Set cfg flags for Rust code
    if use_posix {
        println!("cargo:rustc-cfg=shim_backend=\"posix\"");
    } else if use_zephyr {
        println!("cargo:rustc-cfg=shim_backend=\"zephyr\"");
    } else if use_smoltcp {
        println!("cargo:rustc-cfg=shim_backend=\"smoltcp\"");
    }

    // Rerun triggers
    println!("cargo:rerun-if-changed=c/shim/zenoh_shim.c");
    println!("cargo:rerun-if-changed=c/platform_smoltcp/system.c");
    println!("cargo:rerun-if-changed=c/platform_smoltcp/network.c");
    println!("cargo:rerun-if-changed=c/platform_smoltcp/zenoh_smoltcp_platform.h");
    println!("cargo:rerun-if-changed=c/platform_smoltcp/zenoh_generic_config.h");
    println!("cargo:rerun-if-changed=c/platform_smoltcp/zenoh_generic_platform.h");
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=cbindgen.toml");
}

/// Check if we're building for an embedded target
fn is_embedded_target(target: &str) -> bool {
    target.contains("zephyr")
        || target.contains("none")
        || target.contains("thumbv")
        || target.contains("riscv")
}

/// Generate C header from Rust FFI declarations using cbindgen
fn generate_header(manifest_dir: &Path, include_dir: &Path) {
    // Create include directory if needed
    if !include_dir.exists() {
        std::fs::create_dir_all(include_dir).unwrap_or_else(|e| {
            println!("cargo:warning=Failed to create include directory: {e}");
        });
    }

    let output_file = include_dir.join("zenoh_shim.h");
    let config_file = manifest_dir.join("cbindgen.toml");

    // Load cbindgen config
    let config = cbindgen::Config::from_file(&config_file).unwrap_or_else(|e| {
        println!("cargo:warning=Failed to load cbindgen config: {e}");
        cbindgen::Config::default()
    });

    // Generate header
    let mut header = Vec::new();
    let result = cbindgen::Builder::new()
        .with_crate(manifest_dir)
        .with_config(config)
        .generate();

    match result {
        Ok(bindings) => {
            bindings.write(&mut header);
            let header_str = String::from_utf8_lossy(&header);

            // Post-process: remove lines starting with "extern " and collapse blank lines
            let processed = post_process_header(&header_str);

            std::fs::write(&output_file, processed).unwrap_or_else(|e| {
                println!("cargo:warning=Failed to write header: {e}");
            });
        }
        Err(e) => {
            println!("cargo:warning=cbindgen failed: {e}");
        }
    }
}

/// Post-process the generated header to remove duplicate declarations
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

/// Build zenoh-pico via CMake for native targets
fn build_zenoh_pico_native(zenoh_pico_src: &Path, out_dir: &Path) -> PathBuf {
    let zenoh_pico_build = out_dir.join("zenoh-pico-build");

    // Copy source to build directory to avoid modifying source tree
    copy_source_tree(zenoh_pico_src, &zenoh_pico_build);

    // Generate version header
    generate_version_header(&zenoh_pico_build);

    // Build via CMake
    let dst = cmake::Config::new(&zenoh_pico_build)
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BUILD_EXAMPLES", "OFF")
        .define("BUILD_TESTING", "OFF")
        .define("BUILD_TOOLS", "OFF")
        .define("ZENOH_DEBUG", "0")
        .define("Z_FEATURE_LOCAL_SUBSCRIBER", "1")
        .build();

    // Link the static library
    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=zenohpico");

    // Link system libraries
    let target = env::var("TARGET").unwrap_or_default();
    if target.contains("linux") || target.contains("darwin") || target.contains("macos") {
        println!("cargo:rustc-link-lib=pthread");
    } else if target.contains("windows") {
        println!("cargo:rustc-link-lib=ws2_32");
    }

    zenoh_pico_build.join("include")
}

/// Copy source tree to build directory
fn copy_source_tree(src: &Path, dst: &Path) {
    if dst.exists() {
        // Check if we need to recopy
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
        let _ = std::fs::remove_dir_all(dst);
    }

    let status = Command::new("cp")
        .args(["-r", src.to_str().unwrap(), dst.to_str().unwrap()])
        .status()
        .expect("Failed to copy zenoh-pico source");

    if !status.success() {
        panic!("Failed to copy zenoh-pico source to build directory");
    }
}

/// Generate zenoh-pico.h version header
fn generate_version_header(build_dir: &Path) {
    let include_dir = build_dir.join("include");
    std::fs::create_dir_all(&include_dir).unwrap();

    let version_header = include_dir.join("zenoh-pico.h");
    let version_file = build_dir.join("version.txt");

    let version = std::fs::read_to_string(&version_file)
        .unwrap_or_else(|_| "0.0.0".to_string())
        .trim()
        .to_string();

    let parts: Vec<&str> = version.split('.').collect();
    let major = parts.first().unwrap_or(&"0");
    let minor = parts.get(1).unwrap_or(&"0");
    let patch = parts.get(2).unwrap_or(&"0");

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

/// Build the C shim library
fn build_c_shim(
    c_dir: &Path,
    include_dir: &Path,
    zenoh_pico_include: &Path,
    use_posix: bool,
    use_zephyr: bool,
    use_smoltcp: bool,
    target: &str,
) {
    let mut build = cc::Build::new();

    // Include paths
    build.include(include_dir);
    build.include(zenoh_pico_include);

    // Core shim source
    build.file(c_dir.join("shim/zenoh_shim.c"));

    // Platform-specific configuration
    if use_posix {
        #[cfg(target_os = "linux")]
        build.define("ZENOH_LINUX", None);
        #[cfg(target_os = "macos")]
        build.define("ZENOH_MACOS", None);
    } else if use_zephyr {
        build.define("__ZEPHYR__", None);
        build.define("ZENOH_ZEPHYR", None);
    } else if use_smoltcp {
        let platform_dir = c_dir.join("platform_smoltcp");

        // Add platform sources
        build.file(platform_dir.join("system.c"));
        build.file(platform_dir.join("network.c"));

        // Include platform headers
        build.include(&platform_dir);

        // Platform defines
        build.define("ZENOH_SHIM_SMOLTCP", None);
        build.define("ZENOH_GENERIC", None);
        build.define("Z_FEATURE_MULTI_THREAD", "0");
        build.define("Z_FEATURE_LINK_TCP", "1");
        build.define("Z_FEATURE_LINK_UDP_MULTICAST", "0");
        build.define("Z_FEATURE_LINK_UDP_UNICAST", "0");
        build.define("Z_FEATURE_SCOUTING_UDP", "0");
        build.define("Z_FEATURE_LINK_SERIAL", "0");

        // ARM cross-compilation flags
        if target.contains("thumbv7em") {
            build
                .flag("-mcpu=cortex-m4")
                .flag("-mthumb")
                .flag("-mfpu=fpv4-sp-d16")
                .flag("-mfloat-abi=hard");
        }
    }

    build.opt_level(2);
    build.compile("zenoh_shim");
}
