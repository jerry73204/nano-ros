fn main() {
    // Build the cxx bridge
    cxx_build::bridge("src/lib.rs")
        .flag_if_supported("-std=c++17")
        .compile("nano_ros_cpp_bridge");

    // Re-run if source files change
    println!("cargo:rerun-if-changed=src/lib.rs");
}
