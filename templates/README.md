# nano-ros Embedded Templates

This directory contains Cargo.toml templates for common embedded configurations.

## Available Templates

| Template                        | Target  | Framework   | Description                            |
|---------------------------------|---------|-------------|----------------------------------------|
| `cargo-stm32f4-rs-rtic.toml`    | STM32F4 | RTIC 2.x    | Real-Time Interrupt-driven Concurrency |
| `cargo-stm32f4-rs-embassy.toml` | STM32F4 | Embassy     | Async embedded runtime                 |
| `cargo-stm32f4-rs-polling.toml` | STM32F4 | None        | Simple polling loop                    |
| `cargo-zephyr.toml`             | Zephyr  | Zephyr RTOS | For west-based projects                |

## Usage

1. Copy the appropriate template to your project as `Cargo.toml`
2. Update the `[package]` section with your project name
3. Adjust paths in `[dependencies]` for nano-ros crates
4. Copy the corresponding `.cargo/config.toml` from `examples/`

## Example

```bash
# Create new RTIC project
mkdir my-rtic-project && cd my-rtic-project
cp path/to/nano-ros/templates/cargo-stm32f4-rs-rtic.toml Cargo.toml
mkdir -p .cargo
cp path/to/nano-ros/examples/stm32f4-rs-rtic/.cargo/config.toml .cargo/
mkdir -p src
# Edit Cargo.toml and create your main.rs
```

## Feature Flags

All templates use these nano-ros features:

- `default-features = false` - Disables std/alloc
- `features = ["rtic"]` - Enables static buffer allocation
- `features = ["sync-critical-section"]` - Uses critical-section for sync

## Hardware Notes

### STM32F4

The templates target STM32F429ZI (NUCLEO-F429ZI board):
- ARM Cortex-M4F @ 168 MHz
- 2 MB Flash, 256 KB RAM + 64 KB CCM
- Ethernet support

For other STM32F4 variants, update:
- `stm32f4xx-hal` features (e.g., `stm32f401`)
- `embassy-stm32` features (e.g., `stm32f401cc`)
- `memory.x` linker script
- `.cargo/config.toml` chip name

### Memory Layout

Create `memory.x` for your target:
```
MEMORY {
  FLASH : ORIGIN = 0x08000000, LENGTH = 2M
  RAM   : ORIGIN = 0x20000000, LENGTH = 192K
}
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
```

### Build Script

Create `build.rs` to copy the linker script:
```rust
use std::{env, fs::File, io::Write, path::PathBuf};

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
```
