/*
 * nano-ros Zephyr Talker - C Stub
 *
 * This stub provides the Zephyr entry point and calls the Rust main function.
 * The actual application logic is in src/lib.rs (Rust).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

/* Rust entry point - defined in src/lib.rs */
extern void main(void);

/*
 * Note: When using zephyr-lang-rust, this stub may not be needed
 * as the Rust code can define the entry point directly.
 * This is provided for compatibility with various build configurations.
 */
