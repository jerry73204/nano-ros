/*
 * nano-ros Zephyr Listener - C Stub
 *
 * This stub provides the Zephyr entry point and calls the Rust main function.
 * The actual application logic is in src/lib.rs (Rust).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

/* Rust entry point - defined in src/lib.rs */
extern void main(void);
