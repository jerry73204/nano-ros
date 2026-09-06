> **phase-321 W1.d — this directory is a SUBMODULE HOST, not a crate.**
> The `xrce-sys` crate (701 LoC + a 307-line build.rs) had zero dependents and
> was `--exclude`d from every workspace build; it was deleted. What remains is
> `micro-xrce-dds-client/` and `micro-cdr/`, the two vendored submodules that
> `nros-rmw-xrce-cffi/build.rs` compiles from (see `nros-sdk-index.toml`). Do
> not re-add a crate here.

> **phase-420 W9 step 4 — who compiles these two trees, and how the other lane
> gets them.** Since 2026-09-05 there is exactly ONE compile of
> `micro-xrce-dds-client/` and `micro-cdr/` in the tree, and it is
> `nros-rmw-xrce-cffi/build.rs` (cc-rs, six target families) — because that is
> the lane that ships: `cmake/NanoRosRmwDispatch.cmake` maps `xrce` to that
> crate for every image, and `zephyr/cmake/nros_rmw_xrce.cmake` is a no-op.
>
> `nros-rmw-xrce/CMakeLists.txt` used to compile the same rows a second time
> with its own flags, for its own two CTest binaries and nothing else — so that
> harness validated objects no image contained. It now LINKS the archive the
> build script produces. The two are joined by a pointer file the build script
> writes into its `OUT_DIR`, `nros-xrce-vendor-build.txt`, naming the archive
> and the generated-header directory; `just check rmw-xrce` reads `OUT_DIR` out
> of `cargo build --message-format=json` and hands it to cmake as
> `-DNROS_XRCE_CFFI_OUT_DIR`. Neither consumer restates a cc-rs fact, and
> `just check xrce-one-vendored-compile` is what keeps it that way.
>
> None of this is discoverable from either directory alone, which is why it is
> written here: the three shared statements live one level up
> (`../xrce-sources.txt` = which files, `../xrce-config.txt` = with what values,
> and the `NROS-XRCE-COMPILED-TREES` block in each lane = who compiles them).

# xrce-sys

FFI bindings + bundled C source for [Micro-XRCE-DDS-Client](https://github.com/eProsima/Micro-XRCE-DDS-Client) and Micro-CDR. Used by `nros-rmw-xrce`.

## License

Licensed under either of [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) or [MIT](https://opensource.org/licenses/MIT) at your option (unless the crate header says otherwise — `nros`, `nros-c`, `nros-cpp`, `nros-sizes-build`, `zpico-alloc` are Apache-2.0 only).

Part of the [nano-ros](https://github.com/NEWSLabNTU/nano-ros) project.
