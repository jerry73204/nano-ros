# Mixed C / C++ / Rust Workspace

This workspace demonstrates mixed-language Node packages with a C++ native
Entry package.

```text
mixed/
├── .colcon_workspace
└── src/
    ├── c_talker_pkg/          # C Node pkg: publishes std_msgs/Int32 on /chatter
    ├── cpp_listener_pkg/      # C++ Node pkg: subscribes std_msgs/Int32 on /chatter
    ├── rust_heartbeat_pkg/    # Rust Node pkg: declares a timer callback
    ├── demo_bringup/          # Bringup pkg: package.xml + system.toml + launch/
    └── zephyr_entry/         # the ONE entry pkg — west needs a real app dir
```

From the repository root:

```bash
source ./activate.sh
cd examples/workspaces/mixed
nros setup native
nros build native
```

`nros build` walks the packages, resolves the image, checks the
toolchain, generates what the build system needs, and hands off to
cargo/cmake/west — so compiler errors are the compiler's, unchanged
(RFC-0065). `nros build` with no image lists what this workspace
declares. The old `nros sync` / `codegen-system` / `check` sequence
still works; it is just no longer something you have to type.
