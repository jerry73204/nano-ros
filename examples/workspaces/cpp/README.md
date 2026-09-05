# C++ Workspace

This workspace demonstrates C++ Node packages with a C++ native Entry package.

```text
cpp/
├── .colcon_workspace
└── src/
    ├── talker_pkg/      # Node pkg: publishes std_msgs/Int32 on /chatter
    ├── listener_pkg/    # Node pkg: subscribes std_msgs/Int32 on /chatter
    ├── demo_bringup/    # Bringup pkg: package.xml + system.toml + launch/
    └── native_entry/    # Entry pkg: native main()
```

From the repository root:

```bash
source ./activate.sh
cd examples/workspaces/cpp
nros setup native
nros build native
```

`nros build` walks the packages, resolves the image, checks the
toolchain, generates what the build system needs, and hands off to
cargo/cmake/west — so compiler errors are the compiler's, unchanged
(RFC-0065). `nros build` with no image lists what this workspace
declares. The old `nros sync` / `codegen-system` / `check` sequence
still works; it is just no longer something you have to type.
