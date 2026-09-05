# Node packages

> **This page is part of the Multi-Node Projects group.**
> Previous: [Project layout](./workspace-from-app-node.md) —
> Next: [Bringup packages](./workspace-bringup.md)

A **Node pkg** is the unit of reusable behaviour in a multi-node workspace.
It is a Rust library — a `[lib]` crate — that implements one node and
registers it with `nros::node!(T)`.
The Entry pkg is what boots the binary; the Node pkg is what runs inside it.

---

## Prereqs

Pick one path from a fresh checkout — `just` is NOT a prereq.

**A. Front door** (bare machine OK — no Rust, no `just`):
```sh
./scripts/bootstrap.sh
```
Installs rustup if needed and builds the in-tree `nros` CLI from
source at `packages/cli/target/release/nros`, leaving it on PATH for
this shell (in a checkout, the tree's own build is the binary this tree
accepts — RFC-0090 / phase-431).

**B. Already have cargo** (equivalent — same build, same binary):
```sh
git submodule update --init packages/cli/third-party/play_launch
cargo build --release --manifest-path packages/cli/Cargo.toml --bin nros
export PATH="$PWD/packages/cli/target/release:$PATH"
```

Every subsequent shell sources the workspace env via one of:
```sh
direnv allow                  # if you use direnv
source ./activate.sh          # bash / zsh
source ./activate.fish        # fish
```

Then provision the native host:
```sh
nros setup native --rmw zenoh
```

---

## Scaffolding a Node pkg

Use `nros new` to create a skeleton:

```bash
nros new talker --platform native --lang rust
```

`nros new` creates a project skeleton.
For a workspace, move (or create) the result under `src/talker_pkg/` so the
workspace root's `Cargo.toml` can include it as a member:

```toml
# workspace Cargo.toml
[workspace]
resolver = "2"
members = ["src/talker_pkg", "src/listener_pkg", "src/native_entry"]
```

---

## Node pkg anatomy

A Node pkg has three files:

```
src/talker_pkg/
├── package.xml          # ROS 2 manifest — <exec_depend> per message package
├── Cargo.toml           # [lib] + [package.metadata.nros.node] metadata
└── src/lib.rs           # impl Node + ExecutableNode; ends with nros::node!(Talker);
```

No `fn main()` here — a Node pkg is a library linked into an Entry pkg.
The Entry pkg's macro-generated runtime owns `nros::init`, executor open,
RMW registration, and the spin/yield loop.

---

## `Cargo.toml` — the `[package.metadata.nros.node]` block

The metadata block is what the `nros` CLI reads to discover, name, and
wire this node into a topology.
From [`examples/workspaces/rust/src/talker_pkg/Cargo.toml`](../../../examples/workspaces/rust/src/talker_pkg/Cargo.toml):

```toml
[lib]
path = "src/lib.rs"

[package.metadata.nros.node]
class = "talker_pkg::Talker"
name = "talker"
default_namespace = "/"

[dependencies]
nros = { path = "../../../../../packages/api/nros", default-features = false,
         features = ["alloc", "rmw-cffi"] }
```

The three fields in `[package.metadata.nros.node]`:

| Field | Purpose |
|---|---|
| `class` | Fully-qualified Rust path to the type that `impl`s `Node + ExecutableNode` |
| `name` | Default ROS 2 node name (remappable at launch) |
| `default_namespace` | Default namespace (remappable at launch) |

A Node pkg names **no** platform and **no** RMW. `alloc` is the universal
baseline and `rmw-cffi` is the vtable seam; the concrete backend and the
platform both flow in from the board crate that the sibling Entry pkg depends
on, via cargo feature unification. That is what lets one `talker_pkg` deploy to
a Linux host and a Cortex-M board without a fork.

---

## `src/lib.rs` — the node implementation

A Node pkg implements two traits: `Node` (declarative registration) and
`ExecutableNode` (per-callback body), then calls `nros::node!` to export the
trampolines the Entry macro expects.

Here is the essential shape, drawn from
[`examples/workspaces/rust/src/talker_pkg/src/lib.rs`](../../../examples/workspaces/rust/src/talker_pkg/src/lib.rs)
(see that file for the full worked version):

```rust
use nros::{
    CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};

pub struct Talker;

impl Node for Talker {
    const NAME: &'static str = "talker";

    // What this class's per-instance registries hold, in order: publishers,
    // service servers, service clients, action clients, action servers.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("talker"))?;
        let chatter = node.create_publisher_for_topic::<MyMsg>("/chatter")?;
        let _timer =
            node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(1000))?;
        node.callback_for_name("on_tick")
            .publishes_entity(&chatter)?;
        Ok(())
    }
}

impl ExecutableNode for Talker {
    type State = i32;

    fn init() -> Self::State { 0 }

    // See the full example for the callback body.
}

nros::node!(Talker);   // <-- exports the trampolines; this is the last line
```

Key points:

- `Node::register` is **declarative** — it runs once at startup to declare
  publishers, subscriptions, timers, and callback edges. No message bytes
  flow here.
- `ExecutableNode::on_callback` is the **body** — called by the Entry pkg's
  executor each time a callback fires. `state` is your per-node mutable storage.
- `nros::node!(Talker)` **must be the last public API call** in the file.
  It generates the `extern "C"` trampolines the Entry macro imports.
- There is **no `fn main()`** in a Node pkg.
- `ENTITY_BOUNDS` is **static RAM you either pay or do not** (issue 0857).
  `nros::node!` emits a `static` sized from it, and a publisher slot carries a
  loan arena — so the default (`NROS_RUNTIME_MAX_CELL_ENTITIES`, 8 per kind)
  measured **50,824 bytes** of `.bss` for a one-service component against
  **568 bytes** for the same component declaring what it creates. Count the
  five kinds in your own `register()`; a timer or a subscription needs no slot
  here (they claim an executor callback slot instead). Declaring FEWER than
  `register()` creates is a registration error at boot, never a silent drop,
  so the number is safe to make exact.

---

## `package.xml` — the ROS 2 manifest

A Node pkg that uses generated message types lists them as `<exec_depend>`
entries. Minimal example:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>talker_pkg</name>
  <version>0.1.0</version>
  <description>Talker node</description>
  <maintainer email="dev@example.com">Developer</maintainer>
  <license>MIT OR Apache-2.0</license>

  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

If your node uses no external message packages, the `<depend>` line can be
omitted.

---

## Services & clients

### Server (responds to requests)

Declare the service in `Node::register` and handle requests in the node body.

- **Rust** — declare a service edge in `register`; requests dispatch into
  `ExecutableNode::on_callback` like any other callback (read the request and
  write the reply through the `CallbackCtx`).
- **C++/C** (the `configure(Node&)` component shape) — bind a **typed** member
  with `nros::bind_service`:

  ```cpp
  // Response on_request(const Request&) — no hand-rolled CDR.
  AddTwoInts::Response on_add(const AddTwoInts::Request& req) {
      AddTwoInts::Response r; r.sum = req.a + req.b; return r;
  }
  ::nros::Result configure(::nros::Node& node) {
      return ::nros::bind_service<AddTwoInts, MyServer, &MyServer::on_add>(
          node, "/add_two_ints", this);   // service-type name from AddTwoInts::TYPE_NAME
  }
  ```

  `bind_service` deserializes the request into the generated `Svc::Request`,
  calls your typed member, and serializes the returned `Svc::Response` — no
  manual byte offsets. (`bind_service_raw<&method>` is still available for the
  rare hand-rolled case.)

### Client (issues requests) — call from `tick`, not `on_callback`

A node **cannot** make a blocking service/action client call from
`on_callback`: that runs while the executor is mid-dispatch, so a blocking call
would deadlock the spin. Client calls go on the **separate per-spin hook**
`ExecutableNode::tick(&mut TickCtx)`, which the executor runs once per
`spin_once` *between* callback dispatch:

```rust
fn on_callback(state: &mut State, cb: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
    // ...decide a request is needed; just arm a flag / stash args:
    state.want_call = true;
}

fn tick(state: &mut State, ctx: &mut TickCtx<'_>) {
    if state.want_call {
        state.want_call = false;
        let reply = ctx.call_for_name("add_two_ints", &request);  // blocking, safe here
        // ...use reply
    }
}
```

The two-surface pattern (arm in `on_callback`, call in `tick`) is the supported
way to drive a client from a declarative node. See
`examples/workspaces/rust/.../service_client_pkg` for the reference.

---

## Building

From the workspace root, sync generated interfaces first, then let Cargo
compile the Node pkgs and Entry pkg:

```bash
# From examples/workspaces/rust/ (or your workspace root):
nros sync
nros codegen-system --bringup demo_bringup
cargo build -p native_entry
```

No per-Node-pkg invocation is needed — the workspace resolver handles
dependency ordering.

To cross-compile for an embedded target, pass `--target` and ensure
`.cargo/config.toml` in the workspace root sets the right linker and target:

```bash
cargo build --target thumbv7em-none-eabihf
```

---

## Next steps

- **[Bringup packages](./workspace-bringup.md)** — wire the Node
  pkgs together into a topology.
- **[Entry packages](./workspace-entry-pkg.md)** — build the
  binary that boots the topology on real hardware or a host process.
- **[Role reference](../user-guide/component-and-entry-pkg.md)** —
  the full reference for all three roles, metadata fields, and the
  `nros::main!()` four forms.
