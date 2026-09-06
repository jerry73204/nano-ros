# nano-ros CI base image (Phase 196.9).
#
# The stable, heavy provisioning every host/per-platform CI lane re-installs each
# run — ROS 2 Humble (base image, for the interface codegen's AMENT lookup) + the
# host build tools (cmake/ninja/dtc/gperf, uv, just, rustup + cross targets) +
# the arm cross-compiler. NOT baked: the `nros` CLI (released binary, version-
# pinned, fetched per-run via install.sh) or any board SDK/source (provisioned
# per-run by `nros setup`, the way a user does).
#
# Sibling of the zephyr CI image, which carries the ~1 GB Zephyr SDK layer. NOTE
# they are SIBLINGS, not parent/child: `ci/docker/zephyr-ros/Dockerfile` is
# `FROM ros:humble-ros-base` itself and inherits NOTHING from this file. This
# header used to claim it was `FROM` this one, which is how a fix landing here
# could look like it covered both (issue 0866). A change to the shared base
# environment has to be made in BOTH. `platform-ci.yml` + `host-unit-
# tests` run their cells in this image — no per-cell `setup-ros` / apt / tool
# installs, and the AMENT/ROS gap that fails `nros generate-rust` is closed.
#
# Build/publish via the `ci-base` job in .github/workflows/images.yml →
#   ghcr.io/newslabntu/nano-ros-ci:humble  (tag = ROS distro; bump = new tag)
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ---- ROS 2 environment, baked (issue 0866) --------------------------------
#
# The image HAS ROS; what it lacked was ROS's ENVIRONMENT. `ros:*` images ship
# `/ros_entrypoint.sh`, which sources `setup.bash` — but GitHub Actions runs a
# job's steps directly and does NOT use the image's ENTRYPOINT, so every `just`
# invocation in these lanes ran with `AMENT_PREFIX_PATH` unset and ROS's
# site-packages off `PYTHONPATH`.
#
# `ROS_DISTRO=humble` IS set by the base image, so "is ROS here?" probes that
# read only that variable answered YES while nothing else was wired — which is
# how this stayed invisible.
#
# What it broke: `scripts/cyclonedds/msg_to_cyclone_idl.py` resolves ROS's stock
# `rosidl_adapter`, and requires not just the directory but that a subprocess
# can `import rosidl_adapter`. Measured in this exact image:
#
#     $ docker run --entrypoint bash ros:humble-ros-base \
#         -c 'python3 -c "import rosidl_adapter"'
#     ModuleNotFoundError: No module named 'rosidl_adapter'
#
# which is verbatim the failure that script's own comment predicts for "a host
# with ROS installed but not sourced through to the nested cmake/ninja
# invocation". Every cold cyclonedds build in CI died on it
# (`check-rmw-cyclonedds` → `AddTwoInts.idl` Error 1).
#
# Baked as ENV rather than sourced per-lane so it reaches EVERY process without
# each workflow, recipe and nested build remembering to source it — the
# propagation this exact bug is about. Values captured from
# `. /opt/ros/humble/setup.bash` in this image, not hand-written:
#
#     docker run --entrypoint bash ros:humble-ros-base -c \
#       'env|sort >/tmp/a; . /opt/ros/humble/setup.bash; env|sort >/tmp/b; diff /tmp/a /tmp/b'
#
# `LD_LIBRARY_PATH` names the arch triplet because setup.bash does; these lanes
# run on x86_64 runners. `PATH` is extended here and the Rust block below
# prepends to it, so cargo still wins.
#
# CAVEAT: this is a SNAPSHOT of what sourcing produces for THIS package set. It
# holds for anything installed into the single `/opt/ros/humble` prefix (the apt
# packages below are), but a ROS package that installs to a DIFFERENT prefix, or
# one whose own setup extends these variables further, would need this block
# updated — `ros-humble-rmw-zenoh-cpp` adding
# `<prefix>/opt/zenoh_cpp_vendor/lib` to `LD_LIBRARY_PATH` is the case issue
# 0774 is about. Re-run the capture command above after changing the ROS package
# set rather than assuming.
ENV AMENT_PREFIX_PATH=/opt/ros/humble \
    LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib \
    PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages \
    ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3 \
    ROS_LOCALHOST_ONLY=0
ENV PATH=/opt/ros/humble/bin:$PATH

# Host build deps shared across platforms + the arm cross-compiler (the setup-
# time zenoh-pico build + freertos/nuttx/threadx/qemu cross builds need it on
# PATH before `nros setup` provisions the index toolchain). curl/zstd/git for the
# per-run install.sh + nros source provisioning.
RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake ninja-build device-tree-compiler gperf \
        git curl ca-certificates zstd xz-utils file wget \
        python3 python3-pip python3-venv python3-tomli build-essential \
        lld \
        gcc-arm-none-eabi gcc-riscv64-unknown-elf picolibc-riscv64-unknown-elf \
        libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
        `# ^ the ARM cross compiler's LIBC and libstdc++. Debian ships these` \
        `# separately from gcc-arm-none-eabi, and RISC-V got` \
        `# picolibc-riscv64-unknown-elf while ARM got nothing — so the ARM` \
        `# toolchain could compile only freestanding code. Two gates paid for` \
        `# it: check-sched-dim-arms(freertos) died on "string.h: No such file` \
        `# or directory" and FAILED the merge group, while` \
        `# cross_libc_precedence_gate SKIPPED with "no usable libstdc++" —` \
        `# one absent package, one hard red and one silent coverage hole.` \
        kconfig-frontends socat parallel unzip flex bison \
        libslirp0 libpixman-1-0 \
        libmbedtls-dev \
        clang \
        `# ^ NOT clang-format (that is pinned separately by just setup-clang-` \
        `# format). api-parity extracts OUR C and C++ surface by running` \
        `# clang -Xclang -ast-dump=json over the public headers, so without` \
        `# the compiler that gate cannot run in this container at all --` \
        `# which is why check-api-parity is today the only ci-gate step run` \
        `# by NO workflow, on any event. That is how phase-421's two` \
        `# serialization_format accessors landed unledgered.` \
        ros-humble-example-interfaces \
    && rm -rf /var/lib/apt/lists/*

# uv (Python 3.12 venvs) + just (recipe runner) via their canonical installers.
# PINNED, not latest. Both were `curl | sh` fetching whatever was current at
# image-build time, so two rebuilds of the "same" image could ship different
# build tooling with no signal — the floating half of the drift axis in
# `docs/development/ci-image-provisioning.md`. The versions here are the ones
# running on the reference dev host, so CI and a developer's tree agree.
#
# Bump deliberately, and bump the image tag with it (see `images.yml`): the tag
# encodes what is baked precisely so a bump cannot be silent.
ARG UV_VERSION=0.12.5
ARG JUST_VERSION=1.57.0
RUN curl -LsSf "https://astral.sh/uv/${UV_VERSION}/install.sh" \
        | env UV_INSTALL_DIR=/usr/local/bin sh \
    && uv --version | grep -q "$UV_VERSION"
RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh \
        | bash -s -- --tag "$JUST_VERSION" --to /usr/local/bin \
    && just --version | grep -q "$JUST_VERSION"

# Rust toolchain + the cross targets the platforms build for. Pinned to a fixed
# root on PATH for all users (so in-container jobs need no per-cell rustup).
ENV RUSTUP_HOME=/usr/local/rustup CARGO_HOME=/usr/local/cargo
ENV PATH=/usr/local/cargo/bin:$PATH
# Stable toolchain baked WITH the exact components + targets the workspace
# `rust-toolchain.toml` declares (rustfmt/clippy/rust-src/llvm-tools;
# thumbv7em-none-eabihf/armv7r-none-eabihf/aarch64-unknown-none) PLUS the other
# cross targets the platforms build for. rustup auto-installs any declared
# component/target on first use; under `parallel` (build-fixtures) that races on
# /usr/local/rustup/downloads (`could not rename '...partial'` /
# `rustfmt-preview ... conflict bin/cargo-fmt`). Baking the full set = no runtime
# install. Keep in lockstep with rust-toolchain.toml.
RUN curl -fsSL https://sh.rustup.rs | sh -s -- -y --profile minimal --no-modify-path \
        -c rustfmt,clippy,rust-src,llvm-tools \
    && rustup target add \
        x86_64-unknown-none armv7a-none-eabi armv7r-none-eabihf aarch64-unknown-none \
        thumbv7m-none-eabi thumbv7em-none-eabi thumbv7em-none-eabihf \
        riscv32imc-unknown-none-elf riscv64imac-unknown-none-elf riscv64gc-unknown-none-elf \
    && cargo install cargo-nextest --locked

# ...and then every target the tree DECLARES, read from the SSoT rather than
# hand-listed. Issue 0833 split this list into `config/rust-targets.txt`
# because two hand-written copies had drifted; this image was a THIRD copy and
# had drifted the same way -- it never gained `armv8r-none-eabihf`, so
# `check-tier-preconditions` failed on every container job that ran a tier.
#
# Additive on purpose: the list above carries targets the SSoT does not
# declare, and dropping them here would be a silent capability loss. The rows
# below can only add. `scripts/lib/rust-targets.sh` is the one parser (it
# locates the list relative to itself, hence the mirrored paths).
COPY scripts/lib/rust-targets.sh /opt/nros-ssot/scripts/lib/rust-targets.sh
COPY config/rust-targets.txt     /opt/nros-ssot/config/rust-targets.txt
RUN . /opt/nros-ssot/scripts/lib/rust-targets.sh \
    && nros_rust_targets rustup | xargs -r rustup target add

# sccache — the compiler cache the justfile ALREADY wires: `RUSTC_WRAPPER` is
# `command -v sccache || true`, so every `just` recipe picks it up the moment
# the binary is on PATH, and no workflow needs to opt in.
#
# BAKED, not installed per run: `nros setup --tool sccache` builds it from
# source (vendored-openssl), which is minutes on a cold runner and would cost
# more than it saves. The prebuilt release tarball is the same 0.17.0 the
# `[tool.sccache]` index row pins — keep the two in lockstep.
#
# issue 0874 — 0.8.2 (2024-09-28) predates GitHub's Actions Cache v2, which
# replaced the v1 service on 2025-02-01. Its GHA backend asks for the retired
# `ACTIONS_CACHE_URL` and fails permanently; because sccache is the
# `RUSTC_WRAPPER`, that failed every `rustc` rather than merely losing a cache,
# and took down `main` plus every open PR. ghac v2 landed in sccache 0.10.0
# ("Bump opendal to 0.52 to support ghac v2") and moved to v4 in 0.11.0, so the
# floor is 0.10.0 and this takes the current release.
#
# Why sccache rather than caching `target/`: it is CONTENT-hashed, so a restored
# cache cannot make a stale artifact read as fresh. Restoring a `target/` dir
# into a fresh checkout gives artifacts mtimes newer than their sources, which
# is exactly the museum-binary class CLAUDE.md's fixture-mtime treadmill
# describes. Content addressing sidesteps it entirely.
ARG SCCACHE_VERSION=0.17.0
RUN set -eux; \
    arch="$(dpkg --print-architecture)"; \
    case "$arch" in \
        amd64) target="x86_64-unknown-linux-musl" ;; \
        arm64) target="aarch64-unknown-linux-musl" ;; \
        *) echo "unsupported arch: $arch" >&2; exit 1 ;; \
    esac; \
    url="https://github.com/mozilla/sccache/releases/download/v${SCCACHE_VERSION}/sccache-v${SCCACHE_VERSION}-${target}.tar.gz"; \
    curl -fsSL "$url" -o /tmp/sccache.tar.gz; \
    tar -xzf /tmp/sccache.tar.gz -C /tmp; \
    install -m0755 "/tmp/sccache-v${SCCACHE_VERSION}-${target}/sccache" /usr/local/bin/sccache; \
    rm -rf /tmp/sccache.tar.gz "/tmp/sccache-v${SCCACHE_VERSION}-${target}"; \
    sccache --version

# Pinned nightly (tools/rust-toolchain.toml `nightly-2026-04-11`) baked WITH its
# declared components (rustfmt/miri/rust-src/llvm-tools) + the cross targets. The
# fixture builds need it (esp32 `-Z build-std` for riscv32imc, `-Z emit-stack-
# sizes`, nightly-only target adds). Same anti-race rationale as stable. Bump in
# lockstep with the pin.
RUN rustup toolchain install nightly-2026-04-11 --profile minimal \
        -c rustfmt -c miri -c rust-src -c llvm-tools \
    && rustup +nightly-2026-04-11 target add \
        x86_64-unknown-none armv7a-none-eabi armv7r-none-eabihf aarch64-unknown-none \
        thumbv7m-none-eabi thumbv7em-none-eabi thumbv7em-none-eabihf \
        riscv32imc-unknown-none-elf riscv64imac-unknown-none-elf \
    && . /opt/nros-ssot/scripts/lib/rust-targets.sh \
    && nros_rust_targets rustup | xargs -r rustup +nightly-2026-04-11 target add

# git-as-root in CI containers: avoid "dubious ownership" on the checkout + the
# source submodules `nros setup` provisions.
RUN git config --system --add safe.directory '*'
