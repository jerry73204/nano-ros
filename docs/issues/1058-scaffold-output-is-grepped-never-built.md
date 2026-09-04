---
id: 1058
title: "`nros new` scaffold tests grep the emitted text and never build it, so a
  template can name three undeclared types and every test passes"
status: open
type: bug
area: cli, api
related: [phase-417, rfc-0043]
---

## Problem

`cargo-nano-ros/tests/integration_tests.rs` verifies every scaffold variant by
substring match on the generated sources — `assert!(hpp.contains("::nros::Result
configure(::nros::Node& node);"))` and ~30 siblings. Nothing compiles the
result.

So the tests answer "did we emit the string we meant to emit", which is a
question about the template, not about the scaffold. A user's first build is
the first compile the emitted code ever gets.

## How it surfaced

Phase-417 W-B3 migrated the C++ template's four type names from `::nros::` to
`::rclcpp::`. Three of the four do not exist:

| renamed to | reality |
| --- | --- |
| `rclcpp::Publisher<M>` | real alias (`publisher.hpp:381`) |
| `rclcpp::Result` | not declared anywhere in the tree |
| `rclcpp::Timer` | not declared anywhere in the tree |
| `rclcpp::Node` | a DISTINCT hosted class — `create_publisher<M>(name)` returns a `shared_ptr` where the template body passes an out-ref |

The migrated template therefore compiled in **no** configuration. The suite
went green: the only test that noticed was the one asserting the OLD string,
and it read as "the rename is incomplete" rather than "the rename is wrong".
Had the survey renamed that assertion too — the obvious next step — the defect
would have shipped with a fully green suite.

## Why it matters more than one bad edit

The scaffold is the first nano-ros code a new user compiles, so a broken
template is a first-run failure with no prior art to compare against. And the
gap is systematic, not specific to this edit: any change to a template — a
renamed header, a changed signature, a dropped include — is invisible to the
suite unless it also changes a string some assertion happens to name.

Same shape as the `<type_traits>` gap found the same day: a check standing next
to the real question, asking a proxy. There the proxy was a synthesised minimal
libcpp more complete than the real toolchain's; here it is text equality
standing in for compilation.

## Fix

Build at least one scaffold variant per language in `check-cli-tests`:
`nros new` into a temp dir, then configure + build it against the in-tree
`NANO_ROS_ROOT`. The C++ variant is the one that matters most — it is the only
template whose types come from a namespace that is partly aliases and partly
distinct classes, which is exactly the confusion that produced this.

Cost is a real compile per variant, so this is a `check-cli-tests` addition
rather than a fast-line one. `check-cli-tests` already builds
`nros-launch-resolve` before running, so the lane is not a no-compile lane.

Cheaper interim: assert every `::rclcpp::`-qualified name a template emits is
declared in the headers, by grepping the header set. That catches the
undeclared-type half (2 of the 3 here) without a build, but not the
`rclcpp::Node` half, whose name resolves and whose SHAPE differs — the
compile-or-conform hazard RFC-0089 exists for.
