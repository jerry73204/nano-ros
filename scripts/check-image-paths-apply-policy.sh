#!/usr/bin/env bash
# Issue 0719 — every path that BUILDS AN IMAGE must apply the image's policies.
#
# `nano_ros_entry()` is where an image's cross-cutting facts are applied, and
# `nano_ros_add_executable()` delegates to it, so ~160 call sites are covered by
# construction. A handful of paths cannot call the entry — a board seam and an
# ESP-IDF component are shaped by the build system that owns the image, not by
# the entry-package contract — and those must call `nros_apply_panic_policy()`
# instead. Two of them didn't, and each was found the same way: a
# `#[panic_handler]` error four crates from its cause (#0688, #0700).
#
# The predicate is STRUCTURAL, deliberately. The first sweep for this issue
# keyed on the text `nano_ros_entry(` and duly EXCLUDED the ESP-IDF shim,
# because a COMMENT in it mentions the name — a gate that reports a clean sweep
# over a site it never examined is issue 0196's rule failing in miniature. So
# comment lines are stripped before matching, and the check self-tests that
# stripping below.
set -o pipefail
cd "$(dirname "$0")/.."

# issue 1077 — `grep -q` on a pipeline can report a MATCH as a miss, and
# `if ! … | grep -q` cannot tell a tool error from a non-match. One helper.
# shellcheck source=lib/grep-q.sh
source "$(dirname "$0")/lib/grep-q.sh"

# A file "builds an image" if it creates an executable or registers an IDF
# component AND links the nano-ros umbrella.
mapfile -t candidates < <(
    git grep -l 'NanoRos::NanoRos' -- '*.cmake' '**/CMakeLists.txt' 'CMakeLists.txt' 2>/dev/null
)

fail=0
checked=0
for f in "${candidates[@]}"; do
    body="$(grep -vE '^[[:space:]]*#' "$f")"
    # The SELECTION above is a raw `git grep`, so a file that only NAMES the
    # umbrella in a comment lands here. Re-ask against the stripped body: this
    # gate's whole premise is that a mention in a comment is not a call, and
    # the candidate step was the one place it did not apply its own rule. A
    # comment reading "links no `NanoRos::NanoRosCpp`" pulled `nros-rmw-uorb`
    # into the subject set and failed it for not calling `nano_ros_entry()`.
    nros_grep_q 'NanoRos::NanoRos' <<<"$body" || continue
    nros_grep_q -E 'add_executable\(|idf_component_register\(' <<<"$body" || continue
    # Infrastructure that DEFINES the umbrella or the verbs is not an image path.
    case "$f" in
        CMakeLists.txt|nano_rosConfig.cmake|cmake/NanoRos*.cmake) continue ;;
        cmake/compat/*) continue ;;   # rclcpp shim: builds no nros image
        examples/templates/*) continue ;;  # copy-out templates, not built here
        packages/testing/*/fixtures/*) continue ;;  # compile-only smoke, never links
    esac
    checked=$((checked + 1))
    if nros_grep_q 'nano_ros_entry(\|nano_ros_add_executable(\|nros_apply_panic_policy(' <<<"$body"; then
        continue
    fi
    fail=1
    echo "  $f"
    echo "      builds an image and links NanoRos::, but calls neither"
    echo "      nano_ros_entry()/nano_ros_add_executable() nor nros_apply_panic_policy()."
done

# Self-test: the comment-stripping is the whole reason this gate is not the
# broken grep that preceded it, so prove it still strips.
probe="$(mktemp)"; trap 'rm -f "$probe"' EXIT
printf '# nano_ros_entry() named only in a comment\nadd_executable(x)\ntarget_link_libraries(x NanoRos::NanoRos)\n' > "$probe"
if nros_grep_q 'nano_ros_entry(' <<<"$(grep -vE '^[[:space:]]*#' "$probe")"; then
    echo "check-image-paths-apply-policy: SELF-TEST FAILED — comment stripping is broken," >&2
    echo "  so this gate would pass files it never examined. Fix before trusting it." >&2
    exit 1
fi

# Second half of the same rule, and the half that was missing: a file whose
# ONLY mention of the umbrella is a comment must not be a subject at all. The
# first probe proves a commented CALL is not counted; this proves a commented
# LINK does not summon the file. Without it, prose could fail a gate.
printf '# links no NanoRos::NanoRosCpp, deliberately\nadd_executable(x)\n' > "$probe"
if nros_grep_q 'NanoRos::NanoRos' <<<"$(grep -vE '^[[:space:]]*#' "$probe")"; then
    echo "check-image-paths-apply-policy: SELF-TEST FAILED — a file naming the umbrella" >&2
    echo "  only in a comment is still being selected, so a comment can fail this gate." >&2
    exit 1
fi

if [ "$fail" -ne 0 ]; then
    echo
    echo "check-image-paths-apply-policy: FAILED (issue 0719)"
    exit 1
fi
echo "check-image-paths-apply-policy: OK ($checked image path(s) outside the entry, all applying the policy)"
