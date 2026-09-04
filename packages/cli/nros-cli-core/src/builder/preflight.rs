//! Stage 3 — is this build's toolchain present? (phase-383 W2.d, RFC-0065 D2/D14).
//!
//! ## What this checks, and what it deliberately does not
//!
//! A missing prerequisite must fail HERE, naming the `nros setup` line that
//! fixes it — never mid-compile with a cryptic linker error. That is D2's whole
//! promise.
//!
//! It checks what is **cheap and board-specific**: the Rust target the board
//! pins, and the SDK directories its site config names. It does NOT re-walk the
//! whole `nros-sdk-index.toml` the way `nros setup --check` does. Two reasons:
//!
//! * `nros setup --check` prints a report; its predicate is not separable
//!   without refactoring `setup.rs`, and doing that inside a wave about
//!   something else is how unrelated changes ride along;
//! * a full index walk probes every tool for every board, most of which this
//!   build does not need. Preflight should be fast enough that nobody wants a
//!   flag to skip it — a skipped check is not a check.
//!
//! So this is the narrow, high-yield subset, and it says so rather than
//! implying full coverage. `nros doctor` remains the thorough answer.
//!
//! ## Offline (D14)
//!
//! Preflight never fetches. `--offline` changes nothing here — it changes what
//! stage 5 is told — which is the point: a build that would have fetched fails
//! at stage 3 either way, with the manual command named.

use std::path::Path;

use crate::orchestration::board_descriptor::BoardDescriptor;

/// A missing prerequisite, with the command that installs it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Missing {
    /// What is absent, in the user's vocabulary.
    pub what: String,
    /// The exact command to run. Never a description of one.
    pub remedy: String,
}

/// Check the prerequisites `board` needs, in the workspace at `root`.
///
/// Returns every problem rather than the first: a user fixing three missing
/// things one build at a time is three round trips, and D2's promise is that
/// stage 3 tells you everything before anything compiles.
#[must_use]
pub fn check(board: &BoardDescriptor, root: &Path, nano_ros_root: Option<&Path>) -> Vec<Missing> {
    let mut out = Vec::new();

    // The rustc target triple. A cross board that pins one needs it installed,
    // and `cargo build --target` fails deep in the build otherwise.
    if let Some(target) = board.target.as_deref() {
        match target_provisioning(nano_ros_root, target) {
            // A prebuilt `rust-std` exists, so "is it installed" is the right
            // question and `rustup target add` is the answer.
            Provisioning::Rustup => {
                if !rust_target_installed(target) {
                    out.push(Missing {
                        what: format!("Rust target `{target}` (board `{}`)", board.names[0]),
                        remedy: format!("rustup target add {target}"),
                    });
                }
            }
            // Tier 3 / custom JSON: there is NOTHING to install, and this used
            // to say `rustup target add armv7a-nuttx-eabihf` — a command that
            // cannot succeed. `config/rust-targets.txt` says so in as many
            // words: "There is nothing to install, so neither the installer nor
            // the doctor touches these rows."
            //
            // It cost the nightly's `nuttx` and `esp32` cells, which failed
            // preflight before compiling anything, with a remedy that would
            // have failed too. The requirement these targets DO have is the
            // `rust-src` component, because the leaf builds them with
            // `-Zbuild-std`.
            Provisioning::BuildStd => {
                if !rust_component_installed("rust-src") {
                    out.push(Missing {
                        what: format!(
                            "`rust-src` for build-std target `{target}` (board `{}`)",
                            board.names[0]
                        ),
                        remedy: "rustup component add rust-src".to_string(),
                    });
                }
            }
        }
    }

    // A workspace that has never been synced has no generated message crates,
    // and every leaf `.cargo/config.toml` include points at a tree that does
    // not exist — a cargo manifest-PARSE error four frames deep that never
    // names sync (issue 0463).
    //
    // **Two locations, and probing only one was a defect W8 caught.** `nros
    // sync` writes generated msg crates to `<ws>/generated/` and resolved
    // models to `<ws>/build/nros/`. The first version of this check looked only
    // at `build/nros`, so `nano-ros-rt-eval` — which carries a populated
    // `generated/` with fourteen msg packages — was told it had never been
    // synced. Either output is evidence that sync has run.
    let synced = root.join("build/nros").is_dir() || root.join("generated").is_dir();
    if root.join("src").is_dir() && !synced {
        out.push(Missing {
            what: "generated message bindings (this workspace has never been synced)".to_string(),
            remedy: "nros sync".to_string(),
        });
    }

    out
}

/// How `config/rust-targets.txt` says a triple is provided.
///
/// Column 2 of the ONE list (issue 0833). Unknown or unreadable ⇒ `Rustup`,
/// which is the pre-existing behaviour: a triple nobody declared is more likely
/// a normal target than a custom JSON one, and preflight must never refuse a
/// build the compiler would have accepted.
#[derive(Debug, PartialEq, Eq)]
enum Provisioning {
    Rustup,
    BuildStd,
}

fn target_provisioning(nano_ros_root: Option<&Path>, target: &str) -> Provisioning {
    // The NANO-ROS CHECKOUT, not the workspace root — `config/rust-targets.txt`
    // lives in this repository, and a user's workspace has no such file.
    //
    // The first version of this read it from the workspace root, which is what
    // `preflight::check` is otherwise given. It therefore found nothing, fell
    // back to `Rustup`, and changed NOTHING: the nightly's `nuttx` cell failed
    // on the identical `rustup target add armv7a-nuttx-eabihf` line after the
    // "fix" landed. The local check that was supposed to catch that passed for
    // an unrelated reason, so the wrong root shipped.
    let Some(repo) = nano_ros_root else {
        return Provisioning::Rustup;
    };
    let Ok(text) = std::fs::read_to_string(repo.join("config/rust-targets.txt")) else {
        return Provisioning::Rustup;
    };
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut cols = line.split_whitespace();
        if cols.next() == Some(target) {
            return match cols.next() {
                Some("build-std") => Provisioning::BuildStd,
                _ => Provisioning::Rustup,
            };
        }
    }
    Provisioning::Rustup
}

/// Whether `rustup` reports a COMPONENT as installed.
///
/// Same false-negative-is-safe rule as [`rust_target_installed`]: no rustup ⇒
/// report installed and let the build speak.
fn rust_component_installed(component: &str) -> bool {
    let Ok(out) = std::process::Command::new("rustup")
        .args(["component", "list", "--installed"])
        .output()
    else {
        return true;
    };
    if !out.status.success() {
        return true;
    }
    String::from_utf8_lossy(&out.stdout)
        .lines()
        .any(|l| l.trim() == component || l.trim().starts_with(&format!("{component}-")))
}

/// Whether `rustup` reports `target` as installed.
///
/// A false NEGATIVE is the safe direction: with no rustup (a distro toolchain,
/// a nix shell), this reports installed and lets the build speak for itself.
/// Preflight exists to give a better message than the compiler, never to refuse
/// a build the compiler would have accepted.
/// The installed-target list, probed ONCE per process.
///
/// `None` means the probe could not answer -- no `rustup` on PATH, or it
/// failed. Both callers then report the target as installed, because inventing
/// a missing target is the worse error.
///
/// Probed once because the two failure modes are INDISTINGUISHABLE at the call
/// site and only one of them is stable. "no rustup here" gives the same `Err`
/// as a spawn that failed under load, and under an 80-way parallel build the
/// second happens intermittently -- so two calls in one process could disagree,
/// and `check()` would report a target as present that a caller had just
/// measured as absent. Caching does not make the probe more accurate; it makes
/// the process SELF-CONSISTENT, which is what a caller comparing two answers
/// actually needs. Issue 0726's class.
fn installed_rust_targets() -> &'static Option<Vec<String>> {
    static CACHE: std::sync::OnceLock<Option<Vec<String>>> = std::sync::OnceLock::new();
    CACHE.get_or_init(|| {
        let out = std::process::Command::new("rustup")
            .args(["target", "list", "--installed"])
            .output()
            .ok()?;
        if !out.status.success() {
            return None;
        }
        Some(
            String::from_utf8_lossy(&out.stdout)
                .lines()
                .map(|l| l.trim().to_string())
                .collect(),
        )
    })
}

fn rust_target_installed(target: &str) -> bool {
    match installed_rust_targets() {
        // Cannot answer -- see above.
        None => true,
        Some(list) => list.iter().any(|l| l == target),
    }
}

/// Render the problems as the message stage 3 fails with.
#[must_use]
pub fn report(missing: &[Missing]) -> String {
    let mut s = String::from("missing prerequisites for this build:\n");
    for m in missing {
        s.push_str(&format!("  - {}\n      run: {}\n", m.what, m.remedy));
    }
    s.push_str(
        "\nnothing was built. `nros doctor` checks the whole toolchain; this \
         list is only what THIS image needs.",
    );
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::orchestration::board_descriptor::BoardCatalog;

    #[derive(serde::Deserialize)]
    struct BoardFile {
        #[serde(rename = "board")]
        boards: Vec<BoardDescriptor>,
    }

    fn board(extra: &str) -> BoardDescriptor {
        let src = format!(
            "[[board]]\nnames = [\"testboard\"]\nplatform = \"freertos\"\n\
             toolchain = \"stable\"\nplatform_feature = \"platform-freertos\"\n\
             link_kind = \"none\"\nentry_kind = \"board-run\"\n{extra}"
        );
        let f: BoardFile = toml::from_str(&src).expect("parse");
        BoardCatalog::from_descriptors(f.boards)
            .descriptors()
            .first()
            .cloned()
            .expect("one board")
    }

    #[test]
    fn a_board_with_no_pinned_target_needs_no_rust_target() {
        let tmp = tempfile::tempdir().unwrap();
        let m = check(&board(""), tmp.path(), None);
        assert!(!m.iter().any(|m| m.what.contains("Rust target")), "{m:?}");
    }

    /// A `build-std` target must NOT be reported as an uninstallable rustup one.
    ///
    /// `config/rust-targets.txt` marks `armv7a-nuttx-eabihf` and
    /// `riscv32imac-unknown-nuttx-elf` as `build-std`: Tier 3, no prebuilt
    /// `rust-std`, provided by `-Zbuild-std` from the leaf. Preflight demanded
    /// `rustup target add` for them anyway — a prerequisite that can never be
    /// satisfied — and the nightly's `nuttx` and `esp32` cells died at stage 3
    /// with a remedy that would also have failed.
    ///
    /// Reads the REAL list rather than a fixture: the point of issue 0833's one
    /// list is that a second copy drifts, and a test asserting against its own
    /// private copy would be exactly that.
    #[test]
    fn the_provisioning_column_decides_which_prerequisite_is_checked() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .find(|p| p.join("config/rust-targets.txt").is_file())
            .expect("the repo's rust-targets list");

        assert_eq!(
            target_provisioning(Some(root), "armv7a-nuttx-eabihf"),
            Provisioning::BuildStd,
            "a NuttX target is build-std; demanding `rustup target add` for it \
             names a command that cannot succeed"
        );
        assert_eq!(
            target_provisioning(Some(root), "thumbv7m-none-eabi"),
            Provisioning::Rustup,
            "a target with a prebuilt rust-std keeps the rustup check"
        );
        // A triple nobody declared falls back to the pre-existing behaviour.
        assert_eq!(
            target_provisioning(Some(root), "nros-not-a-declared-triple"),
            Provisioning::Rustup
        );
    }

    #[test]
    fn an_uninstalled_target_is_reported_with_the_rustup_line() {
        let tmp = tempfile::tempdir().unwrap();
        // A triple no host has installed, and which is not a real target — so
        // this cannot pass by accident on a well-provisioned machine.
        let b = board("target = \"nros-not-a-real-triple\"\n");
        // Probe ONCE, before `check`, and branch on that one answer.
        //
        // `rust_target_installed` returns TRUE when the fork fails (`:186`,
        // `:189`) -- "no rustup here, so report installed". Asking twice means
        // the two answers can disagree under load: `check`'s call forks fine and
        // reports the missing target, this one fails to spawn and takes the
        // "nothing to assert" branch, and the test fails on a tree that is
        // correct. Observed in `check::build`'s parallel lane; passes solo and
        // twice in a row standalone.
        //
        // Issue 0726's class exactly: a forked process that failed to START
        // under a fan-out, reported as a finding about the source tree.
        let installed = rust_target_installed("nros-not-a-real-triple");
        let m = check(&b, tmp.path(), None);
        if installed {
            // No rustup on this host: the check reports installed by design
            // (see `rust_target_installed`), so there is nothing to assert.
            assert!(m.iter().all(|m| !m.what.contains("Rust target")));
            return;
        }
        let hit = m
            .iter()
            .find(|m| m.what.contains("Rust target"))
            .expect("must report the missing target");
        assert_eq!(hit.remedy, "rustup target add nros-not-a-real-triple");
    }

    /// A `build-std` board is not told to `rustup target add` — through
    /// `check()`, with the REAL checkout.
    ///
    /// The unit test on `target_provisioning` alone was not enough, and this is
    /// the test that would have caught what shipped: the classifier was right
    /// and the ROOT handed to it was wrong. `check()` receives the WORKSPACE
    /// root, `config/rust-targets.txt` lives in the nano-ros CHECKOUT, so the
    /// lookup found nothing, fell back to `Rustup`, and the nightly's `nuttx`
    /// cell failed on the same `rustup target add armv7a-nuttx-eabihf` line the
    /// fix was supposed to remove.
    ///
    /// I "verified" that version with a `nros build --dry-run` that passed for
    /// an unrelated reason. A passing command is not a passing assertion.
    #[test]
    fn a_build_std_board_is_never_told_to_rustup_target_add() {
        let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .find(|p| p.join("config/rust-targets.txt").is_file())
            .expect("the repo's rust-targets list");
        let ws = tempfile::tempdir().unwrap();
        let b = board("target = \"armv7a-nuttx-eabihf\"\n");

        let m = check(&b, ws.path(), Some(repo));
        assert!(
            !m.iter().any(|m| m.remedy.starts_with("rustup target add")),
            "a build-std target has nothing to install; reported: {:?}",
            m.iter().map(|m| &m.remedy).collect::<Vec<_>>()
        );

        // ...and with NO checkout to read the list from, the old conservative
        // behaviour stands rather than a silent pass.
        let blind = check(&b, ws.path(), None);
        if !rust_target_installed("armv7a-nuttx-eabihf") {
            assert!(
                blind
                    .iter()
                    .any(|m| m.remedy.starts_with("rustup target add")),
                "without the list this cannot be classified, and preflight must \
                 not invent a pass"
            );
        }
    }

    #[test]
    fn an_unsynced_workspace_is_told_to_sync() {
        // issue 0463 — without this the failure is a cargo manifest-PARSE error
        // four frames deep that never names `nros sync`.
        let tmp = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(tmp.path().join("src/talker_pkg")).unwrap();
        let m = check(&board(""), tmp.path(), None);
        let hit = m
            .iter()
            .find(|m| m.remedy == "nros sync")
            .expect("must name sync");
        assert!(hit.what.contains("never been synced"), "{hit:?}");
    }

    #[test]
    fn a_synced_workspace_is_not_flagged() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(tmp.path().join("src/talker_pkg")).unwrap();
        std::fs::create_dir_all(tmp.path().join("build/nros")).unwrap();
        let m = check(&board(""), tmp.path(), None);
        assert!(m.iter().all(|m| m.remedy != "nros sync"), "{m:?}");
    }

    #[test]
    fn generated_msg_crates_also_count_as_synced() {
        // phase-383 W8.a — `nano-ros-rt-eval` carries a populated `generated/`
        // with fourteen msg packages and no `build/nros`. Probing only the
        // latter told a fully synced workspace it had never been synced.
        let tmp = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(tmp.path().join("src/actuator_pkg")).unwrap();
        std::fs::create_dir_all(tmp.path().join("generated/std_msgs")).unwrap();
        let m = check(&board(""), tmp.path(), None);
        assert!(
            m.iter().all(|m| m.remedy != "nros sync"),
            "generated/ is sync output too: {m:?}"
        );
    }

    #[test]
    fn the_report_names_every_problem_and_the_exact_command() {
        // Every problem, not the first: three missing things fixed one build at
        // a time is three round trips.
        let missing = vec![
            Missing {
                what: "a".to_string(),
                remedy: "nros setup a".to_string(),
            },
            Missing {
                what: "b".to_string(),
                remedy: "nros setup b".to_string(),
            },
        ];
        let r = report(&missing);
        assert!(r.contains("nros setup a"), "{r}");
        assert!(r.contains("nros setup b"), "{r}");
        assert!(r.contains("nothing was built"), "{r}");
    }
}
