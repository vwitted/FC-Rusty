// build.rs
//
// Emits linker args for the embedded firmware binary.
// Guarded by target check so that host-target builds
// (e.g. cargo run --example sim_hover) don't fail
// looking for link.x and defmt.x.

fn main() {
    let target = std::env::var("TARGET").unwrap_or_default();
    if target.starts_with("thumbv") {
        println!("cargo:rustc-link-arg-bins=--nmagic");
        println!("cargo:rustc-link-arg-bins=-Tlink.x");
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    }
    emit_motor_test_env_deps();
    emit_build_stamp();
}

/// `motor_test.rs` reads its configuration through `option_env!`, which cargo
/// does not track as a build input. Without these, `LOOP_KHZ=2 ./scripts/
/// flash-motor-test.sh` recompiles nothing and flashes the *previous* config —
/// the change appears to have no effect on hardware. Declaring them makes a
/// changed value invalidate the build.
fn emit_motor_test_env_deps() {
    for var in [
        "M1_PCT", "M2_PCT", "M3_PCT", "M4_PCT", "BIDIR", "LOOP_KHZ", "DEADTIME_US",
    ] {
        println!("cargo:rerun-if-env-changed={var}");
    }
}

/// Emit `FC_BUILD_STAMP`, logged by the firmware at boot so a bench log can
/// be tied to the source that produced it. Stale-firmware confusion has cost
/// real bench time on this project — a hand-maintained constant only works if
/// you remember to bump it, so this is generated.
///
/// Format: `<epoch>-<git-sha>[-dirty]`, e.g. `1753488000-f74a35a-dirty`.
/// The git SHA identifies committed work; the epoch disambiguates successive
/// builds of a dirty tree, which is the usual bench case. Deliberately no
/// `cargo:rerun-if-changed`: cargo then re-runs this script whenever any file
/// in the package changes, so the stamp moves exactly when the sources do and
/// stays put when a rebuild is a genuine no-op.
///
/// `scripts/flash-*.sh` echo this same value plus the SHA-256 of the flashed
/// binary, so "what I flashed" and "what is running" can be compared directly.
fn emit_build_stamp() {
    use std::time::{SystemTime, UNIX_EPOCH};

    let epoch = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);

    let git = |args: &[&str]| -> Option<String> {
        let out = std::process::Command::new("git").args(args).output().ok()?;
        if !out.status.success() {
            return None;
        }
        Some(String::from_utf8_lossy(&out.stdout).trim().to_string())
    };

    let sha = git(&["rev-parse", "--short", "HEAD"]).unwrap_or_else(|| "nogit".into());
    let dirty = match git(&["status", "--porcelain"]) {
        Some(s) if !s.is_empty() => "-dirty",
        _ => "",
    };

    let stamp = format!("{epoch}-{sha}{dirty}");
    println!("cargo:rustc-env=FC_BUILD_STAMP={stamp}");

    // Also drop it where the flash scripts can read it without re-invoking
    // cargo. OUT_DIR is per-profile; walk up to target/ so the path is stable.
    if let Ok(out_dir) = std::env::var("OUT_DIR") {
        let root = std::path::Path::new(&out_dir)
            .ancestors()
            .find(|p| p.ends_with("target"))
            .map(|p| p.to_path_buf());
        if let Some(root) = root {
            let _ = std::fs::write(root.join("build-stamp.txt"), &stamp);
        }
    }
}
