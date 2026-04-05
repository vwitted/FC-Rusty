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
}
