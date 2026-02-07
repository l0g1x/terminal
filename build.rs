use std::process::Command;

fn main() {
    // Embed the git commit hash at compile time so the binary knows its own version.
    let output = Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output();

    let hash = match output {
        Ok(o) if o.status.success() => {
            String::from_utf8_lossy(&o.stdout).trim().to_string()
        }
        _ => "unknown".to_string(),
    };

    println!("cargo:rustc-env=BUILD_GIT_HASH={hash}");
    // Only re-run if git HEAD changes.
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/refs/heads/");
}
