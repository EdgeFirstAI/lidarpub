// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! End-to-end check that `KEY=""` in the environment behaves as unset.
//!
//! Runs with `harness = false` so this `main` is the only thread in the
//! process when the environment is mutated, which `scrub_empty_env` requires.
#![allow(dead_code)] // args.rs's own #[cfg(test)] unit tests are compiled but never run here

// `Args` lives in a private module of the binary, so include it directly.
// args.rs refers to `crate::common` and `crate::lidar`; alias them from the
// library crate so those paths resolve here.
use edgefirst_lidarpub::{common, lidar};

#[path = "../src/args.rs"]
mod args;
use args::{Args, KEEP, scrub_empty_env};
use clap::Parser;

/// Numeric, boolean, two-value (`num_args = 2`) and bare-flag arguments, all
/// written as `KEY=""` in /etc/default/lidarpub.
const VARS: [&str; 4] = ["CLUSTERING_EPS", "GROUND_FILTER", "AZIMUTH", "DISCOVER"];
const ARGV: [&str; 1] = ["edgefirst-lidarpub"];

fn main() {
    for name in VARS {
        // SAFETY: single-threaded — this is `main` before any thread is spawned.
        unsafe { std::env::set_var(name, "") };
    }
    let before = Args::try_parse_from(ARGV);
    assert!(
        before.is_err(),
        "empty vars must fail to parse before scrubbing: {before:?}"
    );

    // SAFETY: still single-threaded.
    unsafe { scrub_empty_env::<Args>(KEEP) };
    for name in VARS {
        assert!(
            std::env::var_os(name).is_none(),
            "{name} should have been removed"
        );
    }
    let args = Args::try_parse_from(ARGV).expect("defaults must apply after scrubbing");
    assert_eq!(args.clustering_eps, 200);
    assert!(!args.ground_filter);
    assert_eq!(args.azimuth, [0, 360]);
    assert!(!args.discover);
    println!("env_scrub: ok");
}
