# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This repository contains a Rust-based inverted pendulum PID control simulation with Python bindings.

### Project Structure
- `inverted_pendulum_pid/` - Main Rust project directory
  - `src/lib.rs` - Core simulation logic, PID controller, and RK4 solver
  - `src/main.rs` - CLI interface using clap
  - `Cargo.toml` - Rust dependencies and build configuration
  - `pyproject.toml` - Python package configuration for maturin
  - `animate_pendulum.py` - Python visualization script using matplotlib
  - `test_python_bindings.py` - Python bindings test script

## Development Commands

### Rust Development
```bash
cd inverted_pendulum_pid
cargo build          # Build debug version
cargo build --release  # Build optimized version
cargo run -- --help   # Run with help
cargo test           # Run tests
cargo clippy         # Run linter
```

### Python Development
```bash
cd inverted_pendulum_pid
source .venv/bin/activate  # Activate virtual environment
maturin develop --features python  # Build Python module
python animate_pendulum.py  # Run animation
```

### Key Dependencies
- Rust: clap (CLI), serde (serialization), pyo3 (Python bindings)
- Python: matplotlib, numpy (installed via uv)

## Architecture Notes

- The simulation uses RK4 (4th order Runge-Kutta) for ODE solving
- PID controller implementation includes integral windup protection
- All units are SI (meters, kilograms, seconds, radians)
- Python bindings return JSON-serialized results for flexibility