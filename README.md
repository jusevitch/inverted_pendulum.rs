# Inverted Pendulum PID Simulation

A Rust implementation of an inverted pendulum simulation with PID control, featuring Python bindings and visualization.

## Features

- **Rust Core**: High-performance simulation using the Runge-Kutta 4th order (RK4) method
- **PID Control**: Configurable proportional, integral, and derivative gains
- **CLI Interface**: Command-line interface for running simulations with custom parameters
- **Python Bindings**: PyO3-based Python interface for integration with scientific Python ecosystem
- **Visualization**: Animated matplotlib visualization of the pendulum dynamics

## Building

### Rust Binary

```bash
cargo build --release
```

### Python Module

Ensure you have a Python virtual environment activated:

```bash
uv venv
source .venv/bin/activate
uv pip install maturin matplotlib numpy
maturin develop --features python
```

## Usage

### Command Line Interface

Run the simulation with default parameters:

```bash
cargo run --release
```

With custom parameters:

```bash
cargo run --release -- --mass 2.0 --length 1.5 --initial-theta 0.3 --duration 5.0 --json
```

View all available options:

```bash
cargo run -- --help
```

### Python Animation

The animation script saves a video file (GIF or MP4) by default:

```bash
python animate_pendulum.py
```

With custom parameters:

```bash
python animate_pendulum.py --initial-theta 0.8 --kp 150 --ki 75 --kd 30 --output my_simulation.gif
```

To display the animation instead of saving:

```bash
python animate_pendulum.py --show
```

For long simulations, limit the number of frames:

```bash
python animate_pendulum.py --duration 30 --max-frames 300
```

## Parameters

### Physical Parameters
- `mass`: Mass of the pendulum (kg)
- `length`: Length of the pendulum (m)
- `gravity`: Gravitational acceleration (m/s²)
- `damping`: Damping coefficient (N·m·s/rad)

### Control Parameters
- `kp`: Proportional gain
- `ki`: Integral gain
- `kd`: Derivative gain

### Initial Conditions
- `initial-theta`: Initial angle from vertical (rad)
- `initial-theta-dot`: Initial angular velocity (rad/s)

### Simulation Parameters
- `dt`: Time step size (s)
- `duration`: Total simulation time (s)

## Examples

### Example 1: Small disturbance
```bash
cargo run -- --initial-theta 0.1 --duration 5.0
```

### Example 2: Large initial angle with strong control
```bash
python animate_pendulum.py --initial-theta 1.0 --kp 200 --ki 100 --kd 50 --output strong_control.gif
```

### Example 3: JSON output for data analysis
```bash
cargo run -- --initial-theta 0.5 --json > simulation_results.json
```

## Implementation Details

The simulation uses:
- **RK4 ODE Solver**: 4th-order Runge-Kutta method for numerical integration
- **PID Controller**: Standard PID control law with anti-windup
- **SI Units**: All parameters and outputs use SI units

The pendulum dynamics are modeled as:
```
θ̈ = (g/L)sin(θ) - (b/mL²)θ̇ + τ/(mL²)
```

Where:
- θ: angle from vertical
- g: gravitational acceleration
- L: pendulum length
- b: damping coefficient
- m: pendulum mass
- τ: control torque