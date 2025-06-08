#!/usr/bin/env python3
import json
import inverted_pendulum_pid

# Test the Python bindings
print("Testing Python bindings...")

results_json = inverted_pendulum_pid.simulate_pendulum(
    mass=1.0,
    length=1.0,
    gravity=9.81,
    damping=0.1,
    kp=100.0,
    ki=50.0,
    kd=20.0,
    initial_theta=0.5,
    initial_theta_dot=0.0,
    dt=0.01,
    duration=1.0  # Short duration for testing
)

results = json.loads(results_json)
print(f"Simulation complete. Generated {len(results)} data points.")
print(f"First data point: {results[0]}")
print(f"Last data point: {results[-1]}")
print("\nPython bindings working correctly!")