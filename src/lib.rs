use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PendulumParams {
    pub mass: f64,         // kg
    pub length: f64,       // m
    pub gravity: f64,      // m/s^2
    pub damping: f64,      // N·m·s/rad
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PidParams {
    pub kp: f64,           // Proportional gain
    pub ki: f64,           // Integral gain
    pub kd: f64,           // Derivative gain
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct State {
    pub theta: f64,        // rad
    pub theta_dot: f64,    // rad/s
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SimulationResult {
    pub time: f64,
    pub state: State,
    pub control: f64,      // N·m
}

pub struct PidController {
    params: PidParams,
    integral: f64,
    prev_error: f64,
}

impl PidController {
    pub fn new(params: PidParams) -> Self {
        Self {
            params,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    pub fn compute(&mut self, setpoint: f64, current: f64, dt: f64) -> f64 {
        let error = setpoint - current;
        
        self.integral += error * dt;
        
        let derivative = if dt > 0.0 {
            (error - self.prev_error) / dt
        } else {
            0.0
        };
        
        self.prev_error = error;
        
        self.params.kp * error + self.params.ki * self.integral + self.params.kd * derivative
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}

pub fn pendulum_dynamics(state: &State, control: f64, params: &PendulumParams) -> State {
    let theta = state.theta;
    let theta_dot = state.theta_dot;
    
    let theta_ddot = (params.gravity / params.length) * theta.sin() 
                   - (params.damping / (params.mass * params.length * params.length)) * theta_dot
                   + control / (params.mass * params.length * params.length);
    
    State {
        theta: theta_dot,
        theta_dot: theta_ddot,
    }
}

pub fn rk4_step(
    state: &State,
    control: f64,
    params: &PendulumParams,
    dt: f64,
) -> State {
    let k1 = pendulum_dynamics(state, control, params);
    
    let state2 = State {
        theta: state.theta + 0.5 * dt * k1.theta,
        theta_dot: state.theta_dot + 0.5 * dt * k1.theta_dot,
    };
    let k2 = pendulum_dynamics(&state2, control, params);
    
    let state3 = State {
        theta: state.theta + 0.5 * dt * k2.theta,
        theta_dot: state.theta_dot + 0.5 * dt * k2.theta_dot,
    };
    let k3 = pendulum_dynamics(&state3, control, params);
    
    let state4 = State {
        theta: state.theta + dt * k3.theta,
        theta_dot: state.theta_dot + dt * k3.theta_dot,
    };
    let k4 = pendulum_dynamics(&state4, control, params);
    
    State {
        theta: state.theta + (dt / 6.0) * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta),
        theta_dot: state.theta_dot + (dt / 6.0) * (k1.theta_dot + 2.0 * k2.theta_dot + 2.0 * k3.theta_dot + k4.theta_dot),
    }
}

pub fn simulate(
    initial_state: State,
    pendulum_params: PendulumParams,
    pid_params: PidParams,
    dt: f64,
    duration: f64,
) -> Vec<SimulationResult> {
    let mut results = Vec::new();
    let mut state = initial_state;
    let mut controller = PidController::new(pid_params);
    let mut time = 0.0;
    
    let setpoint = 0.0; // Target: upright position
    
    while time <= duration {
        let control = controller.compute(setpoint, state.theta, dt);
        
        results.push(SimulationResult {
            time,
            state,
            control,
        });
        
        state = rk4_step(&state, control, &pendulum_params, dt);
        time += dt;
    }
    
    results
}

#[cfg(feature = "python")]
use pyo3::prelude::*;

#[cfg(feature = "python")]
#[pyfunction]
pub fn simulate_pendulum(
    mass: f64,
    length: f64,
    gravity: f64,
    damping: f64,
    kp: f64,
    ki: f64,
    kd: f64,
    initial_theta: f64,
    initial_theta_dot: f64,
    dt: f64,
    duration: f64,
) -> PyResult<String> {
    let pendulum_params = PendulumParams {
        mass,
        length,
        gravity,
        damping,
    };
    
    let pid_params = PidParams { kp, ki, kd };
    
    let initial_state = State {
        theta: initial_theta,
        theta_dot: initial_theta_dot,
    };
    
    let results = simulate(initial_state, pendulum_params, pid_params, dt, duration);
    
    Ok(serde_json::to_string(&results).unwrap())
}

#[cfg(feature = "python")]
#[pymodule]
fn inverted_pendulum_pid(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(simulate_pendulum, m)?)?;
    Ok(())
}