use clap::Parser;
use inverted_pendulum_pid::{PendulumParams, PidParams, State, simulate};
use std::f64::consts::PI;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(long, default_value_t = 1.0, help = "Mass of pendulum (kg)")]
    mass: f64,

    #[arg(long, default_value_t = 1.0, help = "Length of pendulum (m)")]
    length: f64,

    #[arg(long, default_value_t = 9.81, help = "Gravitational acceleration (m/s^2)")]
    gravity: f64,

    #[arg(long, default_value_t = 0.1, help = "Damping coefficient (N·m·s/rad)")]
    damping: f64,

    #[arg(long, default_value_t = 100.0, help = "PID proportional gain")]
    kp: f64,

    #[arg(long, default_value_t = 50.0, help = "PID integral gain")]
    ki: f64,

    #[arg(long, default_value_t = 20.0, help = "PID derivative gain")]
    kd: f64,

    #[arg(long, default_value_t = 0.1, help = "Initial angle from vertical (rad)")]
    initial_theta: f64,

    #[arg(long, default_value_t = 0.0, help = "Initial angular velocity (rad/s)")]
    initial_theta_dot: f64,

    #[arg(long, default_value_t = 0.01, help = "Time step size (s)")]
    dt: f64,

    #[arg(long, default_value_t = 10.0, help = "Simulation duration (s)")]
    duration: f64,

    #[arg(long, help = "Output results as JSON")]
    json: bool,
}

fn main() {
    let args = Args::parse();

    let pendulum_params = PendulumParams {
        mass: args.mass,
        length: args.length,
        gravity: args.gravity,
        damping: args.damping,
    };

    let pid_params = PidParams {
        kp: args.kp,
        ki: args.ki,
        kd: args.kd,
    };

    let initial_state = State {
        theta: args.initial_theta,
        theta_dot: args.initial_theta_dot,
    };

    println!("Starting inverted pendulum PID simulation...");
    println!("Pendulum parameters:");
    println!("  Mass: {} kg", pendulum_params.mass);
    println!("  Length: {} m", pendulum_params.length);
    println!("  Gravity: {} m/s²", pendulum_params.gravity);
    println!("  Damping: {} N·m·s/rad", pendulum_params.damping);
    println!("\nPID parameters:");
    println!("  Kp: {}", pid_params.kp);
    println!("  Ki: {}", pid_params.ki);
    println!("  Kd: {}", pid_params.kd);
    println!("\nInitial state:");
    println!("  Theta: {} rad ({} degrees)", initial_state.theta, initial_state.theta * 180.0 / PI);
    println!("  Theta_dot: {} rad/s", initial_state.theta_dot);
    println!("\nSimulation parameters:");
    println!("  Time step: {} s", args.dt);
    println!("  Duration: {} s", args.duration);
    println!();

    let results = simulate(
        initial_state,
        pendulum_params,
        pid_params,
        args.dt,
        args.duration,
    );

    if args.json {
        println!("{}", serde_json::to_string_pretty(&results).unwrap());
    } else {
        println!("Simulation complete. {} data points generated.", results.len());
        println!("\nFinal state:");
        if let Some(final_result) = results.last() {
            println!("  Time: {:.3} s", final_result.time);
            println!("  Theta: {:.6} rad ({:.2} degrees)", 
                final_result.state.theta, 
                final_result.state.theta * 180.0 / PI);
            println!("  Theta_dot: {:.6} rad/s", final_result.state.theta_dot);
            println!("  Control: {:.6} N·m", final_result.control);
        }
    }
}