#!/usr/bin/env python3
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
import inverted_pendulum_pid
import argparse

def run_simulation(mass=1.0, length=1.0, gravity=9.81, damping=0.1,
                  kp=100.0, ki=50.0, kd=20.0,
                  initial_theta=0.1, initial_theta_dot=0.0,
                  dt=0.01, duration=10.0):
    """Run the pendulum simulation and return the results."""
    results_json = inverted_pendulum_pid.simulate_pendulum(
        mass, length, gravity, damping,
        kp, ki, kd,
        initial_theta, initial_theta_dot,
        dt, duration
    )
    return json.loads(results_json)

def animate_pendulum(results, length=1.0, interval=10, save_video=True, filename='pendulum_simulation.mp4', max_frames=500):
    """Animate the pendulum simulation results."""
    # Downsample if too many frames
    if len(results) > max_frames:
        step = len(results) // max_frames
        results = results[::step]
        print(f"Downsampling animation to {len(results)} frames for performance")
    
    # Extract data
    times = [r['time'] for r in results]
    thetas = [r['state']['theta'] for r in results]
    theta_dots = [r['state']['theta_dot'] for r in results]
    controls = [r['control'] for r in results]
    
    # Set up the figure and axes
    fig = plt.figure(figsize=(12, 8))
    
    # Pendulum animation
    ax1 = plt.subplot(2, 2, 1)
    ax1.set_xlim(-1.5*length, 1.5*length)
    ax1.set_ylim(-1.5*length, 1.5*length)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Pendulum Animation')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    
    # Initialize pendulum components
    line, = ax1.plot([], [], 'o-', lw=3, color='darkblue')
    cart = Rectangle((-0.1, -0.05), 0.2, 0.1, fc='gray')
    ax1.add_patch(cart)
    trail, = ax1.plot([], [], 'r-', lw=1, alpha=0.3)
    
    # Angle vs time
    ax2 = plt.subplot(2, 2, 2)
    ax2.set_xlim(0, times[-1])
    ax2.set_ylim(min(thetas)*1.1, max(thetas)*1.1)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angle (rad)')
    ax2.set_title('Angle vs Time')
    ax2.grid(True, alpha=0.3)
    angle_line, = ax2.plot([], [], 'b-')
    angle_point, = ax2.plot([], [], 'ro')
    
    # Angular velocity vs time
    ax3 = plt.subplot(2, 2, 3)
    ax3.set_xlim(0, times[-1])
    ax3.set_ylim(min(theta_dots)*1.1, max(theta_dots)*1.1)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angular Velocity (rad/s)')
    ax3.set_title('Angular Velocity vs Time')
    ax3.grid(True, alpha=0.3)
    velocity_line, = ax3.plot([], [], 'g-')
    velocity_point, = ax3.plot([], [], 'ro')
    
    # Control effort vs time
    ax4 = plt.subplot(2, 2, 4)
    ax4.set_xlim(0, times[-1])
    ax4.set_ylim(min(controls)*1.1, max(controls)*1.1)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Control Torque (N·m)')
    ax4.set_title('Control Effort vs Time')
    ax4.grid(True, alpha=0.3)
    control_line, = ax4.plot([], [], 'm-')
    control_point, = ax4.plot([], [], 'ro')
    
    # Trail data
    trail_x = []
    trail_y = []
    
    def init():
        line.set_data([], [])
        trail.set_data([], [])
        angle_line.set_data([], [])
        angle_point.set_data([], [])
        velocity_line.set_data([], [])
        velocity_point.set_data([], [])
        control_line.set_data([], [])
        control_point.set_data([], [])
        return line, cart, trail, angle_line, angle_point, velocity_line, velocity_point, control_line, control_point
    
    def animate(i):
        if i >= len(results):
            return line, cart, trail, angle_line, angle_point, velocity_line, velocity_point, control_line, control_point
        
        theta = thetas[i]
        
        # Pendulum position
        x = length * np.sin(theta)
        y = length * np.cos(theta)
        
        # Update pendulum
        line.set_data([0, x], [0, y])
        
        # Update trail
        trail_x.append(x)
        trail_y.append(y)
        if len(trail_x) > 50:  # Keep last 50 points
            trail_x.pop(0)
            trail_y.pop(0)
        trail.set_data(trail_x, trail_y)
        
        # Update plots
        angle_line.set_data(times[:i+1], thetas[:i+1])
        angle_point.set_data([times[i]], [thetas[i]])
        
        velocity_line.set_data(times[:i+1], theta_dots[:i+1])
        velocity_point.set_data([times[i]], [theta_dots[i]])
        
        control_line.set_data(times[:i+1], controls[:i+1])
        control_point.set_data([times[i]], [controls[i]])
        
        # Update time in title
        fig.suptitle(f'Inverted Pendulum PID Control - Time: {times[i]:.2f}s', fontsize=14)
        
        return line, cart, trail, angle_line, angle_point, velocity_line, velocity_point, control_line, control_point
    
    # Create animation
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                 frames=len(results), interval=interval,
                                 blit=True, repeat=True)
    
    plt.tight_layout()
    
    if save_video:
        print(f"Saving animation to {filename}...")
        fps = 1000 / interval
        
        # Try to use ffmpeg if available for better performance
        try:
            writer = animation.FFMpegWriter(fps=fps, metadata=dict(artist='Inverted Pendulum Simulator'), bitrate=1800)
            anim.save(filename, writer=writer)
            print(f"Animation saved to {filename}")
        except:
            # Fall back to GIF if ffmpeg is not available
            if filename.endswith('.mp4'):
                gif_filename = filename.replace('.mp4', '.gif')
                print(f"FFmpeg not found. Saving as GIF instead: {gif_filename}")
            else:
                gif_filename = filename
            
            writer = animation.PillowWriter(fps=fps, metadata=dict(artist='Inverted Pendulum Simulator'))
            anim.save(gif_filename, writer=writer)
            print(f"Animation saved to {gif_filename}")
        
        plt.close()
    else:
        plt.show()
    
    return anim

def main():
    parser = argparse.ArgumentParser(description='Animate inverted pendulum PID simulation')
    parser.add_argument('--mass', type=float, default=1.0, help='Mass of pendulum (kg)')
    parser.add_argument('--length', type=float, default=1.0, help='Length of pendulum (m)')
    parser.add_argument('--gravity', type=float, default=9.81, help='Gravitational acceleration (m/s^2)')
    parser.add_argument('--damping', type=float, default=0.1, help='Damping coefficient (N·m·s/rad)')
    parser.add_argument('--kp', type=float, default=100.0, help='PID proportional gain')
    parser.add_argument('--ki', type=float, default=50.0, help='PID integral gain')
    parser.add_argument('--kd', type=float, default=20.0, help='PID derivative gain')
    parser.add_argument('--initial-theta', type=float, default=0.5, help='Initial angle from vertical (rad)')
    parser.add_argument('--initial-theta-dot', type=float, default=0.0, help='Initial angular velocity (rad/s)')
    parser.add_argument('--dt', type=float, default=0.01, help='Time step size (s)')
    parser.add_argument('--duration', type=float, default=10.0, help='Simulation duration (s)')
    parser.add_argument('--interval', type=int, default=10, help='Animation interval (ms)')
    parser.add_argument('--output', type=str, default='pendulum_simulation.gif', help='Output video filename (.mp4 if ffmpeg available, .gif otherwise)')
    parser.add_argument('--show', action='store_true', help='Display animation instead of saving to file')
    parser.add_argument('--max-frames', type=int, default=300, help='Maximum frames for animation (downsamples if needed)')
    
    args = parser.parse_args()
    
    print("Running simulation...")
    results = run_simulation(
        mass=args.mass,
        length=args.length,
        gravity=args.gravity,
        damping=args.damping,
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        initial_theta=args.initial_theta,
        initial_theta_dot=args.initial_theta_dot,
        dt=args.dt,
        duration=args.duration
    )
    
    print(f"Simulation complete. {len(results)} data points generated.")
    
    if args.show:
        print("Displaying animation...")
    else:
        print(f"Creating video: {args.output}")
    
    animate_pendulum(results, length=args.length, interval=args.interval, 
                    save_video=not args.show, filename=args.output, max_frames=args.max_frames)

if __name__ == "__main__":
    main()