MODE = "simple"  # Options: "none", "numpy_vectorized", "torch_vectorized", "simple"
RENDER = False

import os
import sys
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="LEGO Assembly Demo")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = not RENDER
ext_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "source"))
sys.argv += ["--ext-folder", ext_folder, "--enable", "lego_assemble", "-v"]
sys.argv += [f"--/lego_assemble/mode={MODE}"]
app_launcher = AppLauncher(args_cli)


"""Rest everything follows."""

from isaacsim.simulation_app import SimulationApp
simulation_app: SimulationApp = app_launcher.app

import torch
import random
import time
import numpy as np
import matplotlib.pyplot as plt
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from lego_assemble.physics.interface import get_brick_physics_interface

random.seed(42)
np.random.seed(42)
torch.random.manual_seed(42)

# --- Scene Design ---
def design_scene(sim: sim_utils.SimulationContext):
    """
    Configures the ground plane and lighting for the simulation scene.
    """
    # Ground plane
    ground_plane_cfg = sim_utils.GroundPlaneCfg()
    ground_plane_cfg.func("/World/defaultGroundPlane", ground_plane_cfg)

    # Dome light
    light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
    light_cfg.func("/World/Light", light_cfg)

# --- Simulator Logic ---
def run_simulator(sim: sim_utils.SimulationContext):
    """
    Runs the simulation, adding bricks and measuring performance.
    """
    iface = get_brick_physics_interface() # Get the physics interface for bricks

    # PRNG for reproducible randomization
    seed = 42
    random_gen = random.Random(seed)

    # Parameters for brick generation and measurement
    bricks_per_batch = 50
    max_total_bricks = 5000
    # max_total_bricks = 50 # For quicker testing
    measurement_steps_N = 10

    # Position sampling ranges
    x_range = (-0.05, 0.05)
    y_range = (-0.05, 0.05)
    z_range = (0.5, 0.6) # Ensure bricks spawn above ground and have space

    performance_data = [] # To store (num_bricks, mean_step_time_ms, std_step_time_ms)
    total_bricks_in_scene = 0

    print(f"Starting simulation benchmark. Max bricks: {max_total_bricks}, Batch size: {bricks_per_batch}, N_measure: {measurement_steps_N}")

    early_stop = False

    # Main loop for adding bricks and measuring
    while total_bricks_in_scene < max_total_bricks and not early_stop:
        # 1. Add a batch of bricks
        bricks_added_this_batch = 0
        for _ in range(bricks_per_batch):
            if total_bricks_in_scene >= max_total_bricks:
                break # Stop if max bricks reached mid-batch

            pos_x = random_gen.uniform(x_range[0], x_range[1])
            pos_y = random_gen.uniform(y_range[0], y_range[1])
            pos_z = random_gen.uniform(z_range[0], z_range[1])

            # Example brick properties (can be randomized further if needed)
            # Dimensions are (studs_x, studs_y, height_plate_units)
            # Common LEGO brick dimensions: (2,2,3), (4,2,3), (1,1,3) etc.
            # Color names would depend on what `iface.create_brick` supports.
            # Let's use a fixed type for simplicity in this benchmark.
            iface.create_brick(
                dimensions=(2, 2, 3), # A standard 2x2 brick
                color_name="Red",      # Example color
                pos=(pos_x, pos_y, pos_z),
            )
            total_bricks_in_scene += 1
            bricks_added_this_batch +=1
        
        if bricks_added_this_batch == 0 and total_bricks_in_scene >= max_total_bricks:
            print("Max bricks reached, ending brick addition.")
            break
        
        print(f"Added {bricks_added_this_batch} bricks. Total bricks: {total_bricks_in_scene}")

        # 2. Take one "rest" step (unmeasured)
        # This allows physics to settle slightly after adding objects
        if total_bricks_in_scene > 0 : # Only step if bricks are present
            sim.step(render=RENDER)

        # 3. Measure performance for the next N steps
        step_times_s = []
        if total_bricks_in_scene > 0: # Only measure if there's something to simulate
            for _ in range(measurement_steps_N):
                start_time = time.perf_counter()
                sim.step(render=RENDER)
                end_time = time.perf_counter()
                step_times_s.append(end_time - start_time)

            # 4. Calculate mean and std deviation for this batch
            if step_times_s:
                mean_step_time_s = np.mean(step_times_s)
                std_step_time_s = np.std(step_times_s)
                mean_step_time_ms = mean_step_time_s * 1000
                std_step_time_ms = std_step_time_s * 1000
                early_stop = mean_step_time_ms > 300 # Stop if step time exceeds 1 second

                performance_data.append((total_bricks_in_scene, mean_step_time_ms, std_step_time_ms))
                print(f"Bricks: {total_bricks_in_scene}, Mean step time: {mean_step_time_ms:.3f} ms, Std: {std_step_time_ms:.3f} ms")
            else:
                print(f"Bricks: {total_bricks_in_scene}, No step times recorded for measurement (N={measurement_steps_N}).")

        # Check if the simulation app is still running (e.g., if user closes window)
        if not simulation_app.is_running():
            print("Simulation app was closed. Stopping benchmark.")
            break
            
    # --- Data Export and Plotting ---
    if not performance_data:
        print("No performance data collected.")
        return

    # Convert to NumPy array
    data_array = np.array(performance_data)

    # Export data as .npy file
    npy_filename = f"perf_{MODE}.npy"
    np.save(npy_filename, data_array)
    print(f"Performance data saved to {npy_filename}")

    # Plotting
    bricks_count = data_array[:, 0]
    mean_times = data_array[:, 1]
    std_times = data_array[:, 2]

    plt.figure(figsize=(10, 6))
    plt.errorbar(bricks_count, mean_times, yerr=std_times, fmt='-o', capsize=5, ecolor='red', label='Mean Step Time')
    plt.xlabel("Total Number of Bricks in Scene")
    plt.ylabel(f"Mean Step Time over N={measurement_steps_N} steps (ms)")
    plt.title("Simulation Step Time vs. Number of Bricks")
    plt.grid(True, which="both", linestyle="--", linewidth=0.5)
    plt.legend()
    
    plot_filename = f"perf_{MODE}.png"
    try:
        plt.savefig(plot_filename)
        print(f"Plot saved to {plot_filename}")
    except Exception as e:
        print(f"Error saving plot: {e}")
    # plt.show() # Optionally show plot if running in an environment that supports it

# --- Main Execution ---
def main():
    """
    Main function to set up and run the simulation.
    """
    # Simulation configuration
    # Modify sim_cfg based on available hardware and desired backend
    # sim_utils.SimulationCfg(device="cuda" if use_gpu else "cpu")
    sim_cfg = sim_utils.SimulationCfg(device="cpu") # As per original, or "cuda" if GPU is preferred/available
    sim = SimulationContext(sim_cfg)

    # Set camera view
    sim.set_camera_view(eye=[1.5, 0.0, 1.5], target=[0.0, 0.0, 0.2]) # Adjusted height for better view of spawning area

    # Design the scene (ground, lighting)
    design_scene(sim)

    # Reset simulation before starting
    sim.reset()
    print("Simulation scene designed and reset.")

    # Run the simulator logic
    run_simulator(sim)

    print("Simulation benchmark finished.")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"An error occurred in main: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Close the simulation app
        if 'simulation_app' in globals() and simulation_app is not None:
            print("Closing simulation app...")
            # simulation_app.close()
        print("Script execution complete.")
        exit(0)
