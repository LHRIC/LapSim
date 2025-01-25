import argparse
import numpy as np
import matplotlib.pyplot as plt

from mf_61 import MF61       # Import the MF61 class
from utility.parse_tir import parse_tir  # Import the parse_tir function

def main():
    """
    Simple test program to read .tir file parameters, instantiate MF61,
    and plot longitudinal & lateral force vs. slip ratio / slip angle.
    """
    
    # 1. Handle command-line arguments for the .tir file
    parser = argparse.ArgumentParser(description='Test MF61 model from .tir file')
    parser.add_argument('tir_file', type=str, help='Path to the .tir file')
    args = parser.parse_args()

    # 2. Parse the .tir file into a dictionary of parameters
    params = parse_tir(args.tir_file)

    # 3. Instantiate the MF61 object with parsed parameters
    mf_tire = MF61(params)

    # For demonstration, choose some typical operating conditions:
    fz   = 400.0       # Normal load (N) -- can tweak as you like
    gamma = 0.0        # Camber angle (rad), set to 0 for simplicity

    # 4a. Sweep over a range of slip ratio kappa (e.g. -0.2 to 0.2)
    kappa_vals = np.linspace(-1, 1, 200)
    fx_vals = []
    for kappa in kappa_vals:
        # Evaluate the longitudinal force fx
        fx_vals.append(mf_tire.fx(fz, kappa, gamma))

    # 4b. Sweep over a range of slip angles alpha (e.g. -10 to 10 degrees in radians)
    #     If you prefer to keep alpha in radians, you can simply do linspace in [-0.2, 0.2].
    alpha_deg_vals = np.linspace(-50, 50, 500)
    alpha_vals     = np.deg2rad(alpha_deg_vals)
    fy_vals        = []
    for alpha in alpha_vals:
        # Evaluate the lateral force fy
        fy_vals.append(mf_tire.fy(fz, alpha, gamma))

    # 5. Plot the results
    plt.figure(figsize=(8, 6))
    plt.plot(kappa_vals, fx_vals, label='Longitudinal Force Fx')
    plt.title('Longitudinal Force vs. Slip Ratio (kappa)')
    plt.xlabel('Slip Ratio, kappa')
    plt.ylabel('Longitudinal Force Fx (N)')
    plt.grid(True)
    plt.legend()

    plt.figure(figsize=(8, 6))
    plt.plot(alpha_deg_vals, fy_vals, label='Lateral Force Fy')
    plt.title('Lateral Force vs. Slip Angle (alpha)')
    plt.xlabel('Slip Angle, alpha (deg)')
    plt.ylabel('Lateral Force Fy (N)')
    plt.grid(True)
    plt.legend()

    # Show both plots
    plt.show()


if __name__ == '__main__':
    main()
