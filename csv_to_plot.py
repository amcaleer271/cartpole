"""
This python script was written in its entirety by ChatGPT
csv_to_plot.py

This script loads a CSV file containing:
position, angle, control, time

and creates 3 separate plots:
- Position vs Time
- Angle vs Time
- Control vs Time

Usage:
    python csv_to_plot.py path/to/data.csv
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt


def main():
    # Check command line arguments
    if len(sys.argv) != 2:
        print("Usage: python csv_to_plot.py <csv_filepath>")
        sys.exit(1)

    csv_path = sys.argv[1]

    # Load CSV
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error loading CSV file: {e}")
        sys.exit(1)

    # Extract columns
    try:
        position = df["Position X"]
        angle = df["Angle Theta"]
        control = df["Control u"]
        time = df["Time t"]
    except KeyError as e:
        print(f"Missing required column: {e}")
        sys.exit(1)

    # Position plot
    plt.figure()
    plt.plot(time, position)
    plt.xlabel("Time")
    plt.ylabel("Position")
    plt.title("Position vs Time")
    plt.grid(True)

    # Angle plot
    plt.figure()
    plt.plot(time, angle)
    plt.xlabel("Time")
    plt.ylabel("Angle")
    plt.title("Angle vs Time")
    plt.grid(True)

    # Control plot
    plt.figure()
    plt.plot(time, control)
    plt.xlabel("Time")
    plt.ylabel("Control")
    plt.title("Control vs Time")
    plt.grid(True)

    # Show all plots
    plt.show()


if __name__ == "__main__":
    main()