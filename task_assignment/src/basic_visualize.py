# import numpy as np
# import pybullet
# import time
# import visualize3d

# viz = visualize3d.gui()
# while True:
#     q = np.random.uniform(-np.pi/2, np.pi/2, 6)
#     viz.set_joints(q)
#     time.sleep(1)

import time
import argparse
import visualize3d

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="Replay robot simulation from log file.")
    parser.add_argument(
        "--log",
        type=str,
        default="log.txt",
        help="Path to the log file (default: log.txt)"
    )
    args = parser.parse_args()

    log_file = args.log
    print(f"Using log file: {log_file}")

    # Start PyBullet GUI
    viz = visualize3d.gui()

    with open(log_file, "r") as f:
        for line in f:
            q = None

            if "state:" in line:
                # Old format
                q_str = line.split("state:")[1].split("|")[0].strip()
                try:
                    q = [float(val) for val in q_str.split()]
                except Exception as e:
                    print(f"Error parsing 'state:' line: {q_str} -> {e}")

            elif "q:" in line:
                # New format with q: and dq:
                q_part = line.split("q:", 1)[1]  # take everything after first q:
                # Cut off at dq: (works even if tabs/spaces before dq:)
                if "dq:" in q_part:
                    q_part = q_part.split("dq:")[0]
                q_str = q_part.strip()
                try:
                    q = [float(val) for val in q_str.split()]
                except Exception as e:
                    print(f"Error parsing 'q:' line: {q_str} -> {e}")

            # If parsed correctly, visualize
            if q is not None:
                viz.set_joints(q)
                time.sleep(0.002) # Causes delay in simulation

if __name__ == "__main__":
    main()
