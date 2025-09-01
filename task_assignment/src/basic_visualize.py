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
import numpy as np
import visualize3d

# Start PyBullet GUI
viz = visualize3d.gui()

# Open log file
with open("log.txt", "r") as f:
    for line in f:
        if "state:" in line:
            # Print the raw line for debugging log text file
            # print("Raw line:", line.strip())

            # Split line by 'state:' to get the joint positions from the log.txt file
            parts = line.split("state:")
            if len(parts) < 2:
                print("Cannot find 'state:' in line")
                continue

            # Take everything before the '|' character (exclude velocities)
            q_str = parts[1].split("|")[0].strip()
            print("Parsed joint string:", q_str)

            # Convert to float array
            try:
                q = [float(val) for val in q_str.split()]
                print("Joint angles:", q)
            except Exception as e:
                print("Error parsing joint angles:", e)
                continue

            # Set joints in PyBullet
            viz.set_joints(q)

            # Pause briefly so we can see it in GUI
            time.sleep(0.01)
