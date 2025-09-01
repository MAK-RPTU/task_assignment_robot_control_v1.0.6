import pybullet
import os

class gui:

    def __init__(self):
        self.gui = pybullet.GUI
        self.client = pybullet.connect(self.gui)

        self.robot = pybullet.loadURDF(
            os.path.dirname(__file__) + "/../share/robot_model/urdf/ur5_robot.urdf")

        pybullet.resetDebugVisualizerCamera(cameraDistance=2,
                                            cameraYaw=30,
                                            cameraPitch=-30,
                                            cameraTargetPosition=[0, 0, 0])

    def __del__(self):
        pybullet.disconnect(self.client)
#        pybullet.disconnect(self.gui)

    def set_joints(self, joint_positions):
        for i, q in zip(range(1, 7), joint_positions):
            pybullet.resetJointState(self.robot, i, q)
