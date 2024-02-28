from pathlib import Path

import numpy as np
from real_robot.utils.visualization import Visualizer, visualizer


def main():
    urdf_path = (
        Path(__file__).resolve().parent / "mycobot_with_gripper_parallel_local.urdf"
    )
    # urdf_path = Path(__file__).resolve().parent / "mycobot_urdf_.urdf"

    vis = Visualizer()
    o3d_vis = vis.o3dvis
    # lower="-0.7" upper="0.15"
    base_pose = np.eye(4)
    base_pose[2, -1] = 1
    o3d_vis.load_urdf(
        urdf_path,
        robot_name="mycobot_280pi_-0.8",
        qpos=np.asarray([0, 0, 0, 0, 0, 0, -0.8]),
        base_pose=base_pose,
    )

    o3d_vis.load_urdf(
        urdf_path,
        robot_name="mycobot_280pi_0.15",
        qpos=np.asarray([0, 0, 0, 0, 0, 0, 0.15]),
    )

    visualizer.pause_render = True  # Pause the Visualizer
    vis.render()  # render once


if __name__ == "__main__":
    main()
