"""PyBullet GUI framing for the standalone flight example."""


CAMERA_DISTANCE_M = 5.0
CAMERA_YAW_DEG = 45
CAMERA_PITCH_DEG = -20
CAMERA_TARGET_POSITION_M = (0.0, 0.0, 1.5)


def frame_flight() -> None:
    import pybullet

    # shortcut: standalone demo owns client 0; pass a client ID if supporting shared processes.
    pybullet.resetDebugVisualizerCamera(
        cameraDistance=CAMERA_DISTANCE_M, cameraYaw=CAMERA_YAW_DEG, cameraPitch=CAMERA_PITCH_DEG,
        cameraTargetPosition=CAMERA_TARGET_POSITION_M,
    )
