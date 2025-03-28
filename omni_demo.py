# -----------------------------------------------------------
# File        : omni_demo.py
# Author      : DaeKwan (kodaekwan@dongguk.edu)
# Created     : 2025-03-22
# Description : Python client for interacting with Omni device via shared memory
# 
# Copyright (c) 2025 DaeKwan
# License     : MIT License
# Reference   : https://github.com/danepowell/phantom_omni
# -----------------------------------------------------------

from utils.omni_shm import *
import time
import numpy as np


def rotation_matrix_to_euler_xyz(R):
    """
    Convert a 3x3 rotation matrix to Euler angles (X-Y-Z convention: roll-pitch-yaw).
    
    Parameters:
    - R: 3x3 rotation matrix (numpy array)

    Returns:
    - tuple of Euler angles (roll, pitch, yaw) in radians
    """
    assert R.shape == (3, 3), "Rotation matrix must be 3x3"

    # Pitch (around Y axis)
    if np.abs(R[0, 2]) < 1:
        pitch = np.arcsin(R[0, 2])
        roll = np.arctan2(-R[1, 2], R[2, 2])
        yaw = np.arctan2(-R[0, 1], R[0, 0])
    else:
        # Gimbal lock case
        pitch = np.pi/2 * np.sign(R[0, 2])
        roll = np.arctan2(R[1, 0], R[1, 1])
        yaw = 0

    return roll, pitch, yaw

if __name__ =="__main__":
    shm_key = 777;
    omni_shm = SHMManager(shm_key);
    
    #force feedback
    m_write = SHMMessageWrite_t();

    theta_list = [];

    for _ in range(1000):#10s 
        omni_shm.update();
        TM = np.ndarray((4,4), dtype=np.float64, buffer=omni_shm.getValue().omnistate.transform).T;
        print(TM);
        roll, pitch, yaw = rotation_matrix_to_euler_xyz(TM[:3,:3])
        print("Roll (X):", np.degrees(roll))
        print("Pitch (Y):", np.degrees(pitch))
        print("Yaw (Z):", np.degrees(yaw))
    
        jointstate = omni_shm.getValue().jointstate;
        buttonevent = omni_shm.getValue().buttonevent;
        joint_angle = [jointstate.waist,jointstate.shoulder,jointstate.elbow,jointstate.wrist1,jointstate.wrist2,jointstate.wrist3];
    
        print(f"joint_angle : {joint_angle}");
        print(f"button_event [grey_button : {buttonevent.grey_button}],[white_button : {buttonevent.white_button}]");

        m_write.omnifeedback.force.z = 0.5;
        omni_shm.setValue(m_write);
        time.sleep(0.01);
    
    m_write.omnifeedback.force.z = 0.0;
    omni_shm.setValue(m_write)
    omni_shm.update();

    omni_shm.close();
