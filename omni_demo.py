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

if __name__ =="__main__":
    shm_key = 777;
    omni_shm = SHMManager(shm_key);
    
    #force feedback
    m_write = SHMMessageWrite_t();

    theta_list = [];

    for _ in range(1000):#10s 
        omni_shm.update();
    
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
