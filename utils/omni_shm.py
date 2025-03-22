# -----------------------------------------------------------
# File        : omni_shm.py
# Author      : DaeKwan (kodaekwan@dongguk.edu)
# Created     : 2025-03-22
# Description : Shared memory structure and access logic for Phantom Omni
# 
# Copyright (c) 2025 DaeKwan
# License     : MIT License
# -----------------------------------------------------------
#! pip install sysv-ipc
import ctypes
import sysv_ipc

class shmVector3Dd_t(ctypes.Structure):
    _fields_ = [('x', ctypes.c_double),
                ('y', ctypes.c_double),
                ('z', ctypes.c_double),
                ]
    
class SHMOmniState_t(ctypes.Structure):
	_fields_ =  [('position',shmVector3Dd_t),
	('velocity',shmVector3Dd_t),
	('inp_vel1',shmVector3Dd_t),
	('inp_vel2',shmVector3Dd_t),
	('inp_vel3',shmVector3Dd_t),
	('out_vel1',shmVector3Dd_t),
	('out_vel2',shmVector3Dd_t),
	('out_vel3',shmVector3Dd_t),
	('pos_hist1',shmVector3Dd_t),
	('pos_hist2',shmVector3Dd_t),
	('rot',shmVector3Dd_t),
	('joints',shmVector3Dd_t),
	('force',shmVector3Dd_t),
	('thetas',ctypes.c_float*7),
    ('buttons',ctypes.c_int*2),
    ('buttons_prev',ctypes.c_int*2),
    ('lock',ctypes.c_bool),
    ('lock_pos',shmVector3Dd_t),]

class SHMJointState_t(ctypes.Structure):
	_fields_ =  [   ('stamp', ctypes.c_uint64),
                    ('waist', ctypes.c_double),
                    ('shoulder', ctypes.c_double),
                    ('elbow', ctypes.c_double),
                    ('wrist1', ctypes.c_double),
                    ('wrist2', ctypes.c_double),
                    ('wrist3', ctypes.c_double),]

class SHMPhantomButtonEvent_t(ctypes.Structure):
    _fields_ =  [  ('grey_button',ctypes.c_int),
                 ('white_button',ctypes.c_int),]

class OmniFeedback_t(ctypes.Structure):
    _fields_ =  [  ('force',shmVector3Dd_t),
                    ('position',shmVector3Dd_t),]
    

class SHMMessageRead_t(ctypes.Structure):
    _fields_ =  [('omnistate',SHMOmniState_t),
                 ('jointstate',SHMJointState_t),
                 ('buttonevent',SHMPhantomButtonEvent_t),
    ];

class SHMMessageWrite_t(ctypes.Structure):
    _fields_ =  [('omnifeedback',OmniFeedback_t),
                 ];

class SHMMessage_t(ctypes.Structure):
    _fields_ =  [   ('write',SHMMessageWrite_t),
                    ('read',SHMMessageRead_t),
                 ];

class SHMManager: 
    is_opend = False;
    memory_structure = None;
    write_memory_structure = None;
    read_memory_structure = None;

    def __init__( self, key ):
        try:
            self.memory = sysv_ipc.SharedMemory(key);
            self.is_opend = True;
        except:
            print("SHMManager problem init ")
            pass;
        if(self.is_opend):
            self.memory_structure = SHMMessage_t();
            self.write_memory_structure = SHMMessageWrite_t();
            self.read_memory_structure = SHMMessageRead_t();
            self.write_size = ctypes.sizeof(self.memory_structure.write);
            self.read_size = ctypes.sizeof(self.memory_structure.read);

    def update(self):
        if(self.is_opend):
            if(not self.memory.attached):
                return;
        
            #write process
            self.memory.write(self.write_memory_structure,0);

            #read process
            memory_value = self.memory.read(self.read_size,self.write_size);
            ctypes.memmove(ctypes.byref(self.read_memory_structure), memory_value, ctypes.sizeof(SHMMessageRead_t));

    def getValue(self):
        return self.read_memory_structure;
    
    def setValue(self, data:SHMMessageWrite_t):
        if(self.is_opend):
            self.write_memory_structure = data;       
         
    def isOpened(self):
        return self.is_opend;

    def close(self):
        if(self.is_opend):
            self.memory.detach();


