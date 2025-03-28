/*
 * File        : main.hpp
 * Author      : DaeKwan (kodaekwan@dongguk.edu)
 * Created     : 2025-03-22
 * Description : Data structures and utility classes for Phantom Omni control
 * 
 * Copyright (c) 2025 DaeKwan
 * License     : MIT License
 */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <iostream>
#include <map>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <sys/shm.h>
#include <csignal> 

typedef struct OmniState_ {
	hduVector3Dd position;  //3x1 vector of position
	hduVector3Dd velocity;  //3x1 vector of velocity
	hduVector3Dd inp_vel1; //3x1 history of velocity used for filtering velocity estimate
	hduVector3Dd inp_vel2;
	hduVector3Dd inp_vel3;
	hduVector3Dd out_vel1;
	hduVector3Dd out_vel2;
	hduVector3Dd out_vel3;
	hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
	hduVector3Dd pos_hist2;
	hduVector3Dd rot;
	hduVector3Dd joints;
	hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
	float thetas[7];
	int buttons[2];
	int buttons_prev[2];
	bool lock;
	hduVector3Dd lock_pos;
	double transform[16];
} OmniState_t;

typedef struct shmVector3Dd_
{
    double x = 0.0;
	double y = 0.0;
	double z = 0.0;
} shmVector3Dd_t;

typedef struct SHMOmniState_ {
	shmVector3Dd_t position;  //3x1 vector of position
	shmVector3Dd_t velocity;  //3x1 vector of velocity
	shmVector3Dd_t inp_vel1; //3x1 history of velocity used for filtering velocity estimate
	shmVector3Dd_t inp_vel2;
	shmVector3Dd_t inp_vel3;
	shmVector3Dd_t out_vel1;
	shmVector3Dd_t out_vel2;
	shmVector3Dd_t out_vel3;
	shmVector3Dd_t pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
	shmVector3Dd_t pos_hist2;
	shmVector3Dd_t rot;
	shmVector3Dd_t joints;
	shmVector3Dd_t force;   //3 element double vector force[0], force[1], force[2]
	float thetas[7];
	int buttons[2];
	int buttons_prev[2];
	bool lock;
	shmVector3Dd_t lock_pos;
	double transform[16];
} SHMOmniState_t;

typedef struct SHMJointState_ {
	uint64_t stamp;
	double waist;
	double shoulder;
	double elbow;
	double wrist1;
	double wrist2;
	double wrist3;
} SHMJointState_t;

typedef struct SHMPhantomButtonEvent_{
	int grey_button;
	int white_button;
} SHMPhantomButtonEvent_t;


typedef struct OmniFeedback_{
	shmVector3Dd_t force;
	shmVector3Dd_t position;
} OmniFeedback_t;

//must be saperate memory space !!!!
typedef struct SHMMessageRead_ {
	SHMOmniState_t omnistate;//read only
	SHMJointState_t jointstate;//read only
	SHMPhantomButtonEvent_t buttonevent;//read only
} SHMMessageRead_t;

typedef struct SHMMessageWrite_ {
	OmniFeedback_t omnifeedback;
} SHMMessageWrite_t;

typedef struct SHMMessage_ {
	SHMMessageWrite_t write;
	SHMMessageRead_t read;
} SHMMessage_t;

class SHM_Manager
{
private:
    int m_shmid;
    key_t m_key;
    void *m_shared_memory;
    bool isopened;
	void *m_shared_memory_read_ptr;

public:
    void open(int key){
        m_key = key;
        isopened = false;
        // Setup shared memory, 11 is the size 
        if ( ( m_shmid = shmget(m_key, sizeof(SHMMessage_t), IPC_CREAT | 0666)) < 0 )
        {
            printf("Error getting shared memory id");
            return;
        }

         // Attached shared memory
        if ( ( m_shared_memory = shmat( m_shmid , NULL , 0 ) ) == (char *)-1)
        {
            printf("Error attaching shared memory id");
            return;
        }

		m_shared_memory_read_ptr = m_shared_memory+sizeof(SHMMessageWrite_t);

        memset(m_shared_memory,0, sizeof(SHMMessage_t));

        isopened = true;
    }

    bool isOpened(){
        return isopened;
    }
     
    void copyToSharedMemroy(SHMMessageRead_t& data)
    {
        // copy data to shared memory
        memcpy( m_shared_memory_read_ptr, &data, sizeof(SHMMessageRead_t));
    }

	void copyFromSharedMemroy(SHMMessageWrite_t& data)
    {
        // copy shared memory to data
		//printf( "shm omnifeed->force.z : %f\n",((SHMMessageWrite_t*)m_shared_memory)->omnifeedback.force.z);
        memcpy( &data, m_shared_memory, sizeof(SHMMessageWrite_t));
    }

    void* get_memory_ptr()
    {
        return m_shared_memory;
    }

    void close()
    {
        shmdt( m_shared_memory );
        shmctl( m_shmid , IPC_RMID, NULL ); 
    }
};



//This code is used as an alternative to ROS::Rate.
class Rate {
	private:
		std::chrono::duration<double> interval_;
		std::chrono::steady_clock::time_point last_time_;
    public:
        explicit Rate(double frequency)
            : interval_(std::chrono::duration<double>(1.0 / frequency)),
              last_time_(std::chrono::steady_clock::now()) {}
    
        void sleep() {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = now - last_time_;
            auto sleep_duration = interval_ - elapsed;
    
            if (sleep_duration > std::chrono::duration<double>(0)) {
                std::this_thread::sleep_for(sleep_duration);
            }
    
            last_time_ = std::chrono::steady_clock::now();
        }
};

// This code is used as an alternative to ROS::Time
class Time {
	public:
		static double now() {
			auto now = std::chrono::system_clock::now();
			auto duration = now.time_since_epoch();
			return std::chrono::duration<double>(duration).count();
		}
	
		static uint64_t now_ms() {
			auto now = std::chrono::system_clock::now();
			auto duration = now.time_since_epoch();
			return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
		}
	};
