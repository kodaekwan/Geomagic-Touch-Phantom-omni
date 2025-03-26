/*
 * File        : main.cpp
 * Author      : DaeKwan (kodaekwan@dongguk.edu)
 * Created     : 2025-03-22
 * Description : Standalone C++ control loop for Phantom Omni (no ROS)
 * 
 * Copyright (c) 2025 DaeKwan
 * License     : MIT License
 * Reference   : https://github.com/danepowell/phantom_omni (Original Author: danepowell)
 */
#include "main.hpp"

int calibrationStyle = 0;

static std::atomic<bool> running(true); // flag for loop control

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    running = false;
}

HDCallbackCode HDCALLBACK omni_state_callback(void* pUserData){
	OmniState_t* omni_state = static_cast<OmniState_t*>(pUserData);

	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
	  	printf("Updating calibration...\n");
	    hdUpdateCalibration(calibrationStyle);
	}

	hdBeginFrame(hdGetCurrentDevice());
	//Get angles, set forces
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);
	hdGetDoublev(HD_CURRENT_POSITION, omni_state->position);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);


	hdGetDoublev(HD_CURRENT_TRANSFORM,omni_state->transform);


	hduVector3Dd vel_buff(0, 0, 0);

	vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
				+ omni_state->pos_hist2) / 0.002;  //mm/s, 2nd order backward dif

	omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
				+ .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
				- (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
				- 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz

	omni_state->pos_hist2 	= omni_state->pos_hist1;
	omni_state->pos_hist1 	= omni_state->position;
	omni_state->inp_vel3 	= omni_state->inp_vel2;
	
	omni_state->inp_vel2 	= omni_state->inp_vel1;
	omni_state->inp_vel1 	= vel_buff;
	omni_state->out_vel3 	= omni_state->out_vel2;
	omni_state->out_vel2 	= omni_state->out_vel1;
	omni_state->out_vel1 	= omni_state->velocity;

	if (omni_state->lock == true) {
		omni_state->force = 0.04 * (omni_state->lock_pos - omni_state->position)
				- 0.001 * omni_state->velocity;
	}

	hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);

	//Get buttons
	int nButtons = 0;

	hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
	
	omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
	omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

	hdEndFrame(hdGetCurrentDevice());

	HDErrorInfo error;

	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Error during main scheduler callback");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}

	float t[7] = { 0., omni_state->joints[0], omni_state->joints[1],
			omni_state->joints[2] - omni_state->joints[1], omni_state->rot[0],
			omni_state->rot[1], omni_state->rot[2] };
	
	
	for(int i = 0; i < 7; i++){
		omni_state->thetas[i] = t[i];
	}

	
	return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration() {
	int supportedCalibrationStyles;
	HDErrorInfo error;

	hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
	if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
		calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
		printf("HD_CALIBRATION_ENCODER_RESE..\n");
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
		calibrationStyle = HD_CALIBRATION_INKWELL;
		printf("HD_CALIBRATION_INKWELL..\n");
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
		calibrationStyle = HD_CALIBRATION_AUTO;
		printf("HD_CALIBRATION_AUTO..\n");
	}
	if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
	  do {
		hdUpdateCalibration(calibrationStyle);
		printf("Calibrating.. (put stylus in well)\n");
		if (HD_DEVICE_ERROR(error = hdGetError())) {
			hduPrintError(stderr, &error, "Reset encoders reset failed.");
			break;
		}
		} while (hdCheckCalibration() != HD_CALIBRATION_OK);
		printf("Calibration complete.\n");
	}
	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
        printf("Please place the device into the inkwell for calibration.\n");
	}
}


void force_callback(OmniState_t* state, OmniFeedback_t* omnifeed) {
	////////////////////Some people might not like this extra damping, but it
	////////////////////helps to stabilize the overall force feedback. It isn't
	////////////////////like we are getting direct impedance matching from the
	////////////////////omni anyway
	state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
	state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
	state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];

	state->lock_pos[0] = omnifeed->position.x;
	state->lock_pos[1] = omnifeed->position.y;
	state->lock_pos[2] = omnifeed->position.z;
	//printf("omnifeed->force.z : %f\n",omnifeed->force.z);
}


void publish_omni_state(OmniState_t* state, SHMJointState_t* joint_state, SHMPhantomButtonEvent_t* button_event) {
	joint_state->stamp = Time::now_ms();
	joint_state->waist= -state->thetas[1];
	joint_state->shoulder = state->thetas[2];
	joint_state->elbow = state->thetas[3];
	joint_state->wrist1 = -state->thetas[4] + M_PI;
	joint_state->wrist2 = -state->thetas[5] - 3*M_PI/4;
	joint_state->wrist3 = -state->thetas[6] - M_PI;

	if ((state->buttons[0] != state->buttons_prev[0])
			or (state->buttons[1] != state->buttons_prev[1])) {

		if ((state->buttons[0] == state->buttons[1])
				and (state->buttons[0] == 1)) {
			state->lock = !(state->lock);
		}
		button_event->grey_button = state->buttons[0];
		button_event->white_button = state->buttons[1];
		state->buttons_prev[0] = state->buttons[0];
		state->buttons_prev[1] = state->buttons[1];
	}
}


int main(int argc, char** argv){
	int shm = 777;
	double hz = 1000.0;

	std::map<std::string, std::string> args;
	for (int i = 1; i < argc; ++i) {
		std::string token(argv[i]);
		auto eq = token.find('=');
		if (eq != std::string::npos) {
			std::string key = token.substr(0, eq);
			std::string val = token.substr(eq + 1);
			args[key] = val;
		}
	}
	if (args.find("shm") != args.end()) {
		shm = std::stoi(args["shm"]);
	}
	if (args.find("hz") != args.end()) {
		hz = std::stod(args["hz"]);  // 🔥 double로 파싱
	}
	std::cout << "SHM Key: " << shm << std::endl;
    std::cout << "Rate Hz: " << hz << std::endl;


	std::signal(SIGINT, signalHandler);
    std::cout << "Press Ctrl+C to stop the loop.\n";

    ////////////////////////////////////////////////////////////////
	// Init Phantom
	////////////////////////////////////////////////////////////////
	HDErrorInfo error;
	HHD hHD;
	hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		//hduPrintError(stderr, &error, "Failed to initialize haptic device");
		printf("Failed to initialize haptic device\n");
		return -1;
	}

	printf("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
	hdEnable(HD_FORCE_OUTPUT);
	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		printf("Failed to start the scheduler\n");
		return -1;
	}
	HHD_Auto_Calibration();

    ////////////////////////////////////////////////////////////////
	// Init Shared Memory
	////////////////////////////////////////////////////////////////
	SHMMessageRead_t SHMmessageR;
	SHMMessageWrite_t SHMmessageW;
	memset(&SHMmessageR,0, sizeof(SHMmessageR));
	memset(&SHMmessageW,0, sizeof(SHMmessageW));
	SHM_Manager shm_manager;
	shm_manager.open(shm);

    ////////////////////////////////////////////////////////////////
	// Sync Event
	////////////////////////////////////////////////////////////////
    OmniState_t state;
    hdScheduleAsynchronous(omni_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);

    ////////////////////////////////////////////////////////////////
	// Loop 
	////////////////////////////////////////////////////////////////
	Rate loop_rate(hz);//1kHz

    while(running)
    {
		if(shm_manager.isOpened()){
			
			publish_omni_state(&state, &(SHMmessageR.jointstate), &(SHMmessageR.buttonevent));
			memcpy(&SHMmessageR.omnistate,&state,sizeof(SHMOmniState_t));
			shm_manager.copyToSharedMemroy(SHMmessageR);

			shm_manager.copyFromSharedMemroy(SHMmessageW);
			force_callback(&state,&(SHMmessageW.omnifeedback));
		}
		
        /* code */
		loop_rate.sleep();  // sleep for 1ms
    }


    printf("Ending Session....\n");
	hdStopScheduler();
	hdDisableDevice(hHD);

    if(shm_manager.isOpened()){
		shm_manager.close();
		printf("Release ShardMemory\n");
	}

}