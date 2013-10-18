/*
 * YouBot.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: matthias
 */

#include "vrep/VRepJointInterface.h"


extern "C" {
    #include "extApi.h"
}
#include "v_repConst.h"


VRepJointInterface::VRepJointInterface(const char* connection_ip, int connection_port, const char* joint_names[]) {

	clientID=simxStart(connection_ip,connection_port,true,true,2000,5);
	int handle = 0;
	float dummy;

	for (int i=0; joint_names[i] != 0; i++) {
		simxGetObjectHandle(clientID, joint_names[i], &handle, simx_opmode_oneshot_wait);

		//start streaming of joint positions
		simxGetJointPosition(clientID, handle, &dummy, simx_opmode_streaming);

		handles.push_back(handle);
	}

}

VRepJointInterface::~VRepJointInterface() {
	simxFinish(clientID);
}

void VRepJointInterface::setJointPosition(int index, double pos) {

	if (simxGetConnectionId(clientID) != -1) {
		//set joint to position mode
		simxSetObjectIntParameter(clientID, handles[index], 2000, 1, simx_opmode_oneshot);
		simxSetObjectIntParameter(clientID, handles[index], 2001, 1, simx_opmode_oneshot);

		simxSetJointTargetPosition(clientID, handles[index], pos, simx_opmode_oneshot);
	}
}

void VRepJointInterface::setJointPosition(int index[], double positions[], int size) {

	for (int i=0; i < size; i++) {
		setJointPosition(index[i], positions[i]);
	}
}


void VRepJointInterface::setJointVelocity(int index, double vel) {

	if (simxGetConnectionId(clientID) != -1) {
		//set joint to velocity mode
		simxSetObjectIntParameter(clientID, handles[index], 2000, 1, simx_opmode_oneshot);
		simxSetObjectIntParameter(clientID, handles[index], 2001, 0, simx_opmode_oneshot);
		simxSetObjectIntParameter(clientID, handles[index], 1000, 0, simx_opmode_oneshot);

		simxSetJointTargetVelocity(clientID, handles[index], vel, simx_opmode_oneshot);
	}
}

void VRepJointInterface::setJointVelocity(int index[], double velocities[], int size) {

	for (int i=0; i < size; i++) {
		setJointVelocity(index[i], velocities[i]);
	}
}


double VRepJointInterface::getJointPosition(int index) {

	float position;

	if (simxGetConnectionId(clientID) != -1) {
		simxGetJointPosition(clientID, handles[index], &position, simx_opmode_streaming);
	}

	return position;
}


double VRepJointInterface::getJointVelocity(int index) {
	return 0;
}
