//============================================================================
// Name        : RobotManipulation.cpp
// Author      : Matthias
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
using namespace std;

#include "extApi.h"


#include "vrep/VRepJointInterface.h"

const char* connection_ip = "127.0.0.1";
const int connection_port = 19998;

const char* joint_names[] = {
		"arm_joint_1",
		"arm_joint_2",
		"arm_joint_3",
		"arm_joint_4",
		"arm_joint_5",
		"gripper_finger_joint_l",
		"gripper_finger_joint_l"
};


int main() {

	VRepJointInterface* jointInterface = new VRepJointInterface(connection_ip, connection_port, joint_names);


	//setting a single joint
	jointInterface->setJointPosition(0, 0);
	sleep(1);
	jointInterface->setJointVelocity(0, 0.1);
	sleep(2);


	//setting a group of joints
	int index[3] = {0, 1, 2};
	double positions[3] = {0.1, 0.2, -0.5};

	jointInterface->setJointPosition(index, positions, 3);

	sleep(2);

	return 0;
}
