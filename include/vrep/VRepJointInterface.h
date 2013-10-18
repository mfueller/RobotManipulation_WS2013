/*
 * YouBot.h
 *
 *  Created on: Oct 17, 2013
 *      Author: matthias
 */

#ifndef VREPJOINTINTERFACE_H_
#define VREPJOINTINTERFACE_H_

#include "JointInterface.h"

#include <vector>

class VRepJointInterface: public JointInterface {
private:

	int clientID;
	std::vector<int> handles;

public:
	VRepJointInterface(const char* connection_ip, int connection_port, const char* joint_names[]);

	virtual ~VRepJointInterface();

	void setJointPosition(int index, double pos);

	void setJointPosition(int index[], double positions[], int size);

	void setJointVelocity(int index, double vel);

	void setJointVelocity(int index[], double velocities[], int size);

	double getJointPosition(int index);

	double getJointVelocity(int index);

};

#endif /* YOUBOT_H_ */
