/*
 * YouBot.h
 *
 *  Created on: Oct 17, 2013
 *      Author: matthias
 */

#ifndef YOUBOT_H_
#define YOUBOT_H_

#include "RobotInterface.h"

#include <vector>

class VRepRobotInterface: public RobotInterface {
private:

	int clientID;
	std::vector<int> handles;

public:
	VRepRobotInterface(const char* connection_ip, int connection_port, const char* joint_names[]);

	virtual ~VRepRobotInterface();

	void setJointPosition(int index, double pos);

	void setJointPosition(int index[], double positions[], int size);

	void setJointVelocity(int index, double vel);

	void setJointVelocity(int index[], double velocities[], int size);

	double getJointPosition(int index);

	double getJointVelocity(int index);

};

#endif /* YOUBOT_H_ */
