/*
 * RobotInterface.h
 *
 *  Created on: Oct 17, 2013
 *      Author: matthias
 */

#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

class RobotInterface {
public:

	virtual void setJointPosition(int index, double pos) = 0;

	virtual void setJointPosition(int index[], double positions[], int size) = 0;

	virtual void setJointVelocity(int index, double vel) = 0;

	virtual void setJointVelocity(int index[], double velocities[], int size) = 0;

	virtual double getJointPosition(int index) = 0;

	virtual double getJointVelocity(int index) = 0;


};

#endif /* ROBOTINTERFACE_H_ */
