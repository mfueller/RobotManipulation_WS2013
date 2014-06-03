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

#include <math.h>
#include "vrep/VRepJointInterface.h"

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"


using namespace youbot;

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

double d2r(double v) {
	return v / 180 * M_PI;
}


int main() {


        //connect with youBot
	YouBotBase youBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
	youBotBase.doJointCommutation();

	YouBotManipulator youBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
        youBotManipulator.doJointCommutation();
        youBotManipulator.calibrateManipulator();



	// connect with V-rep
	VRepJointInterface* jointInterface = new VRepJointInterface(connection_ip, connection_port, joint_names);
	int index[5] = {0, 1, 2, 3, 4};



	double positions[5] = {
			d2r(0) + d2r(169),
			d2r(0) + d2r(65),
			d2r(-90) - d2r(151),
			d2r(0) + d2r(102.5),
			d2r(0) + d2r(165)
	};


    JointAngleSetpoint desiredJointAngle;



    for (int i=0; i<5;i++) {
    	
	//directly control the youBot arm
        youBotManipulator.getArmJoint(i).setData(positions[i] * radian);

	// and the simulated one
	jointInterface->setJointPosition(index[i], positions[i]);

    }






    // calculate the FK using KDL

    KDL::Chain chain;

    double offset[5] = {
			d2r(-169),
			d2r(-65),
			d2r(151),
			d2r(-102.5),
			d2r(-165)
    };


    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.0, M_PI, 0.147, 0)));

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.033,  +M_PI_2,  0.000, offset[0] + M_PI        )));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.155,  0,    0.000, offset[1] - M_PI_2)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.135,  0,    0.000, offset[2]         )));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.000,  0,       0.000, offset[3]         )));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.000, -M_PI_2,  0.000, offset[4] + M_PI_2)));

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.00, 0, -0.218, 0)));



    KDL::ChainFkSolverPos_recursive fksolver(chain);
    KDL::JntArray q(chain.getNrOfJoints());





    // print the FK
    KDL::Frame result;

    for (int i=0; i<8;i++) {
    	std::cout << "=========================================" << std::endl;
    	std::cout << i<< std::endl;

    	fksolver.JntToCart(q, result,i);

    	std::cout << result.p << std::endl;
    	std::cout << result.M << std::endl;
    }
    
    std::cout << "=========================================" << std::endl;
    std::cout << q.data << std::endl;
    std::cout << "=========================================" << std::endl;
    

    sleep(2);


    return 0;
}




//	double positions[5] = {
			//2.9496, 1.13446, -2.54818, 1.78896, 2.93075 //candle
			//2.94958, 0.01564, -2.59489, 2.38586, 2.93068 // out of view
			//3.02221, 2.48996, -1.53309, 1.17502, 2.92980 // pre grasping stangding
			//2.93836, 2.020597, -1.88253, 3.36243, 3.01283 // grasp standing
			//2.5061, 0.0935881, -2.60509, 1.42038, 2.93033 // tower_right
			//2.71339, 0.156002, -3.15581, 1.04624, 3.09898 //platform_right
			//2.96494, 0.134162, -2.97261, 0.745996, 3.04407
			//2.9496, 1.03446, -2.54818, 1.78896, 2.93075
	//};


