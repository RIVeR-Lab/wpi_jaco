/*
 * jaco_conversions.h
 *
 * Common conversions for working with ROS and the JACO api
 *
 *      Author: David Kent
 */

#ifndef JACO_CONVERSIONS_H_
#define JACO_CONVERSIONS_H_

#include <ros/ros.h>
#include <jaco_ros/EulerToQuaternion.h>
#include <jaco_ros/QuaternionToEuler.h>

class JacoConversions {

public:
	
	JacoConversions(void);

	/**
	 * Callback for the Euler to Quaternion service
	 * @param req service request
	 * @param res service response
	 * @return true on success
	 */
	bool callEQ(jaco_ros::EulerToQuaternion::Request &req, jaco_ros::EulerToQuaternion::Response &res);
	
	/**
	 * Callback for the Quaternion to Euler service
	 * @param req service request
	 * @param res service response
	 * @return true on success
	 */
	bool callQE(jaco_ros::QuaternionToEuler::Request &req, jaco_ros::QuaternionToEuler::Response &res);

private:
	ros::NodeHandle n;
	ros::ServiceServer eqServer;
	ros::ServiceServer qeServer;
};

#endif
