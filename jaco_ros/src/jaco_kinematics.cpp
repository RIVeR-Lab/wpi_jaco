/*
 * jaco_kinematics.cpp
 *
 *      Author: David Kent
 */
 
#include <jaco_ros/jaco_kinematics.h>

using namespace std;

JacoKinematics::JacoKinematics(void) 
{
	//calculate additional parameters
	float AA = 11.0*PI/72.0;
	float CA = cos(AA);
	float SA = sin(AA);
	float C2A = cos(2*AA);
	float S2A = sin(2*AA);
	float D4B = D3 + SA/S2A*D4;
	float D5B = SA/S2A*D4 + SA/S2A*D5;
	float D6B = SA/S2A*D5 + D6;

	//set up D-H parameters
	ds.resize(6);
	ds[0] = D1;
	ds[1] = 0;
	ds[2] = -E2;
	ds[3] = -D4B;
	ds[4] = -D5B;
	ds[5] = -D6B;

	as.resize(6);
	for (unsigned int i = 0; i < 6; i ++)
	{
		as[i] = 0;
	}
	as[1] = D2;

	alphas.resize(6);
	alphas[0] = PI/2.0;
	alphas[1] = PI;
	alphas[2] = PI/2.0;
	alphas[3] = 2*AA;
	alphas[4] = 2*AA;
	alphas[5] = PI;

	//advertise service
	fkServer = n.advertiseService("jaco_fk", &JacoKinematics::callFK, this);

	//initialize publisher for debugging
	visPublisher = n.advertise<geometry_msgs::PoseStamped>("fk_pose_debug", 1, this);
}

bool JacoKinematics::callFK(jaco_msgs::JacoFK::Request &req, jaco_msgs::JacoFK::Response &res)
{
	if (req.joints.size() < 6)
	{
		ROS_INFO("Not enough joints specified, could not calculate forward kinematics");
		return false;
	}

	res.handPose = calculateFK(req.joints);

	//debugging:
	visPublisher.publish(res.handPose);
	double roll, pitch, yaw;

	//Manual conversion:
	float q1 = res.handPose.pose.orientation.w;
	float q2 = res.handPose.pose.orientation.x;
	float q3 = res.handPose.pose.orientation.y;
	float q4 = res.handPose.pose.orientation.z;
	float m11 = pow(q1, 2) + pow(q2, 2) - pow(q3, 2) - pow(q4, 2);
	float m12 = 2*(q2*q3 - q1*q4);
	float m13 = 2*(q2*q4 + q1*q3);
	float m23 = 2*(q3*q4 - q1*q2);
	float m33 = pow(q1, 2) - pow(q2, 2) - pow(q3, 2) + pow(q4, 2);
	roll = atan2(-m23, m33);
	pitch = atan2(m13, sqrt(1 - pow(m13, 2)));
	yaw = atan2(-m12, m11);

	ROS_INFO("rpy: %f, %f, %f", roll, pitch, yaw);

	return true;
}

geometry_msgs::PoseStamped JacoKinematics::calculateFK(vector<float> joints)
{
	tf::Transform transform;
	tf::Transform tb1, t12, t23, t34, t45, t56, t6e;
	tf::Quaternion rotQuat(0, 0, 0, 0);
	tf::Matrix3x3 rotMat(1, 0, 0, 0, 1, 0, 0, 0, 1);
	tf::Vector3 trans(0, 0, 0);

	//initialize empty transformation
	rotMat.getRotation(rotQuat);	
	transform.setRotation(rotQuat);
	transform.setOrigin(trans);

	//Transform angles for D-H algorithm
	vector<float> thetas;
	thetas.resize(6);
	thetas[0] = -joints[0];
	thetas[1] = joints[1] - PI/2.0;
	thetas[2] = joints[2] + PI/2.0;
	thetas[3] = joints[3];
	thetas[4] = joints[4] - PI;
	thetas[5] = joints[5] + 5.0*PI/9.0;

	//Calculate transformation from base to end effector
	for (unsigned int i = 0; i < joints.size(); i ++)
	{
		transform = transform * generateTransform(thetas[i], ds[i], as[i], alphas[i]);
	}

	//Calculate Hand Pose 
	geometry_msgs::PoseStamped handPose;
	handPose.header.frame_id = "jaco_link_base";
	handPose.pose.position.x = transform.getOrigin().x();
	handPose.pose.position.y = transform.getOrigin().y();
	handPose.pose.position.z = transform.getOrigin().z();
	handPose.pose.orientation.x = transform.getRotation().x();
	handPose.pose.orientation.y = transform.getRotation().y();
	handPose.pose.orientation.z = transform.getRotation().z();
	handPose.pose.orientation.w = transform.getRotation().w();

	return handPose;
}

tf::Transform JacoKinematics::generateTransform(float theta, float d, float a, float alpha)
{
	tf::Transform transform;
	tf::Quaternion rotQuat(0, 0, 0, 0);	//Rotation quaternion
	tf::Matrix3x3 rotMat(0, 0, 0, 0, 0, 0, 0, 0, 0);	//Rotation matrix
	tf::Vector3 trans(0, 0, 0);	//Translation vector

	//calculate rotation matrix
	rotMat.setValue(cos(theta),	-sin(theta)*cos(alpha),	sin(theta)*sin(alpha), 
					sin(theta),	cos(theta)*cos(alpha),	-cos(theta)*sin(alpha), 
					0,			sin(alpha),				cos(alpha)				);

	//calculate translation vector
	trans.setValue(	a*cos(theta),
					a*sin(theta),
					d			);

	//get rotation as a quaternion
	rotMat.getRotation(rotQuat);

	//fill transformation
	transform.setRotation(rotQuat);
	transform.setOrigin(trans);

	return transform;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "jaco_kinematics");

	JacoKinematics jk;

	ros::spin();
}

