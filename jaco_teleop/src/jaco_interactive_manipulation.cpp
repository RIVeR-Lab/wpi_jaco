/*
 * jaco_interactive_manipulation.h
 *
 *      Author: David Kent
 */
 
#include <jaco_teleop/jaco_interactive_manipulation.h>

using namespace std;

JacoInteractiveManipulation::JacoInteractiveManipulation() :
	acGrasp("jaco_arm/execute_grasp", true),
	acPickup("jaco_arm/execute_pickup", true)
{
	joints.resize(6);

	//messages
	armPoseCommand = n.advertise<geometry_msgs::Pose>("jaco_arm/position_cmd", 1);
	cartesianVelCommand = n.advertise<geometry_msgs::Twist>("jaco_arm/cmd_vel", 1);
	jointStateSubscriber = n.subscribe("joint_states", 1, &JacoInteractiveManipulation::updateJoints, this);

	//services
	jacoFkClient = n.serviceClient<jaco_ros::JacoFK>("jaco_fk");

	//actionlib
	ROS_INFO("Waiting for grasp and pickup action servers...");
	acGrasp.waitForServer();
	acPickup.waitForServer();
	ROS_INFO("Finsihed waiting for grasp and pickup action servers");
	
	lockPose = false;

	imServer.reset( new interactive_markers::InteractiveMarkerServer("jaco_interactive_manipulation", "jaco_markers", false) );

	ros::Duration(0.1).sleep();
	
	makeHandMarker();
	
	imServer->applyChanges();
}

void JacoInteractiveManipulation::updateJoints(const sensor_msgs::JointState::ConstPtr& msg)
{
	for (unsigned int i = 0; i < 6; i ++)
	{
		joints.at(i) = msg->position.at(i);
	}
}

void JacoInteractiveManipulation::makeHandMarker()
{
	visualization_msgs::InteractiveMarker iMarker;
	iMarker.header.frame_id = "jaco_link_base";

	//initialize position to the jaco arm's current position
    jaco_ros::JacoFK fkSrv;
    for (unsigned int i = 0; i < 6; i ++)
    {
    	fkSrv.request.joints.push_back(joints.at(i));
    }
	if (jacoFkClient.call(fkSrv))
	{
		iMarker.pose = fkSrv.response.handPose.pose;
	}
	else
	{
		iMarker.pose.position.x = 0.0;
		iMarker.pose.position.y = 0.0;
		iMarker.pose.position.z = 0.0;
		iMarker.pose.orientation.x = 0.0;
		iMarker.pose.orientation.y = 0.0;
		iMarker.pose.orientation.z = 0.0;
		iMarker.pose.orientation.w = 0.0;
	}
	iMarker.scale = .2;
	
	iMarker.name = "jaco_hand_marker";
	iMarker.description = "JACO Hand Control";
	
	//make a sphere control to represent the end effector position
	visualization_msgs::Marker sphereMarker;
	sphereMarker.type = visualization_msgs::Marker::SPHERE;
	sphereMarker.scale.x = iMarker.scale*1;
	sphereMarker.scale.y = iMarker.scale*1;
	sphereMarker.scale.z = iMarker.scale*1;
	sphereMarker.color.r = .5;
	sphereMarker.color.g = .5;
	sphereMarker.color.b = .5;
	sphereMarker.color.a = 0.0;
	visualization_msgs::InteractiveMarkerControl sphereControl;
	sphereControl.markers.push_back(sphereMarker);
	sphereControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
	sphereControl.name = "jaco_hand_origin_marker";
	iMarker.controls.push_back(sphereControl);
	
	//add 6-DOF controls
	visualization_msgs::InteractiveMarkerControl control;
	
	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "rotate_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	iMarker.controls.push_back(control);
	control.name = "move_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	iMarker.controls.push_back(control);
	
	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "rotate_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	iMarker.controls.push_back(control);
	control.name = "move_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	iMarker.controls.push_back(control);
	
	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "rotate_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	iMarker.controls.push_back(control);
	control.name = "move_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	iMarker.controls.push_back(control);
	
	//menu
	interactive_markers::MenuHandler::EntryHandle fingersSubMenuHandle = menuHandler.insert("Fingers");
	menuHandler.insert(fingersSubMenuHandle, "Grasp", boost::bind(&JacoInteractiveManipulation::processHandMarkerFeedback, this, _1));
	menuHandler.insert(fingersSubMenuHandle, "Release", boost::bind(&JacoInteractiveManipulation::processHandMarkerFeedback, this, _1));
	menuHandler.insert("Pickup", boost::bind(&JacoInteractiveManipulation::processHandMarkerFeedback, this, _1));
	
	visualization_msgs::InteractiveMarkerControl menuControl;
	menuControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
	menuControl.name = "jaco_hand_menu";
	iMarker.controls.push_back(menuControl);
	
	imServer->insert(iMarker);
	imServer->setCallback(iMarker.name, boost::bind(&JacoInteractiveManipulation::processHandMarkerFeedback, this, _1));
	
	menuHandler.apply(*imServer, iMarker.name);
}

void JacoInteractiveManipulation::processHandMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	switch (feedback->event_type)
	{
	//Send a stop command so that when the marker is released the arm stops moving
	case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		if (feedback->marker_name.compare("jaco_hand_marker") == 0)
		{
			lockPose = true;
			sendStopCommand();
		}
	break;
	
	//Menu actions
	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		if (feedback->marker_name.compare("jaco_hand_marker") == 0)
		{
			if (feedback->menu_entry_id == 2)	//grasp requested
			{
				jaco_ros::ExecuteGraspGoal graspGoal;
				graspGoal.closeGripper = true;
				graspGoal.limitFingerVelocity = false;
				acGrasp.sendGoal(graspGoal);
			}
			else if (feedback->menu_entry_id == 3)	//release requested
			{
				jaco_ros::ExecuteGraspGoal graspGoal;
				graspGoal.closeGripper = false;
				graspGoal.limitFingerVelocity = false;
				acGrasp.sendGoal(graspGoal);
			}
			else if (feedback->menu_entry_id == 4)	//pickup requested
			{
				jaco_ros::ExecutePickupGoal pickupGoal;
				pickupGoal.limitFingerVelocity = false;
				pickupGoal.setLiftVelocity = false;
				acPickup.sendGoal(pickupGoal);
			}
		}
	break;
	
	//Send movement commands to the arm to follow the pose marker
	case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
		if (feedback->marker_name.compare("jaco_hand_marker") == 0
			&& feedback->control_name.compare("jaco_hand_origin_marker") != 0)
		{
			if (!lockPose)
			{
				acGrasp.cancelAllGoals();
				acPickup.cancelAllGoals();
				armPoseCommand.publish(feedback->pose);
			}
		}
	break;
	
	//Mouse down events
	case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
		lockPose = false;
	break;
	
	//As with mouse clicked, send a stop command when the mouse is released on the marker
	case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
		if (feedback->marker_name.compare("jaco_hand_marker") == 0)
		{
			lockPose = true;
			sendStopCommand();
		}
	break;
	}
	
	//Update interactive marker server
	imServer->applyChanges();
}

void JacoInteractiveManipulation::sendStopCommand()
{
	geometry_msgs::Twist stopVel;
	stopVel.linear.x = 0.0;
	stopVel.linear.y = 0.0;
	stopVel.linear.z = 0.0;
	stopVel.angular.x = 0.0;
	stopVel.angular.y = 0.0;
	stopVel.angular.z = 0.0;
	cartesianVelCommand.publish(stopVel);
}

void JacoInteractiveManipulation::updateMarkerPosition()
{
    jaco_ros::JacoFK fkSrv;
	for (unsigned int i = 0; i < 6; i ++)
    {
    	fkSrv.request.joints.push_back(joints.at(i));
    }
    
    if (jacoFkClient.call(fkSrv))
    {
    	imServer->setPose("jaco_hand_marker", fkSrv.response.handPose.pose);
    	imServer->applyChanges();
    }
    else
    {
    	ROS_INFO("Failed to call forward kinematics service");
    }
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "jaco_interactive_manipulation");

	JacoInteractiveManipulation jim;

	ros::Rate loop_rate(30);
	while (ros::ok())
	{
		jim.updateMarkerPosition();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

