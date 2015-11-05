#include <jaco_interaction/jaco_interactive_manipulation.h>

using namespace std;

JacoInteractiveManipulation::JacoInteractiveManipulation() :
    acGripper("jaco_arm/manipulation/gripper", true), acLift("jaco_arm/manipulation/lift", true), acHome(
        "jaco_arm/home_arm", true)
{
  loadParameters(n);

  joints.resize(6);

  //messages
  cartesianCmd = n.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd", 1);
  jointStateSubscriber = n.subscribe("jaco_arm/joint_states", 1, &JacoInteractiveManipulation::updateJoints, this);

  //services
  eraseTrajectoriesClient = n.serviceClient<std_srvs::Empty>("jaco_arm/erase_trajectories");
  jacoFkClient = n.serviceClient<wpi_jaco_msgs::JacoFK>("jaco_arm/kinematics/fk");
  qeClient = n.serviceClient<wpi_jaco_msgs::QuaternionToEuler>("jaco_conversions/quaternion_to_euler");

  //actionlib
  ROS_INFO("Waiting for grasp, pickup, and home arm action servers...");
  acGripper.waitForServer();
  acLift.waitForServer();
  acHome.waitForServer();
  ROS_INFO("Finished waiting for action servers");

  lockPose = false;

  imServer.reset(
      new interactive_markers::InteractiveMarkerServer("jaco_interactive_manipulation", "jaco_markers", false));

  ros::Duration(0.1).sleep();

  makeHandMarker();

  imServer->applyChanges();
}

void JacoInteractiveManipulation::updateJoints(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (unsigned int i = 0; i < 6; i++)
  {
    joints.at(i) = msg->position.at(i);
  }
}

void JacoInteractiveManipulation::makeHandMarker()
{
  visualization_msgs::InteractiveMarker iMarker;
  if (arm_name_ == "jaco2")
    iMarker.header.frame_id = "jaco_base_link";
  else
    iMarker.header.frame_id = "jaco_link_base";

  //initialize position to the jaco arm's current position
  wpi_jaco_msgs::JacoFK fkSrv;
  for (unsigned int i = 0; i < 6; i++)
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
    iMarker.pose.orientation.w = 1.0;
  }
  iMarker.scale = .2;

  iMarker.name = "jaco_hand_marker";
  iMarker.description = "JACO Hand Control";

  //make a sphere control to represent the end effector position
  visualization_msgs::Marker sphereMarker;
  sphereMarker.type = visualization_msgs::Marker::SPHERE;
  sphereMarker.scale.x = iMarker.scale * 1;
  sphereMarker.scale.y = iMarker.scale * 1;
  sphereMarker.scale.z = iMarker.scale * 1;
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
  interactive_markers::MenuHandler::EntryHandle fingersSubMenuHandle = menuHandler.insert("Gripper");
  menuHandler.insert(fingersSubMenuHandle, "Close",
                     boost::bind(&JacoInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert(fingersSubMenuHandle, "Open",
                     boost::bind(&JacoInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert("Pickup", boost::bind(&JacoInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert("Home", boost::bind(&JacoInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert("Retract", boost::bind(&JacoInteractiveManipulation::processHandMarkerFeedback, this, _1));

  visualization_msgs::InteractiveMarkerControl menuControl;
  menuControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  menuControl.name = "jaco_hand_menu";
  iMarker.controls.push_back(menuControl);

  imServer->insert(iMarker);
  imServer->setCallback(iMarker.name, boost::bind(&JacoInteractiveManipulation::processHandMarkerFeedback, this, _1));

  menuHandler.apply(*imServer, iMarker.name);
}

void JacoInteractiveManipulation::processHandMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
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
          rail_manipulation_msgs::GripperGoal gripperGoal;
          gripperGoal.close = true;
          acGripper.sendGoal(gripperGoal);
        }
        else if (feedback->menu_entry_id == 3)	//release requested
        {
          rail_manipulation_msgs::GripperGoal gripperGoal;
          gripperGoal.close = false;
          acGripper.sendGoal(gripperGoal);
        }
        else if (feedback->menu_entry_id == 4)	//pickup requested
        {
          rail_manipulation_msgs::LiftGoal liftGoal;
          acLift.sendGoal(liftGoal);
        }
        else if (feedback->menu_entry_id == 5)  //home requested
        {
          acGripper.cancelAllGoals();
          acLift.cancelAllGoals();
          wpi_jaco_msgs::HomeArmGoal homeGoal;
          homeGoal.retract = false;
          acHome.sendGoal(homeGoal);
          acHome.waitForResult(ros::Duration(10.0));
        }
        else if (feedback->menu_entry_id == 6)
        {
          acGripper.cancelAllGoals();
          acLift.cancelAllGoals();
          wpi_jaco_msgs::HomeArmGoal homeGoal;
          homeGoal.retract = true;
          homeGoal.retractPosition.position = true;
          homeGoal.retractPosition.armCommand = true;
          homeGoal.retractPosition.fingerCommand = false;
          homeGoal.retractPosition.repeat = false;
          homeGoal.retractPosition.joints.resize(6);
          homeGoal.retractPosition.joints[0] = -2.57;
          homeGoal.retractPosition.joints[1] = 1.39;
          homeGoal.retractPosition.joints[2] = .527;
          homeGoal.retractPosition.joints[3] = -.084;
          homeGoal.retractPosition.joints[4] = .515;
          homeGoal.retractPosition.joints[5] = -1.745;
          acHome.sendGoal(homeGoal);
          acHome.waitForResult(ros::Duration(15.0));
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
          acGripper.cancelAllGoals();
          acLift.cancelAllGoals();

          //convert pose for compatibility with JACO API
          wpi_jaco_msgs::QuaternionToEuler qeSrv;
          qeSrv.request.orientation = feedback->pose.orientation;
          if (qeClient.call(qeSrv))
          {
            wpi_jaco_msgs::CartesianCommand cmd;
            cmd.position = true;
            cmd.armCommand = true;
            cmd.fingerCommand = false;
            cmd.repeat = false;
            cmd.arm.linear.x = feedback->pose.position.x;
            cmd.arm.linear.y = feedback->pose.position.y;
            cmd.arm.linear.z = feedback->pose.position.z;
            cmd.arm.angular.x = qeSrv.response.roll;
            cmd.arm.angular.y = qeSrv.response.pitch;
            cmd.arm.angular.z = qeSrv.response.yaw;

            cartesianCmd.publish(cmd);
          }
          else
            ROS_INFO("Quaternion to Euler conversion service failed, could not send pose update");
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
  wpi_jaco_msgs::CartesianCommand cmd;
  cmd.position = false;
  cmd.armCommand = true;
  cmd.fingerCommand = false;
  cmd.repeat = true;
  cmd.arm.linear.x = 0.0;
  cmd.arm.linear.y = 0.0;
  cmd.arm.linear.z = 0.0;
  cmd.arm.angular.x = 0.0;
  cmd.arm.angular.y = 0.0;
  cmd.arm.angular.z = 0.0;
  cartesianCmd.publish(cmd);

  std_srvs::Empty srv;
  if (!eraseTrajectoriesClient.call(srv))
  {
    ROS_INFO("Could not call erase trajectories service...");
  }
}

void JacoInteractiveManipulation::updateMarkerPosition()
{
  wpi_jaco_msgs::JacoFK fkSrv;
  for (unsigned int i = 0; i < 6; i++)
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

bool JacoInteractiveManipulation::loadParameters(const ros::NodeHandle n)
{
  n.param("wpi_jaco/arm_name",                arm_name_,              std::string("jaco"));

  //! @todo MdL [IMPR]: Return is values are all correctly loaded.
  return true;
}

int main(int argc, char **argv)
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

