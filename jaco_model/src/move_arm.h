#ifndef MOVE_ARM_H
#define MOVE_ARM_H


#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <jaco_model/JacoPlusReflex.h>


#define NUM_FIXED_STEPS     14
#define NUM_FLEX_STEPS      9


void publish_state_to_rviz(const jaco_model::JacoPlusReflexConstPtr& cmd);


#endif