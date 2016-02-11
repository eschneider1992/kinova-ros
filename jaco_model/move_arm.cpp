#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <reflex_msgs/Hand.h>
#include "./move_arm.h"

using namespace std;


ros::Publisher joint_pub;
sensor_msgs::JointState joint_state;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rhr_hand_visualizer");
  ros::NodeHandle n;
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  // Joint Publisher
  // 'finger[1]/flex_joint_from_1_to_2', 'finger[1]/flex_joint_from_2_to_3', 'finger[1]/flex_joint_from_3_to_4', 'finger[1]/flex_joint_from_4_to_5', 'finger[1]/flex_joint_from_5_to_6', 'finger[1]/flex_joint_from_6_to_7', 'finger[1]/flex_joint_from_7_to_8', 'finger[1]/flex_joint_from_8_to_9', 'finger[1]/flex_joint_from_prox_to_1', 'finger[1]/flex_joint_from_9_to_dist', 'finger[2]/flex_joint_from_1_to_2', 'finger[2]/flex_joint_from_2_to_3', 'finger[2]/flex_joint_from_3_to_4', 'finger[2]/flex_joint_from_4_to_5', 'finger[2]/flex_joint_from_5_to_6', 'finger[2]/flex_joint_from_6_to_7', 'finger[2]/flex_joint_from_7_to_8', 'finger[2]/flex_joint_from_8_to_9', 'finger[2]/flex_joint_from_prox_to_1', 'finger[2]/flex_joint_from_9_to_dist', 'finger[3]/flex_joint_from_1_to_2', 'finger[3]/flex_joint_from_2_to_3', 'finger[3]/flex_joint_from_3_to_4', 'finger[3]/flex_joint_from_4_to_5', 'finger[3]/flex_joint_from_5_to_6', 'finger[3]/flex_joint_from_6_to_7', 'finger[3]/flex_joint_from_7_to_8', 'finger[3]/flex_joint_from_8_to_9', 'finger[3]/flex_joint_from_prox_to_1', 'finger[3]/flex_joint_from_9_to_dist', 'preshape_1', 'proximal_joint_1', 'distal_joint_1', 'preshape_2', 'proximal_joint_2', 'distal_joint_2', 'proximal_joint_3', 'distal_joint_3', 'jaco_joint_1', 'jaco_joint_2', 'jaco_joint_3', 'jaco_joint_4', 'jaco_joint_5', 'jaco_joint_6'

  // Handmade
  // 'proximal_joint_1', 'proximal_joint_2', 'proximal_joint_3', 'preshape_1', 'preshape_2', 'jaco_link_1', 'jaco_link_2', 'jaco_link_3', 'jaco_link_4', 'jaco_link_5', 'jaco_link_6', 'finger[1]/flex_joint_from_prox_to_1', 'finger[1]/flex_joint_from_1_to_2', 'finger[1]/flex_joint_from_2_to_3', 'finger[1]/flex_joint_from_3_to_4', 'finger[1]/flex_joint_from_4_to_5', 'finger[1]/flex_joint_from_5_to_6', 'finger[1]/flex_joint_from_6_to_7', 'finger[1]/flex_joint_from_7_to_8', 'finger[1]/flex_joint_from_8_to_9', 'finger[1]/flex_joint_from_9_to_dist', 'finger[2]/flex_joint_from_prox_to_1', 'finger[2]/flex_joint_from_1_to_2', 'finger[2]/flex_joint_from_2_to_3', 'finger[2]/flex_joint_from_3_to_4', 'finger[2]/flex_joint_from_4_to_5', 'finger[2]/flex_joint_from_5_to_6', 'finger[2]/flex_joint_from_6_to_7', 'finger[2]/flex_joint_from_7_to_8', 'finger[2]/flex_joint_from_8_to_9', 'finger[2]/flex_joint_from_9_to_dist', 'finger[3]/flex_joint_from_prox_to_1', 'finger[3]/flex_joint_from_1_to_2', 'finger[3]/flex_joint_from_2_to_3', 'finger[3]/flex_joint_from_3_to_4', 'finger[3]/flex_joint_from_4_to_5', 'finger[3]/flex_joint_from_5_to_6', 'finger[3]/flex_joint_from_6_to_7', 'finger[3]/flex_joint_from_7_to_8', 'finger[3]/flex_joint_from_8_to_9', 'finger[3]/flex_joint_from_9_to_dist'

  ROS_INFO("Number to resize: %d", NUM_FIXED_STEPS + 3 * (NUM_FLEX_STEPS + 1));
  joint_state.name.resize(NUM_FIXED_STEPS + 3 * (NUM_FLEX_STEPS + 1));
  joint_state.position.resize(NUM_FIXED_STEPS + 3 * (NUM_FLEX_STEPS + 1));
  joint_state.name[0] ="proximal_joint_1";
  joint_state.name[1] ="proximal_joint_2";
  joint_state.name[2] ="proximal_joint_3";
  joint_state.name[3] ="preshape_1";
  joint_state.name[4] ="preshape_2";
  joint_state.name[5] ="distal_joint_1";
  joint_state.name[6] ="distal_joint_2";
  joint_state.name[7] ="distal_joint_3";
  joint_state.name[8] ="jaco_joint_1";
  joint_state.name[9] ="jaco_joint_2";
  joint_state.name[10] ="jaco_joint_3";
  joint_state.name[11] ="jaco_joint_4";
  joint_state.name[12] ="jaco_joint_5";
  joint_state.name[13] ="jaco_joint_6";


  char buffer[50];
  int index = NUM_FIXED_STEPS;
  for (int finger = 1; finger<4; finger++)
  {
    for (int i=1; i<(NUM_FLEX_STEPS+2); i++)
    {
      if (i == 1)
        sprintf(buffer, "finger[%d]/flex_joint_from_prox_to_1", finger);
      else if (i == (NUM_FLEX_STEPS+1))
        sprintf(buffer, "finger[%d]/flex_joint_from_%d_to_dist", finger, NUM_FLEX_STEPS);
      else
        sprintf(buffer, "finger[%d]/flex_joint_from_%d_to_%d", finger, i-1, i);
      joint_state.name[index] = buffer;
      index++;
    }
  }

  ros::Publisher pub = n.advertise<reflex_msgs::Hand>("/reflex_sf/hand_state", 10);
  ros::Subscriber sub = n.subscribe("/reflex_sf/hand_state", 10, publish_finger_to_rviz);
  
  // Zero the hand and make it appear open. The sleeps are to let RVIZ start
  ros::Duration(2.0).sleep();
  reflex_msgs::Hand base_hand_state;
  for (int i=0; i<10; i++) {
    pub.publish(base_hand_state);
    ros::Duration(0.5).sleep();
  }

  ros::spin();
  return 0;
}


void publish_finger_to_rviz(const reflex_msgs::HandConstPtr& hand) {
  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0] = hand->finger[0].proximal;
  joint_state.position[1] = hand->finger[1].proximal;
  joint_state.position[2] = hand->finger[2].proximal;
  joint_state.position[3] = hand->motor[3].joint_angle;
  joint_state.position[4] = -hand->motor[3].joint_angle;

  // joint_state.position[5] = hand->motor[0].joint_angle;
  // joint_state.position[6] = hand->motor[1].joint_angle;
  // joint_state.position[7] = hand->motor[2].joint_angle;

  joint_state.position[8]  = -hand->motor[3].joint_angle;
  joint_state.position[9]  = -hand->motor[3].joint_angle;
  joint_state.position[10] = -hand->motor[3].joint_angle;
  joint_state.position[11] = -hand->motor[3].joint_angle;
  joint_state.position[12] = -hand->motor[3].joint_angle;
  joint_state.position[13] = -hand->motor[3].joint_angle;

  int index = NUM_FIXED_STEPS;
  for (int finger = 0; finger<3; finger++)
  {
    for (int i=0; i<(NUM_FLEX_STEPS+1); i++)
    {
      joint_state.position[index] = hand->finger[finger].distal_approx/((float) (NUM_FLEX_STEPS+1));
      index++;
    }
  }
  joint_pub.publish(joint_state);
}
