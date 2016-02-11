#ifndef MOVE_ARM_H
#define MOVE_ARM_H

#define NUM_FIXED_STEPS     14
#define NUM_FLEX_STEPS      9

void publish_finger_to_rviz(const reflex_msgs::HandConstPtr& hand);
visualization_msgs::Marker makeContactMarker(bool val, int id);
visualization_msgs::Marker makePressureMarker(float val, int id);
visualization_msgs::Marker makeFingerMarker(int id);

#endif