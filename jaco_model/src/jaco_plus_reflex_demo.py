#!/usr/bin/env python

import rospy
import numpy as np
from jaco_model.msg import JacoPlusReflex


def main():
    P7 = PathPolynomial7()
    # Let RVIZ node start up
    rospy.sleep(5.0)
    tf = 2.0
    dt = 0.05
    pose0 = JacoPlusReflex(jaco_joint=[0, 1.8, -0.4, 0, 0, 0],
                           preshape_joint=0,
                           proximal_joint=[0, 0, 0],
                           distal_joint=[0, 0, 0])

    # So this is kind of gross, I just hand-picked frames that I liked
    # Poses could be chosen in some automatic fashion
    pose1 = JacoPlusReflex(jaco_joint=[-np.pi/2, 0, np.pi, 0, 0, 0],
                           proximal_joint=[np.pi*0.1]*3,
                           distal_joint=[np.pi*0.2]*3)
    pose2 = JacoPlusReflex(jaco_joint=[0, 0, -np.pi/4, 0, 0, 0],
                           proximal_joint=[np.pi*0.1]*3,
                           distal_joint=[np.pi*0.2]*3)
    pose3 = JacoPlusReflex(jaco_joint=[0, -np.pi/8, -np.pi/3, -np.pi/2, 0, 0],
                           proximal_joint=[np.pi*0.1]*3,
                           distal_joint=[np.pi*0.2]*3)
    pose4 = JacoPlusReflex(jaco_joint=[0, -np.pi/8, -np.pi/3, -np.pi/2, 0, 0],
                           proximal_joint=[np.pi*0.7]*3,
                           distal_joint=[np.pi*0.4]*3)
    pose5 = JacoPlusReflex(jaco_joint=[np.pi/5, -np.pi/4, np.pi/4, np.pi/2, 0, 0],
                           proximal_joint=[np.pi*0.7]*3,
                           distal_joint=[np.pi*0.4]*3)
    pose6 = JacoPlusReflex(jaco_joint=[np.pi/5, -np.pi/4, np.pi/4, np.pi/2, 0, 0],
                           proximal_joint=[np.pi*0.1]*3,
                           distal_joint=[np.pi*0.2]*3)
    pose7 = JacoPlusReflex(jaco_joint=[np.pi/5, -np.pi/4, np.pi/4, np.pi/2, 0, 0],
                           preshape_joint=np.pi/2,
                           proximal_joint=[np.pi*0.1]*3,
                           distal_joint=[np.pi*0.2]*3)
    pose8 = JacoPlusReflex(jaco_joint=[np.pi/5, -np.pi/4, np.pi/4, np.pi/2, 0, 0],
                           preshape_joint=np.pi/2,
                           proximal_joint=[np.pi*0.7, np.pi*0.1, np.pi*0.1],
                           distal_joint=[np.pi*0.4, np.pi*0.2, np.pi*0.2])
    pose9 = JacoPlusReflex(jaco_joint=[np.pi/5, -np.pi/4, np.pi/4, np.pi/2, 0, 0],
                           preshape_joint=np.pi/2,
                           proximal_joint=[np.pi*0.1, np.pi*0.7, np.pi*0.1],
                           distal_joint=[np.pi*0.2, np.pi*0.4, np.pi*0.2])
    pose10 = JacoPlusReflex(jaco_joint=[np.pi/5, -np.pi/4, np.pi/4, np.pi/2, 0, 0],
                            preshape_joint=np.pi/2,
                            proximal_joint=[np.pi*0.1, np.pi*0.1, np.pi*0.7],
                            distal_joint=[np.pi*0.2, np.pi*0.2, np.pi*0.4])

    while not rospy.is_shutdown():
        P7.calculate_and_publish_path(pose1,  pose2,  dt, tf)
        P7.calculate_and_publish_path(pose2,  pose3,  dt, tf)
        P7.calculate_and_publish_path(pose3,  pose4,  dt, tf)
        P7.calculate_and_publish_path(pose4,  pose5,  dt, tf)
        P7.calculate_and_publish_path(pose5,  pose6,  dt, tf)
        P7.calculate_and_publish_path(pose6,  pose7,  dt, tf / 2.0)
        P7.calculate_and_publish_path(pose7,  pose8,  dt, tf / 2.0)
        P7.calculate_and_publish_path(pose8,  pose9,  dt, tf / 2.0)
        P7.calculate_and_publish_path(pose9,  pose10, dt, tf / 2.0)
        P7.calculate_and_publish_path(pose10, pose7,  dt, tf / 2.0)
        rospy.sleep(1)


class PathPolynomial7():
    """ Makes a path from pose to pose that uses a 7-degree polynomial path """
    def __init__(self):
        self.coefficients = [np.arange(8) for i in range(13)]
        self.joint_pub = rospy.Publisher('/jaco_plus_reflex_cmd',
                                         JacoPlusReflex, queue_size=10)

    def calculate_and_publish_path(self, pose1, pose2, dt, tf):
        if not rospy.is_shutdown():
            t = np.arange(0, tf, dt)
            arm_coefficients = self.calculate_pose_coefficients(pose1, pose2, tf)
            arm_angle_path = self.calculate_arm_angles(arm_coefficients, t)
            self.publish_arm_path(arm_angle_path, dt, tf)
            rospy.sleep(0.5)

    def calculate_pose_coefficients(self, pose1, pose2, tf):
        """
        Takes the start and poses of arm motion, and time to complete path, and
        returns 7-poly coefficients for a path
        """
        # Turn the JointAngles variables into numpy arrays
        pose1 = np.array(pose1.jaco_joint + [pose1.preshape_joint] +
                         pose1.proximal_joint + pose1.distal_joint)
        pose2 = np.array(pose2.jaco_joint + [pose2.preshape_joint] +
                         pose2.proximal_joint + pose2.distal_joint)
        # Loop through the arm joints calculate coefficients for each joint
        for i in range(13):
            # Calculates assuming 0 velocity, acceleration, and jerk at endpoints
            self.coefficients[i] = self.calculate_coefficients(pose1[i], 0, 0, 0,
                                                               pose2[i], 0, 0, 0, tf)
        return(self.coefficients)

    def calculate_arm_angles(self, cf, t):
        """ Given coefficients and time, calculate paths for all arm angles """
        angles = [np.arange(len(t)) for i in range(13)]
        for i in range(13):
            angles[i] = self.angle_equation(cf[i], t)
        return(angles)

    def publish_arm_path(self, ang, dt, tf):
        """ Given angles and a time step, publishes the arm positions in time """
        counter = 0.0
        for i in range(len(ang[0])):
            joint = JacoPlusReflex(jaco_joint=[ang[0][i], ang[1][i], ang[2][i],
                                               ang[3][i], ang[4][i], ang[5][i]],
                                   preshape_joint=ang[6][i],
                                   proximal_joint=[ang[7][i], ang[8][i], ang[9][i]],
                                   distal_joint=[ang[10][i], ang[11][i], ang[12][i]])
            self.joint_pub.publish(joint)
            rospy.sleep(dt)
            counter += 1

    def calculate_coefficients(self, q0, Dq0, DDq0, DDDq0, qf, Dqf, DDqf, DDDqf, tf):
        """
        Takes in joint positions and derivaties from initial to final, as well as time
        Inspiration taken from pg 737 of the Theory of Applied Robotics
        """
        qf = self.correct_q0qf_loops(q0, qf)
        RHS = np.matrix([q0, Dq0, DDq0, DDDq0, qf, Dqf, DDqf, DDDqf]).transpose()
        LHS = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0, 0, 0, 0],
                         [0, 0, 2, 0, 0, 0, 0, 0],
                         [0, 0, 0, 6, 0, 0, 0, 0],
                         [1, tf, tf**2, tf**3, tf**4, tf**5, tf**6, tf**7],
                         [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4, 6*tf**5, 7*tf**6],
                         [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3, 30*tf**4, 42*tf**5],
                         [0, 0, 0, 6, 24*tf, 60*tf**2, 120*tf**3, 210*tf**4]])
        coefficients = np.array((np.linalg.inv(LHS) * RHS).transpose())[0]
        return(coefficients)

    def correct_q0qf_loops(self, q0, qf):
        """ Tries to take loops across 0 and 2pi into account """
        check_deltas = [-2 * np.pi, 0, 2 * np.pi]
        distances = [abs(q0 - (qf + i)) for i in check_deltas]
        return(qf + check_deltas[distances.index(min(distances))])

    def angle_equation(self, cf, t):
        """
        Given coefficients (numpy array) and time vector (numpy array), calculates
        angle position through time (q)
        """
        q = cf[0] + cf[1]*t + cf[2]*t**2 + cf[3]*t**3 + cf[4]*t**4 +\
            cf[5]*t**5 + cf[6]*t**6 + cf[7]*t**7
        return(q)


if __name__ == '__main__':
    rospy.init_node('JacoPlusReflexDemo')
    try:
        main()
    except rospy.ROSInterruptException:
        pass
