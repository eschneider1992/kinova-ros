#include "move_arm.h"


ros::Publisher joint_pub;
sensor_msgs::JointState joint_state;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "MoveJacoArmPlusReflex");
    ros::NodeHandle n;
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    joint_state.name.resize(NUM_FIXED_STEPS + 3 * (NUM_FLEX_STEPS + 1));
    joint_state.position.resize(NUM_FIXED_STEPS + 3 * (NUM_FLEX_STEPS + 1));
    joint_state.name[0] ="jaco_joint_1";
    joint_state.name[1] ="jaco_joint_2";
    joint_state.name[2] ="jaco_joint_3";
    joint_state.name[3] ="jaco_joint_4";
    joint_state.name[4] ="jaco_joint_5";
    joint_state.name[5] ="jaco_joint_6";
    joint_state.name[6] ="proximal_joint_1";
    joint_state.name[7] ="proximal_joint_2";
    joint_state.name[8] ="proximal_joint_3";
    joint_state.name[9] ="preshape_1";
    joint_state.name[10] ="preshape_2";
    joint_state.name[11] ="distal_joint_1";
    joint_state.name[12] ="distal_joint_2";
    joint_state.name[13] ="distal_joint_3";

    char buffer[50];
    int index = NUM_FIXED_STEPS;
    for (int finger = 1; finger < 4; ++finger)
    {
        for (int i = 1; i < (NUM_FLEX_STEPS+2); ++i)
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

    ros::Publisher  pub = n.advertise<jaco_model::JacoPlusReflex>("/jaco_plus_reflex_cmd", 10);
    ros::Subscriber sub = n.subscribe("/jaco_plus_reflex_cmd", 10, publish_state_to_rviz);

    // Zero the system and make it appear open. The sleeps are to let RVIZ start up
    ros::Duration(2.0).sleep();
    jaco_model::JacoPlusReflex base_state;
    for (int i=0; i<10; i++) {
        pub.publish(base_state);
        ros::Duration(0.25).sleep();
    }

    ros::spin();
    return 0;
}


void publish_state_to_rviz(const jaco_model::JacoPlusReflexConstPtr& cmd) {
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = cmd->jaco_joint[0];
    joint_state.position[1] = cmd->jaco_joint[1];
    joint_state.position[2] = cmd->jaco_joint[2];
    joint_state.position[3] = cmd->jaco_joint[3];
    joint_state.position[4] = cmd->jaco_joint[4];
    joint_state.position[5] = cmd->jaco_joint[5];

    joint_state.position[6]  = cmd->proximal_joint[0];
    joint_state.position[7]  = cmd->proximal_joint[1];
    joint_state.position[8]  = cmd->proximal_joint[2];
    joint_state.position[9]  =  cmd->preshape_joint;
    joint_state.position[10] = -cmd->preshape_joint;

    // Distal joints are unused, b/c of the way the flex joints are modeled below
    // joint_state.position[11]
    // joint_state.position[12]
    // joint_state.position[13]

    int index = NUM_FIXED_STEPS;
    for (int finger = 0; finger < 3; ++finger)
    {
        for (int i = 0; i < (NUM_FLEX_STEPS+1); ++i)
        {
            joint_state.position[index] = cmd->distal_joint[finger] /
                                          (static_cast<float>(NUM_FLEX_STEPS+1));
            ++index;
        }
    }

    joint_pub.publish(joint_state);
}
