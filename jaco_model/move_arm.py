import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from reflex_msgs.msg import Hand


rospy.init_node('MoveJaco')
rospy.sleep(1.0)
joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
hand_pub = rospy.Publisher('/reflex_takktile/hand_state', Hand, queue_size=10)

joint_state = JointState()
joint_state.name = ['jaco_joint_1', 'jaco_joint_2', 'jaco_joint_3', 'jaco_joint_4', 'jaco_joint_5', 'jaco_joint_6']
joint_state.header = Header()
joint_state.header.stamp = rospy.Time.now()
joint_state.velocity = []
joint_state.effort = []

hand = Hand()

for i in range(10):
    value = -i / 5.0
    joint_state.position = [value, value, value, value, value, value]
    for j in range(3):
        hand.finger[j].proximal = -value
        hand.finger[j].distal_approx = -0.25 * value
    hand_pub.publish(hand)
    joint_pub.publish(joint_state)
    hand_pub.publish(hand)
    joint_pub.publish(joint_state)
    print("Published... " + str(value))
    rospy.sleep(3)
