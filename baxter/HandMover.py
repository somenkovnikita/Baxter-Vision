from moveit_commander import conversions
from baxter_core_msgs.srv import ( SolvePositionIK,
                                   SolvePositionIKRequest )
from std_msgs.msg import Header
import tf
import rospy
import baxter_interface


class HandMover:

    def __init__(self, limb):
        self.limb = limb
        self.limb_interface = baxter_interface.Limb(limb)
        self.pose = self._get_current_pose()

    def move(self, rpy_pose, move=False):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + self.limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            return False
        if ik_response.isValid[0]:
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            if move:
                self.limb_interface.move_to_joint_positions(limb_joints)
            else:
                self.limb_interface.set_joint_positions(limb_joints)
            return True

        else:
            return False

    def _get_current_pose(self):
        quaternion_pose = self.limb_interface.endpoint_pose()
        position = quaternion_pose["position"]
        quaternion = quaternion_pose["orientation"]
        euler = tf.transformations.euler_from_quaternion(quaternion)

        return [position[0], position[1], position[2], euler[0], euler[1], euler[2]]

    @staticmethod
    def _print_pose(rpy_pose, position, euler):
        print "             request   actual"
        print "front back = %5.4f " % rpy_pose[0], "%5.4f" % position[0]
        print "left right = %5.4f " % rpy_pose[1], "%5.4f" % position[1]
        print "up down    = %5.4f " % rpy_pose[2], "%5.4f" % position[2]
        print "roll       = %5.4f " % rpy_pose[3], "%5.4f" % euler[0]
        print "pitch      = %5.4f " % rpy_pose[4], "%5.4f" % euler[1]
        print "yaw        = %5.4f " % rpy_pose[5], "%5.4f" % euler[2]
