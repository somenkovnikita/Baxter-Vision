# coding=utf-8

import math
import baxter_interface
import cv2
import rospy
import tf
from baxter_core_msgs.srv import SolvePositionIK
from baxter_core_msgs.srv import SolvePositionIKRequest
from moveit_commander import conversions


# TODO: in docs: MoveIt library required


class HandMover:
    """
    Class for kinematics calculate for moving robot limb in plane table
    """
    def __init__(self, limb):
        # type: (str) -> None
        """
        Choose limb to controlling
        
        :param limb: 'right' or 'left' limb 
        """
        self.limb = limb
        self.limb_interface = baxter_interface.Limb(limb)
        self.gripper = baxter_interface.Gripper(limb)
        self.gripper.reset()
        self.gripper.calibrate()
        self.pose = self.get_current_pose()
        self.pose[3:] = [-3.14, 0.0, 0.0]
        self._move(self.pose, move=True)

    def try_move(self, x=None, y=None, z=None):
        # type: (float, float, float) -> bool
        """
        Calculate kinematics and try moving limb
        
        :param x: x in baxter coordinates (forward/backward)
        :param y: y in baxter coordinates (left/right)
        :param z: z in baxter coordinates (down/up)
        :return: True if move success, False otherwise
        """
        rpy_pose = self.get_current_pose()
        rpy_pose[0] = x or rpy_pose[0]
        rpy_pose[1] = y or rpy_pose[1]
        rpy_pose[2] = z or rpy_pose[2]

        return self._move(rpy_pose, move=True)

    def try_delta_move(self, dx=None, dy=None, dz=None):
        # type: (float, float, float) -> bool
        """
        Calculate kinematics and try moving relative to the current limb point 

        :param dx: dx in baxter coordinates (forward/backward)
        :param dy: dy in baxter coordinates (left/right)
        :param dz: dz in baxter coordinates (down/up)
        :return: True if move success, False otherwise
        """
        rpy_pose = self.get_current_pose()
        rpy_pose[0] += dx or 0
        rpy_pose[1] += dy or 0
        rpy_pose[2] += dz or 0

        return self._move(rpy_pose, move=True)

    def get_current_pose(self):
        # type: () -> list
        """
        Return current Baxter pose 
        
        :return: rpy pose
        """
        quaternion_pose = self.limb_interface.endpoint_pose()
        position = quaternion_pose["position"]
        quaternion = quaternion_pose["orientation"]
        euler = tf.transformations.euler_from_quaternion(quaternion)

        return [position[0], position[1], position[2], euler[0], euler[1], euler[2]]

    def rotate_gripper(self, angle):
        rpy_pose = self.get_current_pose()


        return self._move(rpy_pose, move=True)

    def set_gripper(self, command):
        if command is True:
            self.gripper.open()
        elif command is False:
            self.gripper.close()

    def _move(self, rpy_pose, move=False):
        print 'move:', rpy_pose
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

    # only for debug
    @staticmethod
    def _print_pose(rpy_pose, position, euler):
        print "             request   actual"
        print "front back = %5.4f " % rpy_pose[0], "%5.4f" % position[0]
        print "left right = %5.4f " % rpy_pose[1], "%5.4f" % position[1]
        print "up down    = %5.4f " % rpy_pose[2], "%5.4f" % position[2]
        print "roll       = %5.4f " % rpy_pose[3], "%5.4f" % euler[0]
        print "pitch      = %5.4f " % rpy_pose[4], "%5.4f" % euler[1]
        print "yaw        = %5.4f " % rpy_pose[5], "%5.4f" % euler[2]


class KeyboardManipulator:
    delta_move = 0.05
    esc_key = 27

    def __init__(self, limb):
        # type: (str) -> None
        """
        Choose limb to manipulator

        :param limb: 'right' or 'left' limb 
        """
        from camera import Camera
        self.hand = HandMover(limb)
        self.camera = Camera(limb + '_hand')
        self.commands = {
            ord('w'): self.forward,
            ord('s'): self.backward,
            ord('a'): self.left,
            ord('d'): self.right,
            ord('q'): self.down,
            ord('e'): self.up,
        }

    def listen(self):
        while True:
            frame = self.camera.get_frame()
            cv2.imshow('Calibrate, suka!', frame)
            command = cv2.waitKey(20) & 0xFF
            if command == KeyboardManipulator.esc_key:
                break

            execute = self.commands.get(command)
            if execute is not None:
                execute()
        return self.hand.get_current_pose()

    def down(self):
        dz = -KeyboardManipulator.delta_move
        return self.hand.try_delta_move(dz=dz)

    def up(self):
        dz = KeyboardManipulator.delta_move
        return self.hand.try_delta_move(dz=dz)

    def left(self):
        dy = -KeyboardManipulator.delta_move
        return self.hand.try_delta_move(dy=dy)

    def right(self):
        dy = KeyboardManipulator.delta_move
        return self.hand.try_delta_move(dy=dy)

    def backward(self):
        dx = -KeyboardManipulator.delta_move
        return self.hand.try_delta_move(dx=dx)

    def forward(self):
        dx = KeyboardManipulator.delta_move
        return self.hand.try_delta_move(dx=dx)

