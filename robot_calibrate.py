import rospy

from baxter.hand import KeyboardManipulator

"""
This script calibrate your Baxter to set up:
    * borders of the table
    * default position
    * limb height from table 
"""


class RobotCalibrator:
    def __init__(self, limb):
        self.manipulator = KeyboardManipulator(limb)

    def default_position(self):
        pose = self.manipulator.listen()
        return pose[:3]

    def height_table(self):
        pose = self.manipulator.listen()
        return pose[2]

    def borders_table(self):
        p1 = self.manipulator.listen()
        p2 = self.manipulator.listen()
        p3 = self.manipulator.listen()
        p4 = self.manipulator.listen()
        return p1[:3], p2[:3], p3[:3], p4[:3]

    def limb_height_table(self):
        pose = self.manipulator.listen()
        return pose[:3]


def robot_calibrate(limb):
    """
    Calibrating limb for normal work controller.py
    
    :param limb: aim limb for calibrate
    :return: calibrated params as tuple 
    """
    print '__________                  __'
    print '\______   \_____  ___  ____/  |_  ___________'
    print '|    |  _/\__  \ \  \/  /\   __\/ __ \_  __ \\'
    print '|    |   \ / __ \_>    <  |  | \  ___/|  | \/'
    print '|______  /(____  /__/\_ \ |__|  \___  >__|'
    print '       \/      \/      \/           \/'

    calibrator = RobotCalibrator(limb)

    print 'Calibrate your robot, follow instruction'
    print 'Use WASD as Counter-Strike ;)'
    print 'Use E and Q to up and down'

    print 'Set up default position and press ESC'
    position = calibrator.default_position()

    print 'Set up height of table and press ESC'
    height = calibrator.height_table()

    # print 'Set up borders of the table and press ESC'
    # borders = calibrator.borders_table()

    print 'Set up limb height from table and press ESC'
    limb_height = calibrator.limb_height_table()

    return position, height,  limb_height


if __name__ == '__main__':
    rospy.init_node('calibrating')
    result = robot_calibrate('left')

