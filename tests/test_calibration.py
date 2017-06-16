# Calibration test for Baxter by Ivan 2017(c)
import cv2
import numpy as np
from tools import LocalCamera

TEST_NAME = "Camera calibration test for Baxter"


class Calibration:

    """ Compute distortion coefficients of Baxter's camera. """

    def __init__(self, chess_board_size=(6, 9)):
        self.board_size = chess_board_size
        self.objpoints = list()
        self.imgpoints = list()
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.0001)
        self.objp = np.zeros((chess_board_size[0] * chess_board_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:chess_board_size[0], 0:chess_board_size[1]].T.reshape(-1, 2)
        self.img_size = None

    def __str__(self):
        rep = "Class to compute undistortion options for Baxter's camera."
        return rep

    def collect_data(self, images):
        """ Collect object points and image points to calibrating. """
        for img in images:
            ret, corners = cv2.findChessboardCorners(img, self.board_size)  # Finding corners in image.
            self.img_size = (img.shape[1], img.shape[0])
            if ret:
                cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), self.criteria)  # Finding subpix corners coordinates.
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners)
            else:
                print("Can't find corners in photo.")
                # TODO: Print filename.

    def comput_params(self):
        """ Compute distortion options. """
        ret, mtx, dist, r_vec, t_vec = \
            cv2.calibrateCamera(self.objpoints, self.imgpoints, self.img_size)

        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(
            mtx, dist, imageSize=self.img_size, alpha=1, newImgSize=self.img_size)

        return mtx, new_camera_mtx, dist, roi

    def save_params(self, filename, params):
        """ Save undistortion options in file.  """
        mtx, new_camera_mtx, dist, roi = params
        x, y, w, h = roi
        np.savez(filename, x=x, y=y, w=w, h=h, dist=dist,
                 new_camera_mtx=new_camera_mtx, mtx=mtx)

    def read_images(self, filename):
        images = list()
        with open(filename) as paths:
            for path in paths:
                image = cv2.imread(path.strip(), flags=0)  # Flag 0 makes image in grayscale!
                images.append(image)

        return images


class CalibratePhoto:

    """ Use this class to make chessboard photo to calibrate camera. """

    def __init__(self, camera_name, camera_resolution=(640, 400), chessboard_size=(6, 9)):
        self.camera_name = camera_name + "_camera"
        self.resolution = camera_resolution
        self.basedir = ""  # Path to dir for calibrate photos.
        self.board_size = chessboard_size  # Number of rows an columns.
        self.photo_num = 1

    def __str__(self):
        rep = 'Makes data set of 30 photos to using calibration class.'
        return rep

    def photo_maker(self):
        """ Main func witch makes 30 chessboard photos.
         Press s_key to save frame.
         Press ESC to stop."""
        camera = LocalCamera.LocalCamera(self.camera_name, self.resolution)
        while self.photo_num <= 30:
            image = camera.get_frame()
            ret_corners, corners = cv2.findChessboardCorners(image, self.board_size)
            cv2.imshow('photo maker', image)  # If there are not corners video will be freezing.
            if cv2.waitKey(1) & 0xFF == ord('s'):  # Press s key to save frame.
                if ret_corners:  # If corners were found.
                    cv2.imwrite(self.basedir+'/' + str(self.photo_num) + '.png', image)
                    cv2.drawChessboardCorners(image, self.board_size, corners, ret_corners)
                    cv2.imshow('Saved!', image)  # Show saved frame with drawn corners and close it.
                    cv2.waitKey(500)
                    cv2.destroyWindow('Saved!')
                    print(str(30 - self.photo_num)+'frames left!')
                    self.photo_num += 1
                else:
                    print("Corners weren't found!")
            elif cv2.waitKey(1) & 0xFF == 27:  # Press ESC to escape.
                print("Exit")
                break

        if self.photo_num == 30:
            print('You have enough photos!!!')


def calibrate():
    calibrator = Calibration()
    images = calibrator.read_images('')
    # TODO: ask Nikita about open(filename) sa path =)
    calibrator.collect_data(images)
    params = calibrator.comput_params()
    calibrator.save_params('params', params)

def calib_photos():
    photomaker = CalibratePhoto('left')
    photomaker.photo_maker()

