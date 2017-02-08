import cv2 as cv
import numpy as np


class Calibration:
    def __init__(self, chess_board_rows, chess_board_columns, img_width, img_height):
        self.rows = chess_board_rows
        self.columns = chess_board_columns
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.00001)
        self.objpoints = []
        self.imgpoints = []
        self.img_width = img_width
        self.img_height = img_height
        self.objp = np.zeros((self.rows * self.columns, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.columns, 0:self.rows].T.reshape(-1, 2)

    def find_corners(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        self.ret, self.corners = cv.findChessboardCorners(gray, (self.rows, self.columns), None)
        if self.ret:
            return True
        else:
            return False

    def drawing_corners(self, frame):
        cv.drawChessboardCorners(frame, (self.rows, self.columns), self.corners, self.ret)

    def corners(self, frame):  # for each chess board's photo

        ret, corners = cv.findChessboardCorners(frame, (self.rows, self.columns), None)
        cv.cornerSubPix(frame, corners, (11, 11), (-1, -1), self.criteria)
        self.objpoints.append(self.objp)
        self.imgpoints.append(corners)

    def calibration(self):  # after using corners func!
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(self.objpoints, self.imgpoints,
                                                          (self.img_width, self.img_height), None, None)
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, imageSize=(self.img_width, self.img_height),
                                                         alpha=-1, newImgSize=(self.img_width, self.img_height))
        return mtx, dist, newcameramtx

    def remap_params(self, mtx, dist, newcameramtx):  # return two values to undistort image
        mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx,
                                                (self.img_width, self.img_height), cv.CV_16SC2)
        return mapx, mapy  # Use numpy.save to save this and nampy.load to load
