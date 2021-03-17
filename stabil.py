#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 14 23:54:27 2021

@author: samet
"""

import pyrealsense2 as rs
import numpy as np
import cv2
from math import tan, pi


#------------------------------------------------------------------------------
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)


def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])


def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])
#------------------------------------------------------------------------------


def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])


pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.fisheye,1)
config.enable_stream(rs.stream.fisheye,2)
config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)
profile = pipeline.start(config)
frame_data = {"left"  : None,
              "right" : None
              }
try:
    while True:
        frames = pipeline.wait_for_frames()
        nir_lf_frame = frames.get_fisheye_frame(1)
        nir_rg_frame = frames.get_fisheye_frame(2)
        accel = accel_data(frames[2].as_motion_frame().get_motion_data())
        gyro = gyro_data(frames[3].as_motion_frame().get_motion_data())
        nir_lf_image = np.asanyarray(nir_lf_frame.get_data())
        nir_rg_image = np.asanyarray(nir_rg_frame.get_data())
        '''
        frame_data["left"] =  nir_lf_image
        frame_data["right"] = nir_rg_image
        '''
        print(gyro, accel)
        
        # image=np.hstack((nir_lf_image,nir_rg_image)) -> iki goruntu birlesik
        cv2.imshow('Fisheye goruntu', nir_rg_image)
        
        
        '''
        window_size = 5
        min_disp = 0
        # must be divisible by 16
        num_disp = 112 - min_disp
        max_disp = min_disp + num_disp
        stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                   numDisparities = num_disp,
                                   blockSize = 16,
                                   P1 = 8*3*window_size**2,
                                   P2 = 32*3*window_size**2,
                                   disp12MaxDiff = 1,
                                   uniquenessRatio = 10,
                                   speckleWindowSize = 100,
                                   speckleRange = 32)
        profiles = pipeline.get_active_profile()

        # Retreive the stream and intrinsic properties for both cameras
        streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
               "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
        intrinsics = {"left"  : streams["left"].get_intrinsics(),
                  "right" : streams["right"].get_intrinsics()}
        
        K_left  = camera_matrix(intrinsics["left"])
        D_left  = fisheye_distortion(intrinsics["left"])
        K_right = camera_matrix(intrinsics["right"])
        D_right = fisheye_distortion(intrinsics["right"])
        (width, height) = (intrinsics["left"].width, intrinsics["left"].height)
        
        (R, T) = get_extrinsics(streams["left"], streams["right"])

        stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
        stereo_height_px = 300          # 300x300 pixel stereo output
        stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)

        R_left = np.eye(3)
        R_right = R


        stereo_width_px = stereo_height_px + max_disp
        stereo_size = (stereo_width_px, stereo_height_px)
        stereo_cx = (stereo_height_px - 1)/2 + max_disp
        stereo_cy = (stereo_height_px - 1)/2

        P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                       [0, stereo_focal_px, stereo_cy, 0],
                       [0,               0,         1, 0]])
        P_right = P_left.copy()
        P_right[0][3] = T[0]*stereo_focal_px


        Q = np.array([[1, 0,       0, -(stereo_cx - max_disp)],
                  [0, 1,       0, -stereo_cy],
                  [0, 0,       0, stereo_focal_px],
                  [0, 0, -1/T[0], 0]])

        m1type = cv2.CV_32FC1
        (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
        (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
        undistort_rectify = {"left"  : (lm1, lm2),
                         "right" : (rm1, rm2)}

        mode = "stack"
   
        frame_copy = {"left"  : frame_data["left"].copy(),
                          "right" : frame_data["right"].copy()}
        center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                          map1 = undistort_rectify["left"][0],
                                          map2 = undistort_rectify["left"][1],
                                          interpolation = cv2.INTER_LINEAR),
                                  "right" : cv2.remap(src = frame_copy["right"],
                                          map1 = undistort_rectify["right"][0],
                                          map2 = undistort_rectify["right"][1],
                                          interpolation = cv2.INTER_LINEAR)}

       

        color_image = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)
        
        cv2.imshow("Birlesik", color_image)
        '''
        
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
        pipeline.stop()

