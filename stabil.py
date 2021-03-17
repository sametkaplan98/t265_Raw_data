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
     
        print(gyro, accel)
        
        # image=np.hstack((nir_lf_image,nir_rg_image)) -> iki goruntu birlesik
        cv2.imshow('Fisheye goruntu', nir_rg_image)       
        
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
        pipeline.stop()

