#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 22:59:02 2021

@author: samet
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 14 23:54:27 2021

@author: samet
"""

import pyrealsense2 as rs
import numpy as np
import cv2


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
#4 tane stream enable edildiği için frame üzerinden 0-1-2-3 indexleri ile stream'lere ulaşılabilir
profile = pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        left_frame = frames.get_fisheye_frame(1) 
        right_frame = frames.get_fisheye_frame(2)
        accel = accel_data(frames[2].as_motion_frame().get_motion_data())
        gyro = gyro_data(frames[3].as_motion_frame().get_motion_data())
        left_image = np.asanyarray(left_frame.get_data())
        right_image = np.asanyarray(right_frame.get_data())

        print(gyro, accel)
        
        # image=np.hstack((nir_lf_image,nir_rg_image)) -> iki goruntu birlesik
        cv2.imshow('Fisheye goruntu', right_image)
        

        
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
        pipeline.stop()
