# -*- coding: utf-8 -*-
"""
Created on Tue Jan 31 13:37:29 2023

@author: monakhov
"""

import numpy as np
import cv2

projectors = ['master']
for proj in projectors:
    coords_proj_f = '/home/altair/odin/src/calibration/homography/proj_{}_points'.format(proj)
    coords_depth_f = '/home/altair/odin/src/calibration/homography/dm_{}_points'.format(proj)
    
    coords_proj = np.loadtxt(coords_proj_f,dtype=np.float32)
    coords_depth = np.loadtxt(coords_depth_f,dtype=np.float32)
    
    M = cv2.findHomography(coords_depth,coords_proj)[0]
    np.save('/home/altair/odin/src/calibration/homography/depth_proj_{}'.format(proj),M)