'''
ref: https://qiita.com/nataly510/items/c676a28a1119907a560d
'''

import os
import numpy as np
import cv2
# import yaml
# import time, datetime
import pyrealsense2 as rs
import argparse

parser = argparse.ArgumentParser(description='extract png and ply image from bag')
parser.add_argument("-b", "--bag_file", help="bag_file",  required=True, type=str)
parser.add_argument("-i", "--interval", help="saving interval",  required=True, type=int)

if __name__ == "__main__":
    args = parser.parse_args()
    conf = {
        "rgb_size": [640, 480],
        "depth_size": [640, 480],
        "fps": 30,
    }

    print("open : ", args.bag_file)
    config = rs.config()
    config.enable_device_from_file('./log/20220929_204021.bag')

    # fps = conf["fps"]
    # w, h = conf["depth_size"]
    # config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    # w, h = conf["rgb_size"]
    # config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)

    pipeline = rs.pipeline()
    prof = pipeline.start(config)
    depth_sensor = prof.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print('depth_scale: ', depth_scale)

    align_to = rs.stream.color
    align = rs.align(align_to)

    ite = 0
    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not depth_frame or not color_frame:
                continue

            image = np.array(color_frame.get_data())
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            depth = np.array(depth_frame.get_data())
            cv2.imshow("image", image)
            cv2.imshow("depth", depth * depth_scale)
            # print("mx: ", np.max(depth.ravel()), "mn: ", np.min(depth.ravel()))
            key = cv2.waitKey(1)
            if key == ord('q'):
                break

    finally:    
        pipeline.stop()   