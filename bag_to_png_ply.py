'''
ref: https://qiita.com/nataly510/items/c676a28a1119907a560d
'''

import os
import numpy as np
import cv2
# import yaml
# import time, datetime
import trimesh
import pyrealsense2 as rs
import argparse

from utils import rgbd_to_pcd

parser = argparse.ArgumentParser(description='extract png and ply image from bag')
parser.add_argument("-b", "--bag_file", help="bag_file",  required=True, type=str)
parser.add_argument("-i", "--interval", help="saving interval",  required=True, type=int)

if __name__ == "__main__":
    args = parser.parse_args()
    conf = {
        "dst_dir": "./log/png_and_ply/"
    }

    print("open : ", args.bag_file)
    config = rs.config()
    config.enable_device_from_file(args.bag_file,  repeat_playback=False)

    # config.enable_stream(...) is not required when read a bag file.
    # https://teratail.com/questions/218884
    # fps = conf["fps"]
    # w, h = conf["depth_size"]
    # config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    # w, h = conf["rgb_size"]
    # config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)

    pipeline = rs.pipeline()
    prof = pipeline.start(config)
    depth_sensor = prof.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale() * 1000.0 # to mm
    print('depth_scale: ', depth_scale)


    # apply an align process to align fov of rgb and depth frames. 
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    # calculate inv_intrinsics to convert rgbd images to plys.
    # Downcast to video_stream_profile and fetch intrinsics
    intrinsics = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    intrinsics = np.array([
        [intrinsics.fx, 0.0, intrinsics.ppx],
        [0.0, intrinsics.fy, intrinsics.ppy],
        [0.0, 0.0, 1.0],
    ])
    inv_intrinsics = np.linalg.inv(intrinsics)

    basename = os.path.basename(args.bag_file).split('.')[-2]    
    ite = 0

    while True:
        try:
            frames = pipeline.wait_for_frames()
        except RuntimeError:
            print("reached EOF or caused a loading error.")
            break
        frames = align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not depth_frame or not color_frame:
            continue

        image = np.array(color_frame.get_data())
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        depth = np.array(depth_frame.get_data()) * depth_scale
        cv2.imshow("image", image)
        cv2.imshow("depth", depth / 4000.0)
        key = cv2.waitKey(1)
        
        if key == ord('q'):
            break

        if ite % args.interval == 0:
            # increase a digit when a frame index is overflowed (e.g., {:010d} to {:015d}). 
            f = '{}_{:010d}.ply'.format(basename, ite)
            dst = os.path.join(conf['dst_dir'], f)
            points, point_colors = rgbd_to_pcd(inv_intrinsics, image, depth)
            pcd = trimesh.points.PointCloud(points, point_colors)
            pcd.export(dst)
            dst = dst.replace('.ply', '.png')
            cv2.imwrite(dst, image)
            print("{} / .ply are exported".format(dst))

        ite += 1

    pipeline.stop()   