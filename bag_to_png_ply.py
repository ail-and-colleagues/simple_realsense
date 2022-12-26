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
import datetime
import math
from utils import rgbd_to_pcd

parser = argparse.ArgumentParser(description='extract png and ply image from bag')
parser.add_argument("-b", "--bag_file", help="bag_file or dir",  required=True, type=str)
parser.add_argument("-i", "--interval", help="saving interval",  required=True, type=int)
parser.add_argument("-o", "--out_dir", help="output directory",  required=True, type=str)

def create_contents_list(dataPath):
    t = os.listdir(dataPath)
    t = [os.path.join(dataPath, c) for c in t]
    files = [c for c in t if os.path.isfile(c)]
    directories = [c for c in t if os.path.isdir(c)]
    return files, directories

def proc(conf, bag_file):
    config = rs.config()
    config.enable_device_from_file(bag_file,  repeat_playback=False)

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
    intrinsics_mtx = np.array([
        [intrinsics.fx, 0.0, intrinsics.ppx],
        [0.0, intrinsics.fy, intrinsics.ppy],
        [0.0, 0.0, 1.0],
    ])
    inv_intrinsics = np.linalg.inv(intrinsics_mtx)
    # intrinsics.width
    w, h = intrinsics.width, intrinsics.height
    uv_vecs = [(np.mod(i, w), int(i / w), 1.0) for i in range(w * h)]
    uv_vecs = [inv_intrinsics @ t for t in uv_vecs]
    uv_vecs = np.array(uv_vecs)    

    basename = os.path.basename(bag_file).split('.')[-2]
    dst_dir = os.path.join(conf['dst_dir'], basename)
    os.makedirs(dst_dir, exist_ok=True)
    
    first_frame_number = None
    tgt_frame_number = None
    while True:
        try:
            frames = pipeline.wait_for_frames()
            # adp, dt = math.modf(frames.timestamp / 1000.0)
            # dt = datetime.datetime.fromtimestamp(dt)
            # print(dt, frames.frame_number, adp)
        except RuntimeError:
            print("reached EOF or caused a loading error.")
            break
        if not first_frame_number:
            first_frame_number = frames.frame_number
            tgt_frame_number = frames.frame_number
            print("first_frame_number: ", first_frame_number)

        if tgt_frame_number > frames.frame_number:
            continue
        tgt_frame_number += args.interval

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

        ofs = frames.frame_number - first_frame_number
        # increase a digit when a frame index is overflowed (e.g., {:010d} to {:015d}). 
        f = '{}_{:010d}.ply'.format(basename, ofs)
        dst = os.path.join(dst_dir, f)
        points, point_colors = rgbd_to_pcd(uv_vecs, image, depth)
        pcd = trimesh.points.PointCloud(points, point_colors)
        pcd.export(dst)
        dst = dst.replace('.ply', '.png')
        cv2.imwrite(dst, image)
        print("{} / .ply are exported".format(dst))


    pipeline.stop()   

if __name__ == "__main__":
    args = parser.parse_args()
    conf = {
        "dst_dir": args.out_dir,
    }

    if os.path.isfile(args.bag_file):        
        print("open : ", args.bag_file)
        proc(conf, args.bag_file)
    else:
        files, _ = create_contents_list(args.bag_file)
        files = [f for f in files if os.path.splitext(f)[1] == '.bag']
        print('{} in {}'.format(files, args.bag_file))
        for f in files:
            print("open : ", f)
            proc(conf, f)
