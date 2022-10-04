'''
ref: https://qiita.com/tom_eng_ltd/items/635414ff0b43e1c506f6
'''

import os
import numpy as np
import cv2
import yaml
import time, datetime
from utils import Realsense, ColorMapper
import argparse

parser = argparse.ArgumentParser(description='capture and record image and depth_map')
parser.add_argument("-s", "--save_per_sec", help="saving rate specified like frame/save_per_sec",  required=True, type=int)


if __name__ == "__main__":
    conf = {
        "mx": 5.0,
        "mn": 0.5,
        "rgb_size": [640, 360],
        "depth_size": [640, 360],
        "fps": 15,
        # "save_per_sec":1
    }
    args = parser.parse_args()

    conf["save_per_sec"] = args.save_per_sec


    colormap = ColorMapper(conf["mx"], conf["mn"])
    realsense = Realsense(conf["rgb_size"], conf["depth_size"], conf["fps"])
    # save intrinsics for rgdb_to_ply
    conf["intrinsics"] = [float(c) for c in realsense.intrinsics.ravel()]
    conf["inv_intrinsics"] = [float(c) for c in realsense.inv_intrinsics.ravel()]

    save_per_sec = conf["save_per_sec"]
    realsense.toggle_align_mode()

    ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M")
    # ts = "test"
    dst_dir = os.path.join("./log", ts)
    os.makedirs(dst_dir, exist_ok=True)

    with open(os.path.join(dst_dir, "conf.yaml"), 'w') as f:
        yaml.dump(conf, f, encoding='utf-8', allow_unicode=True)


    last_frame = time.time()
    last_saved = last_frame
    while True:
        f, img, depth= realsense.next_frame()
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=50.0), cv2.COLORMAP_JET)
        # cv2.imshow("depth_colormap", depth_colormap)
        remapped_color_map = colormap.depth_to_color(depth)
        cv2.imshow("img", img)
        cv2.imshow("remapped_color_map", remapped_color_map)
        
        current = time.time()
        actual_fps = 1 / (current - last_frame)
        last_frame = current
        
        # chk>
        # _depth = colormap.color_to_depth(remapped_color_map)
        
        # print(np.mean(np.abs(depth - _depth)))
        # print(d)
        if current - last_saved >= 1 / save_per_sec:
            # print("{:.1f}".format(actual_fps), ts)
            last_saved = current
            basename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
            dst = os.path.join(dst_dir, (basename + "-img.png"))
            cv2.imwrite(dst, img)
            dst = os.path.join(dst_dir, (basename + "-depth.png"))
            cv2.imwrite(dst, remapped_color_map)
            # realsense.rgbd_to_pcd(img, _depth)

        key = cv2.waitKey(1)    
        if key == ord('q'):      
            break


    cv2.destroyAllWindows()
    









