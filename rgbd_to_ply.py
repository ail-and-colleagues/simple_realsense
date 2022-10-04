import os
import numpy as np
import cv2
import trimesh
import yaml
import argparse

from utils import rgbd_to_pcd, ColorMapper

parser = argparse.ArgumentParser(description='convert image and depth_map to ply')
parser.add_argument("-i", "--image", help="a color image path",  required=True)

if __name__ == "__main__":
    args = parser.parse_args()
    image_path = args.image
    
    dir = os.path.dirname(image_path)
    basename = os.path.basename(image_path)

    conf_path = os.path.join(dir, "conf.yaml")
        
    with open(conf_path) as f:
        conf = yaml.safe_load(f)
        print(conf)

    depth_path = os.path.join(dir, basename.replace("-img.png", "-depth.png"))
    
    image = cv2.imread(image_path)
    cv2.imshow("image", image)
    depth_colormap = cv2.imread(depth_path)
    cv2.imshow("depth_colormap", depth_colormap)
    cv2.waitKey(100)

    colormap = ColorMapper(conf["mx"], conf["mn"])
    depth = colormap.color_to_depth(depth_colormap)
    inv_intrinsics = np.array(conf["inv_intrinsics"]).reshape([3, 3])
    points, point_colors = rgbd_to_pcd(inv_intrinsics, image, depth)
    pcd = trimesh.points.PointCloud(points, point_colors)
    dst = os.path.join(dir, basename.replace("-img.png", ".ply"))
    print(dst)
    pcd.export(dst)
    





    