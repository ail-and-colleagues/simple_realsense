
import numpy as np
import pyrealsense2 as rs
import matplotlib as mpl
import math

class Realsense():
    def __init__(self, rgb_size, depth_size, fps, device_idx=0):

        ctx = rs.context()
        if len(ctx.devices) == 0:
            print("No Intel Device connected")
            exit

        if len(ctx.devices) <= device_idx:
            print("Device {}th is not available".format(device_idx)) 
            exit

        self.device = ctx.devices[device_idx]
        device_name = self.device.get_info(rs.camera_info.name)
        device_serial = self.device.get_info(rs.camera_info.serial_number)

        print ('Found device: ', device_name, ", ", device_serial)


        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, rgb_size[0], rgb_size[1], rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, depth_size[0], depth_size[1], rs.format.z16, fps)
        self.pipeline = rs.pipeline()
        self.prof = self.pipeline.start(self.config)

        # force emitter on.
        # does not work?
        depth_sensor = self.prof.get_device().query_sensors()[0]
        depth_sensor.set_option(rs.option.emitter_enabled, True)
        emitter = depth_sensor.get_option(rs.option.emitter_enabled)
        print("emitter: ", emitter)
        self.align = None

        # https://github.com/IntelRealSense/librealsense/issues/869
        intrinsics = self.prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
        # print(self.intrinsics.ppx, self.intrinsics.ppy, self.intrinsics.fx, self.intrinsics.fy)
        self.intrinsics = np.array([
            [intrinsics.fx, 0.0, intrinsics.ppx],
            [0.0, intrinsics.fy, intrinsics.ppy],
            [0.0, 0.0, 1.0],
        ])

        self.inv_intrinsics = np.linalg.inv(self.intrinsics)
        # print(self.intrinsics, self.inv_intrinsics, self.intrinsics @ self.inv_intrinsics )

        self.iamge_blk = np.zeros([rgb_size[0], rgb_size[1], 4])
        self.depth_blk = np.zeros([depth_size[0], depth_size[1], 1])
        # rs.depth_sensor.get_option(rs.option.depth_units, 0.0001)
        depth_sensor = self.prof.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def toggle_align_mode(self):
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def next_frame(self):

        try:
            frames = self.pipeline.wait_for_frames()
            if self.align:
                frames = self.align.process(frames)

            image = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not image or not depth:
                return False, self.iamge_blk, self.depth_blk
            image = np.asanyarray(image.get_data())
            depth = np.asanyarray(depth.get_data())
            depth = self.depth_scale * depth
            return True, image, depth
        except Exception as e:
            print("err: ", e)
            return False, self.iamge_blk, self.depth_blk

            
class ColorMapper:
    def __init__(self, mx, mn):
        self.mx, self.mn = mx, mn
        # A "gist_rainbow" can create a colormap that has arbitrary steps(resolutions) via the "resampled" function.
        # However, it will reduce when the colors are converted to an 8-bit unsigned int field (i.e., some steps are replaced with the same color).
        # Therefore, actually, this "colormapper" has only 1377 steps.
        # The precision of depth will be rounded as the range(specified by max and min.) / the steps. 
        resol = 10000
        rainbow = mpl.colormaps["gist_rainbow"].resampled(resol)
        rainbow = [rainbow(i / resol) for i in range(resol)]
        rainbow = (255.0 * np.array(rainbow)).astype(np.uint8)
        _, idx = np.unique(rainbow, axis=0, return_index=True)
        rainbow = rainbow[np.sort(idx)][:, :3]
        color_num = rainbow.shape[0]
        self.colormap = np.zeros([color_num + 1, 3], dtype=np.uint8)

        # assign brack(0, 0, 0) to colormap[0].
        # this indicates the depth is smaller than the min, larger than the max, or unmeasured.
        self.colormap[0] = [0, 0, 0]
        self.colormap[1:] = rainbow
        self.colormap_resol = self.colormap.shape[0]

        print("colormap_resol: ", self.colormap_resol)

        # store color and depth correspondence.
        self.color_depth_dict = dict()
        for i, c in enumerate(self.colormap):
            depth = self.mn + (i / (self.colormap_resol - 1)) * (self.mx - self.mn)
            self.color_depth_dict["{}_{}_{}".format(c[0], c[1], c[2])] = depth
        self.color_depth_dict["0_0_0"] = 0.0

    def depth_to_color(self, depth):
        temp = depth.reshape([-1, 1])
        remapped = (temp - self.mn) / (self.mx - self.mn)
        remapped = np.where((remapped < 0.0) | (remapped > 1.0), 0.0, remapped)

        color = self.colormap[(self.colormap_resol * remapped).astype(np.int32)]
        color = np.reshape(color, [depth.shape[0], depth.shape[1], -1])
        return color

    def color_to_depth(self, colored):
        ret = colored.reshape([-1, 3])
        ret = [self.color_depth_dict["{}_{}_{}".format(c[0], c[1], c[2])] for c in ret]
        ret = np.array(ret)
        return ret.reshape([colored.shape[0], colored.shape[1]]) 


def rgbd_to_pcd(uv_vecs, image, depth):
    points = list()
    point_col = list()

    image = image.reshape([-1, 3])
    depth = depth.reshape([-1, 1])

    points = [d * v for d, v in zip(depth, uv_vecs)]
    point_col = image[:, ::-1]

    points = np.array(points)
    point_col = np.array(point_col)

    return points, point_col
