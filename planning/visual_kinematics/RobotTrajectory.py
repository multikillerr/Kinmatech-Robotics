import numpy as np

from .Robot import *


class RobotTrajectory(object):
    def __init__(self, robot, frames, time_points=None, rot_tran_ratio=2.0):
        self.robot = robot
        if len(frames) < 2:
            raise Exception("trajectory must include at least 2 frames")
        if time_points is not None and len(frames) != len(time_points):
            raise Exception("time_points should have same length as frames")
        self.frames = frames
        self.time_points = time_points
        #  rot_tran_ratio 1.0 means 2*Pi rotation is treated as 1.0 meter in translation
        self.rot_tran_ratio = rot_tran_ratio

    def __len__(self):
        return len(self.frames)

    #  length of each segments considering only translation
    @property
    def len_segs_tran(self):
        lengths = np.zeros([len(self) - 1, ], dtype=np.float64)
        for i in range(len(self) - 1):
            lengths[i] = self.frames[i].distance_to(self.frames[i + 1])
        return lengths

    #  length of each segments considering only rotation
    @property
    def len_segs_rot(self):
        lengths = np.zeros([len(self) - 1, ], dtype=np.float64)
        for i in range(len(self) - 1):
            lengths[i] = self.frames[i].angle_to(self.frames[i + 1])
        return lengths

    #  length of each segments considering both rotation and translation
    @property
    def len_segs(self):
        return self.len_segs_rot * self.rot_tran_ratio / 2. / np.pi + self.len_segs_tran

    def interpolate(self, num_segs, motion="p2p", method="linear"):
        # !!! equal division, linear interpolation
        if self.time_points is None:
            lengths = self.len_segs
        else:
            lengths = self.time_points[1:] - self.time_points[:len(self)-1]
        length_total = np.sum(lengths)

        # axis angles for p2p, xyzabc for lin
        tra_array = np.zeros([len(self), max(self.robot.num_axis, 6)])
        for i in range(len(self)):
            if motion == "p2p":
                self.robot.inverse(self.frames[i])
                tra_array[i, 0:self.robot.num_axis] = self.robot.axis_values
            if motion == "lin":
                tra_array[i, 0:3] = np.array(self.frames[i].t_3_1.reshape([3, ]))
                tra_array[i, 3:6] = np.array(self.frames[i].euler_3)

        # interpolation values
        inter_values = np.zeros([num_segs + 1, self.robot.num_axis])
        inter_time_points = np.zeros([num_segs + 1])
        for progress in range(num_segs + 1):
            index = 0
            p_temp = progress * length_total / num_segs
            for i in range(lengths.shape[0]):
                if p_temp - lengths[i] > 1e-5:  # prevent numerical error
                    p_temp -= lengths[i]
                    index += 1
                else:
                    break
            p_temp /= lengths[index]  # the percentage of the segment, in range [0., 1.]
            if motion == "p2p":
                inter_values[progress] = tra_array[index, 0:self.robot.num_axis] * (1 - p_temp) + tra_array[index + 1,
                                                                                                  0:self.robot.num_axis] * p_temp
            if motion == "lin":
                xyzabc = tra_array[index, 0:6] * (1 - p_temp) + tra_array[index + 1, 0:6] * p_temp
                self.robot.inverse(Frame.from_q_4(xyzabc[3:6], xyzabc[0:3].reshape([3, 1])))
                inter_values[progress] = self.robot.axis_values
            inter_time_points[progress] = np.sum(lengths[0:index]) + lengths[index] * p_temp
        return inter_values, inter_time_points

    def show(self, num_segs=100, motion="p2p", method="linear"):
        """Display is disabled – matplotlib has been removed.
        Use interpolate() directly and pipe the result to the PyQtGraph
        visualiser in ui/main.py."""
        print("RobotTrajectory.show() is disabled – matplotlib has been removed.")
        return self.interpolate(num_segs, motion, method)
