import numpy as np


def make_path_generator(cfg, pose_vec_curr):
    return PathGenerator(cfg, pose_vec_curr)


class PathGenerator:
    def __init__(self, cfg, pose_vec_curr, num_segment=5):
        self.curr_idx = 0
        self.init_pose = pose_vec_curr
        data_path = cfg.PATH_GENERATOR.POINT_PATH
        self.num_segment = num_segment

        self.origin_path_list = []
        self.interp_path_list = []

        self._read_pionts(data_path)
        self._generate_path_seq()

    def _read_pionts(self, data_path):
        f = open(data_path, "r")
        lines = f.readlines()
        f.close()
        self.origin_path_list = []
        for line in lines:
            line = line[:-1]
            l = line.split(":")[1].split(",")
            while '' in l:
                l.remove('')
            position = np.array(l, dtype=float)
            self.origin_path_list.append(position)

    def _generate_path_seq(self):
        """
        Generate path by interpolation between adjacent points
        """

        paring_idx = []
        num_points = len(self.origin_path_list) - 1

        for i in range(num_points):
            paring_idx.append([i, i+1])

        seg_factor_list = np.linspace(0, 1, self.num_segment)
        for i, line in enumerate(paring_idx):
            print("Generating Points:%d/%d" % (i, len(paring_idx)))
            point_start = self.origin_path_list[line[0]]
            point_end = self.origin_path_list[line[1]]
            diff = point_end - point_start

            ## uncomment to add pre-defined points into path
            self.interp_path_list.append(point_start)

            for k in seg_factor_list:
                point = point_start + k * diff
                self.interp_path_list.append(point)

            ## uncomment to add pre-defined points into path
            if i == len(paring_idx) - 1:
                self.interp_path_list.append(point_end)

    def _generate_path_full(self):
        """
        Generate path by interpolation pair-wisely for all pre-defined positions
        """
        paring_idx = []
        num_points = len(self.origin_path_list)
        for i in range(num_points):
            for j in range(i + 1, num_points):
                paring_idx.append([i, j])

        seg_factor_list = np.linspace(0, 1, self.num_segment)

        for i, line in enumerate(paring_idx):
            print("Generating Points:%d/%d" % (i, len(paring_idx)))
            point_start = self.origin_path_list[line[0]]
            point_end = self.origin_path_list[line[1]]
            diff = point_end - point_start
            for k in seg_factor_list:
                point = point_start + k * diff
                self.interp_path_list.append(point)

    def get_waypoint_curr(self):
        """
        Get interpolate position with current index
        :return: pose_vec[x,y,z, q_w, q_x, q_y, q_z] or empty array if index reach the end
        """
        if self.curr_idx > 0 and self.curr_idx < len(self.interp_path_list):
            position = self.interp_path_list[self.curr_idx].tolist()
            position.extend(self.init_pose[3:].tolist())
            print("Processing waypoint: %d/%d"%(self.curr_idx, len(self.interp_path_list)))
            return np.array(position)
        else:
            return np.array([])

    def get_waypoint_prev(self):
        """
        Get interpolate position with previous index
        :return: pose_vec[x,y,z, q_w, q_x, q_y, q_z] or empty array if index reach the end
        """
        self.curr_idx -= 1
        return self.get_waypoint_curr()

    def get_waypoint_next(self):
        """
        Get interpolate position with next index
        :return: pose_vec[x,y,z, q_w, q_x, q_y, q_z] or empty array if index reach the end
        """
        self.curr_idx += 1
        return self.get_waypoint_curr()
