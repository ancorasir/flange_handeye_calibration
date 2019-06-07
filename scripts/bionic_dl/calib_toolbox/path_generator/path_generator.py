import numpy as np

class PathGenerator:
    def __init__(self, cfg, pose_vec_curr, num_segment=3):
        self.curr_idx = 0
        self.init_pose = pose_vec_curr
        self.MAX_POINT = cfg.PATH_GENERATOR.MAX_POINT
        data_path = cfg.PATH_GENERATOR.POINT_PATH
        self.num_segment = num_segment

        self.origin_path_list = []
        self.interp_path_list = []

        self._read_pionts(data_path)
        self._generate_path()

    def _read_pionts(self, data_path):
        f = open(data_path, "r")
        lines = f.readlines()
        f.close()
        self.origin_path_list = []
        for line in lines:
            position = np.array(line.split(":")[1].split(","), dtype=float)
            self.origin_path_list.append(position)

    def _generate_path(self):
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
        if self.curr_idx > 0 and self.curr_idx > self.MAX_POINT:
            position = self.interp_path_list[self.curr_idx].tolist()
            position.extend(self.init_pose[3:].tolist())
            return np.array(position)
        else:
            return np.array([])

    def get_waypoint_last(self):
        self.curr_idx -= 1
        return self.get_waypoint_curr()

    def get_waypoint_next(self):
        self.curr_idx += 1
        return self.get_waypoint_curr()

