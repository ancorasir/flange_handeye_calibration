class PathGenerator:
    def __init__(self, init_pose, point_list):
        self.curr_idx = 0
        self.init_pose = init_pose
        self.point_list = point_list
        self.path_list = self._generate_path()

    # TODO: Implementation
    def _generate_path(self):
        path_list = []
        return path_list

    def get_waypoint_last(self):
        self.curr_idx -= 1
        return self.path_list[self.curr_idx]

    def get_waypoint_next(self):
        self.curr_idx += 1
        return self.path_list[self.curr_idx]
