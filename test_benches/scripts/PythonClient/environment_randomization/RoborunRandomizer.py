import roborun_settings
import numpy as np
from EnvRandomizer import EnvRandomizer

class RoborunRandomizer:
    def __init__(self):
        self.env_rand = None
        self.centroid_list = ["Centroid1", "Centroid2", "Centroid3"]
        self.ranged_params = ["SpreadOfObstacles", "PeakCongestion", "GapSize"]
        self.constant_params = ["ArenaSize", "PlayerStart", "End", "GridSize", "EnvGenFlavor"]

        self.gaussian_params = []
        self.gaussian_params.extend(self.centroid_list)
        self.gaussian_params.extend(self.ranged_params)
        self.gaussian_params.extend(self.constant_params)

        self.gaussian_range_dic = roborun_settings.gaussian_range_dic

    def set_gaussian_params(self, args):
        for el in args.keys():
            key = el
            bounds = args[key]
            if key in self.ranged_params:
                assert (type(bounds) is list and len(bounds) == 2), str(key) + \
                        " needs to be list, i.e. the input needs to be provided in the form of [lower bound, upper bound]"
                value = list(np.linspace(bounds[0], bounds[1]))
            else:
                value = [bounds]

            self.gaussian_range_dic[key] = value

    def init_env_gen(self):
        self.env_rand = EnvRandomizer()
        self.env_rand.init_again(self.gaussian_range_dic)

    def randomize_env(self):
        assert (self.env_rand is not None), "Initialize the generator first!"
        self.env_rand.randomize_env()

    def get_airsim_client(self):
        assert (self.env_rand is not None), "Initialize the generator first!"
        return self.env_rand.airsim_client

    def airsim_reset(self):
        assert (self.env_rand is not None), "Initialize the generator first!"
        return self.env_rand.airsim_reset()

    def get_gaussian_params_list(self):
        return self.gaussian_params

    # kept this to keep clct_data from breaking
    def augment_ros_params(self, ros_params):
        assert (type(ros_params) is dict), "ros_params must be a dict!"
        start = self.gaussian_range_dic["PlayerStart"][0]
        end = self.gaussian_range_dic["End"][0]
        goal_offset = [e_i - s_i for s_i, e_i in zip(start, end)]
        ros_params["goal_x"] = goal_offset[0]
        ros_params["goal_y"] = goal_offset[1]
        ros_params["goal_z"] = end[2]

        # arena bounds in mavbench coords
        constant_offset = 300
        ros_params["x_dist_to_sample_from__low_bound"] = -(self.gaussian_range_dic["ArenaSize"][0][1]/2) - self.gaussian_range_dic["PlayerStart"][0][0] - constant_offset
        ros_params["x_dist_to_sample_from__high_bound"] = (self.gaussian_range_dic["ArenaSize"][0][1]/2) - self.gaussian_range_dic["PlayerStart"][0][0] + constant_offset
        ros_params["y_dist_to_sample_from__low_bound"] = -(self.gaussian_range_dic["ArenaSize"][0][0]/2) - self.gaussian_range_dic["PlayerStart"][0][1] - constant_offset
        ros_params["y_dist_to_sample_from__high_bound"] = (self.gaussian_range_dic["ArenaSize"][0][0]/2) - self.gaussian_range_dic["PlayerStart"][0][1] + constant_offset

        return ros_params
