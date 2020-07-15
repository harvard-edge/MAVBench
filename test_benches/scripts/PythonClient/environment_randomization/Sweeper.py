import json
from math import floor, ceil
from itertools import product
from RoborunRandomizer import RoborunRandomizer
import copy

# this class helps generate a new experiment setting per sweep
class Sweeper:
    def __init__(self, meta_experiment_setting):
        self.randomizer = RoborunRandomizer()
        self.meta_experiment_setting = meta_experiment_setting

        # these need to be handled separately
        # ArenaSize and PlayerStart and End need to be generated for GoalDistance
        # v_max is set in ros_params and not in the top level experiment settings
        self.special_vars = ["GoalDistance", "v_max"]

        if "sweep" in self.meta_experiment_setting.keys():
            self.vars_to_sweep = self.meta_experiment_setting["sweep"]
        else:
            self.vars_to_sweep = []

        # make sure sweep values have been set for variables to be sweeped
        for var in self.vars_to_sweep:
            assert var in self.meta_experiment_setting.keys(), "sweep values for " + str(var) + " not set!"

        if "episodes_per_sweep" in self.meta_experiment_setting.keys():
            self.episodes_per_sweep = self.meta_experiment_setting["episodes_per_sweep"]
        else:
            self.episodes_per_sweep = 1

        self.list_of_lists_of_sweep_values = []
        for var in self.vars_to_sweep:
            self.list_of_lists_of_sweep_values.append(self.meta_experiment_setting[var])
        self.all_combinations = list(product(*self.list_of_lists_of_sweep_values))
        self.num_sweeps = len(self.all_combinations)
        self.curr_combination = None
        self.curr_combination_ind = -1

        # we will later generate a new experiment setting with the sweeped params set to constants/
        # constant upper and lower bounds
        self.experiment_setting = None
        self.curr_gaussian_params_dict = {}

    def get_num_sweeps(self):
        return self.num_sweeps

    def get_sweeped_var_value(self, var):
        assert var in self.vars_to_sweep, str(var) + " must be a sweeped variable!"
        sweep_var_index_in_combination = self.vars_to_sweep.index(var)
        val = self.curr_combination[sweep_var_index_in_combination]
        return val

    def set_sweeped_experiment_setting(self):
        for var in self.meta_experiment_setting.keys():
            if var in self.vars_to_sweep:
                # we handle the special vars later
                if var not in self.special_vars:
                    actual_val = self.get_sweeped_var_value(var)
                    if var in self.randomizer.ranged_params:
                        # hack because RoborunRandomizer only accepts ranges for these variables
                        self.experiment_setting[var] = [actual_val, actual_val]
                    else:
                        self.experiment_setting[var] = actual_val

        self.augment_gaussian_params()
        self.augment_ros_params()

    def augment_gaussian_params(self):
        goal_dist = -1
        if "GoalDistance" in self.vars_to_sweep:
            goal_dist = self.get_sweeped_var_value("GoalDistance")
        elif "GoalDistance" in self.meta_experiment_setting.keys():
            goal_dist = self.meta_experiment_setting["GoalDistance"]
        else:
            # goal distance simply isn't specified, which means the user specified start, end, and arena size themselves
            return

        # we set the smallest arena size possible for the given goal constraints to minimize loading time
        # we want a slight angle between the start and the goal, so that the planner
        # is forced to plan through the obstacles
        goal_x_offset = -1
        goal_y_offset = -1
        if goal_dist > 200:
            goal_x_offset = 200
        else:
            # if the goal is very close by, we just keep the straight line
            goal_x_offset = goal_dist
        goal_y_offset = int(ceil((goal_dist**2 - goal_x_offset**2)**0.5))

        # adding 400m to arena bounds for some leeway to spawn obstacles
        constant_offset = 400
        # x's and y's are flipped for arena size in UE, TODO fix
        self.experiment_setting["ArenaSize"] = [goal_y_offset + constant_offset, goal_x_offset + constant_offset, 20]
        self.experiment_setting["PlayerStart"] = [-goal_x_offset/2, -goal_y_offset/2, 20]
        self.experiment_setting["End"] = [goal_x_offset/2, goal_y_offset/2, 50]
        # spawning a centroid at the midway point
        self.experiment_setting["Centroid1"] = [0, 0, 20]

        self.experiment_setting["max_run_time"] = 60#int(ceil(goal_dist * 2.5)) #60

    def augment_ros_params(self):
        ros_params = copy.deepcopy(self.experiment_setting["ros_params"])
        start = self.experiment_setting["PlayerStart"]
        end = self.experiment_setting["End"]
        goal_offset = [e_i - s_i for s_i, e_i in zip(start, end)]
        ros_params["goal_x"] = goal_offset[0]
        ros_params["goal_y"] = goal_offset[1]
        ros_params["goal_z"] = end[2]

        # arena bounds in mavbench coords
        # x's and y's are flipped for arena size in UE, TODO fix
        constant_offset = 300
        ros_params["x_dist_to_sample_from__low_bound"] = -(self.experiment_setting["ArenaSize"][1]/2) - self.experiment_setting["PlayerStart"][0] - constant_offset
        ros_params["x_dist_to_sample_from__high_bound"] = (self.experiment_setting["ArenaSize"][1]/2) - self.experiment_setting["PlayerStart"][0] + constant_offset
        ros_params["y_dist_to_sample_from__low_bound"] = -(self.experiment_setting["ArenaSize"][0]/2) - self.experiment_setting["PlayerStart"][1] - constant_offset
        ros_params["y_dist_to_sample_from__high_bound"] = (self.experiment_setting["ArenaSize"][0]/2) - self.experiment_setting["PlayerStart"][1] + constant_offset

        # v_max
        if "v_max" in self.vars_to_sweep:
            ros_params["v_max"] = self.get_sweeped_var_value("v_max")

        self.experiment_setting["ros_params"] = ros_params

    def get_sweeped_experiment_setting(self):
        return self.experiment_setting

    def get_sweeped_gaussian_params_dict(self):
        return self.curr_gaussian_params_dict

    def set_next_sweep_combination(self):
        self.curr_combination_ind += 1
        self.curr_combination = self.all_combinations[self.curr_combination_ind]

    def sweep_start(self):
        self.set_next_sweep_combination()

        # resetting our experiment setting to be generated
        self.experiment_setting = copy.deepcopy(self.meta_experiment_setting)
        for var in self.vars_to_sweep:
            # removing all the vars to be swept because we reset them to constants/constant bounds later in
            # set_sweeped_experiment_setting
            self.experiment_setting.pop(var, None)

        self.randomizer = RoborunRandomizer()
        self.set_sweeped_experiment_setting()
        self.curr_gaussian_params_dict = {}
        for param in self.randomizer.get_gaussian_params_list():
            if param in self.experiment_setting.keys():
                self.curr_gaussian_params_dict[param] = self.experiment_setting[param]
        self.randomizer.set_gaussian_params(self.curr_gaussian_params_dict)
        self.randomizer.init_env_gen()

    def do_episode(self):
        self.randomizer.randomize_env()
        self.randomizer.airsim_reset()

    def sweep_end(self):
        self.randomizer = None
        self.experiment_setting = None
        self.curr_gaussian_params_dict = {}
