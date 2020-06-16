import os
import math
import numpy as np

# ---------------------------
# imports 
# ---------------------------
# Augmenting the sys.path with relavant folders
settings_file_path = os.path.realpath(__file__)
settings_dir_path = os.path.dirname(settings_file_path)
proj_root_path = os.path.abspath(settings_dir_path + "/..")
os.sys.path.insert(0, proj_root_path)
os.sys.path.insert(0, proj_root_path + "/environment_randomization")
# used for game configuration handling

# ---------------------------
# Machine dependent settings
# ---------------------------

#json_file_addr="C:\\Users\\Behzad-PC\\mavbench_stuff\\env-gen-ue4-my\\Content\\JsonFiles\\EnvGenConfig.json"
json_file_addr="C:\\Users\\Behzad-PC\\mavbench_stuff\\env-gen\\Build\\WindowsNoEditor\\AirLearning\\Content\\JsonFiles\\EnvGenConfig.json"

# ---------------------------
# zoning
# ---------------------------
# how many zones for each variable for the entire range. Note that frequency
# of moving to a new zone is not determined here
# we don't care about zones in roborun, so leaving this empty
zone_dic = {"Seed": 1, "End": 1}  # pay attention

# update_zone_success_threshold = 50
acceptable_success_rate_to_update_zone = .3  # after what ration of success up the zone # pay attention
update_zone_window = 1000  # the window within which the  update_zone_accpetable_success_rate
# needs to be achieved. Note that at the begining of every
# new window we zero out the achieved ratio

success_distance_to_goal = 3
slow_down_activation_distance = 2 * success_distance_to_goal  # detrmines at which distant we will punish the higher velocities

# ------------------------------------------------------------ 
#                               -space related-
# -----------------------------------------------------------
# ---------------------------
# range #pay attention
# ---------------------------
# TODO: set default to something besides easy or fix the number of Mutables equal

# we won't be using the difficulty levels this time, and instead
# manually expose the gaussian params through the experiment json config

# setting some defaults

default_range_dic = gaussian_range_dic = {"End": zone_dic["End"] * ["Mutable"],
                  "MinimumDistance": [4],
                  "EnvType": ["Indoor"],
                  "NumberOfDynamicObjects": list(range(0, 1)),
                  "Walls1": [[255, 255, 10]],
                  "Seed": list(range(0, 5000)),
                  "VelocityRange": [[5, 25]],
                  "Name": ["Name"],
                  "NumberOfObjects": list(range(0, 1)),
                  # for the more "organic" env-gen for
                  # roborun
                  "ArenaSize": [[120, 120, 10]],
                  "PlayerStart": [[0, 0, 0]],
                  "End": [[20, 20, 20]],
                  "Centroid1": [[-1]],
                  "Centroid2": [[-1]],
                  "Centroid3": [[-1]],
                  "GridSize": [5],
                  "PeakCongestion": list(np.linspace(0.3, 0.9)),
                  "SpreadOfObstacles": list(np.linspace(10, 30)),
                  "GapSize": list(np.linspace(5.0, 8.0)),
                  "EnvGenFlavor": ["gaussian"]
                  }


# ------------------------------------------------------------
#                               -game related-
# ------------------------------------------------------------
game_proc_pid = ''  # process associa

# TODO: this has to infered.
max_zone = zone_dic["End"]  # should be equal to mutable or total number of zones possible
# ---------------------------
# sampling frequency
# ---------------------------
# game config variables
# environment_change_frequency = {"Seed":5, "NumberOfObjects":1,\
#        "NumberOfDynamicObjects": 20, "MinimumDistance": 30, "VelocityRange":40} #this is based on episodes

end_randomization_mode = "inclusive"  # whether each level of difficulty should be inclusive (including the previous level) or exclusive

# how frequently to update the environment this is based on epides
environment_change_frequency = {"Seed": 1, "PeakCongestion": 1, "SpreadOfObstacles": 1, "GapSize": 1}

# ------------------------------------------------------------
#                               -Drone related-
## ------------------------------------------------------------
#ip = '10.243.49.243'
ip = '127.0.0.1'

ease_constant = 2  # used when not meeting a zone for window_restart_ctr_threshold times. scales the randomization freq

# ---------------------------
# meta data  reload for reproducability
# ---------------------------
use_preloaded_json = False

