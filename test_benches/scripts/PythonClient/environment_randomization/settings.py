import os
import math

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
json_file_addr="C:\\Users\\Behzad-PC\\mavbench_stuff\\env-gen-ue4-my\\Build\\WindowsNoEditor\\AirLearning\\Content\\JsonFiles\\EnvGenConfig.json"

# ---------------------------
# zoning
# ---------------------------
# how many zones for each variable for the entire range. Note that frequency
# of moving to a new zone is not determined here
zone_dic = {"Seed": 1, "NumberOfDynamicObjects": 1, "MinimumDistance": 1, "VelocityRange": 1, "End": 4}  # pay attention

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
default_range_dic = easy_range_dic = {"End": zone_dic["End"] * ["Mutable"],
                                      "MinimumDistance": [2],
                                      "EnvType": ["Indoor"],
                                      "ArenaSize": [[50, 50, 20]],
                                      "PlayerStart": [[0, 0, 0]],
                                      "NumberOfDynamicObjects": list(range(0, 1)),
                                      "Walls1": [[255, 255, 10]],
                                      "Seed": list(range(0, 10000)),
                                      "VelocityRange": [[5, 25]],
                                      "Name": ["Name"],
                                      "NumberOfObjects": list(range(0, 1))}

medium_range_dic = {"End": zone_dic["End"] * ["Mutable"],
                    "MinimumDistance": [2],
                    "EnvType": ["Indoor"],
                    "ArenaSize": [[60, 60, 20]],
                    "PlayerStart": [[0, 0, 0]],
                    "NumberOfDynamicObjects": list(range(3, 6)),
                    "Walls1": [[255, 255, 10]],
                    "Seed": list(range(0, 5000)),
                    "VelocityRange": [[0, 3]],
                    "Name": ["Name"],
                    "NumberOfObjects": list(range(3, 6))}

hard_range_dic = {"End": zone_dic["End"] * ["Mutable"],
                  "MinimumDistance": [4],
                  "EnvType": ["Indoor"],
                  "ArenaSize": [[30, 30, 10]],
                  "PlayerStart": [[0, 0, 0]],
                  "NumberOfDynamicObjects": list(range(6, 10)),
                  "Walls1": [[255, 255, 10]],
                  "Seed": list(range(0, 5000)),
                  "VelocityRange": [[5, 25]],
                  "Name": ["Name"],
                  "NumberOfObjects": list(range(5, 10))}

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
environment_change_frequency = {"Seed": 1, "NumberOfObjects": 10, "End": 1}

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
