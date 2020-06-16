import settings
import os
from .game_config_class import *
import copy
import random
import numpy as np


def _get_random_end_point(arena_size, split_index, total_num_of_splits):
    # distance from the walls
    wall_halo = floor_halo = roof_halo = 1
    goal_halo = settings.slow_down_activation_distance + 1

    sampling_quanta = .5  # sampling increment

    # how big the split is (in only one direction, i.e pos or neg)
    idx0_quanta = float((arena_size[0] - 2 * goal_halo - 2 * wall_halo)) / (2 * total_num_of_splits)
    idx1_quanta = float((arena_size[1] - 2 * goal_halo - 2 * wall_halo)) / (2 * total_num_of_splits)
    idx2_quanta = float((arena_size[2])) / (2 * total_num_of_splits)

    idx0_up_pos_bndry = (split_index + 1) * idx0_quanta
    idx1_up_pos_bndry = (split_index + 1) * idx1_quanta
    idx2_up_pos_bndry = (split_index + 1) * idx2_quanta

    if (settings.end_randomization_mode == "inclusive"):
        idx0_low_pos_bndry = 0
        idx1_low_pos_bndry = 0
        idx2_low_pos_bndry = 0
    else:
        idx0_low_pos_bndry = (split_index) * idx0_quanta
        idx1_low_pos_bndry = (split_index) * idx1_quanta
        idx2_low_pos_bndry = (split_index) * idx2_quanta

    assert (
            idx0_up_pos_bndry - idx0_low_pos_bndry > sampling_quanta), "End doesn't fit within the zone, expand the arena size or reduce number of zones"
    assert (
            idx1_up_pos_bndry - idx1_low_pos_bndry > sampling_quanta), "End doesn't fit within the zone, expand the arena size or reduce number of zones"
    assert (
            idx2_up_pos_bndry - idx2_low_pos_bndry > sampling_quanta), "End doesn't fit within the zone, expand the arena size or reduce number of zones"

    rnd_pos_idx0 = random.choice(list(np.arange(
        idx0_low_pos_bndry + goal_halo, idx0_up_pos_bndry + goal_halo, sampling_quanta)))
    rnd_pos_idx1 = random.choice(list(np.arange(
        idx1_low_pos_bndry + goal_halo, idx1_up_pos_bndry + goal_halo, sampling_quanta)))
    rnd_pos_idx2 = random.choice(list(np.arange(
        idx2_low_pos_bndry + goal_halo, idx2_up_pos_bndry + goal_halo, sampling_quanta)))

    rnd_neg_idx0 = random.choice(list(np.arange(
        -idx0_up_pos_bndry - goal_halo, -idx0_low_pos_bndry - goal_halo, sampling_quanta)))

    rnd_neg_idx1 = random.choice(list(np.arange(
        -idx1_up_pos_bndry - goal_halo, -idx1_low_pos_bndry - goal_halo, sampling_quanta)))

    rnd_neg_idx2 = random.choice(list(np.arange(
        -idx2_up_pos_bndry - goal_halo, -idx2_low_pos_bndry - goal_halo, sampling_quanta)))

    rnd_idx0 = random.choice([rnd_neg_idx0, rnd_pos_idx0])
    rnd_idx1 = random.choice([rnd_neg_idx1, rnd_pos_idx1])
    rnd_idx2 = random.choice([rnd_neg_idx2, rnd_pos_idx2])

    """
	idx0_up_pos_bndry = int(arena_size[0]/2)
	idx1__up_pos_bndry = int(arena_size[1]/2)
	idx2__up_pos_bndry = int(arena_size[2])

	idx0_neg_bndry = int(-1*arena_size[0]/2)
	idx1_neg_bndry = int(-1*arena_size[1]/2)
	idx2_neg_bndry = 0
	
	rnd_idx0 = random.choice(list(range(
		idx0_neg_bndry + end_halo, idx0_pos_bndry - end_halo)))
	
	rnd_idx1 = random.choice(list(range(
		idx1_neg_bndry + end_halo, idx1_pos_bndry - end_halo)))
	 
	rnd_idx2 = random.choice(list(range(
	   0 + floor_halo, idx2_pos_bndry - roof_halo)))
	"""
    grounded_idx2 = 0  # to force the end on the ground, otherwise, it'll
    # be fallen (due to gravity) but then distance
    # calculation to goal becomes faulty

    if (rnd_idx0 == rnd_idx1 == 0):  # to avoid being on the start position
        rnd_idx0 = idx0_pos_bndry - end_halo

    return [rnd_idx0, rnd_idx1, grounded_idx2]


class AirlearningGameConfigHandler:
    def __init__(self, range_dic=eval("settings.default_range_dic"), zone_dic=eval("settings.zone_dic"),
                 input_file_addr=settings.json_file_addr):
        assert (os.path.isfile(input_file_addr)), input_file_addr + " doesnt exist"
        self.input_file_addr = input_file_addr
        self.cur_game_config = GameConfig(input_file_addr)
        self.game_config_range = copy.deepcopy(self.cur_game_config)
        self.set_range(*[el for el in range_dic.items()])
        self.game_config_zones = copy.deepcopy(self.game_config_range)
        self.zone_dic = zone_dic
        self.total_number_of_zones = 0
        self.populate_zones()
        #if (settings.use_preloaded_json):
        #    self.blah = find_meta_data_files_in_time_order(settings.meta_data_folder)
        #    self.meta_data_files_in_order = iter(find_meta_data_files_in_time_order(settings.meta_data_folder))


    # self.init_range_with_default(range_dic)
    def get_total_number_of_zones(self):
        return self.get_total_number_of_zones

    def populate_zones(self):
        for key in self.cur_game_config.find_all_keys():
            if key in ["Indoor",
                       "GameSetting"]:  # make sure to not touch indoor, cause it'll mess up the keys within it
                continue

            low_bnd = 0

            if key in self.zone_dic:
                number_of_zones = self.zone_dic[key]
                self.total_number_of_zones += number_of_zones
                incr_size = (int)(len(self.game_config_range.get_item(key)) / number_of_zones)
                if (incr_size == 0):  # too many zones, just be happy with increment of 1
                    up_bnd = incr_size = 1
                elif (incr_size == (len(self.game_config_range.get_item(key)))):  # only one zone
                    up_bnd = len(self.game_config_range.get_item(key))
                    incr_size = 0
                else:
                    up_bnd = incr_size
            else:
                number_of_zones = 1
                incr_size = 0
                up_bnd = len(self.game_config_range.get_item(key))

            self.game_config_zones.set_item(key, [low_bnd, up_bnd, incr_size])

            if key == "End":
                assert (len(self.game_config_range.get_item(key)) == number_of_zones), "\
                        number of zones and the number of elements in End range should be the same"

    def set_items_without_modifying_json(self, *arg):
        for el in arg:
            assert (type(
                el) is tuple), el + " needs to be tuple, i.e. the input needs to be provided in the form of (key, new_value)"
            key = el[0]
            value = el[1]
            assert (key in self.cur_game_config.find_all_keys()), key + " is not a key in the json file"
            self.cur_game_config.set_item(key, value)

    def update_json(self, *arg):
        self.set_items_without_modifying_json(*arg)
        outputfile = self.input_file_addr
        output_file_handle = open(outputfile, "w")
        json.dump(self.cur_game_config.config_data, output_file_handle)
        output_file_handle.close()

    def get_cur_item(self, key):
        return self.cur_game_config.get_item(key)

    def set_range(self, *arg):
        for el in arg:
            assert (type(el) is tuple), str(
                el) + " needs to be tuple, i.e. the input needs to be provided in the form of (key, range)"
            assert (type(el[1]) is list), str(
                el) + " needs to be list, i.e. the range needs to be provided in the form of [lower_bound,..., upper_bound]"
            key = el[0]
            value = el[1]
            assert (key in self.cur_game_config.find_all_keys()), key + " is not a key in the json file"
            self.game_config_range.set_item(key, value)

    def get_range(self, key):
        return self.game_config_range.get_item(key)

    # sampling within the entire range
    def sample(self, *arg):
        all_keys = self.game_config_range.find_all_keys()
        if (len(arg) == 0):
            arg = all_keys

        for el in arg:
            assert (el in all_keys), str(el) + " is not a key in the json file"

            # corner cases
            if el in ["Indoor", "GameSetting"]:  # make sure to not touch indoor, cause it'll mess up the keys within it
                continue
            # print(el + str(self.game_config_zones.get_item(el)))
            low_bnd = self.game_config_zones.get_item(el)[0]
            up_bnd = self.game_config_zones.get_item(el)[1]

            range_val = self.game_config_range.get_item(el)[low_bnd:up_bnd]
            random_val = random.choice(range_val)
            self.cur_game_config.set_item(el, random_val)

        # end
        if "End" in arg and self.game_config_range.get_item("End")[0] == "Mutable":
            self.cur_game_config.set_item("End",
                                                _get_random_end_point( \
                                              self.cur_game_config.get_item("ArenaSize"), \
                                              self.game_config_zones.get_item("End")[0], \
                                              self.zone_dic["End"]))

        outputfile = self.input_file_addr
        output_file_handle = open(outputfile, "w")
        json.dump(self.cur_game_config.config_data, output_file_handle)
        output_file_handle.close()
        #if not( settings.ip == '127.0.0.1'):
        #    utils.copy_json_to_server(outputfile)
        #    if(settings.use_preloaded_json):
        #        outputfile = next(self.meta_data_files_in_order)
        #    utils.copy_json_to_server(outputfile)

        #if(settings.use_preloaded_json):
        #    outputfile = next(self.meta_data_files_in_order)
        #    utils.copy_json_to_server(outputfile)
    def increment_zone(self, key):
        low_bnd = self.game_config_zones.get_item(key)[0]
        up_bnd = self.game_config_zones.get_item(key)[1]
        incr_size = self.game_config_zones.get_item(key)[2]
        self.game_config_zones.set_item(key, [ \
            min(low_bnd + incr_size, len(self.game_config_range.get_item(key)) - incr_size),
            min(up_bnd + incr_size, len(self.game_config_range.get_item(key))), incr_size])

    def update_zone(self, *arg):
        all_keys = self.game_config_range.find_all_keys()
        if (len(arg) == 0):
            return

        for el in arg:
            assert (el in all_keys), str(el) + " is not a key in the json file"

            # corner cases
            if el in ["Indoor", "GameSetting"]:  # make sure to not touch indoor, cause it'll mess up the keys within it
                continue
            self.increment_zone(el)
