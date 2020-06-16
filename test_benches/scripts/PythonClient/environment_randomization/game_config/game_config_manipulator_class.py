import settings
import os
from .game_config_class import *

class GameConfigManipulator:
    def __init__(self, input_file_addr=settings.json_file_addr):
        assert(os.path.isfile(input_file_addr)), input_file_addr + " doesnt exist"
        self.input_file_addr = input_file_addr 
        self.game_config = GameConfig(input_file_addr)

    def write_back_json(self):
        outputfile = self.input_file_addr
        output_file_handle = open(outputfile, "w")
        json.dump(self.game_config.config_data, output_file_handle)
        output_file_handle.close()

    def set(self, *arg):
        for el in arg:
            assert (type(el) is tuple), el + " needs to be tuple, i.e. the input needs to be provided in the form of (key, new_value)" 
            key = el[0]
            value = el[1]
            assert(key in self.game_config.find_all_keys()), key + " is not a key in the json file"
            self.game_config.set_item(key, value)

        self.write_back_json()
    
