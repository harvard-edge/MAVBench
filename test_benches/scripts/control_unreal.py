import os
from subprocess import call, Popen, PIPE
import sys

# for env-gen support
file_path=sys.argv[0]
file_dir=os.path.dirname(file_path)
file_dir_abs_path = os.path.abspath(file_dir+"\\..\\..\\")

#airsim_dir_abs_path = os.path.abspath(file_dir_abs_path+"/src/AirSim")
airsim_dir_abs_path = os.path.abspath(file_dir_abs_path+"Users/Behzad-PC/mavbench_stuff/AirSim-my") # for radhika
airsim_py_path = os.path.abspath(airsim_dir_abs_path+"/PythonClient")
os.sys.path.insert(0, airsim_py_path)
env_gen_py_path = os.path.abspath(airsim_py_path+"/environment_randomization")
os.sys.path.insert(0, env_gen_py_path)

#print(os.sys.path)

def messages_dir():
    return os.path.expanduser("~\Documents\AirSim");

def start_game(path, in_editor=False):
    if in_editor:
        print("Not impolemented yet")
    else:
            Popen(path, stdin=PIPE) 
def stop_game():
    f = open(os.path.join(messages_dir(), "exit"), "w");
    f.close()
    
def change_level(level):
    path = os.path.join(messages_dir(), "change_level.txt")
    f = open(path, "w");
    f.write(level)
    f.close()
    try: 
        os.remove(os.path.join(messages_dir(), "change_level"))
    except OSError:
        pass
    os.rename(path, os.path.join(messages_dir(), "change_level"))
    
def restart_level():
    f = open(os.path.join(messages_dir(), "restart"), "w");
    f.close()

# for env-gen

from environment_randomization import EnvRandomizer


def randomize_env(env_rand):
    env_rand.randomize_env()

def randomize_env_difficulty(env_rand, difficulty_level):
    #print("control unreal: " + difficulty_level)
    assert(difficulty_level == "easy" or difficulty_level == "medium"
            or difficulty_level == "hard")
    env_rand.init_difficulty_level(difficulty_level)

def tight_randomization(env_rand):
    env_rand.tight_randomization()
