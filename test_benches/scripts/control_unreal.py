import os
from subprocess import call, Popen, PIPE
import sys

# for env-gen support
file_path=sys.argv[0]
file_dir=os.path.dirname(file_path)
file_dir_abs_path = os.path.abspath(file_dir+"\\..\\..\\")

airsim_py_path = os.path.abspath("./PythonClient")
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
