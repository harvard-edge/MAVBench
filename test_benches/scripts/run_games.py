#import spur
import os 
#import win32gui, win32con
import time
from os import sys
from data_clct_conf_class import *
from control_unreal import *
import traceback
import signal
#from scp import SCPClient
import paramiko
import sys
from shutil import copy 
import time
from multiprocessing import Process
import signal
import sys

import argparse

def get_host_base():
    file_path=sys.argv[0]
    file_dir=os.path.dirname(file_path)
    file_dir_abs_path = os.path.abspath(file_dir+"\\..\\..\\")
    return file_dir_abs_path


parser = argparse.ArgumentParser(description='DARwing collect data.')
parser.add_argument('--config', metavar='c', type=str,
                    default=get_host_base()+"\test_benches\configs\hello-world-config.json",
                    help='config json file path')

args = parser.parse_args()
data_clct_conf_file_addr = args.config
data_clct_conf_obj = DataClctConf(data_clct_conf_file_addr) #parse config file and instantiate a 
companion_setting =  data_clct_conf_obj.get_config_data()["companion_setting"]
host_setting =  data_clct_conf_obj.get_config_data()["host_setting"]

mavbench_apps_base_dir = companion_setting["base_dir"]+"/catkin_ws/src/MAV_apps"


def creat_ssh_client(companion_setting, host_base_dir):
    
    # paramiko
    ssh_client=paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    ssh_client.connect(companion_setting["host_ip"],
            22,
            companion_setting["usr_name"],
            companion_setting["pass_code"])
    return ssh_client 

def start_unreal(host_setting, host_base_dir):
    if host_setting["in_editor"]:
        return
    else:
        game_path = host_base_dir + "\\test_benches\\games\\WindowsNoEditor\\Blocks.exe"
    if not(os.path.isfile(game_path)):
        print("file:" + game_path + " doesn't exist")
        sys.exit()
    
    start_game(game_path); 
    return

   
def get_ros_cmd(experiment_setting):
    application = experiment_setting["application"]
    ros_params = experiment_setting["ros_params"]
    args = "" 
    for key in ros_params.keys():
        args += " " + str(key) + ":=" + str(ros_params[key])
    if (application == "package_delivery"):
        return "roslaunch package_delivery package_delivery.launch" + args
        #return "roslaunch package_delivery y.launch"
    elif (application == "scanning"):
        return "roslaunch package_delivery scanning.launch" + args
    elif (application == "mapping"):
        return "roslaunch mapping_and_sar mapping.launch" + args
    elif (application == "sar"):
        return "roslaunch mapping_and_sar sar.launch" + args
    elif (application == "follow_the_leader"):
        return "roslaunch follow_the_leader follow_the_leader.launch" + args
    else:
        print("this application not defined")
        sys.exit()

def get_supervisor_cmd(companion_setting, experiment_setting):
   termination =  experiment_setting["max_run_time"]
   app = experiment_setting["application"]
   return  "python "+\
            mavbench_apps_base_dir+"/run_time/supervisor.py" +\
            " " + mavbench_apps_base_dir  +\
            " " + app +\
            " " +  str(termination)


def get_pre_mission_cmd(application):
    return mavbench_apps_base_dir+"/pre_mission/"+application+"/pre_mission_cmds.sh"



def signal_start_moving(file_to_write_to):
    time.sleep(30)
    open(file_to_write_to, "w").close()

def get_bind_node_cmd(platform):
    if (platform == "tx2"): 
        return "python  " + mavbench_apps_base_dir+"/run_time/bind_nodes.py"
    else:
        return "echo platform not supported"


def trigger_obj_motion(host_base_dir, experiment_setting):
       trigger_exe_path = host_base_dir + "\\test_benches\\scripts\\trigger.exe"
       p = Popen(trigger_exe_path, stdin=PIPE)
       p.stdin.write(experiment_setting["time_to_trigger_object"])
       #p.communicate()[0]
       time.sleep(.4)
       p.stdin.close()

def schedule_tasks(companion_setting, experiment_setting, ssh_client, host_base_dir):
    if (experiment_setting["application"] == "follow_the_leader"):
        trigger_obj_motion(host_base_dir, experiment_setting)
    #--- cmds to schedul e
    src_ros_cmd = "source " + companion_setting["base_dir"] + "/catkin_ws/devel/setup.sh"
    src_companion_setup= "source " + companion_setting['base_dir'] + "/build_scripts/companion_setup_env_var.sh"
    ros_launch_cmd = get_ros_cmd(experiment_setting) 
    run_time_supervisor_cmd = get_supervisor_cmd(companion_setting, experiment_setting)
    #run_time_supervisor_cmd = ""#get_supervisor_cmd(companion_setting, experiment_setting)
    pre_mission_cmds = get_pre_mission_cmd(experiment_setting["application"])
    platform = companion_setting["platform"] 
    all_cmds = src_ros_cmd + ";" + src_companion_setup + ";" + run_time_supervisor_cmd + "& " +  pre_mission_cmds +  "|" + ros_launch_cmd  + "|" + get_bind_node_cmd(platform)
    #all_cmds = src_ros_cmd + ";" + run_time_supervisor_cmd + "& " +  pre_mission_cmds +  "|" + ros_launch_cmd

        
    #--- pramiko
    stdin,stdout,stderr= ssh_client.exec_command(all_cmds, get_pty=True)
    outlines = stdout.readlines() 
    result=''.join(outlines)
    
    #if (experiment_setting["application"] == "follow_the_leader"):
    #    p.join()
    # errlines = stderr.readlines() 
    # resp_err=''.join(errlines)
    # print(resp_err)
    return result
"""
def copy_results_over(data_clct_conf_obj, ssh_client):
   mavbench_apps_base_dir = data_clct_conf_obj.get_config_data()["mav_bench_dir"]
   application = data_clct_conf_obj.get_config_data()["application"]
   stats_file_name_on_comp_computer = data_clct_conf_obj.get_config_data()["stats_file_on_comp_computer"]
   stats_dir_on_host = data_clct_conf_obj.get_config_data()["stats_dir_on_host"]
   data_addr = mavbench_apps_base_dir +"/data/"+application+"/" + stats_file_name_on_comp_computer
   
   scp_client = SCPClient(ssh_client.get_transport()) 
   scp_client.get(data_addr)
   copy(stats_file_name_on_comp_computer, stats_dir_on_host);  
"""

def restart_unreal():
    restart_level();

def stop_unreal():
    stop_game();

def parse_results(result):
    return 

def minimize_the_window():
    time.sleep(5);
    Minimize = win32gui.GetForegroundWindow()
    win32gui.ShowWindow(Minimize, win32con.SW_MINIMIZE) 

"""
def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        restart_unreal()
        stop_unreal()
	sys.exit(0)
"""

def write_to_stats_file(stat_file_addr, string_to_write, companion_setting, ssh_client):
        python_file_to_run = mavbench_apps_base_dir + "/common/python_files/write_to_file.py"
        cmd = "python" + " " + python_file_to_run + " " + stat_file_addr + " " + string_to_write
        stdin,stdout,stderr= ssh_client.exec_command(cmd, get_pty=True)
        outlines = stdout.readlines() 
        result=''.join(outlines)
        print(result)
        # errlines = stderr.readlines() 
        # resp_err=''.join(errlines)
        # print(resp_err)
        return result


def mk_data_dir(ssh_client):
    cmd = "cd "+mavbench_apps_base_dir + ";" + "mkdir -p data/package_delivery data/scanning data/mapping data/sar data/follow_the_leader" 
    stdin,stdout,stderr= ssh_client.exec_command(cmd, get_pty=True)
    outlines = stdout.readlines() 
    result=''.join(outlines)
    print(result)
    # errlines = stderr.readlines() 
    # resp_err=''.join(errlines)
    # print(resp_err)
    print result 
    return result


def modify_freq(freq, ssh_client):
    print freq
    stdin,stdout,stderr= ssh_client.exec_command("echo nvidia | sudo -S python "+ mavbench_apps_base_dir+"/misc/set_freq_for_all.py " + str(freq), get_pty=True) #we need the following two statement to make it block, otherwise it won;t # have an effect for some reason
    outlines = stdout.readlines() 
    result=''.join(outlines)
    print(result)


def signal_handler(sig, frame):
    restart_unreal(); 
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    host_base_dir= get_host_base(); 
    try:
        #companion_setting =  data_clct_conf_obj.get_config_data()["companion_setting"]
        experiment_setting_list =  data_clct_conf_obj.get_config_data()["experiment_setting_list"]
        time.sleep(3) #there needs to be a sleep between restart and change_level
        start_unreal(host_setting, host_base_dir)
        time.sleep(7) #there needs to be a sleep between restart and change_level
        experiment_setting = experiment_setting_list[0] 
        application = experiment_setting["application"]
        #ros_params =  experiment_setting["ros_params"]
        #stat_file_addr = mavbench_apps_base_dir+"/data/"+application+ "/"+"stats.json"
        if ("map_name" in  experiment_setting.keys()):
            change_level(experiment_setting["map_name"])
        else:
            restart_unreal()
    except Exception as e:
        pass
        print(traceback.format_exception(*sys.exc_info()))

    while(1):
        print("hello") 
        time.sleep(3) #there needs to be a sleep between restart and change_level

if __name__ == "__main__":
    main()
