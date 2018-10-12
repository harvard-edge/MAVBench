#import spur
import os 
#import win32gui, win32con
import time
from os import sys
from data_clct_conf_class import *
from control_unreal import *
import traceback
import signal
from scp import SCPClient
import paramiko
import sys
from shutil import copy 
import time
from multiprocessing import Process

import argparse
parser = argparse.ArgumentParser(description='DARwing collect data.')
parser.add_argument('--config', metavar='c', type=str,
                    default="..\config\data_clct_conf.json",
                    help='config json file path')

args = parser.parse_args()
data_clct_conf_file_addr = args.config


def creat_ssh_client(user_setting):
    
    # paramiko
    ssh_client=paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    ssh_client.connect(user_setting["host_to_cnct_to"],
            22,
            user_setting["usr_name"],
            user_setting["pass_code"])
    return ssh_client 

def start_unreal(user_setting):
    game_path =  user_setting["game_path"]
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

def get_supervisor_cmd(user_setting, experiment_setting):
   mav_bench_dir = user_setting["mav_bench_dir"]
   termination =  experiment_setting["max_run_time"]
   app = experiment_setting["application"]
   return  "python "+\
            mav_bench_dir+"run_time/supervisor.py" +\
            " " + mav_bench_dir  +\
            " " + app +\
            " " +  str(termination)


def get_pre_mission_cmd(application):
    return "./catkin_ws/src/mav-bench/pre_mission/"+application+"/pre_mission_cmds.sh"

def check_start_moving(user_setting, experiment_setting):
    time_to_move = False
    file_to_write_to =  user_setting["AirSim_dir"]+ "\\"+ "companion_comp_msgs.txt"
    time.sleep(12)
    file_to_write_to_handle.open(file_to_write_to, "w")
    file_to_write_to_handle.close()
    sys.exit(0) 
    """ 
    while(not(time_to_move)):
        stdout = ssh_client.exec_command("python " + user_setting["mav_bench_dir"]+
                "/misc/check_start_moving.py")
        print stdout 
        if stdout == "true":
            time_to_move = True
            file_to_write_to_handle.open(file_to_write_to, "w")
            file_to_write_to_handle.write("move")
        time.sleep(2)
    """

def signal_start_moving(file_to_write_to):
    time.sleep(130)
    open(file_to_write_to, "w").close()

def get_bind_node_cmd(platform):
    if (platform == "tx2"): 
        return "python ./catkin_ws/src/mav-bench/run_time/bind_nodes.py"
    else:
        return "echo hello"

def schedule_tasks(user_setting, experiment_setting, ssh_client):
    #--- cmds to schedul e
    src_ros_cmd = "source " + user_setting["catkin_dir"]+"/devel/setup.bash"
    ros_launch_cmd = get_ros_cmd(experiment_setting) 
    run_time_supervisor_cmd = get_supervisor_cmd(user_setting, experiment_setting)
    #run_time_supervisor_cmd = ""#get_supervisor_cmd(user_setting, experiment_setting)
    pre_mission_cmds = get_pre_mission_cmd(experiment_setting["application"])
    platform = user_setting["platform"] 
    #all_cmds = src_ros_cmd + ";" + run_time_supervisor_cmd + "& " +  pre_mission_cmds +  "|" + ros_launch_cmd  + "|" + get_bind_node_cmd(platform)
    all_cmds = src_ros_cmd + ";" + run_time_supervisor_cmd + "& " +  pre_mission_cmds +  "|" + ros_launch_cmd
    p = Process(target= signal_start_moving, args=(user_setting["AirSim_dir"]+ "\\"+ "companion_comp_msgs.txt",))
    if (experiment_setting["application"] == "follow_the_leader"):
        p.start()
    #--- pramiko
    stdin,stdout,stderr= ssh_client.exec_command(all_cmds, get_pty=True)
    outlines = stdout.readlines() 
    result=''.join(outlines)
    
    print(result)
    
    if (experiment_setting["application"] == "follow_the_leader"):
        p.join()
    # errlines = stderr.readlines() 
    # resp_err=''.join(errlines)
    # print(resp_err)
    return result
"""
def copy_results_over(data_clct_conf_obj, ssh_client):
   mav_bench_dir = data_clct_conf_obj.get_config_data()["mav_bench_dir"]
   application = data_clct_conf_obj.get_config_data()["application"]
   stats_file_name_on_comp_computer = data_clct_conf_obj.get_config_data()["stats_file_on_comp_computer"]
   stats_dir_on_host = data_clct_conf_obj.get_config_data()["stats_dir_on_host"]
   data_addr = mav_bench_dir +"/data/"+application+"/" + stats_file_name_on_comp_computer
   
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

def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
	stop_unreal()
	sys.exit(0)

def write_to_stats_file(stat_file_addr, string_to_write, user_setting, ssh_client):
        python_file_to_run = user_setting["mav_bench_dir"]+ "common/python_files/write_to_file.py"
        cmd = "python" + " " + python_file_to_run + " " + stat_file_addr + " " + string_to_write
        stdin,stdout,stderr= ssh_client.exec_command(cmd, get_pty=True)
        outlines = stdout.readlines() 
        result=''.join(outlines)
        print(result)
        # errlines = stderr.readlines() 
        # resp_err=''.join(errlines)
        # print(resp_err)
        return result

def modify_freq(freq, ssh_client, num_of_core=6):
    print freq
    #stdin,stdout,stderr= ssh_client.exec_command("echo nvidia | sudo -S python /home/nvidia/catkin_ws/src/mav-bench/misc/assign_all.py " + str(freq), get_pty=True)
    stdin,stdout,stderr= ssh_client.exec_command("echo nvidia | sudo -S python /home/nvidia/catkin_ws/src/mav-bench/misc/setup_system.py " +  str(num_of_core) + " " + str(freq), get_pty=True)
    #we need the following two statement to make it block, otherwise it won;t
    # have an effect for some reason
    outlines = stdout.readlines() 
    result=''.join(outlines)
    #print(result)
def get_v_max(n_core, freq):
    ctr = 0 
    v_max_l = [1.0, 1.2, 1.3, 1.5, 1.6, 1.7, 1.8, 2.0, 2.2]
    for freq_el in [806400, 1574400, 2035200]:
        for core_el in [2,3,4]:
            if (n_core == core_el) and (freq == freq_el):
                return v_max_l[ctr]
            ctr +=1
    
def main():
    try:
        data_clct_conf_obj = DataClctConf(data_clct_conf_file_addr) #parse config file and instantiate a 
        user_setting =  data_clct_conf_obj.get_config_data()["user_setting"]
        experiment_setting_list =  data_clct_conf_obj.get_config_data()["experiment_setting_list"]
        total_run_ctr = 0
        experiment_set_ctr = 0 
        #write_to_stats_file(stat_file_addr, "{",  user_setting, ssh_client)
        #--- removing the file that triggers the char (only usefull for follow_the_leader) 
        """ 
        try: 
            os.remove(user_setting["AirSim_dir"]+ "\\"+ "companion_comp_msgs.txt")
        except: 
            print "companion_com_msg doesn't exist to remove. This might be ok"
        """
        time.sleep(3) #there needs to be a sleep between restart and change_level
        
        for  experiment_setting in  experiment_setting_list:
            for n_core in [4,3,2] :
                for freq in [2035200, 1574400, 806400]:
                    num_of_runs = experiment_setting["number_of_runs"]
                    application = experiment_setting["application"]
                    ros_params =  experiment_setting["ros_params"]
                    if (application == "mapping" or application == "sar"):
                        ros_params["v_max"] = get_v_max(n_core, freq)
                    #proc_freq = experiment_setting["processor_frequency"]
                    proc_freq = freq
                    num_of_core = n_core
                    stat_file_addr = user_setting["mav_bench_dir"]+"data/"+application+ "/"+"stats.json"
                    
                    try: 
                            os.remove(user_setting["AirSim_dir"]+ "\\"+ "companion_comp_msgs.txt")
                    except:
                        print "companion_com_msg doesn't exist to remove. This might be ok"
                    if ("map_name" in  experiment_setting.keys()):
                        change_level(experiment_setting["map_name"])
                    else:
                        restart_unreal()
                    
                    #start_unreal(user_setting)
                    ssh_client = creat_ssh_client(user_setting)     
                    modify_freq(proc_freq, ssh_client, num_of_core) 
                    
                    #--- preparting the result file 
                    write_to_stats_file(stat_file_addr, '\t\\"experiment_set_'+ str(experiment_set_ctr) +'\\":',  user_setting, ssh_client)
                    write_to_stats_file(stat_file_addr, '[',  user_setting, ssh_client)
                    experiment_set_ctr +=1 
                    #minimize_the_window()
                    
                    #--- start collecting data 
                    for  experiment_run_ctr  in range(0, num_of_runs):
                        total_run_ctr += 1
                        result = schedule_tasks(user_setting, experiment_setting, ssh_client)
                        
                        try: 
                            os.remove(user_setting["AirSim_dir"]+ "\\"+ "companion_comp_msgs.txt")
                        except:
                            print "companion_com_msg doesn't exist to remove. This might be ok"
                        restart_unreal()
                        time.sleep(3) #there needs to be a sleep between restart and change_level
                        write_to_stats_file(stat_file_addr, '\t'+'\\"app\\":\\"'+str(application)+'\\",',  user_setting, ssh_client)
                        write_to_stats_file(stat_file_addr, '\t'+'\\"processor_freq\\":\\"'+str(proc_freq)+'\\",',  user_setting, ssh_client)
                        for  param in ros_params.keys():
                            write_to_stats_file(stat_file_addr, '\t\\"'+param + '\\":\\"'+str(ros_params[param])+'\\",',  user_setting, ssh_client)
                        
                        write_to_stats_file(stat_file_addr, '\t\\"experiment_number\\":'+str(total_run_ctr)+',',  user_setting, ssh_client)
                        write_to_stats_file(stat_file_addr, '\t\\"num_of_cores\\":'+str(num_of_core),  user_setting, ssh_client)
                        if (experiment_run_ctr < num_of_runs - 1): 
                            write_to_stats_file(stat_file_addr, "},",  user_setting, ssh_client)
                    
                    write_to_stats_file(stat_file_addr, "}],",  user_setting, ssh_client)
                #stop_unreal() 
                #write_to_stats_file(stat_file_addr, '\\"experiment_number\\":'+str(experiment_run_ctr)+"}",  user_setting, ssh_client)
                #write_to_stats_file(stat_file_addr, "]}",  user_setting, ssh_client)
    except Exception as e:
        pass
        print(traceback.format_exception(*sys.exc_info()))
        #stop_unreal()

if __name__ == "__main__":
    main()
