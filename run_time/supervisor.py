import os
import time
import sys
import subprocess

def action_upon_termination():
    ctr = 0
    ros_node_pid_list = []
    process_ignore_list = ["stats_manager", "rosmaster", "rosout"]
    
    while(True): #repeately ask for killing of the processes
        ros_process_list = (subprocess.check_output("ps aux | grep ros |grep log", shell=True)).splitlines()
        ros_process_list_filtered = ros_process_list[:]
        
        for process in ros_process_list:  #filter the list
            for ignore_el in process_ignore_list:
                if ignore_el in process:
                    ros_process_list_filtered.remove(process)
                    break

        # block till all processes except the ones in the process ignore_list are killed
        for process in ros_process_list_filtered:
            pid = process.split()[1] 
            subprocess.Popen("kill -INT $(ps aux | grep "+ pid + " | awk '{print $2}')", shell=True)
            time.sleep(.3) 
        time.sleep(3) 
        
        if(len(ros_process_list_filtered)<= len(process_ignore_list) -1):#rosmaster doesn't show up
            break
        else:
            if (ctr > 5): #a time out
                break
        ctr+=1

    #ros_process_list_filtered = (subprocess.check_output(['rosnode', 'list'])).splitlines()
    #time.sleep(2) 
    #if (ctr > 5): #a time out
    #    break
    
    # to kill all other processes (including the rosmaster and core and rosout), the
    # easiest way is to kill the roslaunch itself
    roslaunch_process = (subprocess.check_output("ps aux | grep roslaunch |grep opt", shell=True)).splitlines()[0]
    pid = roslaunch_process.split()[1] 
    #--- using call cause it's blocking 
    subprocess.call("kill -INT $(ps aux | grep "+ pid + " | awk '{print $2}')", shell=True)

def terminate(stat_file):
    try:
        stat_f_hndlr = open(stat_file, "r")
    except IOError:
        handleIOError(stat_file, "source file error")
        exit()
    else:
        with stat_f_hndlr:
            for line in stat_f_hndlr:
                words = line.split(' ')
                #print words[0].strip()
                if words[0].strip() == "mission_status":
                    stat_f_hndlr.close()
                    return True
            stat_f_hndlr.close()
            return False
                    


def main():
    #action_upon_termination();
    #sys.exit(0)
    #sys.argv[1] time based or not
    #sys.argv[2] sleep_time_before checking (optinal)
    SLEEP_TIME_BEFORE_CHECKING = 2 
    assert(len(sys.argv) >= 3)
    stat_file = sys.argv[1]+"data/package_delivery/stats.txt"
    # --- populating variables 
    if (len(sys.argv) > 3):
        sleep_time_before_checking = sys.argv[3]
    else:
        sleep_time_before_checking = SLEEP_TIME_BEFORE_CHECKING
    time_based = sys.argv[2];  
    stat_f_hndlr = open(stat_file, "w")
    stat_f_hndlr.close()
     
    #---- body
    if (time_based == "True" or time_based == "true" ): #after certain time terminate
        time.sleep(float(sleep_time_before_checking))
        action_upon_termination()
    else: 
        while(True): 
            time.sleep(float(SLEEP_TIME_BEFORE_CHECKING))
            if (terminate(stat_file)): 
                action_upon_termination()
                return
            


if __name__ == "__main__":
    main()
