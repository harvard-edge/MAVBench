import os
import subprocess
import sys
def get_pids(node_list):
    result = {} 
    for node in node_list:
        p = subprocess.Popen(["rosnode", "info",node], stdout=subprocess.PIPE)
        info = (p.stdout.read()).split("\n")
        for el in info:
            if "Pid:" in el:
                result[node] = el.split(":")[1]
                break
    return result

def bind(node_pic_dic):
    for node in node_pid_dic.keys():
        if node == "/airsim_imgPublisher":
            os.system("taskset -p -c " + " 1 " + str(node_pic_dic[node]))
        elif node == "/rviz":
            os.system("taskset -p -c " + " 2 " + str(node_pic_dic[node]))
        else:
            os.system("taskset -p -c " + "0,3,4,5 " + str(node_pic_dic[node]))


p = subprocess.Popen(["rosnode", "list"], stdout=subprocess.PIPE)
node_list = p.stdout.read().split("\n")
node_pid_dic = get_pids(node_list)
bind(node_pid_dic)

#print blah
