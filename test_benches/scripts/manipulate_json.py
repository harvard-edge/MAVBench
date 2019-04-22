import os 
from os import sys
from data_clct_conf_class import *
#from scp import SCPClient
#import paramiko
import sys
from shutil import copy 
import time
import traceback
import json

#data_clct_conf_file_addr = "stats.json"
def generate_csv(data_clct_conf_file_addr):
    data_clct_conf_obj = DataClctConf(data_clct_conf_file_addr) 
    data = data_clct_conf_obj.get_config_data()
    """  
    if not 'experiments_0' in data:
        print 'Cannot find 1st experiment named experiments_0. Json file might be broken.'
        exit(0)

    if len(data.get('experiments_0')) <= 0:
        print 'experiments_0 does not have valid data. Json file might be broken.'
        exit(0)
    """
    
    stat_keys = data.get(data.keys()[0])[0].keys()
    try: 
        stat_keys.remove('topic_statistics')
    except:
        print "topic_stats"
    stat_keys.sort()
    fstring = ''
    fstring = fstring + ','.join(stat_keys) + '\n'
    print ','.join(stat_keys)

    for experiment_key in data:
        #print json.dumps(data.get(experiments))
        experiments = data.get(experiment_key)
        for experiment in experiments:
            line = []
            for k in stat_keys:
                if k in experiment:
                    line.append(str(experiment[k]))
                else:
                    line.append('NaN')
            print ','.join(line)
            fstring = fstring + ','.join(line) + '\n'
        print "---"
        pass
    
    #stat_file_addr = data_clct_conf_obj.get_config_data()["mav_bench_dir"] + "data/"+ data_clct_conf_obj.get_config_data()["application"]+ "/"+"stats.csv"
    stat_file_addr = "stats.csv"

    #--- dump the dictionary in a csv file
    with open(stat_file_addr, 'w') as file:
        file.write(fstring)
    return

#combine the two jsons and write it into json1
def combine_json(file_list):
    head = []
    result = {}
    with open("result.json", "w") as outfile:
        for f in file_list:
            with open(f, 'rb') as infile:
                file_data = json.load(infile)
                head = head + (file_data.values()[0])
                main_key = file_data.keys()[0] 

        result[main_key] = head;
        json.dump(result, outfile)
    copy("result.json", file_list[0])
        

def main():
    #copy first    
#    combine_json(["../config/test_data.json", "../config/test_data2.json"])
    generate_csv("./stats.json")
        #sys.exit(0) 
        #copy back    
    #remove all the temps

if __name__ == "__main__":
    main()

