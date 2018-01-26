import os
import time
import sys

def action_upon_termination():
    #--- kill all the ros processes 
    os.system("kill -INT $(ps aux | grep ros | awk '{print $2}')")

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
    #sys.argv[1] time based or not
    #sys.argv[2] sleep_time_before checking (optinal)
    SLEEP_TIME_BEFORE_CHECKING = 2 
    assert(len(sys.argv) >= 3)
    stat_file = sys.argv[1]+"data/stats.txt"
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
