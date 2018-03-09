from __future__ import print_function
import os

print("acceptable freqencies to assign")
available_freqs = [345600, 499200, 652800, 806400, 960000, 1113600, 1267200,\
        1420800, 1574400, 1728000, 1881600, 2035200]






while (True):
    print("acceptable cmds:")   
    print("quit, print current, print available")
    cmd = raw_input("cmd to exec: ")
    if cmd == "quit":
        break;
    elif cmd.split()[0] == "print" and cmd.split()[1] == "current":
        for ctr in range(0, 6):
            os.system("sudo cat /sys/devices/system/cpu/cpu"+str(ctr)+"/cpufreq/cpuinfo_cur_freq")
    
    elif cmd.split()[0] == "print" and cmd.split()[1] == "available":
        print(available_freqs)
    else:
        print("this cmd is not defined. Do you know how to read?")
    """
    elif len(cmd.split()) == 3:
        if cmd.split()[0] == "set" and cmd.split()[1] == "all":
            freq = cmd.split()[2] 
            print(freq)
            for ctr in range(0, 6):
                os.system("echo " + str(freq) + " | sudo dd of=/sys/devices/system/cpu/cpu"+\
                        str(ctr)+"/cpufreq/scaling_setspeed")
                os.system("echo " + str(freq) + " | sudo dd of=/sys/devices/system/cpu/cpu"+\
                        str(ctr)+"/cpufreq/scaling_max_freq")
                os.system("echo " + str(freq) + " | sudo dd of=/sys/devices/system/cpu/cpu"+\
                        str(ctr)+"/cpufreq/scaling_min_freq")
    """
