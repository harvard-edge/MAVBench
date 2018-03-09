import sys
import os
number_of_cores = int(sys.argv[1])
freq = int(sys.argv[2])

assert (number_of_cores <= 6 and number_of_cores>=1);
if (number_of_cores == 6):
    for ctr in range(0, 6):
        os.system("echo " + str(1) + " | sudo dd of=/sys/devices/system/cpu/cpu"+\
                str(ctr)+"/online")
        os.system("echo " + "userspace" + " | sudo dd of=/sys/devices/system/cpu/cpu"+\
                str(ctr)+"/cpufreq/scaling_governor")
else:
    for ctr in range(0, 6):
        if ( ctr == 1 or ctr == 2 or ctr >= number_of_cores+2):
            os.system("echo " + str(0) + " | sudo dd of=/sys/devices/system/cpu/cpu"+\
                str(ctr)+"/online")
        else:
            os.system("echo " + str(1) + " | sudo dd of=/sys/devices/system/cpu/cpu"+\
                    str(ctr)+"/online")
            os.system("echo " + "userspace" + " | sudo dd of=/sys/devices/system/cpu/cpu"+\
                    str(ctr)+"/cpufreq/scaling_governor")

