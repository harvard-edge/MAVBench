from __future__ import print_function
import os
import sys
print("acceptable freqencies to assign")
available_freqs = [345600, 499200, 652800, 806400, 960000, 1113600, 1267200,\
        1420800, 1574400, 1728000, 1881600, 2035200]

freq = int(sys.argv[1])
assert(freq in available_freqs)

for ctr in range(0, 6):
    os.system("echo " + str(freq) + " | sudo dd of=/sys/devices/system/cpu/cpu"+\
            str(ctr)+"/cpufreq/scaling_setspeed")
    os.system("echo " + str(freq) + " | sudo dd of=/sys/devices/system/cpu/cpu"+\
            str(ctr)+"/cpufreq/scaling_max_freq")
    os.system("echo " + str(freq) + " | sudo dd of=/sys/devices/system/cpu/cpu"+\
            str(ctr)+"/cpufreq/scaling_min_freq")




