import os
file_to_write_to = "C:\\Users\\Behzad\\Documents\\AirSim\\companion_comp_msgs.txt"

try:
    os.remove(file_to_write_to)
except:
    print "file not found"
blah = raw_input("tell me when to terminate")
open(file_to_write_to, "w").close()
