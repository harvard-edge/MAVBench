import roborun_settings 
import AirSimClient as airsim

import numpy as np
import os
import tempfile
import pprint
import time
from RoborunRandomizer import RoborunRandomizer

client = airsim.MultirotorClient(ip="127.0.0.1")
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
print(client.simGetPose())

exit(0)

env_rand = RoborunRandomizer()
env_rand.init_env_gen()

client = env_rand.get_airsim_client()

airsim.AirSimClientBase.wait_key('Press any key to start')

x = 0
try:
    while (1):
        x += 1
        env_rand.randomize_env()
        print("Resetting...")
        client = env_rand.airsim_reset()
        print("Done reset")
        state = client.getMultirotorState()
        s = pprint.pformat(state)
        print("state: %s" % s)
        airsim.AirSimClientBase.wait_key('Press any key to move')
#        client.moveByVelocity(0, 0, -2, 2, drivetrain=0)
        airsim.AirSimClientBase.wait_key('Press any key to randomize')
#
#        #if x >= 3:
#        #    env_rand.game_config_handler.decrement_zone("SpreadOfObjects")
#        #else:
#        #    env_rand.game_config_handler.increment_zone("SpreadOfObjects")
#
#        #env_rand.ease_randomization()

except(KeyboardInterrupt):
    pass
#
exit(0)
