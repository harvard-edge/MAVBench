#!/bin/bash  
cd /usr/local/lib/
ls -lrt|grep pcl|awk '{print $9}'|xargs -I '{}' cp {} ../../lib/aarch64-linux-gnu/

