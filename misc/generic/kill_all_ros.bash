#!/bin/bash
ps aux|grep --color=auto ros |awk  '{print $2}'|xargs -I '{}' kill -KILL {}
