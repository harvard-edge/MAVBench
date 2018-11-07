Various mission metrics such mission-time, energy, etc. and compute metrics such as various processes' throughput can be profiled using our toolset. This document descirbes steps toward this end. 

1. cd MAVBench_base_dir/build-scripts
> host_setup_env_var.cmd;   
 cd MAVBench_base_dir/test_benches/configs
 
 2. Modify the config file as you like (provide a link here as to the descriptions of the variables in jason file)
 >  python loader\clct_data.py --config configs\${your_config_file}  (example hellworld-config.json)

# Interpreting the Results
The results will be saved in the data/$pkg_name in a jason file/format that is
sef explanatory.

Note: For follow the leader (you can trigger the person (leader) to start moving by pressing r. This time can also be set using
the config file)


