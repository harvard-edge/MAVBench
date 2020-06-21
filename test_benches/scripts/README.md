## Setting up env-gen

[Link](https://drive.google.com/file/d/1Qh-5oBYvGOKhFwZdKVG2NhdDHXOugzZj/view?usp=sharing) 
to current exe file. You can also build/modify it from scratch from the
[airlearning-ue4](https://github.com/harvard-edge/airlearning-ue4/tree/gaussian-env-gen) 
repository and our fork of [AirSim](https://github.com/RadhikaG/AirSim/tree/env-gen).

Please make the following changes:

* `clct_data.py`: Change `game_path` in the `start_unreal` function to the env-gen exe file.
* `PythonClient/environment_randomization/roborun_settings.py`: Change `json_file_addr` to the JSON file location in the env-gen exe file tree.


See `../configs/randomized_config.json` for a sample experiment run. Also, 
it takes some time to change experiments and start the next experiment in the 
run (env-gen takes some time to settle down in UE4).
