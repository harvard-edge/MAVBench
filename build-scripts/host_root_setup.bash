#! /bin/bash

env_dir="$(dirname $( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd))"
source ${env_dir}/build-scripts/setup_env_var.sh
set -x 
set -e 

cd $AirSim_base_dir &&\
    ./setup.sh && \
    ./build.sh
cd $AirSim_base_dir
wget https://drive.google.com/file/d/1dbJzjN_rH5wXW94eqF6E1eXyL3nPYW65/view
cp DpethMapMaterial.uasset $AirSim_base_dir/Unreal/Plugins/AirSim/Content/HUDAssets/
cp -r $AirSim_base_dir/Unreal/Plugins $AirSim_base_dir/Unreal/Environments/Blocks
