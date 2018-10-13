set ROOT_DIR=%~dp0
chdir %ROOT_DIR%..\src\AirSim
build.cmd
set depth_map_src="%ROOT_DIR%DepthMapMaterial.uasset"
set depth_map_des="%ROOT_DIR%..\src\AirSim\Unreal\Plugins\AirSim\Content\HUDAssets"
copy %depth_map_src% %depth_map_des%
chdir %ROOT_DIR%..\

