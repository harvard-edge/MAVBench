set ROOT_DIR=%~dp0
chdir %ROOT_DIR%..\src\AirSim
REM build.cmd
REM powershell -command "& { [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; iwr https://drive.google.com/file/d/1dbJzjN_rH5wXW94eqF6E1eXyL3nPYW65/view -OutFile ./Unreal/Plugins/AirSim/Content/HUDAssets/DepthMapMaterial.uasse}"

powershell -command "& { [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; iwr https://drive.google.com/file/d/1dbJzjN_rH5wXW94eqF6E1eXyL3nPYW65/view -OutFile .\Unreal\Plugins\AirSim\Content\HUDAssets\DepthMapMaterial.uasset}" 

chdir %ROOT_DIR%

REM cp -r $AirSim_base_dir/Unreal/Plugins $AirSim_base_dir/Unreal/Environments/Blocks
