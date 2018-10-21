# FAQ
### host computer build:
If uppon running host_root_setup.cmd (while in running in "Start x64 Native Tools Command Prompt for VS 2017" shell), you get the following error "The C compiler identification is unknown":
- open up the aformentioned shell in admin mode (right click->Run as an Administrator)
- Proceed to the folder that normaly loads when running "x64 Native Tools Command Prompt for VS 2017" asan user (e.g. C:\Program Files (x86)\Microsoft Visual Studio\2017\Community). start from step 3 again 
Note: that if you want to run as a user, you need to give access to the Community folder (in admin mode) using "icacls "./Community"  /grant $your_user_name:M /T)

Uppon running Unreal, if you get the follow error:

![alt text](https://github.com/MAVBench/MAVBench/blob/master/docs/images/unreal_error.PNG)
- right click the game solution from your solution explorer and select the option "Set as StartUp Project" and that should highlight the game solution. Cntrl-F5 will then work


