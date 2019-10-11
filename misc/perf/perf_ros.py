import psutil
procObjList = [procObj for procObj in psutil.process_iter()]

#collect ros processes 
def collect_procs():
    proc_name_pid = {} 
    for proc in psutil.process_iter():
     pinfo = proc.as_dict(attrs=['pid', 'name', 'cmdline'])
     #print pinfo['cmdline'] 
     for arg in pinfo['cmdline']:
         #print arg 
         #print arg.split("/") 
         for arg_split in arg.split("/"):
             #print arg_split 
             if arg_split == ".ros":
                 print pinfo['name'] 
             #break 
             #name_shortened = pinfo['name'].split("/")[-1]
             #print name_shortened 


collect_procs()
#         proc_name_pid[     
#                 print pinfo['pid']
#
#./perf stat -e cycles 
