#include "ros/ros.h"

#include <iostream>
#include <signal.h>
#include <string>
#include <sstream>
#include <stats_manager/flight_stats_srv.h>
#include "Drone.h"
#include "common.h"

using namespace std;

#ifdef USE_NVML
#include "nvml.h"
#endif  // USE_NVML
#include <cstdio>
#include <chrono>
#include <ctime>

#define MAX_CPUS    1024
#define MAX_PACKAGES    16
#define NUM_RAPL_DOMAINS    4

static int total_cores=0,total_packages=0;
static int package_map[MAX_PACKAGES];

/* TODO: on Skylake, also may support  PSys "platform" domain,    */
/* the whole SoC not just the package.                */
/* see dcee75b3b7f025cc6765e6c92ba0a4e59a4d25f4            */

static int detect_cpu(void) {

    FILE *fff;

    int family,model=-1;
    char buffer[BUFSIZ],*result;
    char vendor[BUFSIZ];

    fff=fopen("/proc/cpuinfo","r");
    if (fff==NULL) return -1;

    while(1) {
        result=fgets(buffer,BUFSIZ,fff);
        if (result==NULL) break;

        if (!strncmp(result,"vendor_id",8)) {
            sscanf(result,"%*s%*s%s",vendor);

            if (strncmp(vendor,"GenuineIntel",12)) {
                printf("%s not an Intel chip\n",vendor);
                return -1;
            }
        }

        if (!strncmp(result,"cpu family",10)) {
            sscanf(result,"%*s%*s%*s%d",&family);
            if (family!=6) {
                printf("Wrong CPU family %d\n",family);
                return -1;
            }
        }

        if (!strncmp(result,"model",5)) {
            sscanf(result,"%*s%*s%d",&model);
        }

    }

    fclose(fff);
    return model;
}

static int detect_packages(void) {

    char filename[BUFSIZ];
    FILE *fff;
    int package;
    int i;

    for(i=0;i<MAX_PACKAGES;i++) package_map[i]=-1;

    // printf("\t");
    for(i=0;i<MAX_CPUS;i++) {
        sprintf(filename,"/sys/devices/system/cpu/cpu%d/topology/physical_package_id",i);
        fff=fopen(filename,"r");
        if (fff==NULL) break;
        fscanf(fff,"%d",&package);
        // printf("%d (%d)",i,package);
        // if (i%8==7) printf("\n\t"); else printf(", ");
        fclose(fff);

        if (package_map[package]==-1) {
            total_packages++;
            package_map[package]=i;
        }

    }

    // printf("\n");

    total_cores=i;

    // printf("\tDetected %d cores in %d packages\n\n",
    //     total_cores,total_packages);

    return 0;
}

struct rapl_sysfs_stats {
	char basename[MAX_PACKAGES][256];
    char event_names[MAX_PACKAGES][NUM_RAPL_DOMAINS][256];
	char filenames[MAX_PACKAGES][NUM_RAPL_DOMAINS][256];
	long long before[MAX_PACKAGES][NUM_RAPL_DOMAINS];
	long long after[MAX_PACKAGES][NUM_RAPL_DOMAINS];
    double energy[MAX_PACKAGES][NUM_RAPL_DOMAINS];
	int valid[MAX_PACKAGES][NUM_RAPL_DOMAINS];
};

static int setup_rapl_sysfs(rapl_sysfs_stats *s) {
	char tempfile[256];
	int i,j;
	FILE *fff;

    int cpu_model;

    memset(s, 0, sizeof(rapl_sysfs_stats));

    cpu_model=detect_cpu();
    if (cpu_model == -1) {
        std::cout << "Cannot detect Intel CPUs.\n";
        return 0;
    }

    detect_packages();
	// printf("\nTrying sysfs powercap interface to gather results\n\n");

	/* /sys/class/powercap/intel-rapl/intel-rapl:0/ */
	/* name has name */
	/* energy_uj has energy */
	/* subdirectories intel-rapl:0:0 intel-rapl:0:1 intel-rapl:0:2 */

	for(j=0;j<total_packages;j++) {
		i=0;
		sprintf(s->basename[j],"/sys/class/powercap/intel-rapl/intel-rapl:%d",
			j);
		sprintf(tempfile,"%s/name",s->basename[j]);
		fff=fopen(tempfile,"r");
		if (fff==NULL) {
			fprintf(stderr,"\tCould not open %s\n",tempfile);
			return -1;
		}
		fscanf(fff,"%s",s->event_names[j][i]);
		s->valid[j][i]=1;
		fclose(fff);
		sprintf(s->filenames[j][i],"%s/energy_uj",s->basename[j]);

		/* Handle subdomains */
		for(i=1;i<NUM_RAPL_DOMAINS;i++) {
			sprintf(tempfile,"%s/intel-rapl:%d:%d/name",
				s->basename[j],j,i-1);
			fff=fopen(tempfile,"r");
			if (fff==NULL) {
				//fprintf(stderr,"\tCould not open %s\n",tempfile);
				s->valid[j][i]=0;
				continue;
			}
			s->valid[j][i]=1;
			fscanf(fff,"%s",s->event_names[j][i]);
			fclose(fff);
			sprintf(s->filenames[j][i],"%s/intel-rapl:%d:%d/energy_uj",
				s->basename[j],j,i-1);

		}
	}

	/* Gather before values */
	for(j=0;j<total_packages;j++) {
		for(i=0;i<NUM_RAPL_DOMAINS;i++) {
			if (s->valid[j][i]) {
				fff=fopen(s->filenames[j][i],"r");
				if (fff==NULL) {
					fprintf(stderr,"\tError opening %s!\n",s->filenames[j][i]);
				}
				else {
					fscanf(fff,"%lld",&s->before[j][i]);
					fclose(fff);
				}
			}
		}
	}

    return 0;
}

static double run_rapl_sysfs(rapl_sysfs_stats *s) {
    long long value = 0;
    int i,j;
	FILE *fff;

    /* Gather after values */
	for(j=0;j<total_packages;j++) {
		for(i=0;i<NUM_RAPL_DOMAINS;i++) {
			if (s->valid[j][i]) {
				fff=fopen(s->filenames[j][i],"r");
				if (fff==NULL) {
					fprintf(stderr,"\tError opening %s!\n",s->filenames[j][i]);
				}
				else {
					fscanf(fff,"%lld",&s->after[j][i]);
					fclose(fff);
				}
			}
		}
	}

    std::string core("core");
    double e = 0;
	for(j=0;j<total_packages;j++) {
		// printf("\tPackage %d\n",j);
		for(i=0;i<NUM_RAPL_DOMAINS;i++) {
			if (s->valid[j][i]) {
                s->energy[j][i] = ((double)s->after[j][i]-(double)s->before[j][i])/1000000.0;
                if (core == s->event_names[j][i]) {
				    e += s->energy[j][i];;
                }
			}
		}
	}

    return e;
}

struct xpu_sample_stat {
    xpu_sample_stat(): running(false), sum(0) {}

    bool running;
    double sum;
    std::chrono::time_point<std::chrono::system_clock> start;
};

double read_gpu_power_sample(xpu_sample_stat *s = nullptr) {
    if (!s->running) return 0;

    unsigned int gpu_power_mwatts = 0;
    std::ifstream f1("/sys/devices/3160000.i2c/i2c-0/0-0040/iio_device/in_power0_input");
    if (f1.good()) {
        f1 >> gpu_power_mwatts;
        // std::cout << "GPU Power (tx2): " << gpu_power_mwatts << " mWatt\n";
    }
    f1.close();

#ifdef USE_NVML
    unsigned int num_devices = 0;
    if (nvmlDeviceGetCount(&num_devices) != NVML_SUCCESS) {
        std::cout << nvmlDeviceGetCount(&num_devices) << "nvmlDeviceGetCount() failed.\n";
        return 0;
    }

    nvmlDevice_t gpu_0;
    if (nvmlDeviceGetHandleByIndex(0, &gpu_0) != NVML_SUCCESS) {
        std::cout << "nvmlDeviceGetHandleByIndex() failed.\n";
        return 0;
    }

    if (nvmlDeviceGetPowerUsage(gpu_0, &gpu_power_mwatts) != NVML_SUCCESS) {
        std::cout << "nvmlDeviceGetPowerUsage() failed.\n";
        return 0;
    }

    // std::cout << "GPU Power (nvml): " << gpu_power_mwatts << " mWatt\n";
#endif  // USE_NVML

    if (gpu_power_mwatts <= 0) return 0;
    if (s == nullptr) return gpu_power_mwatts;

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - s->start;
    auto e = gpu_power_mwatts / 1000.0 * elapsed_seconds.count();
    s->start = end;
    s->sum += e;
    return e;
}

double read_cpu_power_sample(xpu_sample_stat *s = nullptr) {
    if (!s->running) return 0;

    unsigned int cpu_power_mwatts = 0;
    std::ifstream f1("/sys/devices/3160000.i2c/i2c-0/0-0041/iio_device/in_power1_input");
    if (f1.good()) {
        f1 >> cpu_power_mwatts;
        // std::cout << "CPU Power (tx2): " << cpu_power_mwatts << " mWatt\n";
    }
    f1.close();

    if (cpu_power_mwatts <= 0) return 0;
    if (s == nullptr) return cpu_power_mwatts;

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - s->start;
    auto e = cpu_power_mwatts / 1000.0 * elapsed_seconds.count();
    s->start = end;
    s->sum += e;
    return e;
}


string g_ip_addr;
string g_stats_fname;
Drone *g_drone;//ip_addr.c_str(), port);
string g_mission_status = "failed";//ip_addr.c_str(), port);
bool g_end_requested = false;
msr::airlib::FlightStats g_init_stats, g_end_stats;
string g_ns;
uint16_t g_port; 

xpu_sample_stat xs_cpu, xs_gpu;
rapl_sysfs_stats rs;

void initialize_params() {
    if(!ros::param::get("/stats_manager/ip_addr",g_ip_addr)){
        ROS_FATAL_STREAM("Could not start exploration. Parameter missing! Looking for stats_manager/ip_addr");
      return; 
    }
    
    if(!ros::param::get("/stats_file_addr", g_stats_fname)){
        ROS_FATAL("Could not start exploration. Parameter missing! Lookining for stats_manager/stats_file_addr");
      return; 
    }
    g_port = 41451;
    /* 
    if(!ros::param::get("/stats_manager/localization_method",localization_method)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/localization_method").c_str());
       return; 
    }
    if(!ros::param::get("/stats_file_addr",stats_file_addr)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }
    */
}




bool probe_flight_stats_cb(stats_manager::flight_stats_srv::Request &req, stats_manager::flight_stats_srv::Response &res)
{
    
    ROS_ERROR_STREAM("insdie the call back"); 
    if (g_drone == NULL) {
        ROS_ERROR_STREAM("drone object is not initialized");
        return false; 
        //res.acquired = false;
    }

    if(req.key == "clct_init_data"){ 
        g_init_stats = g_drone->getFlightStats();
        xs_gpu.start = xs_cpu.start = std::chrono::system_clock::now();
        xs_gpu.running = xs_cpu.running = true;
        #ifdef USE_INTEL
        setup_rapl_sysfs(&rs);
        #endif // USE_INTEL
    } 
    else if (req.key == "mission_status"){
        if (req.value == 0.0) {
            g_mission_status = "failed";
        }else{
            g_mission_status = "completed";
        }
    }
    else {
        ROS_ERROR_STREAM("this key is not defined for flight_stats collections"); 
        return false; 
        //res.acquired = false;
    }
    /* 
    else if(req.phase == "clct_end_data"){ 
        g_end_requested = true; 
        g_end_stats = g_drone->getFlightStats();
        g_mission_status = req.mission_status;
    }
    else {
        ROS_ERROR_STREAM("this phase is not defined for flight_stats collections"); 
        return false; 
        //res.acquired = false;
    }
     
    else if(req.phase == "register_mission_status"){ 
        g_mission_status = req.mission_status;
    }
   */  
    return true; 
    //res.acquired = true;
}



// *** F:DN main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "stats_manager");
    ros::NodeHandle nh;
    ros::ServiceServer probe_flight_stats_service = nh.advertiseService("probe_flight_stats", probe_flight_stats_cb);
    initialize_params();
    g_drone = new Drone(g_ip_addr.c_str(), g_port);

#ifdef USE_NVML
    if (nvmlInit() != NVML_SUCCESS) {
        std::cout << "nvmlInit() failed.\n";
        return 0;
    }
#endif

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        read_gpu_power_sample(&xs_gpu);
        #ifndef USE_INTEL
        read_cpu_power_sample(&xs_cpu);
        #endif // NOT USE_INTEL
        loop_rate.sleep();
        ros::spinOnce();
    }

#ifdef USE_NVML
    nvmlShutdown();
#endif  // USE_NVML

    double cpu_compute_enenrgy = 0, gpu_compute_enenrgy = 0;
    gpu_compute_enenrgy = xs_gpu.sum;
    #ifdef USE_INTEL
    cpu_compute_enenrgy = run_rapl_sysfs(&rs);
    #else
    cpu_compute_enenrgy = xs_cpu.sum;
    #endif // USE_INTEL
    
    g_end_stats = g_drone->getFlightStats();
    output_flight_summary(g_init_stats, g_end_stats, g_mission_status,
                          cpu_compute_enenrgy, gpu_compute_enenrgy, g_stats_fname);
    //output_flight_summary(drone, stats_fname);
    return 0;
}

