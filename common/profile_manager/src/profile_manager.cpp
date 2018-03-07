#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include <signal.h>
#include <string>
#include <sstream>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include <rosgraph_msgs/TopicStatistics.h>
#include "Drone.h"
#include "common.h"
#include <map>
using namespace std;

#ifdef USE_NVML
#include "nvml.h"
#endif  // USE_NVML
#include <cstdio>
#include <chrono>
#include <ctime>

//globla variables
string g_ip_addr;
string g_stats_fname;
Drone *g_drone;//ip_addr.c_str(), port);
string g_mission_status = "failed";//ip_addr.c_str(), port);
string g_localization_status = "healthy";//ip_addr.c_str(), port);
float g_coverage = 0;
bool g_end_requested = false;
msr::airlib::FlightStats g_init_stats, g_end_stats;
string g_ns;
uint16_t g_port; 

//profiling variable
vector<KeyValuePairStruct> g_highlevel_application_stats;
std::map <std::string, statsStruct> g_topics_stats;
bool g_start_profiling_data = false;

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
	// printf("\nTrying sysfs powercap interface to gather g_highlevel_application_stats\n\n");

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


xpu_sample_stat xs_cpu, xs_gpu;
rapl_sysfs_stats rs;


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



void initialize_params() {
    if(!ros::param::get("/profile_manager/ip_addr",g_ip_addr)){
        ROS_FATAL_STREAM("Could not start exploration. Parameter missing! Looking for profile_manager/ip_addr");
      return; 
    }
    
    if(!ros::param::get("/stats_file_addr", g_stats_fname)){
        ROS_FATAL("Could not start exploration. Parameter missing! Lookining for profile_manager/stats_file_addr");
      return; 
    }
    g_port = 41451;
    /* 
    if(!ros::param::get("/profile_manager/localization_method",localization_method)){
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

void output_flight_summary(void){
    //msr::airlib::FlightStats init, msr::airlib::FlightStats end, 
    //   vector<KeyValuePairStruct>g_highlevel_application_stats,const std::string& fname){
    stringstream stats_ss;
    float total_energy_consumed = 0;
    
    //Flight Stats dependent metrics
    stats_ss << "{"<<endl;
    stats_ss << "\t\"distance_travelled\": " << g_end_stats.distance_traveled - g_init_stats.distance_traveled<< "," << endl;
    stats_ss << "\t\"flight_time\": " << g_end_stats.flight_time - g_init_stats.flight_time<< "," << endl;
    stats_ss << "\t\"collision_count\": " << g_end_stats.collision_count  - g_init_stats.collision_count << "," << endl;
    
    stats_ss << "\t\"initial_voltage\": " << g_init_stats.voltage << "," << endl;
    stats_ss << "\t\"end_voltage\": " << g_end_stats.voltage << "," << endl;
    stats_ss << "\t\"StateOfCharge\": " << 100 - (g_init_stats.state_of_charge  - g_end_stats.state_of_charge) << "," << endl;
    stats_ss << "\t\"rotor energy consumed \": " << g_end_stats.energy_consumed - g_init_stats.energy_consumed << ","<<endl; 

    // the rest of metrics 
    for (auto result_el: g_highlevel_application_stats) {
        stats_ss<<  "\t\""<< result_el.key<<'"' <<": " << result_el.value<<"," << endl;
        if(result_el.key == "gpu_compute_energy" || result_el.key == "cpu_compute_energy"){
            total_energy_consumed += result_el.value; 
        }
    }
    total_energy_consumed +=  (g_end_stats.energy_consumed - g_init_stats.energy_consumed);
    stats_ss << "\t\"" <<"total_energy_consumed"<<'"'<<":" << total_energy_consumed << "," << endl;
    // topic rates
    stats_ss << "\t\""  <<"topic_statistics"<<'"'<<":{" << endl;
    for (auto it = std::begin(g_topics_stats); it !=std::end(g_topics_stats); ++it) {
        if (it->second.ctr != 0){
            it->second.calc_stats(); 
            if (next(it) ==  g_topics_stats.end()){
                stats_ss << "\t\t\"" <<it->first<<'"'<<":{" << endl;
                stats_ss << "\t\t\t\""<<"mean"<<'"'<<":"<< it->second.mean_pub_rate <<","<< endl;
                stats_ss << "\t\t\t\""<<"std"<<'"'<<":"<< it->second.std_pub_rate << ","<< endl;
                stats_ss << "\t\t\t\""<<"std"<<'"'<<":"<< it->second.std_pub_rate << ","<< endl;
                stats_ss << "\t\t\t\""<<"msg_avg_age"<<'"'<<":"<< it->second.stamp_age_mean<< ","<< endl;
                stats_ss << "\t\t\t\""<<"msg_max_age"<<'"'<<":"<< it->second.stamp_age_max<< ","<< endl;
                stats_ss << "\t\t\t\""<<"droppage_rate"<<'"'<<":"<< it->second.mean_droppage_rate << endl <<"\t\t}" << endl;
                stats_ss <<"\t}" << ","<<endl;
            }
            else{
                stats_ss << "\t\t\"" <<it->first<<'"'<<":{" << endl;
                stats_ss << "\t\t\t\""<<"mean"<<'"'<<":"<< it->second.mean_pub_rate<<","<< endl;
                stats_ss << "\t\t\t\""<<"std"<<'"'<<":"<< it->second.std_pub_rate << "," << endl;
                stats_ss << "\t\t\t\""<<"msg_avg_age"<<'"'<<":"<< it->second.stamp_age_mean<< ","<< endl;
                stats_ss << "\t\t\t\""<<"msg_max_age"<<'"'<<":"<< it->second.stamp_age_max<<","<< endl;
                stats_ss << "\t\t\t\""<<"droppage_rate"<<'"'<<":"<<it->second.mean_droppage_rate << endl <<"\t\t}," << endl;
            }
        }
    }
    ROS_INFO_STREAM("g_stats_fname"<<g_stats_fname);
    update_stats_file(g_stats_fname, stats_ss.str());
}


bool start_profiling_cb(profile_manager::start_profiling_srv::Request &req, profile_manager::start_profiling_srv::Response &res){
 if (g_start_profiling_data) {
     res.start = true;
 }else{
     res.start = false;
 }
 return true;
}


bool record_profiling_data_cb(profile_manager::profiling_data_srv::Request &req, profile_manager::profiling_data_srv::Response &res)
{
    ROS_ERROR_STREAM("inside the call back"); 
    if (g_drone == NULL) {
        ROS_ERROR_STREAM("drone object is not initialized");
        return false; 
    }

    if(req.key == "start_profiling"){  
        g_init_stats = g_drone->getFlightStats();
        xs_gpu.start = xs_cpu.start = std::chrono::system_clock::now();
        xs_gpu.running = xs_cpu.running = true;
        #ifdef USE_INTEL
        setup_rapl_sysfs(&rs);
        #endif // USE_INTEL
    
        //get list of topics 
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
              const ros::master::TopicInfo& info = *it;
              g_topics_stats[info.name] = statsStruct();
              //std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << std::endl;
        }
        g_start_profiling_data = true;
    }else{ 
        g_highlevel_application_stats.push_back(KeyValuePairStruct(req.key, req.value));
    } 
    return true; 
}

void topic_statistics_cb(const rosgraph_msgs::TopicStatistics::ConstPtr& msg) {
    if (g_topics_stats.size() == 0) {//while not populated with the topic, return
        return;
    }
    long long window_start_sec =  msg->window_start.sec;
    long long window_stop_sec =  msg->window_stop.sec;
    long long window_start_nsec =  msg->window_start.nsec;
    long long window_stop_nsec =  msg->window_stop.nsec;
    double window_duration = (window_stop_sec - window_start_sec)+
        (window_stop_nsec - window_start_nsec)/1e9; //the reason to have this in nano second 
                                                    //is for division to be greater than zero
    long long msg_droppage_rate = (msg->dropped_msgs*100)/window_duration;
    long long stamp_age_mean = (msg->stamp_age_mean.toSec())*1e9;
    double stamp_age_max = (msg->stamp_age_max.toSec());

    int pub_rate  = (int) ((double)1.0/(double)msg->period_mean.toSec());
    int pub_rate_sqr = pub_rate*pub_rate; 
    
    if (g_start_profiling_data){
    g_topics_stats[msg->topic].acc(pub_rate, msg_droppage_rate, 
                                   stamp_age_mean, stamp_age_max);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "profile_manager");
    ros::NodeHandle nh;
    ros::ServiceServer record_profiling_data_service = nh.advertiseService("record_profiling_data", record_profiling_data_cb);
    ros::ServiceServer start_profiling = nh.advertiseService("start_profiling", start_profiling_cb);
    
    initialize_params();
    ros::Subscriber topic_statistics_sub =  
		nh.subscribe<rosgraph_msgs::TopicStatistics>("/statistics", 20, topic_statistics_cb);
    g_drone = new Drone(g_ip_addr.c_str(), g_port);

#ifdef USE_NVML
    if (nvmlInit() != NVML_SUCCESS) {
        std::cout << "nvmlInit() failed.\n";
        return 0;
    }
#endif

    ros::Rate loop_rate(2);
    while (ros::ok()) {
        read_gpu_power_sample(&xs_gpu);
        #ifndef USE_INTEL
        read_cpu_power_sample(&xs_cpu);
        #endif // NOT USE_INTEL
        
        if (g_drone->getFlightStats().collision_count > 1) {
           ; 
            //ROS_INFO_STREAM("collision count too high");
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

#ifdef USE_NVML
    nvmlShutdown();
#endif  // USE_NVML

    double cpu_compute_energy = 0, gpu_compute_energy = 0;
    gpu_compute_energy = xs_gpu.sum;
    #ifdef USE_INTEL
    cpu_compute_energy = run_rapl_sysfs(&rs);
    #else
    cpu_compute_energy = xs_cpu.sum;
    #endif // USE_INTEL
    
    g_end_stats = g_drone->getFlightStats();
    ROS_ERROR_STREAM("shouldn't be here yet"); 
    g_highlevel_application_stats.push_back(
            KeyValuePairStruct("gpu_compute_energy", gpu_compute_energy));
    g_highlevel_application_stats.push_back(
            KeyValuePairStruct("cpu_compute_energy", cpu_compute_energy));
    output_flight_summary();
    return 0;
}

