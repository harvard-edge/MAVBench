#ifndef PROFILING_H
#define PROFILING_H

#include <string>

typedef struct KeyValuePair {
    std::string key;
    double value;
    KeyValuePair(std::string key, double value): key(key), value(value){
    }

} KeyValuePairStruct;

typedef struct stats {
    long long pub_rate_accumulate;
    long long pub_rate_accumulate_sqr;
    long long droppage_rate_accumulate;
    double mean_pub_rate; 
    double std_pub_rate;
    double  mean_droppage_rate;
    double stamp_age_mean_accumulate;
    double stamp_age_max;
    double stamp_age_mean; 
    int ctr; 
    stats(){
         this->pub_rate_accumulate = 0;
         this->pub_rate_accumulate_sqr = 0;
         this->droppage_rate_accumulate = 0;
         this->mean_pub_rate = 0; 
         this->std_pub_rate = 0;
         this->mean_droppage_rate = 0;
         this->stamp_age_mean_accumulate = 0;
         this->stamp_age_mean =  0;
         this->stamp_age_max =  0;
         this->ctr = 0;
    }

	// accumulate values  
	void acc(long long pub_rate, long long droppage_rate, 
            long long stamp_age_mean, double stamp_age_max){
        this->pub_rate_accumulate += pub_rate;
        this->pub_rate_accumulate_sqr += pub_rate*pub_rate;
        this->droppage_rate_accumulate += droppage_rate;
        this->stamp_age_mean_accumulate += stamp_age_mean;
        this->stamp_age_max = std::max(this->stamp_age_max, stamp_age_max);
        this->ctr += 1;
    }


	void calc_stats() {
        this->mean_pub_rate = (double)this->pub_rate_accumulate/this->ctr;
        double var = -1*pow((double)this->pub_rate_accumulate/this->ctr, 2);
        var +=  (double)this->pub_rate_accumulate_sqr/this->ctr;
        this->std_pub_rate = pow(var,.5);
        this->mean_droppage_rate = ((double)this->droppage_rate_accumulate/this->ctr)/100;
        this->stamp_age_mean = ((double)this->stamp_age_mean_accumulate/this->ctr)/1e9; 
    }
} statsStruct;

void signal_supervisor(std::string file_to_write_to, std::string msg);
void update_stats_file(const std::string& stats_file__addr, const std::string& content);
bool log_data_in_profiler(const std::string& key, double value);

#endif

