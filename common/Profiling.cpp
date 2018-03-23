#include "Profiling.h"
#include <fstream>
void signal_supervisor(std::string file_to_write_to, std::string msg){
    std::ofstream file_to_write_to_h; //file handle write to when completed
    file_to_write_to_h.open(file_to_write_to, std::ofstream::out);
    file_to_write_to_h<< msg;
    file_to_write_to_h.close();
}

void update_stats_file(const std::string& stats_file__addr, const std::string& content){
    std::ofstream myfile;
    myfile.open(stats_file__addr, std::ofstream::out | std::ofstream::app);
    myfile << content << std::endl;
    myfile.close();
    return;
}
