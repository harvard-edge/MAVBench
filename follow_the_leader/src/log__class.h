#ifndef LOG__CLASS_H
#define LOG__CLASS_H
#include <fstream>
using namespace std;
#include "string"
class log__class
{
public:
    log__class();
    void set__addr(std::string addr);
    void write(std::string msg); 	

private:
    ofstream log__f__handle;
    std::string addr;
};

#endif
