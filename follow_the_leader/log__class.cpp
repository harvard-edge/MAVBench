#include "log__class.h"
#include <iostream>
#include "configs.h"
#include <fstream>
#include "string"
using namespace std;

log__class::log__class(){
}


void log__class::set__addr(std::string addr){
    this->addr =  addr;
    log__f__handle.open(this->addr);
    log__f__handle.close();
}

void log__class::write(std::string msg){
    log__f__handle.open(addr, ios::app);
    log__f__handle << msg<<endl; 
    log__f__handle.close();
}
