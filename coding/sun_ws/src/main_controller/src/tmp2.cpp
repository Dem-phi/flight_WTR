//
// Created by demphi on 9/14/21.
//

#include <iostream>
#include <Communication.h>
#include "nlohmann/json.hpp"
#include <ac_remap.h>
using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "ACdata_node");
    ros::NodeHandle nh;

    Remaper remaper(nh);

    remaper.set_serial_port("/dev/ttyUSB0", 115200);

    while(true){
        remaper.read_ac_data();
    }





    return 0;

}