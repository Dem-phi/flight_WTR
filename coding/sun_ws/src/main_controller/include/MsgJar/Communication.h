#ifndef SRC_COMMUNICATION_H
#define SRC_COMMUNICATION_H

#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <math.h>
using namespace std;

class Acfly{
public:
    serial::Serial sp;
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    uint8_t read_buffer[256];
    uint8_t  write_buffer[256];

    bool read();
    void write_pose(float x, float y, float z);
    void deal_float(float num, char* char_num);
    bool is_open();
    void refresh_buffer(uint8_t* buffer);

    Acfly(string port_name, int rate);
    ~Acfly();


};

Acfly::~Acfly(){
    this->sp.close();
}

void Acfly::refresh_buffer(uint8_t *buffer){
    for(int i = 0; i < 0; i++){
        buffer[i] = 0;
    }
    return;
}

Acfly::Acfly(string port_name, int rate){
    this->sp.setBaudrate(rate);
    this->sp.setPort(port_name);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    this->sp.setTimeout(to);
    this->sp.open();
    return;
}


bool Acfly::is_open(){
    return this->sp.isOpen();
}

bool Acfly::read(){
    size_t n = this->sp.available();
    if(n == 0){
        return false;
    }
    this->refresh_buffer(this->read_buffer);
    this->sp.read(this->read_buffer, n);
    return true;
}

// 108.32
void Acfly::deal_float(float num, char *char_num){
    int int_num = (int)100*num;
    int index = 0;
    for(int i = 4; i >= 0; i --){
        int time = pow(10,i);
        char_num[index] = (int)(int_num/time)%10 + 48;
        index ++;
    }
}

void Acfly::write_pose(float x, float y, float z){
    this->refresh_buffer(this->write_buffer);
    int index = 0;
    this->write_buffer[index] = (int)'#'; index++;
    char tmp_words[5];

    deal_float(x, tmp_words);
    for(int i = 0; i < 5; i ++){
        this->write_buffer[index] = (int)tmp_words[i];
        index++;
    }

    deal_float(y, tmp_words);
    for(int i = 0; i < 5; i ++){
        this->write_buffer[index] = (int)tmp_words[i];
        index++;
    }

    deal_float(z, tmp_words);
    for(int i = 0; i < 5; i ++){
        this->write_buffer[index] = (int)tmp_words[i];
        index++;
    }
    this->sp.write(this->write_buffer, index);
    return;
}







#endif
