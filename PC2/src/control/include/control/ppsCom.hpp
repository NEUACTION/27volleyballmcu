#ifndef _PPSCOM_H
#define _PPSCOM_H

#include <chrono>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces/msg/pos_data.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class Pos_t {
public:
    float x;
    float y;
    float pitch;
    float roll;
    float theta;
    float w1;
    float w2;
    float w3;
    float a1;
    float a2;
    float a3;
    float timestamps;
public:
    explicit Pos_t()
    {
        x = y = pitch = roll = theta = w1 = w2 = w3 = a1 = a2 = a3 = 0.0f;
    }
    ~Pos_t(){}
};

#endif