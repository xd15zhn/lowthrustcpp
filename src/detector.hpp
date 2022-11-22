/**********************
火星探测器被控对象 头文件
**********************/
#ifndef ORBITALSIM_H
#define ORBITALSIM_H

#include <cmath>
#include "simucpp.hpp"
using namespace simucpp;
using namespace zhnmat;
using namespace std;

#define PI 3.14159265358979323846f

/**********************
火星探测器被控对象
**********************/
class MarsDetector: public PackModule {
public:
    MarsDetector(Simulator *sim, std::string name);
    ~MarsDetector() {};
    virtual PMatModule Get_InputBus(int n) const;
    virtual PMatModule Get_OutputBus(int n) const;
    // Mat Get_Position();
    // Mat Get_Velocity();
    std::string _name;
    MFcnMISO *simfLD=nullptr;
    MFcnMISO *simfA=nullptr;
    MStateSpace* simIntr=nullptr;
    MStateSpace* simIntv=nullptr;
    MGain *simgain=nullptr;
};
typedef MarsDetector* PMarsDetector;

#endif // ORBITALSIM_H
