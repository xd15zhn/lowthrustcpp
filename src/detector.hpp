/**********************
火星探测器被控对象 头文件
**********************/
#ifndef ORBITALSIM_H
#define ORBITALSIM_H

#include <cmath>
#include "simucpp.hpp"
#include "parameters.hpp"
using namespace simucpp;
using namespace zhnmat;
using namespace std;

#define PI 3.14159265358979323846f


/**********************
轨道仿真器
**********************/
class Orbital_Solver {
public:
    Orbital_Solver();
    Simulator _sim1;
    UInput *_inTheta=nullptr;  // 发动机方位角
    UInput *_inPhi=nullptr;  // 发动机俯仰角
    UConstant* _cnstF=nullptr;  // 发动机推力大小
    UIntegrator *_intRx=nullptr;  // 探测器位置向量
    UIntegrator *_intRy=nullptr;  // 探测器位置向量
    UIntegrator *_intRz=nullptr;  // 探测器位置向量
    UIntegrator *_intVx=nullptr;  // 探测器速度向量
    UIntegrator *_intVy=nullptr;  // 探测器速度向量
    UIntegrator *_intVz=nullptr;  // 探测器速度向量
    UFcnMISO *_misoAx=nullptr;
    UFcnMISO *_misoAy=nullptr;
    UFcnMISO *_misoAz=nullptr;
    UFcnMISO *_misoMu=nullptr;
    UIntegrator *_intM=nullptr;
    UGain *_gainKm=nullptr;
};

#endif // ORBITALSIM_H
