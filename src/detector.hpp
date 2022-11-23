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
    UConstant *_cnstF=nullptr;  // 发动机推力大小
    MStateSpace *_mssr=nullptr;  // 探测器位置向量
    MStateSpace *_mssv=nullptr;  // 探测器速度向量
    MFcnMISO *_misoFmu=nullptr;  // 加速度中的重力项
    MFcnMISO *_misoFa=nullptr;  // 探测器加速度
    Mux *_mux=nullptr;  //
    UIntegrator *_intM=nullptr;  // 探测器总质量
    UGain *_gainKm=nullptr;  //
};

#endif // ORBITALSIM_H
