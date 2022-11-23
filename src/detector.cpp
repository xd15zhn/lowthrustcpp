/**********************
火星探测器被控对象源文件
**********************/
#include "detector.hpp"
#include <cmath>


/**********************
仿真器
**********************/
Orbital_Solver::Orbital_Solver() {
    _inTheta = new UInput(&_sim1, "_inTheta");
    _inPhi = new UInput(&_sim1, "_inPhi");
    _cnstF = new UConstant(&_sim1, "_cnstF");
    _intRx = new UIntegrator(&_sim1, "_intRx");
    _intRy = new UIntegrator(&_sim1, "_intRy");
    _intRz = new UIntegrator(&_sim1, "_intRz");
    _intVx = new UIntegrator(&_sim1, "_intVx");
    _intVy = new UIntegrator(&_sim1, "_intVy");
    _intVz = new UIntegrator(&_sim1, "_intVz");
    _misoAx = new UFcnMISO(&_sim1, "_misoAx");
    _misoAy = new UFcnMISO(&_sim1, "_misoAy");
    _misoAz = new UFcnMISO(&_sim1, "_misoAz");
    _misoMu = new UFcnMISO(&_sim1, "_misoMu");
    _intM = new UIntegrator(&_sim1, "_intM");
    _gainKm = new UGain(&_sim1, "_gainKm");
    _sim1.connectU(_misoAx, _intVx);
    _sim1.connectU(_misoAy, _intVy);
    _sim1.connectU(_misoAz, _intVz);
    _sim1.connectU(_intVx, _intRx);
    _sim1.connectU(_intVy, _intRy);
    _sim1.connectU(_intVz, _intRz);
    _sim1.connectU(_intRx, _misoMu);
    _sim1.connectU(_intRy, _misoMu);
    _sim1.connectU(_intRz, _misoMu);
    _sim1.connectU(_misoMu, _misoAx);
    _sim1.connectU(_misoMu, _misoAy);
    _sim1.connectU(_misoMu, _misoAz);
    _sim1.connectU(_intRx, _misoAx);
    _sim1.connectU(_intRy, _misoAy);
    _sim1.connectU(_intRz, _misoAz);
    _sim1.connectU(_inTheta, _misoAx);
    _sim1.connectU(_inTheta, _misoAy);
    _sim1.connectU(_inTheta, _misoAz);
    _sim1.connectU(_inPhi, _misoAx);
    _sim1.connectU(_inPhi, _misoAy);
    _sim1.connectU(_inPhi, _misoAz);
    _sim1.connectU(_cnstF, _misoAx);
    _sim1.connectU(_cnstF, _misoAy);
    _sim1.connectU(_cnstF, _misoAz);
    _sim1.connectU(_intM, _misoAx);
    _sim1.connectU(_intM, _misoAy);
    _sim1.connectU(_intM, _misoAz);
    _misoMu->Set_Function([](double *u){
        double ans = u[0]*u[0] + u[1]*u[1] + u[2]*u[2];
        ans = sqrt(ans);
        return -SUN_MU / (ans*ans*ans);
    });
    // double kmu   = u[0];
    // double rx    = u[1];
    // double theta = u[2];
    // double phi   = u[3];
    // double f     = u[4];
    // double m     = u[5];
    // return kmu*rx + f*cos(phi)*cos(theta)/m;
    _misoAx->Set_Function([](double *u){
        return u[0]*u[1] + u[4]*cos(u[3])*cos(u[2])/u[5];
    });
    _misoAy->Set_Function([](double *u){
        return u[0]*u[1] + u[4]*cos(u[3])*sin(u[2])/u[5];
    });
    _misoAz->Set_Function([](double *u){
        return u[0]*u[1] + u[4]*sin(u[3])/u[5];
    });
    _sim1.Set_EnableStore(false);
    _sim1.Initialize();
}
