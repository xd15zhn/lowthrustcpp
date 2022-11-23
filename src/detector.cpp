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
    _mssr = new MStateSpace(&_sim1, BusSize(3, 1), "_mssr");
    _mssv = new MStateSpace(&_sim1, BusSize(3, 1), "_mssv");
    _misoFmu = new MFcnMISO(&_sim1, BusSize(3, 1), "_misoFmu");
    _misoFa = new MFcnMISO(&_sim1, BusSize(3, 1), "_misoFa");
    _mux = new Mux(&_sim1, BusSize(4, 1), "_mux");
    _intM = new UIntegrator(&_sim1, "_intM");
    _gainKm = new UGain(&_sim1, "_gainKm");
    _sim1.connectM(_misoFa, _mssv);
    _sim1.connectM(_mssv, _mssr);
    _sim1.connectM(_mssr, _misoFmu);
    _sim1.connectM(_misoFmu, _misoFa);
    _sim1.connectM(_mux, _misoFa);
    _sim1.connectU(_inTheta, _mux, BusSize(0, 0));
    _sim1.connectU(_inPhi, _mux, BusSize(1, 0));
    _sim1.connectU(_cnstF, _mux, BusSize(2, 0));
    _sim1.connectU(_intM, _mux, BusSize(3, 0));
    _sim1.connectU(_cnstF, _gainKm);
    _sim1.connectU(_gainKm, _intM);
    _intM->Set_InitialValue(USV_M);
    _gainKm->Set_Gain(USV_Km);
    _misoFmu->Set_Function([](Mat *u){
        Vector3d r = u[0];
        double k = r.norm2();
        k = -SUN_MU / (k*k*k);
        return Mat(k*r);
    });
    _misoFa->Set_Function([](Mat *u){
        double theta = u[1].at(0, 0);
        double phi = u[1].at(1, 0);
        double f = u[1].at(2, 0);
        double m = u[1].at(3, 0);
        return Mat(3, 1, vecdble{
            u[0].at(0, 0) + f/m*cos(phi)*cos(theta),
            u[0].at(1, 0) + f/m*cos(phi)*sin(theta),
            u[0].at(2, 0) + f/m*sin(phi),
        });
    });
    _sim1.Set_EnableStore(false);
    _sim1.Initialize();
}
