/**********************
轨道元素与位置速度换算 源文件
**********************/
#include "elements.hpp"
#include <cmath>

Orbital_Elements::Orbital_Elements(double mu, Mat orbitalElement) {
    Initialize(mu, orbitalElement);
}
void Orbital_Elements::Initialize(double mu, Mat orbitalElement) {
    double i, n, w, f, E;
    _mu = mu;
    _a = orbitalElement.at(0, 0);  // a-半长轴
    _e = orbitalElement.at(1, 0);  // e-偏心率
     i = orbitalElement.at(2, 0);  // i-轨道倾角
     n = orbitalElement.at(3, 0);  // Ω-升交点赤经
     w = orbitalElement.at(4, 0);  // ω-近地点幅角
     f = orbitalElement.at(5, 0);  // f-真近点角
     E = 2*atan(sqrt((1+_e)/(1-_e))*tan(0.5*f));
    _m = E - _e*sin(E);
    _kv = sqrt(_mu/(_a*(1-_e*_e)));
    _omega = sqrt(_mu/(_a*_a*_a));
    _RT = Mat(3, 3, vecdble{
        cos(n)*cos(w)-sin(n)*cos(i)*sin(w), -cos(n)*sin(w)-sin(n)*cos(i)*cos(w),  sin(n)*sin(i),
        sin(n)*cos(w)+cos(n)*cos(i)*sin(w), -sin(n)*sin(w)+cos(n)*cos(i)*cos(w), -cos(n)*sin(i),
        sin(i)*sin(w),                          sin(i)*cos(w),                          cos(i)
    });
}

void Orbital_Elements::Update_TrueAnomaly(double t) {
    double ma = _m + _omega*t;  // 当前平近点角(mean anomaly)
    double ta = ma;  // 真近点角(true anomaly)
    ta += (_e+_e-_e*_e*_e*0.25)*sin(ma);
    ta += 1.25*_e*_e*sin(2*ma);
    ta += 13/12*_e*_e*_e*sin(3*ma);
    _ta = ta;
}
Mat Orbital_Elements::Update_Position() {
    double r = _a*(1-_e*_e)/(1+_e*cos(_ta));
    _position = Mat(3, 1, vecdble{r*cos(_ta), r*sin(_ta), 0});
    _position = _RT*_position;
    return _position;
}
Mat Orbital_Elements::Update_Velocity() {
    _velocity = Mat(3, 1, vecdble{-_kv*sin(_ta), _kv*(_e+cos(_ta)), 0});
    _velocity = _RT*_velocity;
    return _velocity;
}
