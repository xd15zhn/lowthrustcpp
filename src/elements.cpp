/**********************
轨道元素与位置速度换算 源文件
**********************/
#include "elements.hpp"
#include <cmath>

Orbital_Elements::Orbital_Elements(double mu, Mat orbitalElement) {
    Initialize(mu, orbitalElement);
}
void Orbital_Elements::Initialize(double mu, Mat orbitalElement) {
    _mu = mu;
    _a = orbitalElement.at(0, 0);  // a-半长轴
    _e = orbitalElement.at(1, 0);  // e-偏心率
    _i = orbitalElement.at(2, 0);  // i-轨道倾角
    _n = orbitalElement.at(3, 0);  // Ω-升交点赤经
    _w = orbitalElement.at(4, 0);  // ω-近地点幅角
    _f = orbitalElement.at(5, 0);  // f-真近点角
    if (_e > 1) _a = -_a;
    _kv = sqrt(_mu/(_a*(1-_e*_e)));
    _omega = sqrt(_mu/(_a*_a*_a));
    _RT = Mat(3, 3, vecdble{
        cos(_n)*cos(_w)-sin(_n)*cos(_i)*sin(_w), -cos(_n)*sin(_w)-sin(_n)*cos(_i)*cos(_w),  sin(_n)*sin(_i),
        sin(_n)*cos(_w)+cos(_n)*cos(_i)*sin(_w), -sin(_n)*sin(_w)+cos(_n)*cos(_i)*cos(_w), -cos(_n)*sin(_i),
        sin(_i)*sin(_w),                          sin(_i)*cos(_w),                          cos(_i)
    });
}

void Orbital_Elements::Update_TrueAnomaly(double t) {
    double ma = _omega*t;  // 平近点角(mean anomaly)
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
    double r = _a*(1-_e*_e)/(1+_e*cos(_ta));
    _velocity = Mat(3, 1, vecdble{-_kv*sin(_ta), _kv*(_e+cos(_ta)), 0});
    _velocity = _RT*_velocity;
    return _velocity;
}
