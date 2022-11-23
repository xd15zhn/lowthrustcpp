/**********************
轨道元素与位置速度换算 头文件
**********************/
#include <iostream>
#include "zhnmat.hpp"
using namespace zhnmat;
using namespace std;
typedef std::vector<double>  vecdble;

/*********************
记录中心天体μ值和椭圆/双曲线轨道的轨道六要素，抛物线不适用
轨道六要素中的前5个为常数，真近点角记录为初始角度，之后根据经过的时间更新
**********************/
struct Orbital_Elements {
    Orbital_Elements(double mu=1, Mat orbitalElement=Mat(6, 1));
    void Initialize(double mu, Mat orbitalElement);
    void Update_TrueAnomaly(double t);  // 计算经过时间[t]后的真近点角
    Mat Update_Position();  // 根据更新后的真近点角计算位置向量
    Mat Update_Velocity();  // 根据更新后的真近点角计算速度向量
    double _mu;  // 中心天体μ值
    double _kv;  // 用于求速度向量的常数系数
    double _omega;  // 轨道平均角速度
    double _ta;  // 真近点角(true anomaly)
    double _a, _e, _m;  // 半长轴，偏心率，初始平近点角
    Mat _RT;  // i,n,w确定的坐标变换矩阵
    Mat _position;  // 位置向量
    Mat _velocity;  // 速度向量
};
