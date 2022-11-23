constexpr double PI = 3.14159265358979323846;
/* 下面的值需要换算到求解器时空坐标下，注释中为换算前的单位 */
constexpr double AU = 149597870e-6;  // 天文单位
constexpr double SUN_MU = 132706538114e-10;  // 太阳μ值
/* 地球轨道处开普勒轨道根数 */
constexpr double EARTH_a = 1.000840*AU;
constexpr double EARTH_e =  0.016507;
constexpr double EARTH_i =  0.000021;
constexpr double EARTH_n =  0.030896;
constexpr double EARTH_w = -1.422358;
constexpr double EARTH_f =  3.824526;
/* 火星轨道处开普勒轨道根数 */
constexpr double MARS_a =  1.523677*AU;
constexpr double MARS_e =  0.093435;
constexpr double MARS_i =  0.032276;
constexpr double MARS_n =  0.864609;
constexpr double MARS_w = -1.281742;
constexpr double MARS_f =  6.182348;
constexpr double USV_M = 1000;  // 飞行器初始质量(kg)(Unmanned Space Vehicle)
constexpr double USV_F = 1e-3;  // 发动机推力(kN)
constexpr double USV_dm = 1/19600;  // 燃料消耗速度(kg/s)
