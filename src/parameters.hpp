/*下面的值需要换算到求解器时空坐标下，注释中为换算前的单位*/
constexpr double MU_SUN = 132706538114e-10;  // 太阳μ值
constexpr double MU_EARTH = 398601e-10;  // 地球μ值
constexpr double MU_MARS = 42808e-10;  // 火星μ值
constexpr double M_USV=1000;  // 飞行器初始质量(kg)(Unmanned Space Vehicle)
constexpr double F=1e-3;  // 发动机推力(kN)
constexpr double dm=1/19600;  // 燃料消耗速度(kg/s)
