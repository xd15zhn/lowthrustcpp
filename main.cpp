#include <cmath>
#include <list>
#include "detector.hpp"
#include "elements.hpp"
#include "parameters.hpp"
#include "camera.h"

// #define DEBUG
#define FULL_SCREEN

// 类型转换
Vector3 MatToVector3(Mat m) {
    return (Vector3) {float(m.at(0,0)), float(m.at(1,0)), float(m.at(2,0))};
}
// 画轨迹
void Draw_Trajectory(list<Vector3>& point, Color color) {
    std::list<Vector3>::iterator it = point.begin();
    Vector3 posstart, posend;
    posstart = *it;
    it++;
    for (; it != point.end(); ++it) {
        posend = *it;
        DrawLine3D(posstart, posend, color);
        posstart = posend;
    }
}

int main(void) {
/**********************
计算地球和火星轨道
**********************/
    Mat startR(3, 1), startV(3, 1);  // 出发点位置和速度向量
    Mat endR(3, 1), endV(3, 1);  // 到达点位置和速度向量
    list<Vector3> earthOrbit, marsOrbit;  // 存储地球和火星轨道轨迹点
    list<Vector3>  usvOrbit1, usvOrbit2, usvOrbit3;  // 存储探测器的三段轨道轨迹点
    Orbital_Elements earthElements(SUN_MU, Mat(6, 1, vecdble{  // 根据开普勒轨道六要素计算地球轨道
        EARTH_a, EARTH_e, EARTH_i, EARTH_n, EARTH_w, EARTH_f}));
    Orbital_Elements marsElements(SUN_MU, Mat(6, 1, vecdble{  // 根据开普勒轨道六要素计算火星轨道
        MARS_a, MARS_e, MARS_i, MARS_n, MARS_w, MARS_f}));
    int t;  // 全局时间，单位千秒
    Mat point;  // 轨迹点暂存
    for (t = 0; t < 1577; t++) {  // 365*86400/1e4/2
        earthElements.Update_TrueAnomaly(t);  // 根据当前时间计算真近点角
        point = earthElements.Update_Position();  // 根据当前真近点角计算位置向量
        earthOrbit.push_back(MatToVector3(point));  // 存储轨迹点
        if (t==0) {
            startR = point;  // 记录从地球出发的位置向量
            startV = earthElements.Update_Velocity();  // 根据当前真近点角计算从地球出发的速度向量
        }
    }
    for (t = 0; t < 2968; t++) {  // 687*86400/1e4/2
        marsElements.Update_TrueAnomaly(t);
        point = marsElements.Update_Position();
        marsOrbit.push_back(MatToVector3(point));
        if (t==0) {
            endR = point;  // 记录到达火星的位置
            endV = earthElements.Update_Velocity();  // 根据当前真近点角计算到达火星的速度向量
        }
    }
    cout << "start position, start velocity, end position, end velocity:" << endl;
    cout << startR << endl;
    cout << startV << endl;
    cout << endR << endl;
    cout << endV << endl;

/**********************
计算转移轨道
**********************/
    vecdble solution{  // 最优解
        0.327971, -22.18204, -0.023281, 0.888952, -1.17108, -23.42276,
        0.760948, 14.537111, 0.256603, -8.726293, 0.510507, -1.214071,
        -1.76277, 8.449013, 0.773741, -5.83875, 0.94229, -0.752097,
    };
    // 以下为4个三次多项式
    function<double(double)> cubicFunc1 = [solution](double t){
        double ans = KT*solution[0]*t + solution[4];
        ans = KT*ans*t + solution[8];
        ans = KT*ans*t + solution[12];
        return ans;
    };
    function<double(double)> cubicFunc2 = [solution](double t){
        double ans = KT*1e-5*solution[1]*t + 1e-5*solution[5];
        ans = KT*ans*t + 0.01*solution[9];
        ans = KT*ans*t + 0.01*solution[13];
        return ans;
    };
    function<double(double)> cubicFunc3 = [solution](double t){
        double ans = KT*solution[2]*t + solution[6];
        ans = KT*ans*t + solution[10];
        ans = KT*ans*t + solution[14];
        return ans;
    };
    function<double(double)> cubicFunc4 = [solution](double t){
        double ans = KT*1e-5*solution[3]*t + 1e-5*solution[7];
        ans = KT*ans*t + 0.01*solution[11];
        ans = KT*ans*t + 0.01*solution[15];
        return ans;
    };
    Orbital_Solver solver;  // 微分方程求解器
    solver._mssr->Set_InitialValue(Mat(vecdble{  // 设置位置向量积分器的初值
        -113.101881186742, 101.033358324501, 0.00219405859350417}));
    solver._mssv->Set_InitialValue(Mat(vecdble{  // 设置速度向量积分器的初值
        -19.3527970698371, -22.1182257893965, -0.000451706663468062}));
    solver._intM->Set_InitialValue(USV_M);  // 设置初始质量
    cout << "Calculating trajectory......" << endl;
    /*分别设置第一段和第三段的发动机工作时间*/
    int dur1 = 25000/PI * atan(solution[16]*solution[16]);
    int dur2 = FLY_TIME - 15000/PI * atan(solution[17]*solution[17]);
    /*第一段发动机工作，为俯仰角和方位角分别设置三次函数*/
    solver._inTheta->Set_Function(cubicFunc1);
    solver._inPhi->Set_Function(cubicFunc2);
    solver._cnstF->Set_OutValue(USV_F);
    for (t = 0; t < dur1; t++) {
        // for (int i = 0; i < 100; i++)
            solver._sim1.Simulate_OneStep();
        point = solver._mssr->Get_OutValue();
        usvOrbit1.push_back(MatToVector3(point));  // 存储轨迹点
    }
    /*第二段发动机关闭*/
    solver._cnstF->Set_OutValue(0);
    for (; t < dur2; t++) {
        // for (int i = 0; i < 100; i++)
            solver._sim1.Simulate_OneStep();
        point = solver._mssr->Get_OutValue();
        usvOrbit2.push_back(MatToVector3(point));
    }
    /*第三段发动机工作，为俯仰角和方位角分别设置新的三次函数*/
    solver._inTheta->Set_Function(cubicFunc3);
    solver._inPhi->Set_Function(cubicFunc4);
    solver._cnstF->Set_OutValue(USV_F);
    for (; t < FLY_TIME; t++) {
        // for (int i = 0; i < 100; i++)
            solver._sim1.Simulate_OneStep();
        point = solver._mssr->Get_OutValue();
        usvOrbit3.push_back(MatToVector3(point));
    }
    cout << "Mars trajectory calculating finished." << endl;
    cout << "duration1: " << dur1 << endl;
    cout << "duration2: " << dur2 << endl;
    cout << "mass remain: "  << 1000 - (dur1 + FLY_TIME - dur2)*1000.0/19600.0 << endl;
    cout << "mass remain: " << solver._intM->Get_OutValue() << endl;

/**********************
绘图
**********************/
#ifndef DEBUG
    Camera camera;
	SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetTargetFPS(60);
#ifdef FULL_SCREEN
    SetWindowMonitor(1);
    SetConfigFlags(FLAG_FULLSCREEN_MODE);
    InitGraph(0, 0, "RayLib-3D");
#else
    InitGraph(1024, 768, "RayLib-3D");
#endif
	Init_Camera(&camera);
    Vector3 cubePosition = { 0.0f, 0.0f, 0.0f };

    while (!WindowShouldClose()) {
        Update_Camera(&camera);
        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode3D(camera);
                // DrawGrid(120, 5);
                DrawSphere((Vector3){0.0f, 0.0f, 0.0f}, 10, ORANGE);  // 画3D球
                // Draw_Frame_Oxyz();  // 坐标系测试
                Draw_Trajectory(earthOrbit, RAYWHITE);  // 画地球轨道
                Draw_Trajectory(marsOrbit, RAYWHITE);  // 画火星轨道
                Draw_Trajectory(usvOrbit1, RED);  // 画第一段加速度轨道
                Draw_Trajectory(usvOrbit2, GREEN);  // 画第二段无动力滑行轨道
                Draw_Trajectory(usvOrbit3, RED);  // 画第三段加速度轨道
            EndMode3D();
            DrawText(TextFormat("%2i FPS", GetFPS()), 0, 0, 20, LIME);
        EndDrawing();
    }
    CloseGraph();
#endif
    return 0;
}
