#include <cmath>
#include <list>
#include "detector.hpp"
#include "elements.hpp"
#include "parameters.hpp"
#include "camera.h"

// #define DEBUG
#define FULL_SCREEN

Vector3 MatToVector3(Mat m) {
    return (Vector3) {float(m.at(0,0)), float(m.at(1,0)), float(m.at(2,0))};
}
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

class Cubic: public simucpp::UserFunc
{
public:
    // Cubic(double a, double b, double c, double d):
    //     _a(a), _b(b),_c(c), _d(d) {}
    void Set(double a, double b, double c, double d)
        {_a=a; _b=b;_c=c; _d=d;}
    virtual double Function(double t) const {
        double ans = _a*t + _b;
        ans = ans*t + _c;
        ans = ans*t + _d;
        return ans;
    };
private:
    double _a, _b, _c, _d;
};

int main(void) {
/**********************
计算地球和火星轨道
**********************/
    Mat startR(3, 1), startV(3, 1);
    Mat endR(3, 1), endV(3, 1);
    list<Vector3> earthOrbit, marsOrbit;
    list<Vector3>  usvOrbit1, usvOrbit2, usvOrbit3;
    Orbital_Elements earthElements(SUN_MU, Mat(6, 1, vecdble{
        EARTH_a, EARTH_e, EARTH_i, EARTH_n, EARTH_w, EARTH_f}));
    Orbital_Elements marsElements(SUN_MU, Mat(6, 1, vecdble{
        MARS_a, MARS_e, MARS_i, MARS_n, MARS_w, MARS_f}));
    int t;
    Mat point;
    for (t = 0; t < 1577; t++) {  // 365*86400/1e4/2
        earthElements.Update_TrueAnomaly(t);
        point = earthElements.Update_Position();
        earthOrbit.push_back(MatToVector3(point));  // 存储轨迹点
        if (t==0) {
            startR = point;
            startV = earthElements.Update_Velocity();
        }
    }
    for (t = 0; t < 2968; t++) {  // 687*86400/1e4/2
        marsElements.Update_TrueAnomaly(t);
        point = marsElements.Update_Position();
        marsOrbit.push_back(MatToVector3(point));
        if (t==0) {
            endR = point;
            endV = earthElements.Update_Velocity();
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
    cout << "Calculating trajectory......" << endl;
    vecdble solution{
        0.287717, 2.77068, 1.31148, 1.01602, -0.610206, 1.1287,
        0.482372, 2.97458, -0.75428, 2.42673, 0.91558, 0.618227,
        -1.53584, -0.00251363, -1.56059, -1.15623, 0.97752, -0.484669,
    };
    Cubic *cubic1 = new Cubic;
    Cubic *cubic2 = new Cubic;
    Orbital_Solver solver;
    usvOrbit1.push_back(MatToVector3(startR));
    solver._mssr->Set_InitialValue(Mat(vecdble{
        -113.101881186742, 101.033358324501, 0.00219405859350417}));
    solver._mssv->Set_InitialValue(Mat(vecdble{
        -19.3527970698371, -22.1182257893965, -0.000451706663468062}));
    solver._intM->Set_InitialValue(USV_M);
    /*分别设置第一段和第三段的发动机工作时间*/
    int dur1 = 25000/PI * atan(solution[16]*solution[16]);
    int dur2 = FLY_TIME - 15000/PI * atan(solution[17]*solution[17]);
    /*第一段发动机工作，为俯仰角和方位角分别设置三次函数*/
    cubic1->Set(solution[0], solution[4], solution[8], solution[12]);
    cubic2->Set(1e-6*solution[1], 1e-6*solution[5], 1e-3*solution[9], 1e-3*solution[13]);
    solver._inTheta->Set_Function(cubic1);
    solver._inPhi->Set_Function(cubic2);
    solver._cnstF->Set_OutValue(USV_F);
    for (t = 0; t < dur1; t++) {
        solver._sim1.Simulate_OneStep();
        point = solver._mssr->Get_OutValue();
        usvOrbit1.push_back(MatToVector3(point));
    }
    /*第二段发动机关闭*/
    solver._cnstF->Set_OutValue(0);
    for (; t < dur2; t++) {
        solver._sim1.Simulate_OneStep();
        point = solver._mssr->Get_OutValue();
        usvOrbit2.push_back(MatToVector3(point));
    }
    /*第三段发动机工作，为俯仰角和方位角分别设置新的三次函数*/
    cubic1->Set(solution[2], solution[6], solution[10], solution[14]);
    cubic2->Set(1e-6*solution[3], 1e-6*solution[7], 1e-3*solution[11], 1e-3*solution[15]);
    solver._inTheta->Set_Function(cubic1);
    solver._inPhi->Set_Function(cubic2);
    solver._cnstF->Set_OutValue(USV_F);
    for (; t < FLY_TIME; t++) {
        solver._sim1.Simulate_OneStep();
        point = solver._mssr->Get_OutValue();
        usvOrbit3.push_back(MatToVector3(point));
    }
    cout << "\nMars trajectory calculating finished." << endl;
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
                Draw_Trajectory(earthOrbit, RAYWHITE);  // 画3D轨迹
                Draw_Trajectory(marsOrbit, RAYWHITE);  // 画3D轨迹
                Draw_Trajectory(usvOrbit1, GREEN);  // 画3D轨迹
                Draw_Trajectory(usvOrbit2, WHITE);  // 画3D轨迹
                Draw_Trajectory(usvOrbit3, GREEN);  // 画3D轨迹
            EndMode3D();
            DrawText(TextFormat("%2i FPS", GetFPS()), 0, 0, 20, LIME);
        EndDrawing();
    }
    CloseGraph();
#endif
    return 0;
}
