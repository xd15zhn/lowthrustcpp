#include <cmath>
#include <list>
#include "detector.hpp"
#include "elements.hpp"
#include "parameters.hpp"
#include "camera.h"

// #define DEBUG
#define FULL_SCREEN

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
    Mat startR(3, 1), startV(3, 1);
    Mat endR(3, 1), endV(3, 1);
    list<Vector3> earthOrbit, marsOrbit, usvOrbit;
    Orbital_Elements earthElements(SUN_MU, Mat(6, 1, vecdble{
        EARTH_a, EARTH_e, EARTH_i, EARTH_n, EARTH_w, EARTH_f}));
    Orbital_Elements marsElements(SUN_MU, Mat(6, 1, vecdble{
        MARS_a, MARS_e, MARS_i, MARS_n, MARS_w, MARS_f}));
    int t;
    Mat point;
    cout << "Calculating trajectory......" << endl;
    for (t = 0; t < 1577; t++) {  // 365*86400/1e4/2
        earthElements.Update_TrueAnomaly(t);
        point = earthElements.Update_Position();
        earthOrbit.push_back((Vector3){  // 存储轨迹点
            float(point.at(0,0)),
            float(point.at(1,0)),
            float(point.at(2,0)),
        });
        if (t==0) {
            startR = point;
            startV = earthElements.Update_Velocity();
        }
    }
    for (t = 0; t < 2968; t++) {  // 687*86400/1e4/2
        marsElements.Update_TrueAnomaly(t);
        point = marsElements.Update_Position();
        marsOrbit.push_back((Vector3){  // 存储轨迹点
            float(point.at(0,0)),
            float(point.at(1,0)),
            float(point.at(2,0)),
        });
        if (t==0) {
            endR = point;
            endV = earthElements.Update_Velocity();
        }
    }
    cout << "\nMars trajectory calculating finished." << endl;
    // cout << startR << endl;
    // cout << startV << endl;

    Orbital_Solver detector;
    detector._mssr->Set_InitialValue(startR);
    detector._mssv->Set_InitialValue(startV);
    detector._cnstF->Set_OutValue(USV_F);
    detector._inTheta->Set_Function([](double u){return -1.57;});
    detector._inPhi->Set_Function([](double u){return 0.0;});
    for (int i = 0; i < 100; i++) {
        for (int n = 0; n < 1000; n++)
            detector._sim1.Simulate_OneStep();
        point = detector._mssr->Get_OutValue();
        usvOrbit.push_back((Vector3) {
            float(point.at(0,0)),
            float(point.at(1,0)),
            float(point.at(2,0)),
        });
    }
    // cout << detector._intM->Get_OutValue() << endl;

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
                Draw_Trajectory(usvOrbit, GREEN);  // 画3D轨迹
            EndMode3D();
            DrawText(TextFormat("%2i FPS", GetFPS()), 0, 0, 20, LIME);
        EndDrawing();
    }
    CloseGraph();
#endif
    return 0;
}
