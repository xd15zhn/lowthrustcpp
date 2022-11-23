#include <cmath>
#include <list>
// #include "detector.hpp"
#include "elements.hpp"
#include "parameters.hpp"
#include "camera.h"

#define FULL_SCREEN

void Draw_Trajectory(list<Vector3>& point) {
    std::list<Vector3>::iterator it = point.begin();
    Vector3 posstart, posend;
    posstart = *it;
    it++;
    for (; it != point.end(); ++it) {
        posend = *it;
        DrawLine3D(posstart, posend, RAYWHITE);
        posstart = posend;
    }
}

int main(void) {
    list<Vector3> earthOrbit, marsOrbit;
    Orbital_Elements earthElements(SUN_MU, Mat(6, 1, vecdble{
        EARTH_a, EARTH_e, EARTH_i, EARTH_n, EARTH_w, EARTH_f}));
    Orbital_Elements marsElements(SUN_MU, Mat(6, 1, vecdble{
        MARS_a, MARS_e, MARS_i, MARS_n, MARS_w, MARS_f}));
    double t;
    Mat point;
    cout << "Calculating trajectory......" << endl;
    for (t = 0; t < 1577; t+=1) {  // 365*86400/1e4/2
        earthElements.Update_TrueAnomaly(t);
        point = earthElements.Update_Position();
        earthOrbit.push_back((Vector3){  // 存储轨迹点
            float(point.at(0,0)),
            float(point.at(1,0)),
            float(point.at(2,0)),
        });
    }
    for (t = 0; t < 2968; t+=1) {  // 687*86400/1e4/2
        marsElements.Update_TrueAnomaly(t);
        point = marsElements.Update_Position();
        marsOrbit.push_back((Vector3){  // 存储轨迹点
            float(point.at(0,0)),
            float(point.at(1,0)),
            float(point.at(2,0)),
        });
    }
    cout << "\nMars trajectory calculating finished." << endl;

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
                Draw_Trajectory(earthOrbit);  // 画3D轨迹
                Draw_Trajectory(marsOrbit);  // 画3D轨迹
            EndMode3D();
            DrawText(TextFormat("%2i FPS", GetFPS()), 0, 0, 20, LIME);
        EndDrawing();
    }
    CloseGraph();
    return 0;
}
