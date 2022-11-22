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
    Orbital_Elements earthElements(MU_EARTH, Mat(6, 1, vecdble{
        149.6, 0.01667835, 0, 49.558*DEG2RAD, 103.30275*DEG2RAD, 1.57}));
    Orbital_Elements marsElements(MU_MARS, Mat(6, 1, vecdble{
        227.9, 0.09341233, 1.850*DEG2RAD, 49.558*DEG2RAD, 286.5*DEG2RAD, 0}));
    double t;
    Mat point;
    cout << "Calculating trajectory......" << endl;
    for (t = 0; t < 3154000; t+=1000) {
        earthElements.Update_TrueAnomaly(t);
        point = earthElements.Update_Position();
        earthOrbit.push_back((Vector3){  // 存储轨迹点
            float(point.at(0,0)), 
            float(point.at(1,0)), 
            float(point.at(2,0)),
        });
    }
    for (t = 0; t < 5935000*2; t+=1000) {
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
