#include <stdio.h>
#include <math.h>

#include <raylib.h>

#define WIDTH 900
#define HEIGHT 600

#define STRING_THICKNESS 4
#define MASS_RADIUS 15

#define L1 250
#define L2 200

#define DEG(deg) ((deg) * DEG2RAD)

Vector2 GetEndPos(Vector2 startPos, float angle, float length)
{
    return (Vector2){startPos.x + length * sinf(angle), startPos.y + length * cosf(angle)};
}

void DrawPendulum(Vector2 startPos, float angle, float length)
{
    Vector2 endPos = GetEndPos(startPos, angle, length);

    DrawLineEx(startPos, endPos, STRING_THICKNESS, RAYWHITE);
    DrawCircleV(endPos, MASS_RADIUS, RED);
}

void DrawDoublePendulum(Vector2 startPos, float angle1, float angle2, float length1, float length2)
{
    // Draw second pendulum first
    Vector2 midPos = GetEndPos(startPos, angle1, length1);

    DrawPendulum(midPos, angle2, length2);

    // Draw first pendulum after second (in order to not draw over mass)
    DrawPendulum(startPos, angle1, length1);
}

int main(void)
{
    InitWindow(WIDTH, HEIGHT, "Double Pendulum");
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));

    Vector2 startPos = (Vector2){WIDTH / 2, 0};

    while (!WindowShouldClose())
    {
        BeginDrawing();

        DrawDoublePendulum(startPos, DEG(30), DEG(-60), L1, L2);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
