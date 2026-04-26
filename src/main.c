#include <stdio.h>
#include <math.h>

#include <raylib.h>

#define WIDTH 900
#define HEIGHT 600

#define STRING_THICKNESS 4
#define MASS_RADIUS 15

#define DEG(deg) ((deg) * DEG2RAD)

void DrawPendulum(float length, Vector2 startPos, float angle)
{
    Vector2 endPos = (Vector2){startPos.x + length * sinf(angle), startPos.y + length * cosf(angle)};

    DrawLineEx(startPos, endPos, STRING_THICKNESS, RAYWHITE);
    DrawCircleV(endPos, MASS_RADIUS, RED);
}

int main(void)
{
    InitWindow(WIDTH, HEIGHT, "Double Pendulum");
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));

    Vector2 startPos = (Vector2){WIDTH / 2, 0};

    while (!WindowShouldClose())
    {
        BeginDrawing();

        DrawPendulum(150, startPos, DEG(0));

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
