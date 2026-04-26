#include <stdio.h>

#include <raylib.h>

#define WIDTH 900
#define HEIGHT 600

int main(void)
{
    InitWindow(WIDTH, HEIGHT, "Double Pendulum");
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));

    while (!WindowShouldClose())
    {
        BeginDrawing();

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
