#include <stdio.h>
#include <math.h>

#include <raylib.h>

#define WIDTH 900
#define HEIGHT 600

#define STRING_THICKNESS 4
#define MASS_RADIUS 15

#define L1 250
#define L2 200

#define M1 1
#define M2 1

#define DEG(deg) ((float)(deg) * DEG2RAD)

#define g 9.8f

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

void StepSimulation(float length1, float length2, float mass1, float mass2, float *angle1, float *angle2, float *dAngle1, float *dAngle2, float *d2Angle1, float *d2Angle2, float dt)
{
    float delta = *angle1 - *angle2;

    float sinDelta = sinf(delta);
    float cosDelta = cosf(delta);

    // Angular acceleration 1
    float gravityTerm1 = -g * (2 * mass1 + mass2) * sinf(*angle1);
    float gravityTerm2 = -mass2 * g * sinf((*angle1) - 2 * (*angle2));

    float velocityTerm1 = -2 * mass2 * sinDelta * ((*dAngle2) * (*dAngle2) * length2 + (*dAngle1) * (*dAngle1) * length1 * cosDelta);

    float denominator1 = length1 * (2 * mass1 + mass2 - mass2 * cosf(2 * delta));

    *d2Angle1 = (gravityTerm1 + gravityTerm2 + velocityTerm1) / denominator1;

    // Angular acceleration 2
    float gravityTerm3 = (mass1 + mass2) * g * cosf((*angle1));

    float velocityTerm2 = (*dAngle1) * (*dAngle1) * length1 * (mass1 + mass2) + (*dAngle2) * (*dAngle2) * length2 * mass2 * cosDelta;

    float numerator2 = 2 * sinDelta * (velocityTerm2 + gravityTerm3);

    float denominator2 = length2 * (2 * mass1 + mass2 - mass2 * cosf(2 * delta));

    *d2Angle2 = numerator2 / denominator2;

    // Angular velocity
    *dAngle1 += *d2Angle1 * dt;
    *dAngle2 += *d2Angle2 * dt;

    // Angle
    *angle1 += *dAngle1 * dt;
    *angle2 += *dAngle2 * dt;
}

int main(void)
{
    InitWindow(WIDTH, HEIGHT, "Double Pendulum");
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));

    Vector2 startPos = (Vector2){WIDTH / 2, 0};

    float length1, length2, mass1, mass2, angle1, angle2, dAngle1, dAngle2, d2Angle1, d2Angle2;

    length1 = L1;
    length2 = L2;

    mass1 = M1;
    mass2 = M2;

    angle1 = DEG(GetRandomValue(-90, 90));
    angle2 = DEG(GetRandomValue(-90, 90));

    dAngle1 = 0;
    dAngle2 = 0;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        StepSimulation(length1, length2, mass1, mass2, &angle1, &angle2, &dAngle1, &dAngle2, &d2Angle1, &d2Angle2, dt);

        BeginDrawing();
        ClearBackground(BLACK);
        DrawDoublePendulum(startPos, angle1, angle2, length1, length2);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}
