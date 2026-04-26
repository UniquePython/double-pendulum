#include <stdio.h>
#include <math.h>
#include <string.h>

#include <raylib.h>

#define WIDTH 900
#define HEIGHT 600

#define TIME_SCALE 10.0f

#define TRAIL_LEN 500
#define TRAIL_THICKNESS 2

#define STRING_THICKNESS 4
#define MASS_RADIUS 15

#define L1 240
#define L2 120

#define M1 1
#define M2 2

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

void DrawTrail(Vector2 trail[TRAIL_LEN], int trailIndex)
{
    for (int i = 0; i < TRAIL_LEN; i++)
    {
        int idx = (trailIndex + i) % TRAIL_LEN;
        float t = (float)i / TRAIL_LEN;
        Color c = Fade(RED, t);
        DrawCircleV(trail[idx], TRAIL_THICKNESS, c);
    }
}

typedef struct
{
    float angle1, angle2;
    float angularVel1, angularVel2;
} State;

typedef struct
{
    float dAngle1, dAngle2;
    float dAngularVel1, dAngularVel2;
} Derivative;

void UpdateTrail(Vector2 trail[TRAIL_LEN], int *trailIndex, Vector2 startPos, float angle1, float angle2, float length1, float length2)
{
    Vector2 midPos = GetEndPos(startPos, angle1, length1);
    Vector2 endPos = GetEndPos(midPos, angle2, length2);

    trail[*trailIndex] = endPos;
    *trailIndex = (*trailIndex + 1) % TRAIL_LEN;
}

Derivative ComputeDerivatives(float length1, float length2, float mass1, float mass2, State s)
{
    float delta = s.angle1 - s.angle2;

    float sinDelta = sinf(delta);
    float cosDelta = cosf(delta);

    // Acceleration 1
    float gravityTerm1 = -g * (2 * mass1 + mass2) * sinf(s.angle1);
    float gravityTerm2 = -mass2 * g * sinf(s.angle1 - 2 * s.angle2);

    float velocityTerm1 = -2 * mass2 * sinDelta * (s.angularVel2 * s.angularVel2 * length2 + s.angularVel1 * s.angularVel1 * length1 * cosDelta);

    float denominator1 = length1 * (2 * mass1 + mass2 - mass2 * cosf(2 * delta));

    float angularAcc1 = (gravityTerm1 + gravityTerm2 + velocityTerm1) / denominator1;

    // Acceleration 2
    float gravityTerm3 = (mass1 + mass2) * g * cosf(s.angle1);

    float velocityTerm2 = s.angularVel1 * s.angularVel1 * length1 * (mass1 + mass2) + s.angularVel2 * s.angularVel2 * length2 * mass2 * cosDelta;

    float numerator2 = 2 * sinDelta * (velocityTerm2 + gravityTerm3);

    float denominator2 = length2 * (2 * mass1 + mass2 - mass2 * cosf(2 * delta));

    float angularAcc2 = numerator2 / denominator2;

    Derivative d;

    d.dAngle1 = s.angularVel1;
    d.dAngle2 = s.angularVel2;

    d.dAngularVel1 = angularAcc1;
    d.dAngularVel2 = angularAcc2;

    return d;
}

State AddState(State s, Derivative d, float dt)
{
    State result;

    result.angle1 = s.angle1 + d.dAngle1 * dt;
    result.angle2 = s.angle2 + d.dAngle2 * dt;
    result.angularVel1 = s.angularVel1 + d.dAngularVel1 * dt;
    result.angularVel2 = s.angularVel2 + d.dAngularVel2 * dt;

    return result;
}

State RK4Step(State s, float dt, float length1, float length2, float mass1, float mass2)
{
    Derivative k1 = ComputeDerivatives(length1, length2, mass1, mass2, s);

    Derivative k2 = ComputeDerivatives(length1, length2, mass1, mass2, AddState(s, k1, dt * 0.5f));

    Derivative k3 = ComputeDerivatives(length1, length2, mass1, mass2, AddState(s, k2, dt * 0.5f));

    Derivative k4 = ComputeDerivatives(length1, length2, mass1, mass2, AddState(s, k3, dt));

    State result;

    result.angle1 = s.angle1 + (dt / 6.0f) * (k1.dAngle1 + 2 * k2.dAngle1 + 2 * k3.dAngle1 + k4.dAngle1);
    result.angle2 = s.angle2 + (dt / 6.0f) * (k1.dAngle2 + 2 * k2.dAngle2 + 2 * k3.dAngle2 + k4.dAngle2);

    result.angularVel1 = s.angularVel1 + (dt / 6.0f) * (k1.dAngularVel1 + 2 * k2.dAngularVel1 + 2 * k3.dAngularVel1 + k4.dAngularVel1);
    result.angularVel2 = s.angularVel2 + (dt / 6.0f) * (k1.dAngularVel2 + 2 * k2.dAngularVel2 + 2 * k3.dAngularVel2 + k4.dAngularVel2);

    return result;
}

void ResetSimulation(State *state, Vector2 trail[TRAIL_LEN], int *trailIndex)
{
    state->angle1 = DEG(GetRandomValue(-90, 90));
    state->angle2 = DEG(GetRandomValue(-120, 120));

    state->angularVel1 = 0.0f;
    state->angularVel2 = 0.001f;

    memset(trail, 0, sizeof(Vector2) * TRAIL_LEN);
    *trailIndex = 0;
}

int main(void)
{
    InitWindow(WIDTH, HEIGHT, "Double Pendulum");
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));

    Vector2 trail[TRAIL_LEN];
    memset(trail, 0, sizeof(trail));
    int trailIndex = 0;

    Vector2 startPos = (Vector2){WIDTH / 2, 0};

    float length1, length2, mass1, mass2;

    length1 = L1;
    length2 = L2;

    mass1 = M1;
    mass2 = M2;

    State state;

    ResetSimulation(&state, trail, &trailIndex);

    while (!WindowShouldClose())
    {
        float frameDt = GetFrameTime() * TIME_SCALE;
        float maxDt = 0.006944f;

        while (frameDt > 0.0f)
        {
            if (IsKeyPressed(KEY_SPACE))
                ResetSimulation(&state, trail, &trailIndex);

            float step = frameDt > maxDt ? maxDt : frameDt;
            state = RK4Step(state, step, length1, length2, mass1, mass2);
            frameDt -= step;
        }

        UpdateTrail(trail, &trailIndex, startPos, state.angle1, state.angle2, length1, length2);

        BeginDrawing();
        ClearBackground(BLACK);

        DrawTrail(trail, trailIndex);
        DrawDoublePendulum(startPos, state.angle1, state.angle2, length1, length2);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
