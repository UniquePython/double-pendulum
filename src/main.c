#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>

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

#define TO_RAD(deg) ((float)(deg) * DEG2RAD)

#define G_ACCEL 9.8f

// ─── Geometry ────────────────────────────────────────────────────────────────
//
// Coordinate convention: angle = 0 is straight down, positive Y is screen-down.

Vector2 GetEndPos(Vector2 startPos, float angle, float length)
{
    return (Vector2){startPos.x + length * sinf(angle), startPos.y + length * cosf(angle)};
}

// ─── Config ───────────────────────────────────────────────────────────────────

typedef struct
{
    float length1, length2;
    float mass1, mass2;
} PendulumConfig;

// ─── Drawing ──────────────────────────────────────────────────────────────────

static Color s_trailColors[TRAIL_LEN];

void InitTrailColors(void)
{
    for (int i = 0; i < TRAIL_LEN; i++)
    {
        float t = (float)i / TRAIL_LEN;
        s_trailColors[i] = Fade(RED, t);
    }
}

void DrawPendulumArm(Vector2 startPos, float angle, float length)
{
    Vector2 endPos = GetEndPos(startPos, angle, length);
    DrawLineEx(startPos, endPos, STRING_THICKNESS, RAYWHITE);
    DrawCircleV(endPos, MASS_RADIUS, RED);
}

void DrawDoublePendulum(Vector2 pivotPos, float angle1, float angle2, float length1, float length2)
{
    Vector2 midPos = GetEndPos(pivotPos, angle1, length1);
    // Draw second arm first so the first arm's mass is drawn on top
    DrawPendulumArm(midPos, angle2, length2);
    DrawPendulumArm(pivotPos, angle1, length1);
}

void DrawTrail(Vector2 points[TRAIL_LEN], int index)
{
    for (int i = 0; i < TRAIL_LEN; i++)
    {
        int idx = (index + i) % TRAIL_LEN;
        DrawCircleV(points[idx], TRAIL_THICKNESS, s_trailColors[i]);
    }
}

void DrawControls(bool paused)
{
    DrawText("SPACE: Reset  |  P: Pause", 10, HEIGHT - 24, 16, GRAY);
    if (paused)
        DrawText("PAUSED", WIDTH / 2 - 36, HEIGHT / 2, 24, YELLOW);
}

// ─── Physics ──────────────────────────────────────────────────────────────────

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

Derivative ComputeDerivatives(PendulumConfig cfg, State s)
{
    float delta = s.angle1 - s.angle2;
    float sinDelta = sinf(delta);
    float cosDelta = cosf(delta);

    // ── Angular acceleration of arm 1 ──
    float gt1 = -G_ACCEL * (2 * cfg.mass1 + cfg.mass2) * sinf(s.angle1);
    float gt2 = -cfg.mass2 * G_ACCEL * sinf(s.angle1 - 2 * s.angle2);
    float vt1 = -2 * cfg.mass2 * sinDelta * (s.angularVel2 * s.angularVel2 * cfg.length2 + s.angularVel1 * s.angularVel1 * cfg.length1 * cosDelta);

    float den1 = cfg.length1 * (2 * cfg.mass1 + cfg.mass2 - cfg.mass2 * cosf(2 * delta));
    if (fabsf(den1) < 1e-6f)
        den1 = copysignf(1e-6f, den1);

    float angularAcc1 = (gt1 + gt2 + vt1) / den1;

    // ── Angular acceleration of arm 2 ──
    float gt3 = (cfg.mass1 + cfg.mass2) * G_ACCEL * cosf(s.angle1);
    float vt2 = s.angularVel1 * s.angularVel1 * cfg.length1 * (cfg.mass1 + cfg.mass2) + s.angularVel2 * s.angularVel2 * cfg.length2 * cfg.mass2 * cosDelta;
    float num2 = 2 * sinDelta * (vt2 + gt3);

    float den2 = cfg.length2 * (2 * cfg.mass1 + cfg.mass2 - cfg.mass2 * cosf(2 * delta));
    if (fabsf(den2) < 1e-6f)
        den2 = copysignf(1e-6f, den2);

    float angularAcc2 = num2 / den2;

    return (Derivative){
        .dAngle1 = s.angularVel1,
        .dAngle2 = s.angularVel2,
        .dAngularVel1 = angularAcc1,
        .dAngularVel2 = angularAcc2,
    };
}

State AddState(State s, Derivative d, float dt)
{
    return (State){
        .angle1 = s.angle1 + d.dAngle1 * dt,
        .angle2 = s.angle2 + d.dAngle2 * dt,
        .angularVel1 = s.angularVel1 + d.dAngularVel1 * dt,
        .angularVel2 = s.angularVel2 + d.dAngularVel2 * dt,
    };
}

State RK4Step(State s, float dt, PendulumConfig cfg)
{
    Derivative k1 = ComputeDerivatives(cfg, s);
    Derivative k2 = ComputeDerivatives(cfg, AddState(s, k1, dt * 0.5f));
    Derivative k3 = ComputeDerivatives(cfg, AddState(s, k2, dt * 0.5f));
    Derivative k4 = ComputeDerivatives(cfg, AddState(s, k3, dt));

    return (State){
        .angle1 = s.angle1 + (dt / 6.0f) * (k1.dAngle1 + 2 * k2.dAngle1 + 2 * k3.dAngle1 + k4.dAngle1),
        .angle2 = s.angle2 + (dt / 6.0f) * (k1.dAngle2 + 2 * k2.dAngle2 + 2 * k3.dAngle2 + k4.dAngle2),
        .angularVel1 = s.angularVel1 + (dt / 6.0f) * (k1.dAngularVel1 + 2 * k2.dAngularVel1 + 2 * k3.dAngularVel1 + k4.dAngularVel1),
        .angularVel2 = s.angularVel2 + (dt / 6.0f) * (k1.dAngularVel2 + 2 * k2.dAngularVel2 + 2 * k3.dAngularVel2 + k4.dAngularVel2),
    };
}

// ─── Trail ────────────────────────────────────────────────────────────────────

typedef struct
{
    Vector2 points[TRAIL_LEN];
    int index;
} Trail;

void ResetTrail(Trail *trail)
{
    memset(trail->points, 0, sizeof(trail->points));
    trail->index = 0;
}

void UpdateTrail(Trail *trail, Vector2 pivotPos, State state, PendulumConfig cfg)
{
    Vector2 midPos = GetEndPos(pivotPos, state.angle1, cfg.length1);
    Vector2 endPos = GetEndPos(midPos, state.angle2, cfg.length2);
    trail->points[trail->index] = endPos;
    trail->index = (trail->index + 1) % TRAIL_LEN;
}

// ─── Simulation ───────────────────────────────────────────────────────────────

typedef struct
{
    State state;
    Trail trail;
    bool paused;
} Simulation;

void ResetSimulation(Simulation *sim)
{
    SetRandomSeed(time(NULL));

    sim->state.angle1 = TO_RAD(GetRandomValue(-90, 90));
    sim->state.angle2 = TO_RAD(GetRandomValue(-180, 180));
    sim->state.angularVel1 = 0.0f;
    sim->state.angularVel2 = 0.0f;

    ResetTrail(&sim->trail);
    sim->paused = false;
}

void StepSimulation(Simulation *sim, PendulumConfig cfg)
{
    if (sim->paused)
        return;

    float frameDt = GetFrameTime() * TIME_SCALE;

    // Cap individual step size for integrator stability
    const float maxDt = 1.0f / (float)GetMonitorRefreshRate(GetCurrentMonitor());

    while (frameDt > 0.0f)
    {
        float step = (frameDt > maxDt) ? maxDt : frameDt;
        sim->state = RK4Step(sim->state, step, cfg);
        frameDt -= step;
    }
}

void HandleInput(Simulation *sim)
{
    if (IsKeyPressed(KEY_SPACE))
        ResetSimulation(sim);
    if (IsKeyPressed(KEY_P))
        sim->paused = !sim->paused;
}

// ─── Entry point ──────────────────────────────────────────────────────────────

int main(void)
{
    InitWindow(WIDTH, HEIGHT, "Double Pendulum");
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));

    InitTrailColors();

    const Vector2 pivotPos = {WIDTH / 2.0f, 0};

    const PendulumConfig cfg = {
        .length1 = L1,
        .length2 = L2,
        .mass1 = M1,
        .mass2 = M2,
    };

    Simulation sim;
    ResetSimulation(&sim);

    while (!WindowShouldClose())
    {
        HandleInput(&sim);

        StepSimulation(&sim, cfg);
        if (!sim.paused)
            UpdateTrail(&sim.trail, pivotPos, sim.state, cfg);

        BeginDrawing();
        ClearBackground(BLACK);
        DrawTrail(sim.trail.points, sim.trail.index);
        DrawDoublePendulum(pivotPos, sim.state.angle1, sim.state.angle2, cfg.length1, cfg.length2);
        DrawControls(sim.paused);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}