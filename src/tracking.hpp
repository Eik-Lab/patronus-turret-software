#pragma once

#include <algorithm>
#include <cmath>

static constexpr float WEAPON_DX = 67.0f;
static constexpr float WEAPON_DY = -34.0f;
static constexpr float WEAPON_DZ = 0.0f;

static constexpr float TARGET_DISTANCE_MM = 100000.0f;

static constexpr float FRAME_CX = 1920.0f / 2.0f;
static constexpr float FRAME_CY = 1080.0f / 2.0f;

struct Point
{
    float cx;
    float cy;
};

struct AimAngles
{
    float pan;
    float tilt;
};

// TODO: on a later point we will implement a more complex Kalman filter
inline AimAngles compute_control_sensor(Point center, float Kp)
{
    float pan = Kp * (center.cx - FRAME_CX);
    float tilt = Kp * (center.cy - FRAME_CY);

    pan = std::clamp(pan, -1.0f, 1.0f);
    tilt = std::clamp(tilt, -1.0f, 1.0f);

    if (std::abs(pan) < 0.02f)
        pan = 0.0f;
    if (std::abs(tilt) < 0.02f)
        tilt = 0.0f;

    return {.pan = pan, .tilt = tilt};
}

inline AimAngles compute_control_weapon(float sensor_pan, float sensor_tilt)
{
    float pan_rad = sensor_pan * ((float)M_PI / 180.0f);
    float tilt_rad = sensor_tilt * ((float)M_PI / 180.0f);

    float tx = TARGET_DISTANCE_MM * std::cosf(tilt_rad) * std::cosf(pan_rad);
    float ty = TARGET_DISTANCE_MM * std::cosf(tilt_rad) * std::sinf(pan_rad);
    float tz = TARGET_DISTANCE_MM * std::sinf(tilt_rad);

    float vx = tx - WEAPON_DX;
    float vy = ty - WEAPON_DY;
    float vz = tz - WEAPON_DZ;

    float pan = std::atan2f(vy, vx) * (180.0f / (float)M_PI);
    float tilt = std::atan2f(vz, std::sqrtf(vx * vx + vy * vy)) * (180.0f / (float)M_PI);

    return {.pan = pan, .tilt = tilt};
}
