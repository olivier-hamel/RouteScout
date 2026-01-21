#pragma once

#include <cmath>

namespace robot::angles
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;

    inline float wrapPi(float rad)
    {
        while (rad > kPi)
        {
            rad -= kTwoPi;
        }
        while (rad < -kPi)
        {
            rad += kTwoPi;
        }
        return rad;
    }

    // Unwrap a wrapped angle (typically in [-pi,pi]) into a continuous angle series.
    // prev_continuous can be any value; new_wrapped is assumed wrapped to [-pi,pi].
    inline float unwrap(float prev_continuous, float new_wrapped)
    {
        const float prev_wrapped = wrapPi(prev_continuous);
        const float delta = wrapPi(new_wrapped - prev_wrapped);
        return prev_continuous + delta;
    }

    inline float shortestDistance(float from_rad, float to_rad)
    {
        return wrapPi(to_rad - from_rad);
    }
}
