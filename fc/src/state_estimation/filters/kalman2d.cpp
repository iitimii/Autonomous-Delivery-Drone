#include "state_estimation/filters/kalman2d.hpp"

KalmanFilter2D::KalmanFilter2D() : dt{0.004f}
{
    F = {1, dt, 0, 1};
    G = {0.5f * dt * dt, dt};
    H = {1, 0};
    I = {1, 0, 0, 1};

    Q = G * ~G * 10.0f * 10.0f;
    R = {30 * 30};
    P = {1, 0, 0, 1};
    S = {0, 0};
}

void KalmanFilter2D::update(const float &pos, const float &acc)
{
    // F(0,1) = dt;
    G(0) = 0.5f * dt * dt;
    G(1) = dt;

    Acc = {acc};
    S = F * S + G * Acc;
    P = F * P * ~F + Q;
    L = H * P * ~H + R;
    K = P * ~H * BLA::Inverse(L);
    M = {pos};
    S = S + K * (M - H * S);
    P = (I - K * H) * P;

    p = S(0, 0);
    v = S(1, 0);
}

void KalmanFilter2D::update(const float &acc)
{
    float acc_dt = acc * dt;
    v = v + acc_dt;
    p = p + v * dt + 0.5f * acc_dt * dt;
}

void KalmanFilter2D::init(const float &pos)
{
    S = {pos, 0};
    P = {1, 0, 0, 1};
}