class KalmanFilter1d
{
private:
    float x{0}, p{4}, dt{0.004}, q{16}, r{9};

public:
    KalmanFilter1d() {}

    float calculate(float z, float x_dot)
    {
        // Prediction
        x += x_dot * dt;
        p += q * dt * dt;

        // Update
        float k = p / (p + r);
        x += k * (z - x);
        p *= (1 - k);

        return x;
    }
};
