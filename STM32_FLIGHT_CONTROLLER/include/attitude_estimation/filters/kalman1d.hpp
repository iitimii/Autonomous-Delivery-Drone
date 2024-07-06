#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

class KalmanFilter1D
{
private:
    float K, R, Q;
    float P_roll, P_pitch;
    float dt;

    float roll;
    float pitch;
    float yaw;

public:
    KalmanFilter1D();

    

    void update(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az);
    void setTimeStep(float new_dt);
    void reset(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az);

    float getRoll()
    {
        return roll;
    }
    float getPitch()
    {
        return pitch;
    }
    float getYaw()
    {
        return yaw;
    }
};

#endif // KALMAN_FILTER_HPP