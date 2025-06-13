#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

class KalmanFilter1D
{
private:
    float K, R, R_yaw, Q;
    float P_roll, P_pitch, P_yaw;
    float dt;

    float roll;
    float pitch;
    float yaw;

    float mag_declination;
    float mag_offset;

public:
    KalmanFilter1D();

    

    void update(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az, const float &mx, const float &my, const float &mz);
    void setTimeStep(float new_dt);
    void init(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az, const float &mx, const float &my, const float &mz);
    void reset(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az, const float &mx, const float &my, const float &mz);

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

    void setRoll(const float &x)
    {
        roll = x;
    }

    void setPitch(const float &x)
    {
        pitch = x;
    }

    void setYaw(const float &x)
    {
        yaw = x;
    }
};

#endif // KALMAN_FILTER_HPP