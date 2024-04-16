class PIDController
{
private:
    float kp, ki, kd;
    float integral, prev_error;
    const float i_max;
    float dt;
    
public:

    PIDController(float kp, float ki, float kd, float dt = 0.004, float i_max = 400)
    : kp(kp), ki(ki), kd(kd), i_max(i_max), integral(0), prev_error(0), dt(dt)
    {

    }

    float calculate(int setpoint, float input)
    {
        float error = setpoint - input;
        float proportional = kp * error;

        if (ki == 0.0 && kd == 0.0) return proportional;
    
        integral += ki * ((error + prev_error)/2) * dt;
        // i += ki * error * dt;
        // Anti-windup: Limit integral term
        if (integral > i_max)
            integral = i_max;
        else if (integral < -i_max)
            integral = -i_max;
        float derivative = kd * (error - prev_error) / dt;
        prev_error = error;
        return proportional + integral + derivative;
    }

    void reset()
    {
        integral = 0;
        prev_error = 0;
    }

    void setPIDgains(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    int channel_setpoint(int channel_value)
    {
        int setpoint{0};
        if (channel_value > 1508)
            setpoint = channel_value - 1508;
        else if (channel_value < 1492)
            setpoint = channel_value - 1492;
        return setpoint;
    }
};
