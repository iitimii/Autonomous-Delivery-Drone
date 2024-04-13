class PIDController
{
private:
public:
    float kp, ki, kd, dt;
    float i = 0;
    float prev_error = 0;

    PIDController(float kp, float ki, float kd, float dt)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->dt = dt;

        this->i = 0;
        this->prev_error = 0;
    }

    float calculate(int setpoint, float input)
    {
        float error = setpoint - input;
        float p = kp * error;
        i += ki * (error + prev_error)/2 * dt;
        if (i > 400)
            i = 400;
        else if (i < -400)
            i = -400;
        float d = kd * (error - prev_error) / dt;
        prev_error = error;
        return p + i + d;
    }

    void reset()
    {
        i = 0;
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
