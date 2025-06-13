#ifndef COMPLEMENTARY_FILTER_HPP
#define COMPLEMENTARY_FILTER_HPP

class ComplementaryFilter
{
private:
    float _alpha;
    float roll;
    float pitch;
    float yaw;
    float dt;
    float roll_acc;
    float pitch_acc;

public:
    ComplementaryFilter();

    void update(const float &gx, const float &gy, const float &gz, const float &ax, const float &ay, const float &az);
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

#endif // COMPLEMENTARY_FILTER_HPP