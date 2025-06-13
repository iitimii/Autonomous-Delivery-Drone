#include <Arduino.h>
#include <math.h>
#include <BasicLinearAlgebra.h>

class KalmanFilter2D
{
private:
    BLA::Matrix<2, 2> F;
    BLA::Matrix<2, 1> G;
    BLA::Matrix<2, 2> P;
    BLA::Matrix<2, 2> Q;
    BLA::Matrix<2, 1> S;
    BLA::Matrix<1, 2> H;
    BLA::Matrix<2, 2> I;
    BLA::Matrix<1, 1> Acc;
    BLA::Matrix<2, 1> K;
    BLA::Matrix<1, 1> R;
    BLA::Matrix<1, 1> L;
    BLA::Matrix<1, 1> M;


    float dt, p, v;

public:
    KalmanFilter2D();
    void update(const float &pos, const float &acc);
    void update(const float &acc);
    void init(const float &pos);

    // Getters
    inline float getPos() const { return p; }
    inline float getVel() const { return v; }
};