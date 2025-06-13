#ifndef STATE_ESTIMATION_HPP
#define STATE_ESTIMATION_HPP
#include <cstdint>
#include "sensors/imu.hpp"
#include "filters/madgwick.hpp"
#include "filters/complementary.hpp"
#include "filters/kalman1d.hpp"
#include "filters/kalman2d.hpp"
#include "filters/ekf.hpp"
#include "sensors/barometer.hpp"
#include "sensors/compass.hpp"
#include "sensors/gps.hpp"

class StateEstimator
{
public:
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    float  temperature, acc_resultant;

    // Barometer
    float initial_altitude, absolute_altitude, altitude;
    float pressure, initial_pressure;

    // Compass
    float heading, declination;

    // GPS
    double latitude, longitude, gps_altitude, gps_vertical_speed;
    double initial_latitude, initial_longitude;
    uint8_t num_satelites, fix_type;
    bool gps_start;

    // ECEF coordinates
    double X0, Y0, Z0;
    double X, Y, Z;

    // ENU coordinates
    double e, n, u;

    static constexpr double SEMI_MAJOR_AXIS_LENGTH_M = 6378137.0;
    /* Flattening */
    static constexpr double FLATTENING = 1.0 / 298.257223563;
    /* Semi-minor axis, m (derived) */
    static constexpr double SEMI_MINOR_AXIS_LENGTH_M = 6356752.3142;
    /* First eccentricity (derived) */
    static constexpr double ECC = 8.1819190842622e-2;
    /* First eccentricity, squared (derived) */
    static constexpr double ECC2 = 6.69437999014e-3;
    /* Angular velocity of the Earth, rad/s */
    static constexpr double WE_RADPS = 7292115.0e-11;
    /* Angular velocity of the Earth according to ICD-GPS-200, rad/s */
    static constexpr double WE_GPS_RADPS = 7292115.1467e-11;
    /* Earth's Gravitational Constant, m^3/s^2 */
    static constexpr double GM_M3PS2 = 3986004.418e8;
    /* Earth's Gravitational Constant according to ICD-GPS-200, m^3/s^2 */
    static constexpr double GM_GPS_M3PS2 = 3986005.0e8;
        
    float roll, pitch, yaw;

    float x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r;
    float x_d_dot, y_d_dot, z_d_dot; 

    IMU imu;
    Compass compass;
    Barometer barometer;
    GPS gps;

    // KalmanFilter1D filter;
    Madgwick filter;
    // EKF filter;

    KalmanFilter2D x_filter;
    KalmanFilter2D y_filter;
    KalmanFilter2D z_filter;

    StateEstimator();
    void setup();
    void update();
    void init();
    void reset();

private:
    void lla2ecef();
    void ecef2enu();
    void lla2enu();
};

extern StateEstimator state_estimator;

#endif // STATE_ESTIMATION_HPP