#include "state_estimation/state_estimation.hpp"
#include "communication/debugging.hpp"
#include "uav/global_stuff.hpp"
#include "uav/led.hpp"
#include <math.h>

StateEstimator::StateEstimator() : declination(-1.25), gps_start(false), X0(0), Y0(0), Z0(0) {}

void StateEstimator::setup()
{
    debugging::log("State Estimator setup");
    gps.setup();
    barometer.setup();
    barometer.calibrate();
    initial_pressure = barometer.get_initial_pressure();
    initial_altitude = barometer.get_initial_altitude();
    imu.setup();
    imu.calibrate();
    compass.setup();

    debugging::log("State Estimator setup complete");
    init();
}

void StateEstimator::lla2ecef()
{
    double lat_rad = latitude * DEG_TO_RAD;
    double lon_rad = longitude * DEG_TO_RAD;
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_lon = cos(lon_rad);
    double sin_lon = sin(lon_rad);

    double Rn = SEMI_MAJOR_AXIS_LENGTH_M / sqrt(fabs(1.0 - (ECC2 * sin_lat * sin_lat)));
    X = (Rn + gps_altitude) * cos_lat * cos_lon;
    Y = (Rn + gps_altitude) * cos_lat * sin_lon;
    Z = (Rn * (1.0 - ECC2) + gps_altitude) * sin_lat;
}

void StateEstimator::ecef2enu()
{
    double lat0_rad = initial_latitude * DEG_TO_RAD;
    double lon0_rad = initial_longitude * DEG_TO_RAD;

    double sin_lat = sin(lat0_rad);
    double cos_lat = cos(lat0_rad);
    double sin_lon = sin(lon0_rad);
    double cos_lon = cos(lon0_rad);

    double dx = X - X0;
    double dy = Y - Y0;
    double dz = Z - Z0;

    e = -sin_lon * dx + cos_lon * dy;
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
}

void StateEstimator::lla2enu()
{
    lla2ecef();
    ecef2enu();
}

void StateEstimator::update()
{
    imu.read();
    gps.read();
    barometer.read();
    compass.read();

    gx = imu.getGx(); // dps
    gy = imu.getGy();
    gz = imu.getGz();
    ax = imu.getAx();
    ay = imu.getAy();
    az = imu.getAz();
    temperature = imu.getTemperature();
    mx = compass.getMx();
    my = compass.getMy();
    mz = compass.getMz();

    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
    yaw += declination;
    yaw = 90.0 - yaw; // now 0° = East (ENU frame)
    yaw = remainder(yaw, 360.0);
    phi = roll;
    theta = pitch;
    psi = yaw;
    p = gx;
    q = gy;
    r = gz;

    pressure = barometer.get_pressure();
    absolute_altitude = 44330 * (1 - pow(pressure / 1013.25, 0.1903)); // meters
    altitude = absolute_altitude - initial_altitude;

    latitude = gps.getLatitude();
    longitude = gps.getLongitude();
    num_satelites = gps.getNumSats();
    fix_type = gps.getFixType();
    gps_altitude = gps.getAltitude();
    gps_vertical_speed = gps.getVerticalSpeed();

    // Initialize GPS reference position when we get first valid fix
    if (fix_type >= 2 && !gps_start)
    {
        led::on();
        initial_latitude = latitude;
        initial_longitude = longitude;
        lla2ecef();
        X0 = X;
        Y0 = Y;
        Z0 = Z;
        ecef2enu();
        gps_start = true;
        debugging::log("GPS reference position initialized");
    }

    // Convert current GPS position to local ENU coordinates
    float gps_x = 0, gps_y = 0;
    if (gps_start && fix_type >= 2)
    {
        lla2enu();
        gps_x = n; // North component goes to x
        gps_y = e; // East component goes to y
    }

    float roll_rad = roll * DEG_TO_RAD;
    float pitch_rad = pitch * DEG_TO_RAD;
    float yaw_rad = yaw * DEG_TO_RAD;
    float cos_roll = cos(roll_rad);
    float sin_roll = sin(roll_rad);
    float cos_pitch = cos(pitch_rad);
    float sin_pitch = sin(pitch_rad);
    float cos_yaw = cos(yaw_rad);
    float sin_yaw = sin(yaw_rad);

    // Transform accelerations from body to inertial frame
    x_d_dot = cos_pitch * cos_yaw * ax + cos_pitch * sin_yaw * ay - sin_pitch * az;
    y_d_dot = (-cos_roll * sin_yaw + sin_roll * sin_pitch * cos_yaw) * ax +
              (cos_roll * cos_yaw + sin_roll * sin_pitch * sin_yaw) * ay +
              (sin_roll * cos_pitch) * az;
    z_d_dot = -sin_pitch * ax + cos_pitch * sin_roll * ay + cos_pitch * cos_roll * az;
    z_d_dot -= 9.81;

    x_filter.update(gps_x, x_d_dot);
    y_filter.update(gps_y, y_d_dot);
    z_filter.update(altitude, z_d_dot);

    x = x_filter.getPos();
    y = y_filter.getPos();
    z = z_filter.getPos();
    x_dot = x_filter.getVel();
    y_dot = y_filter.getVel();
    z_dot = z_filter.getVel();
}

void StateEstimator::init()
{
    imu.read();
    barometer.read();
    compass.read();
    gps.read();

    gx = imu.getGx(); // dps
    gy = imu.getGy();
    gz = imu.getGz();
    ax = imu.getAx();
    ay = imu.getAy();
    az = imu.getAz();
    mx = compass.getMx();
    my = compass.getMy();
    mz = compass.getMz();

    filter.init(gx, gy, gz, ax, ay, az, mx, my, mz);
    x_filter.init(0.0f);
    y_filter.init(0.0f);
    z_filter.init(0.0f);
}

void StateEstimator::reset()
{
    imu.read();
    compass.read();

    gx = imu.getGx(); // dps
    gy = imu.getGy();
    gz = imu.getGz();
    ax = imu.getAx();
    ay = imu.getAy();
    az = imu.getAz();
    mx = compass.getMx();
    my = compass.getMy();
    mz = compass.getMz();

    filter.reset(gx, gy, gz, ax, ay, az, mx, my, mz);
}