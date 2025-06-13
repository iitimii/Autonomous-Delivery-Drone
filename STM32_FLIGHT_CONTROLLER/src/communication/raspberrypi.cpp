#include "communication/raspberrypi.hpp"
#include "communication/receiver.hpp"
#include "sensors/imu.hpp"
#include "attitude_estimation/attitude_estimation.hpp"
#include "uav/flight.hpp"
#include "actuators/outputs.hpp"
#include "actuators/motors.hpp"
#include "uav/drone.hpp"



HardwareSerial rpi::rpiSerial(PA10, PA9);

void rpi::setup()
{
    rpiSerial.begin(115200);
}

void rpi::send()
{
    send_imu_data();
}

void rpi::receive()
{
    // if (rpiSerial.available())
    // {
    //     double cmd = rpiSerial.readString().toDouble();
    // }

}

void rpi::send_imu_data()
{
    // float ax = attitude_estimator.ax;
    // float ay = attitude_estimator.ay;
    // float az = attitude_estimator.az;
    // float gx = attitude_estimator.gx;
    // float gy = attitude_estimator.gy;
    // float gz = attitude_estimator.gz;
    // float temp = attitude_estimator.temperature;

    // float roll = attitude_estimator.roll;
    // float pitch = attitude_estimator.pitch;
    // float yaw = attitude_estimator.yaw;

    //  String data = "ax: " + String(ax) + ", ay: " + String(ay) + ", az: " + String(az) + ", gx: " +
    //                       String(gx) + ", gy: " + String(gy) + ", gz: " + String(gz) + ", temp: " + String(temp);

    // String data = "pitch: " + String(pitch) + ", roll: " + String(roll) + ", yaw: " + String(yaw); 

    // String data = "mode: " + String(flight::mode) + ", roll_output: " + String(outputs::roll) + ", pitch_output: " + String(outputs::pitch) + ", yaw_output: " + String(outputs::yaw) + ", throttle_output: " + String(outputs::throttle); 

    // String data = "mode: " + String(flight::mode) + ", fl: " + String(motors::fl) + ", fr: " + String(motors::fr) + ", bl: " + String(motors::bl) + ", br: " + String(motors::br);

    // String data = "mode: " + String(flight::mode) + ", Loop Time: " + String(drone::loop_time_actual);

    // rpiSerial.println(data);
}

void rpi::send_receiver_data()
{

}