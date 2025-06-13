#include "sensors/gps.hpp"
#include "uav/led.hpp"
#include "communication/debugging.hpp"

#define GPS_RX_PIN 16 // Define RX pin
#define GPS_TX_PIN 17 // Define TX pin

HardwareSerial gpsSerial(2);

GPS::GPS() : latitude(0.0), longitude(0.0), num_satelites(0.0), fix_type(0.0)
{
}

void GPS::setup()
{
    debugging::log("GPS setup");
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(250);

    // Disable GPGSV messages by using the ublox protocol.
    uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    gpsSerial.write(Disable_GPGSV, 11);
    delay(350);
    // Set the refresh rate to 5Hz by using the ublox protocol.
    uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    gpsSerial.write(Set_to_5Hz, 14);
    delay(350);
    // Set the baud rate to 57.6kbps by using the ublox protocol.
    uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                                 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1};
    gpsSerial.write(Set_to_57kbps, 28);
    delay(200);

    gpsSerial.updateBaudRate(57600);
    delay(200);
}

void GPS::read()
{
    if (!gpsSerial.available())
        return;

    while (gpsSerial.available())
    {
        char c = gpsSerial.read();
        if (M8_Gps.encode(c))
        {
            altitude = M8_Gps.altitude;
            latitude = M8_Gps.latitude;
            longitude = M8_Gps.longitude;
            num_satelites = M8_Gps.sats_in_use;
            vertical_speed = M8_Gps.vert_speed;
            fix_type = M8_Gps.fix;
        }
    }
}

// void GPS::read()
// {
//     while (gpsSerial.available() && new_line_found == 0)
//     {
//         char read_serial_byte = gpsSerial.read();
//         if (read_serial_byte == '$')
//         {
//             for (message_counter = 0; message_counter <= 99; message_counter++)
//             {
//                 incomming_message[message_counter] = '-';
//             }
//             message_counter = 0;
//         }
//         else if (message_counter <= 99)
//             message_counter++;
//         incomming_message[message_counter] = read_serial_byte;
//         if (read_serial_byte == '*')
//             new_line_found = 1; // Every NMEA line end with a *. If this character is detected the new_line_found variable is set to 1.
//     }

//     // If the software has detected a new NMEA line it will check if it's a valid line that can be used.
//     if (new_line_found == 1)
//     {
//         new_line_found = 0; // Reset the new_line_found variable for the next line.
//         if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',') // When there is no GPS fix or latitude/longitude information available.
//         {
//             led::blink_once();

//             num_satelites = 0;
//             fix_type = 0;
//         }
//         // If the line starts with GA and if there is a GPS fix we can scan the line for the latitude, longitude and number of satellites.
//         if (incomming_message[4] == 'G' && incomming_message[5] == 'A' && (incomming_message[44] == '1' || incomming_message[44] == '2'))
//         {
//             latitude = ((int)incomming_message[19] - 48) * (long)10000000;   // Filter the minutes for the GGA line multiplied by 10.
//             latitude += ((int)incomming_message[20] - 48) * (long)1000000;   // Filter the minutes for the GGA line multiplied by 10.
//             latitude += ((int)incomming_message[22] - 48) * (long)100000;    // Filter the minutes for the GGA line multiplied by 10.
//             latitude += ((int)incomming_message[23] - 48) * (long)10000;     // Filter the minutes for the GGA line multiplied by 10.
//             latitude += ((int)incomming_message[24] - 48) * (long)1000;      // Filter the minutes for the GGA line multiplied by 10.
//             latitude += ((int)incomming_message[25] - 48) * (long)100;       // Filter the minutes for the GGA line multiplied by 10.
//             latitude += ((int)incomming_message[26] - 48) * (long)10;        // Filter the minutes for the GGA line multiplied by 10.
//             latitude /= (long)6;                                             // To convert the minutes to degrees we need to divide the minutes by 6.
//             latitude += ((int)incomming_message[17] - 48) * (long)100000000; // Add the degrees multiplied by 10.
//             latitude += ((int)incomming_message[18] - 48) * (long)10000000;  // Add the degrees multiplied by 10.
//             latitude /= 10;                                                  // Divide everything by 10.

//             longitude = ((int)incomming_message[33] - 48) * (long)10000000;    // Filter the minutes for the GGA line multiplied by 10.
//             longitude += ((int)incomming_message[34] - 48) * (long)1000000;    // Filter the minutes for the GGA line multiplied by 10.
//             longitude += ((int)incomming_message[36] - 48) * (long)100000;     // Filter the minutes for the GGA line multiplied by 10.
//             longitude += ((int)incomming_message[37] - 48) * (long)10000;      // Filter the minutes for the GGA line multiplied by 10.
//             longitude += ((int)incomming_message[38] - 48) * (long)1000;       // Filter the minutes for the GGA line multiplied by 10.
//             longitude += ((int)incomming_message[39] - 48) * (long)100;        // Filter the minutes for the GGA line multiplied by 10.
//             longitude += ((int)incomming_message[40] - 48) * (long)10;         // Filter the minutes for the GGA line multiplied by 10.
//             longitude /= (long)6;                                              // To convert the minutes to degrees we need to divide the minutes by 6.
//             longitude += ((int)incomming_message[30] - 48) * (long)1000000000; // Add the degrees multiplied by 10.
//             longitude += ((int)incomming_message[31] - 48) * (long)100000000;  // Add the degrees multiplied by 10.
//             longitude += ((int)incomming_message[32] - 48) * (long)10000000;   // Add the degrees multiplied by 10.
//             longitude /= 10;                                                   // Divide everything by 10.

//             if (incomming_message[28] == 'N')
//                 latitude_north = 1; // When flying north of the equator the latitude_north variable will be set to 1.
//             else
//                 latitude_north = 0; // When flying south of the equator the latitude_north variable will be set to 0.

//             if (incomming_message[42] == 'E')
//                 longitude_east = 1; // When flying east of the prime meridian the longitude_east variable will be set to 1.
//             else
//                 longitude_east = 0; // When flying west of the prime meridian the longitude_east variable will be set to 0.

//             num_satelites = ((int)incomming_message[46] - 48) * (long)10; // Filter the number of satillites from the GGA line.
//             num_satelites += (int)incomming_message[47] - 48;             // Filter the number of satillites from the GGA line.
//         }

//         // If the line starts with SA and if there is a GPS fix we can scan the line for the fix type (none, 2D or 3D).
//         if (incomming_message[4] == 'S' && incomming_message[5] == 'A')
//             fix_type = (int)incomming_message[9] - 48;

//         // TODO negate the latitude and logitude to embed the directions
//     }
// }
