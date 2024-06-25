#ifndef WIFI_CONNECT_HPP
#define WIFI_CONNECT_HPP

#include <WiFi.h>
#include <ESPmDNS.h>



namespace wifi
{
    extern const char* ssid;
    extern const char* password;
    extern const char* esp_ssid;
    extern const char* esp_password;

    int connect(const char* ssid, const char* password);
    int DNS_setup();
}

#endif // WIFI_CONNECT_HPP
