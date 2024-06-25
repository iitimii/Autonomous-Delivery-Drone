#include "wifi.hpp"

namespace wifi
{
    // Define the variables declared as extern in the header
    const char *ssid = "hacker";
    const char *password = ".JesusIsTheChrist@.";
    const char *esp_ssid = "Ground Station";
    const char *esp_password = "totheskies";

    int connect(const char *ssid, const char *password)
    {
        WiFi.begin(ssid, password);
        int retries = 0;
        while (WiFi.status() != WL_CONNECTED && retries < 20)
        {
            delay(1000);
            Serial.println("Connecting to WiFi...");
            ++retries;
        }
        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.println("Connected to WiFi");
            Serial.println(WiFi.localIP());
            return 0; // Success
        }
        else
        {
            Serial.println("Failed to connect to WiFi");
            return 1; // Failure
        }
    }

    int DNS_setup()
    {
        const char *host = "gs";
        int retries = 0;
        while (!MDNS.begin(host) && retries < 5)
        {
            Serial.println("Error setting up MDNS responder!");
            ++retries;
            delay(1000);
        }

        if (retries >= 5)
        {
            Serial.println("Failed to set up MDNS responder after 5 retries.");
            return -1; // Indicate failure
        }

        Serial.print(host);
        Serial.println(".local");
        return 0;
    }

}
