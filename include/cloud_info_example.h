/*
COPY THIS TO include/cloud_info.h AND UPDATE THE VALUES LOCALLY
cloud_info.h IS IN THE .gitignore SO IT IS NOT COMMITTED TO VERSION CONTROL
THIS IS TO PREVENT ACCIDENTAL EXPOSURE OF SENSITIVE INFORMATION
*/ 
#include <WString.h>
// Update these values locally
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define SAS_TOKEN "" // Insert certificate here
// Root CA certificate for Azure IoT Hub
const char* root_ca = "";
// If necessary, update these values as well
String iothubName = ""; // Your hub name
String deviceName = "";    // Your device name
String url = "https://" + iothubName + ".azure-devices.net/devices/" +
             deviceName + "/messages/events?api-version=2021-04-12";