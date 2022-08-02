#ifdef LIE_FLAT_WIFI_STA
const char* SSID = "Your SSID here";
const char* PASSWORD = "Your password here";
#elif defined LIE_FLAT_WIFI_AP
const char* AP_SSID = "lie-flat";
const char* AP_PASSWORD = "flat-lie";
#else
#error "LIE_FLAT_WIFI_STA or LIE_FLAT_WIFI_AP must be defined!"
#endif
