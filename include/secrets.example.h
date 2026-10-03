#pragma once

// Sablon: másold le secrets.h néven, és töltsd ki a valós adatokkal.
// A secrets.h fájl .gitignore-ozva van, nem kerül verziókezelésbe.

struct WifiCred { const char* ssid; const char* password; };

static const WifiCred KNOWN_NETWORKS[] = {
  { "SSID_1",            "PASSWORD_1" },
  { "SSID_2",            "PASSWORD_2" },
  { "SSID_3",             "PASSWORD_3" }
};
static const int KNOWN_NETWORK_COUNT = sizeof(KNOWN_NETWORKS) / sizeof(KNOWN_NETWORKS[0]);

static const char* AP_SSID     = "WICKEDPAP";
static const char* AP_PASSWORD = "VALASSZ_SAJAT_AP_JELSZOT";
