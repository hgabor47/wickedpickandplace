
# Wicked Pick and Place

ESP32-WROVER-B (TTGO T7 V1.3) alapú polargraph pick-and-place vezérlő. 2 léptetőmotor
(A4988), 1 saját tekercselésű elektromágnes (IBT-2/BTS7960 PWM-vezérléssel), 8 gomb
6-jegyű kód beviteléhez, WiFi STA/AP fallback, webes felület + kétféle OTA feltöltés.

## Fejlesztői környezet

- PlatformIO (VS Code)
- `board = esp-wrover-kit`, 4MB flash, PSRAM (ellenőrizve: ~4MB PSRAM jelen van)
- Framework: Arduino

### Szükséges könyvtárak (`platformio.ini`)

```ini
lib_deps =
    gin66/FastAccelStepper
    ESP32Async/AsyncTCP @ 3.3.2
    ESP32Async/ESPAsyncWebServer @ 3.6.0
    ayushsharma82/ElegantOTA
build_flags =
    -DBOARD_HAS_PSRAM
    -mfix-esp32-psram-cache-issue
    -DELEGANTOTA_USE_ASYNC_WEBSERVER=1
```

## WiFi viselkedés

Induláskor a firmware sorban megpróbál csatlakozni a következő hálózatokhoz (max 8mp/hálózat):

1. `HGPLSOFT`
2. `HGPLSOFT_EXT2.4G`
3. `HGPLSOFT2`

Ha egyik sem elérhető, saját hozzáférési pontot indít: **SSID: `WICKEDPAP`**.

Mindkét esetben elérhető a `http://wickedpickandplace.local` címen (mDNS), STA módban
a router kliens-listájában is `wickedpickandplace` néven jelenik meg.

---

## OTA (vezeték nélküli) feltöltés — két mód

Az eszköz **egyszerre mindkét OTA-módszert** támogatja, egymás mellett futnak.

### Mód A — PlatformIO "Upload" gomb hálózaton keresztül (ArduinoOTA)

Ez ugyanaz a felhasználói élmény, mint az USB-s feltöltés, csak WiFi-n megy.

**Első feltöltés mindig USB-n keresztül kell történjen** — az eszközön futnia kell
már egy olyan firmware-nek, ami tartalmazza az `ArduinoOTA` kódot, különben nincs
mit "meghallgatnia" a hálózaton.

**Külön PlatformIO környezet hozzáadása** az OTA-feltöltéshez (tedd a
`platformio.ini`-be a meglévő `[env:wicked_pickandplace]` mellé):

```ini
[env:wicked_pickandplace_ota]
extends = env:wicked_pickandplace
upload_protocol = espota
upload_port = wickedpickandplace.local
; ha az mDNS nem oldódik fel megbizhatoan a haloizatodon, hasznald helyette
; az eszkoz IP-cimet közvetlenul, pl.:
; upload_port = 192.168.1.123
```

**Feltöltés OTA-n keresztül:**

- VS Code-ban: válaszd ki alul a `wicked_pickandplace_ota` környezetet, majd nyomd
  meg a szokásos Upload gombot
- Terminálból:
  ```bash
  pio run -e wicked_pickandplace_ota -t upload
  ```

**Jelszó (ajánlott élesben)**: a `main.cpp`-ben van egy kikommentezett sor:

```cpp
// ArduinoOTA.setPassword("VALASSZ_JELSZOT");
```

Vedd ki a kommentet, adj meg egy jelszót, és a `platformio.ini`-ben a megfelelő
környezethez add hozzá:

```ini
upload_flags = --auth=VALASSZ_JELSZOT
```

Jelszó nélkül bárki, aki eléri a hálózatot (vagy a `WICKEDPAP` fallback AP-ot),
tud firmware-t feltölteni az eszközre — hobbiprojektként elfogadható lehet otthoni
hálózaton, de érdemes tudni erről a kockázatról.

### Mód B — böngészős feltöltés (ElegantOTA)

Nem igényel PlatformIO-t, bármilyen eszközről (telefon, tablet) is működik.

1. Csatlakozz a `wickedpickandplace.local` (vagy az eszköz IP-je) címre böngészőben
2. Kattints a **"Firmware feltoltes (OTA)"** linkre, vagy nyisd meg közvetlenül:
   `http://wickedpickandplace.local/update`
3. Válaszd ki a lefordított binárist: `.pio/build/wicked_pickandplace/firmware.bin`
4. Töltsd fel — az oldal mutatja a folyamatot, a végén az eszköz újraindul

**Parancssorból (curl-lal), PlatformIO Upload gomb nélkül:**

```bash
curl -F "file=@.pio/build/wicked_pickandplace/firmware.bin" http://wickedpickandplace.local/update
```

---

## Hibaelhárítás

| Tünet                                       | Valószínű ok / teendő                                                                                                                     |
| -------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `wickedpickandplace.local` nem érhető el | mDNS néha nem működik minden routeren/OS-en — próbáld az eszköz IP-címét közvetlenül (Serial monitorból kiolvasható induláskor) |
| ArduinoOTA "Authentication Failed"           | Jelszó van beállítva a firmware-ben, de a`platformio.ini`-ben nincs megadva (vagy fordítva)                                             |
| Első OTA-próbálkozás nem talál eszközt | Az eszközön még nincs OTA-képes firmware — tölts fel egyszer USB-n                                                                      |
| ElegantOTA`/update` oldal nem tölt be     | Ellenőrizd, hogy a`-DELEGANTOTA_USE_ASYNC_WEBSERVER=1` build flag be van-e állítva                                                       |
