# Wicked Pick and Place

> ESP32-alapú polargraph (karos/kötélvezérelt) **pick-and-place** robot: két léptetőmotor
> tartja és mozgatja egy vékony fonálpárral a kocsit ("gondola"), melyen egy saját
> tekercselésű elektromágnes emeli-engedi a tárgyakat. 8 gombos kezelőfelület,
> böngészős webes UI, és kétféle vezeték nélküli (OTA) firmware-frissítés.

![platform](https://img.shields.io/badge/platform-ESP32-blue)
![framework](https://img.shields.io/badge/framework-Arduino-00979D)
![build](https://img.shields.io/badge/build-PlatformIO-orange)

## Mi ez?

Egy polargraph-szerű gép (két rögzített pont, két fonál, a köztük lévő kocsi
pozícióját a fonálhosszak változtatásával lehet vezérelni), amit nem rajzolásra,
hanem **tárgyak felszedésére és lerakására** építettünk:

- a kocsin egy elektromágnes van, amivel fém tárgyakat lehet megfogni/elengedni
- a gép egy rácsozott munkaterületen ("cellák") navigál, és előre megtanított
  útvonalak (pick → place szekvenciák) alapján dolgozik
- a vezérlést egy 8 gombos panel biztosítja (kódbevitel, menürendszer), illetve
  egy beépített webes felület is kezeli a konfigurációt és a teszt-parancsokat

## Fő funkciók

- **Inverz/direkt kinematika** a kétfonalas ("két-horgonyos") geometriára, forgó
  kulisszával (a fonalak nem a kocsi középpontjában, hanem a kerületén,
  ±45°-ban csatlakoznak — lásd [kinematika_v2.md](kinematika_v2.md))
- **Pick & place szekvenciák**: cellák közötti mozgás, állítható utazási és
  "teher alatti" sebesség, gyorsulás
- **Elektromágnes vezérlés** PWM-es H-híd (IBT-2/BTS7960) meghajtóval, boost
  impulzussal a megbízható megfogáshoz
- **8 gombos kezelőfelület**: PLAY mód (4-jegyű kódok lejátszása) és EDIT mód
  (menürendszer gyors/hosszú koppintásokkal a kalibrációhoz és útvonal-szerkesztéshez)
- **FreeRTOS két-magos architektúra**: a mozgásvégrehajtás (Core 0) teljesen
  elkülönül a webszervertől, OTA-tól és gombolvasástól (Core 1) — mozgás közben
  sincs "vak" időszak
- **Webes felület** (ESPAsyncWebServer): állapot, teszt-parancsok, konfiguráció
- **Hangjelzések** (CLICK/ERR/START/DROP) a firmware-be égetve, külön
  fájlfeltöltés nélkül
- **Kétféle OTA frissítés** egymás mellett: PlatformIO hálózati feltöltés
  (ArduinoOTA) és böngészős feltöltés (ElegantOTA)
- **WiFi STA + AP fallback**, mDNS (`wickedpickandplace.local`)

## Hardver

| Komponens          | Eszköz                                   |
| ------------------- | ----------------------------------------- |
| Mikrovezérlő       | ESP32-WROVER-B (TTGO T7 V1.3)             |
| Léptetőmotorok     | 2× A4988 meghajtó, 1.8°/lépés, 1/16 microstepping |
| Elektromágnes       | Saját tekercselésű, IBT-2/BTS7960 PWM-meghajtóval |
| Kezelőfelület       | 8 nyomógomb                               |
| Kommunikáció        | WiFi (STA/AP), webes UI, OTA              |

## Projekt felépítése

```
src/main.cpp        - teljes firmware (kinematika, mozgásvezérlés, gombkezelés, webszerver, OTA)
src/Readme.md       - fejlesztői útmutató: build, WiFi, OTA módok, hibaelhárítás
kinematika_v2.md     - a forgó kulisszás kétfonalas kinematika levezetése
plan_5.md / ppplan_MOVE.md - tervezési jegyzetek a pick-and-place logikához
data/                - firmware-be égetett hangfájlok (click/err/start/drop)
platformio.ini      - PlatformIO projekt- és környezetkonfiguráció
```

## Gyors start

Részletes fejlesztői útmutatóért (szükséges könyvtárak, WiFi-viselkedés, mindkét
OTA-mód lépésről lépésre, hibaelhárítási táblázat) lásd a [src/Readme.md](src/Readme.md) fájlt.

```bash
# Build + USB feltöltés
pio run -e wicked_pickandplace -t upload

# Build + OTA feltöltés (a firmware-nek már futnia kell a gépen)
pio run -e wicked_pickandplace_ota -t upload
```

## Technológiák

[PlatformIO](https://platformio.org/) · [Arduino core for ESP32](https://github.com/espressif/arduino-esp32) ·
[FastAccelStepper](https://github.com/gin66/FastAccelStepper) ·
[ESPAsyncWebServer](https://github.com/ESP32Async/ESPAsyncWebServer) · [ElegantOTA](https://github.com/ayushsharma82/ElegantOTA)
