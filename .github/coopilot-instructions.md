
Használható parancs példák:


1. `curl -X POST http://wickedpickandplace.local/cmd -d $'cellNum 3'` — alap konfiguráció.
2. `curl -X POST .../cmd -d $'setCell 1,100001,{70,70,1,1},{680,70,255,0}'` — egyszerű cella mentése.
3. `curl -X POST .../cmd -d 'listCell'` — ellenőrizni, hogy pontosan visszaadja-e, amit beírtál.
4. Reboot (áramtalanítás), majd újra `listCell` — ez teszteli, hogy a LittleFS-ben tényleg megmaradt-e.
5. Gombnyomással a `100001` kód beütése — nézd meg a `/log` oldalon (SSE), hogy a queue-ba kerül-e, majd lefut-e a mozgás.
6. `testMotor 0,200` és `testMagnet 30,2` — ezek a legkockázatlanabb módon tesztelik a queue-t motor/mágnes mozgás nélkül a teljes cella-logikától függetlenül.
