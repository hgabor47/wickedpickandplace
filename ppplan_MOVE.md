# ppplan_MOVE — a MOVE almód (és a rá épülő PATH/PLAY) helyreállítási terve

> **Célközönség:** kódoló LLM / fejlesztő, aki a `src/main.cpp`-t módosítja.
> **Előfeltétel:** a firmware build/upload lépéseit a felhasználó futtatja kézzel
> (PlatformIO). A kódoló ne indítson build/upload parancsot.

## STÁTUSZ: M1, M2, M3 (C változat), M4, M5 implementálva `src/main.cpp`-ben

A döntés: **M3 = "C" változat (auto-kalibráció legkisebb négyzetekkel)**.
Rövid összefoglaló, mi került a kódba (a build/upload a felhasználó feladata):

- **M5**: `geom` webparancs (teljes diagnosztikai lenyomat), SETUP 4. sarkánál
  összefoglaló log a számolt sarok-koordinátákkal.
- **M1**: `gridUVForSteps()` (inverz leképezés, Newton-iterációval) +
  `syncStateFromMotors()`, meghívva minden releváns átmenetnél (SETUP/ORIGIN jog
  elengedés, ORIGIN megerősítés, 4. sarok mentése, almód-belépés, boot, EDIT→PLAY
  átmenet — utóbbi csak a `currentX/currentY` becslés frissítése, a PLAY-mód
  kinematikája változatlan maradt, lásd H2 megjegyzés a kódban).
- **M2**: `moveJogApply(u,v)` arányos per-motor sebességgel; `updateMoveJogTick()`
  mindig a tényleges motorpozícióból (`gridUVForSteps`) tervez újra, `JOG_REPLAN_MS`
  (30 ms) throttlinggal, `JOG_LOOKAHEAD` (0.05) előretekintéssel.
- **M3-C**: `setArea XMAX,YMAX` webparancs + `runAutoCalibration()` (Gauss-Newton,
  6 ismeretlen: `calAx,calAy,calBx,calBy,calOffAmm,calOffBmm`, numerikus Jacobi +
  6x6 Gauss-elimináció), NVS-perzisztens (`saveCalibration()`/`loadCalibration()`).
  Automatikusan lefut a 4. SETUP-sarok mentésekor, ha az `setArea` már megtörtént;
  különben a `setArea` parancs futtatja le utólag. `trilaterateSteps()`/`xyToSteps()`
  a kalibrált horgonyokat használja, ha `calValid`, egyébként a régi hardkódolt
  `ANCHOR_A/B`-re esik vissza (0 eltolással) — visszafelé kompatibilis.
- **M4**: `circleIntersectBelow()` explicit hibát ad vissza (nem hallgat el `h2<0`
  esetet), `cornersFormReasonableRect()` konvexitás/terület-ellenőrzés
  `computeGridCornerXY()`-ban, `enterEditSubMode()` elutasítja a MOVE/CELLS
  belépést érvénytelen kalibráció esetén.

**Használat (első kalibráláskor)**: `setArea <XMAX_mm>,<YMAX_mm>` a `/program`
oldal parancssorán, majd a szokásos SETUP (4 sarok). Ha a SETUP már korábban kész
volt, a `setArea` utólag is lefuttatja az auto-kalibrációt. A `geom` paranccsal
bármikor ellenőrizhető az aktuális állapot/kalibráció.

---

## 0. Kontextus — miért készült ez a terv

A MOVE almód jelenleg használhatatlan: a kulissza belépéskor egy másik pontra
ugrik, a „RIGHT" gomb akár lefelé is viheti, a pálya ívelt, és a motorok
összehangolatlanul, lassulva forognak. Mivel a PATH almód és végső soron a PLAY
üzemmód pontossága a MOVE-ra épül, ezt kell először szilárd alapokra helyezni.

### A feltárt öt hiba (a javítások ezekre hivatkoznak)

| # | Hiba | Hol | Tünet |
|---|---|---|---|
| **H1** | `moveU`/`moveV` sosem szinkronizálódik a tényleges motorpozícióhoz; boot-kor `0.5,0.5` (rácsközép), és csak a MOVE/CELLS jog írja | `main.cpp` ~636, `updateMoveJogTick()` | MOVE-ba belépve az első gombnyomás a rácsközépre ugrat |
| **H2** | `currentX`/`currentY` csak `moveTo()`-ban frissül, az EDIT-jog sosem hívja | `main.cpp` ~387, 429-430 | EDIT után a PLAY-mód rossz pozíciót hisz |
| **H3** | Nincs abszolút referencia: a step-számláló nullpontja a boot-kori hardkódolt `currentX=680, currentY=70` + kalibrálatlan `ANCHOR_A/B` feltevésből származik, de a `trilaterate()` abszolút kötélhosszként értelmezi | `setup()` ~1304, `trilaterate()` | A „valós sík" torzított/eltolt → irányok összekeverednek |
| **H4** | A jog mindkét motornak **azonos** max. sebességet ad, noha eltérő a delta-juk | `moveJogApply()` ~776 | A rövidebb utat futó motor hamarabb végez → dogleg/ív |
| **H5** | Nyílt hurkú vezérlés: ~2 ms-enként új `moveTo()` egy szabadon araszoló `moveU/moveV`-re; a motorok lemaradnak, a cél elszalad | `updateMoveJogTick()` | Elengedés után a hiedelem ≠ valóság → következő nyomásnál ugrás |

---

## Mérföldkövek

Az M1–M2 és M5 **feltétel nélkül elvégzendő**. Az M3 tartalma a
**3. fázis döntésétől** függ (lásd lent) — a döntést a projekt tulajdonosa hozza
meg, addig az M3 nem indítható.

---

## M1 — Egyetlen igazságforrás: a nyers step-pozíció

**Cél:** bármely almódból kilépve, és bármely almódba belépve a rendszer mindig
tudja, hol áll a kulissza. Megszünteti: **H1, H2**.

### M1.1 Inverz leképezés: step → (u,v)

Új függvény a `gridTargetForUV()` mellé:

```cpp
// A jelenlegi (vagy megadott) step-pozicioból visszaadja a normalizalt (u,v)-t.
// Visszateres: false, ha nincs ervenyes kalibracio (gridCornerMask != 0x0F).
bool gridUVForSteps(long a, long b, float &outU, float &outV);
```

Megvalósítás:
- Ha az M3 után valós (X,Y)-térben dolgozunk: `trilaterate()`-tel (X,Y)-t
  számolunk, majd a 4 sarok (X,Y)-jából **inverz bilineáris** interpolációval
  kapjuk (u,v)-t. Az inverz bilineáris zárt alakban másodfokú egyenletre vezet;
  egyszerűbb és robusztusabb 5-10 iterációs Newton/fixpont-közelítés a
  `gridTargetForUV()` felhasználásával.
- Ha kötélhossz-térben maradunk (B-változat): ugyanez, csak a `bilerpLong()`
  invertálásával az (A,B) step-párra.

**Elfogadási kritérium:** `gridUVForSteps(gridTargetForUV(u,v))` visszaadja az
eredeti `(u,v)`-t ±0.001 hibán belül, a `[0,1]×[0,1]` tartomány 25 mintapontján.

### M1.2 Központi szinkronizáló függvény

```cpp
// A motorok tenyleges step-poziciojabol frissiti a szarmaztatott allapotot:
// moveU/moveV (EDIT) es currentX/currentY (PLAY). Ez az EGYETLEN hely, ahol
// ezek a valtozok a jog utan ujra ervenyes erteket kapnak.
void syncStateFromMotors();
```

Tartalma:
1. `long a = motorA->getCurrentPosition(); long b = motorB->getCurrentPosition();`
2. `gridUVForSteps(a, b, moveU, moveV)` — ha sikerül, `constrain(0..1)`.
3. `currentX/currentY` frissítése ugyanabból a step-párból (M3 után
   `trilaterate()`-tel; addig a `moveU/moveV`→`gridTargetForUV` konzisztencia
   fenntartása elég).
4. `webLogf()` debug sor a kapott értékekről.

### M1.3 A szinkron beillesztése a hívási pontokba

`syncStateFromMotors()` hívása kötelező:

| Hívási pont | Függvény | Miért |
|---|---|---|
| minden EDIT-almódba belépéskor | `enterEditSubMode()`, és a `pendingSetupEntry` ág `updatePendingMenuActions()`-ben | MOVE ott folytassa, ahol a gép áll |
| nyers jog elengedésekor | `setupHandleRelease()`, `originHandleRelease()` | SETUP/ORIGIN jog után ismert legyen a hely |
| ORIGIN megerősítés után | `originConfirm()` — a `setCurrentPosition()` hívások **után** | a referencia-váltás után újra kell számolni |
| SETUP 4. sarok tárolása után | `storeCorner()`, a `computeDefaultGridCells()` után | új kalibráció → új (u,v) jelentés |
| EDIT → PLAY váltáskor | `onButtonPressed()` gomb3×8 ága | a PLAY-mód `currentX/currentY`-ja helyes legyen |

**Elfogadási kritérium:** SETUP-ban nyers jog-gal elmozgatva a kulisszát, majd
ESC → MOVE belépés → egyetlen irány rövid megnyomása **nem** okoz ugrást; a gép
onnan indul, ahol fizikailag áll.

---

## M2 — Zárt hurkú, összehangolt jog

**Cél:** egyenes, kiszámítható, a gombot követő mozgás. Megszünteti: **H4, H5**.

### M2.1 A cél a tényleges pozícióból számoljon (H5)

`updateMoveJogTick()` átírása: a tick **ne** a szabadon futó `moveU/moveV`-t
növelje, hanem minden újratervezéskor:

1. `gridUVForSteps(getCurrentPosition() ...)` → `curU, curV` (**a valóság**)
2. `tgtU = curU ± lookahead`, `tgtV = curV ± lookahead` a lenyomott gombok szerint
3. `constrain(0..1)`, majd `gridTargetForUV(tgtU, tgtV, ta, tb)` → `moveTo`

Így a cél soha nem tud a valóságtól elszakadni. A `lookahead` legyen akkora,
hogy a mozgás folyamatos maradjon (nagyságrendileg a következő újratervezési
periódus alatt megtehető út 1.5-2×-e).

### M2.2 Arányos sebességek (H4)

`moveJogApply()`-ban a `moveTo()` előtt a `moveTo()` (PLAY-mód, ~405-430. sor)
mintájára:

```cpp
long dA = labs(ta - motorA->getCurrentPosition());
long dB = labs(tb - motorB->getCurrentPosition());
long dMax = max(dA, dB);
if (dMax == 0) return;
uint32_t vMax = mmSpeedToStepsHz(JOG_SPEED_MM_S);
motorA->setSpeedInHz(max(1UL, (unsigned long)((uint64_t)vMax * dA / dMax)));
motorB->setSpeedInHz(max(1UL, (unsigned long)((uint64_t)vMax * dB / dMax)));
```

Így mindkét motor **egyszerre** ér célba → a köztes pálya egyenes.

### M2.3 Újratervezési periódus ritkítása

Vezess be egy `JOG_REPLAN_MS` konstanst (**kiindulás: 30 ms**). A
`updateMoveJogTick()` a `loop()`-ból továbbra is minden ciklusban hívódjon (a
gombfelengedés-detektálás miatt), de a cél-újrakiadás csak `JOG_REPLAN_MS`-enként
történjen. Ez megszünteti a folyamatos rámpa-újraindítást („traktor"-érzet).

### M2.4 Elengedés kezelése

A meglévő `stopMove()` ág maradjon, de utána **kötelezően** hívd meg
`syncStateFromMotors()`-t, hogy a lefékezési út is beszámítódjon.

**Elfogadási kritérium:** vízszintes (RIGHT) jog közben a kulissza Y-eltérése a
teljes szélességen ne haladja meg a cellaméret 10%-át; a gomb elengedése után a
gép ≤0.5 s-on belül megáll, és a `geom` (M5) kiírás pozíciója egyezik a fizikai
helyzettel.

---

## M3 — Abszolút geometriai referencia *(DÖNTÉST IGÉNYEL)*

**Megszünteti: H3.** A három lehetséges út közül **a projekt tulajdonosa
választ**; a kódoló csak a kiválasztottat valósítsa meg.

### Változat A — ORIGIN adjon valódi abszolút referenciát *(ajánlott ár/érték)*

1. Új web-parancs: `setGeom AX,AY,BX,BY` — a két horgony valós koordinátái
   mm-ben, NVS-be mentve; a `ANCHOR_A_X/Y`, `ANCHOR_B_X/Y` konstansokat futásidejű
   változókra kell cserélni.
2. Új web-parancs: `setOriginXY X,Y` — a munkaterület (0,0) sarkának valós
   koordinátái ugyanabban a rendszerben.
3. `originConfirm()` átírása: a jelenlegi „`setCurrentPosition(gridCornerA[0])`"
   helyett a **valós** kötélhosszakból számoljon:
   `computeStringLengths(originX, originY, lenA, lenB)` →
   `motorA->setCurrentPosition(mmToSteps(lenA))` (és `B`).
4. Ettől kezdve a step-számláló **valódi kötélhossz**, a `trilaterate()` érvényes,
   és minden downstream számítás (MOVE, CELLS, PATH, PLAY) helyes.
5. A `setup()`-beli hardkódolt `currentX=680, currentY=70` alapú
   `setCurrentPosition()` hívást el kell távolítani / „kalibrálatlan" állapotra
   cserélni, ami tiltja a MOVE/CELLS/PLAY belépést, amíg ORIGIN nem futott le.

### Változat B — Visszatérés kötélhossz-térhez *(leggyorsabb stabilizálás)*

1. `gridTargetForUV()` térjen vissza a tisztán `bilerpLong()`-alapú változatra
   (a `trilaterate()`/`computeGridCornerXY()` hívások kikapcsolása vagy törlése).
2. Az M1/M2/M5 javítások ettől függetlenül elvégzendők.
3. Ismert korlát: a pálya enyhén ívelt marad (a kötélhossz-tér nemlinearitása
   miatt), de kiszámítható és a sarkokban pontos. Dokumentálni kell.

### Változat C — Auto-kalibráció legkisebb négyzetekkel *(legpontosabb, legösszetettebb)*

1. Új web-parancs: `setArea XMAX,YMAX` — a munkaterület valós mérete mm-ben.
2. Ismeretlenek: `ax, ay, bx, by, offA, offB` (6 db); egyenletek: 4 sarok × 2
   kötél = 8 db, alakjuk
   `(gridCornerA[i] + offA)/STEPS_PER_MM == dist((x_i,y_i),(ax,ay))`, ahol a
   `(x_i,y_i)` a `setArea` szerinti `(0,0)/(0,YMAX)/(XMAX,0)/(XMAX,YMAX)`.
3. Megoldás Gauss-Newton iterációval (5-8 iteráció, `double` aritmetika,
   numerikus Jacobi-mátrixszal is elfogadható), a SETUP 4. sarkának tárolásakor.
4. Konvergencia-ellenőrzés: ha a maradék hiba > 5 mm, log-figyelmeztetés és a
   kalibráció elutasítása.
5. Az eredmény (`ax, ay, bx, by, offA, offB`) NVS-be mentendő; a step→mm
   átváltás mindenhol `+offX` eltolással történjen.

---

## M4 — Bemeneti validáció és hibatűrés

1. `trilaterate()`: a `h2 < 0` ág jelenleg csendben `h = 0`-ra esik vissza. Ehelyett
   térjen vissza `false`-szal, és a hívó logoljon konkrét hibaüzenetet
   („a mért kötélhosszak geometriailag lehetetlenek — újrakalibrálás szükséges").
2. `computeGridCornerXY()` (ha az A vagy C változat marad): a 4 kapott (X,Y)-ból
   ellenőrizze, hogy **konvex, nem elfajult, nem tükrözött** négyszöget alkotnak
   (előjeles terület > küszöb, sarkok sorrendje helyes). Ha nem, állítsa
   `gridCornerXYValid = false`-ra és logoljon.
3. A MOVE/CELLS almódba belépés **utasítsa vissza** magát (log + ACK nélkül),
   ha nincs érvényes kalibráció, ahelyett hogy néma tartalék-ágra esne vissza.

---

## M5 — Diagnosztika

### M5.1 `geom` web-parancs

Új parancs a `/cmd` parserben, ami egyetlen szöveges válaszban kiadja:

- `appMode`, `editSubMode`
- `motorA/B->getCurrentPosition()` (step)
- `moveU`, `moveV`
- `currentX`, `currentY`
- a 4 sarok step-értékei + `gridCornerMask`
- a 4 sarok számított (X,Y)-ja + `gridCornerXYValid` (A/C változat esetén)
- `STEPS_PER_MM`, `MICROSTEPPING`, és az M3-ban bevezetett kalibrációs paraméterek

### M5.2 SETUP-lezáró összefoglaló log

A 4. sarok tárolása után a `storeCorner()` írja ki a számított sarok-(X,Y)-kat és
a munkaterület levezetett méretét, hogy azonnal látszódjon, ha a kalibráció
képtelen eredményt adott.

---

## Végrehajtási sorrend és függőségek

```
M1.1 ──► M1.2 ──► M1.3 ──┐
                         ├──► M2.1 ──► M2.2 ──► M2.3 ──► M2.4
M5.1 (bármikor, segíti a hibakeresést) ┘

M3 (döntés után) ──► M4 ──► az M1.1/M1.2 (X,Y)-ágának véglegesítése
```

- **M5.1-et érdemes elsőként megcsinálni**, mert e nélkül a többi mérföldkő
  hibakeresése vakrepülés.
- **M1 és M2 az M3 döntésétől függetlenül elkezdhető** — csak az M1.1/M1.2 belső
  (X,Y)-számítása véglegesíthető az M3 után.
- **M4 az M3 után** végezhető el értelmesen.

---

## Regressziós ellenőrzőlista (minden mérföldkő után)

- [ ] A PLAY mód gombkód-beolvasása (6 jegyű kód → cella) változatlanul működik
- [ ] A gomb3 ×3 (ESC) nem tárol el mellékesen sarkot SETUP-ban
- [ ] A gomb6 ×8 / gomb3 ×8 mód-váltás továbbra is működik, helyes ACK-kal
- [ ] A MAGNET (gomb6) momentary viselkedése MOVE/CELLS-ben megmaradt
- [ ] SETUP → ESC → MOVE belépés után nincs pozícióugrás
- [ ] Újraindítás után ORIGIN lefuttatásával a korábbi SETUP-adatok érvényesek
- [ ] A CELLS „legközelebbi cella felülírása" a helyes cellát találja meg
