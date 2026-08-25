// =====================================================================
//  KINEMATIKA v2 - polusmentes nyomateki reziduum + 1D theta-megoldas
//
//  Mit valt ki: computeStringLengths(), evalGondolaResiduals(),
//               trilaterateSteps().
//  Valtozatlan marad: circleIntersectBelow(), mmToSteps(), xyToSteps(),
//               computeGridCornerXY() es minden hivo.
//
//  Ket lenyegi valtozas a regihez kepest:
//
//  1) A nyomateki reziduumbol eltunt az osztas. Regen:
//        tA = -uBx/uAx;  torque = tA*(rA x uA) + (rB x uB)
//     Ennek POLUSA van ott, ahol az A kotel fuggolegesse valik (uAx -> 0),
//     es a "ha uAx tul kicsi, akkor tA = 1.0" ag ott meg egy szakadast is
//     bevisz. Newton ezekbe szaladt bele. Ha az egeszet uAx-szel szorozzuk:
//        g = -uBx*(rA x uA) + uAx*(rB x uB)
//     akkor a gyokok UGYANAZOK, de nincs se osztas, se pol, se eseti ag.
//     (R-rel normalva g dimenziotlan, O(1) nagysagrendu.)
//
//  2) A 3 ismeretlenbol (x, y, theta) valojaban csak theta "nehez": ha
//     theta ismert, a ket kotelhossz-egyenlet ZART KEPLETTEL megoldhato -
//     eleg a horgonyokbol levonni a rogzitesi pont eltolasat ("virtualis
//     horgonyok"), es kesz a kor-metszes. Ezert a 3x3 Newton helyett egy
//     1D gyokkereses megy theta-ra, bisekcios tartalekkal: nem tud
//     divergalni, nem kell Jacobi-determinans, es a kotelhossz-egyenletek
//     MINDEN iteracioban pontosan teljesulnek.
// =====================================================================

// ---- iteracios parameterek ----
static const int   TRI_MAX_ITER      = 12;      // 1D Newton lepesek
static const int   TRI_SCAN_STEPS    = 48;      // durva scan felbontas (tartalek)
static const int   TRI_BISECT_ITER   = 24;      // bisekcio lepesszam
static const float TRI_TORQUE_TOL    = 2.0e-5f; // elfogadasi kuszob (normalt)
static const float TRI_TORQUE_ACCEPT = 1.0e-3f; // "meg elfogadhato" kuszob
static const float TRI_THETA_H       = 0.002f;  // rad - numerikus derivalt lepese
static const float THETA_MIN         = -1.2f;
static const float THETA_MAX         =  1.2f;

// Adott (x, y) kozeppont es theta mellett kiszamolja a ket kotelhosszat,
// az A kotel relativ erotenyezojet (tA) es a POLUSMENTES nyomateki
// reziduumot. Ez a modell EGYETLEN definicios helye - a forward es az
// inverz irany is ezt hasznalja, igy nem tudnak szetcsuszni.
//   visszateres: g (normalt nyomateki reziduum, gyoke = egyensuly)
static float gondolaTorqueAt(float x, float y, float theta,
                             float &lenA, float &lenB, float &tA) {
  float phiA = theta - GONDOLA_ARM_ANGLE_RAD;
  float rAx  =  GONDOLA_RADIUS_MM * sin(phiA);
  float rAy  = -GONDOLA_RADIUS_MM * cos(phiA);
  float vAx  = ANCHOR_A_X - (x + rAx), vAy = ANCHOR_A_Y - (y + rAy);
  float dA   = sqrt(vAx * vAx + vAy * vAy);
  if (dA < 1e-3f) { lenA = lenB = 0; tA = -1.0f; return 0.0f; }
  float uAx = vAx / dA, uAy = vAy / dA;

  float phiB = theta + GONDOLA_ARM_ANGLE_RAD;
  float rBx  =  GONDOLA_RADIUS_MM * sin(phiB);
  float rBy  = -GONDOLA_RADIUS_MM * cos(phiB);
  float vBx  = ANCHOR_B_X - (x + rBx), vBy = ANCHOR_B_Y - (y + rBy);
  float dB   = sqrt(vBx * vBx + vBy * vBy);
  if (dB < 1e-3f) { lenA = lenB = 0; tA = -1.0f; return 0.0f; }
  float uBx = vBx / dB, uBy = vBy / dB;

  // A ket kotel nyomatekkarja a korong kozeppontjara (keresztszorzat z-komponense)
  float mA = rAx * uAy - rAy * uAx;
  float mB = rBx * uBy - rBy * uBx;

  lenA = dA;
  lenB = dB;
  // tA csak ERVENYESSEG-ELLENORZESRE kell (pozitiv kotelerot varunk),
  // a megoldasba nem szol bele - ezert nyugodtan lehet osztani itt.
  tA = (fabs(uAx) > 1e-6f) ? (-uBx / uAx) : -1.0f;
  // Polusmentes alak, R-rel normalva -> dimenziotlan, O(1)
  return (-uBx * mA + uAx * mB) / GONDOLA_RADIUS_MM;
}

// FORWARD: (X,Y) -> kotelhosszak. Egyetlen ismeretlen (theta), csillapitott
// 1D Newton. A csillapitas (line search) miatt nem tud tullendulni.
void computeStringLengths(float x, float y, float &lenA, float &lenB) {
  float theta = constrain(lastThetaForward, THETA_MIN, THETA_MAX);
  float tA;
  float g = gondolaTorqueAt(x, y, theta, lenA, lenB, tA);

  for (int iter = 0; iter < TRI_MAX_ITER; iter++) {
    if (fabs(g) < TRI_TORQUE_TOL) break;
    float thH = min(THETA_MAX, theta + TRI_THETA_H);
    float lA_h, lB_h, tA_h;
    float gH = gondolaTorqueAt(x, y, thH, lA_h, lB_h, tA_h);
    float deriv = (gH - g) / (thH - theta);
    if (fabs(deriv) < 1e-9f) break;

    float step = g / deriv;
    float lambda = 1.0f;
    bool improved = false;
    for (int bt = 0; bt < 8; bt++) {         // line search: csak javulo lepest fogadunk el
      float thNew = constrain(theta - lambda * step, THETA_MIN, THETA_MAX);
      float lA_n, lB_n, tA_n;
      float gNew = gondolaTorqueAt(x, y, thNew, lA_n, lB_n, tA_n);
      if (fabs(gNew) < fabs(g)) {
        theta = thNew; g = gNew; lenA = lA_n; lenB = lB_n; tA = tA_n;
        improved = true;
        break;
      }
      lambda *= 0.5f;
    }
    if (!improved) break;                    // elertuk a float32 zajszintet
  }

  lastThetaForward = theta;
  // lenA/lenB mar a legutolso elfogadott theta-hoz tartozik
}

// Adott theta mellett a korong kozeppontja ZART KEPLETTEL: a rogzitesi pont
// eltolasat levonjuk a horgonybol ("virtualis horgony"), es a ket kotelhossz
// mint sugar egy sima kor-metszest ad. Igy a kotelhossz-egyenletek MINDIG
// pontosan teljesulnek, barmilyen theta mellett.
static bool gondolaSolveAtTheta(float theta, float targetLenA, float targetLenB,
                                float &x, float &y, float &g, float &tA) {
  float phiA = theta - GONDOLA_ARM_ANGLE_RAD;
  float rAx  =  GONDOLA_RADIUS_MM * sin(phiA);
  float rAy  = -GONDOLA_RADIUS_MM * cos(phiA);
  float phiB = theta + GONDOLA_ARM_ANGLE_RAD;
  float rBx  =  GONDOLA_RADIUS_MM * sin(phiB);
  float rBy  = -GONDOLA_RADIUS_MM * cos(phiB);

  if (!circleIntersectBelow(ANCHOR_A_X - rAx, ANCHOR_A_Y - rAy,
                            ANCHOR_B_X - rBx, ANCHOR_B_Y - rBy,
                            targetLenA, targetLenB, x, y)) {
    return false;
  }
  float lA, lB;
  g = gondolaTorqueAt(x, y, theta, lA, lB, tA);
  return true;
}

// INVERZ: step-par -> valodi (X,Y).
// Ez az EGYETLEN hely, ahol step->(X,Y) tortenik (M1: egyetlen igazsagforras).
bool trilaterateSteps(long stepsA, long stepsB, float &outX, float &outY) {
  float targetLenA = stepsA / STEPS_PER_MM;
  float targetLenB = stepsB / STEPS_PER_MM;

  float theta = constrain(lastThetaInverse, THETA_MIN, THETA_MAX);
  float x, y, g, tA;
  bool  have = gondolaSolveAtTheta(theta, targetLenA, targetLenB, x, y, g, tA);
  if (!have) {                               // warm-start nem hasznalhato -> hideg indulas
    theta = 0.0f;
    have  = gondolaSolveAtTheta(theta, targetLenA, targetLenB, x, y, g, tA);
  }

  // --- 1. fazis: csillapitott 1D Newton theta-ra ---
  if (have) {
    for (int iter = 0; iter < TRI_MAX_ITER; iter++) {
      if (fabs(g) < TRI_TORQUE_TOL) break;
      float thH = min(THETA_MAX, theta + TRI_THETA_H);
      float xH, yH, gH, tAH;
      if (!gondolaSolveAtTheta(thH, targetLenA, targetLenB, xH, yH, gH, tAH)) break;
      float deriv = (gH - g) / (thH - theta);
      if (fabs(deriv) < 1e-9f) break;

      float step = g / deriv;
      float lambda = 1.0f;
      bool improved = false;
      for (int bt = 0; bt < 8; bt++) {
        float thNew = constrain(theta - lambda * step, THETA_MIN, THETA_MAX);
        float xN, yN, gN, tAN;
        if (gondolaSolveAtTheta(thNew, targetLenA, targetLenB, xN, yN, gN, tAN) &&
            fabs(gN) < fabs(g)) {
          theta = thNew; x = xN; y = yN; g = gN; tA = tAN;
          improved = true;
          break;
        }
        lambda *= 0.5f;
      }
      if (!improved) break;                  // float32 zajszint
    }
  }

  // --- 2. fazis (tartalek): durva scan + bisekcio, ha a Newton nem ert celba ---
  if (!have || fabs(g) > TRI_TORQUE_ACCEPT) {
    bool  havePrev = false;
    float prevTh = 0, prevG = 0;
    float loTh = 0, loG = 0, hiTh = 0, hiG = 0;
    bool  bracketed = false;

    for (int i = 0; i <= TRI_SCAN_STEPS; i++) {
      float t = THETA_MIN + (THETA_MAX - THETA_MIN) * ((float)i / (float)TRI_SCAN_STEPS);
      float xs, ys, gs, tAs;
      if (!gondolaSolveAtTheta(t, targetLenA, targetLenB, xs, ys, gs, tAs)) {
        havePrev = false;                    // ertelmezesi tartomanyon kivul
        continue;
      }
      // Elojelvaltast csak akkor fogadunk el gyoknek, ha mindket oldal
      // KICSI - igy nem ulunk fel egy esetleges ugrasnak.
      if (havePrev && prevG * gs < 0 && fabs(prevG) < 2.0f && fabs(gs) < 2.0f) {
        loTh = prevTh; loG = prevG; hiTh = t; hiG = gs;
        bracketed = true;
        break;
      }
      havePrev = true; prevTh = t; prevG = gs;
    }

    if (!bracketed) {
      lastThetaInverse = 0.0f;               // ne mergezzuk a kovetkezo hivast
      webLogf("KINEMATIKA: nincs egyensulyi theta (stepsA=%ld, stepsB=%ld, lenA=%.1f, lenB=%.1f)",
              stepsA, stepsB, targetLenA, targetLenB);
      return false;
    }

    for (int b = 0; b < TRI_BISECT_ITER; b++) {
      float mid = 0.5f * (loTh + hiTh);
      float xm, ym, gm, tAm;
      if (!gondolaSolveAtTheta(mid, targetLenA, targetLenB, xm, ym, gm, tAm)) break;
      if (loG * gm <= 0) { hiTh = mid; hiG = gm; }
      else               { loTh = mid; loG = gm; }
      theta = mid; x = xm; y = ym; g = gm; tA = tAm;
    }
  }

  // --- 3. fazis: fizikai ervenyesseg ---
  // A kotel csak HUZNI tud: tA <= 0 azt jelenti, hogy egy matematikailag
  // letezo, de fizikailag lehetetlen gyokre futottunk.
  if (tA <= 0.0f) {
    lastThetaInverse = 0.0f;
    webLogf("KINEMATIKA: fizikailag ervenytelen gyok - negativ koteleroe (stepsA=%ld, stepsB=%ld, x=%.1f, y=%.1f, th=%.3f, tA=%.2f)",
            stepsA, stepsB, x, y, theta, tA);
    return false;
  }
  // A kulissza a horgonyok ALATT van (nagyobb Y). Ha fole kerult, rossz ag.
  if (y <= ANCHOR_A_Y || y <= ANCHOR_B_Y) {
    lastThetaInverse = 0.0f;
    webLogf("KINEMATIKA: fizikailag ervenytelen gyok - a kulissza a horgonyok folott (stepsA=%ld, stepsB=%ld, x=%.1f, y=%.1f, th=%.3f)",
            stepsA, stepsB, x, y, theta);
    return false;
  }
  if (fabs(g) > TRI_TORQUE_ACCEPT) {
    lastThetaInverse = 0.0f;
    webLogf("KINEMATIKA: nem konvergalt (stepsA=%ld, stepsB=%ld, x=%.1f, y=%.1f, th=%.3f, g=%.6f, tA=%.2f)",
            stepsA, stepsB, x, y, theta, g, tA);
    return false;
  }

  outX = x;
  outY = y;
  lastThetaInverse = theta;                  // warm-start CSAK sikeres gyokre
  return true;
}
