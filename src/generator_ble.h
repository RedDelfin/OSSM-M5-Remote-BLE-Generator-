// =====================================================================
//  Generator-Modus (BLE) fuer die OSSM-M5-Remote (Ortlof v2.2.0)  -  gen5
//
//  iPhone (Bluefy + Generator-Seite)  --BLE-->  M5  --ESP-NOW-->  OSSM-Stroke
//
//  Die M5 nimmt Werte per BLE entgegen und schickt sie ueber die normale
//  SendCommand()-Funktion an die OSSM, genau wie beim Drehen am Encoder.
//  Einheiten = die Werte, die auch auf dem M5-Display stehen.
//
//  Bedienung / Sicherheit (Regeln aus dem Grill-me, 15.11.2026):
//   - Start: iPhone fragt an (GEN:1), Bestaetigen NUR per Touch auf das
//     mittlere Display-Feld ("BESTAETIGEN"). Die Hardware-Mitte bestaetigt nie.
//   - Hardware-Mitte waehrend des Generators = PAUSE (Maschine AUS, Generator
//     bleibt aktiv), erneut = weiter. Solange pausiert, wird ON vom iPhone
//     ignoriert. Das mittlere Display-Feld tut waehrend des Generators nichts.
//   - Encoder/Slider hoch und runter erlaubt; Hand-Wert gilt bis zum naechsten
//     Block (BLK). Hand-Werte werden auf die Grenzen (LIM) gekappt.
//   - Linke/rechte Taste und linkes/rechtes Display-Feld werden ignoriert.
//   - Verbindung weg / Lebenszeichen 2 s weg: Generator aus, Speed in 5 s
//     sanft auf 30 (Tiefe/Hub bleiben), Maschine laeuft langsam weiter bis
//     Eingriff an der M5. Weitermachen: iPhone fragt neu an + Bestaetigen.
//   - Home-Screen verlassen / OSSM entkoppelt / GEN:0: Maschine AUS.
//   - Einfrier-Schutz: das ESP-NOW-Lebenszeichen an die OSSM geht nur raus,
//     solange loop() laeuft (genLoopAliveMs). Haengt die M5, stoppt die OSSM
//     nach ca. 5 s von selbst.
//
//  Befehle (Text, mehrere mit ';' oder Zeilenumbruch getrennt):
//   GEN:1            Modus anfragen (M5 zeigt "BESTAETIGEN")
//   GEN:0            Modus beenden (Maschine AUS)
//   PING             Lebenszeichen (jeder Schreibzugriff zaehlt auch)
//   S:<wert> D:<wert> H:<wert> X:<wert>   Speed / Tiefe / Hub / Sensation
//   P:<index>        Pattern (0 SimpleStroke, 1 TeasingPounding, 2 RoboStroke,
//                    3 HalfnHalf, 4 Deeper, 5 StopNGo, 6 Insist, 7 Knot)
//   ON / OFF         Maschine an / aus (ON wird bei M5-Pause ignoriert)
//   BLK              neuer Block: Hand-Werte verfallen
//   LIM:S=<v>,D=<v>,H=<v>   Grenzen (jederzeit, gelten fuer Generator- und Hand-Werte)
//
//  Status (Notify/Read, JSON), ca. alle 500 ms:
//   {"p","g","a","on","s","d","h","x","pt","ms","md","ls","ld","lh","r","b",
//    "hold","ov"(1 Speed,2 Tiefe,4 Hub,8 Sensation),"rmp"(sanft runter aktiv),"t"(Touch an)}
// =====================================================================
#pragma once
#include <NimBLEDevice.h>

#define GEN_SVC_UUID  "6f73736d-6765-6e00-8000-00805f9b0001"
#define GEN_CMD_UUID  "6f73736d-6765-6e00-8000-00805f9b0002"
#define GEN_ST_UUID   "6f73736d-6765-6e00-8000-00805f9b0003"

#define GEN_TIMEOUT_MS      2000   // Lebenszeichen iPhone
#define GEN_ARM_TIMEOUT_MS  30000  // so lange wartet "BESTAETIGEN"
#define GEN_STATUS_MS       500
#define GEN_SOFT_MS         5000   // sanft runter in 5 s
#define GEN_SOFT_SPEED      30.0f  // ... auf diesen Speed (15 hatte zu wenig Moment)
#define GEN_ALIVE_MS        2000   // Einfrier-Schutz

// ---------- zwischen Tasks geteilt ----------
static portMUX_TYPE genMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t genLastRxMs = 0;
static volatile bool genBleConnected = false;
static volatile bool genBleLost = false;
volatile uint32_t genLoopAliveMs = 0;         // von loop() gesetzt, vom ESP-NOW-Task gelesen
#define GEN_RX_MAX 512
static char genRx[GEN_RX_MAX];                // eingegangene Befehle (kein malloc im kritischen Abschnitt)
static size_t genRxLen = 0;

// ---------- nur in loop() ----------
static bool genBleStarted = false;
static bool genActive = false;
static bool genArmed = false;
static uint32_t genArmedMs = 0;
static uint32_t genStatusMs = 0;
static uint32_t genBannerUntil = 0;
static String genReason = "";
static float genLimS = 1e9, genLimD = 1e9, genLimH = 1e9;
static bool genHold = false;
static uint8_t genOv = 0;
static float genSnapS = 0, genSnapD = 0, genSnapH = 0, genSnapX = 0;
static bool genHwMx = false;                  // Hardware-Mitte in diesem loop-Durchlauf gedrueckt
static bool genSoft = false;                  // sanft runter laeuft
static uint32_t genSoftStartMs = 0, genSoftSentMs = 0;
static float genSoftFrom = 0;
static char genMidText[24] = "";

static NimBLECharacteristic* genStChr = nullptr;
static lv_obj_t* genBanner = nullptr;

class GenSrvCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s) override {
    genBleConnected = true;
    genLastRxMs = millis();
  }
  void onDisconnect(NimBLEServer* s) override {
    genBleConnected = false;
    genBleLost = true;
    NimBLEDevice::startAdvertising();
  }
};

class GenCmdCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    std::string v = c->getValue();
    genLastRxMs = millis();
    const size_t n = v.size();
    portENTER_CRITICAL(&genMux);
    if (genRxLen + n + 1 < GEN_RX_MAX) {
      memcpy(genRx + genRxLen, v.data(), n);
      genRxLen += n;
      genRx[genRxLen++] = ';';
    }
    portEXIT_CRITICAL(&genMux);
  }
};

// ---------------------------------------------------------------- Anzeige
static void genShowBanner(const char* txt, uint32_t color, uint32_t forMs) {
  if (!genBanner) return;
  lv_label_set_text(genBanner, txt);
  lv_obj_set_style_bg_color(genBanner, lv_color_hex(color), 0);
  lv_obj_clear_flag(genBanner, LV_OBJ_FLAG_HIDDEN);
  genBannerUntil = forMs ? millis() + forMs : 0;
}
static void genHideBanner() {
  if (genBanner) lv_obj_add_flag(genBanner, LV_OBJ_FLAG_HIDDEN);
  genBannerUntil = 0;
}
static void genMidLabel(bool confirm) {
  if (confirm) lv_label_set_text(ui_HomeButtonMText, "BESTAETIGEN");
  else if (genMidText[0]) lv_label_set_text(ui_HomeButtonMText, genMidText);
}
// Rechtes Display-Feld (Pattern/Menue) sperren, solange der Generator laeuft
static void genLockTouch(bool lock) {
  if (lock) lv_obj_add_state(ui_HomeButtonR, LV_STATE_DISABLED);
  else lv_obj_clear_state(ui_HomeButtonR, LV_STATE_DISABLED);
}

// ---------------------------------------------------------------- Zustaende
static void genEndCommon() {
  genActive = false;
  genArmed = false;
  genHold = false;
  genOv = 0;
  genLockTouch(false);
  genMidLabel(false);
}

// Hartes Ende: Maschine AUS (GEN:0, Home verlassen, OSSM entkoppelt)
static void genAbort(const char* reason) {
  bool wasOn = genActive;
  genEndCommon();
  genSoft = false;
  genReason = reason;
  if (wasOn) {
    SendCommand(OFF, 0.0, OSSM_ID);
    vibrate(255, 300);
    String t = String("GENERATOR AUS: ") + reason;
    genShowBanner(t.c_str(), 0xC62828, 6000);
  } else {
    genHideBanner();
  }
  LogDebugFormatted("[GEN] Ende: %s\n", reason);
}

// Weiches Ende bei Verbindungsverlust: Speed in 5 s auf 15, Maschine laeuft weiter
static void genSoftLoss(const char* reason) {
  bool wasOn = genActive;
  bool wasHold = genHold;
  genEndCommon();
  genReason = reason;
  if (!wasOn) { genHideBanner(); return; }
  vibrate(255, 300);
  if (!wasHold && speed > GEN_SOFT_SPEED) {
    genSoft = true;
    genSoftFrom = speed;
    genSoftStartMs = millis();
    genSoftSentMs = 0;
  }
  genShowBanner(wasHold ? "VERBINDUNG WEG - Maschine steht"
                        : "VERBINDUNG WEG - langsam weiter, Mitte = Stopp",
                0xC62828, 0);
  LogDebugFormatted("[GEN] Verbindung weg: %s\n", reason);
}

// Pause / weiter (nur Hardware-Mitte)
static void genToggleHold() {
  genHold = !genHold;
  if (genHold) {
    SendCommand(OFF, 0.0, OSSM_ID);
    genReason = "Pause an der M5";
    genShowBanner("PAUSE (M5) - Mitte = weiter", 0xEF6C00, 0);
  } else {
    SendCommand(ON, 0.0, OSSM_ID);
    genReason = "weiter an der M5";
    genShowBanner("GENERATOR AKTIV - Mitte = Pause", 0xC62828, 0);
  }
}

// Bestaetigung per Touch auf das mittlere Display-Feld
static void genConfirm() {
  genArmed = false;
  genMidLabel(false);
  if (st_screens == ST_UI_HOME && Ossm_paired && genBleConnected) {
    genActive = true;
    genSoft = false;
    genHold = false;
    genOv = 0;
    genLastRxMs = millis();
    genReason = "";
    genSnapS = speed; genSnapD = depth; genSnapH = stroke; genSnapX = sensation;
    genLockTouch(true);
    genShowBanner("GENERATOR AKTIV - Mitte = Pause", 0xC62828, 0);
    vibrate(200, 150);
  } else {
    genAbort("Start nicht moeglich");
  }
}

// Aufruf aus homebuttonmevent() (mittleres Feld, Touch ODER per Hardware-Mitte ausgeloest).
// Rueckgabe true = Ereignis vom Generator verbraucht.
static bool genMidField() {
  if (genActive) return true;                   // waehrend des Generators: Feld tut nichts
  if (genArmed && !genHwMx) { genConfirm(); return true; }  // nur Touch bestaetigt
  return false;                                 // normales Start/Stopp
}

// ---------------------------------------------------------------- Setup
static void genSetup() {
  genBanner = lv_label_create(ui_Home);
  lv_obj_set_width(genBanner, 300);
  lv_obj_align(genBanner, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_set_style_bg_opa(genBanner, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(genBanner, lv_color_hex(0xC62828), 0);
  lv_obj_set_style_text_color(genBanner, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_align(genBanner, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_pad_all(genBanner, 4, 0);
  lv_label_set_text(genBanner, "");
  lv_obj_add_flag(genBanner, LV_OBJ_FLAG_HIDDEN);
  strncpy(genMidText, lv_label_get_text(ui_HomeButtonMText), sizeof(genMidText) - 1);
  genLoopAliveMs = millis();
}

// BLE erst starten, wenn ESP-NOW gekoppelt ist (BLE stoert sonst die Kopplung)
static void genBleInit() {
  NimBLEDevice::init("OSSM-M5-GEN");
  NimBLEServer* srv = NimBLEDevice::createServer();
  srv->setCallbacks(new GenSrvCb());
  NimBLEService* svc = srv->createService(GEN_SVC_UUID);
  NimBLECharacteristic* cmd = svc->createCharacteristic(
      GEN_CMD_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  cmd->setCallbacks(new GenCmdCb());
  genStChr = svc->createCharacteristic(
      GEN_ST_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  genStChr->setValue("{}");
  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(GEN_SVC_UUID);
  adv->setScanResponse(true);
  adv->start();
  genBleStarted = true;
  LogDebug("[GEN] BLE gestartet");
}

// ---------------------------------------------------------------- Befehle
static float genClamp(float v, float lo, float hi) {
  if (isnan(v)) return lo;
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void genApplyPattern(int idx) {
  if (idx < 0) idx = 0;
  if (idx > 7) idx = 7;
  pattern = idx;
  lv_roller_set_selected(ui_PatternS, idx, LV_ANIM_OFF);
  lv_roller_get_selected_str(ui_PatternS, patternstr, 0);
  lv_label_set_text(ui_HomePatternLabel, patternstr);
  SendCommand(PATTERN, (float)idx, OSSM_ID);
}

static void genHandleOne(String c) {
  c.trim();
  if (c.length() == 0) return;
  String u = c;
  u.toUpperCase();

  if (u == "PING") return;

  if (u == "GEN:1") {
    if (!Ossm_paired) { genReason = "nicht gekoppelt"; return; }
    if (st_screens != ST_UI_HOME) { genReason = "M5 nicht im Home-Screen"; return; }
    if (genActive) return;
    genArmed = true;
    genArmedMs = millis();
    genReason = "";
    genMidLabel(true);
    if (touch_disabled)
      genShowBanner("GENERATOR: Touch einschalten, dann BESTAETIGEN tippen", 0xEF6C00, 0);
    else
      genShowBanner("GENERATOR: Feld BESTAETIGEN antippen", 0xEF6C00, 0);
    vibrate(200, 150);
    return;
  }
  if (u == "GEN:0") {
    if (genArmed && !genActive) { genEndCommon(); genHideBanner(); genReason = "Anfrage zurueckgezogen"; return; }
    genAbort("vom iPhone beendet");
    return;
  }

  if (u.startsWith("LIM:")) {                   // jederzeit erlaubt
    String r = u.substring(4);
    int p = 0;
    while (p < (int)r.length()) {
      int e = r.indexOf(',', p);
      if (e < 0) e = r.length();
      String kv = r.substring(p, e);
      int q = kv.indexOf('=');
      if (q > 0) {
        String k = kv.substring(0, q);
        float v = kv.substring(q + 1).toFloat();
        if (v > 0) {
          if (k == "S") genLimS = v;
          else if (k == "D") genLimD = v;
          else if (k == "H") genLimH = v;
        }
      }
      p = e + 1;
    }
    return;
  }

  if (!genActive) return;                       // Bewegungsbefehle nur im aktiven Modus

  if (u == "BLK") { genOv = 0; return; }
  if (u == "ON") {
    if (genHold) { genReason = "an der M5 pausiert"; return; }
    SendCommand(ON, 0.0, OSSM_ID);
    return;
  }
  if (u == "OFF") { SendCommand(OFF, 0.0, OSSM_ID); return; }

  int q = u.indexOf(':');
  if (q != 1) return;
  char k = u.charAt(0);
  float v = u.substring(2).toFloat();

  switch (k) {
    case 'S':
      if (genOv & 1) break;
      speed = genClamp(v, 0, min(speedlimit, genLimS));
      SendCommand(SPEED, speed, OSSM_ID);
      break;
    case 'D':
      if (genOv & 2) break;
      depth = genClamp(v, 0, min(maxdepthinmm, genLimD));
      if (stroke > depth) stroke = depth;
      SendCommand(DEPTH, depth, OSSM_ID);
      SendCommand(STROKE, stroke, OSSM_ID);
      break;
    case 'H':
      if (genOv & 4) break;
      stroke = genClamp(v, 0, min(min(maxdepthinmm, genLimH), depth));
      SendCommand(STROKE, stroke, OSSM_ID);
      SendCommand(DEPTH, depth, OSSM_ID);
      break;
    case 'X':
      if (genOv & 8) break;
      sensation = genClamp(v, -100, 100);
      SendCommand(SENSATION, sensation, OSSM_ID);
      break;
    case 'P':
      genApplyPattern((int)(v + 0.5f));
      break;
  }
}

static void genSendStatus() {
  if (!genStChr) return;
  char buf[340];
  snprintf(buf, sizeof(buf),
           "{\"p\":%d,\"g\":%d,\"a\":%d,\"on\":%d,\"s\":%.0f,\"d\":%.0f,\"h\":%.0f,\"x\":%.0f,"
           "\"pt\":%d,\"ms\":%.0f,\"md\":%.0f,\"ls\":%.0f,\"ld\":%.0f,\"lh\":%.0f,\"r\":\"%s\",\"b\":%d,"
           "\"hold\":%d,\"ov\":%d,\"rmp\":%d,\"t\":%d}",
           Ossm_paired ? 1 : 0, genActive ? 1 : 0, genArmed ? 1 : 0, OSSM_On ? 1 : 0,
           speed, depth, stroke, sensation, pattern, speedlimit, maxdepthinmm,
           genLimS > 1e8 ? -1.0f : genLimS, genLimD > 1e8 ? -1.0f : genLimD, genLimH > 1e8 ? -1.0f : genLimH,
           genReason.c_str(), (int)M5.Power.getBatteryLevel(),
           genHold ? 1 : 0, (int)genOv, genSoft ? 1 : 0, touch_disabled ? 0 : 1);
  genStChr->setValue((uint8_t*)buf, strlen(buf));
  if (genBleConnected) genStChr->notify();
}

// ---------------------------------------------------------------- loop-Teil 1
// Muss in loop() VOR dem Screen-switch laufen (Tasten/Encoder noch unverbraucht)
static void genLoop() {
  const uint32_t now = millis();
  genLoopAliveMs = now;                         // Einfrier-Schutz: loop laeuft
  genHwMx = mxclick_short_waspressed;

  if (!genBleStarted) {
    if (Ossm_paired) genBleInit();
    return;
  }

  // 1) Verbindung weg / Lebenszeichen weg -> weich
  if (genBleLost) {
    genBleLost = false;
    if (genActive) genSoftLoss("Bluetooth getrennt");
    else if (genArmed) { genEndCommon(); genHideBanner(); genReason = "Bluetooth getrennt"; }
  }
  if (genActive && now - genLastRxMs > GEN_TIMEOUT_MS) genSoftLoss("kein Lebenszeichen");

  // 2) Wartet auf Bestaetigung: Zeitlimit, Ablehnen mit linker/rechter Taste
  if (genArmed) {
    if (now - genArmedMs > GEN_ARM_TIMEOUT_MS) {
      genEndCommon(); genHideBanner(); genReason = "nicht bestaetigt";
    } else if (click2_short_waspressed || click3_short_waspressed) {
      click2_short_waspressed = false;
      click3_short_waspressed = false;
      genEndCommon(); genHideBanner(); genReason = "an der M5 abgelehnt";
    }
  }

  // 3) Bedienung an der M5 waehrend des Generators
  if (genActive) {
    if (mxclick_short_waspressed) {             // Hardware-Mitte = Pause / weiter
      mxclick_short_waspressed = false;
      genToggleHold();
    }
    mxclick_long_waspressed = false;            // alles andere ignorieren
    click2_short_waspressed = false;
    click3_short_waspressed = false;
    click3_long_waspressed = false;
    click3_double_waspressed = false;

    if (st_screens != ST_UI_HOME) genAbort("Home-Screen verlassen");
    else if (!Ossm_paired) genAbort("OSSM nicht gekoppelt");
  }

  // 4) Sanft runter nach Verbindungsverlust
  if (genSoft) {
    bool input = abs(encoder1.getCount()) >= 2 || mxclick_short_waspressed ||
                 lv_slider_is_dragged(ui_homespeedslider);
    if (input || !Ossm_paired) {
      genSoft = false;                          // Mensch uebernimmt
    } else {
      float t = (float)(now - genSoftStartMs) / GEN_SOFT_MS;
      if (t > 1) t = 1;
      float target = genSoftFrom + (GEN_SOFT_SPEED - genSoftFrom) * t;
      target = roundf(target);
      if (target < speed && now - genSoftSentMs >= 200) {
        speed = target;
        SendCommand(SPEED, speed, OSSM_ID);
        genSoftSentMs = now;
      }
      if (t >= 1 && speed <= GEN_SOFT_SPEED) genSoft = false;
    }
  }

  // 5) eingegangene Befehle abarbeiten (im loop-Kontext: LVGL + ESP-NOW sicher)
  static char local[GEN_RX_MAX + 1];
  size_t ln;
  portENTER_CRITICAL(&genMux);
  ln = genRxLen;
  memcpy(local, genRx, ln);
  genRxLen = 0;
  portEXIT_CRITICAL(&genMux);
  local[ln] = 0;
  if (ln) {
    String batch = String(local);
    batch.replace("\n", ";");
    int p = 0;
    while (p < (int)batch.length()) {
      int e = batch.indexOf(';', p);
      if (e < 0) e = batch.length();
      genHandleOne(batch.substring(p, e));
      p = e + 1;
    }
  }

  // 6) Banner ausblenden, Status senden
  if (genBannerUntil && now > genBannerUntil && !genActive && !genArmed) genHideBanner();
  if (now - genStatusMs >= GEN_STATUS_MS) {
    genStatusMs = now;
    genSendStatus();
  }

  // Stand nach den Generator-Werten merken; was sich danach aendert, kam von Hand
  genSnapS = speed; genSnapD = depth; genSnapH = stroke; genSnapX = sensation;
}

// ---------------------------------------------------------------- loop-Teil 2
// Nach dem Screen-switch: Hand-Aenderungen erkennen und auf die Grenzen kappen
static void genPostLoop() {
  genHwMx = false;
  if (!genActive) return;
  uint8_t ov = genOv;
  bool capped = false;
  if (speed != genSnapS) {
    ov |= 1;
    if (speed > genLimS) { speed = genLimS; SendCommand(SPEED, speed, OSSM_ID); capped = true; }
  }
  if (depth != genSnapD) {
    ov |= 2;
    if (depth > genLimD) { depth = genLimD; capped = true; }
    if (stroke > depth) stroke = depth;
    if (capped) { SendCommand(DEPTH, depth, OSSM_ID); SendCommand(STROKE, stroke, OSSM_ID); }
  }
  if (stroke != genSnapH) {
    ov |= 4;
    float hmax = min(genLimH, depth);
    if (stroke > hmax) { stroke = hmax; SendCommand(STROKE, stroke, OSSM_ID); SendCommand(DEPTH, depth, OSSM_ID); capped = true; }
  }
  if (sensation != genSnapX) ov |= 8;
  if (capped) { genReason = "Grenze erreicht"; vibrate(255, 60); }
  if (ov != genOv) {
    genOv = ov;
    if (!capped) genReason = "Hand-Wert bis Blockende";
    LogDebugFormatted("[GEN] Hand-Werte: %d\n", (int)genOv);
  }
  genSnapS = speed; genSnapD = depth; genSnapH = stroke; genSnapX = sensation;
}
