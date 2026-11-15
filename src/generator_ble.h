// =====================================================================
//  Generator-Modus (BLE) fuer die OSSM-M5-Remote (Ortlof v2.2.0)
//
//  iPhone (Bluefy + Generator-Seite)  --BLE-->  M5  --ESP-NOW-->  OSSM-Stroke
//
//  Die M5 nimmt Werte per BLE entgegen und schickt sie ueber die normale
//  SendCommand()-Funktion an die OSSM, genau wie beim Drehen am Encoder.
//  Einheiten = die Werte, die auch auf dem M5-Display stehen.
//
//  Sicherheit / Bedienung (v2: gen2):
//   - Aktivieren nur im Home-Screen und nur nach Bestaetigung an der M5
//     (Mitte-Taste druecken).
//   - Beenden des Generators NUR ueber die Seite (GEN:0) - oder automatisch:
//     Lebenszeichen 2 s weg / BLE getrennt / Home-Screen verlassen -> OFF + aus.
//   - Mitte-Taste waehrend des Generators = PAUSE (Maschine AUS, Generator
//     bleibt aktiv). Erneut Mitte = weiter (Maschine AN). Solange pausiert,
//     ignoriert die M5 ein ON vom iPhone.
//   - Encoder/Slider (Speed, Tiefe, Hub, Sensation) hoch UND runter erlaubt:
//     der Hand-Wert gilt bis zum naechsten Block (BLK), Generator-Werte fuer
//     diesen Parameter werden bis dahin ignoriert.
//   - Linke/rechte Taste (kurz, lang, doppelt) werden ignoriert.
//   - Grenzen (LIM) fuer Speed/Tiefe/Hub gelten fuer Generator-Werte.
//   - Werte werden nur angenommen, solange der Modus aktiv ist.
//
//  Befehle (Text, mehrere mit ';' oder Zeilenumbruch getrennt):
//   GEN:1            Modus anfragen (M5 zeigt "Mitte druecken")
//   GEN:0            Modus beenden (OFF)
//   PING             Lebenszeichen (jeder Schreibzugriff zaehlt auch)
//   S:<wert>         Speed
//   D:<wert>         Tiefe
//   H:<wert>         Hub (Stroke), nie groesser als Tiefe
//   X:<wert>         Sensation -100..100
//   P:<index>        Pattern (0 SimpleStroke, 1 TeasingPounding, 2 RoboStroke,
//                    3 HalfnHalf, 4 Deeper, 5 StopNGo, 6 Insist, 7 Knot)
//   ON / OFF         Maschine an / aus (ON wird bei M5-Pause ignoriert)
//   BLK              neuer Block: Hand-Werte verfallen, Generator-Werte gelten wieder
//   LIM:S=<v>,D=<v>,H=<v>   Grenzen setzen (nur wenn Modus nicht aktiv)
//
//  Status (Notify/Read, JSON), ca. alle 500 ms:
//   {"p":gekoppelt,"g":modus,"a":wartet_auf_bestaetigung,"on":OSSM_On,
//    "s":speed,"d":depth,"h":stroke,"x":sensation,"pt":pattern,
//    "ms":speedlimit,"md":maxdepth,"ls":..,"ld":..,"lh":..,"r":"grund","b":akku,
//    "hold":Pause_an_M5,"ov":Hand-Bitmaske(1 Speed,2 Tiefe,4 Hub,8 Sensation)}
// =====================================================================
#pragma once
#include <NimBLEDevice.h>

#define GEN_SVC_UUID  "6f73736d-6765-6e00-8000-00805f9b0001"
#define GEN_CMD_UUID  "6f73736d-6765-6e00-8000-00805f9b0002"
#define GEN_ST_UUID   "6f73736d-6765-6e00-8000-00805f9b0003"

#define GEN_TIMEOUT_MS      2000   // Lebenszeichen
#define GEN_ARM_TIMEOUT_MS  15000  // so lange wartet "Mitte druecken"
#define GEN_STATUS_MS       500

// ---------- zwischen BLE-Task und loop() geteilt ----------
static portMUX_TYPE genMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t genLastRxMs = 0;
static volatile bool genBleConnected = false;
static volatile bool genBleLost = false;     // Verbindung abgerissen
#define GEN_RX_MAX 512
static char genRx[GEN_RX_MAX];                // eingegangene Befehle (geschuetzt durch genMux, kein malloc im kritischen Abschnitt)
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
static bool genHold = false;            // Pause per Mitte-Taste
static uint8_t genOv = 0;               // Hand-Werte im aktuellen Block
static float genSnapS = 0, genSnapD = 0, genSnapH = 0, genSnapX = 0;

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

// Modus beenden: Maschine aus, Grund merken, Banner zeigen
static void genAbort(const char* reason) {
  bool wasOn = genActive;
  genActive = false;
  genArmed = false;
  genHold = false;
  genOv = 0;
  genReason = reason;
  if (wasOn) {
    SendCommand(OFF, 0.0, OSSM_ID);
    vibrate(255, 300);
    String t = String("GENERATOR AUS: ") + reason;
    genShowBanner(t.c_str(), 0xC62828, 6000);
  } else {
    genHideBanner();
  }
  LogDebugFormatted("[GEN] abort: %s\n", reason);
}

static void genSetup() {
  // Banner auf dem Home-Screen (unsichtbar, bis gebraucht)
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
    genShowBanner("GENERATOR: Mitte druecken zum Starten", 0xEF6C00, 0);
    vibrate(200, 150);
    return;
  }
  if (u == "GEN:0") { genAbort("vom iPhone beendet"); return; }

  if (u.startsWith("LIM:")) {
    if (genActive) return;  // waehrend der Session keine Aenderung
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

  if (!genActive) return;  // Bewegungsbefehle nur im aktiven Modus

  if (u == "BLK") { genOv = 0; return; }
  if (u == "ON")  {
    if (genHold) { genReason = "an der M5 pausiert"; return; }
    SendCommand(ON, 0.0, OSSM_ID); return;
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
  char buf[300];
  snprintf(buf, sizeof(buf),
           "{\"p\":%d,\"g\":%d,\"a\":%d,\"on\":%d,\"s\":%.0f,\"d\":%.0f,\"h\":%.0f,\"x\":%.0f,"
           "\"pt\":%d,\"ms\":%.0f,\"md\":%.0f,\"ls\":%.0f,\"ld\":%.0f,\"lh\":%.0f,\"r\":\"%s\",\"b\":%d,"
           "\"hold\":%d,\"ov\":%d}",
           Ossm_paired ? 1 : 0, genActive ? 1 : 0, genArmed ? 1 : 0, OSSM_On ? 1 : 0,
           speed, depth, stroke, sensation, pattern, speedlimit, maxdepthinmm,
           genLimS > 1e8 ? -1.0f : genLimS, genLimD > 1e8 ? -1.0f : genLimD, genLimH > 1e8 ? -1.0f : genLimH,
           genReason.c_str(), (int)M5.Power.getBatteryLevel(), genHold ? 1 : 0, (int)genOv);
  genStChr->setValue((uint8_t*)buf, strlen(buf));
  if (genBleConnected) genStChr->notify();
}

// Muss in loop() VOR dem Screen-switch laufen (Encoder-Zaehler noch unverbraucht)
static void genLoop() {
  if (!genBleStarted) {
    if (Ossm_paired) genBleInit();
    return;
  }
  const uint32_t now = millis();

  // 1) Verbindung weg?
  if (genBleLost) {
    genBleLost = false;
    if (genActive || genArmed) genAbort("Bluetooth getrennt");
  }

  // 2) Bestaetigung an der M5 (Mitte-Taste) - der Klick wird verbraucht
  if (genArmed) {
    if (mxclick_short_waspressed) {
      mxclick_short_waspressed = false;
      genArmed = false;
      if (st_screens == ST_UI_HOME && Ossm_paired && genBleConnected) {
        genActive = true;
        genLastRxMs = now;
        genReason = "";
        genHold = false;
        genOv = 0;
        genSnapS = speed; genSnapD = depth; genSnapH = stroke; genSnapX = sensation;
        genShowBanner("GENERATOR AKTIV - Mitte = Pause", 0xC62828, 0);
      } else {
        genAbort("Start nicht moeglich");
      }
    } else if (now - genArmedMs > GEN_ARM_TIMEOUT_MS) {
      genArmed = false;
      genReason = "nicht bestaetigt";
      genHideBanner();
    } else if (click2_short_waspressed || click3_short_waspressed) {
      genArmed = false;
      genReason = "an der M5 abgelehnt";
      genHideBanner();
    }
  }

  // 3) Bedienung an der M5 waehrend des Generators
  if (genActive) {
    if (mxclick_short_waspressed) {            // Mitte = Pause / weiter
      mxclick_short_waspressed = false;
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
    // linke/rechte Taste ignorieren (kein Menue, kein Eject, kein Sensation-Reset)
    mxclick_long_waspressed = false;
    click2_short_waspressed = false;
    click3_short_waspressed = false;
    click3_long_waspressed = false;
    click3_double_waspressed = false;

    if (st_screens != ST_UI_HOME) {
      genAbort("Home-Screen verlassen");
    } else if (!Ossm_paired) {
      genAbort("OSSM nicht gekoppelt");
    } else if (now - genLastRxMs > GEN_TIMEOUT_MS) {
      genAbort("kein Lebenszeichen");
    }
  }

  // 4) eingegangene Befehle abarbeiten (im loop-Kontext: LVGL + ESP-NOW sicher)
  static char local[GEN_RX_MAX + 1];
  size_t ln;
  portENTER_CRITICAL(&genMux);
  ln = genRxLen;
  memcpy(local, genRx, ln);
  genRxLen = 0;
  portEXIT_CRITICAL(&genMux);
  local[ln] = 0;
  String batch = String(local);
  if (batch.length()) {
    batch.replace("\n", ";");
    int p = 0;
    while (p < (int)batch.length()) {
      int e = batch.indexOf(';', p);
      if (e < 0) e = batch.length();
      genHandleOne(batch.substring(p, e));
      p = e + 1;
    }
  }

  // 5) Banner ausblenden, Status senden
  if (genBannerUntil && now > genBannerUntil && !genActive && !genArmed) genHideBanner();
  if (now - genStatusMs >= GEN_STATUS_MS) {
    genStatusMs = now;
    genSendStatus();
  }

  // Stand nach den Generator-Werten merken; was sich danach aendert, kam von Hand
  genSnapS = speed; genSnapD = depth; genSnapH = stroke; genSnapX = sensation;
}

// Nach dem Screen-switch aufrufen: erkennt Hand-Aenderungen (Encoder/Slider)
static void genPostLoop() {
  if (!genActive) return;
  uint8_t ov = genOv;
  if (speed != genSnapS) ov |= 1;
  if (depth != genSnapD) ov |= 2;
  if (stroke != genSnapH) ov |= 4;
  if (sensation != genSnapX) ov |= 8;
  if (ov != genOv) {
    genOv = ov;
    genReason = "Hand-Wert bis Blockende";
    LogDebugFormatted("[GEN] Hand-Werte: %d\n", (int)genOv);
  }
  genSnapS = speed; genSnapD = depth; genSnapH = stroke; genSnapX = sensation;
}
