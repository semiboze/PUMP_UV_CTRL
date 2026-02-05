#include "uv_control.h"
#include "general.h" // デバッグマクロ包含

// ----------------------------------------------------------------
// [仕様切り替えスイッチ]
// 以下の行のコメントを外すと、UVランプ信号断絶時にパイロットランプが「点滅」するようになります。
// コメントアウトしたままの場合は、元の「消灯」する仕様で動作します。

// [点滅仕様] 接触不良/ランプ切れ(=NG)は常に点滅させる
//=========================================================
const int UV_BLINK_INTERVAL_MS = 500; // 点滅間隔 (ミリ秒)

// --- グローバル変数 ---
int numActiveUvLamps = 0; // 実行時に確定する、実際に接続されているランプの数

//=========================================================
// [追加] Start拒否時の「未接続チャンネル点滅」用（非ブロッキング）
//  - UVは起動できないが、配線/未接続をユーザーに通知したい用途
//=========================================================
static bool uvMissingBlinkActive = false;
//=========================================================
// [追加] 配線チェック優先：Start押下でUVリレーだけ先にON
//  - pumpStartupOk が false でもリレーをONにできる（配線確認用）
//  - Stopで必ず解除
//=========================================================
static bool uvRelayForceOnForCheck = false;

//=========================================================
// [追加] UV「運転ランプ」ラッチ
//  - UV_STARTを押したら点灯開始
//  - UV_STOPを押すまで点灯を維持
//  - UV本体リレーのON/OFFとは分離（＝警告点滅中でも運転ランプは点けられる）
//=========================================================
static bool uvRunLampLatched = false;

static unsigned long uvCheckStartMs = 0;  // 点滅チェック開始時刻
//=========================================================
// [追加] UV断線状態の保持用
//  - true  = NG（断線/浮き）
//  - false = OK
//=========================================================
static bool uvBroken[MAX_UV_LAMPS] = {false};
static bool uvHalfBrokenWarning = false;         // ★追加★ UVランプの一部断線警告フラグ
// UV断線チェックを有効にするかどうか
static bool uvFaultCheckEnabled = false;

//=========================================================
// [追加] UV入力の「接続/断線/浮き」を外付け抵抗なしで判定する
//  - あなたの実測:
//      接続(通電) = 5V (HIGH)
//      断線       = 0V (LOW)
//  - INPUT_PULLUP だけだと「未接続(浮き)」が HIGH に見えてしまうため、
//    「プルアップ有り/無しで2回読む」ことで浮きを推定する。
//=========================================================
enum UvSense {
  UV_SENSE_OK,       // 外部がHIGHを駆動（接続正常）
  UV_SENSE_BROKEN,   // 外部がLOWを駆動（断線/異常）
  UV_SENSE_FLOATING  // 浮いてる（未接続/配線抜け/電源OFFでHi-Z等）
};
// 判定関数（実体は後ろでもOK）
static UvSense readUvSenseNoResistor(int pin);

// ▼▼▼ UV機能定義 ▼▼▼
const int UV_SW_START_PIN   = 5 , UV_SW_STOP_PIN        = 6;
const int UV_LAMP_PIN       = 7; // ★★★ T_CNT_PINの定義を削除 ★★★
const int UV_IN_1_PIN       = 20; // UVランプ1基目の断線警告ピン
const int UV_IN_2_PIN       = 21; // UVランプ2基目の断線警告ピン
const int UV_IN_3_PIN       = 22; // UVランプ3基目の断線警告ピン
const int UV_IN_4_PIN       = 23; // UVランプ4基目の断線警告ピン
const int UV_IN_5_PIN       = 24; // UVランプ5基目の断線警告ピン
const int UV_IN_6_PIN       = 25; // UVランプ6基目の断線警告ピン
const int UV_IN_7_PIN       = 26; // UVランプ7基目の断線警告ピン
const int UV_IN_8_PIN       = 27; // UVランプ8基目の断線警告ピン
const int UV_IN_9_PIN       = 28; // UVランプ9基目の断線警告ピン
const int UV_IN_10_PIN      = 29; // UVランプ10基目の断線警告ピン

const int UV_OUT_1_PIN      = 30; // UVランプ1基目のパイロットランプ出力ピン
const int UV_OUT_2_PIN      = 31; // UVランプ2基目のパイロットランプ出力ピン
const int UV_OUT_3_PIN      = 32; // UVランプ3基目のパイロットランプ出力ピン
const int UV_OUT_4_PIN      = 33; // UVランプ4基目のパイロットランプ出力ピン
const int UV_OUT_5_PIN      = 34; // UVランプ5基目のパイロットランプ出力ピン
const int UV_OUT_6_PIN      = 35; // UVランプ6基目のパイロットランプ出力ピン
const int UV_OUT_7_PIN      = 36; // UVランプ7基目のパイロットランプ出力ピン
const int UV_OUT_8_PIN      = 37; // UVランプ8基目のパイロットランプ出力ピン
const int UV_OUT_9_PIN      = 38; // UVランプ9基目のパイロットランプ出力ピン
const int UV_OUT_10_PIN     = 39; // UVランプ10基目のパイロットランプ出力ピン
const int UV_GROUP_A_PIN    = 40; // UVランプグループA制御ピン
const int UV_GROUP_B_PIN    = 41; // UVランプグループB制御ピン
const int LED_UV_RUN_PIN    = 47; // 操作盤の稼働灯(現在ハード未実装)
// const int LED_UV_STOP_PIN   = 48; //操作盤の稼働灯・停止灯
//const int RPM_ANALOG_IN_PIN     = 0; // 可変抵抗 (ポテンショメーター) 接続ピン 
//const int CURRENT_ANALOG_IN_PIN = 1; // ポンプの電流センサー接続ピン
//const int P_SW_START_PIN        = 2; // ポンプ起動スイッチ接続ピン
//const int P_SW_STOP_PIN         = 3; // ポンプ停止スイッチ接続ピン
//const int P_LAMP_PIN            = 4; // ポンプ運転表示ランプ接続ピン
//const int UV_SW_START_PIN       = 5; // UVランプ起動スイッチ接続ピン
//const int UV_SW_STOP_PIN        = 6; // UVランプ停止スイッチ接続ピン
//const int UV_LAMP_PIN           = 7; // UVランプ運転表示ランプ接続ピン
//const int EM_LAMP_PIN           = 8; // 電磁弁表示ランプ接続ピン
//const int T_CNT_PIN             = 9; // UVランプタイマー接続ピン
//const int PIN_CLK1              = 12; // TM1637 1 (回転数表示) 接続ピン
//const int PIN_DIO1              = 13; // TM1637 1 (回転数表示) 接続ピン
//const int PIN_CLK2              = 14; // TM1637 2 (電流表示) 接続ピン
//const int PIN_DIO2              = 15; // TM1637 2 (電流表示) 接続ピン
//const int PIN_CLK3              = 16; // TM1637 3 (タイマー表示) 接続ピン
//const int PIN_DIO3              = 17; // TM1637 3 (タイマー表示) 接続ピン
// const int THERESHOLD_HARD_CODE = 42; // アナログダイヤルで閾値調整する場合このピンをGNDに落とす
// const int LED_PUMP_RUN_PIN      = 45; 
// const int LED_PUMP_STOP_PIN     = 46;
//const int LED_UV_RUN_PIN        = 47; // 操作盤の稼働灯
//const int LED_UV_STOP_PIN       = 48; //操作盤の稼働灯・停止灯
//const int LED_ISR_PIN           = 49; // タイマー割り込み動作確認用LED
//const int LED_SERIAL_RX_PIN     = 50; // シリアル受信確認用LED

// 対応する可能性のある最大ランプ数でピン配列を定義しておく
const int uvInPins[MAX_UV_LAMPS] = {
  UV_IN_1_PIN, UV_IN_2_PIN, UV_IN_3_PIN, UV_IN_4_PIN, UV_IN_5_PIN,
  UV_IN_6_PIN, UV_IN_7_PIN, UV_IN_8_PIN, UV_IN_9_PIN, UV_IN_10_PIN
};
const int uvOutPins[MAX_UV_LAMPS] = {
  UV_OUT_1_PIN, UV_OUT_2_PIN, UV_OUT_3_PIN, UV_OUT_4_PIN, UV_OUT_5_PIN,
  UV_OUT_6_PIN, UV_OUT_7_PIN, UV_OUT_8_PIN, UV_OUT_9_PIN, UV_OUT_10_PIN
};
//=========================================================
// [追加] NGラッチ（接触不良の瞬断を見逃さない）
//  - NGを検出したら一定時間点滅を維持する
//=========================================================
static unsigned long uvNgLatchUntilMs[MAX_UV_LAMPS] = {0};

// NGを何msラッチするか（目視できる最低時間）
const unsigned long UV_NG_LATCH_MS = 3000;

// UVランプの数を自動で計算
// const int NUM_UV_LAMPS = sizeof(uvInPins) / sizeof(uvInPins[0]);

// グローバル変数
// SystemState uvLampState = STATE_STOPPED;
Switch uvStartSwitch   = {UV_SW_START_PIN,  HIGH, HIGH, 0};
Switch uvStopSwitch    = {UV_SW_STOP_PIN,   HIGH, HIGH, 0};

//=========================================================
// UVグループ境界
//=========================================================
static inline int uvGroupSize() {
  return numActiveUvLamps / 2;
}

//=========================================================
// [改] 外付け抵抗なしでも「未接続(浮き)」をNGにするUV入力判定
//  - 目的：未接続チャンネル(浮き)を「OK」と誤判定させない
//  - 方法：プルアップ無し/有りで2回読んで「浮き」を推定
//=========================================================
static inline bool isUvSignalOk(int pin) {

  // 既存の enum UvSense と判定関数を使う（ファイル内に既に宣言済み）
  // UvSense s = readUvSenseNoResistor(pin);2026-01-22
  return (digitalRead(pin) == HIGH);

  // OK（外部がHIGHを駆動）だけ true
  // BROKEN(外部LOW駆動) / FLOATING(未接続) は false → 点滅側へ
  // return (s == UV_SENSE_OK);
}
// UV入力ピンの pinMode を返す
static inline uint8_t getUvInputPinMode() {
#if UV_IN_USE_INTERNAL_PULLUP
  return INPUT_PULLUP;
#else
  return INPUT;
#endif
}

//=========================================================
// [追加] UVインジケータ表示（OK=点灯 / NG=点滅）
//  - NGはラッチして一定時間点滅を継続
//=========================================================
static void driveUvIndicator(int idx, bool okNow) {
  unsigned long now = millis();
  bool blinkState = (now / UV_BLINK_INTERVAL_MS) % 2;

  // NGを検出したらラッチ延長
  if (!okNow) {
    uvNgLatchUntilMs[idx] = now + UV_NG_LATCH_MS;
  }

  // ラッチ期間中はNG扱い（点滅）
  bool latchedNg = (now < uvNgLatchUntilMs[idx]);

  if (!latchedNg && okNow) {
    // OK（ラッチなし）→点灯
    digitalWrite(uvOutPins[idx], HIGH);
  } else {
    // NG（現時点NG or ラッチ中）→点滅
    digitalWrite(uvOutPins[idx], blinkState ? HIGH : LOW);
  }
}

// UVランプ接続チェック関数
void checkUvLampConnection() {
  // ★ガード★ UVが有効でないときは何もしない
  if (!uvFaultCheckEnabled) {
    return;
  }
  // チェック開始直後はUV側信号が安定しないので待つ
  if (uvMissingBlinkActive && (millis() - uvCheckStartMs < 3000)) {
    for (int i = 0; i < numActiveUvLamps; i++) {
      digitalWrite(uvOutPins[i], LOW);
      uvNgLatchUntilMs[i] = 0; // ★安定待ち中はラッチもクリア
    }
    return;
  }

  bool blinkState = (millis() / UV_BLINK_INTERVAL_MS) % 2;

  if (uvMissingBlinkActive) {
    for (int i = 0; i < numActiveUvLamps; i++) {
      bool ok = isUvSignalOk(uvInPins[i]);

      // OKは点灯、NGは点滅（常に）
      if (ok) digitalWrite(uvOutPins[i], HIGH);
      else    digitalWrite(uvOutPins[i], blinkState);
    }
    // return;
  }

  for (int i = 0; i < numActiveUvLamps; i++) {
    //=========================================================
    // RUNNING：通常監視（OK=点灯 / NG=点滅(ラッチあり)）
    //=========================================================
    if (uvLampState == STATE_RUNNING) {
      bool ok = isUvSignalOk(uvInPins[i]);
      driveUvIndicator(i, ok);
      continue;
    }

    //=========================================================
    // STOPPED：
    //  - チェック中(uvMissingBlinkActive=true)は表示したいので driveUvIndicator() を使う
    //  - チェック中でないなら完全消灯＋ラッチクリア
    //=========================================================
    if (uvMissingBlinkActive) {
      bool ok = isUvSignalOk(uvInPins[i]);
      driveUvIndicator(i, ok);         // ★STOPPEDでもチェック中は点滅を生かす
    } else {
      digitalWrite(uvOutPins[i], LOW); // ★通常STOPは消灯
      uvNgLatchUntilMs[i] = 0;         // ★通常STOPはラッチも消す（分かりやすさ優先）
    }
  }
  //=========================================================
  // [デバッグ] 10秒ごと
  //=========================================================
  static unsigned long lastDebugPrintTime = 0;
  if (millis() - lastDebugPrintTime > 10000) {
    lastDebugPrintTime = millis();

    UV_DEBUG_PRINT("UV LANP (state=");
    UV_DEBUG_PRINT(uvLampState == STATE_RUNNING ? "RUNNING" : "STOPPED");
    UV_DEBUG_PRINTLN(")");

    UV_DEBUG_PRINT("UV Lamps Status ");
    for (int i = 0; i < numActiveUvLamps; i++) {
      bool ok = isUvSignalOk(uvInPins[i]);

      // ★追加★ UV断線状態を保持（警告判定用）
      uvBroken[i] = !ok;

      driveUvIndicator(i, ok);

      UV_DEBUG_PRINT(i + 1);
      UV_DEBUG_PRINT(":");
      UV_DEBUG_PRINT(ok ? "OK" : "NG");
      UV_DEBUG_PRINT(", ");
    }
    UV_DEBUG_PRINTLN("");
  }
  //=========================================================
  // [追加] UV断線による警告ランプ制御
  //  - UVが有効なときのみ判定
  //=========================================================
  if (uvFaultCheckEnabled) {
    if (isUvHalfBroken()) {
      digitalWrite(EM_LAMP_PIN, HIGH);
    } else {
      // ポンプ由来の警告が無い前提でのみ消す
      if (!pumpStartupError) {
        digitalWrite(EM_LAMP_PIN, LOW);
      }
    }
  }
}

// UVランプのスイッチ入力処理
void handleUvSwitchInputs() {

  //=========================================================
  // [STOP] 最優先：STOPで全部解除
  //=========================================================
  if (isButtonPressed(uvStopSwitch)) {

    // ★追加★ UV断線チェック停止
    uvFaultCheckEnabled = false;

    // ★追加★ UV関連警告は必ず消す
    digitalWrite(EM_LAMP_PIN, LOW);

    if (uvLampState == STATE_RUNNING) {
      uvLampState = STATE_STOPPED;
      persist.uv = 0;
      savePersistState();
      UV_DEBUG_PRINTLN("UV Stop Switch ON");
    }

    // ★追加★：未接続点滅ラッチ解除
    uvMissingBlinkActive = false;

    // ★追加★：運転ランプもSTOPまで維持していたので解除
    uvRunLampLatched = false;

    // 配線チェック用に強制ONしていたUVリレーを解除
    uvRelayForceOnForCheck = false;
  
    // ★既存★：インジケータ全消灯
    for (int i = 0; i < numActiveUvLamps; i++) {
      digitalWrite(uvOutPins[i], LOW);
    }

    return;
  }

  //=========================================================
  // [START] START押下
  //=========================================================
  if (isButtonPressed(uvStartSwitch)) {
    uvRunLampLatched = true;

    // ★追加★ UV断線チェックを開始
    uvFaultCheckEnabled = true;

    UV_DEBUG_PRINT("START PIN28 mode=");
    UV_DEBUG_PRINT(digitalRead(28));
    UV_DEBUG_PRINT("   PIN29 mode=");
    UV_DEBUG_PRINTLN(digitalRead(29));

    if (uvLampState == STATE_STOPPED) {

      // 起動条件未達：UV本体は動かさないが、警告点滅はラッチ
      if (!pumpStartupOk) {
        UV_DEBUG_PRINTLN("UV Start: CHECK (relay ON) - pump not ready");

        // ★配線チェック：リレーだけONにする
        uvRelayForceOnForCheck = true;

        // ★未接続/断線も見たいので点滅ラッチON
        uvMissingBlinkActive = true;
        uvCheckStartMs = millis();   // ★追加  
        // ★★★ ここを追加 ★★★
        persist.uv = 1;
        savePersistState();
        return;
      }

      // 起動条件OK：通常RUNNING
      uvLampState = STATE_RUNNING;
      persist.uv = 1;
      savePersistState();

      UV_DEBUG_PRINTLN("UV Start Switch ON");
      // ★即時に状態表示（10秒待たない）
      UV_DEBUG_PRINT("UV Lamps Status ");
      for (int i = 0; i < numActiveUvLamps; i++) {
        bool ok = isUvSignalOk(uvInPins[i]);
        UV_DEBUG_PRINT(i + 1);
        UV_DEBUG_PRINT(":");
        UV_DEBUG_PRINT(ok ? "OK" : "NG");
        UV_DEBUG_PRINT(", ");
      }
      UV_DEBUG_PRINTLN("");
      // RUNNINGになったら「未接続点滅」は解除（通常ロジックへ）
      uvMissingBlinkActive = false;
      UV_DEBUG_PRINT("PIN28 mode=");
      UV_DEBUG_PRINT(digitalRead(28));
      UV_DEBUG_PRINT("   PIN29 mode=");
      UV_DEBUG_PRINTLN(digitalRead(29));
    }
  }
}

// UVランプの状態更新処理
void updateUvSystemState() {

  //=========================================================
  // [運転ランプ] Start押下後は「Stop押下まで」点灯を維持したい
  //  - RUNNING中は当然点灯
  //  - RUNNINGでなくても uvRunLampLatched が true なら点灯（警告点滅中など）
  //=========================================================
  if (uvLampState == STATE_RUNNING || uvRunLampLatched) {
    digitalWrite(UV_LAMP_PIN, HIGH);
  } else {
    digitalWrite(UV_LAMP_PIN, LOW);
  }
  //=========================================================
  // [UV本体リレー] RUNNING または チェック強制ON のときにON
  //=========================================================
  if (uvLampState == STATE_RUNNING || uvRelayForceOnForCheck) {
    digitalWrite(UV_GROUP_A_PIN, RELAY_ON);
    digitalWrite(UV_GROUP_B_PIN, RELAY_ON);
  } else {
    digitalWrite(UV_GROUP_A_PIN, RELAY_OFF);
    digitalWrite(UV_GROUP_B_PIN, RELAY_OFF);
  }
}

// --- メインから呼び出される公開関数 ---

void uv_setup(int detected_lamp_count) {
    numActiveUvLamps = detected_lamp_count;
  
    // ▼▼▼ ガード節 ▼▼▼
  // ランプ数が0なら、この先の初期化処理をすべてスキップする
  if (numActiveUvLamps == 0) {
    return;
  }
  
  // ピンモード設定のループを、固定値ではなく検出したランプ数で行う
  for(int i = 0; i < numActiveUvLamps; i++) {
    pinMode(uvInPins[i], getUvInputPinMode());
    pinMode(uvOutPins[i], OUTPUT);
  } 

  pinMode(UV_SW_START_PIN, INPUT_PULLUP);
  pinMode(UV_SW_STOP_PIN, INPUT_PULLUP);
  pinMode(UV_LAMP_PIN, OUTPUT);

  // --- UVグループは必ずOFFで立ち上げる（グリッチ防止）---
  // 先に出力ラッチをOFF側へ（INPUT状態でもpull-up的に働くことがある）
  pinMode(UV_GROUP_A_PIN, OUTPUT);
  pinMode(UV_GROUP_B_PIN, OUTPUT);
  digitalWrite(UV_GROUP_A_PIN, RELAY_OFF);
  digitalWrite(UV_GROUP_B_PIN, RELAY_OFF);
}

void uv_loop_task() {
  // ▼▼▼ ガード節 ▼▼▼
  // ランプ数が0なら、ループ処理をすべてスキップする
  if (numActiveUvLamps == 0) {
    return;
  }
  
  // loop内で実行していたUV関連の処理をここに記述
  handleUvSwitchInputs();
  updateUvSystemState();
  checkUvLampConnection();
  updateUvHalfBrokenWarning();   // ★追加
}
bool is_uv_running() {
  // ランプがなければ、当然稼働していない
  if (numActiveUvLamps == 0) {
    return false;
  }
  return (uvLampState == STATE_RUNNING);
}

// ... (is_uv_running() 関数の下など、ファイルの末尾に) ...

// ================================================================
// ▼▼▼ ここから追記 ▼▼▼
// ================================================================
/**
 * @brief 起動時にLEDのセルフチェックを行う（UVランプ対応版）
 * @param lampCount メインファイルで検出されたランプの数
 */
void runStartupLedSequence(int lampCount) {
  const int staticLEDs[] = {
    EM_LAMP_PIN, 
    P_LAMP_PIN, 
    UV_LAMP_PIN
   };

  const char* staticLEDNames[] = {
    "EM_LAMP_PIN      ", 
    "P_LAMP_PIN       ", 
    "UV_LAMP_PIN      "
  };
  const int numStaticLEDs = sizeof(staticLEDs) / sizeof(staticLEDs[0]);
  const int main_interval = 500;  // 固定LEDの点灯/消灯の間隔 (ミリ秒)
  const int sweep_interval = 250;  // UVランプを流れるように点灯させる間隔 (ミリ秒)
  const int Sequence_interval = 50; // シーケンス開始前の待機時間 (ミリ秒)

  // --- シーケンス開始 ---
  UV_DEBUG_PRINTLN("Starting LED self-check sequence...");

  //                                                      1. 制御ランプを順番に点灯・消灯
  for(int i = 0; i < numStaticLEDs; i++) {                // 制御ランプ点灯
    digitalWrite(staticLEDs[i], HIGH);                    // 制御ランプ点灯
    delay(50);                                            // 少し待つ
    UV_DEBUG_PRINTLN(String(staticLEDNames[i]) + " ON");  // デバッグ表示
  }

  // 2. 検出したUVランプを流れるように点灯と消灯 (スイープON)
  if (lampCount > 0) {                                    // UVランプがある場合のみ実行
    for(int i = 0; i < lampCount; i++) {                  // UVランプ点灯     
      digitalWrite(uvOutPins[i], HIGH);                   // UVランプ点灯
      delay(100);                              // 少し待つ            
      digitalWrite(uvOutPins[i], LOW);                    // UVランプ消灯      
    }
    delay(50);                                           // 全消灯後に少し間を空ける

    // 3. 検出したUVランプを流れるように点灯と消灯
    for(int i = 0; i < lampCount; i++) {                  // UVランプ点灯    
      digitalWrite(uvOutPins[i], HIGH);                   // UVランプ点灯
      delay(100);                              // 少し待つ              
    }
    for(int i = lampCount - 1; i >= 0; i--) {
      digitalWrite(uvOutPins[i], LOW);
      delay(100);
    }
  }
  for(int i = 0; i < numStaticLEDs; i++) {                // 制御ランプ消灯
    digitalWrite(staticLEDs[i], LOW);                     // 制御ランプ消灯
    delay(50);                                           // 少し待つ
    UV_DEBUG_PRINTLN(String(staticLEDNames[i]) + " OFF"); // デバッグ表示
  }

  int lampLoopCount = lampCount < 1 ? 1 : lampCount; // ランプが0の場合は1回だけループする
  // 4. 全てのLEDを同時に点灯・消灯をlampLoopCount回
  for(int j = 0; j < lampLoopCount; j++) {
    // 4. 全てのLEDを同時に点灯・消灯をj回
    for(int i = 0; i < numStaticLEDs; i++) { // 制御ランプ点灯
      digitalWrite(staticLEDs[i], HIGH);
      delay(1);
    }
    for(int i = 0; i < lampCount; i++) {  // UVランプ点灯
      digitalWrite(uvOutPins[i], HIGH);
      delay(1);
    }
    delay(250);

    for(int i = 0; i < numStaticLEDs; i++) { // 制御ランプ消灯
      digitalWrite(staticLEDs[i], LOW);
      delay(1);
    }
    for(int i = 0; i < lampCount; i++) { // UVランプ消灯
      digitalWrite(uvOutPins[i], LOW);
      delay(1);
    }
    delay(250);
  }
  UV_DEBUG_PRINTLN("LED self-check sequence complete.");
}

static UvSense readUvSenseNoResistor(int pin) {

  //=========================================================
  // 【重要】AVR(Mega2560)の注意点
  //  - pinMode(pin, INPUT) だけでは「前回のINPUT_PULLUP」の状態(PORT=1)が残る場合がある
  //  - そのままだと「プルアップ無しで読む」つもりが、実際はプルアップが残り v_no_pull=HIGH になりやすい
  //  - 対策：INPUTにした直後に digitalWrite(pin, LOW) を必ず入れて内部プルアップを確実にOFFにする
  //=========================================================
  // ここはINPUT_PULLUPを指定しないでソース通りに一度INPUTで読んでみて外部抵抗の有無を確認する

  // --- 1) プルアップ無しで読む（外部が本当に駆動してるかを見る） ---
  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);       // ★これが肝：内部プルアップを確実にOFF
  delayMicroseconds(200);       // 少し長めに（配線容量/ノイズ対策）
  int v_no_pull = digitalRead(pin);

  // --- 2) プルアップ有りで読む（浮きをHIGHに寄せる） ---
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(200);
  int v_pull = digitalRead(pin);

  // --- 判定 ---
  UvSense result;
  if (v_pull == LOW) {
    // プルアップしてもLOW → 外部がLOWを駆動（断線/異常/あるいは仕様がActive-Low）
    result = UV_SENSE_BROKEN;
  } else if (v_no_pull == HIGH) {
    // プルアップ無しでもHIGH → 外部がHIGHを駆動（正常）
    result = UV_SENSE_OK;
  } else {
    // プルアップ無しではHIGHでないが、プルアップでHIGH → 浮き（未接続）の典型
    result = UV_SENSE_FLOATING;
  }

  // --- 後始末：次回の誤判定を防ぐ（★ここも重要） ---
  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);       // 内部プルアップOFF状態で戻す

  return result;
}
void uv_force_restore(bool run) {
  if (run) {
    uvLampState = STATE_RUNNING;
    uvFaultCheckEnabled = true;   // ★追加★
    uvRunLampLatched = true;      // ★必須
    uvMissingBlinkActive = false;
    uvRelayForceOnForCheck = false;
  } else {
    uvLampState = STATE_STOPPED;
    uvRunLampLatched = false;
    uvRelayForceOnForCheck = false;
  }
}
//=========================================================
// [追加] UV左右筒の断線数カウント
//=========================================================
static int countUvBrokenLeft() {
  int cnt = 0;
  int sideCount = numActiveUvLamps / 2;

  for (int i = 0; i < sideCount; i++) {
    if (uvBroken[i]) cnt++;
  }
  return cnt;
}

static int countUvBrokenRight() {
  int cnt = 0;
  int sideCount = numActiveUvLamps / 2;

  for (int i = sideCount; i < numActiveUvLamps; i++) {
    if (uvBroken[i]) cnt++;
  }
  return cnt;
}
//=========================================================
// [追加] 片側過半数断線判定
//=========================================================
static bool isUvHalfBroken(int brokenCount) {
  int sideCount = numActiveUvLamps / 2;
  return (brokenCount > (sideCount / 2));
}
//=========================================================
// [追加] UV過半数断線警告の更新
//=========================================================
static void updateUvHalfBrokenWarning() {

  // UVが存在しない構成では警告しない
  if (numActiveUvLamps < 2) {
    uvHalfBrokenWarning = false;
    return;
  }

  int leftBroken  = countUvBrokenLeft();
  int rightBroken = countUvBrokenRight();

  uvHalfBrokenWarning =
    isUvHalfBroken(leftBroken) ||
    isUvHalfBroken(rightBroken);
}
//=========================================================
// [追加] UV左右筒ごとの過半数断線チェック
//  - どちらか一方がNGなら true を返す
//=========================================================
static bool isUvHalfBroken() {

  int half = uvGroupSize();
  if (half == 0) return false; // 念のため

  int brokenA = 0;
  int brokenB = 0;

  // --- グループA ---
  for (int i = 0; i < half; i++) {
    if (!isUvSignalOk(uvInPins[i])) {
      brokenA++;
    }
  }

  // --- グループB ---
  for (int i = half; i < numActiveUvLamps; i++) {
    if (!isUvSignalOk(uvInPins[i])) {
      brokenB++;
    }
  }

  // 過半数しきい値
  int threshold = (half / 2) + 1;

  if (brokenA >= threshold || brokenB >= threshold) {
    return true;
  }

  return false;
}
