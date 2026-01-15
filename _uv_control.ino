#include "uv_control.h"
#include "general.h" // デバッグマクロ包含

// ----------------------------------------------------------------
// [仕様切り替えスイッチ]
// 以下の行のコメントを外すと、UVランプ信号断絶時にパイロットランプが「点滅」するようになります。
// コメントアウトしたままの場合は、元の「消灯」する仕様で動作します。
#define BLINK_ON_SIGNAL_LOSS

#ifdef BLINK_ON_SIGNAL_LOSS
  // 点滅機能が有効な場合のみ、この定数を定義します
  const int UV_BLINK_INTERVAL_MS = 500; // 点滅間隔 (ミリ秒)
#endif

// --- グローバル変数 ---
int numActiveUvLamps = 0; // 実行時に確定する、実際に接続されているランプの数
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
// --- ここから追加 (11〜15本目) ---
// 注意：これらのピン番号は仮のものです。ご自身のArduino Megaの空きピンに合わせて再割り当てしてください。
/* const int UV_IN_11_PIN      = 10; // UVランプ11基目の断線警告ピン
const int UV_IN_12_PIN      = 11; // UVランプ12基目の断線警告ピン
const int UV_IN_13_PIN      = 18; // UVランプ13基目の断線警告ピン
const int UV_IN_14_PIN      = 19; // UVランプ14基目の断線警告ピン
const int UV_IN_15_PIN      = ; // UVランプ15基目の断線警告ピン
const int UV_IN_16_PIN      = 59; // UVランプ16基目の断線警告ピン */

/* const int UV_OUT_11_PIN     = 51; // UVランプ11基目のパイロットランプ出力ピン
const int UV_OUT_12_PIN     = 52; // UVランプ12基目のパイロットランプ出力ピン
const int UV_OUT_13_PIN     = 53; // UVランプ13基目のパイロットランプ出力ピン
const int UV_OUT_14_PIN     = A7; // UVランプ14基目のパイロットランプ出力ピン
const int UV_OUT_15_PIN     = A8; // UVランプ15基目のパイロットランプ出力ピン
const int UV_OUT_16_PIN     = 67; // UVランプ16基目のパイロットランプ出力ピン */
// --- ここまで追加 ---
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
  //,UV_IN_11_PIN, UV_IN_12_PIN, UV_IN_13_PIN, UV_IN_14_PIN, UV_IN_15_PIN , UV_IN_16_PIN
};
const int uvOutPins[MAX_UV_LAMPS] = {
  UV_OUT_1_PIN, UV_OUT_2_PIN, UV_OUT_3_PIN, UV_OUT_4_PIN, UV_OUT_5_PIN,
  UV_OUT_6_PIN, UV_OUT_7_PIN, UV_OUT_8_PIN, UV_OUT_9_PIN, UV_OUT_10_PIN
  // ,UV_OUT_11_PIN, UV_OUT_12_PIN, UV_OUT_13_PIN, UV_OUT_14_PIN, UV_OUT_15_PIN , UV_OUT_16_PIN
};
// UVランプの数を自動で計算
// const int NUM_UV_LAMPS = sizeof(uvInPins) / sizeof(uvInPins[0]);

// グローバル変数
SystemState uvLampState = STATE_STOPPED;
Switch uvStartSwitch   = {UV_SW_START_PIN,  HIGH, HIGH, 0};
Switch uvStopSwitch    = {UV_SW_STOP_PIN,   HIGH, HIGH, 0};

// --- UVランプ関連の関数を全てこちらに移動 ---

// ▼▼▼ この関数全体を置き換えてください ▼▼▼

// UVランプ接続チェック関数
void checkUvLampConnection() {

#ifdef BLINK_ON_SIGNAL_LOSS
  // --- ▼▼▼ 新仕様：信号断絶時に点滅するロジック ▼▼▼ ---
  // 点滅の状態を決定 (trueなら点灯、falseなら消灯)
  bool blinkState = (millis() / UV_BLINK_INTERVAL_MS) % 2;

  for(int i = 0; i < numActiveUvLamps; i++) {
    if (uvLampState == STATE_RUNNING) {
      if (digitalRead(uvInPins[i]) == LOW) { // 2025-11-21 PULLUPに変更したので異常でLOWになる
        // 信号が正常な場合は常時点灯
        digitalWrite(uvOutPins[i], HIGH);
      } else {
        // 信号が途絶えた場合は点滅
        digitalWrite(uvOutPins[i], blinkState);
      }
    } else {
      // 停止中は消灯
      digitalWrite(uvOutPins[i], LOW);
    }
  }

#else
  // --- ▼▼▼ 元の仕様：信号断絶時に消灯するロジック ▼▼▼ ---
  for(int i = 0; i < numActiveUvLamps; i++) {
    if (uvLampState == STATE_RUNNING) {
      // UVランプからの入力信号を、そのままパイロットランプの出力に反映
      digitalWrite(uvOutPins[i], digitalRead(uvInPins[i]));
      delay(1); // 信号が安定するのを待つ
    } else {
      digitalWrite(uvOutPins[i], LOW);
      delay(1); // 信号が安定するのを待つ
    }
  }
#endif

  // --- 共通のデバッグ出力 --- 2025年12月10日
  static unsigned long lastDebugPrintTime = 0;
  if (millis() - lastDebugPrintTime > 10000) {
    lastDebugPrintTime = millis();
    UV_DEBUG_PRINT("UV LANP (state=");
    UV_DEBUG_PRINT(uvLampState == STATE_RUNNING ? "RUNNING" : "STOPPED");
    UV_DEBUG_PRINTLN(")");
    UV_DEBUG_PRINT("UV Lamps Status ");
    for (int i = 0; i < numActiveUvLamps; i++) {
      int val = digitalRead(uvInPins[i]);

      // 1(HIGH) → "NONE", 0(LOW) → "NORMAL"
      const char* status = (val == HIGH) ? "NONE" : "NORMAL";

      UV_DEBUG_PRINT(i + 1);    UV_DEBUG_PRINT(":");    UV_DEBUG_PRINT(status);    UV_DEBUG_PRINT(", ");
    }
    UV_DEBUG_PRINTLN("");
  }
}

// UVランプのスイッチ入力処理
void handleUvSwitchInputs() {
  if (isButtonPressed(uvStartSwitch)) {
    if (uvLampState == STATE_STOPPED) {
      // ★追加★ ポンプの起動電流がしきい値に達するまではUVを起動させない 2025-12-09
      if (!pumpStartupOk) {
        UV_DEBUG_PRINTLN("UV Start ignored: pump startup current not yet reached threshold.");
      } else {
        uvLampState = STATE_RUNNING;
        UV_DEBUG_PRINTLN("UV Start Switch ON");
      }
    }
  }
  if (isButtonPressed(uvStopSwitch)) {
    if (uvLampState == STATE_RUNNING) {
      uvLampState = STATE_STOPPED;
      UV_DEBUG_PRINTLN("UV Stop Switch ON");
    }
  }
}

// UVランプの状態更新処理
void updateUvSystemState() {
  if (uvLampState == STATE_RUNNING) {
    digitalWrite(UV_LAMP_PIN, HIGH);
    // digitalWrite(T_CNT_PIN, HIGH);
    digitalWrite(UV_GROUP_A_PIN, RELAY_ON);// リレーはLOWアクティブだった2025-11-21
    digitalWrite(UV_GROUP_B_PIN, RELAY_ON);
  } else {
    digitalWrite(UV_LAMP_PIN, LOW);
    digitalWrite(UV_GROUP_A_PIN, RELAY_OFF);
    digitalWrite(UV_GROUP_B_PIN, RELAY_OFF);
  }
  if (pumpState == STATE_STOPPED || uvLampState == STATE_RUNNING) {
      // uvLampState = STATE_STOPPED;
      // UV_DEBUG_PRINTLN("UV lamp stopped along with the pump.");
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
    pinMode(uvInPins[i], INPUT_PULLUP);// 2025-11-21 PULLUPに変更
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
  // pinMode(LED_UV_RUN_PIN, OUTPUT);
  // pinMode(LED_UV_STOP_PIN, OUTPUT);
  // digitalWrite(LED_UV_STOP_PIN, HIGH);
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
}
bool is_uv_running() {
  // ランプがなければ、当然稼働していない
  if (numActiveUvLamps == 0) {
    return false;
  }
  return (uvLampState == STATE_RUNNING);
}

// uv_control.ino

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
    // LED_PUMP_RUN_PIN, 
    // LED_PUMP_STOP_PIN, 
    UV_LAMP_PIN
    // LED_UV_RUN_PIN , 
    // LED_UV_STOP_PIN
   };

  const char* staticLEDNames[] = {
    "EM_LAMP_PIN      ", 
    "P_LAMP_PIN       ", 
    // "LED_PUMP_RUN_PIN ", 
    // "LED_PUMP_STOP_PIN", 
    "UV_LAMP_PIN      "
    // "LED_UV_RUN_PIN   " , 
    // "LED_UV_STOP_PIN  " 
  };
  const int numStaticLEDs = sizeof(staticLEDs) / sizeof(staticLEDs[0]);
  const int main_interval = 500;  // 固定LEDの点灯/消灯の間隔 (ミリ秒)
  const int sweep_interval = 80;  // UVランプを流れるように点灯させる間隔 (ミリ秒)

  // --- シーケンス開始 ---
  UV_DEBUG_PRINTLN("Starting LED self-check sequence...");

  // 1. 固定LEDを順番に点灯・消灯
  for(int i = 0; i < numStaticLEDs; i++) {
    digitalWrite(staticLEDs[i], HIGH);
    delay(1);
    UV_DEBUG_PRINTLN(String(staticLEDNames[i]) + " ON");
  }
  delay(main_interval);  
  for(int i = 0; i < numStaticLEDs; i++) {
    digitalWrite(staticLEDs[i], LOW);
    delay(1);
    UV_DEBUG_PRINTLN(String(staticLEDNames[i]) + " OFF");
  }

  // 2. 検出したUVランプを流れるように点灯 (スイープON)
  if (lampCount > 0) {
    for(int i = 0; i < lampCount; i++) {
      digitalWrite(uvOutPins[i], HIGH);
      delay(sweep_interval);
    }
    delay(50); // 全点灯後に少し間を空ける

    // 3. 検出したUVランプを流れるように消灯 (スイープOFF)
    for(int i = 0; i < lampCount; i++) {
      digitalWrite(uvOutPins[i], LOW);
      delay(sweep_interval);
    }
  }

  // delay(main_interval / 2); // 少し間を空ける

  for(int j = 0; j < 2; j++) {
    // 4. 全てのLEDを同時に点灯・消灯を2回
    for(int i = 0; i < numStaticLEDs; i++) { // 制御ランプ点灯
      digitalWrite(staticLEDs[i], HIGH);
      delay(1);
    }
    for(int i = 0; i < lampCount; i++) {  // UVランプ点灯
      digitalWrite(uvOutPins[i], HIGH);
      delay(1);
    }
    delay(50);

    for(int i = 0; i < numStaticLEDs; i++) { // 制御ランプ消灯
      digitalWrite(staticLEDs[i], LOW);
      delay(1);
    }
    for(int i = 0; i < lampCount; i++) { // UVランプ消灯
      digitalWrite(uvOutPins[i], LOW);
      delay(1);
    }
  }
  UV_DEBUG_PRINTLN("LED self-check sequence complete.");
}