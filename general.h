#ifndef GENERAL_H_INCLUDED
#define GENERAL_H_INCLUDED
//====================================================
// general.h - 共通設定・マクロ・型定義

// --- 変更点: Serial.printをマクロ化 ---
// DEBUG_MODEが定義されている時だけ、DEBUG_PRINTがSerial.printとして機能します。
// 定義されていない場合は、コンパイラによって完全に無視され、コードに残りません。
#ifdef DEBUG_MODE
  #define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif
#ifdef UV_DEBUG_MODE
  #define UV_DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define UV_DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define UV_DEBUG_PRINT(...)
  #define UV_DEBUG_PRINTLN(...)
#endif
#ifdef PU_DEBUG_MODE
  #define PU_DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define PU_DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define PU_DEBUG_PRINT(...)
  #define PU_DEBUG_PRINTLN(...)
#endif

// --- ここから共通型定義・共有シンボル宣言 ---

// ポンプ／UV 共通の状態
// enum SystemState { STATE_STOPPED, STATE_RUNNING };

// メイン .ino に実装がある関数
//bool isButtonPressed(Switch &sw);

// メイン .ino 側で定義しているランプ用ピン
extern const int EM_LAMP_PIN;
extern const int P_LAMP_PIN;
extern const int LED_PUMP_RUN_PIN;
extern const int LED_PUMP_STOP_PIN;
// uv_control.h などに追加
extern const int UV_SW_START_PIN;
extern const int UV_SW_STOP_PIN;

// ★追加★ ポンプ起動完了フラグ／エラーフラグ 2025-12-09
extern bool pumpStartupOk;
extern bool pumpStartupError;
//====================================================
// [追加] UV側：片側過半数断線警告フラグ
//====================================================
extern bool uvHalfBrokenWarning;

struct Switch;   // ← 型が後で出てくることを宣言だけする
extern Switch uvStopSwitch;
extern Switch pumpStopSwitch;

//====================================================
// [追加] リレーがLOWアクティブの場合の定義
//  - RELAY_ON  = LOW  （コイルON）
//  - RELAY_OFF = HIGH （コイルOFF）
//====================================================
static const uint8_t RELAY_ON  = LOW;
static const uint8_t RELAY_OFF = HIGH;

const int MAX_UV_LAMPS = 10;

//====================================================
// UV接続検知入力のモード設定
//====================================================

// 0: INPUT（外付け抵抗あり想定）
// 1: INPUT_PULLUP（抵抗なし検証用）
#define UV_IN_USE_INTERNAL_PULLUP  0

// ディップスイッチ読み出しマクロ (LOW=ON)今後変更したときにここだけ変えれば良いようににする
#define DIP_ON(pin)  (digitalRead(pin) == LOW)

#endif  // GENERAL_H_INCLUDED
// --- ここまで共通型定義・共有シンボル宣言 ---