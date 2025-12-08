

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

// --- ここから共通型定義・共有シンボル宣言 ---

// ポンプ／UV 共通の状態
// enum SystemState { STATE_STOPPED, STATE_RUNNING };

// チャタリング対策付きスイッチ
// struct Switch {
//   const int pin;
//   int lastReading;
//   int stableState;
//   unsigned long lastDebounceTime;
// };

// 実体はメイン .ino 側で定義する
//extern SystemState pumpState;
//extern Switch pumpStartSwitch;
//extern Switch pumpStopSwitch;

// メイン .ino に実装がある関数
//bool isButtonPressed(Switch &sw);

// メイン .ino 側で定義しているランプ用ピン
extern const int EM_LAMP_PIN;
extern const int P_LAMP_PIN;
extern const int LED_PUMP_RUN_PIN;
extern const int LED_PUMP_STOP_PIN;
struct Switch;   // ← 型が後で出てくることを宣言だけする
extern Switch uvStopSwitch;
extern Switch pumpStopSwitch;