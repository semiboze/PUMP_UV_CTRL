/**
 * @file dynamic_rpm_pump_controller.ino
 * @brief 統合・改良版 ポンプ＆UVランプコントローラー (ハードウェア自動検知版)
 * @version 20250910_R2 (Revision 2)
 *
 * @details
 * 起動時にハードウェアのピン設定を読み取り、回転数制御モードを自動で切り替えます。
 *
 * [回転数モードの切り替え方法]
 * - ピン42とGNDをショート  : 手動回転数モード(可変抵抗による)。 解放でNORMAL_MAX_RPMで動作 
 * - ピン43とGNDをショート  : 電流しきい値可変モード(可変抵抗による)。解放で固定しきい値(PUMP_CURRENT_THRESHOLD_DEFAULTで動作)
 * [UVランプモデルの自動検出]
 * - ピンA3からA6ピンで本数を２進数表現  15本までプログラム上では実装可能
 * 例 :           A6 A5 A4 A3
 *      0本の場合  0  0  0  0  → 0本
 *      2本の場合  0  0  1  0  → 2本
 *      4本の場合  0  1  0  0  → 4本
 *     12本の場合  1  1  0  0  → 12本
 * [注意]ピン42, 43, A3～A7はプルアップ抵抗内蔵のINPUT_PULLUPモードで使用してください。
 */
#define DEBUG_MODE          // デバッグ用シリアル出力を有効にする場合はコメントアウトを外す
#define UV_DEBUG_MODE       // UV装置制御関連のデバッグ
#define PU_DEBUG_MODE       // ポンプ制御関連のデバッグ

/*  ポンプ起動後電流が閾値に到達するまでの監視タイマー(秒)
    ポンプと水槽の距離によって変動させる必要があるが最大長さに合わせておくのもあり*/
#define DEFINE_CURRENT_STATUS 10 

//====================================================
// [追加] RPM送信ごとのログを出すか（流れるので通常は0推奨）
//====================================================
#define DEBUG_RPM_EACH_SEND 0
const char* FirmwareVersion = "20251225_R1";
// ----------------------------------------------------------------
// ▼▼▼ 動作設定 ▼▼▼
// ----------------------------------------------------------------
// 固定回転数モードで動作する場合の回転数を設定します。
const int NORMAL_MAX_RPM = 2500;        // 固定回転数モードでの最大回転数
const int PRIMING_DURATION_SEC = 0;     // プライミングを行う時間（秒）
const float HOLD_DURATION_SEC = 1.0;    // 最高回転数での保持時間（秒）
const int PRIMING_MAX_RPM      = 2500;  // プライミング中の最大回転数
const int PRIMING_MIN_RPM      = 2000;  // プライミング中の最小回転数
const float PRIMING_CYCLE_SEC = 4.0;    // プライミングの1サイクルの時間（秒）
// 回転数モードを判定するためのピン番号
const int MANUAL_RPM_MODE_PIN = 42;     // アナログダイヤルで回転数可変にする場合このピンをGNDに落とす

// ポンプの電流閾値可変モードにするピン番号
const int MANUAL_THRESHOLD_MODE_PIN = 43;  // アナログダイヤルで閾値調整する場合このピンをGNDに落とす

// ----------------------------------------------------------------
// ライブラリのインクルード
// ----------------------------------------------------------------
#include <FlexiTimer2.h>
#include <math.h> // ← この行を追加
#include "TM1637.h"
#include "general.h" // デバッグマクロ包含
// 【変更点】UVコントロール用のヘッダファイルをインクルード
#include "uv_control.h"
// ----------------------------------------------------------------
// ピン定義
// ----------------------------------------------------------------
const int HOURMETER_RESET_PIN  = 49; // ★追加★ アワーメーターリセット出力用GPIO（未使用ピン利用）2025年12月11日
const int PIN_CLK1          = 12, PIN_DIO1              = A9; 
const int PIN_CLK2          = 14, PIN_DIO2              = 15; 
const int PIN_CLK3          = 16, PIN_DIO3              = 17;
const int P_SW_START_PIN    = 2 , P_SW_STOP_PIN         = 3;
const int P_LAMP_PIN        = 4;
const int EM_LAMP_PIN = 8;      // 非常停止ランプ (EM_LAMP_PIN) 接続ピン
const int T_CNT_PIN         = 9;        // ★★★ T_CNT_PINの定義をこちらに移動 ★★★
const int INPUT_FEEDBACK_LED_PIN = 44;  // スイッチ押下中インジケータLED
const int LED_PUMP_RUN_PIN  = 45, LED_PUMP_STOP_PIN     = 46; // 操作盤の稼働灯・停止灯 現在ハード未実装
const int FAN_CTRL_PIN = 48;                // 冷却ファン制御ピン
const int LED_ISR_PIN       = LED_BUILTIN, LED_SERIAL_RX_PIN     = 50;
const int RPM_ANALOG_IN_PIN = A0 ;       // 回転数調整ダイヤル可変抵抗 (ポテンショメーター) 接続ピン
const int CURRENT_ANALOG_IN_PIN = A1;    // ポンプの電流センサー接続ピン
const int THRESHOLD_ANALOG_IN_PIN = A2;  // [変更点] 電流しきい値調整用のダイヤル可変抵抗を接続するアナログピン

// UVランプ装着数自動検知用のピン定義（4ビットバイナリ）ただし10本まで対応
const int UV_DETECT_BIT0_PIN = A3; // 1加算 (2^0)
const int UV_DETECT_BIT1_PIN = A4; // 2加算 (2^1)
const int UV_DETECT_BIT2_PIN = A5; // 4加算 (2^2)
const int UV_DETECT_BIT3_PIN = A6; // 8加算 (2^3)
// ----------------------------------------------------------------
// グローバル変数
// ----------------------------------------------------------------
enum SystemState { STATE_STOPPED, STATE_RUNNING };
SystemState pumpState = STATE_STOPPED;
const int SERIAL0_BAUD_RATE    = 115200;
const int TIMER_INTERVAL_MS   = 50;           // タイマー割り込み間隔 (ms)
const int LED_ISR_BLINK_INTERVAL_SEC = 1;     // 1秒ごとに点滅させる。変えたければここを変える 2025年12月10日
const int DEBOUNCE_DELAY_MS   = 50;           // スイッチのチャタリング防止時間 (ms)
const unsigned long PUMP_TIMEOUT_SEC  = 60;   // 既存の過電流チェック用の時間（既存仕様を維持）
// ★追加★ ポンプ起動時の「低電流チェック」の猶予時間
const unsigned long PUMP_STARTUP_TIMEOUT_SEC  = DEFINE_CURRENT_STATUS;  // 起動後電流が閾値に到達するまでの監視タイマー秒 2025-12-09
const int PUMP_CURRENT_THRESHOLD_DEFAULT = 512; // デフォルトのポンプ電流しきい値
int PUMP_CURRENT_THRESHOLD      = PUMP_CURRENT_THRESHOLD_DEFAULT;  // ポンプ運転を判断する電流のしきい値
const int COMMAND_INTERVAL_MS = 300;          // コマンド送信間隔 (ms)
// [変更点] 実行時に決定される回転数制御モードを格納する変数
enum varControlMode { MODE_VOLUME, MODE_FIXED };
varControlMode rpmControlMode;
varControlMode currThresholdCntMode; // [変更点] 電流しきい値可変モード

unsigned long pumpStartTime = 0;
// ★追加★ ポンプ起動時の電流監視用 変数 2025-12-09
bool pumpStartupOk        = false;   // 一度でもしきい値以上の電流を検出したら true
bool pumpStartupError     = false;   // 「低電流エラー」で停止したら true
int  maxCurrentSinceStart = 0;       // 起動開始から今までの最大電流(ADC値)

volatile bool processFlag = false;
int rpm_value = 0;
int lastCurrentPeak = 512;
int detectedLamps = 0;              // 検出したランプ数を格納する変数

TM1637 tm1(PIN_CLK1, PIN_DIO1);     // 回転数表示用
TM1637 tm2(PIN_CLK2, PIN_DIO2);     // 電流しきい値表示用
TM1637 tm3(PIN_CLK3, PIN_DIO3);     // 最後の電流ピーク値表示用

// ▼▼▼ ここから追加 ▼▼▼ 2025年9月16日 インバーターからの受信用
// インバーターからの応答データ受信バッファ
const int INVERTER_RESPONSE_SIZE = 13; // 仕様書より応答データは13バイト
byte inverterResponseBuffer[INVERTER_RESPONSE_SIZE];
int responseByteCount = 0;
// ▲▲▲ ここまで追加 ▲▲▲

// 追加2025年12月11日 両方停止出力ピン制御用変数
bool hourMeterResetComboLatched = false;  // STOPボタン同時押しでのリセット済みフラグ

struct Switch {
  const int pin;
  int lastReading, stableState;
  unsigned long lastDebounceTime;
};
// チャタリング対策付きスイッチ
// ===== 暫定：Switch が見えていない場合の応急定義 =====
// typedef struct {
//   uint8_t pin;
//   bool    active_low;          // trueならLOWが押下
//   uint16_t debounce_ms;        // チャタリング
//   bool    last_raw;
//   bool    stable;
//   unsigned long last_change_ms;
// } Switch;

Switch pumpStartSwitch = {P_SW_START_PIN, HIGH, HIGH, 0};
Switch pumpStopSwitch  = {P_SW_STOP_PIN,  HIGH, HIGH, 0};

// プロトタイプ宣言
void initializePins();            // ピンの初期化
void initializeDisplays();        // TM1637ディスプレイの初期化
void timerInterrupt();            // タイマー割り込み処理
// bool isButtonPressed(Switch &sw); // スイッチのチャタリング防止付き押下検出
void handleSwitchInputs();        // スイッチ入力処理
void updateSystemState();         // ポンプとUVランプの状態更新
void updateDisplays();            // 3桁表示のため、1000以上は999として表示
void handleSerialCommunication(); // シリアルからのコマンド受信可否によるLED点灯消灯
void handlePeriodicTasks();       // タイマー処理でトリガーされる定期処理（コマンド送信、ピーク電流測定）
void measurePeakCurrent();        // ポンプのピーク電流を測定
int getTargetRpm();               // 目標回転数を取得
int calculateRpmFromVolume();     // 可変抵抗から回転数を計算
void trim(char* str);             // 文字列の前後の空白を削除
void sendRpmCommand(int rpm);     // 回転数コマンド送信 2026-01-08 変更
void updateCurrentThreshold();    // しきい値を更新する関数のプロトタイプ宣言
void updateTCntPin();             // ★★★ T_CNT_PINを制御する関数のプロトタイプ宣言 ★★★
void runStartupLedSequence(int);  // 起動時のLEDシーケンス
void resetUvHourMeter();          // ★追加★ UVアワーメーターリセット関数プロトタイプ 2025年12月11日
void both_stop_check_task();      // ★追加★★ 両方停止出力ピンの制御タスクプロトタイプ 2025年12月11日
//====================================================
// [追加] ポンプ(インバーター)通信用シリアルの統一窓口
// Serial  : USBデバッグ用
// Serial1 : ポンプ通信用（Mega: TX1=D18, RX1=D19）
//====================================================
#define PUMP_SERIAL   Serial1
//====================================================
// [ファン制御] ここだけ見ればON/OFFが分かるようにする
// もし「LOWで回る」なら FAN_ACTIVE_LOW を 1 にする
//====================================================
// 1: LOWでON（アクティブLOW） / 0: HIGHでON（アクティブHIGH）
#define FAN_ACTIVE_LOW  0
#if FAN_ACTIVE_LOW
  const uint8_t FAN_ON_LEVEL  = LOW;
  const uint8_t FAN_OFF_LEVEL = HIGH;
#else
  const uint8_t FAN_ON_LEVEL  = HIGH;
  const uint8_t FAN_OFF_LEVEL = LOW;
#endif

inline void fan_on()  { digitalWrite(FAN_CTRL_PIN, FAN_ON_LEVEL);  }
inline void fan_off() { digitalWrite(FAN_CTRL_PIN, FAN_OFF_LEVEL); }

// ----------------------------------------------------------------
// setup() - 初期化処理
// ----------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL0_BAUD_RATE);
  //--- ポンプ通信用（インバーター） ---
  PUMP_SERIAL.begin(2400);       // ← いまインバーターに合わせている速度にする
  DEBUG_PRINTLN("--- System Start ---");
  // ★追加★ 確認コマンドを最低1回送る（仕様書要求）
  // ここでは3回だけ軽くリトライ（応答処理は後述の受信側でconfirmedにする）
  for (int i = 0; i < 3; i++) {
    sendConfirmCommand();
    delay(60);  // 応答時間0～100msの記載があるので少し待つ
  }

  // [変更点] 起動時にハードウェア設定を読み込み、RPM制御モードを決定
  pinMode(MANUAL_RPM_MODE_PIN, INPUT_PULLUP);
  delay(5); // プルアップが安定するのを待つ

  // --- UVランプモデルの自動検出 ---
  // --- UVランプモデルの自動検出 (4ビットバイナリ) ---
  pinMode(UV_DETECT_BIT0_PIN, INPUT_PULLUP);
  pinMode(UV_DETECT_BIT1_PIN, INPUT_PULLUP);
  pinMode(UV_DETECT_BIT2_PIN, INPUT_PULLUP);
  pinMode(UV_DETECT_BIT3_PIN, INPUT_PULLUP);
  delay(5); // プルアップが安定するのを待つ

  pinMode(HOURMETER_RESET_PIN, OUTPUT);   // ★追加★★ 両方停止出力ピン 初期化
  digitalWrite(HOURMETER_RESET_PIN, LOW); // ★追加★★ 初期状態はLOW
  pinMode(INPUT_FEEDBACK_LED_PIN, OUTPUT);    // スイッチ押下中インジケータLEDピン初期化
  digitalWrite(INPUT_FEEDBACK_LED_PIN, LOW);  // 初期状態は消灯

  // 4つのピンの状態を読み取り、2進数としてランプ数を計算
  detectedLamps = 0; // 0に初期化
  if (digitalRead(UV_DETECT_BIT0_PIN) == LOW) { detectedLamps += 1; } // 1の位
  if (digitalRead(UV_DETECT_BIT1_PIN) == LOW) { detectedLamps += 2; } // 2の位
  if (digitalRead(UV_DETECT_BIT2_PIN) == LOW) { detectedLamps += 4; } // 4の位
  if (digitalRead(UV_DETECT_BIT3_PIN) == LOW) { detectedLamps += 8; } // 8の位
  if (detectedLamps > MAX_UV_LAMPS) {
    UV_DEBUG_PRINTLN("Warning: Detected UV lamp count exceeds 10. Limiting to 10.");
    detectedLamps = MAX_UV_LAMPS; // 安全のため最大10本に制限
  } 
  UV_DEBUG_PRINT("UV Lamp Bits (8,4,2,1): ");
  UV_DEBUG_PRINT(digitalRead(UV_DETECT_BIT3_PIN) == LOW ? "1" : "0");
  UV_DEBUG_PRINT(digitalRead(UV_DETECT_BIT2_PIN) == LOW ? "1" : "0");
  UV_DEBUG_PRINT(digitalRead(UV_DETECT_BIT1_PIN) == LOW ? "1" : "0");
  UV_DEBUG_PRINT(digitalRead(UV_DETECT_BIT0_PIN) == LOW ? "1" : "0");
  UV_DEBUG_PRINT(" -> Detected Lamps: ");
  UV_DEBUG_PRINTLN(detectedLamps);

  if (digitalRead(MANUAL_RPM_MODE_PIN) == LOW) {
    rpmControlMode = MODE_VOLUME;
  } else {
    rpmControlMode = MODE_FIXED;
  }
  pinMode(MANUAL_THRESHOLD_MODE_PIN, INPUT_PULLUP); // [変更点] しきい値調整フラグ用ピンの初期化
  delay(5); // プルアップが安定するのを待つ
  if(digitalRead(MANUAL_THRESHOLD_MODE_PIN) == LOW) {
    PU_DEBUG_PRINTLN("Current Threshold Adjustment: ENABLED");
    currThresholdCntMode = MODE_VOLUME;
  } else {
    PU_DEBUG_PRINTLN("Current Threshold Adjustment: DISABLED");
    currThresholdCntMode = MODE_FIXED;
  }

  pinMode(FAN_CTRL_PIN, OUTPUT);    // 冷却ファン制御ピンの初期化
  digitalWrite(FAN_CTRL_PIN, FAN_OFF_LEVEL);  // ★ 起動時は必ずOFF

  DEBUG_PRINT("Firmware: ");
  DEBUG_PRINT(FirmwareVersion);
  if (rpmControlMode == MODE_VOLUME) {
    PU_DEBUG_PRINTLN(" (RPM Control: Volume)");
  } else {
    PU_DEBUG_PRINTLN(" (RPM Control: Fixed)");
  }
  
  initializePins();
  // 検出したランプの数を渡して、UVモジュールを初期化
  // detectedLampsが0なら、uv_setupは何もしない
  uv_setup(detectedLamps); 
  
  // initializeDisplays();

  FlexiTimer2::set(TIMER_INTERVAL_MS, timerInterrupt);
  FlexiTimer2::start();
  runStartupLedSequence(detectedLamps); // LEDの起動シーケンスを実行
  DEBUG_PRINTLN("Initialization complete. Starting main loop.");
}

// ----------------------------------------------------------------
// loop() - メインループ
// ----------------------------------------------------------------
void loop() {
  updateCurrentThreshold();     // [変更点] 毎ループ、可変抵抗の値を読み込んでしきい値を更新
  handleSwitchInputs();         // スイッチ入力処理
  updateSystemState();          // ポンプの状態更新

  uv_loop_task(); // UV機能のループ処理を呼び出す

  updateTCntPin();              // ★★★ T_CNT_PINの状態を更新 ★★★
  updateDisplays();             // 3桁表示のため、1000以上は999として表示
  handleSerialCommunication();  // シリアルからのコマンド受信可否によるLED点灯消灯
  handlePeriodicTasks();        // タイマー処理でトリガーされる定期処理（コマンド送信、ピーク電流測定）

  both_stop_check_task();       // ★追加★★ 両方停止出力ピンの制御タスク

  // debug_print_raw_buttons_on_change(); // デバッグ用：スイッチの状態変化を生ログ出力
  // debug_fan_pin_on_change();
  updateInputFeedbackLed();     // スイッチ押下中LEDの更新
}

// ================================================================
// 機能別関数
// ================================================================

// ★★★ T_CNT_PINの出力を制御する新設関数 ★★★
void updateTCntPin() {
  bool isPumpRunning = (pumpState == STATE_RUNNING);
  bool isUvRunning = false;

  isUvRunning = is_uv_running(); // UV制御モジュールから状態を取得


  if (isPumpRunning || isUvRunning) {
    digitalWrite(T_CNT_PIN, HIGH);
  } else {
    digitalWrite(T_CNT_PIN, LOW);
  }
}

// [変更点] 電流しきい値を可変抵抗から読み取り更新する関数
void updateCurrentThreshold() {
  if(currThresholdCntMode == MODE_FIXED) {
    // ピンが開放されている場合、しきい値調整を行わない
    PUMP_CURRENT_THRESHOLD = PUMP_CURRENT_THRESHOLD_DEFAULT; // デフォルト値に戻す
  } else {
    // ピンがGNDに接続されている場合、可変抵抗からしきい値を読み取る
    int sensorValue = analogRead(THRESHOLD_ANALOG_IN_PIN);
    // analogReadの値(0-1023)を、しきい値の範囲(例: 550-950)に変換する
    // この範囲は実際の運用に合わせて調整してください。
    PUMP_CURRENT_THRESHOLD = map(sensorValue, 0, 1023, 500, 950);
  }
}

// ピンの初期化
void initializePins() {
  pinMode(P_SW_START_PIN, INPUT_PULLUP);
  pinMode(P_SW_STOP_PIN, INPUT_PULLUP);
  pinMode(EM_LAMP_PIN, OUTPUT);
  pinMode(P_LAMP_PIN, OUTPUT);
  // pinMode(LED_PUMP_RUN_PIN, OUTPUT);
  // pinMode(LED_PUMP_STOP_PIN, OUTPUT);
  pinMode(LED_ISR_PIN, OUTPUT);
  
  pinMode(LED_SERIAL_RX_PIN, OUTPUT);
  pinMode(T_CNT_PIN, OUTPUT); // ★★★ T_CNT_PINの初期化をこちらに移動 ★★★
  delay(5);
  digitalWrite(T_CNT_PIN, LOW);// ★★★ 初期状態はLOW ★★★

  digitalWrite(LED_PUMP_STOP_PIN, HIGH);
  // ★追加★ アワーメーターリセット出力ピン の初期化 2025年12月11日
  pinMode(HOURMETER_RESET_PIN, OUTPUT);
  digitalWrite(HOURMETER_RESET_PIN, LOW); // 通常時は非リセット状態（アクティブHIGH前提）
}

// TM1637ディスプレイの初期化
void initializeDisplays() {
  tm1.init();
  tm1.set(BRIGHT_TYPICAL);
  tm1.clearDisplay();
  tm2.init();
  tm2.set(BRIGHT_TYPICAL);
  tm2.clearDisplay();
  tm3.init();
  tm3.set(BRIGHT_TYPICAL);
  tm3.clearDisplay();
}

// チャタリング防止付きスイッチ押下検出 2026-01-08 コメント化
// bool isButtonPressed(Switch &sw) {
//   int currentReading = digitalRead(sw.pin);
//   if (currentReading != sw.lastReading) {
//     sw.lastDebounceTime = millis();
//   }
  
//   if ((millis() - sw.lastDebounceTime) > DEBOUNCE_DELAY_MS) {
//     if (currentReading != sw.stableState) {
//       sw.stableState = currentReading;
//       if (sw.stableState == LOW) {
//         sw.lastReading = currentReading;
//         return true;
//       } 
//     } 
//   } 
  
//   sw.lastReading = currentReading;
//   return false;
// }
bool isButtonPressed(Switch &sw) {
  // INPUT_PULLUP 前提：押すと LOW
  bool current = digitalRead(sw.pin);

  // 変化があったらデバウンス開始
  if (current != sw.lastReading) {
    sw.lastDebounceTime = millis();
  }

  bool pressed_event = false;

  // 一定時間安定していたら「確定状態」を更新
  if ((millis() - sw.lastDebounceTime) > DEBOUNCE_DELAY_MS) {
    if (current != sw.stableState) {
      sw.stableState = current;

      // 押下イベントは「LOWに落ちた瞬間」だけ
      if (sw.stableState == LOW) {
        pressed_event = true;
      }
    }
  }

  // ★最重要：毎回更新（これが無いと離した判定が永遠に終わらない）
  sw.lastReading = current;

  return pressed_event;
}

// ▼▼▼ ここから追加 ▼▼▼ 2025年9月16日 インバーターに停止コマンドを送信する関数
/**
 * @brief インバーターに停止コマンド（回転数0）を送信する
 */
void sendStopCommand() {
  byte stop_command[8] = {0x00, 0x01, 0x10, 0x02, 0x00, 0x01, 0x00, 0x00};
  byte sum = 0;
  for(int i=0; i < 7; i++) {
    sum += stop_command[i];
  }
  stop_command[7] = 0x55 - sum; // チェックサムを計算
  pump_write8(stop_command, "STOP"); // インバーターへ停止コマンド送信
  PU_DEBUG_PRINTLN("Sent: Stop Command to Inverter");
}
// ▲▲▲ ここまで追加 ▲▲▲
// 回転数コマンド送信（ポンプ起動時や定期送信で使用）
void sendRpmCommand(int rpm) {
  double analog_value_f = (rpm + 5.092) / 17.945;
  if (analog_value_f > 139.6) analog_value_f = 139.6;
  if (analog_value_f < 34.0) analog_value_f = 34.0;
  byte analog_value = (byte)analog_value_f;

  // ★重要★ 継続運転なら D5=0xFF を推奨（運転継続側）
  byte command[8] = {0x00, 0x01, 0x10, 0x02, analog_value, 0xFF, 0x00, 0x00};

  byte sum = 0;
  for(int i = 0; i < 7; i++) { sum += command[i]; }
  command[7] = 0x55 - sum;
  pump_write8(command, "RPM"); // インバーターへ回転数コマンド送信
#if DEBUG_RPM_EACH_SEND
  PU_DEBUG_PRINT("Sent: Set RPM Command to Inverter - RPM=");
  PU_DEBUG_PRINTLN(rpm);
#endif
}

// ============================================================
// ★追加★ 固定コード確認コマンド(0x00) 送信
// 仕様書：電源投入後最低1度は送信。リターンがあるまで再送。:contentReference[oaicite:7]{index=7}
// ============================================================

// 確認完了フラグ（ACKが取れたらtrueにする）
volatile bool inverter_confirmed = false;

// 9バイト送信（確認コマンド用）
void pump_write9(const uint8_t cmd[9], const char* label) {
  PUMP_SERIAL.write(cmd, 9);

  // デバッグ表示（任意）
  PU_DEBUG_PRINT("[PUMP] ");
  PU_DEBUG_PRINT(label);
  PU_DEBUG_PRINT(" : ");
  for (int i = 0; i < 9; i++) {
    if (cmd[i] < 0x10) PU_DEBUG_PRINT('0');
    PU_DEBUG_PRINT(cmd[i], HEX);
    PU_DEBUG_PRINT(' ');
  }
  PU_DEBUG_PRINTLN();
}

// チェックサム：D0～D8の総和が0x55になるようにする
void sendConfirmCommand() {
  // 仕様書の例そのまま（最後はchecksum=0x38）
  uint8_t cmd[9] = {0x00, 0x01, 0x00, 0x03, 0x01, 0x12, 0x00, 0x06, 0x00};

  uint8_t sum = 0;
  for (int i = 0; i < 8; i++) sum += cmd[i];
  cmd[8] = (uint8_t)(0x55 - sum);

  pump_write9(cmd, "CONFIRM");
}


/**
 * @brief ポンプを停止させる（状態変更とコマンド送信）
 */
void stopPump() {

  //====================================================
  // [重要] 「ポンプ未接続」や「状態ズレ」でも確実に表示を落とす
  //====================================================
  pumpState = STATE_STOPPED;

  //--- 稼働ランプOFF（※あなたの実ピン名に合わせて）---
  digitalWrite(P_LAMP_PIN, LOW);        // 稼働ランプ（リレー/ランプ）
  // digitalWrite(LED_PUMP_RUN_PIN, LOW);  // パネル上のRUN LED
  // digitalWrite(LED_PUMP_STOP_PIN, HIGH);// STOP LEDがあるならON
  fan_off();          // ★追加：停止で確実にファンOFF

  //--- インバータ停止コマンド（接続されていないなら副作用なし）---
  sendStopCommand();

  PU_DEBUG_PRINTLN("Pump Stop Sequence Executed (lamp forced OFF).");
}

// スイッチ検出処理
void handleSwitchInputs() {
  // ポンプスタートボタン
  if (isButtonPressed(pumpStartSwitch)) {
    if (pumpState == STATE_STOPPED) {
      PU_DEBUG_PRINTLN("Pump Start Switch ON");

      //====================================================
      // [改善] 体感レスポンス最優先：押した瞬間にランプ点灯
      // ・ユーザーに「受け付けた」を即表示
      //====================================================
      digitalWrite(P_LAMP_PIN, HIGH);  // まず点ける（この後エラーならstopPumpで消える）
      fan_on();                        // ファンも先に回してOK

      //====================================================
      // 起動前に一度停止コマンドを送り、インバーターの状態をリセット
      //====================================================
      PU_DEBUG_PRINTLN("Sending pre-start stop command to clear inverter state.");
      sendStopCommand();

      // ★ここは長いほど体感が悪化するので最小に
      delay(5);

      //====================================================
      // 起動監視関連をリセット
      //====================================================
      pumpStartupOk        = false;
      pumpStartupError     = false;
      maxCurrentSinceStart = 0;
      digitalWrite(EM_LAMP_PIN, LOW);  // 以前のエラーで点灯していた非常停止ランプを消灯

      //====================================================
      // 状態遷移
      //====================================================
      pumpState = STATE_RUNNING;
      pumpStartTime = millis();

      //====================================================
      // [改善] 体感レスポンス向上のため、起動直後に回転数コマンドを即送信
      //====================================================
      rpm_value = getTargetRpm();
      sendRpmCommand(rpm_value);
    }
  }
  // ポンプストップボタン
  if (isButtonPressed(pumpStopSwitch)) {

    // 押されたことは必ずログ
    PU_DEBUG_PRINT("Pump Stop Switch ON (state=");
    PU_DEBUG_PRINT(pumpState == STATE_RUNNING ? "RUNNING" : "STOPPED");
    PU_DEBUG_PRINTLN(")");

    //====================================================
    // [改善] 状態に関係なく「強制停止」する
    // ・pumpStateが何になってても必ず停止コマンド送る
    // ・ランプも必ず消す
    //====================================================
    stopPump();
  }
}

// ポンプの状態更新
void updateSystemState() {
  rpm_value = getTargetRpm();

  if (pumpState == STATE_RUNNING) {
    digitalWrite(P_LAMP_PIN, HIGH);
    // digitalWrite(LED_PUMP_RUN_PIN, HIGH);
    // digitalWrite(LED_PUMP_STOP_PIN, LOW);

    unsigned long elapsedTimeSec = (millis() - pumpStartTime) / 1000UL;

    // ★追加★ 起動後120秒以内にしきい値に達しなかった場合の「低電流エラー」 2025-12-09
    if (!pumpStartupOk && !pumpStartupError &&
        elapsedTimeSec >= PUMP_STARTUP_TIMEOUT_SEC) {

      if (maxCurrentSinceStart < PUMP_CURRENT_THRESHOLD) {
        pumpStartupError = true;
        PU_DEBUG_PRINT("Pump startup failed: maxCurrentSinceStart=");
        PU_DEBUG_PRINT(maxCurrentSinceStart);
        PU_DEBUG_PRINT(" < threshold=");
        PU_DEBUG_PRINTLN(PUMP_CURRENT_THRESHOLD);

        stopPump();
        digitalWrite(EM_LAMP_PIN, HIGH); // 非常停止ランプ点灯
      } else {
        // 念のため OK が立っていなければここで立てる
        pumpStartupOk = true;
      }
    }

    // ★既存仕様★ 過電流保護（ロジック自体はそのまま維持）
    if (elapsedTimeSec > PUMP_TIMEOUT_SEC &&
        lastCurrentPeak > PUMP_CURRENT_THRESHOLD) {
      PU_DEBUG_PRINTLN("Pump stopped automatically due to over current.");
      stopPump();
      digitalWrite(EM_LAMP_PIN, HIGH); // 過電流でも非常停止ランプ点灯
    }

  } else {
    digitalWrite(P_LAMP_PIN, LOW);
    // digitalWrite(LED_PUMP_RUN_PIN, LOW);
    // digitalWrite(LED_PUMP_STOP_PIN, HIGH);
  }
}


// 3桁表示のため、1000以上は999として表示
void updateDisplays() {
  // ▼▼▼ 【変更点】tm1に回転数(rpm_value)の代わりに、しきい値(PUMP_CURRENT_THRESHOLD)を表示 ▼▼▼
  tm2.displayNum(PUMP_CURRENT_THRESHOLD);

  tm1.displayNum(rpm_value);
  if (pumpState == STATE_RUNNING) {
    tm3.displayNum(lastCurrentPeak);
    // tm3.displayNum((millis() - pumpStartTime) / 1000);
  } else {
    // tm2.displayNum(0);
    // tm3.displayNum(0);
  }
}

// シリアルからのコマンド受信可否によるLED点灯消灯
#if 0
void handleSerialCommunication() {
  static uint8_t buf[32];
  static uint8_t idx = 0;
  static uint8_t expected = 0;

  // デバッグ用: handleSerialCommunication() 内の最初に追加
  if (PUMP_SERIAL.available()) {
    PU_DEBUG_PRINT("RX RAW: ");
    while (PUMP_SERIAL.available()) {
      int b = PUMP_SERIAL.read();
      if (b < 0x10) PU_DEBUG_PRINT("0");
      PU_DEBUG_PRINT(b, HEX);
      PU_DEBUG_PRINT(" ");
      delay(1);
    }
    PU_DEBUG_PRINTLN("");
  }

  
  while (PUMP_SERIAL.available() > 0) {
    uint8_t b = (uint8_t)PUMP_SERIAL.read();

    // 受信インジケータ
    digitalWrite(LED_SERIAL_RX_PIN, HIGH);

    // フレーム同期：先頭は基本 01 00（スレーブ→マスター）:contentReference[oaicite:16]{index=16}
    if (idx == 0) {
      if (b != 0x01) continue;
    } else if (idx == 1) {
      if (b != 0x00) { idx = 0; continue; }
    }

    buf[idx++] = b;

    // D3まで来たら、期待長を決める
    if (idx == 4) {
      uint8_t d3 = buf[3];          // Data length-1
      expected = (uint8_t)(d3 + 6); // total bytes
      if (expected > sizeof(buf)) { // 異常防御
        idx = 0; expected = 0;
        continue;
      }
    }

    // フレーム完成
    if (expected > 0 && idx >= expected) {
      // チェックサム：総和が0x55 になるか確認（仕様書）:contentReference[oaicite:17]{index=17}
      uint8_t sum = 0;
      for (uint8_t i = 0; i < expected; i++) sum += buf[i];
      bool checksum_ok = (sum == 0x55);

      uint8_t cmd = buf[2];

      if (!checksum_ok) {
        PU_DEBUG_PRINTLN("RX checksum NG -> drop frame");
      } else {
        // --- 確認応答(0x00) ---
        if (cmd == 0x00 && expected == 9) {
          // 固定コード一致なら confirmed
          if (buf[4] == 0x01 && buf[5] == 0x12 && buf[6] == 0x00 && buf[7] == 0x06) {
            inverter_confirmed = true;
            PU_DEBUG_PRINTLN("Inverter CONFIRMED (fixed code OK)");
          } else {
            inverter_confirmed = false;
            PU_DEBUG_PRINTLN("Inverter CONFIRM mismatch");
          }
        }

        // --- 運転応答(0x10) ---
        if (cmd == 0x10) {
          // あなたの既存処理（errorCode=buf[7]等）に合わせて必要ならここで展開
          // ※「どのバイトがエラーコードか」は仕様書の応答レイアウトに準拠して整理してください
          // for debug:無事動いたら消す
          // 期待：R3(Data length-1)=0x07 → 全長13バイト :contentReference[oaicite:13]{index=13}

          // バッファ buf[] が 13 バイト揃っている前提
          const uint8_t target = buf[4]; // R4 目標周波数
          const uint8_t actual = buf[5]; // R5 実運転周波数
          const uint8_t err    = buf[7]; // R7 エラーコード :contentReference[oaicite:14]{index=14}
          const uint8_t temp   = buf[8]; // R8 IPM温度 :contentReference[oaicite:15]{index=15}

          PU_DEBUG_PRINT("[ACK 0x10] target=0x");
          if (target < 0x10) PU_DEBUG_PRINT("0");
          PU_DEBUG_PRINT(target, HEX);

          PU_DEBUG_PRINT(" actual=0x");
          if (actual < 0x10) PU_DEBUG_PRINT("0");
          PU_DEBUG_PRINT(actual, HEX);

          PU_DEBUG_PRINT(" err=0x");
          if (err < 0x10) PU_DEBUG_PRINT("0");
          PU_DEBUG_PRINT(err, HEX);

          PU_DEBUG_PRINT(" (");
          PU_DEBUG_PRINT(motor_err_str(err));
          PU_DEBUG_PRINT(")");

          PU_DEBUG_PRINT(" temp=");
          PU_DEBUG_PRINT(temp);
          PU_DEBUG_PRINTLN("C");
        }
      }

      // 次フレームへ
      idx = 0;
      expected = 0;

      digitalWrite(LED_SERIAL_RX_PIN, LOW);
    }
  }

  digitalWrite(LED_SERIAL_RX_PIN, LOW);
  #if 0
  // インバーターからの応答を受信・解析する
  while (PUMP_SERIAL.available() > 0) {
    digitalWrite(LED_SERIAL_RX_PIN, HIGH); // 受信中にLED点灯

    byte incomingByte = PUMP_SERIAL.read();

    // バッファにデータを格納
    if (responseByteCount < INVERTER_RESPONSE_SIZE) {
      inverterResponseBuffer[responseByteCount] = incomingByte;
      responseByteCount++;
    }

    // 規定のバイト数を受信したら、パケットを処理
    if (responseByteCount >= INVERTER_RESPONSE_SIZE) {
      // パケットのヘッダを確認 (宛先:自分, 送信元:ｲﾝﾊﾞｰﾀ, ｺﾏﾝﾄﾞ:運転)
      if (inverterResponseBuffer[0] == 0x01 && inverterResponseBuffer[1] == 0x00 && inverterResponseBuffer[2] == 0x10) {
        
        // エラーコードを抽出（仕様書より8バイト目、配列インデックスは7）
        byte errorCode = inverterResponseBuffer[7];

        // エラーコードが0x00以外で、かつ現在ポンプが運転中の場合
        if (errorCode != 0x00 && pumpState == STATE_RUNNING) {
          // 要件①：コマンドを送らずに、プログラムの状態を停止に移行する
          pumpState = STATE_STOPPED;
          digitalWrite(EM_LAMP_PIN, HIGH); // 非常停止ランプ点灯 2025年9月17日
          PU_DEBUG_PRINT("Inverter Error Detected! Code: 0x");
          if (errorCode < 0x10) PU_DEBUG_PRINT("0"); // 見やすいようにゼロ埋め
          PU_DEBUG_PRINTLN(errorCode, HEX);
          PU_DEBUG_PRINTLN("Transitioning to STOPPED mode without sending command.");
        }
      }
      
      responseByteCount = 0; // 次のパケットのためにカウンタをリセット
    }
  }
  digitalWrite(LED_SERIAL_RX_PIN, LOW); // 受信処理後にLED消灯
#endif

// 受信生ダンプ（切り分け用）
#if 1
  while (PUMP_SERIAL.available() > 0) {
    digitalWrite(LED_SERIAL_RX_PIN, HIGH);
    byte incomingByte = PUMP_SERIAL.read();
    if (responseByteCount < INVERTER_RESPONSE_SIZE) {
      inverterResponseBuffer[responseByteCount++] = incomingByte;
    }
    if (responseByteCount >= INVERTER_RESPONSE_SIZE) {
      if (inverterResponseBuffer[0] == 0x01 && inverterResponseBuffer[1] == 0x00 && inverterResponseBuffer[2] == 0x10) {
        byte errorCode = inverterResponseBuffer[7];
        if (errorCode != 0x00 && pumpState == STATE_RUNNING) {
          pumpState = STATE_STOPPED;
          digitalWrite(EM_LAMP_PIN, HIGH);
          PU_DEBUG_PRINT("Inverter Error Detected! Code: 0x");
          if (errorCode < 0x10) PU_DEBUG_PRINT("0");
          PU_DEBUG_PRINTLN(errorCode, HEX);
        }
      }
      responseByteCount = 0;
    }
  }
  digitalWrite(LED_SERIAL_RX_PIN, LOW);
#endif
}
#endif
//====================================================
// インバーター応答の受信（可変長フレーム）
// - 「総和が0x55」になるチェックサム方式
// - d3(Data length-1) から全長を決めてフレームを復元
// - ★重要★ 受信処理はここ1本に統一する（RAW dump等は消す）
//====================================================
//====================================================
// インバーター応答の受信（可変長フレーム）
// - 「総和が0x55」になるチェックサム方式
// - d3(Data length-1) から全長を決めてフレームを復元
// - ★重要★ 受信処理はここ1本に統一する（RAW dump等は消す）
//====================================================
void handleSerialCommunication() {
  static uint8_t buf[32];
  static uint8_t idx = 0;
  static uint8_t expected = 0;

  while (PUMP_SERIAL.available() > 0) {
    digitalWrite(LED_SERIAL_RX_PIN, HIGH);

    uint8_t b = (uint8_t)PUMP_SERIAL.read();

    // 受信バッファへ格納（溢れ防止）
    if (idx < sizeof(buf)) {
      buf[idx++] = b;
    } else {
      // ありえない長さになったら捨ててやり直し
      idx = 0;
      expected = 0;
      digitalWrite(LED_SERIAL_RX_PIN, LOW);
      continue;
    }

    // ヘッダが揃って、長さが確定できる段階になったら expected を計算
    // フォーマット: [addr2][cmd1][len1][data...][checksum1]
    // len は "Data length - 1" と仕様にあるので total = len + 6 を想定
    if (idx == 4) {
      uint8_t d3 = buf[3];              // Data length-1
      expected = (uint8_t)(d3 + 6);     // total bytes
      if (expected > sizeof(buf)) {     // 異常防御
        idx = 0;
        expected = 0;
        digitalWrite(LED_SERIAL_RX_PIN, LOW);
        continue;
      }
    }

    // フレーム完成
    if (expected > 0 && idx >= expected) {
      // チェックサム：総和が0x55になるか確認
      uint8_t sum = 0;
      for (uint8_t i = 0; i < expected; i++) sum += buf[i];
      bool checksum_ok = (sum == 0x55);

      // 見える化：受信フレームを必ず1回表示（切り分け用）
      PU_DEBUG_PRINT("RX FRAME: ");
      for (uint8_t i = 0; i < expected; i++) {
        if (buf[i] < 0x10) PU_DEBUG_PRINT("0");
        PU_DEBUG_PRINT(buf[i], HEX);
        PU_DEBUG_PRINT(" ");
      }
      PU_DEBUG_PRINTLN("");

      if (!checksum_ok) {
        PU_DEBUG_PRINTLN("RX checksum NG -> drop frame");
      } else {
        uint8_t cmd = buf[2];

        // 0x00: CONFIRM 応答（あなたの固定コードに合わせる）
        if (cmd == 0x00 && expected == 9) {
          if (buf[4] == 0x01 && buf[5] == 0x12 && buf[6] == 0x00 && buf[7] == 0x06) {
            inverter_confirmed = true;
            PU_DEBUG_PRINTLN("Inverter CONFIRMED (fixed code OK)");
          } else {
            inverter_confirmed = false;
            PU_DEBUG_PRINTLN("Inverter CONFIRM mismatch");
          }
        }

        // 0x10: 運転系 応答（とりあえずエラーコードらしき位置を表示）
        if (cmd == 0x10) {
          // ※ ここは仕様書の「応答レイアウト」に合わせて確定させる
          // いまは“見える化”としてbuf[7]を出す（既存コードがそう想定しているため）
          uint8_t errorCode = buf[7];
          PU_DEBUG_PRINT("Inverter RUN RESP errorCode?=0x");
          if (errorCode < 0x10) PU_DEBUG_PRINT("0");
          PU_DEBUG_PRINTLN(errorCode, HEX);
        }
      }

      // 次フレームへ
      idx = 0;
      expected = 0;
      digitalWrite(LED_SERIAL_RX_PIN, LOW);
    }
  }

  digitalWrite(LED_SERIAL_RX_PIN, LOW);
}

// タイマー処理でトリガーされる定期処理（コマンド送信、ピーク電流測定）
void handlePeriodicTasks() {
  if (!processFlag) return;
  processFlag = false;

  measurePeakCurrent();

  static int commandTimerCount = 0;
  commandTimerCount++;
  if (commandTimerCount >= (COMMAND_INTERVAL_MS / TIMER_INTERVAL_MS)) {
    commandTimerCount = 0;
    if (pumpState == STATE_RUNNING) {
//       double analog_value_f = (rpm_value + 5.092) / 17.945;
//       if (analog_value_f > 139.6) analog_value_f = 139.6;
//       if (analog_value_f < 34.0) analog_value_f = 34.0;
//       byte analog_value = (byte)analog_value_f;

//       // byte command[8] = {0x00, 0x01, 0x10, 0x02, analog_value, 0x01, 0x00, 0x00};
//       // ★重要★ 継続運転なら D5=0xFF を推奨（運転継続側）
//       // 仕様の細部はPDFの「周波数変更時間」や継続運転の注記に従う:contentReference[oaicite:14]{index=14}
//       byte command[8] = {0x00, 0x01, 0x10, 0x02, analog_value, 0xFF, 0x00, 0x00};

//       byte sum = 0;
//       for(int i=0; i < 7; i++) { sum += command[i]; }
//       command[7] = 0x55 - sum;
//       pump_write8(command, "RPM"); // インバーターへ回転数コマンド送信
// #if DEBUG_RPM_EACH_SEND
//       PU_DEBUG_PRINT("Sent: Set RPM Command to Inverter - RPM=");
//       PU_DEBUG_PRINTLN(rpm_value);
// #endif    
      sendRpmCommand(rpm_value);
    }else{
      // // ポンプ停止中は回転数0のコマンドを送信する
      // byte stop_command[8] = {0x00, 0x01, 0x10, 0x02, 0x00, 0x01, 0x00, 0x00};
      // byte sum = 0;
      // for(int i=0; i < 7; i++) {
      //   sum += stop_command[i];
      // }
      // stop_command[7] = 0x55 - sum; // チェックサムを計算
      // PUMP_SERIAL.write(stop_command, 8);
    }
  }
}

// --- 設定項目 ---
// 移動平均で平均化するサンプル数。
// この値が大きいほどノイズに強くなりますが、実際の電流の変化に対する反応が少し緩やかになります。
// まずは10で試し、効きが弱い/強すぎる場合は5〜20の範囲で調整してみてください。
const int MOVING_AVG_SIZE = 10;
// ポンプのピーク電流を測定
void measurePeakCurrent() {
  // --- 移動平均フィルタ用の静的変数 ---
  static bool is_initialized = false;
  static int readings[MOVING_AVG_SIZE];
  static int readIndex = 0;
  static long total = 0;
  // ★追加★ LED点滅用カウンタ（タイマータスク呼び出し回数） 2025年12月10日
  static int ledBlinkCnt = 0;

  // --- ピーク検出用の静的変数 ---
  static int analog_cnt = 0;
  // ADCの有効範囲の下限値(512)をピークの初期値（兼、最低値）とする
  static int current_reading_max = 512;

  // --- 初回実行時にフィルタの変数を初期化する処理 ---
  if (!is_initialized) {
    for (int i = 0; i < MOVING_AVG_SIZE; i++) {
      readings[i] = 512; // 想定される最低値で配列を埋める
    }
    total = 512L * MOVING_AVG_SIZE; // long型で合計値を計算しておく
    is_initialized = true;
  }
  
  // ★ここで一定時間ごとにだけLEDをトグル★ 2025年12月10日
  // TIMER_INTERVAL_MSごとにこの関数が呼ばれる想定なので、
  // 「(秒数 * 1000) / TIMER_INTERVAL_MS」回呼ばれたら1回トグルする。
  ledBlinkCnt++;
  if (ledBlinkCnt >= (LED_ISR_BLINK_INTERVAL_SEC * 1000 / TIMER_INTERVAL_MS)) {
    ledBlinkCnt = 0;
    digitalWrite(LED_ISR_PIN, !digitalRead(LED_ISR_PIN));
  }
  // --- 移動平均フィルタの計算 ---
  // 1. 合計から一番古い測定値を引く
  total = total - readings[readIndex];
  
  // 2. ADCから新しい値を読み取る
  int new_reading = analogRead(CURRENT_ANALOG_IN_PIN);
  
  // 3. 新しい測定値を配列に格納（古い値は上書きされる）
  readings[readIndex] = new_reading;
  
  // 4. 合計に新しい測定値を足す
  total = total + readings[readIndex];
  
  // 5. 次に上書きする配列の場所を更新
  readIndex++;
  if (readIndex >= MOVING_AVG_SIZE) {
    readIndex = 0;
  }
  
  // 6. 平均値を計算して「平滑化された現在値」とする
  int smoothed_val = total / MOVING_AVG_SIZE;

  // --- ピーク検出（平滑化された値を使用）---
  // 平滑化された値が、今までの最大値より大きいかチェック
  if (smoothed_val > current_reading_max) {
    // 範囲内の最大値として更新
    if (smoothed_val <= 1023) {
      current_reading_max = smoothed_val;
    }
  }

  // --- 1.5秒ごとにピーク値を確定 ---
  analog_cnt++;
  if (analog_cnt >= (1500 / TIMER_INTERVAL_MS)) {
    // 確定したピーク値が、初期値(512)より大きい場合のみ有効なピークとする
    if (current_reading_max > 512) {
      lastCurrentPeak = current_reading_max;
      // PU_DEBUG_PRINT("Current Peak (Smoothed): ");
      // PU_DEBUG_PRINTLN(lastCurrentPeak);

      // ★追加★ ポンプ起動中の最大電流と「しきい値到達」を記録 2025-12-09
      if (pumpState == STATE_RUNNING) {
        if (lastCurrentPeak > maxCurrentSinceStart) {
          maxCurrentSinceStart = lastCurrentPeak;
        }
        if (!pumpStartupOk && lastCurrentPeak >= PUMP_CURRENT_THRESHOLD) {
          pumpStartupOk = true;
          PU_DEBUG_PRINTLN("Pump startup current reached threshold.");
        }
      }

    } else {
      lastCurrentPeak = 0; // または512など、無効を示す値
      PU_DEBUG_PRINTLN("No valid peak detected.");
    }
    
    // 次の測定期間のためにリセット
    current_reading_max = 512;
    analog_cnt = 0;
  }
#if 0 // 元の簡易版
  static int analog_cnt = 0;
  static int current_reading_max = 512;
  
  digitalWrite(LED_ISR_PIN, !digitalRead(LED_ISR_PIN));

  int current_val = analogRead(CURRENT_ANALOG_IN_PIN);
  if (current_val > current_reading_max) {
    current_reading_max = current_val;
  }

  analog_cnt++;
  if (analog_cnt >= (1500 / TIMER_INTERVAL_MS)) {
    lastCurrentPeak = current_reading_max;
    PU_DEBUG_PRINT("Current Peak: ");
    PU_DEBUG_PRINTLN(lastCurrentPeak);
    current_reading_max = 512;
    analog_cnt = 0;
  }
#endif
}

/**
 * @brief 実行時モードに応じて目標回転数を取得する（サインカーブ・プライミング機能付き）
 * @details 最高速度でのみ2秒間の保持時間を設けた修正版。
 */
int getTargetRpm() {
  // ポンプが運転中で、かつ起動後プライミング時間内の場合にシーケンスを実行
  if (pumpState == STATE_RUNNING) {
    unsigned long elapsedTimeMillis = millis() - pumpStartTime;
    if (elapsedTimeMillis < (PRIMING_DURATION_SEC * 1000UL)) {
      // 1. 定数を定義
      const float RAMP_CYCLE_SEC = PRIMING_CYCLE_SEC; // 回転数が上下する時間（4秒）
      // const float HOLD_DURATION_SEC = 2.0;          // 最高回転数での保持時間（秒）
      // 1サイクルの合計時間 = 回転の上下時間(4秒) + 最高保持(2秒)
      const float TOTAL_CYCLE_SEC = RAMP_CYCLE_SEC + HOLD_DURATION_SEC;

      // 2. 現在の経過時間が、1サイクル(6秒)の中でどの位置にあるかを計算
      unsigned long timeInCycleMillis = elapsedTimeMillis % (unsigned long)(TOTAL_CYCLE_SEC * 1000.0);

      // 3. 保持時間を考慮した「見かけ上の経過時間」を計算する
      unsigned long rampTimeMillis;
      // サインカーブが頂点に達する時間 (4秒サイクルの1/4 = 1秒)
      unsigned long maxRpmHoldStart = (unsigned long)((RAMP_CYCLE_SEC / 4.0) * 1000.0); // 1000ms
      // 最高回転数での保持が終了する時間
      unsigned long maxRpmHoldEnd   = maxRpmHoldStart + (unsigned long)(HOLD_DURATION_SEC * 1000.0); // 3000ms

      if (timeInCycleMillis < maxRpmHoldStart) {
        // 最高回転数に達するまで（サインカーブの0秒 -> 1秒地点）
        rampTimeMillis = timeInCycleMillis;
      } else if (timeInCycleMillis < maxRpmHoldEnd) {
        // 最高回転数で保持（サインカーブの1秒地点で時間を止める）
        rampTimeMillis = maxRpmHoldStart;
      } else {
        // 最低回転数に向かって下降し、再び上昇する（サインカーブの1秒 -> 4秒地点）
        // 止まっていた時間(2秒)を考慮して、サインカーブの時間を進める
        rampTimeMillis = maxRpmHoldStart + (timeInCycleMillis - maxRpmHoldEnd);
      }

      // 4. 「見かけ上の経過時間」を使って、元のサインカーブ計算を実行
      float angle = (rampTimeMillis / (RAMP_CYCLE_SEC * 1000.0)) * 2.0 * PI;
      float sinValue = sin(angle);

      // 5. -1.0〜1.0の値を、最小RPM〜最大RPMの範囲に変換(マッピング)
      float rpm_range = PRIMING_MAX_RPM - PRIMING_MIN_RPM;
      float rpm_midpoint = (PRIMING_MAX_RPM + PRIMING_MIN_RPM) / 2.0;
      int targetRpm = (int)(rpm_midpoint + (sinValue * rpm_range / 2.0));
      
      return targetRpm;
    }
  }

  // プライミング時間終了後、またはポンプ停止時は通常の回転数制御に戻る
  if (rpmControlMode == MODE_VOLUME) {
    return calculateRpmFromVolume(); // ボリュームから計算
  } else { // MODE_FIXED
    return NORMAL_MAX_RPM; // 設定された固定値を返す
  }
}

// ▼▼▼ ここから追加 ▼▼▼
/**
 * @brief 配列を小さい順に並べ替える（バブルソート）
 * @param arr 並べ替える配列
 * @param n 配列の要素数
 */
void simpleSort(int arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        // 隣の要素と比較して大きければ入れ替える
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}
// ▲▲▲ ここまで追加 ▲▲▲

// 可変抵抗から回転数を計算
int calculateRpmFromVolume() {
    const int sampleCount = 5;
    int readings[sampleCount];

    // 指定回数、値を読み取って配列に格納
    for (int i = 0; i < sampleCount; i++) {
        readings[i] = analogRead(RPM_ANALOG_IN_PIN);
        delay(2);
    }

    // 配列を小さい順に並べ替える
    simpleSort(readings, sampleCount);
    // 並べ替えた後の中央の値を取得
    int median = readings[sampleCount / 2];
    // 取得した中央値を使って計算
    double analog_value = median * 0.1032 + 34.0;
    
    // --- ▼▼▼ ここから修正箇所 ▼▼▼ ---
    // 先ほど設定した NORMAL_MAX_RPM から、コマンドで送る上限値を計算する
    double max_analog_value = (NORMAL_MAX_RPM + 5.092) / 17.945;

    // 計算した上限値で analog_value をクリッピング（頭打ち）する
    if (analog_value > max_analog_value) analog_value = max_analog_value;
    // --- ▲▲▲ ここまで修正箇所 ▲▲▲ ---

    if (analog_value < 34.0) analog_value = 34.0;
    return (int)(analog_value * 17.945 - 5.092);
}
void timerInterrupt() {
  // 一定間隔で行うインバーターへのコマンド送信やピーク電流測定のためのフラグをセット
  processFlag = true;
}

// 文字列の前後の空白を削除
void trim(char* str) {
  if (str == nullptr) return;
  char* start = str;
  while (isspace(*start)) {
    start++;
  }
  char* end = start + strlen(start) - 1;
  while (end > start && isspace(*end)) {
    end--;
  }
  *(end + 1) = '\0';
  if (start != str) {
    memmove(str, start, end - start + 2);
  }
}
// ★追加★★ 両方停止出力ピンの制御タスク
//====================================================
// [仕様] アワーメーターリセット
//  - UVあり(detectedLamps>0) : (P_STOP + UV_STOP) 同時押し も有効
//  - 全機種共通            : P_STOP の長押しでも実行（UVボタン無し機種対策）
//  - 1回の押下シーケンスで1回だけ発火（連打/二重発火防止）
//====================================================
void both_stop_check_task() {

    //---- 調整ポイント：長押し判定時間 ----
    const unsigned long LONGPRESS_MS = 2000;  // 例：2秒（好きに変えてOK）

    //---- チャタリング後の安定状態（押下=LOW 前提）----
    const bool pumpStopPressed = (pumpStopSwitch.stableState == LOW);

    // UVボタンは「物理的に無い機種」があるので、
    // UVあり判定（detectedLamps>0）のときだけ参照する
    const bool uvPresent = (detectedLamps > 0);
    const bool uvStopPressed = (uvPresent && (uvStopSwitch.stableState == LOW));

    //================================================
    // この関数内だけで状態保持（1箇所修正のためstaticで完結）
    //================================================
    static unsigned long pressStartMs = 0;  // ポンプ停止ボタン押下開始時刻
    static bool firedThisPress = false;     // 押しっぱなしでの多重発火防止

    //================================================
    // 1) UVありのときだけ：同時押し（即時）でリセット
    //================================================
    if (uvPresent && pumpStopPressed && uvStopPressed) {

        // 同時押しは「即時」で1回だけ
        if (!firedThisPress) {
            firedThisPress = true;                // 先にラッチ（超重要：二重発火防止）
            hourMeterResetComboLatched = true;    // 既存ラッチも立てておく（互換維持）

            DEBUG_PRINTLN("Hour meter reset by P_SW_STOP + UV_SW_STOP combo.");
            resetUvHourMeter();
        }

        // 同時押し中は長押しタイマは使わない（誤発火防止）
        pressStartMs = 0;
        return;
    }

    //================================================
    // 2) 全機種共通：ポンプ停止ボタン長押しでリセット
    //================================================
    if (pumpStopPressed) {

        // 押し始めを検出してタイマ開始
        if (pressStartMs == 0) {
            pressStartMs = millis();
        }

        // 規定時間を超えたら「1回だけ」リセット
        if (!firedThisPress && (millis() - pressStartMs >= LONGPRESS_MS)) {
            firedThisPress = true;                // 先にラッチ
            hourMeterResetComboLatched = true;    // 既存ラッチも立てておく（互換維持）

            DEBUG_PRINTLN("Hour meter reset by long press of P_SW_STOP.");
            resetUvHourMeter();
        }

    } else {
        //================================================
        // 3) ボタンを離したら全ラッチ解除（次の操作に備える）
        //================================================
        pressStartMs = 0;
        firedThisPress = false;
        hourMeterResetComboLatched = false;
    }
}

// -------------------------------------------------------------
// アワーメーターリセット
// HOURMETER_RESET_PIN を一定時間アクティブにしてリセットをかける
// -------------------------------------------------------------
void resetUvHourMeter() {
  DEBUG_PRINTLN("resetUvHourMeter: pulse LOW on HOURMETER_RESET_PIN");

  digitalWrite(HOURMETER_RESET_PIN, HIGH);
  delay(500);  // リセットパルス幅（仕様に合わせて調整：100〜500msくらいでOKなことが多い）
  digitalWrite(HOURMETER_RESET_PIN, LOW);
}
//====================================================
// [追加] 8バイト固定の送信を統一（デバッグログもここに集約）
//====================================================
void pump_write8(const uint8_t cmd[8], const char* label) {
  // 送信
  PUMP_SERIAL.write(cmd, 8);

  // デバッグ（USB側に出す）
  PU_DEBUG_PRINT("[PUMP] ");
  PU_DEBUG_PRINT(label);
  PU_DEBUG_PRINT(" : ");
  for (int i = 0; i < 8; i++) {
    if (cmd[i] < 0x10) PU_DEBUG_PRINT('0');
    PU_DEBUG_PRINT(cmd[i], HEX);
    PU_DEBUG_PRINT(' ' );
  }
  PU_DEBUG_PRINTLN();

  //====================================================
  // [追加] RPMを送る＝インバータ動作中なのでファンON
  //====================================================
  // if (label && strcmp(label, "RPM") == 0) {
  //   fan_on();   // ※ファンがLOWアクティブならLOWにする
  //   PU_DEBUG_PRINTLN("pump_write8: FAN_CTRL_PIN set HIGH (RPM command sent)");
  // } else {
  //   // それ以外のコマンドはファンOFF（停止コマンド等）
  //   fan_off();    // ※ファンがLOWアクティブならLOWにする
  //   PU_DEBUG_PRINTLN("pump_write8: FAN_CTRL_PIN set LOW (non-RPM command sent)");
  // }
}

// for debug:無事動いたら削除してよい
//====================================================
// 0x10 応答のエラーコード文字列化（仕様書の項目）
//====================================================
const char* motor_err_str(uint8_t code) {
  switch (code) {
    case 0x00: return "正常";
    // 以降の並びは仕様書の「項目」列に対応 :contentReference[oaicite:11]{index=11}
    case 0x01: return "インバータ直流過電圧";
    case 0x02: return "インバータ直流低電圧";
    case 0x03: return "インバータ交流過電流";
    case 0x04: return "速度推定下限エラー(脱調)";
    case 0x05: return "欠相検出1(速度推定脈動)";
    case 0x06: return "欠相検出2(電流アンバランス)";
    case 0x07: return "IPMエラー1(エッジ検出)";
    case 0x08: return "IPMエラー2(レベル検出)";
    case 0x09: return "電流センサ異常";
    case 0x0B: return "インバータPWM端子異常(端子電圧不安定等)";
    case 0x0C: return "通信エラー(6秒以上コマンド途絶で停止)";
    case 0x0D: return "COM/Dutyモード設定異常";
    case 0x0E: return "IPM温度異常";
    default:   return "不明(仕様外)";
  }
}
void updateInputFeedbackLed() { // ★追加★ 入力フィードバックLED制御タスク
  bool isPressed = false;

  if (digitalRead(P_SW_START_PIN) == LOW ||
      digitalRead(P_SW_STOP_PIN) == LOW ||
      digitalRead(UV_SW_START_PIN) == LOW ||
      digitalRead(UV_SW_STOP_PIN) == LOW) {
    isPressed = true;
  }

  digitalWrite(INPUT_FEEDBACK_LED_PIN, isPressed ? HIGH : LOW);
}
