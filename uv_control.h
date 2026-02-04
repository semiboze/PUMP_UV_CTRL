// uv_control.h

#pragma once // このファイルが複数回読み込まれるのを防ぐおまじない

// USE_UV_LAMP_CONTROLが定義されている場合のみ、このファイルの中身を有効にする
// #ifdef USE_UV_LAMP_CONTROL

#include <Arduino.h> // StringやHIGH/LOWなどArduinoの基本機能を使うために必要

// メインファイルから呼び出す関数の宣言
void uv_setup(int detected_lamp_count);      // ← int 引数付きに修正
void uv_loop_task();
bool is_uv_running(); // ★★★ UVランプが稼働中か判定する関数を追加 ★★★
void runStartupLedSequence(int lampCount); // ★★★ LEDの起動シーケンス関数を追加 ★★★
void uv_force_restore(bool run);
// #endif