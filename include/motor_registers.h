/**
 * @file motor_registers.h
 * @brief Oriental Motor AZM46MK-TS3.6 Modbus Register Definitions
 *
 * Modbus TCP Register addresses for Oriental Motor AZM series
 * Network servo driver
 */

#ifndef MOTOR_REGISTERS_H
#define MOTOR_REGISTERS_H

#include <IPAddress.h>

// ===== MOTOR NETWORK CONFIG =====
#define MOTOR_IP_ADDRESS IPAddress(192, 168, 100, 10)   // Oriental Motor IP as string
#define MOTOR_PORT 502              // Modbus TCP port
#define MOTOR_SLAVE_ID 1           // Modbus slave ID

// Oriental Motor AZシリーズ レジスタアドレス (az_define.pyより)
// モニタ系 (Monitor)
#define ADDR_ALARM_MON      0x011F  // アラームモニタ
#define ADDR_STATIC_IO_IN   0x0106  // 静的入力
#define ADDR_STATIC_IO_OUT  0x011E  // 静的出力
#define ADDR_POS            0x0120  // 現在位置
#define ADDR_VEL            0x0122  // 現在速度
#define ADDR_TARGET_POS     0x0124  // 目標位置

// ダイレクトデータ運転 (Direct Data Operation)
#define ADDR_TRIG_MODE      0x0106  // トリガーモード

// パラメータ読み書き用レジスタ
#define ADDR_READ_PARAM_ID_R    0x012B
#define ADDR_READ_WRITE_STATUS  0x012C
#define ADDR_WRITE_PARAM_ID_R   0x012D
#define ADDR_READ_DATA_LOW      0x012E
#define ADDR_READ_DATA_HIGH     0x012F
#define ADDR_READ_PARAM_ID      0x0113
#define ADDR_WRITE_REQ          0x0114
#define ADDR_WRITE_PARAM_ID     0x0115

// ソフトリミット
#define ADDR_SOFT_LIMIT_MIN     0x01C5
#define ADDR_SOFT_LIMIT_MAX     0x01C4


// 運転モード (OpMode)
enum OpMode {
  ABS_POS = 1,            // 絶対位置決め
  REL_POS_CMD = 2,        // 相対位置決め（指令基準）
  REL_POS_DET = 3,        // 相対位置決め（検出基準）
  CONT_SPEED_CTRL = 16,   // 連続速度制御
  CONT_PUSH = 17,         // 連続押し当て
  CONT_TORQUE = 18        // 連続トルク
};

// 静的IO出力ビット割り当て (StaticIOOutBitAssign)
enum StaticIOOutBit {
  READY = 5,      // 運転準備完了
  MOVE = 1,       // モーター動作中
  IN_POS = 2,     // 位置決め完了
  DCMD_RDY = 6,   // ダイレクトデータ運転準備完了
  ALM_A = 7       // アラーム状態
};

// 静的IO入力ビット割り当て (StaticIOInBitAssign)
enum StaticIOInBit {
  START = 3,      // ストアードデータ運転実行
  STOP = 5,       // モーター停止
  FREE = 6,       // 無励磁
  ALM_RST = 7,    // アラームリセット
  TRIG = 8        // ダイレクトデータ運転実行
};

// メンテナンスコマンド
#define ALARM_RESET         0x00C0  // アラームリセット
#define P_PRESET_EXECUTE    0x00C5  // P-PRESET実行

// =============== ORIENTAL MOTOR CONFIG ===============


#endif // MOTOR_REGISTERS_H