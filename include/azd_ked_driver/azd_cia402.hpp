// azd_cia402.hpp
// AZシリーズ EtherCAT (CiA-402) 用ヘッダ：最小PP制御に必要なオブジェクト・定数を網羅
// - RxPDO: 6040/16 + 607A/32
// - TxPDO: 6041/16 + 6064/32
// - SM2/SM3 の PDO 割付 (1C12/1C13) と PDO マップ (1600/1A00) も定義
// - 既存コードの「数値リテラル」をそのまま名前置換できる実用定数を用意

#pragma once
#include <cstdint>

namespace azd {
namespace cia402 {

// ===== オブジェクト辞書 Index/SubIndex =====
namespace idx {
  // CiA-402 基本
  constexpr std::uint16_t CONTROLWORD         = 0x6040; // U16
  constexpr std::uint16_t STATUSWORD          = 0x6041; // U16
  constexpr std::uint16_t MODES_OF_OPERATION  = 0x6060; // S8
  constexpr std::uint16_t MODES_OF_DISPLAY    = 0x6061; // S8 (RO)
  constexpr std::uint16_t POSITION_ACTUAL     = 0x6064; // S32 (RO)
  constexpr std::uint16_t TARGET_POSITION     = 0x607A; // S32 (RW)
  constexpr std::uint16_t PROFILE_VELOCITY    = 0x6081; // U32
  constexpr std::uint16_t PROFILE_ACCEL       = 0x6083; // U32
  constexpr std::uint16_t PROFILE_DECEL       = 0x6084; // U32

  // PDO マップ（本コードは 1600/1A00 のみ使用）
  constexpr std::uint16_t RX_PDO_MAP1         = 0x1600; // sub 00..: PDOエントリ数＋マップ
  constexpr std::uint16_t TX_PDO_MAP1         = 0x1A00;

  // Sync Manager PDO 割付（※ここを欲していた箇所）
  constexpr std::uint16_t SM2_PDO_ASSIGN      = 0x1C12; // RxPDO（Outputs）割付
  constexpr std::uint16_t SM3_PDO_ASSIGN      = 0x1C13; // TxPDO（Inputs）割付
} // namespace idx


// ===== Operation Mode (6060h) 値 =====
enum class OpMode : std::int8_t {
  Disabled = 0,
  PP       = 1,  // Profile Position（本コードで使用）
  PV       = 3,
  HM       = 6,
  CSP      = 8,
  CSV      = 9,
};


// ===== Controlword（6040h）便利マスク =====
namespace cw {
  constexpr std::uint16_t SWITCH_ON        = 1u << 0;
  constexpr std::uint16_t ENABLE_VOLTAGE   = 1u << 1;
  constexpr std::uint16_t QUICK_STOP       = 1u << 2;
  constexpr std::uint16_t ENABLE_OPERATION = 1u << 3;
  constexpr std::uint16_t NEW_SETPOINT     = 1u << 4; // PP
  constexpr std::uint16_t CHANGE_IMMED     = 1u << 5; // PP

  // 状態遷移の定型（元コードの数値と等価）
  constexpr std::uint16_t SHUTDOWN       = (ENABLE_VOLTAGE | QUICK_STOP);                  // 0x0006
  constexpr std::uint16_t SWITCH_ON_ONLY = (SHUTDOWN | SWITCH_ON);                         // 0x0007
  constexpr std::uint16_t ENABLE_OP      = (SWITCH_ON_ONLY | ENABLE_OPERATION);            // 0x000F
} // namespace cw


// ===== Statusword（6041h）主要ビット（元コード参照用） =====
namespace sw {
  constexpr std::uint16_t READY_TO_SWITCH_ON = 1u << 0;
  constexpr std::uint16_t SWITCHED_ON        = 1u << 1;
  constexpr std::uint16_t OPERATION_ENABLED  = 1u << 2;
  constexpr std::uint16_t FAULT              = 1u << 3;
  constexpr std::uint16_t QUICK_STOP_ACTIVE  = 1u << 5;
  constexpr std::uint16_t SWITCH_ON_DISABLED = 1u << 6;
  constexpr std::uint16_t WARNING            = 1u << 7;
  constexpr std::uint16_t REMOTE             = 1u << 9;
  constexpr std::uint16_t TARGET_REACHED     = 1u << 10; // ※装置依存
  constexpr std::uint16_t SETPOINT_ACK       = 1u << 12;
} // namespace sw


// ===== PDO 構造（6B：Rx 6040/16 + 607A/32，Tx 6041/16 + 6064/32） =====
#pragma pack(push, 1)
struct RxPdoPP6B {
  std::uint16_t controlword;     // 6040h:16
  std::int32_t  target_position; // 607Ah:32
};
static_assert(sizeof(RxPdoPP6B) == 6, "RxPdoPP6B must be 6 bytes");

struct TxPdoPP6B {
  std::uint16_t statusword;      // 6041h:16
  std::int32_t  position_actual; // 6064h:32
};
static_assert(sizeof(TxPdoPP6B) == 6, "TxPdoPP6B must be 6 bytes");
#pragma pack(pop)


// ===== PDO マッピング値ユーティリティ =====
constexpr std::uint32_t mapU32(std::uint16_t index, std::uint8_t sub, std::uint8_t bitlen) {
  return (static_cast<std::uint32_t>(index) << 16) |
         (static_cast<std::uint32_t>(sub)   << 8 ) |
          static_cast<std::uint32_t>(bitlen);
}

// --- 元コードの「即値」をそのまま名前置換できる定数（可読性＆保守性向上） ---
namespace mapval {
  // RxPDO1: 6040/16, 607A/32
  constexpr std::uint32_t RX_6040_16 = 0x60400010; // mapU32(0x6040,0x00,16)
  constexpr std::uint32_t RX_607A_32 = 0x607A0020; // mapU32(0x607A,0x00,32)

  // TxPDO1: 6041/16, 6064/32
  constexpr std::uint32_t TX_6041_16 = 0x60410010; // mapU32(0x6041,0x00,16)
  constexpr std::uint32_t TX_6064_32 = 0x60640020; // mapU32(0x6064,0x00,32)
}

// デフォルトの「6Bマップ」定義（必要なら利用）
struct DefaultPdoMap6B {
  // RxPDO1 (1600h)
  static constexpr std::uint8_t  rx_entries = 2;
  static constexpr std::uint32_t rx_1 = mapval::RX_6040_16;
  static constexpr std::uint32_t rx_2 = mapval::RX_607A_32;

  // TxPDO1 (1A00h)
  static constexpr std::uint8_t  tx_entries = 2;
  static constexpr std::uint32_t tx_1 = mapval::TX_6041_16;
  static constexpr std::uint32_t tx_2 = mapval::TX_6064_32;
};

} // namespace cia402
} // namespace azd
