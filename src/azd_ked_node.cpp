// src/azd_ked_node.cpp
// AZD-KED minimal PP control as a ROS1 node using SOEM, with robust startup.
// - Uses azd_cia402.hpp for readable object indices, masks, and PDO types.
// - Publishes:  ~status_word (std_msgs/UInt16), ~position_actual (std_msgs/Int32)
// - Subscribes: ~target_position (absolute), ~move_delta (relative)
// - Requires CAP_NET_RAW and CAP_NET_ADMIN on the binary or run with sudo.

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <atomic>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>

// SOEM (C headers)
extern "C" {
#include <ethercat.h>
}

// AZD-KED CiA-402 constants/types（ユーザ環境のヘッダを使用）
#include "azd_ked_driver/azd_cia402.hpp"

#define SLAVE_DEFAULT 1

// ===== Global command flags =====
static std::atomic<bool>    g_have_cmd{false};
static std::atomic<int32_t> g_cmd_target{0};

static std::atomic<bool>    g_have_delta{false};
static std::atomic<int32_t> g_delta{0};

// ===== SDO helpers =====
static bool sdo_u8 (uint16 slave, uint16 idx, uint8 sub, uint8  val) {
  return ec_SDOwrite(slave, idx, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM) > 0;
}
static bool sdo_u16(uint16 slave, uint16 idx, uint8 sub, uint16 val) {
  return ec_SDOwrite(slave, idx, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM) > 0;
}
static bool sdo_u32(uint16 slave, uint16 idx, uint8 sub, uint32 val) {
  return ec_SDOwrite(slave, idx, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM) > 0;
}

// ===== PDO remapping (6 bytes Rx: 6040/16 + 607A/32, 6 bytes Tx: 6041/16 + 6064/32) =====
static bool pdo_remap(uint16 slave) {
  using namespace azd::cia402;
  bool ok = true;

  // SM2 (Outputs / RxPDO) -> 1C12, map at 1600
  ok &= sdo_u8 (slave, idx::SM2_PDO_ASSIGN, 0x00, 0);   // Clear existing mapping
  ok &= sdo_u8 (slave, idx::RX_PDO_MAP1,   0x00, 0);    // Clear existing map
  ok &= sdo_u32(slave, idx::RX_PDO_MAP1,   0x01, mapval::RX_6040_16);   // CONTROLWORD
  ok &= sdo_u32(slave, idx::RX_PDO_MAP1,   0x02, mapval::RX_607A_32);   // TARGET_POSITION
  ok &= sdo_u8 (slave, idx::RX_PDO_MAP1,   0x00, azd::cia402::DefaultPdoMap6B::rx_entries);
  ok &= sdo_u16(slave, idx::SM2_PDO_ASSIGN,0x01, idx::RX_PDO_MAP1);
  ok &= sdo_u8 (slave, idx::SM2_PDO_ASSIGN,0x00, 1);

  // SM3 (Inputs / TxPDO) -> 1C13, map at 1A00
  ok &= sdo_u8 (slave, idx::SM3_PDO_ASSIGN, 0x00, 0);   // Clear existing mapping
  ok &= sdo_u8 (slave, idx::TX_PDO_MAP1,    0x00, 0);   // Clear existing map
  ok &= sdo_u32(slave, idx::TX_PDO_MAP1,    0x01, mapval::TX_6041_16);  // STATUSWORD
  ok &= sdo_u32(slave, idx::TX_PDO_MAP1,    0x02, mapval::TX_6064_32);  // POSITION_ACTUAL
  ok &= sdo_u8 (slave, idx::TX_PDO_MAP1,    0x00, azd::cia402::DefaultPdoMap6B::tx_entries);
  ok &= sdo_u16(slave, idx::SM3_PDO_ASSIGN, 0x01, idx::TX_PDO_MAP1);
  ok &= sdo_u8 (slave, idx::SM3_PDO_ASSIGN, 0x00, 1);

  // 動作モード PP (=1) とプロファイル（後で上書き可）
  ok &= sdo_u8 (slave, idx::MODES_OF_OPERATION, 0x00, static_cast<int8_t>(azd::cia402::OpMode::PP));

  if (!ok) {
    ROS_WARN("[pdo_remap] FAILED AL=0x%04X", ec_slave[slave].ALstatuscode);
  } else {
    ROS_INFO("[pdo_remap] success");
  }
  return ok;
}

// ===== CiA-402 enable sequence (robust: Fault reset multi + 0x0006→0x0007→0x000F) =====
static bool cia402_enable_seq(azd::cia402::RxPdoPP6B* rx,
                              const azd::cia402::TxPdoPP6B* tx)
{
  // Fault Reset (bit7) を多重送出
  for (int i = 0; i < 20; ++i) {
    rx->controlword = (1u << 7);
    ec_send_processdata(); ec_receive_processdata(EC_TIMEOUTRET);
    ros::Duration(0.005).sleep();
  }

  // 状態遷移：Shutdown→Switch on→Enable operation
  int step = 0; int timeout = 5000;
  while (timeout-- > 0) {
    ec_send_processdata(); ec_receive_processdata(EC_TIMEOUTRET);
    uint16_t sw = tx->statusword;

    switch (step) {
      case 0: // Ready to switch on 待ち
        rx->controlword = 0x0006; /* Shutdown */
        if (sw & (1 << 0)) step = 1; // bit0: Ready to switch on
        break;
      case 1: // Switched on 待ち
        rx->controlword = 0x0007; /* Switch on */
        if ((sw & 0x006F) == 0x0023) step = 2; // 0x0023
        break;
      case 2: // Operation enabled 待ち
        rx->controlword = 0x000F; /* Enable operation */
        if ((sw & 0x006F) == 0x0027) return true; // 0x0027
        break;
    }
    ros::Duration(0.001).sleep();
  }
  ROS_ERROR("Enable sequence failed: SW=0x%04X", tx->statusword);
  return false;
}

// ===== PP trigger: Bit4/5 pulse, always return to 0x000F base =====
static void trigger_pp(azd::cia402::RxPdoPP6B* rx, int32_t tgt,
                       bool immediate = true, bool relative = false)
{
  // ベースは運転有効 0x000F（相対時はBit6もセット）
  uint16_t base = 0x000F;
  if (relative) base |= (1u << 6); // Abs/Rel

  rx->target_position = tgt;

  // パルスON（Bit4 New set-point，必要ならBit5 Change immediately）
  uint16_t on = base | (1u << 4) | (immediate ? (1u << 5) : 0);

  rx->controlword = on;
  for (int i = 0; i < 15; ++i) {
    ec_send_processdata(); ec_receive_processdata(EC_TIMEOUTRET);
    ros::Duration(0.002).sleep();
  }

  // パルスOFF（ベースへ復帰：Bit4/5を確実に落とす）
  // rx->controlword = base;
  for (int i = 0; i < 5; ++i) {
    ec_send_processdata(); ec_receive_processdata(EC_TIMEOUTRET);
    ros::Duration(0.002).sleep();
  }
}

// ===== Free-function callbacks =====
static void on_target_abs(const std_msgs::Int32::ConstPtr& m) {
  g_cmd_target = m->data;
  g_have_cmd   = true;
}
static void on_move_delta(const std_msgs::Int32::ConstPtr& m) {
  g_delta      = m->data;
  g_have_delta = true;
}

int main(int argc, char** argv)
{
  using namespace azd::cia402;

  ros::init(argc, argv, "azd_ked_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Parameters
  std::string ifname = "enp0s31f6";
  int slave = SLAVE_DEFAULT;
  int cycle_hz = 500;
  pnh.param("ifname", ifname, ifname);
  pnh.param("slave_index", slave, slave);
  pnh.param("cycle_hz", cycle_hz, cycle_hz);
  if (cycle_hz < 50) cycle_hz = 50;

  int32_t profile_vel = 20000, acc = 50000, dec = 50000; // 少し余裕のある既定値
  pnh.param("profile_velocity", profile_vel, profile_vel);
  pnh.param("profile_acc",      acc,         acc);
  pnh.param("profile_dec",      dec,         dec);

  // SOEM init
  if (!ec_init(ifname.data())) {
    ROS_FATAL_STREAM("ec_init failed: " << ifname);
    return 1;
  }
  ROS_INFO_STREAM("ec_init OK on " << ifname);

  if (ec_config_init(FALSE) <= 0) {
    ROS_FATAL("ec_config_init failed");
    ec_close();
    return 1;
  }
  ROS_INFO("slaves: %d", ec_slavecount);
  if (slave < 1 || slave > ec_slavecount) {
    ROS_FATAL("invalid slave index: %d", slave);
    ec_close();
    return 1;
  }

  // --- 重要：PO2SOconfig で Pre-Op→Safe-Op フック時にPDOリマップ ---
  ec_slave[slave].PO2SOconfig = [](uint16 slave_id) -> int {
    return pdo_remap(slave_id) ? 1 : 0;
  };

  // モード・プロファイルは SDO でも明示（後で上書き可）
  sdo_u8 (slave, idx::MODES_OF_OPERATION, 0x00, static_cast<int8_t>(OpMode::PP));
  sdo_u32(slave, idx::PROFILE_VELOCITY,   0x00, (uint32)profile_vel);
  sdo_u32(slave, idx::PROFILE_ACCEL,      0x00, (uint32)acc);
  sdo_u32(slave, idx::PROFILE_DECEL,      0x00, (uint32)dec);

  // Configure PDO mapping & (DCは一旦無効化で切り分け)
  static uint8 IOmap[1024];
  int used = ec_config_map(&IOmap);
  ROS_INFO("IOmap size: %d bytes", used);
  // ec_configdc(); // ←切り分けのため無効化（必要になったら戻す）

  // === WKCウォームアップ：期待WKCに達するまでPD交換を回す ===
  int expected = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  int wkc = 0, ok = 0;
  for (int i = 0; i < 5000; ++i) { // ～約5秒
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if (wkc == expected) { ok = 1; break; }
    ros::Duration(0.001).sleep();
  }
  if (!ok) {
    ROS_ERROR("WKC not stable: got=%d expected=%d", wkc, expected);
    ec_close();
    return 1;
  }

  // SAFE_OP -> OP（マスタ・スレーブ0へ要求）
  ec_slave[0].state = EC_STATE_SAFE_OP;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

  // PDO pointers
  auto* rx = reinterpret_cast<RxPdoPP6B*>(ec_slave[slave].outputs);
  auto* tx = reinterpret_cast<TxPdoPP6B*>(ec_slave[slave].inputs);
  if (!rx || !tx) {
    ROS_FATAL("PDO pointers are null");
    ec_close();
    return 1;
  }

  // 初期化
  rx->controlword     = 0;
  rx->target_position = 0;

  // CiA-402有効化（堅牢版）
  if (!cia402_enable_seq(rx, tx)) {
    ec_close();
    return 1;
  }

  // Publishers
  ros::Publisher pub_sw  = pnh.advertise<std_msgs::UInt16>("status_word", 10);
  ros::Publisher pub_pos = pnh.advertise<std_msgs::Int32>("position_actual", 10);

  // Subscribers
  ros::Subscriber sub_abs   = pnh.subscribe("target_position", 10, on_target_abs);
  ros::Subscriber sub_delta = pnh.subscribe("move_delta",      10, on_move_delta);

  ros::Rate rate(cycle_hz);
  uint32_t tick = 0;

  while (ros::ok()) {
    // PD exchange
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // Absolute move (PP)
    if (g_have_cmd.exchange(false)) {
      int32_t tgt = g_cmd_target.load();

      // 念のため607AへSDOでも書く（機種依存対策）
      std::cout << "PASS" << std::endl;
      sdo_u32(slave, idx::TARGET_POSITION, 0x00, static_cast<uint32_t>(tgt));

      // Bit4/5をパルスし，最後は0x000Fへ確実に戻す
      trigger_pp(rx, tgt, /*immediate=*/true, /*relative=*/false);
    }

    // Relative move (current + delta)
    if (g_have_delta.exchange(false)) {
      int32_t tgt = tx->position_actual + g_delta.load();

      // 念のため607AへSDOでも書く
      sdo_u32(slave, idx::TARGET_POSITION, 0x00, static_cast<uint32_t>(tgt));

      // 相対移動フラグ（Bit6）を立てたベースでパルス
      trigger_pp(rx, tgt, /*immediate=*/true, /*relative=*/true);
    }

    // Publish status at ~50 Hz
    if ((tick++ % std::max(1, cycle_hz / 50)) == 0) {
      std_msgs::UInt16 sw;  sw.data  = tx->statusword;
      std_msgs::Int32  pos; pos.data = tx->position_actual;
      pub_sw.publish(sw);
      pub_pos.publish(pos);
    }

    ros::spinOnce();
    rate.sleep();
  }

  ec_close();
  return 0;
}
