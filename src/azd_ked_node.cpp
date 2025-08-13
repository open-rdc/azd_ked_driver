// src/azd_ked_node.cpp
// AZD-KED minimal PP control as a ROS1 node using SOEM.
// - Uses azd_cia402.hpp for readable object indices, masks, and PDO types.
// - Publishes:  ~status_word (std_msgs/UInt16), ~position_actual (std_msgs/Int32)
// - Subscribes: ~target_position (absolute), ~move_delta (relative)
// - Requires CAP_NET_RAW and CAP_NET_ADMIN on the binary or run with sudo.

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <atomic>
#include <algorithm>   // std::max
#include <cstdlib>
#include <cstring>

// SOEM (C headers)
extern "C" {
#include <ethercat.h>
}

// AZD-KED CiA-402 constants/types
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
  ok &= sdo_u8 (slave, idx::SM2_PDO_ASSIGN, 0x00, 0);
  ok &= sdo_u8 (slave, idx::RX_PDO_MAP1,   0x00, 0);
  ok &= sdo_u32(slave, idx::RX_PDO_MAP1,   0x01, mapval::RX_6040_16);
  ok &= sdo_u32(slave, idx::RX_PDO_MAP1,   0x02, mapval::RX_607A_32);
  ok &= sdo_u8 (slave, idx::RX_PDO_MAP1,   0x00, DefaultPdoMap6B::rx_entries);
  ok &= sdo_u16(slave, idx::SM2_PDO_ASSIGN,0x01, idx::RX_PDO_MAP1);
  ok &= sdo_u8 (slave, idx::SM2_PDO_ASSIGN,0x00, 1);

  // SM3 (Inputs / TxPDO) -> 1C13, map at 1A00
  ok &= sdo_u8 (slave, idx::SM3_PDO_ASSIGN, 0x00, 0);
  ok &= sdo_u8 (slave, idx::TX_PDO_MAP1,    0x00, 0);
  ok &= sdo_u32(slave, idx::TX_PDO_MAP1,    0x01, mapval::TX_6041_16);
  ok &= sdo_u32(slave, idx::TX_PDO_MAP1,    0x02, mapval::TX_6064_32);
  ok &= sdo_u8 (slave, idx::TX_PDO_MAP1,    0x00, DefaultPdoMap6B::tx_entries);
  ok &= sdo_u16(slave, idx::SM3_PDO_ASSIGN, 0x01, idx::TX_PDO_MAP1);
  ok &= sdo_u8 (slave, idx::SM3_PDO_ASSIGN, 0x00, 1);

  return ok;
}

// ===== CiA-402 state transition: Shutdown -> SwitchOn -> EnableOperation =====
static void cia402_enable(uint16 /*slave*/, azd::cia402::RxPdoPP6B* rx) {
  using namespace azd::cia402;
  auto sendrecv = [](){
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
  };

  rx->controlword = cw::SHUTDOWN;
  for (int i = 0; i < 10; ++i) { sendrecv(); ros::Duration(0.02).sleep(); }

  rx->controlword = cw::SWITCH_ON_ONLY;
  for (int i = 0; i < 10; ++i) { sendrecv(); ros::Duration(0.02).sleep(); }

  rx->controlword = cw::ENABLE_OP;
  for (int i = 0; i < 10; ++i) { sendrecv(); ros::Duration(0.02).sleep(); }
}

// ===== Trigger new set-point in PP mode =====
static void trigger_pp(azd::cia402::RxPdoPP6B* rx, int32_t tgt) {
  using namespace azd::cia402;
  rx->target_position = tgt;
  rx->controlword |= cw::NEW_SETPOINT;
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  rx->controlword &= ~cw::NEW_SETPOINT;
}

// ===== Free-function callbacks to avoid subscribe() ambiguity =====
static void on_target_abs(const std_msgs::Int32::ConstPtr& m) {
  g_cmd_target = m->data;
  g_have_cmd   = true;
}
static void on_move_delta(const std_msgs::Int32::ConstPtr& m) {
  g_delta      = m->data;  // store delta only; apply in main loop using current position
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

  int32_t profile_vel = 20000, acc = 20000, dec = 20000;
  pnh.param("profile_velocity", profile_vel, profile_vel);
  pnh.param("profile_acc",      acc,         acc);
  pnh.param("profile_dec",      dec,         dec);

  // SOEM init
  if (!ec_init(ifname.c_str())) {
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

  // PDO remap + mode and motion profile SDOs
  if (!pdo_remap(slave)) {
    ROS_WARN("PDO remap failed (continuing)");
  }
  sdo_u8 (slave, idx::MODES_OF_OPERATION, 0x00, static_cast<int8_t>(OpMode::PP));
  sdo_u32(slave, idx::PROFILE_VELOCITY,   0x00, (uint32)profile_vel);
  sdo_u32(slave, idx::PROFILE_ACCEL,      0x00, (uint32)acc);
  sdo_u32(slave, idx::PROFILE_DECEL,      0x00, (uint32)dec);

  // Configure PDO mapping & DC
  static uint8 IOmap[128];
  int used = ec_config_map(&IOmap);
  ROS_INFO("IOmap size: %d bytes", used);
  ec_configdc();

  // SAFE_OP -> OP (set master / slave 0 state)
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

  // Initialize outputs
  rx->controlword     = 0;
  rx->target_position = 0;

  // Enable CiA-402
  cia402_enable(slave, rx);

  // Publishers
  ros::Publisher pub_sw  = pnh.advertise<std_msgs::UInt16>("status_word", 10);
  ros::Publisher pub_pos = pnh.advertise<std_msgs::Int32>("position_actual", 10);

  // Subscribers (free functions to avoid overload ambiguity)
  ros::Subscriber sub_abs   = pnh.subscribe("target_position", 10, on_target_abs);
  ros::Subscriber sub_delta = pnh.subscribe("move_delta",      10, on_move_delta);

  ros::Rate rate(cycle_hz);
  uint32_t tick = 0;

  while (ros::ok()) {
    // Process data exchange
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // Absolute move (PP)
    if (g_have_cmd.exchange(false)) {
      trigger_pp(rx, g_cmd_target.load());
    }

    // Relative move: current position + delta
    if (g_have_delta.exchange(false)) {
      int32_t tgt = tx->position_actual + g_delta.load();
      trigger_pp(rx, tgt);
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
