// src/azd_ked_multi_node.cpp
// Multi-motor AZD-KED PP control as a ROS1 node using SOEM.
// - Per-motor private namespace: ~motors/<name>/...
// - Sub:  target_position (absolute), move_delta (relative)
// - Pub:  status_word (UInt16), position_actual (Int32)
// - Requires CAP_NET_RAW and CAP_NET_ADMIN on the binary or run with sudo.

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <list>
#include <string>
#include <atomic>
#include <algorithm>

// SOEM (C headers)
extern "C" {
#include <ethercat.h>
}

// CiA-402 constants/types
#include "azd_ked_driver/azd_cia402.hpp"

using azd::cia402::RxPdoPP6B;
using azd::cia402::TxPdoPP6B;
namespace idx    = azd::cia402::idx;
namespace cw     = azd::cia402::cw;
using azd::cia402::OpMode;
using azd::cia402::DefaultPdoMap6B;
namespace mapval = azd::cia402::mapval;

struct MotorCfg {
  std::string name;      // unique motor name (namespace under ~motors)
  int         slave;     // EtherCAT slave index (1..ec_slavecount)
  int         profile_velocity{20000};
  int         profile_acc{20000};
  int         profile_dec{20000};
};

static bool sdo_u8 (uint16 slave, uint16 idx_, uint8 sub, uint8  val) {
  return ec_SDOwrite(slave, idx_, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM) > 0;
}
static bool sdo_u16(uint16 slave, uint16 idx_, uint8 sub, uint16 val) {
  return ec_SDOwrite(slave, idx_, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM) > 0;
}
static bool sdo_u32(uint16 slave, uint16 idx_, uint8 sub, uint32 val) {
  return ec_SDOwrite(slave, idx_, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM) > 0;
}

// PDO remap for 6B layout (6040/16 + 607A/32, 6041/16 + 6064/32)
static bool pdo_remap(uint16 slave) {
  bool ok = true;

  // SM2 (Outputs / RxPDO)
  ok &= sdo_u8 (slave, idx::SM2_PDO_ASSIGN, 0x00, 0);
  ok &= sdo_u8 (slave, idx::RX_PDO_MAP1,    0x00, 0);
  ok &= sdo_u32(slave, idx::RX_PDO_MAP1,    0x01, mapval::RX_6040_16);
  ok &= sdo_u32(slave, idx::RX_PDO_MAP1,    0x02, mapval::RX_607A_32);
  ok &= sdo_u8 (slave, idx::RX_PDO_MAP1,    0x00, DefaultPdoMap6B::rx_entries);
  ok &= sdo_u16(slave, idx::SM2_PDO_ASSIGN, 0x01, idx::RX_PDO_MAP1);
  ok &= sdo_u8 (slave, idx::SM2_PDO_ASSIGN, 0x00, 1);

  // SM3 (Inputs / TxPDO)
  ok &= sdo_u8 (slave, idx::SM3_PDO_ASSIGN, 0x00, 0);
  ok &= sdo_u8 (slave, idx::TX_PDO_MAP1,    0x00, 0);
  ok &= sdo_u32(slave, idx::TX_PDO_MAP1,    0x01, mapval::TX_6041_16);
  ok &= sdo_u32(slave, idx::TX_PDO_MAP1,    0x02, mapval::TX_6064_32);
  ok &= sdo_u8 (slave, idx::TX_PDO_MAP1,    0x00, DefaultPdoMap6B::tx_entries);
  ok &= sdo_u16(slave, idx::SM3_PDO_ASSIGN, 0x01, idx::TX_PDO_MAP1);
  ok &= sdo_u8 (slave, idx::SM3_PDO_ASSIGN, 0x00, 1);

  return ok;
}

// CiA-402 enable sequence (Shutdown -> SwitchOn -> EnableOp)
static void cia402_enable(RxPdoPP6B* rx) {
  auto sendrecv = [](){
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
  };
  rx->controlword = cw::SHUTDOWN;
  for (int i=0;i<10;i++){ sendrecv(); ros::Duration(0.02).sleep(); }
  rx->controlword = cw::SWITCH_ON_ONLY;
  for (int i=0;i<10;i++){ sendrecv(); ros::Duration(0.02).sleep(); }
  rx->controlword = cw::ENABLE_OP;
  for (int i=0;i<10;i++){ sendrecv(); ros::Duration(0.02).sleep(); }
}

// Per-motor runtime handle
struct Motor {
  Motor() = default;
  explicit Motor(const MotorCfg& c) : cfg(c) {}

  MotorCfg cfg;
  RxPdoPP6B* rx{nullptr};
  TxPdoPP6B* tx{nullptr};

  // command flags
  std::atomic<bool>    have_abs{false};
  std::atomic<int32_t> target_abs{0};
  std::atomic<bool>    have_delta{false};
  std::atomic<int32_t> delta{0};

  // pubs/subs
  ros::Publisher  pub_sw;
  ros::Publisher  pub_pos;
  ros::Subscriber sub_abs;
  ros::Subscriber sub_delta;

  // init ROS I/O under ~motors/<name>
  void setup_io(ros::NodeHandle& pnh) {
    ros::NodeHandle nh_mtr(pnh, std::string("motors/")+cfg.name);

    pub_sw  = nh_mtr.advertise<std_msgs::UInt16>("status_word", 10);
    pub_pos = nh_mtr.advertise<std_msgs::Int32>("position_actual", 10);

    sub_abs = nh_mtr.subscribe<std_msgs::Int32>(
      "target_position", 10,
      [this](const std_msgs::Int32::ConstPtr& m){
        this->target_abs = m->data;
        this->have_abs   = true;
      });

    sub_delta = nh_mtr.subscribe<std_msgs::Int32>(
      "move_delta", 10,
      [this](const std_msgs::Int32::ConstPtr& m){
        this->delta      = m->data;
        this->have_delta = true;
      });
  }

  // apply SDO settings for this motor (mode + profile)
  void apply_sdos() const {
    sdo_u8 (cfg.slave, idx::MODES_OF_OPERATION, 0x00, static_cast<int8_t>(OpMode::PP));
    sdo_u32(cfg.slave, idx::PROFILE_VELOCITY,   0x00, (uint32)cfg.profile_velocity);
    sdo_u32(cfg.slave, idx::PROFILE_ACCEL,      0x00, (uint32)cfg.profile_acc);
    sdo_u32(cfg.slave, idx::PROFILE_DECEL,      0x00, (uint32)cfg.profile_dec);
  }

  void trigger_pp_abs(int32_t tgt) {
    rx->target_position = tgt;
    rx->controlword |= cw::NEW_SETPOINT;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    rx->controlword &= ~cw::NEW_SETPOINT;
  }

  void spin_once_publish() {
    std_msgs::UInt16 sw;  sw.data  = tx->statusword;
    std_msgs::Int32  pos; pos.data = tx->position_actual;
    pub_sw.publish(sw);
    pub_pos.publish(pos);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "azd_ked_multi_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // --- global params ---
  std::string ifname = "enp0s31f6";
  int cycle_hz = 500;
  pnh.param("ifname", ifname, ifname);
  pnh.param("cycle_hz", cycle_hz, cycle_hz);
  if (cycle_hz < 50) cycle_hz = 50;

  // --- load motors list from YAML ---
  std::vector<std::string> motor_names;
  if (!pnh.getParam("motors/names", motor_names) || motor_names.empty()) {
    ROS_FATAL("~motors/names is empty. Define motor names in YAML.");
    return 1;
  }

  std::list<Motor> motors;
  for (const auto& name : motor_names) {
    MotorCfg cfg;
    cfg.name = name;
    ros::NodeHandle nh_mtr(pnh, std::string("motors/")+name);
    if (!nh_mtr.getParam("slave", cfg.slave)) {
      ROS_FATAL("~motors/%s/slave is missing", name.c_str());
      return 1;
    }
    nh_mtr.param("profile_velocity", cfg.profile_velocity, cfg.profile_velocity);
    nh_mtr.param("profile_acc",      cfg.profile_acc,      cfg.profile_acc);
    nh_mtr.param("profile_dec",      cfg.profile_dec,      cfg.profile_dec);
    motors.emplace_back(cfg);
  }

  // --- SOEM init ---
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

  // validate slave indices
  for (const auto& m : motors) {
    if (m.cfg.slave < 1 || m.cfg.slave > ec_slavecount) {
      ROS_FATAL("invalid slave index: %d for motor '%s'", m.cfg.slave, m.cfg.name.c_str());
      ec_close(); return 1;
    }
  }

  // --- PDO remap for each target slave & SDO settings ---
  for (const auto& m : motors) {
    if (!pdo_remap((uint16)m.cfg.slave)) {
      ROS_WARN("PDO remap failed on slave %d (continuing)", m.cfg.slave);
    }
  }

  // finalize PDO map & DC once for the whole bus
  static uint8 IOmap[256];
  int used = ec_config_map(&IOmap);
  ROS_INFO("IOmap size: %d bytes", used);
  ec_configdc();

  // SAFE_OP -> OP
  ec_slave[0].state = EC_STATE_SAFE_OP;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

  // PDO pointers & per-motor SDO applies, ROS I/O setup
  for (auto& m : motors) {
    m.rx = reinterpret_cast<RxPdoPP6B*>(ec_slave[m.cfg.slave].outputs);
    m.tx = reinterpret_cast<TxPdoPP6B*>(ec_slave[m.cfg.slave].inputs);
    if (!m.rx || !m.tx) {
      ROS_FATAL("PDO pointers are null on slave %d", m.cfg.slave);
      ec_close(); return 1;
    }
    // init outputs
    m.rx->controlword     = 0;
    m.rx->target_position = 0;

    m.apply_sdos();
    cia402_enable(m.rx);
    m.setup_io(pnh);

    ROS_INFO("motor '%s' ready on slave %d", m.cfg.name.c_str(), m.cfg.slave);
  }

  ros::Rate rate(cycle_hz);
  uint32_t tick = 0;

  while (ros::ok()) {
    // bus I/O
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // handle commands
    for (auto& m : motors) {
      if (m.have_abs.exchange(false)) {
        m.trigger_pp_abs(m.target_abs.load());
      }
      if (m.have_delta.exchange(false)) {
        int32_t tgt = m.tx->position_actual + m.delta.load();
        m.trigger_pp_abs(tgt);
      }
    }

    // publish at ~50 Hz
    if ((tick++ % std::max(1, cycle_hz/50)) == 0) {
      for (auto& m : motors) m.spin_once_publish();
    }

    ros::spinOnce();
    rate.sleep();
  }

  ec_close();
  return 0;
}
