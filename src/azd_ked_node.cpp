// src/azd_ked_node.cpp
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <mutex>
#include <atomic>
#include <cstring>
#include <cstdlib>

// SOEM
extern "C" {
#include <ethercat.h>
}

#define SLAVE_DEFAULT 1

#pragma pack(push,1)
struct RxPDO {
  uint16_t control;   // 0x6040
  int32_t  target;    // 0x607A
};
struct TxPDO {
  uint16_t status;    // 0x6041
  int32_t  position;  // 0x6064
};
#pragma pack(pop)

static std::mutex g_cmd_mtx;
static std::atomic<bool> g_have_cmd{false};
static std::atomic<int32_t> g_cmd_target{0};

static uint8_t sdo_u8(uint16 slave, uint16 idx, uint8 sub, uint8 val) {
  int wkc = ec_SDOwrite(slave, idx, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
  return (wkc > 0);
}
static uint8_t sdo_u16(uint16 slave, uint16 idx, uint8 sub, uint16 val) {
  int wkc = ec_SDOwrite(slave, idx, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
  return (wkc > 0);
}
static uint8_t sdo_u32(uint16 slave, uint16 idx, uint8 sub, uint32 val) {
  int wkc = ec_SDOwrite(slave, idx, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
  return (wkc > 0);
}

static bool pdo_remap(uint16 slave) {
  bool ok = true;

  // SM2 (Outputs / RxPDO) -> 0x1C12
  ok &= sdo_u8 (slave, 0x1C12, 0x00, 0);
  ok &= sdo_u8 (slave, 0x1600, 0x00, 0);
  ok &= sdo_u32(slave, 0x1600, 0x01, 0x60400010); // 0x6040:16
  ok &= sdo_u32(slave, 0x1600, 0x02, 0x607A0020); // 0x607A:32
  ok &= sdo_u8 (slave, 0x1600, 0x00, 2);
  ok &= sdo_u16(slave, 0x1C12, 0x01, 0x1600);
  ok &= sdo_u8 (slave, 0x1C12, 0x00, 1);

  // SM3 (Inputs / TxPDO) -> 0x1C13
  ok &= sdo_u8 (slave, 0x1C13, 0x00, 0);
  ok &= sdo_u8 (slave, 0x1A00, 0x00, 0);
  ok &= sdo_u32(slave, 0x1A00, 0x01, 0x60410010); // 0x6041:16
  ok &= sdo_u32(slave, 0x1A00, 0x02, 0x60640020); // 0x6064:32
  ok &= sdo_u8 (slave, 0x1A00, 0x00, 2);
  ok &= sdo_u16(slave, 0x1C13, 0x01, 0x1A00);
  ok &= sdo_u8 (slave, 0x1C13, 0x00, 1);

  return ok;
}

// CiA402: 状態遷移（Shutdown -> SwitchOn -> EnableOperation）
static void cia402_enable(uint16 slave, RxPDO* rx, const TxPDO* tx) {
  auto sendrecv = [&](){
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
  };

  // Shutdown（0x0006）
  rx->control = 0x0006;
  for (int i=0;i<10;i++){ sendrecv(); ros::Duration(0.02).sleep(); }

  // Switch On（0x0007）
  rx->control = 0x0007;
  for (int i=0;i<10;i++){ sendrecv(); ros::Duration(0.02).sleep(); }

  // Enable Operation（0x000F）
  rx->control = 0x000F;
  for (int i=0;i<10;i++){ sendrecv(); ros::Duration(0.02).sleep(); }
}

// PPモードで新セットポイントをトリガ
static void trigger_pp(RxPDO* rx, int32_t tgt) {
  rx->target = tgt;
  // toggle bit4 (new set-point). bit5(change immediately)は0のままでも動く実装が多い
  rx->control |=  (1<<4);
  // 1サイクル送ってから戻す
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  rx->control &= ~(1<<4);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "azd_ked_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string ifname = "enp0s31f6";
  int slave = SLAVE_DEFAULT;
  int cycle_hz = 500;
  pnh.param<std::string>("ifname", ifname, ifname);
  pnh.param<int>("slave_index", slave, slave);
  pnh.param<int>("cycle_hz", cycle_hz, cycle_hz);
  if (cycle_hz < 50) cycle_hz = 50;

  // (任意) 速度等 SDO初期化（必要に応じてパラメータ化）
  int32_t profile_vel = 20000, acc = 20000, dec = 20000;
  pnh.param<int>("profile_velocity", profile_vel, profile_vel);
  pnh.param<int>("profile_acc", acc, acc);
  pnh.param<int>("profile_dec", dec, dec);

  // SOEM 初期化
  if (!ec_init(ifname.c_str())) {
    ROS_FATAL_STREAM("ec_init failed: " << ifname);
    return 1;
  }
  ROS_INFO_STREAM("ec_init OK on " << ifname);

  if ( ec_config_init(FALSE) <= 0 ) {
    ROS_FATAL("ec_config_init failed");
    ec_close();
    return 1;
  }
  ROS_INFO("slaves: %d", ec_slavecount);

  // PDO再マップ + PPモード + 速度/加減速
  if (!pdo_remap(slave)) {
    ROS_WARN("PDO remap failed (continuing)");
  }
  // PP mode
  sdo_u8(slave, 0x6060, 0x00, 1);
  // profile vel/acc/dec
  sdo_u32(slave, 0x6081, 0x00, (uint32)profile_vel);
  sdo_u32(slave, 0x6083, 0x00, (uint32)acc);
  sdo_u32(slave, 0x6084, 0x00, (uint32)dec);

  // マッピング確定
  static uint8 IOmap[128];
  int used = ec_config_map(&IOmap);
  ROS_INFO("IOmap size: %d bytes", used);
  ec_configdc();

  // SAFE_OP -> OP（マスター＝スレーブ0の state を設定）
  ec_slave[0].state = EC_STATE_SAFE_OP;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

  // PDOポインタ
  RxPDO* rx = reinterpret_cast<RxPDO*>(ec_slave[slave].outputs);
  TxPDO* tx = reinterpret_cast<TxPDO*>(ec_slave[slave].inputs);
  if (!rx || !tx) {
    ROS_FATAL("PDO pointers are null");
    ec_close();
    return 1;
  }
  // 初期値
  rx->control = 0;
  rx->target  = 0;

  // CiA402 Enable
  cia402_enable(slave, rx, tx);

  // ROS pub/sub
  ros::Publisher pub_sw  = pnh.advertise<std_msgs::UInt16>("status_word", 10);
  ros::Publisher pub_pos = pnh.advertise<std_msgs::Int32>("position_actual", 10);

  auto cb_target_abs = [&](const std_msgs::Int32::ConstPtr& m){
    g_cmd_target = m->data;
    g_have_cmd = true;
  };
  auto cb_move_delta = [&](const std_msgs::Int32::ConstPtr& m){
    // 相対指令：現在位置 + delta
    int32_t tgt = tx->position + m->data;
    g_cmd_target = tgt;
    g_have_cmd = true;
  };
  ros::Subscriber sub_abs   = pnh.subscribe<std_msgs::Int32>("target_position", 10, cb_target_abs);
  ros::Subscriber sub_delta = pnh.subscribe<std_msgs::Int32>("move_delta", 10, cb_move_delta);

  ros::Rate rate(cycle_hz);
  uint32_t tick = 0;

  while (ros::ok()) {
    // 受信・送信
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // 新しいコマンドが来ていれば PP トリガ
    if (g_have_cmd.exchange(false)) {
      int32_t tgt = g_cmd_target.load();
      trigger_pp(rx, tgt);
    }

    // たまに(例: 50Hz)だけ状態をPub
    if ((tick++ % std::max(1, cycle_hz/50)) == 0) {
      std_msgs::UInt16 sw;   sw.data  = tx->status;
      std_msgs::Int32  pos;  pos.data = tx->position;
      pub_sw.publish(sw);
      pub_pos.publish(pos);
    }

    ros::spinOnce();
    rate.sleep();
  }

  ec_close();
  return 0;
}
