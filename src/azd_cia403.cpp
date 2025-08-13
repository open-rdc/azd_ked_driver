// azd_pp_driver.cpp
// Minimal PP control helper for SOEM + AZD (CiA-402)

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <soem/ethercat.h>

#include "azd_ked_driver/azd_cia402.hpp"

#define TIMEOUT   2000
#define MAX_IOMAP 4096

// === 内部状態 ===
static uint16_t g_slave_pos = 1;
static char     g_IOmap[MAX_IOMAP];

typedef azd::cia402::RxPdoPP6B RxPDO_t;
typedef azd::cia402::TxPdoPP6B TxPDO_t;

static inline void msleep(int ms){ struct timespec ts={ms/1000,(ms%1000)*1000000L}; nanosleep(&ts,nullptr); }
static int wait_state(uint16_t slave, uint16_t want, int to_ms){
  int waited=0; 
  while(ec_slave[slave].state!=want && waited<to_ms){
    ec_statecheck(slave,want,5000);
    msleep(10);
    waited+=10;
  }
  return (ec_slave[slave].state==want);
}

// --- SDO helpers ---
static int sdo_u8 (uint16 s, uint16 i, uint8 sub, uint8  v){ return ec_SDOwrite(s,i,sub,FALSE,sizeof(v),&v,TIMEOUT)>0; }
static int sdo_u16(uint16 s, uint16 i, uint8 sub, uint16 v){ return ec_SDOwrite(s,i,sub,FALSE,sizeof(v),&v,TIMEOUT)>0; }
static int sdo_u32(uint16 s, uint16 i, uint8 sub, uint32 v){ return ec_SDOwrite(s,i,sub,FALSE,sizeof(v),&v,TIMEOUT)>0; }
static int sdo_i32(uint16 s, uint16 i, uint8 sub, int32_t v){ return ec_SDOwrite(s,i,sub,FALSE,sizeof(v),&v,TIMEOUT)>0; }
static int sdo_rd_u8 (uint16 s, uint16 i, uint8 sub, uint8*  v){ int sz=1; return ec_SDOread(s,i,sub,FALSE,&sz,v,TIMEOUT)>0; }

// --- Set-point パルス ---
static void pulse_new_set_point(volatile RxPDO_t* rx)
{
  uint16_t base = azd::cia402::cw::ENABLE_OP; // 0x000F
  uint16_t on   = base | azd::cia402::cw::NEW_SETPOINT | azd::cia402::cw::CHANGE_IMMED;
  rx->controlword = on;
  for(int i=0;i<15;i++){ ec_send_processdata(); ec_receive_processdata(TIMEOUT); msleep(2); }
  rx->controlword = base;
  for(int i=0;i<5;i++){  ec_send_processdata(); ec_receive_processdata(TIMEOUT); msleep(2); }
}

// --- PDO再マップ ---
static int pdo_remap(uint16 slave)
{
  using namespace azd::cia402;
  int ok=1;

  // SM2 (Outputs) -> 0x1600: 6040/16, 607A/32
  ok&=sdo_u8 (slave, idx::SM2_PDO_ASSIGN, 0x00, 0);
  ok&=sdo_u8 (slave, idx::RX_PDO_MAP1,    0x00, 0);
  ok&=sdo_u32(slave, idx::RX_PDO_MAP1,    0x01, mapval::RX_6040_16);
  ok&=sdo_u32(slave, idx::RX_PDO_MAP1,    0x02, mapval::RX_607A_32);
  ok&=sdo_u8 (slave, idx::RX_PDO_MAP1,    0x00, 2);
  ok&=sdo_u16(slave, idx::SM2_PDO_ASSIGN, 0x01, idx::RX_PDO_MAP1);
  ok&=sdo_u8 (slave, idx::SM2_PDO_ASSIGN, 0x00, 1);

  // SM3 (Inputs) -> 0x1A00: 6041/16, 6064/32
  ok&=sdo_u8 (slave, idx::SM3_PDO_ASSIGN, 0x00, 0);
  ok&=sdo_u8 (slave, idx::TX_PDO_MAP1,    0x00, 0);
  ok&=sdo_u32(slave, idx::TX_PDO_MAP1,    0x01, mapval::TX_6041_16);
  ok&=sdo_u32(slave, idx::TX_PDO_MAP1,    0x02, mapval::TX_6064_32);
  ok&=sdo_u8 (slave, idx::TX_PDO_MAP1,    0x00, 2);
  ok&=sdo_u16(slave, idx::SM3_PDO_ASSIGN, 0x01, idx::TX_PDO_MAP1);
  ok&=sdo_u8 (slave, idx::SM3_PDO_ASSIGN, 0x00, 1);

  // 6060=1 (PP)
  ok&=sdo_u8 (slave, idx::MODES_OF_OPERATION, 0x00, static_cast<uint8_t>(azd::cia402::OpMode::PP));

  // プロファイル初期値
  ok&=sdo_u32(slave, idx::PROFILE_VELOCITY, 0x00, 20000);
  ok&=sdo_u32(slave, idx::PROFILE_ACCEL,    0x00, 50000);
  ok&=sdo_u32(slave, idx::PROFILE_DECEL,    0x00, 50000);

  if(!ok){ printf("PDO Remap Failed\n"); return 0; }
  return 1;
}

static int expected_wkc(void){ return (ec_group[0].outputsWKC*2)+ec_group[0].inputsWKC; }

// ===================== 公開API =====================

// 初期化
bool az_init(const char* ifname, uint16_t slave_pos)
{
  g_slave_pos = slave_pos;

  if(!ec_init((char*)ifname)){ fprintf(stderr,"ec_init failed\n"); return false; }
  if(ec_config_init(0) <= 0){ fprintf(stderr,"no slaves found\n"); ec_close(); return false; }

  ec_slave[g_slave_pos].PO2SOconfig = pdo_remap;

  int iomap = ec_config_map(&g_IOmap[0]);
  printf("IOmap: %d bytes (ID: %u, Output: %d byte, Input: %d byte)\n",
         iomap, g_slave_pos, ec_slave[g_slave_pos].Obytes, ec_slave[g_slave_pos].Ibytes);

  ec_slave[0].state = EC_STATE_SAFE_OP;
  ec_writestate(0);
  if(!wait_state(0,EC_STATE_SAFE_OP,5000)){ fprintf(stderr,"SAFE_OP failed\n"); ec_close(); return false; }
  ec_send_processdata(); ec_receive_processdata(TIMEOUT);

  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_writestate(0);
  if(!wait_state(0,EC_STATE_OPERATIONAL,5000)){ fprintf(stderr,"STATE_OP failed\n"); ec_close(); return false; }

  int wkc, exp=expected_wkc(), ok=0;
  for(int i=0;i<5000;i++){
    ec_send_processdata(); wkc=ec_receive_processdata(TIMEOUT);
    if(wkc==exp){ ok=1; break; }
    msleep(1);
  }
  if(!ok){ fprintf(stderr,"WKC not stable\n"); ec_close(); return false; }

  // Fault Reset相当（元コードと同じ処理・回数・sleep）
  volatile RxPDO_t* rx=(volatile RxPDO_t*)ec_slave[g_slave_pos].outputs;
  for(int i=0;i<20;i++){ ((RxPDO_t*)rx)->controlword=(1u<<7); ec_send_processdata(); ec_receive_processdata(TIMEOUT); msleep(5); }

  // Enable sequence（元コードと同じ）
  volatile TxPDO_t* tx=(volatile TxPDO_t*)ec_slave[g_slave_pos].inputs;
  int step=0, timeout=5000;
  while(timeout-- > 0){
    ec_send_processdata(); ec_receive_processdata(TIMEOUT);
    uint16_t sw = ((const TxPDO_t*)tx)->statusword;

    switch(step){
      case 0: ((RxPDO_t*)rx)->controlword = azd::cia402::cw::SHUTDOWN;       /* 0x0006 */ if(sw & 1) step=1; break;
      case 1: ((RxPDO_t*)rx)->controlword = azd::cia402::cw::SWITCH_ON_ONLY; /* 0x0007 */ if((sw & 0x006F)==0x0023) step=2; break;
      case 2: ((RxPDO_t*)rx)->controlword = azd::cia402::cw::ENABLE_OP;      /* 0x000F */ if((sw & 0x006F)==0x0027) step=3; break;
    }
    if(step==3) break;
    msleep(1);
  }
  if(step!=3){ fprintf(stderr,"Enable Sequence Failed\n"); ec_close(); return false; }

  // モード確認（元コードと同じ）
  uint8_t mode_disp=0;
  if(sdo_rd_u8(g_slave_pos, azd::cia402::idx::MODES_OF_DISPLAY, 0x00, &mode_disp))
    if(mode_disp!=static_cast<uint8_t>(azd::cia402::OpMode::PP)) {fprintf(stderr, "Position Control Mode Failed\n"); ec_close(); return false; }

  return true;
}

// 速度・加速度プロファイル設定
bool az_set_profiles(uint32_t vel, uint32_t acc, uint32_t dec)
{
  using namespace azd::cia402;
  int ok=1;
  ok&=sdo_u32(g_slave_pos, idx::PROFILE_VELOCITY, 0x00, vel);
  ok&=sdo_u32(g_slave_pos, idx::PROFILE_ACCEL,    0x00, acc);
  ok&=sdo_u32(g_slave_pos, idx::PROFILE_DECEL,    0x00, dec);
  return ok!=0;
}

// 目標位置を与えて駆動
bool az_move_to(int32_t target, int timeout_ms, int* out_pos, int* acked, int* reached)
{
  volatile RxPDO_t* rx=(volatile RxPDO_t*)ec_slave[g_slave_pos].outputs;
  volatile TxPDO_t* tx=(volatile TxPDO_t*)ec_slave[g_slave_pos].inputs;

  printf("Pos=%d\n", ((const TxPDO_t*)tx)->position_actual);

  // SDOに 607A、PDOにも同値（元コードと同じ手順）
  if(!sdo_i32(g_slave_pos, azd::cia402::idx::TARGET_POSITION, 0x00, target)) return false;
  ((RxPDO_t*)rx)->target_position = target;
  for(int k=0;k<5;k++){ ec_send_processdata(); ec_receive_processdata(TIMEOUT); msleep(5); }

  // Set-point パルス（元コード関数と同じ）
  pulse_new_set_point(rx);
  printf("Moving to %d ...\n", target);

  int _acked=0, _reached=0;
  for(int t=0; t<timeout_ms; ++t){
    ec_send_processdata(); ec_receive_processdata(TIMEOUT);
    uint16_t sw = ((const TxPDO_t*)tx)->statusword;
    if(!_acked && (sw & azd::cia402::sw::SETPOINT_ACK)) _acked=1;     // bit12
    if(sw & azd::cia402::sw::TARGET_REACHED){ _reached=1; break; }    // bit10（装置依存）
    msleep(1);
  }

  int32_t pos = ((const TxPDO_t*)tx)->position_actual;
  printf("Result: ACK=%d, REACHED=%d, Pos=%d\n", _acked, _reached, pos);

  if(out_pos) *out_pos = pos;
  if(acked)   *acked   = _acked;
  if(reached) *reached = _reached;
  return true;
}

void az_close()
{
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  ec_close();
}

// ===================== チェック用 main =====================
// 使い方: sudo ./azd_pp_driver <iface> <delta>
// 例:     sudo ./azd_pp_driver enp0s31f6 20000
int main(int argc, char* argv[])
{
  const char* ifname = (argc>1)? argv[1] : "enp0s31f6";
  int32_t delta = (argc>2)? atoi(argv[2]) : 20000;

  if(!az_init(ifname, /*slave_pos=*/1)) return 1;

  // 必要に応じて上書き（元の初期値と同じでもOK）
  if(!az_set_profiles(/*vel=*/20000, /*acc=*/50000, /*dec=*/50000)){
    fprintf(stderr, "Failed to set profiles\n");
    az_close();
    return 1;
  }

  // 現在位置 + delta に移動（元コード同様に SDO と PDO に書いてパルス）
  volatile TxPDO_t* tx=(volatile TxPDO_t*)ec_slave[g_slave_pos].inputs;
  int32_t target = ((const TxPDO_t*)tx)->position_actual + delta;

  int out=0, ack=0, reached=0;
  if(!az_move_to(target, /*timeout_ms=*/2000, &out, &ack, &reached)){
    fprintf(stderr, "Move command failed\n");
    az_close();
    return 1;
  }

  az_close();
  return 0;
}
