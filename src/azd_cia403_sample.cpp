#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <soem/ethercat.h>

#include "azd_cia402.hpp"  // ← 追加

#define TIMEOUT 2000

#define SLAVE_POS 1
static char IOmap[4096];

// ヘッダの6B PDO構造にエイリアス（既存名を維持して処理は不変）
typedef azd::cia402::RxPdoPP6B RxPDO_t;
typedef azd::cia402::TxPdoPP6B TxPDO_t;

static void msleep(int ms){ struct timespec ts={ms/1000,(ms%1000)*1000000L}; nanosleep(&ts,NULL); }
static int wait_state(uint16_t slave, uint16_t want, int to_ms){
  int waited=0; while(ec_slave[slave].state!=want && waited<to_ms){ ec_statecheck(slave,want,5000); msleep(10); waited+=10; }
  return (ec_slave[slave].state==want);
}

static int sdo_u8 (uint16 s, uint16 i, uint8 sub, uint8  v){ return ec_SDOwrite(s,i,sub,FALSE,sizeof(v),&v,TIMEOUT)>0; }
static int sdo_u16(uint16 s, uint16 i, uint8 sub, uint16 v){ return ec_SDOwrite(s,i,sub,FALSE,sizeof(v),&v,TIMEOUT)>0; }
static int sdo_u32(uint16 s, uint16 i, uint8 sub, uint32 v){ return ec_SDOwrite(s,i,sub,FALSE,sizeof(v),&v,TIMEOUT)>0; }
static int sdo_i32(uint16 s, uint16 i, uint8 sub, int32_t v){ return ec_SDOwrite(s,i,sub,FALSE,sizeof(v),&v,TIMEOUT)>0; }
static int sdo_rd_u8 (uint16 s, uint16 i, uint8 sub, uint8*  v){ int sz=1; return ec_SDOread(s,i,sub,FALSE,&sz,v,TIMEOUT)>0; }

static void pulse_new_set_point(RxPDO_t* rx)
{
  // そのまま：制御語値・回数・sleepは変更なし
  uint16_t base = azd::cia402::cw::ENABLE_OP;                 // 0x000F
  uint16_t on   = base | azd::cia402::cw::NEW_SETPOINT | azd::cia402::cw::CHANGE_IMMED; // bit4/5
  rx->controlword = on;
  for(int i=0;i<15;i++){ ec_send_processdata(); ec_receive_processdata(TIMEOUT); msleep(2); }
  rx->controlword = base;
  for(int i=0;i<5;i++){  ec_send_processdata(); ec_receive_processdata(TIMEOUT); msleep(2); }
}

static int pdo_remap(uint16 slave)
{
  using namespace azd::cia402;

  int ok=1;

  // SM2 (Outputs) -> 0x1600: 6040/16, 607A/32
  ok&=sdo_u8 (slave, idx::SM2_PDO_ASSIGN, 0x00, 0);             // clear PDO
  ok&=sdo_u8 (slave, idx::RX_PDO_MAP1,    0x00, 0);             // clear PDO map
  ok&=sdo_u32(slave, idx::RX_PDO_MAP1,    0x01, mapval::RX_6040_16); // add control word (0x60400010)
  ok&=sdo_u32(slave, idx::RX_PDO_MAP1,    0x02, mapval::RX_607A_32); // add target position (0x607A0020)
  ok&=sdo_u8 (slave, idx::RX_PDO_MAP1,    0x00, 2);             // set PDO map
  ok&=sdo_u16(slave, idx::SM2_PDO_ASSIGN, 0x01, idx::RX_PDO_MAP1);   // set PDO address
  ok&=sdo_u8 (slave, idx::SM2_PDO_ASSIGN, 0x00, 1);             // set PDO

  // SM3 (Inputs) -> 0x1A00: 6041/16, 6064/32
  ok&=sdo_u8 (slave, idx::SM3_PDO_ASSIGN, 0x00, 0);             // clear PDO
  ok&=sdo_u8 (slave, idx::TX_PDO_MAP1,    0x00, 0);             // clear PDO map
  ok&=sdo_u32(slave, idx::TX_PDO_MAP1,    0x01, mapval::TX_6041_16); // add status word (0x60410010)
  ok&=sdo_u32(slave, idx::TX_PDO_MAP1,    0x02, mapval::TX_6064_32); // add measured position (0x60640020)
  ok&=sdo_u8 (slave, idx::TX_PDO_MAP1,    0x00, 2);             // set PDO map
  ok&=sdo_u16(slave, idx::SM3_PDO_ASSIGN, 0x01, idx::TX_PDO_MAP1);   // set PDO address
  ok&=sdo_u8 (slave, idx::SM3_PDO_ASSIGN, 0x00, 1);             // set PDO

  ok&=sdo_u8 (slave, idx::MODES_OF_OPERATION, 0x00, static_cast<uint8_t>(OpMode::PP)); // 6060=1

  ok&=sdo_u32(slave, idx::PROFILE_VELOCITY, 0x00, 20000);  // 6081
  ok&=sdo_u32(slave, idx::PROFILE_ACCEL,    0x00, 50000);  // 6083
  ok&=sdo_u32(slave, idx::PROFILE_DECEL,    0x00, 50000);  // 6084

  if(!ok){ printf("PDO Remap Failed\n"); return 0; }
  return 1;
}

static int expected_wkc(void){ return (ec_group[0].outputsWKC*2)+ec_group[0].inputsWKC; }

int main(int argc, char *argv[])
{
  const char* ifname = (argc>1)? argv[1] : "enp0s31f6";
  if(!ec_init((char*)ifname)){ fprintf(stderr,"ec_init failed\n"); return 1; }
  if(ec_config_init(0) <= 0){ fprintf(stderr,"no slaves found\n"); ec_close(); return 1; }

  ec_slave[SLAVE_POS].PO2SOconfig = pdo_remap;  // callback func (pre -> safe mode)

  int iomap = ec_config_map(&IOmap);
  printf("IOmap: %d bytes (ID: %u, Output: %d byte, Input: %d byte)\n", iomap, SLAVE_POS, ec_slave[SLAVE_POS].Obytes, ec_slave[SLAVE_POS].Ibytes);

  ec_slave[0].state = EC_STATE_SAFE_OP;     // safe mode
  ec_writestate(0);
  if(!wait_state(0,EC_STATE_SAFE_OP,5000)){ fprintf(stderr,"SAFE_OP failed\n"); ec_close(); return 1; }
  ec_send_processdata(); ec_receive_processdata(TIMEOUT);

  ec_slave[0].state = EC_STATE_OPERATIONAL; // operation mode
  ec_writestate(0);
  if(!wait_state(0,EC_STATE_OPERATIONAL,5000)){ fprintf(stderr,"STATE_OP failed\n"); ec_close(); return 1; }

  int wkc, exp=expected_wkc(), ok=0;
  for(int i=0;i<5000;i++){ ec_send_processdata(); wkc=ec_receive_processdata(TIMEOUT); if(wkc==exp){ ok=1; break; } msleep(1); }
  if(!ok){ fprintf(stderr,"WKC not stable\n"); ec_close(); return 1; }

  RxPDO_t* rx=(RxPDO_t*)ec_slave[SLAVE_POS].outputs;
  TxPDO_t* tx=(TxPDO_t*)ec_slave[SLAVE_POS].inputs;

  for(int i=0;i<20;i++){ rx->controlword=(1u<<7); ec_send_processdata(); ec_receive_processdata(TIMEOUT); msleep(5); }
  int step=0, timeout=5000;
  while(timeout-- > 0){
    ec_send_processdata(); ec_receive_processdata(TIMEOUT);
    uint16_t sw = tx->statusword;

    switch(step){
      case 0: rx->controlword = azd::cia402::cw::SHUTDOWN;       /* 0x0006 */ if(sw & 1) step=1; break;
      case 1: rx->controlword = azd::cia402::cw::SWITCH_ON_ONLY; /* 0x0007 */ if((sw & 0x006F)==0x0023) step=2; break;
      case 2: rx->controlword = azd::cia402::cw::ENABLE_OP;      /* 0x000F */ if((sw & 0x006F)==0x0027) step=3; break;
    }
    if(step==3) break;
    msleep(1);
  }
  if(step!=3){ fprintf(stderr,"Enable Sequence Failed\n"); ec_close(); return 1; }

  uint8_t mode_disp=0;
  if(sdo_rd_u8(SLAVE_POS, azd::cia402::idx::MODES_OF_DISPLAY, 0x00, &mode_disp))
    if(mode_disp!=static_cast<uint8_t>(azd::cia402::OpMode::PP)) {fprintf(stderr, "Position Control Mode Failed\n"); ec_close(); return 1; }

  printf("Pos=%d\n", tx->position_actual);
  int32_t target = tx->position_actual + 20000;
  sdo_i32(SLAVE_POS, azd::cia402::idx::TARGET_POSITION, 0x00, target);
  rx->target_position = target;
  for(int k=0;k<5;k++){ ec_send_processdata(); ec_receive_processdata(TIMEOUT); msleep(5); }

  pulse_new_set_point(rx);
  printf("Moving to %d ...\n", target);

  int acked=0, reached=0;
  for(int t=0;t<2000;++t){
    ec_send_processdata(); ec_receive_processdata(TIMEOUT);
    uint16_t sw = tx->statusword;
    if(!acked && (sw & azd::cia402::sw::SETPOINT_ACK)) acked=1;   // Set-point Acknowledge (bit12)
    if(sw & azd::cia402::sw::TARGET_REACHED){ reached=1; break; } // Target reached (bit10) ※装置依存
    msleep(1);
  }
  printf("Result: ACK=%d, REACHED=%d, Pos=%d, SW=", acked, reached, tx->position_actual);

  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  ec_close();
  return 0;
}
