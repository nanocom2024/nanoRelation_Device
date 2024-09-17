//=====================================================================
//  Leafony Platform sample sketch
//     Application  : STM32 Simple BLE Beacon Example
//     Processor    : STM32L452RE (Nucleo-64/Nucleo L452RE)
//     Arduino IDE  : 1.8.12
//     STM32 Core   : Ver1.9.0
//
//     Leaf configuration
//       (1) AC02 BLE Sugar     :Bus-A or Bus-B
//       (2) AP03 STM32 MCU
//       (3) AZ01 USB           :Bus-A
//
//    (c) 2021 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01 First release
//=====================================================================
//  Required Libraries
//    https://github.com/Leafony/TBGLib
//    https://github.com/stm32duino/STM32LowPower
//    https://github.com/stm32duino/STM32RTC
//=====================================================================
#include "STM32LowPower.h"
#include "TBGLib.h"

String strDeviceName = "Leafony_dayo";

//=====================================================================
// シリアルコンソールへのデバック出力
//      #define DEBUG = 出力あり
//　　//#define DEBUG = 出力なし（コメントアウトする）
//=====================================================================
#define DEBUG

//=====================================================================
// スリープ時間、送信時間の設定
//  SLEEP_INTERVAL : スリープ時間 (秒)
//  WAKE_INTERVAL　：パケット送信時間 (秒)
//=====================================================================
#define SLEEP_INTERVAL (8)
#define WAKE_INTERVAL  (1)

//=====================================================================
// IOピンの名前定義
//=====================================================================
#define BLE_WAKEUP PB12 // D7   PB12
#define BLE_RX PA0      // [A2] PA1
#define BLE_TX PA1      // [A1] PA0
#define INT_0 PC7       // INT0
#define INT_1 PB3       // INT1

//=====================================================================
// objects
//=====================================================================
// BLE
HardwareSerial Serialble(BLE_TX, BLE_RX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0);

//=====================================================================
// Variables
//=====================================================================
// BLE
volatile bool bSystemBootBle = false;

//=====================================================================
// IOピンの入出力設定
// 接続するリーフに合わせて設定する
//=====================================================================
void setupPort()
{
  pinMode(BLE_WAKEUP, OUTPUT);    // BLE Wakeup/Sleep
  digitalWrite(BLE_WAKEUP, HIGH); // BLE Wakeup
}

//-----------------------------------------------
// BLE
//-----------------------------------------------
void setupBLE()
{
  uint8  stLen;
  uint8 adv_data[31];

  // set up internal status handlers (these are technically optional)
  ble112.onBusy = onBusy;
  ble112.onIdle = onIdle;
  ble112.onTimeout = onTimeout;
  // ONLY enable these if you are using the <wakeup_pin> parameter in your firmware's hardware.xml file
  // BLE module must be woken up before sending any UART data

  // set up BGLib response handlers (called almost immediately after sending commands)
  // (these are also technicaly optional)

  // set up BGLib event handlers
  /* [gatt_server] */
  ble112.ble_evt_gatt_server_attribute_value = my_evt_gatt_server_attribute_value;    /* [BGLib] */
  /* [le_connection] */
  ble112.ble_evt_le_connection_opend = my_evt_le_connection_opend;                    /* [BGLib] */
  ble112.ble_evt_le_connection_closed = my_evt_le_connection_closed;                  /* [BGLib] */
  /* [system] */
  ble112.ble_evt_system_boot = my_evt_system_boot;                                    /* [BGLib] */

  ble112.ble_evt_system_awake = my_evt_system_awake;
  ble112.ble_rsp_system_get_bt_address = my_rsp_system_get_bt_address;

  uint8_t tm=0;
  Serialble.begin(9600);
  while (!Serialble && tm <150){                              // Wait for Serial to start Timeout 1.5s
    tm++;
    delay(10);
  }

  tm=0;
  while (!bSystemBootBle && tm <150){                         // Waiting for BLE to start
    ble112.checkActivity(100);
    tm++;
    delay(10);
  }

  /* setting */
  /* [set Advertising Data] */
  uint8 ad_data[21] = {
      (2),                                                    // field length
      BGLIB_GAP_AD_TYPE_FLAGS,                                // field type (0x01)
      (6),                                                    // data
      (1),                                                    // field length (1は仮の初期値)
      BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE                    // field type (0x09)
  };

  /*  */
  size_t lenStr2 = strDeviceName.length();

  ad_data[3] = (lenStr2 + 1);                                 // field length
  uint8 u8Index;
  for( u8Index=0; u8Index < lenStr2; u8Index++){
    ad_data[5 + u8Index] = strDeviceName.charAt(u8Index);
  }

  /*   */
  stLen = (5 + lenStr2);

  //ble112.ble_cmd_le_gap_bt5_set_adv_data(0,SCAN_RSP_ADVERTISING_PACKETS, stLen, ad_data);
  ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, stLen, ad_data);

  while (ble112.checkActivity(1000));                         /* Receive check */
  delay(20);

  /* interval_min :   40ms( =   64 x 0.625ms ) */
  //ble112.ble_cmd_le_gap_bt5_set_adv_parameters( 0, 64, 1600, 7, 0 );/* [BGLIB] <handle> <interval_min> <interval_max> <channel_map> <report_scan>*/
  /* interval_max : 1000ms( = 1600 x 0.625ms ) */
  ble112.ble_cmd_le_gap_set_adv_parameters( 64, 1600, 7 );    /* [BGLIB] <interval_min> <interval_max> <channel_map> */

  while (ble112.checkActivity(1000));                         /* [BGLIB] Receive check */

  /* start */
//    ble112.ble_cmd_le_gap_bt5_set_mode(0,LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE,0,2);
//    ble112.ble_cmd_le_gap_set_mode(LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE);
//    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);     // index = 0
  ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE);                // index = 0
  while (ble112.checkActivity(1000));                         /* Receive check */
  /*  */
}

//---------------------------------------
// sleep BLE
// BLE リーフをスリープさせる
//---------------------------------------
void sleepBLE()
{
  ble112.ble_cmd_le_gap_stop_advertising(0);
  while (ble112.checkActivity());

  ble112.ble_cmd_system_halt(1);
  while (ble112.checkActivity());

  digitalWrite(BLE_WAKEUP, LOW);
  delay(500);
}

//---------------------------------------
// wakeup BLE
// BLEリーフをスリープから復帰させる
//---------------------------------------
void wakeupBLE()
{
  digitalWrite(BLE_WAKEUP, HIGH);
  delay(500);

  ble112.ble_cmd_system_halt(0);
  while (ble112.checkActivity());

  ble112.ble_cmd_le_gap_set_adv_parameters(400, 800, 7);
  while (ble112.checkActivity(1000));
}

//---------------------------------------
// setup
//
//---------------------------------------
void setup()
{
  Serial.begin(115200);
  LowPower.begin(); // Configure low power

  setupPort();
  delay(10);

  setupBLE();
}

//---------------------------------------
// loop
//
//---------------------------------------
void loop()
{
}


// ================================================================
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
// ================================================================

// called when the module begins sending a command
void onBusy()
{
}

// called when the module receives a complete response or "system_boot" event
void onIdle()
{
}

// called when the parser does not read the expected response in the specified time limit
void onTimeout()
{
}

// called immediately before beginning UART TX of a command
void onBeforeTXCommand()
{
}

// called immediately after finishing UART TX
void onTXCommandComplete()
{
}

// called when the attribute value changed
void my_evt_gatt_server_attribute_value(const struct ble_msg_gatt_server_attribute_value_evt_t *msg)
{
}

// called when the connection is opened
void my_evt_le_connection_opend(const ble_msg_le_connection_opend_evt_t *msg)
{
  Serial.print(F("###\tconnection_opend: { "));
  Serial.print(F("address: "));
  // this is a "bd_addr" data type, which is a 6-byte uint8_t array
  for (uint8_t i = 0; i < 6; i++) {
      if (msg -> address.addr[i] < 16) Serial.write('0');
      Serial.print(msg -> address.addr[i], HEX);
  }
  Serial.print(", address_type: "); Serial.print(msg -> address_type, HEX);
  Serial.print(", master: "); Serial.print(msg -> master, HEX);
  Serial.print(", connection: "); Serial.print(msg -> connection, HEX);
  Serial.print(", bonding: "); Serial.print(msg -> bonding, HEX);
  Serial.print(", advertiser: "); Serial.print(msg -> advertiser, HEX);
  Serial.println(" }");
}

// called when connection is closed
void my_evt_le_connection_closed(const struct ble_msg_le_connection_closed_evt_t *msg)
{
}

// called when the system booted
void my_evt_system_boot(const ble_msg_system_boot_evt_t *msg)
{
#ifdef DEBUG
  Serial.print("###\tsystem_boot: { ");
  Serial.print("major: ");
  Serial.print(msg->major, HEX);
  Serial.print(", minor: ");
  Serial.print(msg->minor, HEX);
  Serial.print(", patch: ");
  Serial.print(msg->patch, HEX);
  Serial.print(", build: ");
  Serial.print(msg->build, HEX);
  Serial.print(", bootloader_version: ");
  Serial.print(msg->bootloader, HEX);
  Serial.print(", hw: ");
  Serial.print(msg->hw, HEX);
  Serial.println(" }");
#endif

  bSystemBootBle = true;
}

// called when the system awake
void my_evt_system_awake(void)
{
#ifdef DEBUG
  Serial.println("###\tsystem_awake");
#endif
}

//
void my_rsp_system_get_bt_address(const struct ble_msg_system_get_bt_address_rsp_t *msg)
{
#ifdef DEBUG
  Serial.print("###\tsystem_get_bt_address: { ");
  Serial.print("address: ");
  for (int i = 0; i < 6; i++)
  {
    Serial.print(msg->address.addr[i], HEX);
  }
  Serial.println(" }");
#endif
  unsigned short addr = 0;
  char cAddr[30];
  addr = msg->address.addr[0] + (msg->address.addr[1] * 0x100);
  sprintf(cAddr, "Device name is Leaf_A_#%05d ", addr);
  Serial.println(cAddr);
}