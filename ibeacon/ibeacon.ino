#include "STM32LowPower.h"
#include "TBGLib.h"

//=====================================================================
// シリアルコンソールへのデバック出力
//=====================================================================
#define DEBUG

//=====================================================================
// スリープ時間、送信時間の設定
//  SLEEP_INTERVAL : スリープ時間 (秒)
//  WAKE_INTERVAL：パケット送信時間 (秒)
//=====================================================================
#define SLEEP_INTERVAL (3)
#define WAKE_INTERVAL  (5)

//=====================================================================
// IOピンの名前定義
//=====================================================================
#define BLE_WAKEUP PB12 // D7   PB12
#define BLE_RX PA0      // [A2] PA1
#define BLE_TX PA1      // [A1] PA0
#define INT_0 PC7       // INT0
#define INT_1 PB3       // INT1

#define BEACON_UUID "e2c56db5-dffb-48d2-b060-d0f5a71096e0"
#define BEACON_MAJOR 1000
#define BEACON_MINOR 2000

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
void setupPort() {
  pinMode(BLE_WAKEUP, OUTPUT);    // BLE Wakeup/Sleep
  digitalWrite(BLE_WAKEUP, HIGH); // BLE Wakeup
}

//-----------------------------------------------
// BLE
//-----------------------------------------------
void setupBLE() {
  // set up internal status handlers
  ble112.onBusy = onBusy;
  ble112.onIdle = onIdle;
  ble112.onTimeout = onTimeout;
  // set up BGLib event handlers
  ble112.ble_evt_gatt_server_attribute_value = my_evt_gatt_server_attribute_value;
  ble112.ble_evt_le_connection_opend = my_evt_le_connection_opend;
  ble112.ble_evt_le_connection_closed = my_evt_le_connection_closed;
  ble112.ble_evt_system_boot = my_evt_system_boot;
  ble112.ble_evt_system_awake = my_evt_system_awake;
  ble112.ble_rsp_system_get_bt_address = my_rsp_system_get_bt_address;
  uint8_t tm = 0;
  Serialble.begin(9600);
  while (!Serialble && tm < 150) { // Serial起動待ち タイムアウト1.5s
    tm++;
    delay(10);
  }
  while (!bSystemBootBle) { // BLE起動待ち
    ble112.checkActivity(100);
  }
  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(1000));
  /* interval_min : 250ms( = 400 x 0.625ms ) */
  /* interval_max : 500ms( = 800 x 0.625ms ) */
  ble112.ble_cmd_le_gap_set_adv_parameters(400, 800, 7); /* [BGLIB] <interval_min> <interval_max> <channel_map> */
  while (ble112.checkActivity(1000));
}

void uuidToBytes(const char* uuid, uint8_t* bytes) {
  int len = strlen(uuid);
  int j = 0;

  for (int i = 0; i < len; i++) {
    if (uuid[i] == '-') continue;

    char byte_str[3] = { uuid[i], uuid[i+1], 0 };
    bytes[j++] = strtol(byte_str, NULL, 16);
    i++;
  }
}

//-----------------------------------------------
// アドバタイズするデータの設定
//-----------------------------------------------
void StartAdvData() {
  // UUIDをstrからbyteに変換
  uint8_t uuid_bytes[16];
  char* uuid_str = BEACON_UUID;
  uuidToBytes(uuid_str, uuid_bytes);

  // MajorとMinorを設定
  uint16_t major = BEACON_MAJOR;
  uint16_t minor = BEACON_MINOR;

  // Advertising data; 25byte MAX
  uint8_t adv_data[] = {
    // AD Structure 1: Flag
    (2),  // 0: field length
    BGLIB_GAP_AD_TYPE_FLAGS,  // 1: field type (0x01)
    (6),  // 2: data
    // AD Structure 2: Complete local name
    (26),  // 3: field length 0x1A
    (255),  // 4: field type (0xFF)
    (76),  // 5: company ID[0] 0x4C
    (0),   // 6: company ID[1] 0x00
    (2),   // 7: Beacon Type[0] 0x02
    (21),  // 8: Beacon Type[1] 0x15

    // UUID
    uuid_bytes[0], 
    uuid_bytes[1],
    uuid_bytes[2],
    uuid_bytes[3],
    uuid_bytes[4],
    uuid_bytes[5],
    uuid_bytes[6],
    uuid_bytes[7],
    uuid_bytes[8],
    uuid_bytes[9],
    uuid_bytes[10],
    uuid_bytes[11],
    uuid_bytes[12],
    uuid_bytes[13],
    uuid_bytes[14],
    uuid_bytes[15],

    // Major
    (major >> 8) & 0xFF,
    major & 0xFF,

    // Minor
    (minor >> 8) & 0xFF,
    minor & 0xFF,

    (60)  // Measured Power
  };


  // Register advertising packet
  uint8_t stLen = sizeof(adv_data);
  ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, stLen, adv_data);
  while (ble112.checkActivity(1000));
  // index = 0  LE_GAP_SCANNABLE_NON_CONNECTABLE / LE_GAP_UNDIRECTED_CONNECTABLE
  ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_SCANNABLE_NON_CONNECTABLE);
  while (ble112.checkActivity(1000));
}

//---------------------------------------
// sleep BLE
// BLE リーフをスリープさせる
//---------------------------------------
void sleepBLE() {
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
void wakeupBLE() {
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
void setup() {
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
void loop() {
  StartAdvData();
#ifdef DEBUG
  Serial.println(F("Start advertise"));
  Serial.flush();
#endif
  // Continue Advertising (during that STM32 sleeps.)
  LowPower.deepSleep(WAKE_INTERVAL * 1000);
#ifdef DEBUG
  Serial.println(F("Sleep BLE"));
#endif
  sleepBLE();
#ifdef DEBUG
  Serial.println(F("Sleep STM32"));
  Serial.println(F(">>> Sleep >>>"));
  Serial.flush();
#endif
  LowPower.deepSleep(SLEEP_INTERVAL * 1000);
#ifdef DEBUG
  Serial.println(F("Wakeup STM32"));
#endif
  wakeupBLE();
#ifdef DEBUG
  Serial.println(F("Wakeup BLE"));
#endif
#ifdef DEBUG
  Serial.println(F("<<< Wake up <<<"));
#endif
}

// ================================================================
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
// ================================================================
// called when the module begins sending a command
void onBusy() {}
// called when the module receives a complete response or "system_boot" event
void onIdle() {}
// called when the parser does not read the expected response in the specified time limit
void onTimeout() {}
// called immediately before beginning UART TX of a command
void onBeforeTXCommand() {}
// called immediately after finishing UART TX
void onTXCommandComplete() {}
// called when the attribute value changed
void my_evt_gatt_server_attribute_value(const struct ble_msg_gatt_server_attribute_value_evt_t *msg) {}
// called when the connection is opened
void my_evt_le_connection_opend(const ble_msg_le_connection_opend_evt_t *msg) {}
// called when connection is closed
void my_evt_le_connection_closed(const struct ble_msg_le_connection_closed_evt_t *msg) {}
// called when the system booted
void my_evt_system_boot(const ble_msg_system_boot_evt_t *msg) {
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
void my_evt_system_awake(void) {
  #ifdef DEBUG
    Serial.println("###\tsystem_awake");
  #endif
}
//
void my_rsp_system_get_bt_address(const struct ble_msg_system_get_bt_address_rsp_t *msg) {
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
