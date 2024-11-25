#include "TBGLib.h"         // BLE
#include <SoftwareSerial.h> // Software UART
#include "STM32LowPower.h"

#define SERIAL_MONITOR
#define DEBUG

// iBeaconのSleepとWakeの時間
#define SLEEP_INTERVAL (3)
#define WAKE_INTERVAL  (5)

// iBeaconのUUID
#define BEACON_UUID "e2c56db5-dffb-48d2-b060-d0f5a71096e0"
// 迷子検知用のiBeaconUUID
#define LOST_BEACON_UUID "e2c56db5-dffb-48d2-b060-d0f5a71096e1"  // 要変更

// GATTとiBeaconモード切り替えのBool
volatile bool ibeacon = false;

// 迷子検知モードのフラグ
volatile bool lostChild = false;

volatile uint16_t ibeacon_major;
volatile uint16_t ibeacon_minor;

//-----------------------------------------------
// Setting the transmission interval
//  SEND_INTERVAL  :transmission interval (Set the interval for sending sensor
//  data in 1 second increments.)
//-----------------------------------------------
#define SEND_INTERVAL (1) // 1s

// IOピンの名前定義
#define BLE_WAKEUP PB12 // D7   PB12
#define BLE_RX PA1      // [A2] PA1
#define BLE_TX PA0      // [A1] PA0
#define INT_0 PC7       // INT0
#define INT_1 PB3       // INT1

//------------------------------
// Loop interval
// Timer interrupt interval (ms)
//------------------------------
#define LOOP_INTERVAL 125000 // 125000us = 125ms interval

// BLE state
#define BLE_STATE_STANDBY (0)
#define BLE_STATE_SCANNING (1)
#define BLE_STATE_ADVERTISING (2)
#define BLE_STATE_CONNECTING (3)
#define BLE_STATE_CONNECTED_MASTER (4)
#define BLE_STATE_CONNECTED_SLAVE (5)

// BLE
HardwareSerial Serialble(BLE_RX, BLE_TX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0);

uint8_t iLoop1s = 0;
uint8_t iSendCounter = 0;

//------------------------------
// Event
//------------------------------
bool event1s = false;

//------------------------------
// interval Timer2 interrupt
//------------------------------
volatile bool bInterval = false;

//--------------------
// Data for two-point correction
//--------------------
// Temperature correction data 0
float TL0 = 25.0; // 4-Sensors Temperature measurement value
float TM0 = 25.0; // Thermometer and other measurements value
// Temperature correction data 1
float TL1 = 40.0; // 4-Sensors Temperature measurement value
float TM1 = 40.0; // Thermometer and other measurements value

// Humidity correction data 0
float HL0 = 60.0; // 4-Sensors Humidity measurement value
float HM0 = 60.0; // Hygrometer and other measurements value
// Humidity correction data 1
float HL1 = 80.0; // 4-Sensors Humidity measurement value
float HM1 = 80.0; // Hygrometer and other measurements value

//------------------------------
// OPT3001 : Ambient Light Sensor
//------------------------------
float dataLight;


// BLEのState
bool bBLEconnect = false;
bool bBLEsendData = false;
volatile bool bSystemBootBle = false;

volatile uint8_t ble_state = BLE_STATE_STANDBY;
volatile uint8_t ble_encrypted = 0; // 0 = not encrypted, otherwise = encrypted
volatile uint8_t ble_bonding = 0xFF; // 0xFF = no bonding, otherwise = bonding handle

float dataBatt = 0;

void generateRandomString(uint8 *outputData, int stringLength){
  char *letters = "abcdefghijklmnopqrstuvwxyz0123456789";

  for(int i = 0; i < stringLength; i++){
    outputData[i] = letters[random(36)];
  }
}

// IOピンの入出力設定
// 接続するリーフに合わせて設定する
void setupPort() {
  pinMode(BLE_WAKEUP, OUTPUT);    // BLE Wakeup/Sleep
  digitalWrite(BLE_WAKEUP, HIGH); // BLE Wakeup
}

// void intTimer(HardwareTimer*){      // STM32 Core 1.7.0
void intTimer(void) { // STM32 Core 1.9.0
    bInterval = true;
}

void setupTimerInt() {
    HardwareTimer *timer2 = new HardwareTimer(TIM2);

    timer2->setOverflow(LOOP_INTERVAL, MICROSEC_FORMAT); // 125ms
    timer2->attachInterrupt(intTimer);
    timer2->resume();
}

// BLEの設定
void setupBLE() {
    uint8 stLen;
    uint8 adv_data[31];

    // set up internal status handlers (these are technically optional)
    ble112.onBusy = onBusy;
    ble112.onIdle = onIdle;
    ble112.onTimeout = onTimeout;
    // ONLY enable these if you are using the <wakeup_pin> parameter in your
    // firmware's hardware.xml file BLE module must be woken up before sending
    // any UART data

    // set up BGLib response handlers (called almost immediately after sending
    // commands) (these are also technicaly optional)

    // set up BGLib event handlers
    /* [gatt_server] */
    ble112.ble_evt_gatt_server_attribute_value = my_evt_gatt_server_attribute_value; /* [BGLib] */
    /* [le_connection] */
    ble112.ble_evt_le_connection_opend = my_evt_le_connection_opend; /* [BGLib] */
    ble112.ble_evt_le_connection_closed = my_evt_le_connection_closed; /* [BGLib] */
    /* [system] */
    ble112.ble_evt_system_boot = my_evt_system_boot; /* [BGLib] */

    ble112.ble_evt_system_awake = my_evt_system_awake;
    ble112.ble_rsp_system_get_bt_address = my_rsp_system_get_bt_address;

    ble112.ble_evt_gatt_server_user_read_request = my_msg_gatt_server_user_read_request_evt_t;

    uint8_t tm = 0;
    Serialble.begin(9600);
    while (!Serialble && tm < 150) { // Wait for Serial to start Timeout 1.5s
        tm++;
        delay(10);
    }

    tm = 0;
    while (!bSystemBootBle && tm < 150) { // Waiting for BLE to start
        ble112.checkActivity(100);
        tm++;
        delay(10);
    }

    /* setting */
    /* [set Advertising Data] */
    uint8 ad_data[64] = {
        (2),                                                    // field length
        BGLIB_GAP_AD_TYPE_FLAGS,                                // field type 
        (6),
    };

    stLen = 3;

    ad_data[stLen] = (17);
    stLen++;
    ad_data[stLen] = 0x07;
    stLen++;
    ad_data[stLen] = 0xeb;
    stLen++;
    ad_data[stLen] = 0x53;
    stLen++;
    ad_data[stLen] = 0x2d;
    stLen++;
    ad_data[stLen] = 0x21;
    stLen++;
    ad_data[stLen] = 0xd4;
    stLen++;
    ad_data[stLen] = 0xe1;
    stLen++;
    ad_data[stLen] = 0xe1;
    stLen++;
    ad_data[stLen] = 0xcb;
    stLen++;
    ad_data[stLen] = 0x28;
    stLen++;
    ad_data[stLen] = 0x9a;
    stLen++;
    ad_data[stLen] = 0x00;
    stLen++;
    ad_data[stLen] = 0x8a;
    stLen++;
    ad_data[stLen] = 0x70;
    stLen++;
    ad_data[stLen] = 0x15;
    stLen++;
    ad_data[stLen] = 0x2f;
    stLen++;
    ad_data[stLen] = 0x44;
    stLen++;


    ad_data[stLen] = (6);
    stLen++;
    ad_data[stLen] = BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE;
    stLen++;

    ad_data[stLen] = 'n';
    stLen++;
    ad_data[stLen] = 'a';
    stLen++;
    ad_data[stLen] = 'n';
    stLen++;
    ad_data[stLen] = 'o';
    stLen++;
    ad_data[stLen] = 'R';
    stLen++;
    

    // ble112.ble_cmd_le_gap_bt5_set_adv_data(0,SCAN_RSP_ADVERTISING_PACKETS,
    // stLen, ad_data);
    ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, stLen, ad_data);

    while (ble112.checkActivity(1000))
        ; /* Receive check */
    delay(20);

    /* interval_min :   40ms( =   64 x 0.625ms ) */
    // ble112.ble_cmd_le_gap_bt5_set_adv_parameters( 0, 64, 1600, 7, 0 );/*
    // [BGLIB] <handle> <interval_min> <interval_max> <channel_map>
    // <report_scan>*/
    /* interval_max : 1000ms( = 1600 x 0.625ms ) */
    ble112.ble_cmd_le_gap_set_adv_parameters(
        64, 1600, 7); /* [BGLIB] <interval_min> <interval_max> <channel_map> */

    while (ble112.checkActivity(1000))
        ; /* [BGLIB] Receive check */

    /* start */
    //    ble112.ble_cmd_le_gap_bt5_set_mode(0,LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE,0,2);
    //    ble112.ble_cmd_le_gap_set_mode(LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE);
    //    ble112.ble_cmd_le_gap_start_advertising(0,
    //    LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);     //
    //    index = 0
    ble112.ble_cmd_le_gap_start_advertising(
        0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE); // index = 0
    while (ble112.checkActivity(1000))
        ; /* Receive check */
}

void setup() {
    //  delay(1000);

    Serial.begin(115200); // UART 115200bps
#ifdef SERIAL_MONITOR
    Serial.println(F(""));
    Serial.println(F("========================================="));
    Serial.println(F("setup start"));
#endif

    LowPower.begin();
    setupPort();
    setupBLE();

    ble112.ble_cmd_system_get_bt_address();
    while (ble112.checkActivity(1000))
        ;

    setupTimerInt(); // Timer inverval start

#ifdef SERIAL_MONITOR
    Serial.println(F(""));
    Serial.println("=========================================");
    Serial.println(F("loop start"));
    Serial.println(F(""));
#endif
}




// Loop counter
void loopCounter() {
    iLoop1s += 1;

    // 1s period
    if (iLoop1s >= 8) { // 125ms x 8 = 1s
        iLoop1s = 0;

        iSendCounter += 1;
        if (iSendCounter >= SEND_INTERVAL) {
            iSendCounter = 0;
            event1s = true;
        }
    }
}

// GATTのデータ送信
// 現状はHelloを送信しているだけ
void bt_sendData() {
    char sendData[40];
    uint8 sendLen;
    sendLen = sprintf(sendData, "Hello\n");
    // Send to BLE device
    ble112.ble_cmd_gatt_server_send_characteristic_notification(
        1, 0x000C, sendLen, (const uint8 *)sendData);
    while (ble112.checkActivity(1000))
        ;
}

void bt_read(){
  char sendData[40];
  uint8 sendLen;
  sendLen = sprintf(sendData, "read\n");
  ble112.ble_cmd_gatt_server_send_user_read_response(
    1, 0x000C, 0, sendLen, (const uint8 *)sendData);
  while (ble112.checkActivity(1000))
    ;
}

void loopBleRcv(void) {
    // keep polling for new data from BLE
    ble112.checkActivity(0); /* Receive check */

    if (ble_state == BLE_STATE_STANDBY) {
        bBLEconnect = false; /* [BLE] connection state */
    } else if (ble_state == BLE_STATE_ADVERTISING) {
        bBLEconnect = false; /* [BLE] connection state */
    } else if (ble_state == BLE_STATE_CONNECTED_SLAVE) {
        bBLEconnect = true; /* [BLE] connection state */
    }
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

// iBeaconのアドバタイズデータを設定
void StartiBeaconAdvData() {
    // UUIDをstrからbyteに変換
    uint8_t uuid_bytes[16];
    char* uuid_str = BEACON_UUID;
    if (lostChild) {
        uuid_str = LOST_BEACON_UUID;
    }
    uuidToBytes(uuid_str, uuid_bytes);

    // MajorとMinorを設定
    uint16_t major = ibeacon_major;
    uint16_t minor = ibeacon_minor;

    Serial.print("major: ");
    Serial.println(major);
    Serial.print("minor: ");
    Serial.println(minor);

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

        (-65)  // Measured Power
    };


    // Register advertising packet
    Serial.println(F("Start iBeacon"));
    uint8_t stLen = sizeof(adv_data);
    ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, stLen, adv_data);
    while (ble112.checkActivity(1000));
    // index = 0  LE_GAP_SCANNABLE_NON_CONNECTABLE / LE_GAP_UNDIRECTED_CONNECTABLE
    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_SCANNABLE_NON_CONNECTABLE);
    while (ble112.checkActivity(1000));
}

void sleepBLE() {
    ble112.ble_cmd_le_gap_stop_advertising(0);
    while (ble112.checkActivity());
    ble112.ble_cmd_system_halt(1);
    while (ble112.checkActivity());
    digitalWrite(BLE_WAKEUP, LOW);
    delay(500);
}

void wakeupBLE() {
    digitalWrite(BLE_WAKEUP, HIGH);
    delay(500);
    ble112.ble_cmd_system_halt(0);
    while (ble112.checkActivity());
    ble112.ble_cmd_le_gap_set_adv_parameters(400, 800, 7);
    while (ble112.checkActivity(1000));
}

void loop() {
    // Timer interval Loop once in 125ms
    if(ibeacon){
        StartiBeaconAdvData();

        Serial.println(F("Start advertise"));
        Serial.flush();

        // Continue Advertising (during that STM32 sleeps.)
        LowPower.deepSleep(WAKE_INTERVAL * 1000);
        Serial.println(F("Sleep BLE"));
        sleepBLE();
        Serial.println(F("Sleep STM32"));
        Serial.println(F(">>> Sleep >>>"));
        Serial.flush();
        LowPower.deepSleep(SLEEP_INTERVAL * 1000);
        Serial.println(F("Wakeup STM32"));
        wakeupBLE();
        Serial.println(F("Wakeup BLE"));
        Serial.println(F("<<< Wake up <<<"));
    }
    else{
        if (bInterval == true) {
            bInterval = false;

            loopCounter(); // loop counter
            // Run once in 1s
            if (event1s == true) {
                event1s = false;
                // bt_sendData(); // Data send
            }
        }
        loopBleRcv();
    }
}


// called when the module begins sending a command
void onBusy() {
  // Serial.println("onBusy");
    // turn LED on when we're busy
    // digitalWrite( D13_LED, HIGH );
}


// called when the module receives a complete response or "system_boot" event
void onIdle() {
  // Serial.println("onIdle");
    // turn LED off when we're no longer busy
    // digitalWrite( D13_LED, LOW );
}


// called when the parser does not read the expected response in the specified
// time limit
void onTimeout() {
  Serial.println("onTimeout");
    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
    /*  */
    bBLEconnect = false; /* [BLE] connection state */
    bBLEsendData = false;
#ifdef DEBUG
    Serial.println(F("on time out"));
#endif
}


// called immediately before beginning UART TX of a command
void onBeforeTXCommand() {
  Serial.println("onBeforeTXCommand");
}


// called immediately after finishing UART TX
void onTXCommandComplete() {
  Serial.println("onTXCommandComplete");
    // allow module to return to sleep (assuming here that digital pin 5 is
    // connected to the BLE wake-up pin)
#ifdef DEBUG
    Serial.println(F("onTXCommandComplete"));
#endif
}

// 通信を受信したときのやつ
void my_evt_gatt_server_attribute_value(
    const struct ble_msg_gatt_server_attribute_value_evt_t *msg) {
      Serial.println("my_evt_gatt_server_attribute_value");
    uint16 attribute = (uint16)msg->attribute;
    uint16 offset = 0;
    uint8 value_len = msg->value.len;
    uint8 value_data[20];
    String rcv_data;
    rcv_data = "";
    for (uint8_t i = 0; i < value_len; i++) {
        rcv_data += (char)(msg->value.data[i]);
    }

#ifdef DEBUG
    Serial.print(F("###\tgatt_server_attribute_value: { "));
    Serial.print(F("connection: "));
    Serial.print(msg->connection, HEX);
    Serial.print(F(", attribute: "));
    Serial.print((uint16_t)msg->attribute, HEX);
    Serial.print(F(", att_opcode: "));
    Serial.print(msg->att_opcode, HEX);

    Serial.print(", offset: ");
    Serial.print((uint16_t)msg->offset, HEX);
    Serial.print(", value_len: ");
    Serial.print(msg->value.len, HEX);
    Serial.print(", value_data: ");
    Serial.print(rcv_data);

    Serial.println(F(" }"));
#endif

    // "major,minor"の形式で受信したデータを分割
    int pos = rcv_data.indexOf(",");
    if (pos > 0) {
        String strMajor = rcv_data.substring(0, pos);
        String strMinor = rcv_data.substring(pos + 1);
        ibeacon_major = strMajor.toInt();
        ibeacon_minor = strMinor.toInt();
        ibeacon = true;
    }

    // (major, minor, "lost") で最後にlostがいるか
    if (rcv_data == "lost") {
        ibeacon = false;
        lostChild = true;
    }

    if(ibeacon) {
      ble112.ble_cmd_le_connection_close(1);
      while (ble112.checkActivity(1000));
    }
}

void my_evt_le_connection_opend(const ble_msg_le_connection_opend_evt_t *msg) {
  Serial.println("my_evt_le_connection_opend");
#ifdef DEBUG
    Serial.print(F("###\tconnection_opend: { "));
    Serial.print(F("address: "));
    // this is a "bd_addr" data type, which is a 6-byte uint8_t array
    for (uint8_t i = 0; i < 6; i++) {
        if (msg->address.addr[i] < 16)
            Serial.write('0');
        Serial.print(msg->address.addr[i], HEX);
    }
    Serial.print(", address_type: ");
    Serial.print(msg->address_type, HEX);
    Serial.print(", master: ");
    Serial.print(msg->master, HEX);
    Serial.print(", connection: ");
    Serial.print(msg->connection, HEX);
    Serial.print(", bonding: ");
    Serial.print(msg->bonding, HEX);
    Serial.print(", advertiser: ");
    Serial.print(msg->advertiser, HEX);
    Serial.println(" }");
#endif
    ble_state = BLE_STATE_CONNECTED_SLAVE;

    uint8 write_data[10];

    generateRandomString(write_data, 10);

    // uidを10桁にし、charに変換ののち、write_dataに格納

    Serial.print("write_data: ");
    for (int i = 0; i < 10; i++) {
        Serial.print(write_data[i]);
    }
    Serial.print("\n");


    Serial.println("ble_cmd_gatt_server_write_attribute_value");
    ble112.ble_cmd_gatt_server_write_attribute_value(0x000C, 0, 10, write_data);
    while (ble112.checkActivity(1000));
}

void my_evt_le_connection_closed(
    const struct ble_msg_le_connection_closed_evt_t *msg) {
      Serial.println("my_evt_le_connection_closed");
#ifdef DEBUG
    Serial.print(F("###\tconnection_closed: { "));
    Serial.print(F("reason: "));
    Serial.print((uint16_t)msg->reason, HEX);
    Serial.print(F(", connection: "));
    Serial.print(msg->connection, HEX);
    Serial.println(F(" }"));
#endif

    ble112.ble_cmd_le_gap_set_mode(LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE);

    while (ble112.checkActivity(1000))
        ;

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
    /*  */
    bBLEconnect = false; /* [BLE] connection state */
    bBLEsendData = false;

    // 迷子モードがオンの時、iBeacon発信開始
    if (lostChild) {
        ibeacon = true;
    }
}

void my_evt_system_boot(const ble_msg_system_boot_evt_t *msg) {
  Serial.println("my_evt_system_boot");
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
    Serial.print(msg->bootloader, HEX); /*  */
    Serial.print(", hw: ");
    Serial.print(msg->hw, HEX);
    Serial.println(" }");
#endif

    bSystemBootBle = true;

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;
}


void my_evt_system_awake(void) {
  Serial.println("my_evt_system_awake");
    ble112.ble_cmd_system_halt(0);
    while (ble112.checkActivity(1000))
        ;
}

void my_rsp_system_get_bt_address(
    const struct ble_msg_system_get_bt_address_rsp_t *msg) {
      Serial.println("my_rsp_system_get_bt_address");
#ifdef DEBUG
    Serial.print("###\tsystem_get_bt_address: { ");
    Serial.print("address: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(msg->address.addr[i], HEX);
    }
    Serial.println(" }");
#endif

#ifdef SERIAL_MONITOR
    unsigned short addr = 0;
    char cAddr[30];
    addr = msg->address.addr[0] + (msg->address.addr[1] * 0x100);
    sprintf(cAddr, "Device name is Leaf_A_#%05d ", addr);
    Serial.println(cAddr);
#endif
}

void my_msg_gatt_server_user_read_request_evt_t(
  const ble_msg_gatt_server_user_read_request_evt_t *msg) {
  Serial.println("my_ble_msg_gatt_server_user_read_request_evt_t");
}