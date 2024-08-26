#include <Arduino.h>
#include <Rak3172_Canopus.h>
#include "Canopus_Modbus.h"

ModbusMaster node;

uint8_t result;
long startTime;
bool rx_done = false;
double myFreq = 922500000;
uint16_t sf = 12, bw = 0, cr = 0, preamble = 8, txPower = 22;
unsigned long lastMillis = 0;

//____________________ADAM___________________-
uint8_t result_analog;
uint8_t result_digital_in;
uint8_t status;
uint16_t rawValue;
uint8_t batt_int;
float current;
uint16_t inputStatus;
uint8_t adamOutputStatus = 0x00;  // Trạng thái hiện tại của các chân output của ADAM
uint8_t check_adamData;

// Khai báo biến global
uint8_t frame[16];
uint8_t len;
uint8_t temp_data[2];
char buff[16];
uint8_t count_sendFrame = 1;
uint8_t status_adamControl = 0x00;
float battery;
// Biến trạng thái
bool join_sent = false;      // Biến trạng thái để theo dõi việc gửi function 0x01
bool join_accepted = false;  // Biến trạng thái để theo dõi việc nhận phản hồi từ gateway
bool check_response = false;
// Địa chỉ MAC của node
uint8_t MAC[6] = { 0x0A, 0x0B, 0x0C, 0x0D, 0x11, 0x22 };  // Thay bằng địa chỉ MAC thực tế của bạn

//________________________________OneWire - DS18B20 __________________________
// Định nghĩa các chân và các hàm hỗ trợ
#define ONEWIRE_PIN_DATA GPIO_PIN_11
#define ONEWIRE_PORT_DATA GPIOA

void set_pin_as_output(GPIO_TypeDef *port, uint16_t pin) {
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void set_pin_as_input(GPIO_TypeDef *port, uint16_t pin) {
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void onewire_reset(void) {
  set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
  HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
  delayMicroseconds(480);
  HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
  delayMicroseconds(480);
  set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
  delayMicroseconds(480);
}

void onewire_write_bit(uint8_t bit) {
  set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
  if (bit) {
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
    delayMicroseconds(1);
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
    delayMicroseconds(60);
  } else {
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
    delayMicroseconds(60);
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
    delayMicroseconds(1);
  }
  set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
}

uint8_t onewire_read_bit(void) {
  uint8_t bit = 0;
  set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
  HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
  delayMicroseconds(1);
  HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
  set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
  delayMicroseconds(15);
  bit = HAL_GPIO_ReadPin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
  delayMicroseconds(45);
  return bit;
}

void onewire_write_byte(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    onewire_write_bit(data & 0x01);
    data >>= 1;
  }
}

uint8_t onewire_read_byte(void) {
  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    data |= (onewire_read_bit() << i);
  }
  return data;
}

float read_temperature(void) {
  uint8_t temp_LSB, temp_MSB;
  int16_t temp;

  onewire_reset();
  onewire_write_byte(0xCC);
  onewire_write_byte(0x44);
  delay(750);
  onewire_reset();
  onewire_write_byte(0xCC);
  onewire_write_byte(0xBE);
  temp_LSB = onewire_read_byte();
  temp_MSB = onewire_read_byte();
  temp = ((int16_t)temp_MSB << 8) | temp_LSB;
  return (float)temp / 16.0;
}

void hexDump(uint8_t *buf, uint16_t len) {
  for (uint16_t i = 0; i < len; i += 16) {
    char s[len];
    uint8_t iy = 0;
    for (uint8_t j = 0; j < 16; j++) {
      if (i + j < len) {
        uint8_t c = buf[i + j];
        if (c > 31 && c < 128)
          s[iy++] = c;
      }
    }
    String msg = String(s);
    Serial.println(msg);
  }
  Serial.println("Buffer!");
}

void recv_cb(rui_lora_p2p_recv_t data) {
  rx_done = true;
  if (data.BufferSize == 0) {
    Serial.println("Empty buffer.");
    return;
  }
  sprintf(buff, "Incoming message, length: %d, RSSI: %d, SNR: %d",
          data.BufferSize, data.Rssi, data.Snr);
  Serial.println(buff);

  // Hiển thị dữ liệu nhận được dưới dạng hex
  hexDump(data.Buffer, data.BufferSize);
  digitalWrite(LED_RECV, !digitalRead(LED_RECV));
  uint8_t frame_typeControl = data.Buffer[2];
  uint8_t function = data.Buffer[3];
  uint8_t control_state = data.Buffer[10];
  uint8_t dataControl[1] = { status };
  uint8_t result_DO;
  // Kiểm tra địa chỉ MAC
  bool macMatch = true;
  for (int i = 0; i < 6; i++) {
    if (data.Buffer[4 + i] != MAC[i]) {  //Kiểm tra địa chỉ Mac của buffer vừa nhận được , Nếu khác buffer thì break.
      macMatch = false;
      break;
    }
  }
  if (macMatch && frame_typeControl == 0x01 && function == 0x06) {
    if (control_state == 0x01) {
      digitalWrite(LED_SYNC, HIGH);
      Serial.println("On LED");
    } else if (control_state == 0x00) {
      digitalWrite(LED_SYNC, LOW);
      Serial.println("Off LED");
    }
  }
  if (macMatch && frame_typeControl == 0x01 && function == 0x05) {
    if (control_state == 0x01) {
      result_DO = node.writeSingleCoil(0x0000, 1);
      Serial.println("Control ADAM ON");
      status_adamControl = !status_adamControl;

    } else if (control_state == 0x00) {
      result_DO = node.writeSingleCoil(0x0000, 0);
      Serial.println("Control ADAM OFF");
      status_adamControl = !status_adamControl;
    }
  }
}

void send_cb(void) {
  Serial.printf("P2P set Rx mode %s\r\n",
                api.lora.precv(65534) ? "Success" : "Fail");
}

void send_frame(uint8_t frame_type, uint8_t function, uint8_t *data, uint8_t data_len) {
  len = 0;
  frame[len++] = 0xFE;
  frame[len++] = 0;
  frame[len++] = frame_type;
  frame[len++] = function;
  memcpy(&frame[len], MAC, 6);
  len += 6;
  memcpy(&frame[len], data, data_len);
  len += data_len;

  frame[1] = len - 2;
  Serial.println();
  Serial.print("Len of frame: ");
  Serial.println(frame[1], HEX);

  uint8_t crc = crc8(&frame[1], len - 1);
  frame[len++] = crc;
  frame[len++] = 0xEF;

  bool send_result = false;
  uint8_t retry_count = 0;
  const uint8_t max_retries = 5;

  while (!send_result && retry_count < max_retries) {
    send_result = api.lora.psend(len, frame);
    Serial.printf("P2P send %s\r\n", send_result ? "Success" : "Fail");
    if (!send_result) {
      Serial.printf("P2P finish Rx mode %s\r\n", api.lora.precv(0) ? "Success" : "Fail");
    }
    retry_count++;
    Serial.print("Check resend: ");
    Serial.println(retry_count);
  }

  if (send_result) {
    Serial.printf("P2P send Success\r\n");
    digitalWrite(LED_SEND, !digitalRead(LED_SEND));
  }

  Serial.print("Sending frame (Serial): ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void read_ADAM() {
  result_analog = node.readHoldingRegisters(0, 1);
  delay(10);
  if (result_analog == node.ku8MBSuccess) {
    rawValue = node.getResponseBuffer(0);
    current = (rawValue * 20.0) / 4095.0;
    Serial.printf("\r\nValue 4000%d: %d -> Current: %.2f mA", rawValue, current);
  } else {
    Serial.printf("\nRead Fail node 1");
  }

  result_digital_in = node.readDiscreteInputs(0x00, 2);
  delay(10);
  if (result_digital_in == node.ku8MBSuccess) {
    inputStatus = node.getResponseBuffer(0);
    Serial.printf("\nReceived Data: 0x%04X", inputStatus);

    for (uint8_t i = 0; i < 2; i++) {
      bool state = (inputStatus >> i) & 0x01;
      Serial.printf("\nDI_%02d: %s", i + 1, state ? "ON" : "OFF");
    }
  } else {
    Serial.printf("\nRead Fail, Error: %02X", result_digital_in);
  }
}

uint8_t crc8(const uint8_t *data, int len) {
  unsigned crc = 0;
  int i, j;
  for (j = 0; j < len; j++, data++) {
    crc ^= (*data << 8);
    for (i = 8; i > 0; --i) {
      if (crc & 0x8000)
        crc ^= (0x1070 << 3);
      crc <<= 1;
    }
  }
  return (uint8_t)(crc >> 8);
}

String decToHex(byte dec) {
  char hexString[3];
  sprintf(hexString, "%02X", dec);
  return String(hexString);
}

void sleep_mode() {
  Serial.println("_________________  Sleeping ____________________-");
  api.system.sleep.all();
  api.system.scheduler.task.destroy();
}

void setup() {
  Serial.begin(115200);
  Serial.println("_______RAK3172_Canopus lora P2P");
  Serial.println("------------------------------------------------------");
  Serial.println("_______RAKwireless System Powersave");
  Serial.println("------------------------------------------------------");
  delay(2000);
  init_io();
  enable_Vss5();
  Serial_Canopus.begin(9600, SERIAL_8N1);
  enable_Vrs485();
  analogReadResolution(12);
  pinMode(LED_SYNC, OUTPUT);
  int ii;
  for (ii = 0; ii <= 10; ii++) {
    digitalWrite(LED_SYNC, HIGH);
    delay(100);
    digitalWrite(LED_SYNC, LOW);
    delay(100);
  }
  digitalWrite(LED_SYNC, LOW);
  startTime = millis();

  if (api.lora.nwm.get() != 0) {
    Serial.printf("Set Node device work mode %s\r\n",
                  api.lora.nwm.set() ? "Success" : "Fail");
    api.system.reboot();
  }
  Serial.println("P2P Start");
  Serial.printf("Hardware ID: %s\r\n", api.system.chipId.get().c_str());
  Serial.printf("Model ID: %s\r\n", api.system.modelId.get().c_str());
  Serial.printf("RUI API Version: %s\r\n",
                api.system.apiVersion.get().c_str());
  Serial.printf("Firmware Version: %s\r\n",
                api.system.firmwareVersion.get().c_str());
  Serial.printf("AT Command Version: %s\r\n",
                api.system.cliVersion.get().c_str());
  Serial.printf("Set P2P mode frequency %3.3f: %s\r\n", (myFreq / 1e6),
                api.lora.pfreq.set(myFreq) ? "Success" : "Fail");
  Serial.printf("Set P2P mode spreading factor %d: %s\r\n", sf,
                api.lora.psf.set(sf) ? "Success" : "Fail");
  Serial.printf("Set P2P mode bandwidth %d: %s\r\n", bw,
                api.lora.pbw.set(bw) ? "Success" : "Fail");
  Serial.printf("Set P2P mode code rate 4/%d: %s\r\n", (cr + 5),
                api.lora.pcr.set(cr) ? "Success" : "Fail");
  Serial.printf("Set P2P mode preamble length %d: %s\r\n", preamble,
                api.lora.ppl.set(preamble) ? "Success" : "Fail");
  Serial.printf("Set P2P mode tx power %d: %s\r\n", txPower,
                api.lora.ptp.set(txPower) ? "Success" : "Fail");
  api.lora.registerPRecvCallback(recv_cb);
  api.lora.registerPSendCallback(send_cb);
  Serial.printf("P2P set Rx mode %s\r\n",
                api.lora.precv(65534) ? "Success" : "Fail");

  /*
  // Cấu hình timer cho frameLed
    if (api.system.timer.create(RAK_TIMER_1, (RAK_TIMER_HANDLER)frameLedHandler, RAK_TIMER_PERIODIC) != true) {
        Serial.println("Creating timer for frameLed failed.");
    } else if (api.system.timer.start(RAK_TIMER_1, 1000, (void *)1) != true) { // Gọi mỗi 1 giây
        Serial.println("Starting timer for frameLed failed.");
    }
*/
  if (!join_sent) {
    uint8_t join_data[1] = { 0x02 };
    send_frame(0x01, 0x01, join_data, 1);
    join_sent = true;
    Serial.println("_____________________________________");
    Serial.println("__________sent join_data to Gateway____________");
  }
}

void frameOnewire() {
  //________________Function 04: Onewire________________________
  float temperature = read_temperature();
  Serial.print("Temperature: ");
  Serial.print(temperature, 1);
  Serial.println(" *C");
  int16_t temp_scaled = (temperature * 10);
  uint8_t temp_data[2];
  temp_data[0] = temp_scaled & 0xFF;
  temp_data[1] = (temp_scaled >> 8) & 0xFF;
  send_frame(0x03, 0x04, temp_data, 2);
}

void frameAdam() {
  //_______________Function 05: Adam_____________________________
  read_ADAM();
  int16_t current_scaled = current * 10;
  uint8_t adam_data[4];
  uint8_t control_state = buff[10];
  adam_data[0] = status_adamControl ;
  adam_data[1] = (inputStatus >> 1) & 0x01;
  adam_data[2] = current_scaled & 0xFF;
  adam_data[3] = (current_scaled >> 8) & 0xFF;
  send_frame(0x03, 0x05, adam_data, 4);
}

//Ngắt - Interrupt cho việc điều khiển led
void frameLedHandler(void *data) {
  Serial.printf("[%lu] This is the handler of frameLed\r\n", millis());
  //frameLed();
}

void frameBattery() {
  float raw_battery_voltage = api.system.bat.get();
  battery = (raw_battery_voltage / 4.2) * 100;
  if (battery > 100) battery = 100; // Ensure it doesn't exceed 100%
  batt_int = battery;
  Serial.printf("Battery Voltage: %.2f V, Battery: %d%% ", raw_battery_voltage, batt_int);
  send_frame(0x03, 0x07, &batt_int, 1);
}
void loop() {
  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;
  const unsigned long interval = 31000;  // Khoảng thời gian giữa các lần gửi

  node.begin(1, Serial_Canopus);
  if (join_sent && !join_accepted && rx_done && !check_response) {
    check_response = true;
    rx_done = false;
    uint8_t response = buff[10];
    if (response == 0x00 || response == 0x01) {
      join_accepted = true;
      Serial.println("___________________JOIN SUCCESS__________________");
    } else {
      Serial.println("Join request failed. Gateway don't response ");
    }
  }

  if (join_accepted && (currentMillis - previousMillis >= interval)) {
    previousMillis = currentMillis;
    Serial.println("_____________________________________");
  float raw_battery_voltage = api.system.bat.get();
  battery = (raw_battery_voltage / 4.2) * 100;
  if (battery > 100) battery = 100; // Ensure it doesn't exceed 100%
  batt_int = battery;
  Serial.printf("Battery Voltage: %.2f V, Battery: %d%% ", raw_battery_voltage, batt_int);
    Serial.println("_____________________________________");
    if (join_accepted) {
      switch (count_sendFrame) {
        case 1:
          frameOnewire();
          break;
        case 2:
          frameAdam();
          break;
        case 3:
          frameBattery();
          break;
      }
      if (count_sendFrame >= 3) {
        count_sendFrame = 0;
      }
      count_sendFrame++;
      Serial.print("count_sendFrame: ");
      Serial.println(count_sendFrame);
      //sleep_mode();
    }
  }
}
