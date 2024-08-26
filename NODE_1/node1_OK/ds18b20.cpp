#include <Rak3172_Canopus.h>
#include <Arduino.h>
#include "ds18b20.h"
#define ONEWIRE_PIN_DATA GPIO_PIN_12
#define ONEWIRE_PORT_DATA GPIOA
void set_pin_as_output(GPIO_TypeDef *port, uint16_t pin) {
  // Cấu hình chân GPIO thành chế độ xuất (output).
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}
void set_pin_as_input(GPIO_TypeDef *port, uint16_t pin) {
    // Cấu hình chân GPIO thành chế độ nhập (input).
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}
void onewire_reset(void) {
    // Đặt lại cảm biến DS18B20.
    set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);  // Cấu hình chân dữ liệu thành chế độ xuất.
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);  // Kéo chân dữ liệu xuống mức thấp.
    delayMicroseconds(480);  // Chờ 480 microseconds.
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);  // Kéo chân dữ liệu lên mức cao.
    delayMicroseconds(480);  // Chờ thêm 480 microseconds.
    set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);  // Cấu hình chân dữ liệu thành chế độ nhập.
    delayMicroseconds(480);  // Chờ thêm 480 microseconds.
}
void onewire_write(uint8_t data) {
  set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
  for(uint8_t i=0;i<8;i++)
  {
    if((data&(1<<i))!=0){
        set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
        HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
        delayMicroseconds(1);
        set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
        delayMicroseconds(60);
    }else{
        set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
        HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
        delayMicroseconds(60);
        set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
    }
  }
}
uint8_t onewire_read(void) {
  
	uint8_t value=0;
	set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);

	for (int i=0;i<8;i++)
	{
		set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);   // set as output

		HAL_GPIO_WritePin (ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);  // pull the data pin LOW
		delayMicroseconds(2);  // wait for 2 us

		set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);  // set as input
		if (HAL_GPIO_ReadPin (ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delayMicroseconds(60);  // wait for 60 us
	}
	return value;
}
float read_temperature(void) {
    uint8_t temp_LSB, temp_MSB;
    int16_t temp;

    // Khởi động lại cảm biến
    onewire_reset();
    delayMicroseconds(1);
    // Gửi lệnh Skip ROM
    onewire_write(0xCC);

    // Gửi lệnh Convert T
    onewire_write(0x44);

    // Chờ cảm biến hoàn thành quá trình chuyển đổi nhiệt độ
    delayMicroseconds(750);

    // Khởi động lại cảm biến
    onewire_reset();
    delayMicroseconds(1);
    // Gửi lệnh Skip ROM
    onewire_write(0xCC);
    // Gửi lệnh Read Scratchpad
    onewire_write(0xBE);
    // Đọc 2 byte dữ liệu nhiệt độ
    temp_LSB = onewire_read();
    temp_MSB = onewire_read();
    // Chuyển đổi dữ liệu nhiệt độ
    temp = ((uint16_t)temp_MSB << 8) | temp_LSB;
    // Trả về giá trị nhiệt độ dạng float
    
    return (float)temp / 16.0;
}