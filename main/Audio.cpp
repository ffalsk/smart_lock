/********************************************************************
 * \file   Audio.cpp
 * \brief  音频播放相关函数
 *
 * \author HDC
 * \date   September 2023
 *********************************************************************/
#include "Audio.h"
void DY_SV17F::begin(uint8_t TX_PIN, uint8_t RX_PIN, uart_port_t Audio_uart_in)
{
	Audio_uart = Audio_uart_in;
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 0,
		.source_clk = UART_SCLK_DEFAULT,
	};
	// Configure UART parameters
	ESP_ERROR_CHECK(uart_param_config(Audio_uart, &uart_config));
	// Set UART pins
	ESP_ERROR_CHECK(uart_set_pin(Audio_uart, TX_PIN, RX_PIN, -1, -1));
	// Install UART driver
	ESP_ERROR_CHECK(uart_driver_install(Audio_uart, 256, 256, 0, NULL, 0));
}
/**
 * @brief 发送数据到DY_SV17F设备
 * @param CMD 8位命令字节，表示要执行的命令
 * @param add_data_len 8位整数，表示附加数据的长度，默认为0
 * @param add_data 指向8位无符号整数数组的指针，表示附加数据，默认为nullptr
 */
void DY_SV17F::SendData(uint8_t CMD, uint8_t add_data_len = 0, uint8_t *add_data = nullptr, bool need_len = true)
{
	Send_s send_buffer;
	send_buffer.header = 0xAA; // 固定帧头
	send_buffer.CMD = CMD;
	send_buffer.data_len = add_data_len; // 有指令不遵守协议
	uint8_t data_checksum = protocol_get_checksum(reinterpret_cast<uint8_t *>(&send_buffer), 2 + need_len, add_data_len, add_data);
	uart_write_bytes(Audio_uart, reinterpret_cast<uint8_t *>(&send_buffer), sizeof(send_buffer) - 1 + need_len);
	// printf(reinterpret_cast<uint8_t*>(&send_buffer), sizeof(send_buffer) - 1 + need_len);
	if (add_data_len > 0)
		uart_write_bytes(Audio_uart, add_data, add_data_len); // printf(add_data, add_data_len);
	uart_write_bytes(Audio_uart, &data_checksum, 1);		  // printf(&data_checksum, 1);
}
/**
 * @brief 设置DY_SV17F设备播放特定音频文件
 * @param id 16位整数，表示要播放的音频文件的ID
 */
void DY_SV17F::SetSoundPlay(uint16_t id)
{
	uint8_t length = 11;
	std::stringstream ss;
	ss << std::setw(5) << std::setfill('0') << id; // 设置宽度为5，用 '0' 填充
	uint8_t char_ID[5]{};
	ss.read((char *)char_ID, 5);
	uint8_t buffer[] = {
		length,
		0x02,
		0x2F,
		char_ID[0],
		char_ID[1],
		char_ID[2],
		char_ID[3],
		char_ID[4],
		0x2A,
		0x3F,
		0x3F,
		0x3F,
	};
	SendData(0x08, 12, buffer, false);
}
void DY_SV17F::Play()
{
	SendData(0x02);
}
void DY_SV17F::Stop()
{
	SendData(0x04);
}
// 十进制数转换为两位的十六进制数，以 uint8_t 返回
inline uint8_t DecimalToHex(uint8_t decimalNumber)
{
	return ((decimalNumber / 16) << 4) | (decimalNumber % 16);
}
/**
 * @brief 设置DY_SV17F指纹模块的音量。
 * @param volume 音量值，应在0到31之间。
 */
void DY_SV17F::SetVolume(uint8_t volume)
{
	uint8_t buffer = DecimalToHex(volume);
	SendData(0x13, 1, &buffer);
}