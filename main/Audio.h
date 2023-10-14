#ifndef _AUDIO_h
#define _AUDIO_h
#include "main.h"

enum
{
	correct,
	wrong,
	forbidden
};
class DY_SV17F
{
public:
#pragma pack(1)
	// 发送数据结构体
	typedef struct
	{
		uint8_t header;
		uint8_t CMD;
		uint8_t data_len;
	} Send_s;
#pragma pack()
	void begin(uint8_t TX_PIN, uint8_t RX_PIN, uart_port_t Audio_uart);
	void SetSoundPlay(uint16_t id);
	void Play();
	void Stop();
	/**
	 * @brief 设置DY_SV17F指纹模块的音量。
	 * @param volume 音量值，应在0到31之间。
	 */
	void SetVolume(uint8_t volume);

private:
	uint8_t protocol_get_checksum(uint8_t *data, uint8_t length, uint8_t add_data_len = 0, uint8_t *add_data = nullptr)
	{
		uint8_t i = 0;
		uint32_t sum = 0;
		for (i = 0; i < length; i++)
		{
			sum += data[i];
		}
		if (add_data_len > 0)
		{
			for (i = 0; i < add_data_len; i++)
			{
				sum += add_data[i];
			}
		}
		return (uint8_t)sum;
	}
	/**
	 * @brief 发送数据到DY_SV17F设备
	 * @param CMD 8位命令字节，表示要执行的命令
	 * @param add_data_len 8位整数，表示附加数据的长度，默认为0
	 * @param add_data 指向8位无符号整数数组的指针，表示附加数据，默认为nullptr
	 */
	void SendData(uint8_t CMD, uint8_t add_data_len, uint8_t *add_data, bool);
	// uint8_t ReceiveData(uint16_t Timeout, uint8_t header_checksum, uint16_t CMD);
	uart_port_t Audio_uart;
};

#endif