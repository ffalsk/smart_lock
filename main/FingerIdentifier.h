#ifndef _FINGERIDENTIFIER_h
#define _FINGERIDENTIFIER_h

#include "main.h"
#define Finger_Bufferlength 100
#define Finger_RxPacklength 100
#define Finger_password 0x00000000 // 0x10403020
enum
{
	OFF,
	ON,
	Auto,
	PWM,
	Blink
};
enum
{
	No_color,
	G,
	R,
	RG,
	B,
	RB,
	GB
};
// Modified From <WinSock2.h>
#define SWAP_SHORT(l)        \
	((((l) >> 8) & 0x00FF) | \
	 (((l) << 8) & 0xFF00))
#define SWAP_LONG(l)               \
	((((l) >> 24) & 0x000000FFL) | \
	 (((l) >> 8) & 0x0000FF00L) |  \
	 (((l) << 8) & 0x00FF0000L) |  \
	 (((l) << 24) & 0xFF000000L))
#define SWAP_LONGLONG(l)                    \
	((((l) >> 56) & 0x00000000000000FFLL) | \
	 (((l) >> 40) & 0x000000000000FF00LL) | \
	 (((l) >> 24) & 0x0000000000FF0000LL) | \
	 (((l) >> 8) & 0x00000000FF000000LL) |  \
	 (((l) << 8) & 0x000000FF00000000LL) |  \
	 (((l) << 24) & 0x0000FF0000000000LL) | \
	 (((l) << 40) & 0x00FF000000000000LL) | \
	 (((l) << 56) & 0xFF00000000000000LL))
__inline uint64_t htonll(uint64_t Value)
{
	const uint64_t Retval = SWAP_LONGLONG(Value);
	return Retval;
}
__inline uint32_t htonl(uint32_t Value)
{
	const uint32_t Retval = SWAP_LONG(Value);
	return Retval;
}
__inline uint16_t htons(uint16_t Value)
{
	const uint16_t Retval = SWAP_SHORT(Value);
	return Retval;
}
// 错误类型
typedef enum
{
	no_error,
	header_error,
	data_error,
	password_error,
	internal_error,
	cmd_error,
} Error_e;
class FPM383D
{
public:
#pragma pack(1)
	// 发送数据结构体
	typedef struct
	{
		uint64_t header;
		uint16_t data_len;
		uint8_t header_checksum;
		uint32_t password;
		uint16_t CMD;
	} Send_s;
#pragma pack()
	struct
	{
		uint16_t score;
		uint16_t ID;
	} Finger_INFO;
	bool ScanState = false;
	void ISR_Resume(void);
	void begin(uint8_t TX_PIN, uint8_t RX_PIN, uart_port_t Finger_uart_in, gpio_num_t Interrupt_pin);
	/**
	 * @brief 自动注册指纹
	 * @param ID 指纹 ID，通常是一个唯一的标识符
	 * @param wait 设置为 1 则表示按压后需要等待手指抬起再次按压才可以进行下一次注册，设置为 0 则不需要检测手指抬起（默认为 0x01）
	 * @param press_num 需要按压的次数（默认为 0x06）
	 * @return uint8_t 操作结果代码，通常是成功或失败
	 */
	uint8_t AutoEnroll(uint16_t ID, uint8_t wait = 0x01, uint8_t press_num = 0x06);
	/**
	 * @brief 设置系统策略并发送到FPM383D设备
	 * @param check_always 是否启用重复指纹检查策略（true表示启用，false表示禁用）
	 * @param auto_stuty 是否启用自学习功能策略（true表示启用，false表示禁用）
	 * @param _360Identify 是否启用360度识别策略（true表示启用，false表示禁用）
	 * @return uint8_t 表示设置策略的结果
	 *                - 返回值为0表示成功设置策略并发送到设备
	 *                - 非0值表示设置策略失败
	 */
	uint8_t SetSystemPolicy(bool check_always, bool auto_stuty, bool _360Identify);
	/**
	 * @brief 设置新的设备密码并发送给设备。
	 * @param New_password 新的密码，为32位整数
	 * @return 设置密码后的响应值
	 */
	uint8_t SetPassword(uint32_t New_password);
	/**
	 * 设置设备的休眠模式，并发送给设备。
	 * @param sleep_mode 休眠模式，00 表示进入普通休眠模式,01 表示进入深度休眠模式，默认为0x00
	 * @return 设置休眠模式后的响应值
	 */
	uint8_t SetSleepMode(uint8_t sleep_mode = 0x00);
	uint8_t Identify();
	/**
	 * @brief 从FPM383D指纹模块中删除一个或多个指纹ID。
	 * @param clean_flag 0x01 表示清除所有指纹，0x02 表示清除多条指纹，
	 * 0x03 表示清除一块指纹，默认 0x00 表示清除单个指纹。
	 * @param count 要删除的指纹ID的数量。
	 * @param ... 要删除的指纹ID列表 uint16_t
	 * @return uint8_t 删除操作的结果或状态码。
	 */
	uint8_t Delete(uint8_t clean_flag = 0x01, uint16_t count = 1, ...);
	/**
	 * @brief 设置FPM383D指纹模块的LED灯的控制参数。
	 * @param control_mode LED灯的控制模式 0：关闭LED 灯 1：开启LED 灯 2：当手指触碰时自动点亮LED 灯
	 * 3：PWM 控制 LED 灯(呼吸灯) 4：闪烁LED 灯
	 * @param color LED灯的颜色 0：无颜色控制 1：绿色 2：红色 3：红色+绿色 4：蓝色 5：红色+蓝色 6：绿色+蓝色
	 * @PWM模式
	 * @param arg1 最大占空比(范围：0~100)
	 * @param arg2 最小占空比(范围：0~100)
	 * @param arg3 占空比每秒变化速率(单位：1%/s，范围：0~100）
	 * @闪烁模式
	 * @param arg1 LED 点亮时长（单位：10ms）
	 * @param arg2 LED 熄灭时长（单位：10ms）
	 * @param arg3 闪烁周期数量
	 * @return uint8_t 设置操作的结果或状态码。
	 */
	uint8_t SetLED(uint8_t control_mode, uint8_t color, uint8_t arg1 = 0, uint8_t arg2 = 0, uint8_t arg3 = 0);
	QueueHandle_t uart_queue;
	uint8_t RXdata[Finger_Bufferlength];
	uint8_t Finger_uart;

private:
	// 校验函数
	uint8_t protocol_get_checksum(uint8_t *data, uint8_t length, uint8_t add_data_len = 0, uint8_t *add_data = nullptr)
	{
		uint8_t i = 0;
		int8_t sum = 0;
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
		return (uint8_t)((~sum) + 1);
	}
	/**
	 * @brief 向指纹模块发送数据包
	 * @param CMD 发送的命令码
	 * @param add_data_len 附加数据的长度
	 * @param add_data 指向附加数据的指针
	 * @return void
	 */
	void SendData(uint16_t CMD, uint8_t add_data_len = 0, uint8_t *add_data = nullptr);
	/**
	 * @brief 从指纹模块接收数据
	 * @param Timeout 超时时间 ms
	 * @param header_checksum 期望的帧头校验和
	 * @param CMD 期望的命令码
	 * @return 错误码
	 */
	uint8_t ReceiveData(uint16_t Timeout, uint8_t header_checksum, uint16_t CMD);
	/**
	 * @brief 错误码解码
	 * @return void
	 */
	void ErrorAnalyzer();
	uint32_t password = Finger_password;
	uint8_t FPM383D_Receive_Pack[Finger_RxPacklength] = {0};
	uint32_t Error_INFO = 0;

	gpio_num_t Finger_Interrupt_pin;
};

#endif
