/******************************************************************
 * \file FingerIdentifier.cpp
 * \brief 指纹模块相关处理函数
 *
 * \author HDC
 * \date September 2023
 ********************************************************************/

#include "FingerIdentifier.h"
void uart_event_task(void *pvParameters);
// Setup UART buffered IO with event queue

static void Finger_ISR_Handler(void *arg);
void FPM383D::begin(uint8_t TX_PIN, uint8_t RX_PIN, uart_port_t Finger_uart_in, gpio_num_t Interrupt_pin)
{
	Finger_uart = Finger_uart_in;
	Finger_Interrupt_pin = Interrupt_pin;
	uart_config_t uart_config = {
		.baud_rate = 57600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 64,
		.source_clk = UART_SCLK_DEFAULT,
	};
	// Configure UART parameters
	ESP_ERROR_CHECK(uart_param_config(Finger_uart, &uart_config));
	// Set UART pins
	ESP_ERROR_CHECK(uart_set_pin(Finger_uart, TX_PIN, RX_PIN, -1, -1));

	// Install UART driver using an event queue here
	ESP_ERROR_CHECK(uart_driver_install(Finger_uart, 256,
										256, 10, &uart_queue, 0));
	// 创建串口事件处理任务
	xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 3, NULL);
	// // 中断设置
	// // 配置GPIO引脚
	// gpio_config_t io_conf;
	// io_conf.intr_type = GPIO_INTR_POSEDGE; // 上升沿触发中断
	// io_conf.pin_bit_mask = (1ULL << Finger_Interrupt_pin);
	// io_conf.mode = GPIO_MODE_INPUT;
	// io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
	// io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	// ESP_ERROR_CHECK(gpio_config(&io_conf));
	// // 安装GPIO中断处理程序
	// gpio_install_isr_service(0); // 初始化GPIO中断服务
	// gpio_isr_handler_add(Finger_Interrupt_pin, Finger_ISR_Handler, (void *)Finger_Interrupt_pin);
}
/**
 * @brief 向指纹模块发送数据包
 * @param CMD 发送的命令码
 * @param add_data_len 附加数据的长度
 * @param add_data 指向附加数据的指针
 * @return void
 */
void FPM383D::SendData(uint16_t CMD, uint8_t add_data_len, uint8_t *add_data)
{
	Send_s send_buffer{};
	send_buffer.header = 0xF11FE22EB66BA88A; // 帧头
	send_buffer.data_len = sizeof(send_buffer) - 10 + add_data_len;
	send_buffer.header_checksum = protocol_get_checksum(reinterpret_cast<uint8_t *>(&send_buffer), 10);
	send_buffer.password = password;
	send_buffer.CMD = CMD;
	uint8_t data_checksum = protocol_get_checksum(reinterpret_cast<uint8_t *>(&send_buffer) + 11, 6, add_data_len, add_data);
	send_buffer.header = htonll(send_buffer.header);
	send_buffer.data_len = htons(send_buffer.data_len);
	send_buffer.password = htonl(send_buffer.password);
	send_buffer.CMD = htons(send_buffer.CMD);
	uart_write_bytes(Finger_uart, reinterpret_cast<uint8_t *>(&send_buffer), sizeof(send_buffer));
	if (add_data_len > 0)
		uart_write_bytes(Finger_uart, add_data, add_data_len);
	uart_write_bytes(Finger_uart, &data_checksum, 1);
}

/**
 * @brief 从指纹模块接收数据
 * @param Timeout 超时时间 ms
 * @param header_checksum 期望的帧头校验和
 * @param CMD 期望的命令码
 * @return 错误码
 */
uint8_t FPM383D::ReceiveData(uint16_t Timeout, uint8_t header_checksum, uint16_t CMD)
{
	uint8_t FPM383D_ReceiveBuffer[Finger_Bufferlength] = {0};
	Error_e status = no_error;
	uint8_t data_length = 0;
	uint8_t bytes_read = 1;
	while (bytes_read)
	{
		uint8_t data;
		bytes_read = uart_read_bytes(Finger_uart, &data, 1, portMAX_DELAY);
		if (bytes_read == 1)
		{
			// 将接收到的数据存入缓冲区
			if (data_length < Finger_Bufferlength)
			{
				FPM383D_ReceiveBuffer[data_length] = data;
				data_length++;
			}
			else
			{
				break;
				// 缓冲区已满，可以在这里添加错误处理逻辑
			}

			if (data_length == 10)
			{
				if (*(uint64_t *)FPM383D_ReceiveBuffer != 0xF11FE22EB66BA88A || // 帧头
					protocol_get_checksum(FPM383D_ReceiveBuffer, 10) != header_checksum)
				{ // 帧头校验
					status = header_error;
					ESP_LOGE("Finger", "包头错误");
				}
			}
			else if (data_length == 14)
			{
				if (*(uint32_t *)(FPM383D_ReceiveBuffer + 10) != password)
				{
					status = password_error;
					ESP_LOGE("Finger", "密码错误");
				}
			}
			else if (data_length == 16)
			{
				if (*(uint16_t *)(FPM383D_ReceiveBuffer + 14) != CMD)
				{
					status = cmd_error;
					ESP_LOGE("Finger", "命令错误");
				}
			}
			else if (data_length == 20)
			{
				Error_INFO = *(uint32_t *)(FPM383D_ReceiveBuffer + 16);
				if (Error_INFO != 0x00000000)
				{
					status = internal_error;
					ESP_LOGE("Finger", "内部错误");
				}
			}
			else if (data_length > 20)
			{
				FPM383D_Receive_Pack[data_length - 21] = FPM383D_ReceiveBuffer[data_length];
			}
		}
	}
	if (FPM383D_Receive_Pack[data_length - 21] != protocol_get_checksum(FPM383D_ReceiveBuffer + 10, data_length - 11))
	{
		status = data_error;
		ESP_LOGE("Finger", "数据错误");
	}
	FPM383D_Receive_Pack[data_length - 21] = 0; // 从数据中清除校验位
	return status;
}
/**
 * @brief 自动注册指纹
 * @param ID 指纹 ID，通常是一个唯一的标识符
 * @param wait 设置为 1 则表示按压后需要等待手指抬起再次按压才可以进行下一次注册，设置为 0 则不需要检测手指抬起（默认为 0x01）
 * @param press_num 需要按压的次数（默认为 0x06）
 * @return uint8_t 操作结果代码，通常是成功或失败
 */
uint8_t FPM383D::AutoEnroll(uint16_t ID, uint8_t wait, uint8_t press_num)
{
	uint8_t TxBuffer[4] = {wait, press_num, static_cast<uint8_t>(ID >> 8), static_cast<uint8_t>(ID & 0xFF)};
	SendData(0x0118, 4, TxBuffer);
	for (uint8_t i = 0; i < 7; i++)
	{
		ReceiveData(10000, 0x7E, 0x0118);
		ErrorAnalyzer();
		ESP_LOGI("Finger", "次数：%hhu，进度：%hhu", FPM383D_Receive_Pack[0], FPM383D_Receive_Pack[3]);
	}
	if (FPM383D_Receive_Pack[3] >= 0x64)
	{
		// 成功
		ESP_LOGI("Finger", "录入成功");
		return 1;
	}
	else
	{
		// 失败
		ESP_LOGI("Finger", "录入失败");
		return 0;
	}
}
/**
 * @brief 设置系统策略并发送到FPM383D设备
 * @param check_always 是否启用重复指纹检查策略（true表示启用，false表示禁用）
 * @param auto_stuty 是否启用自学习功能策略（true表示启用，false表示禁用）
 * @param _360Identify 是否启用360度识别策略（true表示启用，false表示禁用）
 * @return uint8_t 表示设置策略的结果
 *                - 返回值为0表示成功设置策略并发送到设备
 *                - 非0值表示设置策略失败
 */
uint8_t FPM383D::SetSystemPolicy(bool check_always, bool auto_stuty, bool _360Identify)
{
	uint32_t policy = 0u;
	uint32_t mask = 0u;
	mask |= (0 ? 1u : 0u) << 5; // 内部使用
	mask |= (_360Identify ? 1u : 0u) << 4;
	mask |= (0 ? 1u : 0u) << 3; // 内部使用
	mask |= (auto_stuty ? 1u : 0u) << 2;
	mask |= (check_always ? 1u : 0u) << 1;
	mask |= (0 ? 1u : 0u); // 内部使用
	policy |= mask;
	policy = htonl(policy);
	SendData(0x02FC, 4, (uint8_t *)&policy);

	uint8_t error = ReceiveData(1000, 0x82, 0x02FC);
	ErrorAnalyzer();
	return error;
	return 1;
}
/**
 * @brief 设置新的设备密码并发送给设备。
 * @param New_password 新的密码，为32位整数
 * @return 设置密码后的响应值
 */
uint8_t FPM383D::SetPassword(uint32_t New_password)
{
	// SWAP_LONG(New_password);
	SendData(0x0305, 4, (uint8_t *)&New_password);
	password = New_password; ///@warning 修改新密码后需要修改宏定义  @todo 密码写入ESP32C3的flash中
	uint8_t error = ReceiveData(1000, 0x82, 0x0305);
	ErrorAnalyzer();
	return error;
}
/**
 * 设置设备的休眠模式，并发送给设备。
 * @param sleep_mode 休眠模式，00 表示进入普通休眠模式,01 表示进入深度休眠模式，默认为0x00
 * @return 设置休眠模式后的响应值
 */
uint8_t FPM383D::SetSleepMode(uint8_t sleep_mode)
{
	SendData(0x020C, 1, &sleep_mode);
	uint8_t error = ReceiveData(1000, 0x82, 0x020C);
	ErrorAnalyzer();
	return error;
}
/**
 * @brief 发起设备身份验证请求，并接收响应结果。
 * @return 0正常 1错误
 */
uint8_t FPM383D::Identify()
{
	SendData(0x0123);
	ReceiveData(2000, 0x7C, 0x0123);
	if (FPM383D_Receive_Pack[1] == 0x01)
	{
		// 成功
		Finger_INFO.score = ((uint16_t)FPM383D_Receive_Pack[2] << 8) + (uint16_t)FPM383D_Receive_Pack[3];
		Finger_INFO.ID = ((uint16_t)FPM383D_Receive_Pack[4] << 8) + (uint16_t)FPM383D_Receive_Pack[5];
		ESP_LOGI("Finger", "成功验证，ID=%hu,score=%hu", Finger_INFO.ID, Finger_INFO.score);
		return 0;
	}
	else
	{
		// 失败
		ESP_LOGI("Finger", "验证失败");
		return 1;
	}
}
/**
 * @brief 从FPM383D指纹模块中删除一个或多个指纹ID。
 * @param clean_flag 0x01 表示清除所有指纹，0x02 表示清除多条指纹，
 * 0x03 表示清除一块指纹，默认 0x00 表示清除单个指纹。
 * @param count 要删除的指纹ID的数量。
 * @param ... 要删除的指纹ID列表 uint16_t
 * @return uint8_t 删除操作的结果或状态码。
 */
uint8_t FPM383D::Delete(uint8_t clean_flag, uint16_t count, ...)
{
	va_list args;
	va_start(args, count);
	uint8_t ID[120] = {0};
	ID[0] = clean_flag;
	for (uint16_t i = 0; i < count; i++)
	{
		int value_int = va_arg(args, int);						   // 接收为 int
		uint16_t value = static_cast<uint16_t>(value_int);		   // 强制转换为 uint16_t
		ID[i * 2 + 1] = static_cast<uint8_t>(value & 0xFF);		   // 高位字节
		ID[i * 2 + 2] = static_cast<uint8_t>((value >> 8) & 0xFF); // 低位字节
	}
	va_end(args);
	SendData(0x0136, 1 + count * 2, ID);
	uint8_t error = ReceiveData(1000, 0x82, 0x0136);
	ErrorAnalyzer();
	return error;
}
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
uint8_t FPM383D::SetLED(uint8_t control_mode, uint8_t color, uint8_t arg1, uint8_t arg2, uint8_t arg3)
{
	uint8_t buffer[5] = {control_mode, color, arg1, arg2, arg3};
	SendData(0x020F, 5, buffer);
	uint8_t error = ReceiveData(1000, 0x82, 0x020F);
	ErrorAnalyzer();
	return error;
}
/**
 * @brief 错误码解码
 * @return void
 */
void FPM383D::ErrorAnalyzer()
{
	switch (Error_INFO)
	{
	case 0x00000000:
		ESP_LOGI("Finger", "正常完成");
		break;
	case 0x00000001:
		ESP_LOGE("Finger", "无法识别的命令\n");
		break;
	case 0x00000002:
		ESP_LOGE("Finger", "命令数据长度非法\n");
		break;
	case 0x00000003:
		ESP_LOGE("Finger", "命令字段数据非法\n");
		break;
	case 0x00000004:
		ESP_LOGE("Finger", "系统忙，无法执行当前命令\n");
		break;
	case 0x00000005:
		ESP_LOGE("Finger", "没有发送该命令的请求，就查询结果\n");
		break;
	case 0x00000006:
		ESP_LOGE("Finger", "系统软件上报错误\n");
		break;
	case 0x00000007:
		ESP_LOGE("Finger", "硬件错误\n");
		break;
	case 0x00000008:
		ESP_LOGE("Finger", "没有检测到手指按压，超时退出\n");
		break;
	case 0x00000009:
		ESP_LOGE("Finger", "指纹提取发生错误\n");
		break;
	case 0x0000000A:
		ESP_LOGE("Finger", "指纹匹配发生错误(指纹模板库为空)\n");
		break;
	case 0x0000000B:
		ESP_LOGE("Finger", "指纹数据存储空间满\n");
		break;
	case 0x0000000C:
		ESP_LOGE("Finger", "存储写入失败\n");
		break;
	case 0x0000000D:
		ESP_LOGE("Finger", "存储读取失败\n");
		break;
	case 0x0000000E:
		ESP_LOGE("Finger", "采集的指纹图像质量不佳\n");
		break;
	case 0x0000000F:
		ESP_LOGE("Finger", "指纹重复\n");
		break;
	case 0x00000010:
		ESP_LOGE("Finger", "采图面积太小，手指与 sensor 接触面太小\n");
		break;
	case 0x00000011:
		ESP_LOGE("Finger", "采图时手指移动范围过大\n");
		break;
	case 0x00000012:
		ESP_LOGE("Finger", "采图时手指移动范围过小\n");
		break;
	case 0x00000013:
		ESP_LOGE("Finger", "指纹 ID 被占用\n");
		break;
	case 0x00000014:
		ESP_LOGE("Finger", "模组采图失败\n");
		break;
	case 0x00000015:
		ESP_LOGE("Finger", "命令强制中断\n");
		break;
	case 0x00000016:
		ESP_LOGE("Finger", "指纹特征数据不需要更新\n");
		break;
	case 0x00000017:
		ESP_LOGE("Finger", "无效指纹 ID\n");
		break;
	case 0x00000018:
		ESP_LOGE("Finger", "增益调整失败\n");
		break;
	case 0x00000019:
		ESP_LOGE("Finger", "数据缓冲区溢出\n");
		break;
	case 0x0000001A:
		ESP_LOGE("Finger", "sensor 休眠状态下收到采图相关消息返回错误\n");
		break;
	case 0x0000001C:
		ESP_LOGE("Finger", "校验和错误\n");
		break;
	case 0x00000022:
		ESP_LOGE("Finger", "注册存储时写 Flash 失败\n");
		break;
	case 0x000000FF:
		ESP_LOGE("Finger", "其他错误\n");
		break;
	}
}
void FPM383D::ISR_Resume(void)
{
	gpio_isr_handler_add(Finger_Interrupt_pin, Finger_ISR_Handler, (void *)Finger_Interrupt_pin);
}
extern FPM383D myFPM;
static void IRAM_ATTR Finger_ISR_Handler(void *arg)
{
	// uint32_t GPIO_num = (uint32_t)arg;
	// if (GPIO_num == GPIO_NUM_1)
	// {
	// 	BaseType_t YieldRequired;
	// 	YieldRequired = xTaskResumeFromISR(finger_identify_task_handle); // 启动指纹任务
	// 	if (YieldRequired == pdTRUE)
	// 	{
	// 		portYIELD_FROM_ISR(YieldRequired);
	// 	}
	// 	myFPM.ScanState = 1;
	// 	gpio_isr_handler_remove(GPIO_NUM_1);
	// }
}
// 最简化实现版本
extern FPM383D myFPM;
extern DY_SV17F myAudio;
extern uint8_t Audio_status;
extern TaskHandle_t voice_play_task_handle;
extern TaskHandle_t servo_task_handle;
void uart_event_task(void *pvParameters)
{
	static const uint8_t send_buffer[] = {0xF1, 0x1F, 0xE2, 0x2E, 0xB6, 0x6B, 0xA8, 0x8A, 0x00, 0x07, 0x86, 0x00, 0x00, 0x00, 0x00, 0x01, 0x23, 0xDC};
	uart_event_t event;
	static uint8_t wrong_times = 0;
	static uint16_t tick_time = 0;
	vTaskDelay(3000); // 启动延时3000ms
	uart_write_bytes(myFPM.Finger_uart, send_buffer, sizeof(send_buffer));
	while (1)
	{
		// 等待串口事件
		if (xQueueReceive(myFPM.uart_queue, (void *)&event, portMAX_DELAY))
		{
			bzero(myFPM.RXdata, Finger_Bufferlength);
			// 处理不同的事件类型
			switch (event.type)
			{
			case UART_DATA:
				uart_read_bytes(myFPM.Finger_uart, myFPM.RXdata, event.size, portMAX_DELAY);
				if (myFPM.RXdata[20] == 0x00)
				{
					if (myFPM.RXdata[22] == 0x01)
					{
						Audio_status = correct;
						vTaskResume(servo_task_handle);
					}
					else if (myFPM.RXdata[22] == 0x00)
					{
						Audio_status = wrong;
						wrong_times++;
						tick_time = 0;
						if (wrong_times > 3)
						{
							Audio_status = forbidden;
							wrong_times = 0;
						}
					}
					vTaskResume(voice_play_task_handle);
					vTaskDelay(1000);
				}
				else
				{
					tick_time++;
					if (tick_time > 12)
					{
						wrong_times = 0; // 一段时间后重置错误次数
					}
				}
				uart_write_bytes(myFPM.Finger_uart, send_buffer, sizeof(send_buffer));
				// printf("Received data: %s\n", data);
				break;
			case UART_FIFO_OVF:
				printf("UART FIFO overflow\n");
				uart_flush_input(myFPM.Finger_uart);
				xQueueReset(myFPM.uart_queue);
				break;
			case UART_BUFFER_FULL:
				printf("UART buffer full\n");
				uart_flush_input(myFPM.Finger_uart);
				xQueueReset(myFPM.uart_queue);
				break;
			case UART_PARITY_ERR:
				printf("UART parity error\n");
				break;
			case UART_FRAME_ERR:
				printf("UART frame error\n");
				break;
			default:
				break;
			}
		}
	}
}
