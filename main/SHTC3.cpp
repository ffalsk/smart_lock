#include "SHTC3.h"

SHTC3_Status_TypeDef SHTC3::sendCommand(SHTC3_Commands_TypeDef cmd)
{
	i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
	i2c_master_start(cmd_handle);
	i2c_master_write_byte(cmd_handle, SHTC3_ADDR_7BIT << 1 | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd_handle, (((uint16_t)cmd) >> 8), true);
	i2c_master_write_byte(cmd_handle, (((uint16_t)cmd) & 0x00FF), true);
	i2c_master_stop(cmd_handle);

	esp_err_t res = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000);
	i2c_cmd_link_delete(cmd_handle);

	if (res != ESP_OK)
	{
		return SHTC3_Status_Error;
	}
	return SHTC3_Status_Nominal;
}

SHTC3_Status_TypeDef SHTC3::sendCommand(SHTC3_MeasurementModes_TypeDef cmd)
{
	return sendCommand((SHTC3_Commands_TypeDef)cmd);
}

SHTC3_Status_TypeDef SHTC3::abortUpdate(SHTC3_Status_TypeDef status, const char *file, uint16_t line)
{
	passRHcrc = false;
	passTcrc = false;
	passIDcrc = false;

	RH = 0x00;
	T = 0x00;
	ID = 0x00;

	return exitOp(status, file, line);
}

SHTC3_Status_TypeDef SHTC3::exitOp(SHTC3_Status_TypeDef status, const char *file, uint16_t line)
{
	lastStatus = status;
	SHTC3_exitOp_Callback(status, _inProcess, file, line);
	return status;
}

SHTC3_Status_TypeDef SHTC3::startProcess(void)
{
	SHTC3_Status_TypeDef retval = SHTC3_Status_Nominal;
	_inProcess = true;
	if (_isAsleep)
	{
		retval = wake();
	}
	if (retval == SHTC3_Status_Nominal)
	{
		_isAsleep = false;
	}
	return exitOp(retval, __FILE__, __LINE__);
}

SHTC3_Status_TypeDef SHTC3::endProcess(void)
{
	SHTC3_Status_TypeDef retval = SHTC3_Status_Nominal;
	_inProcess = false;
	if (_staySleeping)
	{
		retval = sleep();
	}
	if (retval == SHTC3_Status_Nominal)
	{
		_isAsleep = true;
	}
	return exitOp(retval, __FILE__, __LINE__);
}

SHTC3::SHTC3(uint8_t sdaPin, uint8_t sclPin) // Constructor with custom I2C pins
{
	_mode = SHTC3_CMD_CSE_RHF_NPM; // My default pick

	_inProcess = false;	  // Definitely not doing anything when the object is instantiated
	_staySleeping = true; // Default to storing the sensor in low-power mode
	_isAsleep = true;	  // Assume the sensor is asleep to begin (there won't be any harm in waking it up if it is already awake)

	passRHcrc = false;
	passTcrc = false;
	passIDcrc = false;

	RH = 0x00;
	T = 0x00;
	ID = 0x00;

	// Configure custom I2C pins
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = sdaPin;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = sclPin;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = SHTC3_MAX_CLOCK_FREQ;

	i2c_param_config(I2C_NUM_0, &conf);
	i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
	i2c_set_timeout(I2C_NUM_0, 10000);
}

float SHTC3::toDegC()
{
	return SHTC3_raw2DegC(T);
}

float SHTC3::toDegF()
{
	return SHTC3_raw2DegF(T);
}

float SHTC3::toPercent()
{
	return SHTC3_raw2Percent(RH);
}

SHTC3_Status_TypeDef SHTC3::begin(i2c_port_t port)
{
	SHTC3_Status_TypeDef retval = SHTC3_Status_Nominal;

	i2c_port = port;

	retval = startProcess(); // Multiple commands will go to the sensor before sleeping
	if (retval != SHTC3_Status_Nominal)
	{
		return exitOp(retval, __FILE__, __LINE__);
	}

	retval = wake(); // Wake up the sensor
	if (retval != SHTC3_Status_Nominal)
	{
		return exitOp(retval, __FILE__, __LINE__);
	}

	retval = checkID(); // Verify that the sensor is actually an SHTC3
	if (retval != SHTC3_Status_Nominal)
	{
		return exitOp(retval, __FILE__, __LINE__);
	}

	retval = endProcess(); // We are about to return to user-land
	if (retval != SHTC3_Status_Nominal)
	{
		return exitOp(retval, __FILE__, __LINE__);
	}

	return exitOp(retval, __FILE__, __LINE__);
}

SHTC3_Status_TypeDef SHTC3::softReset()
{
	return exitOp(sendCommand(SHTC3_CMD_SFT_RST), __FILE__, __LINE__);
	vTaskDelay(1); // Use vTaskDelay for non-blocking delay
}

SHTC3_Status_TypeDef SHTC3::checkID()
{
	SHTC3_Status_TypeDef retval = SHTC3_Status_Nominal;

	retval = startProcess(); // There will be several commands sent before returning control to the user
	if (retval != SHTC3_Status_Nominal)
	{
		return abortUpdate(retval, __FILE__, __LINE__);
	}

	retval = wake();
	if (retval != SHTC3_Status_Nominal)
	{
		return exitOp(retval, __FILE__, __LINE__);
	}

	retval = sendCommand(SHTC3_CMD_READ_ID);
	if (retval != SHTC3_Status_Nominal)
	{
		return exitOp(retval, __FILE__, __LINE__);
	}

	uint8_t IDhb = 0x00;
	uint8_t IDlb = 0x00;
	uint8_t IDcs = 0x00;

	i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
	i2c_master_start(cmd_handle);
	i2c_master_write_byte(cmd_handle, (uint8_t)((SHTC3_ADDR_READ << 1) | I2C_MASTER_READ), true);
	i2c_master_read(cmd_handle, &IDhb, 1, I2C_MASTER_ACK);
	i2c_master_read(cmd_handle, &IDlb, 1, I2C_MASTER_ACK);
	i2c_master_read(cmd_handle, &IDcs, 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd_handle);

	esp_err_t res = i2c_master_cmd_begin(i2c_port, cmd_handle, 1000);
	i2c_cmd_link_delete(cmd_handle);

	if (res != ESP_OK)
	{
		return exitOp(SHTC3_Status_Error, __FILE__, __LINE__);
	}

	ID = (((uint16_t)IDhb << 8) | ((uint16_t)IDlb));

	passIDcrc = false;

	if (checkCRC(ID, IDcs) == SHTC3_Status_Nominal)
	{
		passIDcrc = true;
	}

	if ((ID & 0b0000100000111111) != 0b0000100000000111)
	{
		return exitOp(SHTC3_Status_ID_Fail, __FILE__, __LINE__);
	}

	retval = endProcess(); // We are about to return to user-land
	if (retval != SHTC3_Status_Nominal)
	{
		return exitOp(retval, __FILE__, __LINE__);
	}

	return exitOp(retval, __FILE__, __LINE__);
}

SHTC3_Status_TypeDef SHTC3::sleep(bool hold)
{
	_isAsleep = true;
	if (hold)
	{
		_staySleeping = true;
	}
	return exitOp(sendCommand(SHTC3_CMD_SLEEP), __FILE__, __LINE__);
}

SHTC3_Status_TypeDef SHTC3::wake(bool hold)
{
	SHTC3_Status_TypeDef retval = sendCommand(SHTC3_CMD_WAKE);
	if (retval == SHTC3_Status_Nominal)
	{
		_isAsleep = false;
	}
	if (hold)
	{
		_staySleeping = false;
	}
	vTaskDelay(1); // Use vTaskDelay for non-blocking delay
	return exitOp(retval, __FILE__, __LINE__);
}

SHTC3_Status_TypeDef SHTC3::setMode(SHTC3_MeasurementModes_TypeDef mode)
{
	SHTC3_Status_TypeDef retval = SHTC3_Status_Nominal;
	switch (mode)
	{
	case SHTC3_CMD_CSE_RHF_NPM:
		_mode = SHTC3_CMD_CSE_RHF_NPM;
		break;
	case SHTC3_CMD_CSE_RHF_LPM:
		_mode = SHTC3_CMD_CSE_RHF_LPM;
		break;
	case SHTC3_CMD_CSE_TF_NPM:
		_mode = SHTC3_CMD_CSE_TF_NPM;
		break;
	case SHTC3_CMD_CSE_TF_LPM:
		_mode = SHTC3_CMD_CSE_TF_LPM;
		break;
	default:
		retval = SHTC3_Status_Error;
		break;
	}
	return exitOp(retval, __FILE__, __LINE__);
}

SHTC3_MeasurementModes_TypeDef SHTC3::getMode(void)
{
	return _mode;
}

SHTC3_Status_TypeDef SHTC3::update()
{
	SHTC3_Status_TypeDef retval = SHTC3_Status_Nominal;

	uint8_t RHhb = 0x00;
	uint8_t RHlb = 0x00;
	uint8_t RHcs = 0x00;

	uint8_t Thb = 0x00;
	uint8_t Tlb = 0x00;
	uint8_t Tcs = 0x00;

	retval = startProcess();
	if (retval != SHTC3_Status_Nominal)
	{
		return abortUpdate(retval, __FILE__, __LINE__);
	}

	retval = sendCommand(_mode);
	if (retval != SHTC3_Status_Nominal)
	{
		return abortUpdate(retval, __FILE__, __LINE__);
	}

	switch (_mode)
	{
	case SHTC3_CMD_CSE_RHF_NPM:
	case SHTC3_CMD_CSE_RHF_LPM:
	case SHTC3_CMD_CSE_TF_NPM:
	case SHTC3_CMD_CSE_TF_LPM:
	{ // RH First
		i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
		i2c_master_start(cmd_handle);
		i2c_master_write_byte(cmd_handle, (uint8_t)((SHTC3_ADDR_READ << 1) | I2C_MASTER_READ), true);
		i2c_master_read(cmd_handle, &RHhb, 1, I2C_MASTER_ACK);
		i2c_master_read(cmd_handle, &RHlb, 1, I2C_MASTER_ACK);
		i2c_master_read(cmd_handle, &RHcs, 1, I2C_MASTER_ACK);
		i2c_master_read(cmd_handle, &Thb, 1, I2C_MASTER_ACK);
		i2c_master_read(cmd_handle, &Tlb, 1, I2C_MASTER_ACK);
		i2c_master_read(cmd_handle, &Tcs, 1, I2C_MASTER_NACK);
		i2c_master_stop(cmd_handle);

		esp_err_t res = i2c_master_cmd_begin(i2c_port, cmd_handle, 1000);
		i2c_cmd_link_delete(cmd_handle);

		if (res != ESP_OK)
		{
			return abortUpdate(SHTC3_Status_Error, __FILE__, __LINE__);
		}

		break;
	}
	default:
	{
		return abortUpdate(SHTC3_Status_Error, __FILE__, __LINE__);
		break;
	}
	}

	// Update values
	RH = ((uint16_t)RHhb << 8) | ((uint16_t)RHlb << 0);
	T = ((uint16_t)Thb << 8) | ((uint16_t)Tlb << 0);

	passRHcrc = false;
	passTcrc = false;

	if (checkCRC(RH, RHcs) == SHTC3_Status_Nominal)
	{
		passRHcrc = true;
	}
	if (checkCRC(T, Tcs) == SHTC3_Status_Nominal)
	{
		passTcrc = true;
	}

	retval = endProcess(); // We are about to return to user-land
	if (retval != SHTC3_Status_Nominal)
	{
		return abortUpdate(retval, __FILE__, __LINE__);
	}

	return exitOp(retval, __FILE__, __LINE__);
}

SHTC3_Status_TypeDef SHTC3::checkCRC(uint16_t packet, uint8_t cs)
{
	uint8_t upper = packet >> 8;
	uint8_t lower = packet & 0x00FF;
	uint8_t data[2] = {upper, lower};
	uint8_t crc = 0xFF;
	uint8_t poly = 0x31;

	for (uint8_t indi = 0; indi < 2; indi++)
	{
		crc ^= data[indi];

		for (uint8_t indj = 0; indj < 8; indj++)
		{
			if (crc & 0x80)
			{
				crc = (uint8_t)((crc << 1) ^ poly);
			}
			else
			{
				crc <<= 1;
			}
		}
	}

	if (cs ^ crc)
	{
		return exitOp(SHTC3_Status_CRC_Fail, __FILE__, __LINE__);
	}
	return exitOp(SHTC3_Status_Nominal, __FILE__, __LINE__);
}

float SHTC3_raw2DegC(uint16_t T)
{
	return -45 + 175 * ((float)T / 65535);
}

float SHTC3_raw2DegF(uint16_t T)
{
	return static_cast<float>(SHTC3_raw2DegC(T) * (9.0 / 5) + 32.0);
}

float SHTC3_raw2Percent(uint16_t RH)
{
	return 100 * ((float)RH / 65535);
}

void SHTC3_exitOp_Callback(SHTC3_Status_TypeDef status, bool inProcess, const char *file, uint16_t line)
{ // Empty implementation. You can make your own implementation
}
