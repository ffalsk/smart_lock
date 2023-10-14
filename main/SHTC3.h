#ifndef SHTC3_H
#define SHTC3_H

#include "driver/i2c.h"
#include "esp_log.h"
#include <stdint.h>

#define SHTC3_ADDR_7BIT 0x70
#define SHTC3_ADDR_WRITE 0xE0
#define SHTC3_ADDR_READ 0xE1

#define SHTC3_MAX_CLOCK_FREQ 1000000

typedef enum
{
	SHTC3_CMD_WAKE = 0x3517,
	SHTC3_CMD_SLEEP = 0xB098,

	SHTC3_CMD_SFT_RST = 0x805D,

	SHTC3_CMD_READ_ID = 0xEFC8,
} SHTC3_Commands_TypeDef;

typedef enum
{
	SHTC3_CMD_CSE_RHF_NPM = 0x5C24, // Clock stretching, RH first, Normal power mode
	SHTC3_CMD_CSE_RHF_LPM = 0x44DE, // Clock stretching, RH first, Low power mode
	SHTC3_CMD_CSE_TF_NPM = 0x7CA2,	// Clock stretching, T first, Normal power mode
	SHTC3_CMD_CSE_TF_LPM = 0x6458,	// Clock stretching, T first, Low power mode

	SHTC3_CMD_CSD_RHF_NPM = 0x58E0, // Polling, RH first, Normal power mode
	SHTC3_CMD_CSD_RHF_LPM = 0x401A, // Polling, RH first, Low power mode
	SHTC3_CMD_CSD_TF_NPM = 0x7866,	// Polling, T first, Normal power mode
	SHTC3_CMD_CSD_TF_LPM = 0x609C	// Polling, T first, Low power mode
} SHTC3_MeasurementModes_TypeDef;

typedef enum
{
	SHTC3_Status_Nominal = 0,
	SHTC3_Status_Error,
	SHTC3_Status_CRC_Fail,
	SHTC3_Status_ID_Fail
} SHTC3_Status_TypeDef;

class SHTC3
{
private:
	i2c_port_t i2c_port;
	uint8_t i2c_address;
	SHTC3_MeasurementModes_TypeDef _mode;
	bool _inProcess;
	bool _staySleeping;
	bool _isAsleep;
	SHTC3_Status_TypeDef sendCommand(SHTC3_Commands_TypeDef cmd);
	SHTC3_Status_TypeDef sendCommand(SHTC3_MeasurementModes_TypeDef cmd);
	SHTC3_Status_TypeDef abortUpdate(SHTC3_Status_TypeDef status, const char *file, uint16_t line);
	SHTC3_Status_TypeDef exitOp(SHTC3_Status_TypeDef status, const char *file, uint16_t line);
	SHTC3_Status_TypeDef startProcess(void);
	SHTC3_Status_TypeDef endProcess(void);

public:
	SHTC3(uint8_t sdaPin, uint8_t sclPin);
	SHTC3_Status_TypeDef lastStatus;
	bool passRHcrc;
	bool passTcrc;
	bool passIDcrc;
	uint16_t RH;
	uint16_t T;
	uint16_t ID;
	float toDegC();
	float toDegF();
	float toPercent();
	SHTC3_Status_TypeDef begin(i2c_port_t port);
	SHTC3_Status_TypeDef softReset();
	SHTC3_Status_TypeDef checkID();
	SHTC3_Status_TypeDef sleep(bool hold = false);
	SHTC3_Status_TypeDef wake(bool hold = false);
	SHTC3_Status_TypeDef setMode(SHTC3_MeasurementModes_TypeDef mode = SHTC3_CMD_CSD_RHF_NPM);
	SHTC3_MeasurementModes_TypeDef getMode(void);
	SHTC3_Status_TypeDef update();
	SHTC3_Status_TypeDef checkCRC(uint16_t packet, uint8_t cs);
};

float SHTC3_raw2DegC(uint16_t T);
float SHTC3_raw2DegF(uint16_t T);
float SHTC3_raw2Percent(uint16_t RH);
void SHTC3_exitOp_Callback(SHTC3_Status_TypeDef status, bool inProcess, const char *file, uint16_t line) __attribute__((weak));

#endif /* SHTC3_H */
