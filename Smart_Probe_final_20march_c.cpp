/*
 * Smart_Probe.cpp
 * Created: 17-03-2026
 * Author : cw
 */

#define F_CPU  3333333UL
#define PERIOD_EXAMPLE_VALUE (0x1299)
#define DUTY_CYCLE_EXAMPLE_VALUE (0x0000)
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 *(float)BAUD_RATE)) + 0.5)
#define USART1_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 *(float)BAUD_RATE)) + 0.5)
#define PERIOD_EXAMPLE_VALUE_H (250)
#define DUTY_CYCLE_EXAMPLE_VALUE_H (10)

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdint-gcc.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <math.h>

#define I2C_TIMEOUT 10000
#define I2C_SCL   (1<<0)
#define I2C_SDA   (1<<1)
#define I2C_STATE_IS_HIGH(x) ((TWI0.MSTATUS & (x)) == (x))
#define I2C_STATE_IS_LOW(x)  ((TWI0.MSTATUS & (x)) != (x))
#define I2C_BUS_STATE        (TWI0.MSTATUS & TWI_BUSSTATE_gm)
#define I2C_BUS_NOT_IDLE  I2C_STATE_IS_LOW(TWI_BUSSTATE_IDLE_gc)
#define I2C_BUS_IDLE      I2C_STATE_IS_HIGH(TWI_BUSSTATE_IDLE_gc)
#define I2C_BUS_NOT_BUSY  I2C_STATE_IS_LOW(TWI_BUSSTATE_BUSY_gc)
#define I2C_BUS_BUSY      I2C_STATE_IS_HIGH(TWI_BUSSTATE_BUSY_gc)
#define I2C_BUS_NOT_OWNER I2C_STATE_IS_LOW(TWI_BUSSTATE_OWNER_gc)
#define I2C_BUS_OWNER     I2C_STATE_IS_HIGH(TWI_BUSSTATE_OWNER_gc)
#define I2C_NOT_CLOCKHOLD I2C_STATE_IS_LOW(TWI_CLKHOLD_bm)
#define I2C_CLOCKHOLD     I2C_STATE_IS_HIGH(TWI_CLKHOLD_bm)
#define I2C_NOT_BUSERR    I2C_STATE_IS_LOW(TWI_BUSERR_bm)
#define I2C_BUSERR        I2C_STATE_IS_HIGH(TWI_BUSERR_bm)
#define I2C_NOT_ARBLOST   I2C_STATE_IS_LOW(TWI_ARBLOST_bm)
#define I2C_ARBLOST       I2C_STATE_IS_HIGH(TWI_ARBLOST_bm)
#define TWI0_BAUD(F_SCL, T_RISE) ((((((float)F_CPU / (float)F_SCL)) - 10 - ((float)F_CPU * T_RISE / 1000000))) / 2)
#define MASTER_ENABLE  1
#define MASTER_DISABLE 0
#define SLAVE_ENABLE   1
#define SLAVE_DISABLE  0
#define I2C_READ  0x01
#define I2C_WRITE 0x00
#define I2C_WAIT_TIMEOUT  512

#define RETURN_OK          0
#define RETURN_FAILED      1
#define RETURN_NO_SLAVE    2
#define RETURN_BUS_ERROR   3
#define RETURN_BUS_ARBLOST 4
#define RETURN_BUS_BUSY    5

#define ADS122C04_ADDR               0x80
#define ADS122C04_CONVERSION_TIMEOUT 75

#define ADS122C04_RESET_CMD     0x06
#define ADS122C04_START_CMD     0x08
#define ADS122C04_POWERDOWN_CMD 0x02
#define ADS122C04_RDATA_CMD     0x10
#define ADS122C04_RREG_CMD      0x20
#define ADS122C04_WREG_CMD      0x40

#define ADS122C04_WRITE_CMD(reg) (ADS122C04_WREG_CMD | (reg << 2))
#define ADS122C04_READ_CMD(reg)  (ADS122C04_RREG_CMD | (reg << 2))

#define ADS122C04_CONFIG_0_REG  0
#define ADS122C04_CONFIG_1_REG  1
#define ADS122C04_CONFIG_2_REG  2
#define ADS122C04_CONFIG_3_REG  3

#define ADS122C04_MUX_AIN0_AIN1  0x0
#define ADS122C04_MUX_AIN0_AIN2  0x1
#define ADS122C04_MUX_AIN0_AIN3  0x2
#define ADS122C04_MUX_AIN1_AIN0  0x3
#define ADS122C04_MUX_AIN1_AIN2  0x4
#define ADS122C04_MUX_AIN1_AIN3  0x5
#define ADS122C04_MUX_AIN2_AIN3  0x6
#define ADS122C04_MUX_AIN3_AIN2  0x7
#define ADS122C04_MUX_AIN0_AVSS  0x8
#define ADS122C04_MUX_AIN1_AVSS  0x9
#define ADS122C04_MUX_AIN2_AVSS  0xa
#define ADS122C04_MUX_AIN3_AVSS  0xb
#define ADS122C04_MUX_REFPmREFN  0xc
#define ADS122C04_MUX_AVDDmAVSS  0xd
#define ADS122C04_MUX_SHORTED    0xe

#define ADS122C04_GAIN_1    0x0
#define ADS122C04_GAIN_2    0x1
#define ADS122C04_GAIN_4    0x2
#define ADS122C04_GAIN_8    0x3
#define ADS122C04_GAIN_16   0x4
#define ADS122C04_GAIN_32   0x5
#define ADS122C04_GAIN_64   0x6
#define ADS122C04_GAIN_128  0x7

#define ADS122C04_PGA_DISABLED  0x1
#define ADS122C04_PGA_ENABLED   0x0

#define ADS122C04_DATA_RATE_20SPS    0x0
#define ADS122C04_DATA_RATE_45SPS    0x1
#define ADS122C04_DATA_RATE_90SPS    0x2
#define ADS122C04_DATA_RATE_175SPS   0x3
#define ADS122C04_DATA_RATE_330SPS   0x4
#define ADS122C04_DATA_RATE_600SPS   0x5
#define ADS122C04_DATA_RATE_1000SPS  0x6

#define ADS122C04_OP_MODE_NORMAL    0x0
#define ADS122C04_OP_MODE_TURBO     0x1

#define ADS122C04_CONVERSION_MODE_SINGLE_SHOT  0x0
#define ADS122C04_CONVERSION_MODE_CONTINUOUS   0x1

#define ADS122C04_VREF_INTERNAL      0x0
#define ADS122C04_VREF_EXT_REF_PINS  0x1
#define ADS122C04_VREF_AVDD          0x2

#define ADS122C04_TEMP_SENSOR_OFF  0x0
#define ADS122C04_TEMP_SENSOR_ON   0x1

#define ADS122C04_DCNT_DISABLE  0x0
#define ADS122C04_DCNT_ENABLE   0x1

#define ADS122C04_CRC_DISABLED      0x0
#define ADS122C04_CRC_INVERTED      0x1
#define ADS122C04_CRC_CRC16_ENABLED 0x2

#define ADS122C04_BURN_OUT_CURRENT_OFF  0x0
#define ADS122C04_BURN_OUT_CURRENT_ON   0x1

#define ADS122C04_IDAC_CURRENT_OFF     0x0
#define ADS122C04_IDAC_CURRENT_10_UA   0x1
#define ADS122C04_IDAC_CURRENT_50_UA   0x2
#define ADS122C04_IDAC_CURRENT_100_UA  0x3
#define ADS122C04_IDAC_CURRENT_250_UA  0x4
#define ADS122C04_IDAC_CURRENT_500_UA  0x5
#define ADS122C04_IDAC_CURRENT_1000_UA 0x6
#define ADS122C04_IDAC_CURRENT_1500_UA 0x7

#define ADS122C04_IDAC1_DISABLED  0x0
#define ADS122C04_IDAC1_AIN0      0x1
#define ADS122C04_IDAC1_AIN1      0x2
#define ADS122C04_IDAC1_AIN2      0x3
#define ADS122C04_IDAC1_AIN3      0x4
#define ADS122C04_IDAC1_REFP      0x5
#define ADS122C04_IDAC1_REFN      0x6

#define ADS122C04_IDAC2_DISABLED  0x0
#define ADS122C04_IDAC2_AIN0      0x1
#define ADS122C04_IDAC2_AIN1      0x2
#define ADS122C04_IDAC2_AIN2      0x3
#define ADS122C04_IDAC2_AIN3      0x4
#define ADS122C04_IDAC2_REFP      0x5
#define ADS122C04_IDAC2_REFN      0x6

#define ADC_RESET PIN7_bm
#define RSDIR     PIN4_bm

bool I2C_RawRead(uint8_t ACK);
void InitI2C();
bool I2C_RawStart(uint8_t deviceAddr, uint8_t Direction);
bool I2C_RawWrite(uint8_t write_data);
bool I2C_RawStop(void);
unsigned char I2C_ReadByte(uint8_t address, uint8_t reg);
unsigned char I2C_WriteByte(uint8_t address, uint16_t reg, uint8_t data);
void I2C_StartMaster();
bool I2C_StopMaster();

unsigned char rxdata_1;

long long int intmp_reading,sensor_output,sensor_pos,sensor_neg,sensor_final_output,pre_adc_value,diff_adc_value,pub_adc_value,pre_disp_value,Reference_Set_Value;
unsigned char rxdata,rxflag,rxcntr,strt,offset_value;
unsigned int limit;
unsigned char memdata;

volatile uint32_t millis_count = 0;

uint8_t diag_dr   = 0;
uint8_t diag_pga  = 0;
uint8_t diag_gain = 0;

uint16_t inter_sample_ms  = 50;
uint16_t drdy_timeout_ms  = 75;

unsigned int upflag   = 0;
unsigned int overrange = 0;

char str1[] = {"0000      "};
char str2[] = {"0000      "};
char str3[] = {"0000      "};

unsigned char company[150]={' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};

struct CONFIG_REG_0 { uint8_t PGA_BYPASS:1; uint8_t GAIN:3; uint8_t MUX:4; };
union  CONFIG_REG_0_U { uint8_t all; struct CONFIG_REG_0 bit; };
struct CONFIG_REG_1 { uint8_t TS:1; uint8_t VVREF:2; uint8_t CMBIT:1; uint8_t MODE:1; uint8_t DR:3; };
union  CONFIG_REG_1_U { uint8_t all; struct CONFIG_REG_1 bit; };
struct CONFIG_REG_2 { uint8_t IDAC:3; uint8_t BCS:1; uint8_t CRCbits:2; uint8_t DCNT:1; uint8_t DRDY:1; };
union  CONFIG_REG_2_U { uint8_t all; struct CONFIG_REG_2 bit; };
struct CONFIG_REG_3 { uint8_t RESERVED:2; uint8_t I2MUX:3; uint8_t I1MUX:3; };
union  CONFIG_REG_3_U { uint8_t all; struct CONFIG_REG_3 bit; };
typedef struct ADS122C04Reg { union CONFIG_REG_0_U reg0; union CONFIG_REG_1_U reg1; union CONFIG_REG_2_U reg2; union CONFIG_REG_3_U reg3; } ADS122C04Reg_t;
union raw_voltage_union { int32_t INT32; uint32_t UINT32; };

typedef struct {
	uint8_t inputMux; uint8_t gainLevel; uint8_t pgaBypass; uint8_t dataRate;
	uint8_t opMode; uint8_t convMode; uint8_t selectVref; uint8_t tempSensorEn;
	uint8_t dataCounterEn; uint8_t dataCRCen; uint8_t burnOutEn;
	uint8_t idacCurrent; uint8_t routeIDAC1; uint8_t routeIDAC2;
} ADS122C04_initParam;

int32_t readRawVoltage(uint8_t rate = ADS122C04_DATA_RATE_20SPS);
uint32_t readADC(void);
void reset(void); void start(void); void ADC_powerdown(void);
void setInputMultiplexer(uint8_t mux_config = ADS122C04_MUX_AIN1_AIN0);
void setGain(uint8_t gain_config = ADS122C04_GAIN_1);
void enablePGA(uint8_t enable = ADS122C04_PGA_DISABLED);
void setDataRate(uint8_t rate = ADS122C04_DATA_RATE_20SPS);
void setOperatingMode(uint8_t mode = ADS122C04_OP_MODE_NORMAL);
void setConversionMode(uint8_t mode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT);
void setVoltageReference(uint8_t ref = ADS122C04_VREF_INTERNAL);
void enableInternalTempSensor(uint8_t enable = ADS122C04_TEMP_SENSOR_OFF);
void setDataCounter(uint8_t enable = ADS122C04_DCNT_DISABLE);
void setDataIntegrityCheck(uint8_t setting = ADS122C04_CRC_DISABLED);
void setBurnOutCurrent(uint8_t enable = ADS122C04_BURN_OUT_CURRENT_OFF);
void setIDACcurrent(uint8_t current = ADS122C04_IDAC_CURRENT_OFF);
void setIDAC1mux(uint8_t setting = ADS122C04_IDAC1_DISABLED);
void setIDAC2mux(uint8_t setting = ADS122C04_IDAC2_DISABLED);
bool checkDataReady(void);

#define ADS122C04_RAW_MODE  0x4
uint8_t _wireMode = ADS122C04_RAW_MODE;
const float PT100_REFERENCE_RESISTOR = 1620.0;
const float PT100_AMPLIFIER_GAIN = 8.0;
const float PT100_AMP_GAIN_HI_TEMP = 4.0;
const float TEMPERATURE_SENSOR_RESOLUTION = 0.03125;

ADS122C04Reg_t ADS122C04_Reg;

void ADS122C04_writeReg(uint8_t reg, uint8_t writeValue);
bool ADS122C04_readReg(uint8_t reg, uint8_t *readValue);
bool ADS122C04_getConversionData(uint32_t *conversionData);
void ADS122C04_sendCommand(uint8_t command);
void ADS122C04_sendCommandWithValue(uint8_t command, uint8_t value);

bool I2C_StopMaster()
{
	TWI0.MCTRLA &= ~(TWI_ENABLE_bm);
	PORTB.DIRSET &= ~(I2C_SCL);
	return(RETURN_OK);
}

bool I2C_RawStart(uint8_t deviceAddr, uint8_t Direction)
{
	volatile uint16_t timeout;
	deviceAddr = deviceAddr | Direction;
	if (I2C_BUS_NOT_BUSY)
	{
		for (volatile uint16_t arbLoop = 0x04; arbLoop > 0; arbLoop--)
		{
			TWI0.MADDR = deviceAddr;
			if (Direction)
			{ for (timeout = I2C_WAIT_TIMEOUT; I2C_STATE_IS_LOW(TWI_RIF_bm) && (timeout > 0); timeout--){} }
			else
			{ for (timeout = I2C_WAIT_TIMEOUT; I2C_STATE_IS_LOW(TWI_WIF_bm) && (timeout > 0); timeout--){} }
			if (I2C_ARBLOST) continue;
			if (I2C_BUSERR)  return(RETURN_BUS_ERROR);
			if (I2C_STATE_IS_LOW(TWI_RXACK_bm)) return(RETURN_OK);
			else                                 return(RETURN_NO_SLAVE);
		}
	}
	else return(RETURN_BUS_BUSY);
	return(RETURN_BUS_ARBLOST);
}

bool I2C_RawStop(void)
{
	volatile uint16_t timeout;
	for (timeout = I2C_WAIT_TIMEOUT; I2C_STATE_IS_LOW(TWI_CLKHOLD_bm) && (timeout > 0); timeout--){}
	if (timeout == 0) return(RETURN_FAILED);
	TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;
	for (timeout = I2C_WAIT_TIMEOUT; I2C_STATE_IS_LOW(TWI_CLKHOLD_bm) && (timeout > 0); timeout--){}
	if (I2C_BUSERR) TWI0.MSTATUS = TWI_BUSERR_bm;
	return(RETURN_OK);
}

void I2C_RawForceStop(void)
{
	I2C_RawStop();
	TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
	TWI0.MCTRLB  |= TWI_FLUSH_bm;
}

bool I2C_RawWrite(uint8_t write_data)
{
	volatile uint16_t timeout;
	if (I2C_BUS_OWNER)
	{
		for (timeout = I2C_WAIT_TIMEOUT; I2C_STATE_IS_LOW(TWI_CLKHOLD_bm) && (timeout > 0); timeout--){}
		if (timeout == 0) return(RETURN_FAILED);
		TWI0.MDATA = write_data;
		for (timeout = I2C_WAIT_TIMEOUT; I2C_STATE_IS_LOW(TWI_WIF_bm) && (timeout > 0); timeout--){}
		if (I2C_BUSERR) return(RETURN_BUS_ERROR);
		if (I2C_STATE_IS_LOW(TWI_RXACK_bm)) return(RETURN_OK);
		else return(RETURN_NO_SLAVE);
	}
	return(RETURN_FAILED);
}

bool I2C_RawRead(uint8_t ACK)
{
	volatile uint16_t timeout;
	if (I2C_BUS_OWNER)
	{
		for (timeout = I2C_WAIT_TIMEOUT; I2C_STATE_IS_LOW(TWI_CLKHOLD_bm) && (timeout > 0); timeout--){}
		if (timeout == 0) return(RETURN_FAILED);
		if (ACK == 1) TWI0.MCTRLB &= ~(1 << TWI_ACKACT_bp);
		else          TWI0.MCTRLB |=  (1 << TWI_ACKACT_bp);
		for (timeout = I2C_WAIT_TIMEOUT; I2C_STATE_IS_LOW(TWI_RIF_bm) && (timeout > 0); timeout--){}
		if (timeout == 0) return(RETURN_FAILED);
		memdata = TWI0.MDATA;
		if (I2C_BUSERR) return(RETURN_FAILED);
		if (I2C_STATE_IS_LOW(TWI_RXACK_bm)) return(RETURN_OK);
		else return(RETURN_FAILED);
	}
	return(RETURN_FAILED);
}

unsigned char I2C_WriteByte(uint8_t address, uint16_t reg, uint8_t data)
{
	unsigned char result=0;
	unsigned int hi_addr=reg>>8, lo_addr=reg&0x00FF;
	if (I2C_BUS_NOT_BUSY)
	{
		if (I2C_RawStart(address, I2C_WRITE)){ I2C_RawStop(); return(RETURN_FAILED); }
		if (I2C_RawWrite(hi_addr))           { I2C_RawStop(); return(RETURN_FAILED); }
		if (I2C_RawWrite(lo_addr))           { I2C_RawStop(); return(RETURN_FAILED); }
		if (I2C_RawWrite(data))              { I2C_RawStop(); return(RETURN_FAILED); }
		I2C_RawStop();
	}
	else return(RETURN_BUS_BUSY);
	return(result);
}

unsigned char I2C_ReadByte(uint8_t address, uint16_t reg)
{
	unsigned int hi_addr=reg>>8, lo_addr=reg&0x00FF;
	if (I2C_BUS_NOT_BUSY)
	{
		if (I2C_RawStart(address, I2C_WRITE)){ I2C_RawStop(); return(RETURN_FAILED); }
		if (I2C_RawWrite(hi_addr))           { I2C_RawStop(); return(RETURN_FAILED); }
		if (I2C_RawWrite(lo_addr))           { I2C_RawStop(); return(RETURN_FAILED); }
		if (I2C_RawStart(address, I2C_READ)) { I2C_RawStop(); return(RETURN_FAILED); }
		if (I2C_RawRead(0))                  { I2C_RawStop(); return(RETURN_FAILED); }
		I2C_RawStop();
	}
	return(memdata);
}

void I2C_StartMaster()
{
	PORTB.DIRSET = I2C_SCL;
	PORTB.DIRSET = I2C_SDA;
	TWI0.CTRLA   = TWI_SDAHOLD_500NS_gc | TWI_SDASETUP_8CYC_gc;
	TWI0.MBAUD   = (uint8_t)TWI0_BAUD(100000, 1000);
	TWI0.MCTRLA  = 1 << TWI_ENABLE_bp | 0 << TWI_QCEN_bp | 0 << TWI_RIEN_bp
	             | 1 << TWI_SMEN_bp | TWI_TIMEOUT_DISABLED_gc | 0 << TWI_WIEN_bp;
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
	TWI0.MSTATUS = (TWI_RIF_bm | TWI_WIF_bm);
	TWI0.MCTRLB  = TWI_FLUSH_bm;
}

void InitI2C() { I2C_StartMaster(); }

bool start_transmission(char device_address) { return(I2C_RawStart(device_address, I2C_WRITE)); }
bool stop_transmission()                      { return(I2C_RawStop()); }
void write_on_bus(unsigned char ch)           { I2C_RawWrite(ch); }

void ADS122C04_sendCommand(uint8_t command)
{
	start_transmission(ADS122C04_ADDR);
	write_on_bus(command);
	stop_transmission();
}

void start(void)         { ADS122C04_sendCommand(ADS122C04_START_CMD);     }
void reset(void)         { ADS122C04_sendCommand(ADS122C04_RESET_CMD);     }
void ADC_powerdown(void) { ADS122C04_sendCommand(ADS122C04_POWERDOWN_CMD); }

void ADS122C04_sendCommandWithValue(uint8_t command, uint8_t value)
{
	start_transmission(ADS122C04_ADDR);
	write_on_bus(command);
	write_on_bus(value);
	stop_transmission();
}

void ADS122C04_writeReg(uint8_t reg, uint8_t writeValue)
{
	ADS122C04_sendCommandWithValue(ADS122C04_WRITE_CMD(reg), writeValue);
}

bool ADS122C04_readReg(uint8_t reg, uint8_t *readValue)
{
	uint8_t command = ADS122C04_READ_CMD(reg);
	start_transmission(ADS122C04_ADDR);
	write_on_bus(command);
	stop_transmission();
	if (I2C_RawStart(ADS122C04_ADDR, I2C_READ)) { I2C_RawStop(); }
	if (I2C_RawRead(1)) { I2C_RawStop(); return(false); }
	else { *readValue=memdata; I2C_RawStop(); return(true); }
}

bool ADS122C04_getConversionData(uint32_t *conversionData)
{
	uint8_t RXByte[3] = {0};
	start_transmission(ADS122C04_ADDR);
	write_on_bus(ADS122C04_RDATA_CMD);
	stop_transmission();
	if (I2C_RawStart(ADS122C04_ADDR, I2C_READ)) { I2C_RawStop(); }
	if (I2C_RawRead(1)) { I2C_RawStop(); } RXByte[0]=memdata;
	if (I2C_RawRead(1)) { I2C_RawStop(); } RXByte[1]=memdata;
	if (I2C_RawRead(1)) { I2C_RawStop(); } RXByte[2]=memdata;
	*conversionData = ((uint32_t)RXByte[2]) | ((uint32_t)RXByte[1]<<8) | ((uint32_t)RXByte[0]<<16);
	return(true);
}

uint32_t readADC(void)
{
	uint32_t ret_val;
	if(ADS122C04_getConversionData(&ret_val) == false) return(0);
	return(ret_val);
}

void setInputMultiplexer(uint8_t mux_config)
	{ ADS122C04_Reg.reg0.bit.MUX = mux_config; ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all); }
void setGain(uint8_t gain_config)
	{ ADS122C04_Reg.reg0.bit.GAIN = gain_config; ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all); }
void enablePGA(uint8_t enable)
	{ ADS122C04_Reg.reg0.bit.PGA_BYPASS = enable; ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all); }
void setDataRate(uint8_t rate)
	{ ADS122C04_Reg.reg1.bit.DR = rate; ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all); }
void setOperatingMode(uint8_t mode)
	{ ADS122C04_Reg.reg1.bit.MODE = mode; ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all); }
void setConversionMode(uint8_t mode)
	{ ADS122C04_Reg.reg1.bit.CMBIT = mode; ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all); }
void setVoltageReference(uint8_t ref)
	{ ADS122C04_Reg.reg1.bit.VVREF = ref; ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all); }
void enableInternalTempSensor(uint8_t enable)
	{ ADS122C04_Reg.reg1.bit.TS = enable; ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all); }
void setDataCounter(uint8_t enable)
	{ ADS122C04_Reg.reg2.bit.DCNT = enable; ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all); }
void setDataIntegrityCheck(uint8_t setting)
	{ ADS122C04_Reg.reg2.bit.CRCbits = setting; ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all); }
void setBurnOutCurrent(uint8_t enable)
	{ ADS122C04_Reg.reg2.bit.BCS = enable; ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all); }
void setIDACcurrent(uint8_t current)
	{ ADS122C04_Reg.reg2.bit.IDAC = current; ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all); }
void setIDAC1mux(uint8_t setting)
	{ ADS122C04_Reg.reg3.bit.I1MUX = setting; ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all); }
void setIDAC2mux(uint8_t setting)
	{ ADS122C04_Reg.reg3.bit.I2MUX = setting; ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all); }

bool checkDataReady(void)
{
	ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
	return(ADS122C04_Reg.reg2.bit.DRDY > 0);
}

void set_sample_timing(uint8_t dr)
{
	switch(dr)
	{
		case ADS122C04_DATA_RATE_20SPS:   inter_sample_ms=50;  drdy_timeout_ms=75;  break;
		case ADS122C04_DATA_RATE_45SPS:   inter_sample_ms=22;  drdy_timeout_ms=75;  break;
		case ADS122C04_DATA_RATE_90SPS:   inter_sample_ms=44;  drdy_timeout_ms=44;  break;
		case ADS122C04_DATA_RATE_175SPS:  inter_sample_ms=30;  drdy_timeout_ms=36;  break;
		case ADS122C04_DATA_RATE_330SPS:  inter_sample_ms=12;  drdy_timeout_ms=18;  break;
		case ADS122C04_DATA_RATE_600SPS:  inter_sample_ms=8;   drdy_timeout_ms=12;  break;
		case ADS122C04_DATA_RATE_1000SPS: inter_sample_ms=4;   drdy_timeout_ms=8;   break;
		default:                          inter_sample_ms=50;  drdy_timeout_ms=75;  break;
	}
}

void timer_init(void)
{
	TCA0.SINGLE.CTRLA   = TCA_SINGLE_CLKSEL_DIV64_gc;
	TCA0.SINGLE.PER     = 52;
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
	TCA0.SINGLE.CTRLA  |= TCA_SINGLE_ENABLE_bm;
}

ISR(TCA0_OVF_vect)
{
	millis_count++;
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

void mem_write(unsigned int addr, long int val)
{
	unsigned int r=val/10000, s=val%10000;
	eeprom_write_byte((uint8_t*)(1*addr),r); while(!eeprom_is_ready());
	addr++; eeprom_write_byte((uint8_t*)(1*addr),s/256); while(!eeprom_is_ready());
	addr++; eeprom_write_byte((uint8_t*)(1*addr),s%256); while(!eeprom_is_ready());
}

long int mem_read(uint8_t addr)
{
	long int l=eeprom_read_byte((uint8_t*)(1*addr)); asm("nop"); addr++;
	unsigned int m=eeprom_read_byte((uint8_t*)(1*addr)); asm("nop"); addr++;
	unsigned int n=eeprom_read_byte((uint8_t*)(1*addr));
	_delay_ms(30);
	return (l*10000+m*256+n);
}

void USART1_sendChar(char c)
{
	while (!(USART1.STATUS & USART_DREIF_bm)) {}
	USART1.TXDATAL = c;
}

void USART1_init()
{
	PORTA.DIR &= ~PIN2_bm;
	PORTA.DIR |=  PIN1_bm;
	USART1.BAUD = (uint16_t)USART1_BAUD_RATE(9600);
	USART1.CTRLA = 0 << USART_ABEIE_bp | 0 << USART_DREIE_bp | 0 << USART_LBME_bp
	             | 1 << USART_RS485_bp  | 1 << USART_RXCIE_bp | 0 << USART_RXSIE_bp
	             | 0 << USART_TXCIE_bp;
	USART1.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

void chk()
{
	long int a,b,c; unsigned int i,d,e,f,g,addr; i=0;
	if(company[1]=='R' && company[2]=='F' && rxcntr==12)
	{
		a=(company[3]-0x30); a=a*1000000; b=(company[4]-0x30); b=b*100000;
		c=(company[5]-0x30); c=c*10000;  d=(company[6]-0x30); d=d*1000;
		e=(company[8]-0x30); e=e*100;    f=(company[9]-0x30)*1; f=f*10;
		g=(company[10]-0x30)*1;
		Reference_Set_Value=a+b+c+d+e+f+g;
		mem_write((220),Reference_Set_Value);
		USART1_sendChar('O'); USART1_sendChar('K'); USART1_sendChar(0x0D);
		strt=0; rxcntr=0;
	}
	if(company[1]=='T' && rxcntr==10)
	{
		a=(company[2]-0x30); a=a*1000; b=(company[3]-0x30); b=b*100;
		c=(company[4]-0x30); c=c*10;  d=(company[5]-0x30); d=d*1;
		limit=a+b+c+d;
		a=(company[7]-0x30); a=a*10; b=(company[8]-0x30); b=b*1;
		offset_value=a+b;
		mem_write(6,limit); mem_write(150,offset_value);
		USART1_sendChar('O'); USART1_sendChar('K'); USART1_sendChar(0x0D);
		strt=0; rxcntr=0;
	}
}

void update_1()
{
	if(rxdata=='*') { strt=1; rxcntr=0; }
	if(strt==1) { company[rxcntr]=rxdata; rxcntr=rxcntr+1; if(rxdata=='#') chk(); }
}

ISR(USART1_RXC_vect) { rxdata=USART1.RXDATAL; update_1(); }

void home_online_tx(uint32_t elapsed_ms, uint32_t timestamp_ms, uint8_t dr, uint8_t pga, uint8_t gain)
{
	unsigned char i;
	uint32_t t_min = timestamp_ms / 60000;
	uint32_t t_sec = (timestamp_ms % 60000) / 1000;
	uint32_t t_ms  = timestamp_ms % 1000;

	USART1_sendChar(0x0A);
	for(i=0;i<8;i++) USART1_sendChar(str2[i]);
	USART1_sendChar(' '); USART1_sendChar(' ');
	for(i=0;i<9;i++) USART1_sendChar(str3[i]);
	USART1_sendChar(' '); USART1_sendChar(' ');
	USART1_sendChar((elapsed_ms/1000)%10+'0');
	USART1_sendChar((elapsed_ms/100)%10+'0');
	USART1_sendChar((elapsed_ms/10)%10+'0');
	USART1_sendChar(elapsed_ms%10+'0');
	USART1_sendChar(' '); USART1_sendChar(' ');
	USART1_sendChar((t_min/10)%10+'0');
	USART1_sendChar(t_min%10+'0');
	USART1_sendChar(':');
	USART1_sendChar((t_sec/10)%10+'0');
	USART1_sendChar(t_sec%10+'0');
	USART1_sendChar('.');
	USART1_sendChar((t_ms/100)%10+'0');
	USART1_sendChar((t_ms/10)%10+'0');
	USART1_sendChar(t_ms%10+'0');
	USART1_sendChar(' '); USART1_sendChar(' ');
	USART1_sendChar('D'); USART1_sendChar(':'); USART1_sendChar(dr+'0');
	USART1_sendChar(' '); USART1_sendChar(' ');
	USART1_sendChar('P'); USART1_sendChar('G'); USART1_sendChar('A');
	USART1_sendChar('B'); USART1_sendChar('Y'); USART1_sendChar('P');
	USART1_sendChar(':'); USART1_sendChar(pga+'0');
	USART1_sendChar(' '); USART1_sendChar(' ');
	USART1_sendChar('G'); USART1_sendChar(':'); USART1_sendChar(gain+'0');
	USART1_sendChar(' '); USART1_sendChar(' ');
	USART1_sendChar(0x0D);
}

int32_t readRawVoltage(uint8_t rate)
{
	raw_voltage_union raw_v;
	bool drdy = false;
	start();
	do { _delay_ms(10); drdy = checkDataReady(); } while ((drdy == false));
	if (drdy == false) return(0);
	if(ADS122C04_getConversionData(&raw_v.UINT32) == false) return(0);
	if ((raw_v.UINT32 & 0x00800000) == 0x00800000) raw_v.UINT32 |= 0xFF000000;
	return(raw_v.INT32);
}

void filter_raw_sensor()
{
	if(sensor_final_output>=pub_adc_value)
	{
		diff_adc_value=sensor_final_output-pub_adc_value;
		if(diff_adc_value>limit) pub_adc_value=sensor_final_output;
		else                     sensor_final_output=pub_adc_value;
	}
	else
	{
		diff_adc_value=pub_adc_value-sensor_final_output;
		if(diff_adc_value>limit) pub_adc_value=sensor_final_output;
		else                     sensor_final_output=pub_adc_value;
	}
}

static void read_sensor_slow()
{
	long long adc_data, adc_avg;
	unsigned int millis_cnt;
	adc_data=0; adc_avg=0;

	for(unsigned int i=0; i<4; i++)
	{
		bool drdy = 0;
		millis_cnt = 0;
		start();

		do
		{
			millis_cnt = millis_cnt + 1;
			_delay_ms(1);
			drdy = checkDataReady();
		} while ((drdy == 0) && (millis_cnt < drdy_timeout_ms));

		uint32_t raw_ADC_data = readADC();
		if(raw_ADC_data >= 0x800000) raw_ADC_data = 0x0000;

		float volts = ((float)raw_ADC_data) / 4096;
		adc_data = volts * 1000;
		adc_avg  = adc_avg + adc_data;

		for(uint16_t d=0; d<inter_sample_ms; d++) _delay_ms(1);
	}

	adc_data = adc_avg / 4;
	sensor_final_output = adc_data;
	if(sensor_final_output > 2047999) sensor_final_output = 2047999;
	filter_raw_sensor();
}

static void read_sensor_90()
{
	long long adc_data=0, adc_avg=0;
	for(unsigned int i=0; i<4; i++)
	{
		bool drdy=0; uint32_t t=millis_count; start();
		do { drdy=checkDataReady(); } while((drdy==0)&&((millis_count-t)<drdy_timeout_ms));
		uint32_t raw=readADC(); if(raw>=0x800000) raw=0;
		adc_data=(long long)(((float)raw)/4096*1000); adc_avg+=adc_data;
		for(uint16_t d=0; d<inter_sample_ms; d++) _delay_ms(1);
	}
	adc_data=adc_avg/4;
	sensor_final_output=adc_data;
	if(sensor_final_output>2047999) sensor_final_output=2047999;
	filter_raw_sensor();
}

static void read_sensor_175()
{
	long long adc_data=0;
	bool drdy=0; uint32_t t=millis_count; start();
	do { drdy=checkDataReady(); } while((drdy==0)&&((millis_count-t)<drdy_timeout_ms));
	uint32_t raw=readADC(); if(raw>=0x800000) raw=0;
	adc_data=(long long)(((float)raw)/4096*1000);
	sensor_final_output=adc_data;
	if(sensor_final_output>2047999) sensor_final_output=2047999;
	t=millis_count; while((millis_count-t)<inter_sample_ms){}
	filter_raw_sensor();
}

static void read_sensor_330()
{
	long long adc_data=0;
	bool drdy=0; uint32_t t=millis_count; start();
	do { drdy=checkDataReady(); } while((drdy==0)&&((millis_count-t)<drdy_timeout_ms));
	uint32_t raw=readADC(); if(raw>=0x800000) raw=0;
	adc_data=(long long)(((float)raw)/4096*1000);
	sensor_final_output=adc_data;
	if(sensor_final_output>2047999) sensor_final_output=2047999;
	t=millis_count; while((millis_count-t)<inter_sample_ms){}
	filter_raw_sensor();
}

static void read_sensor_600()
{
	long long adc_data=0;
	bool drdy=0; uint32_t t=millis_count; start();
	do { drdy=checkDataReady(); } while((drdy==0)&&((millis_count-t)<drdy_timeout_ms));
	uint32_t raw=readADC(); if(raw>=0x800000) raw=0;
	adc_data=(long long)(((float)raw)/4096*1000);
	sensor_final_output=adc_data;
	if(sensor_final_output>2047999) sensor_final_output=2047999;
	t=millis_count; while((millis_count-t)<inter_sample_ms){}
	filter_raw_sensor();
}

static void read_sensor_1000()
{
	long long adc_data=0;
	bool drdy=0; uint32_t t=millis_count; start();
	do { drdy=checkDataReady(); } while((drdy==0)&&((millis_count-t)<drdy_timeout_ms));
	uint32_t raw=readADC(); if(raw>=0x800000) raw=0;
	adc_data=(long long)(((float)raw)/4096*1000);
	sensor_final_output=adc_data;
	if(sensor_final_output>2047999) sensor_final_output=2047999;
	t=millis_count; while((millis_count-t)<inter_sample_ms){}
	filter_raw_sensor();
}

void read_sensor()
{
	switch(diag_dr)
	{
		case ADS122C04_DATA_RATE_20SPS:   read_sensor_slow();  break;
		case ADS122C04_DATA_RATE_45SPS:   read_sensor_slow();  break;
		case ADS122C04_DATA_RATE_90SPS:   read_sensor_90();    break;
		case ADS122C04_DATA_RATE_175SPS:  read_sensor_175();   break;
		case ADS122C04_DATA_RATE_330SPS:  read_sensor_330();   break;
		case ADS122C04_DATA_RATE_600SPS:  read_sensor_600();   break;
		case ADS122C04_DATA_RATE_1000SPS: read_sensor_1000();  break;
		default:                          read_sensor_slow();  break;
	}

	str2[0]=(sensor_final_output/1000000)+0x30;
	str2[1]=((sensor_final_output%1000000)/100000)+0x30;
	str2[2]=(((sensor_final_output%1000000)%100000)/10000)+0x30;
	str2[3]=((((sensor_final_output%1000000)%100000)%10000)/1000)+0x30;
	str2[4]='.';
	str2[5]=(((((sensor_final_output%1000000)%100000)%10000)%1000)/100)+0x30;
	str2[6]=((((((sensor_final_output%1000000)%100000)%10000)%1000)%100)/10)+0x30;
	str2[7]=(((((((sensor_final_output%1000000)%100000)%10000)%1000)%100)%10)/1)+0x30;

	sensor_final_output = sensor_final_output - Reference_Set_Value;

	if(sensor_final_output >= 0)
	{
		str3[0]='+';
		str3[1]=(sensor_final_output/1000000)+0x30;
		str3[2]=((sensor_final_output%1000000)/100000)+0x30;
		str3[3]=(((sensor_final_output%1000000)%100000)/10000)+0x30;
		str3[4]=((((sensor_final_output%1000000)%100000)%10000)/1000)+0x30;
		str3[5]='.';
		str3[6]=(((((sensor_final_output%1000000)%100000)%10000)%1000)/100)+0x30;
		str3[7]=((((((sensor_final_output%1000000)%100000)%10000)%1000)%100)/10)+0x30;
		str3[8]=((((((sensor_final_output%1000000)%100000)%10000)%1000)%100)%10)+0x30;
	}
	else
	{
		sensor_final_output=(sensor_final_output)*(-1);
		str3[0]='-';
		str3[1]=(sensor_final_output/1000000)+0x30;
		str3[2]=((sensor_final_output%1000000)/100000)+0x30;
		str3[3]=(((sensor_final_output%1000000)%100000)/10000)+0x30;
		str3[4]=((((sensor_final_output%1000000)%100000)%10000)/1000)+0x30;
		str3[5]='.';
		str3[6]=(((((sensor_final_output%1000000)%100000)%10000)%1000)/100)+0x30;
		str3[7]=((((((sensor_final_output%1000000)%100000)%10000)%1000)%100)/10)+0x30;
		str3[8]=((((((sensor_final_output%1000000)%100000)%10000)%1000)%100)%10)+0x30;
		sensor_final_output=(sensor_final_output)*(-1);
	}
}

void run_mode()
{
	upflag=0; overrange=0;
	offset_value=mem_read(150);
	if(offset_value>100) offset_value=30;

	uint32_t timestamp_ms = 0;

	while(1)
	{
		uint32_t t0 = millis_count;
		read_sensor();
		uint32_t elapsed = millis_count - t0;
		home_online_tx(elapsed, timestamp_ms, diag_dr, diag_pga, diag_gain);
		timestamp_ms += (millis_count - t0);
	}
}

int main(void)
{
	unsigned char i;

	PORTA_DIR    = 0xFF;
	PORTA_OUTSET = ADC_RESET;
	PORTB_DIR    = 0xFF;
	_delay_ms(500);
	InitI2C();
	USART1_init();
	_delay_ms(100);

	timer_init();
	sei();

	limit=mem_read(6);
	_delay_ms(20);

	i=eeprom_read_byte((uint8_t*)(1*160));
	asm("nop");

	Reference_Set_Value=mem_read(220);
	if(Reference_Set_Value>2000000) Reference_Set_Value=0;

	USART1_sendChar('M'); USART1_sendChar('N');
	USART1_sendChar('S'); USART1_sendChar('T');
	USART1_sendChar(0x0D); USART1_sendChar(0x0A);

	setInputMultiplexer(ADS122C04_MUX_AIN1_AIN0);
	setGain(ADS122C04_GAIN_1);
	enablePGA(ADS122C04_PGA_DISABLED);
	setDataRate(ADS122C04_DATA_RATE_1000SPS);   /* change rate here */
	setOperatingMode(ADS122C04_OP_MODE_NORMAL);
	setConversionMode(ADS122C04_CONVERSION_MODE_SINGLE_SHOT);
	setVoltageReference(ADS122C04_VREF_INTERNAL);
	enableInternalTempSensor(ADS122C04_TEMP_SENSOR_OFF);
	setDataCounter(ADS122C04_DCNT_DISABLE);
	setDataIntegrityCheck(ADS122C04_CRC_DISABLED);
	setBurnOutCurrent(ADS122C04_BURN_OUT_CURRENT_OFF);
	setIDACcurrent(ADS122C04_IDAC_CURRENT_OFF);
	setIDAC1mux(ADS122C04_IDAC1_DISABLED);
	setIDAC2mux(ADS122C04_IDAC2_DISABLED);
	_delay_ms(5);

	uint8_t rb0 = 0, rb1 = 0;
	ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &rb0);
	ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &rb1);
	diag_dr   = (rb1 >> 5) & 0x07;
	diag_pga  = rb0 & 0x01;
	diag_gain = (rb0 >> 1) & 0x07;
	set_sample_timing(diag_dr);

	reset();
	_delay_ms(2);
	setDataRate(ADS122C04_Reg.reg1.bit.DR);

	run_mode();
}
