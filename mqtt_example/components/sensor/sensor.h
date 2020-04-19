#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "driver/adc.h"
#include "esp_adc_cal.h"

typedef struct
{
 int16_t  SV;
 int16_t  PV;
 uint8_t  MV;
 uint8_t  A;
 int16_t  PARA;
 uint16_t CHECK;

}_aibus_stru;

typedef struct
{
	uint16_t cnt;
	uint16_t time_h;
	uint16_t time_l;

	int16_t  tempA;
	uint16_t humA;
	int16_t  dewA;
	int16_t  tempB;
	uint16_t humB;
	int16_t  dewB;
	int16_t  tempC;
	uint16_t humC;
	int16_t  dewC;
	int16_t  tempD;
	uint16_t humD;
	int16_t  dewD;

	uint16_t volt;
} _sensor_value;

typedef struct
{
	float  tempA;
	float  humA;
	float  dewA;
	float  tempB;
	float  humB;
	float  dewB;
	float  tempC;
	float  humC;
	float  dewC;
	float  tempD;
	float  humD;
	float  dewD;

	uint16_t volt;
} _sensor_value_f;

extern _sensor_value   sensor,sensor_store_read;
extern _sensor_value_f sensor_f;

typedef struct
{
	bool  HeatSwitch;
    float HeatSV;
	bool  FanSwitch;
    int   FanSpeed;
    bool  FlueSwitch;
    int   Interval;
} _set_value;

extern _set_value    set_value;

void sensor_get(void);

uint32_t ADC_value_get(adc1_channel_t channel, adc_atten_t atten);

uint8_t uart1_rx485_init(void);

void Heater_set_on(bool onoff);
void Heater_set_sv(double sv);

void FAN_init(void);
void FAN_set_speed(bool FanSwitch,uint16_t speed);

void STEPPER_init(void);
void Flue_set_on(bool FlueSwitch);

bool GPIO_init(void);

#endif
