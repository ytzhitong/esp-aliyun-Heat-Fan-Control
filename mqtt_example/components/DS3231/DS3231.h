//#include "sys.h"

#include "esp_err.h"

#ifndef __DS3231_H__
#define __DS3231_H__

#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */


typedef struct
{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint32_t w_year;
	uint8_t  w_month;
	uint8_t  w_date;
	uint8_t  week;

	uint8_t data_str[30];
}_calendar_obj;
extern _calendar_obj calendar;	//閺冦儱宸荤紒鎾寸�担锟�

extern uint8_t const mon_table[12];	//閺堝牅鍞ら弮銉︽埂閺佺増宓佺悰锟�

extern long  TimeStamp;
extern char nowtime[24];

esp_err_t DS3231_i2c_init(void);

void DS3231_Get(void);
//uint8_t RTC_Get_Week(u16 year,uint8_t month,uint8_t day);
void DS3231_Set(uint8_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec);//鐠佸墽鐤嗛弮鍫曟？

#endif
