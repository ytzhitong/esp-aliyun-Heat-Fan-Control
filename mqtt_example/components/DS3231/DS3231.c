#include <time.h>
#include <string.h>

#include "DS3231.h"
#include "driver/i2c.h"

//#include "BH1750_driver.h"

_calendar_obj calendar;

#define DS3231_WriteAddress 0xD0
#define DS3231_ReadAddress  0xD1

uint8_t BCD2HEX(uint8_t val)
{
    uint8_t i;
    i= val&0x0f;
    val >>= 4;
    val &= 0x0f;
    val *= 10;
    i += val;

    return i;
}

uint16_t B_BCD(uint8_t val)
{
  uint8_t i,j,k;
  i=val/10;
  j=val%10;
  k=j+(i<<4);
  return k;
}


#define I2C_DS3231_MASTER_SCL_IO          27                /*!< gpio number for I2C master clock */
#define I2C_DS3231_MASTER_SDA_IO          14               /*!< gpio number for I2C master data  */
#define I2C_DS3231_MASTER_NUM             I2C_NUM_0        /*!< I2C port number for master dev */

#define I2C_MASTER_FREQ_HZ 10000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */


#define DS3231_SENSOR_ADDR                 0xD0


/**
 * @brief i2c master initialization
 */
esp_err_t DS3231_i2c_init(void)
{
    int i2c_master_port = I2C_DS3231_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_DS3231_MASTER_SDA_IO;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = I2C_DS3231_MASTER_SCL_IO;
    conf.scl_pullup_en = 0;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}


static esp_err_t DS3231ByteWrite( uint8_t addr, uint8_t data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DS3231_SENSOR_ADDR  | WRITE_BIT, ACK_CHECK_EN);

    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);

    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_DS3231_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

uint8_t DS3231ByteRead( uint8_t addr)
{
    int ret;
	uint8_t temp;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DS3231_SENSOR_ADDR  | WRITE_BIT, ACK_CHECK_EN);

    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DS3231_SENSOR_ADDR  | READ_BIT, ACK_CHECK_EN);

    i2c_master_read_byte(cmd, &temp, ACK_CHECK_EN);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_DS3231_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return temp;
}

long  TimeStamp;
char nowtime[24];

void DS3231_Get(void)
{
	struct tm stm;
	struct tm *p;

	calendar.w_year=DS3231ByteRead(0x06);
	calendar.w_year=BCD2HEX(calendar.w_year);

	calendar.w_month=DS3231ByteRead(0x05);
	calendar.w_month=BCD2HEX(calendar.w_month);

	calendar.w_date=DS3231ByteRead(0x04);
	calendar.w_date=BCD2HEX(calendar.w_date);

	calendar.hour=DS3231ByteRead(0x02);
	calendar.hour&=0x3f;
	calendar.hour=BCD2HEX(calendar.hour);

	calendar.min=DS3231ByteRead(0x01);
	calendar.min=BCD2HEX(calendar.min);

	calendar.sec=DS3231ByteRead(0x00);
	calendar.sec=BCD2HEX(calendar.sec);

    sprintf((char *)calendar.data_str,"20%02d-%02d-%02d %02d:%02d:%02d \n",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);

    stm.tm_year=calendar.w_year+100;
    stm.tm_mon=calendar.w_month-1;
    stm.tm_mday=calendar.w_date;
    stm.tm_hour=calendar.hour;
    stm.tm_min=calendar.min;
    stm.tm_sec=calendar.sec;

    TimeStamp=mktime(&stm);
//    printf("TimeStamp=%ld \n",TimeStamp);


    //char nowtime[24];
    memset(nowtime, 0, sizeof(nowtime));
    p=localtime(&TimeStamp);
    strftime(nowtime, 24, "%Y-%m-%d %H:%M:%S", p);
//    printf("nowtime = %s\n", nowtime);
}

void DS3231_Set(uint8_t yea,uint8_t mon,uint8_t da,uint8_t hou,uint8_t min,uint8_t sec)
{
  uint8_t temp=0;

  temp=B_BCD(yea);
  DS3231ByteWrite(0x06,temp);

  temp=B_BCD(mon);
  DS3231ByteWrite(0x05,temp);

  temp=B_BCD(da);
  DS3231ByteWrite(0x04,temp);

  temp=B_BCD(hou);
  DS3231ByteWrite(0x02,temp);

  temp=B_BCD(min);
  DS3231ByteWrite(0x01,temp);

  temp=B_BCD(sec);
  DS3231ByteWrite(0x00,temp);
}


