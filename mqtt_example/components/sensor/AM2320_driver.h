#ifndef _AM2320_DRIVE_H_
#define _AM2320_DRIVE_H_

#define GPIO_AM2320_A    19
#define GPIO_AM2320_B    21
#define GPIO_AM2320_C    22
#define GPIO_AM2320_D    23

uint8_t  AM2320_get_value(gpio_num_t gpio_num,uint16_t* hum,short* temp,short* dew);
// esp_err_t am2320_i2c_init(void);

void AM2320_gpio_init(void);

#endif
