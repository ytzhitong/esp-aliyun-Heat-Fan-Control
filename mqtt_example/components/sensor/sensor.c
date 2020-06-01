#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include "esp_timer.h"

#include "sensor.h"

static const char* TAG = "app sensor";

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_unit_t unit = ADC_UNIT_1;

uint32_t ADC_value_get(adc1_channel_t channel, adc_atten_t atten)
{
	uint32_t adc_reading = 0;

	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(channel, atten);

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
#if debug
    print_char_val_type(val_type);
#endif
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
    	adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    adc_reading /= NO_OF_SAMPLES;
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
#if debug
    printf("Raw: %d \n", adc_reading);
#endif
    return voltage;
}


_aibus_stru aibus_rx_value;
static uint8_t aibus_rx_buff[10];
static char    aibus_tx_buff[10];

#define aibus_read  0x52
#define aibus_write 0x43
#define aibus_addr           1

#define aibus_para_sv   0x00
#define aibus_para_srun 0x1B

#define BUF_SIZE (128)

uint16_t aibus_rx_cmd(uint8_t* rx_buff)
{
	uint16_t check_buf=0;

	aibus_rx_value.PV=(rx_buff[0]|(rx_buff[1]<<8));
	aibus_rx_value.SV=(rx_buff[2]|(rx_buff[3]<<8));
	aibus_rx_value.MV=rx_buff[4];
	aibus_rx_value.A =rx_buff[5];
	aibus_rx_value.PARA =(rx_buff[6]|(rx_buff[7]<<8));
	aibus_rx_value.CHECK=(rx_buff[8]|(rx_buff[9]<<8));

	check_buf=(aibus_rx_value.PV+aibus_rx_value.SV+(aibus_rx_value.MV|(aibus_rx_value.A<<8))+aibus_rx_value.PARA+aibus_addr);

	if(aibus_rx_value.CHECK==check_buf)
	{
		ESP_LOGI(TAG, "PV:%.1f SV:%.1f \n", aibus_rx_value.PV*0.1, aibus_rx_value.SV*0.1);
		return aibus_rx_value.PARA;
	}
	else
		return 0;
}

//addr     :璁惧鍦板潃
//cmd_code :鎸囦护浠ｇ爜锛岃鍙栦负0x52锛屽啓鍏ヤ负0x67
//para_code:鍙傛暟浠ｇ爜
//para_val :鍙傛暟鍊硷紝璇诲彇鎸囦护涓�0
//杩斿洖鍊�              :鍙傛暟鍊硷紝
uint16_t send_aibus_cmd(uint8_t addr,uint8_t cmd_code, uint8_t para_code ,uint16_t para_val)
{
	uint16_t check_buf=0;
	check_buf=para_code;
	check_buf=(check_buf<<8)+cmd_code+para_val+addr;

	aibus_tx_buff[0]=addr+0x80;
	aibus_tx_buff[1]=addr+0x80;
	aibus_tx_buff[2]=cmd_code;
	aibus_tx_buff[3]=para_code;
	aibus_tx_buff[4]=para_val;
	aibus_tx_buff[5]=para_val>>8;
	aibus_tx_buff[6]=check_buf;
	aibus_tx_buff[7]=check_buf>>8;

	uart_write_bytes(UART_NUM_1,  aibus_tx_buff, 8);

	int len =uart_read_bytes(UART_NUM_1, aibus_rx_buff, BUF_SIZE, 200 / portTICK_RATE_MS);

	if(len>=10)
	{
	    return aibus_rx_cmd(aibus_rx_buff);//澶勭悊鎺ユ敹鍒扮殑鏁版嵁
	}
	else
	{
	    return 0xff;
	}
}

void Heater_set_on(bool onoff)
{
	if(onoff==0)
	send_aibus_cmd(aibus_addr,aibus_write, aibus_para_srun ,1);
	else
	send_aibus_cmd(aibus_addr,aibus_write, aibus_para_srun ,0);
}

void Heater_set_sv(double sv)
{
	send_aibus_cmd(aibus_addr,aibus_write, aibus_para_sv,(int)sv*10);
}

void Heater_get_pv(void)
{
	send_aibus_cmd(aibus_addr,aibus_read, aibus_para_sv,0);
}


#define RX485_TXD  (GPIO_NUM_4)
#define RX485_RXD  (GPIO_NUM_16)
#define RX485_RTS  (UART_PIN_NO_CHANGE)
#define RX485_CTS  (UART_PIN_NO_CHANGE)

uint8_t uart1_rx485_init(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, RX485_TXD, RX485_RXD, RX485_RTS, RX485_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    vTaskDelay(2000 / portTICK_RATE_MS);

    Heater_set_on(set_value.HeatSwitch);
    Heater_set_sv(set_value.HeatSV);

    return 0;
}


#define GPIO_OUT_LED_SEL  (1ULL<<GPIO_OUTPUT_LED)| (1ULL<<LEDC_FAN_GPIO)

#define GPIO_INPUT_LED_SEL   1ULL<<GPIO_INPUT_POW

uint16_t flash_gap = 450;

void LED_task(void *pvParameter)
{
    for(; ; )
    {
    	gpio_set_level(GPIO_OUTPUT_LED, 1);
    	vTaskDelay(50 / portTICK_RATE_MS);
    	gpio_set_level(GPIO_OUTPUT_LED, 0);
    	vTaskDelay(flash_gap / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

bool GPIO_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO0
    io_conf.pin_bit_mask = GPIO_OUT_LED_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_LED_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

	gpio_hold_dis(LEDC_FAN_GPIO);
	gpio_deep_sleep_hold_dis();

    gpio_set_level(GPIO_OUTPUT_LED,0);
    gpio_set_level(LEDC_FAN_GPIO,0);

    xTaskCreate(LED_task, "LED_task", 2048, NULL, 5, NULL);

    return gpio_get_level(GPIO_INPUT_POW);
}

#define STEPPER_A    26
#define STEPPER_B    25
#define STEPPER_C    33
#define STEPPER_D    32

#define STEP_A  gpio_set_level(STEPPER_A, 1);gpio_set_level(STEPPER_B, 0);gpio_set_level(STEPPER_C, 0);gpio_set_level(STEPPER_D, 0);
#define STEP_AB gpio_set_level(STEPPER_A, 1);gpio_set_level(STEPPER_B, 1);gpio_set_level(STEPPER_C, 0);gpio_set_level(STEPPER_D, 0);
#define STEP_B  gpio_set_level(STEPPER_A, 0);gpio_set_level(STEPPER_B, 1);gpio_set_level(STEPPER_C, 0);gpio_set_level(STEPPER_D, 0);
#define STEP_BC gpio_set_level(STEPPER_A, 0);gpio_set_level(STEPPER_B, 1);gpio_set_level(STEPPER_C, 1);gpio_set_level(STEPPER_D, 0);
#define STEP_C  gpio_set_level(STEPPER_A, 0);gpio_set_level(STEPPER_B, 0);gpio_set_level(STEPPER_C, 1);gpio_set_level(STEPPER_D, 0);
#define STEP_CD gpio_set_level(STEPPER_A, 0);gpio_set_level(STEPPER_B, 0);gpio_set_level(STEPPER_C, 1);gpio_set_level(STEPPER_D, 1);
#define STEP_D  gpio_set_level(STEPPER_A, 0);gpio_set_level(STEPPER_B, 0);gpio_set_level(STEPPER_C, 0);gpio_set_level(STEPPER_D, 1);
#define STEP_DA gpio_set_level(STEPPER_A, 1);gpio_set_level(STEPPER_B, 0);gpio_set_level(STEPPER_C, 0);gpio_set_level(STEPPER_D, 1);
#define STEP_OFF  gpio_set_level(STEPPER_A, 0);gpio_set_level(STEPPER_B, 0);gpio_set_level(STEPPER_C, 0);gpio_set_level(STEPPER_D, 0);

#define GPIO_OUT_STEPPER_SEL  (1ULL<<STEPPER_A|1ULL<<STEPPER_B|1ULL<<STEPPER_C|1ULL<<STEPPER_D)

_stepper stepper={0,1,0,0};

static void periodic_timer_callback(void* arg)
{
//    int64_t time_since_boot = esp_timer_get_time();
//    ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld us", time_since_boot);
	if(stepper.step_switch==0)
	{
		STEP_OFF
	}
	else
	{
	  switch(stepper.step_index)
	  {
		case 0:STEP_A
			break;
		case 1:STEP_AB
			break;
		case 2:STEP_B
			break;
		case 3:STEP_BC
			break;
		case 4:STEP_C
			break;
		case 5:STEP_CD
			break;
		case 6:STEP_D
			break;
		case 7:STEP_DA
			break;
	  }

	  stepper.step_num--;
	  if(stepper.step_num<=0)
		  stepper.step_switch=0;

	  if(stepper.step_turn==1)//foreward
	  {
		  stepper.step_index++;
		  if(stepper.step_index>7)
			  stepper.step_index=0;
	  }
	  else//
	  {
		  stepper.step_index--;
		  if(stepper.step_index<0)
			  stepper.step_index=7;
	  }
	}

}

void STEPPER_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO0
    io_conf.pin_bit_mask = GPIO_OUT_STEPPER_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 2000));

}

bool FlueSwitch_last=0;

void Flue_set_on(bool FlueSwitch)
{
	if(FlueSwitch!=FlueSwitch_last)
	{
	  if(FlueSwitch==0)
	  {
		  stepper.step_switch=1;
		  stepper.step_turn=0;
		  stepper.step_num=1024;
	  }
	  else
	  {
		  stepper.step_switch=1;
		  stepper.step_turn=1;
		  stepper.step_num=1024;
	  }
	  FlueSwitch_last=FlueSwitch;
	}
}


#define LEDC_FAN_TIMER      LEDC_TIMER_0
#define LEDC_FAN_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_FAN_CHANNEL    LEDC_CHANNEL_0

void FAN_init(void)
{

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_FAN_MODE,           // timer mode
        .timer_num  = LEDC_FAN_TIMER            // timer index
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel={
		.channel    = LEDC_FAN_CHANNEL,
		.duty       = 0,
		.gpio_num   = LEDC_FAN_GPIO,
		.speed_mode = LEDC_FAN_MODE,
		.timer_sel  = LEDC_FAN_TIMER
	};
    ledc_channel_config(&ledc_channel);
}

void FAN_set_speed(bool FanSwitch,uint16_t speed)
{
	if(FanSwitch==0)
	{
	    ledc_set_duty(LEDC_FAN_MODE, LEDC_FAN_CHANNEL, 0);
	    ledc_update_duty(LEDC_FAN_MODE, LEDC_FAN_CHANNEL);
	}
	else
	{
		ledc_set_duty(LEDC_FAN_MODE, LEDC_FAN_CHANNEL, speed*8192/100);
		ledc_update_duty(LEDC_FAN_MODE, LEDC_FAN_CHANNEL);
	}
}

void sleep_enter(void)
{
    const int ext_wakeup_pin_1 = 34;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;

    gpio_set_level(LEDC_FAN_GPIO,0);
	gpio_hold_en(LEDC_FAN_GPIO);
	gpio_deep_sleep_hold_en();

//	esp_deep_sleep(120000000);
	esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask , ESP_EXT1_WAKEUP_ANY_HIGH);

	esp_sleep_enable_timer_wakeup(120000000);

	esp_deep_sleep_start();

}


