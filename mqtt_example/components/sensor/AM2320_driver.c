#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"

#include "AM2320_driver.h"

#include <math.h>

static const char *TAG = "app am2320";

#define GPIO_AM2320_PIN_SEL  (1ULL<<GPIO_AM2320_A)|(1ULL<<GPIO_AM2320_B)|(1ULL<<GPIO_AM2320_C)|(1ULL<<GPIO_AM2320_D)

void AM2320_gpio_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO0
    io_conf.pin_bit_mask = GPIO_AM2320_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(GPIO_AM2320_A,1);
    gpio_set_level(GPIO_AM2320_B,1);
    gpio_set_level(GPIO_AM2320_C,1);
    gpio_set_level(GPIO_AM2320_D,1);
}


uint8_t AM2320_read_byte(gpio_num_t gpio_num)
{
	uint16_t j = 0;
	uint8_t data = 0, bit = 0;

    for(uint8_t i = 0; i < 8; i++)
    {
        // 濠碘槅鍋撻幏閿嬬箾閺夋埈鍎庣紒妤�鍊搁埢搴ㄥ灳閼碱剛校闂佹眹鍨藉褔鏌﹂埡鍛強妞ゆ牗纰嶉崕濠勭磽娴ｈ灏版繛纭锋嫹
        while(!gpio_get_level(gpio_num))
        {
            // 闂傚倸鍟鍫曨敆濞戞瑦浜ゆ繛鎴灻铏叏濠靛鍤欓柟顖氾躬閹娊鏁撻敓锟�
            if(++j>=50000)
            {
                break;
            }
        }
        // 閻庣偣鍊涢崺鏍ь渻娓氱杯n=26us Max70us 闁荤姴鎼悿鍥╂崲閸愵喖鏋侀柣妤�鐗嗙粊锟�"0" 闂佹眹鍔岀�氫即鎮甸锟介幃浠嬫濞戙垺灏�
        ets_delay_us(30);

        // 闂佸憡甯囬崐鏍蓟閸ャ劌顕遍柣妯哄暱婵℃娊鏌涢敐鍐ㄥ鐟滄澘鍊块弻鍛媴鐟欏嫭顔嶉梺纭咁嚃閸犳洜绱為敓锟�
        bit = gpio_get_level(gpio_num);
        j = 0;
        // 缂備焦绋戦ˇ顖滄閻斿壊娈楁俊顖氬悑閺嗏晜顨ラ悙鑼紞缂侇喓鍔戝鍫曟晸閿燂拷
        while(gpio_get_level(gpio_num))
        {
            // 闂傚倸鍟鍫曨敆濞戞瑦浜ゆ繛鎴灻铏叏濠靛鍤欓柟顖氾躬閹娊鏁撻敓锟�
            if(++j >= 50000)
            {
                break;
            }
        }
        data <<= 1;
        data |= bit;
    }
    return data;
}


uint8_t  AM2320_get_value(gpio_num_t gpio_num,uint16_t* hum,int16_t* temp,int16_t* dew)
{
	int i=0;
	int j=0;
//	int a,b;
	uint8_t HumHigh, HumLow, TempHigh, TempLow, TempChecksum, Temp;

	gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
	gpio_set_level(gpio_num, 0);		//鎷変綆1000us
	ets_delay_us(1000);

	gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
//	gpio_set_level(GPIO_AM2320, 1);
	ets_delay_us(30); //閲婃斁鎬荤嚎30us

//	gpio_set_direction(GPIO_AM2320, GPIO_MODE_INPUT);
	ets_delay_us(6); //寤舵椂6us锛岀瓑寰呯ǔ瀹�
    while(gpio_get_level(gpio_num)==0) //鍒ゆ柇浠庢満鏄惁鏈変綆鐢靛钩鍝嶅簲淇″彿
    {
       i++;
       ets_delay_us(1);
       if(i>85)
    	   break;
    }

    ets_delay_us(6); //寤舵椂6us锛岀瓑寰呯ǔ瀹�
    while(gpio_get_level(gpio_num)==1) //鍒ゆ柇浠庢満鏄惁鏈夐珮鐢靛钩鍝嶅簲淇″彿
    {
       j++;
       ets_delay_us(1);
       if(j>85)
    	   break;
    }

    if(i>80||j>80||i<20||j<20) //鍝嶅簲寮傚父
    return 1;

    // 鎺ユ敹鏁版嵁
    HumHigh   = AM2320_read_byte(gpio_num);
    HumLow    = AM2320_read_byte(gpio_num);
    TempHigh  = AM2320_read_byte(gpio_num);
    TempLow   = AM2320_read_byte(gpio_num);
    TempChecksum = AM2320_read_byte(gpio_num);

    Temp = (uint8_t)(HumHigh + HumLow + TempHigh + TempLow);
    if(Temp!=TempChecksum)
    return 2;

    if((TempHigh>>7)==0x01)
    {
    	*temp =-(( (TempHigh&0x07)<<8)+ TempLow);
    }
    else
    {
    	*temp =(( TempHigh<<8)+ TempLow);
    }

    *hum =(( HumHigh<<8 )+ HumLow);

    float t=(float)(*temp)/10;
    float RH=(float)(*hum)/1000;

    float A=((18.678-t/234.5)*t)/(257.14+t);
    float B=log(RH)+A;
    float c=(-257.14)*B;
    float b=18.678-B;
    float a=2/234.5;

    float Tdp=(b-sqrt(b*b+2*a*c))/a;

    *dew=Tdp*10;

    ESP_LOGI(TAG, "t:%.4f RH:%.4f A:%.4f B:%.4f c:%.4f b:%.4f a:%.4f Tdp:%.1f \n",t,RH, A, B, c,b,a,Tdp);

    return 0;
}

