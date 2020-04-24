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

    gpio_set_level(GPIO_AM2320_A,0);
    gpio_set_level(GPIO_AM2320_B,1);
    gpio_set_level(GPIO_AM2320_C,1);
    gpio_set_level(GPIO_AM2320_D,1);

    vTaskDelay(100 / portTICK_RATE_MS);

    gpio_set_level(GPIO_AM2320_A,1);
    gpio_set_level(GPIO_AM2320_B,1);
    gpio_set_level(GPIO_AM2320_C,1);
    gpio_set_level(GPIO_AM2320_D,1);

    vTaskDelay(400 / portTICK_RATE_MS);
}


uint8_t AM2320_read_byte(gpio_num_t gpio_num)
{
	uint16_t j = 0;
	uint8_t data = 0, bit = 0;

    for(uint8_t i = 0; i < 8; i++)
    {
        // 婵犵妲呴崑鎾诲箯闁垮绠鹃柡澶嬪焾閸庡海绱掑Δ锟介崐鎼佸煝鎼淬劌鐏抽柤纰卞墰鏍￠梻浣圭湽閸ㄨ棄顭囪閺岋箓鍩￠崨顓炲挤濡炪倖鐗楃喊宥夊磿婵犲嫮纾藉ù锝堫潐鐏忕増绻涚涵閿嬪
        while(!gpio_get_level(gpio_num))
        {
            // 闂傚倸鍊搁崯顐㈩嚕閸洦鏁嗘繛鎴炵懄娴溿倖绻涢幋鐏活亜顕ｉ搹顐ｅ弿婵犻潧顭崵娆撴煙椤栨熬韬柟顔藉▕閺佹捇鏁撻敓锟�
            if(++j>=50000)
            {
                break;
            }
        }
        // 闁诲海鍋ｉ崐娑㈠春閺嵮屾富濞撴氨鏉痭=26us Max70us 闂佽崵濮撮幖顐︽偪閸モ晜宕查柛鎰靛枛閺嬩線鏌ｅΔ锟介悧鍡欑矈閿燂拷"0" 闂備焦鐪归崝宀�锟芥矮鍗抽幃鐢割敋閿熶粙骞冩禒瀣棃婵炴垯鍨虹亸锟�
        ets_delay_us(30);

        // 闂備礁鎲＄敮鍥磹閺嶎厼钃熼柛銉ｅ妼椤曢亶鏌ｅΟ鍝勬毐濠碘剝濞婇弻娑㈡晲閸愩劌顬堥悷婊勬緲閸婂潡寮婚崨顔藉閻熸瑥瀚宥夋⒑绾拋鍤冮柛鐘虫礈缁辩偤鏁撻敓锟�
        bit = gpio_get_level(gpio_num);
        j = 0;
        // 缂傚倷鐒︾粙鎴λ囬婊勵偨闁绘柨澹婂▓妤佷繆椤栨艾鎮戦柡鍡忔櫆椤ㄣ儵鎮欓懠顒婄礊缂備緡鍠撻崝鎴濐嚕閸洘鏅搁柨鐕傛嫹
        while(gpio_get_level(gpio_num))
        {
            // 闂傚倸鍊搁崯顐㈩嚕閸洦鏁嗘繛鎴炵懄娴溿倖绻涢幋鐏活亜顕ｉ搹顐ｅ弿婵犻潧顭崵娆撴煙椤栨熬韬柟顔藉▕閺佹捇鏁撻敓锟�
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
	gpio_set_level(gpio_num, 0);		//閹峰缍�1000us
	ets_delay_us(1000);

	gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
//	gpio_set_level(GPIO_AM2320, 1);
	ets_delay_us(30); //闁插﹥鏂侀幀鑽ゅ殠30us

//	gpio_set_direction(GPIO_AM2320, GPIO_MODE_INPUT);
	ets_delay_us(6); //瀵よ埖妞�6us閿涘瞼鐡戝鍛旂�癸拷
    while(gpio_get_level(gpio_num)==0) //閸掋倖鏌囨禒搴㈡簚閺勵垰鎯侀張澶夌秵閻㈤潧閽╅崫宥呯安娣団�冲娇
    {
       i++;
       ets_delay_us(1);
       if(i>85)
    	   break;
    }

    ets_delay_us(6); //瀵よ埖妞�6us閿涘瞼鐡戝鍛旂�癸拷
    while(gpio_get_level(gpio_num)==1) //閸掋倖鏌囨禒搴㈡簚閺勵垰鎯侀張澶愮彯閻㈤潧閽╅崫宥呯安娣団�冲娇
    {
       j++;
       ets_delay_us(1);
       if(j>85)
    	   break;
    }

    if(i>80||j>80||i<20||j<20) //閸濆秴绨插鍌氱埗
    return 1;

    // 閹恒儲鏁归弫鐗堝祦
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

    ESP_LOGI(TAG, "num:%d t:%.4f RH:%.4f A:%.4f B:%.4f c:%.4f b:%.4f a:%.4f Tdp:%.1f \n",gpio_num,t,RH, A, B, c,b,a,Tdp);

    return 0;
}

