/* 
 * File:   main_2.c
 * Author: mokhtar
 *
 * Created on March 22, 2024, 9:44 PM
 */


#include"main_2.h"
#include "ECU_layer/button/ecu_button.h"
#include "MKAL_layer/I2C/hal_i2c.h"


void timer1_interrupt(void);
void mssp_I2C_DefaultInterruptHandler(void);
uint8 transfer_0x_d (uint8 num);
 
#define SLAVE_ADDRESS 0x60
/********************************************* 7 SEGMANT ***********************************************************/
segment_t segment={.seg_status=seg_common_anode,.seg_pins[0].port=PORTD_INDEX,.seg_pins[0].pin=pin0,
                   .seg_pins[0].direction=GPIO_OUTPUT,.seg_pins[0].logic=GPIO_LOW,
                   .seg_pins[1].port=PORTD_INDEX,.seg_pins[1].pin=pin1,
                   .seg_pins[1].direction=GPIO_OUTPUT,.seg_pins[1].logic=GPIO_LOW,
                   .seg_pins[2].port=PORTD_INDEX,.seg_pins[2].pin=pin2,
                   .seg_pins[2].direction=GPIO_OUTPUT,.seg_pins[2].logic=GPIO_LOW,
                   .seg_pins[3].port=PORTD_INDEX,.seg_pins[3].pin=pin3,
                   .seg_pins[3].direction=GPIO_OUTPUT,.seg_pins[3].logic=GPIO_LOW,};
/********************************************* PINS ***********************************************************/
pin_config_t pin_1={.port=PORTD_INDEX,.pin=pin4,.direction=GPIO_OUTPUT,.logic=GPIO_LOW};
pin_config_t pin_2={.port=PORTD_INDEX,.pin=pin5,.direction=GPIO_OUTPUT,.logic=GPIO_LOW};
pin_config_t pin_3={.port=PORTD_INDEX,.pin=pin6,.direction=GPIO_OUTPUT,.logic=GPIO_LOW};
pin_config_t pin_4={.port=PORTD_INDEX,.pin=pin7,.direction=GPIO_OUTPUT,.logic=GPIO_LOW};
pin_config_t pin_5={.port=PORTC_INDEX,.pin=pin0,.direction=GPIO_OUTPUT,.logic=GPIO_LOW};
pin_config_t pin_6={.port=PORTC_INDEX,.pin=pin2,.direction=GPIO_OUTPUT,.logic=GPIO_LOW};
pin_config_t led_room_3={.port=PORTC_INDEX,.pin=pin5,.direction=GPIO_OUTPUT,.logic=GPIO_LOW};
relay_t relay={.port=PORTC_INDEX,.pin=pin1,.relay_status=relay_state_off};
/********************************************* TIMER1 ***********************************************************/
Timer1_t Timer1={.Timer1_handler=timer1_interrupt,.timer1_mode=TIMER1_source_internal,.timer1_counter_mode=TIMER1_Asynchronize_clock_input,
.timer1_osc_cfg=TIMER1_oscillator_disable,.timer1_preload_value=22786,.timer1_prescaler_cfg=prescale_value_timer1_div_by_8
,.timer1_wr__reg_cfg=TIMER1_16bit_cfg};
/********************************************* TIMER2 ***********************************************************/
Timer2_t Timer2 ={.Timer2_handler=NULL,
                    .timer1_preload_value=0,.timer2_postscaler_cfg=Postscale_div_by_1,
                    .timer2_prescaler_cfg=prescale_div_by_1};
/********************************************* I2C ***********************************************************/
mssp_i2c_t mssp_i2c={.I2C_DefaultInterruptHandler=mssp_I2C_DefaultInterruptHandler,.i2c_clock=100000,
.i2c_cfg.i2c_SMBus_control=I2C_SMBus_DISABLE,.i2c_cfg.i2c_general_call=I2C_GENERAL_CALL_ENABLE,.i2c_cfg.i2c_mode=I2C_MSSP_SLAVE_MODE
,.i2c_cfg.i2c_mode_cfg=I2C_SLAVE_MODE_7BIT_ADDRESS,.i2c_cfg.i2c_slave_address=SLAVE_ADDRESS,
.i2c_cfg.i2c_slew_rate=I2C_SLEW_RATE_DISABLE,};
/********************************************* CCP ***********************************************************/
ccp2_t ccp2_={.CCP2_handler = NULL ,.ccp2_mode=ccp2_pwm_initialize,.ccp2_pwm_frequence=20000,.pin_config.port=PORTC_INDEX,
.pin_config.pin=pin1,.pin_config.direction=GPIO_OUTPUT,.pin_config.logic=GPIO_LOW,.ccp2_timer2_postscaler_cfg=ccp2_Postscale_div_by_1,
.ccp2_timer2_prescaler_cfg=ccp1_prescale_div_by_1,.ccp2_mode_variant=ccp1_pwm_initialize};
/********************************************* ADC ***********************************************************/
ADC_conf_t ADC_ = {.ADC_CHANNAL_SELECT=ADC_CHANNEL_AN0,.ADC_interruptHandler=NULL,.Acquisition_time=ADC_12_TAD,
.Conversion_Clock=FOSC_div_16,.result_format=ADC_result_right,.voltage_ref=voltage_ref_disable};


Std_ReturnType ret = E_NOT_OK;
//I2C
volatile uint8 ret_i2c_data_1;
//ADC
volatile adc_result_t ADC_DATA_RET ;
//ccp
uint8 data_dev;

uint8 couinter =0 ;
uint8 couinter1  ;
uint8 couinter2  ;
uint8 data[3];

uint8 timer2_counter;
//time
uint8 seconds  =5;
uint8 minutes  =5;
volatile uint8 hours   =14 ;
int main() {

    intitialized_functions();
    while(1)
    {
        
        ret = ADC_get_conversion_blocking(&ADC_,ADC_CHANNEL_AN0,&ADC_DATA_RET);
        
        /* timer */
        ret = segment_wright_logic(&segment,(hours/10));
        ret = GPIO_pIN_WRITE_LOGIC(&pin_1,GPIO_HIGH);
        __delay_us(40);
        ret = GPIO_pIN_WRITE_LOGIC(&pin_1,GPIO_LOW);
        ret = segment_wright_logic(&segment,(hours%10));
        ret = GPIO_pIN_WRITE_LOGIC(&pin_2,GPIO_HIGH);
        __delay_us(40);
        ret = GPIO_pIN_WRITE_LOGIC(&pin_2,GPIO_LOW);
        ret = segment_wright_logic(&segment,(minutes/10));
        ret = GPIO_pIN_WRITE_LOGIC(&pin_3,GPIO_HIGH);
        __delay_us(40);
        ret = GPIO_pIN_WRITE_LOGIC(&pin_3,GPIO_LOW);
        ret = segment_wright_logic(&segment,(minutes%10));
        ret = GPIO_pIN_WRITE_LOGIC(&pin_4,GPIO_HIGH);
        __delay_us(40);
        ret = GPIO_pIN_WRITE_LOGIC(&pin_4,GPIO_LOW);
        ret = segment_wright_logic(&segment,(seconds/10));
        ret = GPIO_pIN_WRITE_LOGIC(&pin_5,GPIO_HIGH);
        __delay_us(40);
        ret = GPIO_pIN_WRITE_LOGIC(&pin_5,GPIO_LOW);
        ret = segment_wright_logic(&segment,(seconds%10));
        ret = GPIO_pIN_WRITE_LOGIC(&pin_6,GPIO_HIGH);
        __delay_us(40);
        ret = GPIO_pIN_WRITE_LOGIC(&pin_6,GPIO_LOW);
        
        
        if(ret_i2c_data_1 == 'a'){
            GPIO_pIN_WRITE_LOGIC(&led_room_3,GPIO_HIGH);
            relay_turn_on(&relay);
              data_dev =((uint32)ADC_DATA_RET*100/1022);
            ccp2_pwm_set_duty(100 - data_dev);
            CCP2_select_statues(ccp2_pwm_initialize);
        }else{
            GPIO_pIN_WRITE_LOGIC(&led_room_3,GPIO_LOW);
            relay_turn_off(&relay);
            CCP2_select_statues(ccp2_module_disabled);
        }
    }
       
    return (EXIT_SUCCESS);
} 

void intitialized_functions(void){
    ret = segment_intialized(&segment);
    ret = Timer1_init(&Timer1);
    ret = GPIO_pIN_INTIALIZE(&pin_1);
    ret = GPIO_pIN_INTIALIZE(&pin_2);
    ret = GPIO_pIN_INTIALIZE(&pin_3);
    ret = GPIO_pIN_INTIALIZE(&pin_4);
    ret = GPIO_pIN_INTIALIZE(&pin_5);
    ret = relay_intialize(&relay);
    ret = GPIO_pIN_INTIALIZE(&led_room_3);
    ret = GPIO_pIN_INTIALIZE(&pin_6);
    ret = MSSP_I2C_Init(&mssp_i2c);
    ret = Timer2_init(&Timer2);
    ret = ccp2_init(&ccp2_);
    ret = ADC_inti(&ADC_);
}

void timer1_interrupt(void){
    couinter++;
    if(couinter == 2){
        couinter =0 ;
        seconds++;
        if(seconds == 60){
            seconds = 0;
            minutes++;
            if(minutes == 60){
                minutes = 0;
                hours++;
                if(hours == 24){
                    hours = 0;
                }
            }
        }
    }
}

 void mssp_I2C_DefaultInterruptHandler(void){
    // holds clock low (clock stretch) 
     couinter1++;
     couinter2++;
    I2C_CLOCK_STRETCH_ENABLE();
    if((SSPSTATbits.D_nA == 0) && (SSPSTATbits.R_NOT_W == 0 )){
        uint8 dummy_buffer =SSPBUF;
        while(!SSPSTATbits.BF);
        ret_i2c_data_1 =SSPBUF;
    }
    I2C_CLOCK_STRETCH_DISABLE();
}
 
 
 uint8 transfer_0x_d (uint8 num){
    uint8 l_bit = num%10;
    uint8 h_bit = num/10;
    l_bit = l_bit*1;
    h_bit = h_bit*16;
    uint8 value = l_bit+h_bit;
    if(value < 10){
        
    }else if((value >= 10) && (value > 20)){
        value = value -6;
    }else if((value >= 20) && (value > 30)){
        value = value -12;
    }
    return (value);
}
