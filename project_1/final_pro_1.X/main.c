/* 
 * File:   main.h
 * Author: mokhtar
 *
 * Created on March 22, 2024, 9:23 PM
 */

#include"main.h"
#include "ECU_layer/button/ecu_button.h"
#include "MKAL_layer/I2C/hal_i2c.h"

void into_interrupt (void);
void int1_interrupt (void);
void timer1_interrupt (void);
void interrupt_usart_rx(void);
/********************************************* ADC ***********************************************************/
ADC_conf_t ADC_ = {.ADC_CHANNAL_SELECT=ADC_CHANNEL_AN0,.ADC_interruptHandler=NULL,.Acquisition_time=ADC_12_TAD,
.Conversion_Clock=FOSC_div_16,.result_format=ADC_result_right,.voltage_ref=voltage_ref_disable};
/********************************************* I2C ***********************************************************/
mssp_i2c_t mssp_i2c = {.i2c_clock=100000,.i2c_cfg.i2c_SMBus_control=I2C_SMBus_DISABLE,
.i2c_cfg.i2c_general_call=I2C_GENERAL_CALL_DISABLE,.i2c_cfg.i2c_master_rec_mode=I2C_MASTER_RECEIVE_DISABLE,
.i2c_cfg.i2c_mode=I2C_MSSP_MASTER_MODE,.i2c_cfg.i2c_mode_cfg=I2C_MASTER_MODE_DEFINED_CLOCK,
.I2C_DefaultInterruptHandler=NULL,.I2C_Report_Receive_Overflow=NULL,.I2C_Report_Write_Collision=NULL};
/********************************************* LCD ***********************************************************/
chr_lcd_4bit_t lcd_4bit = {.lcd_en.port=PORTD_INDEX,.lcd_en.pin=pin5,.lcd_en.direction=GPIO_OUTPUT,.lcd_en.logic=GPIO_LOW,
.lcd_rs.port=PORTD_INDEX,.lcd_rs.pin=pin6,.lcd_rs.direction=GPIO_OUTPUT,.lcd_rs.logic=GPIO_LOW,
.lcd_data[0].port=PORTD_INDEX,.lcd_data[0].pin=pin0,.lcd_data[0].direction=GPIO_OUTPUT,.lcd_data[0].logic=GPIO_LOW,
.lcd_data[1].port=PORTD_INDEX,.lcd_data[1].pin=pin1,.lcd_data[1].direction=GPIO_OUTPUT,.lcd_data[1].logic=GPIO_LOW,
.lcd_data[2].port=PORTD_INDEX,.lcd_data[2].pin=pin2,.lcd_data[2].direction=GPIO_OUTPUT,.lcd_data[2].logic=GPIO_LOW,
.lcd_data[3].port=PORTD_INDEX,.lcd_data[3].pin=pin3,.lcd_data[3].direction=GPIO_OUTPUT,.lcd_data[3].logic=GPIO_LOW};
/********************************************* pins ***********************************************************/
pin_config_t LED_ROOM_1 = {.port=PORTC_INDEX,.pin=pin0,.logic=GPIO_LOW,.direction=GPIO_OUTPUT};
pin_config_t leds_1 = {.port=PORTD_INDEX,.pin=pin4,.logic=GPIO_LOW,.direction=GPIO_OUTPUT};
/********************************************* INT ***********************************************************/
INTERRUPT_INTX_t INT_0 ={.EXT_INTERRUPTHANDLE =into_interrupt,.edge=INTERRUPT_RISING_EDGE,.priority=INTERRUPT_HUGH_PRIORITY,
.sorce=INTERRUPT_INT0,.pin_config.pin=pin0,.pin_config.port=PORTB_INDEX,.pin_config.direction=GPIO_INPUT,.pin_config.logic=GPIO_LOW};
INTERRUPT_INTX_t INT_1 ={.EXT_INTERRUPTHANDLE =int1_interrupt,.edge=INTERRUPT_RISING_EDGE,.priority=INTERRUPT_HUGH_PRIORITY,
.sorce=INTERRUPT_INT1,.pin_config.pin=pin1,.pin_config.port=PORTB_INDEX,.pin_config.direction=GPIO_INPUT,.pin_config.logic=GPIO_LOW};
/********************************************* timer3 ***********************************************************/
Timer1_t Timer1={.Timer1_handler=timer1_interrupt,.timer1_mode=TIMER1_source_internal,.timer1_counter_mode=TIMER1_Asynchronize_clock_input,
.timer1_osc_cfg=TIMER1_oscillator_disable,.timer1_preload_value=3036,.timer1_prescaler_cfg=prescale_value_timer1_div_by_8
,.timer1_wr__reg_cfg=TIMER1_16bit_cfg};
/********************************************* MOTORS ***********************************************************/
motor_t motor_1 = {.motor_pin_config[0].port=PORTC_INDEX,.motor_pin_config[0].pin=pin5,.motor_pin_config[0].direction=GPIO_OUTPUT,
.motor_pin_config[1].port=PORTC_INDEX,.motor_pin_config[1].pin=pin2,.motor_pin_config[0].direction=GPIO_OUTPUT};
motor_t motor_2 = {.motor_pin_config[0].port=PORTC_INDEX,.motor_pin_config[0].pin=pin1,.motor_pin_config[0].direction=GPIO_OUTPUT,
.motor_pin_config[1].port=PORTD_INDEX,.motor_pin_config[1].pin=pin7,.motor_pin_config[0].direction=GPIO_OUTPUT};
/********************************************* USART ***********************************************************/
usart_t usart_1 ={
   .baudrate= 2404,.buadrate_gen=BAUDRATE_ASY_8BIT_LOW_SPEED ,.usart_tx.usart_tx_enable=USART_ASYNCHRONOUS_TX_ENABLE,
   .usart_tx.usart_tx_interrupt_enable=USART_ASYNCHRONOUS_INTERRUPT_TX_ENABLE,
   .usart_tx.usart_tx_9bits_enable=USART_ASYNCHRONOUS_9BITS_TX_DISABLE,.usart_rx.usart_rx_enable=USART_ASYNCHRONOUS_RX_ENABLE,
   .usart_rx.usart_rx_interrupt_enable=USART_ASYNCHRONOUS_INTERRUPT_RX_ENABLE,
   .usart_rx.usart_rx_9bits_enable=USART_ASYNCHRONOUS_9BITS_RX_DISABLE,.USART_RX_INTERRUPT_HANDLER=interrupt_usart_rx
};
/********************************************* CCP ***********************************************************/
ccp1_t ccp1_={.CCP1_handler = NULL ,.ccp1_mode=ccp1_pwm_initialize,.pwm_frequence=10000,.pin_config.port=PORTC_INDEX,
.pin_config.pin=pin2,.pin_config.direction=GPIO_OUTPUT,.timer2_postscaler_cfg=ccp1_Postscale_div_by_1,
.timer2_prescaler_cfg=ccp1_prescale_div_by_1,.ccp1_mode_variant=ccp1_pwm_initialize};
ccp2_t ccp2_={.CCP2_handler = NULL ,.ccp2_mode=ccp2_pwm_initialize,.ccp2_pwm_frequence=10000,.pin_config.port=PORTC_INDEX,
.pin_config.pin=pin1,.pin_config.direction=GPIO_OUTPUT,.pin_config.logic=GPIO_LOW,.ccp2_timer2_postscaler_cfg=ccp2_Postscale_div_by_1,
.ccp2_timer2_prescaler_cfg=ccp1_prescale_div_by_1,.ccp2_mode_variant=ccp1_pwm_initialize};
/********************************************* timer2 ***********************************************************/
Timer2_t Timer2 ={.Timer2_handler=NULL,
                    .timer1_preload_value=0,.timer2_postscaler_cfg=Postscale_div_by_1,
                    .timer2_prescaler_cfg=prescale_div_by_1};




// ADC VARIABLE 
adc_result_t ADC_VALUE ;
// I2C VARIABLE
volatile uint8 i2c_ack ;
volatile uint8 ds13_data[6];
volatile uint8 ds13_data_decimal[6];
volatile uint8 tc74 ;
volatile uint8 tc74_ref ;

uint8 addd = 0 ;
// TIMER 1
volatile uint8 timer1_counter = 0 ;

volatile uint8 counter ;

volatile uint8 led_ret ;

// USART 
volatile uint8 usart_data[7];
uint8 usart_counter;
//control project
uint8 control_project ;
Std_ReturnType ret = E_NOT_OK;
int main() {
    /* usart initialize  */
    
        
   
    intitialized_functions();
    ret = lcd_4bit_send_string_pos(&lcd_4bit,1,5,"author house ");
    ret = lcd_4bit_send_string_pos(&lcd_4bit,2,2,"date : ");
    ret = lcd_4bit_send_string_pos(&lcd_4bit,3,2,"temperature : ");
    ret = lcd_4bit_send_string_pos(&lcd_4bit,4,2,"time : ");
    //ret = lcd_4bit_send_string_pos(&lcd_4bit,4,2,"HOURS : ");
    
    
    ret = usart_ASYNC_write_byte_blocking('E');
    ret = usart_ASYNC_write_byte_blocking('N');
    ret = usart_ASYNC_write_byte_blocking('T');
    ret = usart_ASYNC_write_byte_blocking('E');
    ret = usart_ASYNC_write_byte_blocking('R');
    ret = usart_ASYNC_write_byte_blocking(' ');
    ret = usart_ASYNC_write_byte_blocking('P');
    ret = usart_ASYNC_write_byte_blocking('A');
    ret = usart_ASYNC_write_byte_blocking('S');
    ret = usart_ASYNC_write_byte_blocking('S');
    ret = usart_ASYNC_write_byte_blocking('W');
    ret = usart_ASYNC_write_byte_blocking('O');
    ret = usart_ASYNC_write_byte_blocking('R');
    ret = usart_ASYNC_write_byte_blocking('D');
    ret = usart_ASYNC_write_byte_blocking(' ');

    ds1307_read_values(&mssp_i2c,DS13_ADDRESS,ds13_data,&i2c_ack);
    MSSP_I2C_Master_call_slave(&mssp_i2c,0x60,ds13_data[0],&i2c_ack);
    MSSP_I2C_Master_call_slave(&mssp_i2c,0x60,ds13_data[1],&i2c_ack);
    MSSP_I2C_Master_call_slave(&mssp_i2c,0x60,ds13_data[2],&i2c_ack);
    while(1)
    {
        /* room 1 led */
        ret = ADC_get_conversion_blocking(&ADC_,ADC_CHANNEL_AN0,&ADC_VALUE);
        if(ADC_VALUE > 512){
            ret = GPIO_pIN_WRITE_LOGIC(&LED_ROOM_1,GPIO_HIGH);
        }else if(ADC_VALUE < 510){
            ret = GPIO_pIN_WRITE_LOGIC(&LED_ROOM_1,GPIO_LOW);
        }
        
        /* date and time  */
        ds1307_read_values(&mssp_i2c,DS13_ADDRESS,ds13_data,&i2c_ack);
        for(uint8 i =0 ; i<6 ;i++){
            ds13_data_decimal[i]=ds13_data[i];
        }
        
        /* convert to string */
        if((ds13_data_decimal[1] < 10) ){
            
        }else if((ds13_data_decimal[1] >= 16) && (ds13_data_decimal[1] < 32)){
            ds13_data_decimal[1] = ds13_data_decimal[1] - 6;
        }else if((ds13_data_decimal[1] >= 32) && (ds13_data_decimal[1] < 48)){
            ds13_data_decimal[1] = ds13_data_decimal[1] - (6*2);
        }else if((ds13_data_decimal[1] >= 48) && (ds13_data_decimal[1] < 64)){
            ds13_data_decimal[1] = ds13_data_decimal[1] - (6*3);
        }else if((ds13_data_decimal[1] >= 64) && (ds13_data_decimal[1] < 80)){
            ds13_data_decimal[1] = ds13_data_decimal[1] - 6*4;
        }else if((ds13_data_decimal[1] >= 80) && (ds13_data_decimal[1] < 96)){
            ds13_data_decimal[1] = ds13_data_decimal[1] - 6*5;
        }
        
        
        if((ds13_data_decimal[2] < 10) ){
            
        }else if((ds13_data_decimal[2] >= 0x10) && (ds13_data_decimal[2] < 0x20)){
            ds13_data_decimal[2] = ds13_data_decimal[2] - 6;
        }else if((ds13_data_decimal[2] >= 0x20) && (ds13_data_decimal[2] < 0x30)){
            ds13_data_decimal[2] = ds13_data_decimal[2] - 6*2;
        }else if((ds13_data_decimal[2] >= 0x30) && (ds13_data_decimal[2] < 0x40)){
            ds13_data_decimal[2] = ds13_data_decimal[2] - 6*3;
        }else if((ds13_data_decimal[2] >= 0x40) && (ds13_data_decimal[2] < 0x50)){
            ds13_data_decimal[2] = ds13_data_decimal[2] - 6*4;
        }else if((ds13_data_decimal[2] >= 0x50) && (ds13_data_decimal[2] < 0x60)){
            ds13_data_decimal[2] = ds13_data_decimal[2] - 6*5;
        }
        
        
        if((ds13_data_decimal[3] < 10) ){
            
        }else if((ds13_data_decimal[3] >= 0x10) && (ds13_data_decimal[3] < 0x20)){
            ds13_data_decimal[3] = ds13_data_decimal[3] - 6;
        }else if((ds13_data_decimal[3] >= 0x20) && (ds13_data_decimal[3] < 0x30)){
            ds13_data_decimal[3] = ds13_data_decimal[3] - 12;
        }else if((ds13_data_decimal[3] >= 0x30) && (ds13_data_decimal[3] < 0x40)){
            ds13_data_decimal[3] = ds13_data_decimal[3] - 18;
        }else if((ds13_data_decimal[3] >= 0x40) && (ds13_data_decimal[3] < 0x50)){
            ds13_data_decimal[3] = ds13_data_decimal[3] - 24;
        }else if((ds13_data_decimal[3] >= 0x50) && (ds13_data_decimal[3] < 0x60)){
            ds13_data_decimal[3] = ds13_data_decimal[3] - 30;
        }
        
        
        if((ds13_data_decimal[4] < 10) ){
            
        }else if((ds13_data_decimal[4] >= 0x10) && (ds13_data_decimal[4] < 0x20)){
            ds13_data_decimal[4] = ds13_data_decimal[4] - 6;
        }else if((ds13_data_decimal[4] >= 0x20) && (ds13_data_decimal[4] < 0x30)){
            ds13_data_decimal[4] = ds13_data_decimal[4] - 12;
        }else if((ds13_data_decimal[4] >= 0x30) && (ds13_data_decimal[4] < 0x40)){
            ds13_data_decimal[4] = ds13_data_decimal[4] - 18;
        }else if((ds13_data_decimal[4] >= 0x40) && (ds13_data_decimal[4] < 0x50)){
            ds13_data_decimal[4] = ds13_data_decimal[4] - 24;
        }else if((ds13_data_decimal[4] >= 0x50) && (ds13_data_decimal[4] < 0x60)){
            ds13_data_decimal[4] = ds13_data_decimal[4] - 30;
        }
        
         if((ds13_data_decimal[0] < 10) ){
            
        }else if((ds13_data_decimal[0] >= 0x10) && (ds13_data_decimal[0] < 0x20)){
            ds13_data_decimal[0] = ds13_data_decimal[0] - 6;
        }else if((ds13_data_decimal[0] >= 0x20) && (ds13_data_decimal[0] < 0x30)){
            ds13_data_decimal[0] = ds13_data_decimal[0] - 12;
        }else if((ds13_data_decimal[0] >= 0x30) && (ds13_data_decimal[0] < 0x40)){
            ds13_data_decimal[0] = ds13_data_decimal[0] - 18;
        }else if((ds13_data_decimal[0] >= 0x40) && (ds13_data_decimal[0] < 0x50)){
            ds13_data_decimal[0] = ds13_data_decimal[0] - 24;
        }else if((ds13_data_decimal[0] >= 0x50) && (ds13_data_decimal[0] < 0x60)){
            ds13_data_decimal[0] = ds13_data_decimal[0] - 30;
        }
        
         if((ds13_data_decimal[5] < 10) ){
            
        }else if((ds13_data_decimal[5] >= 0x10) && (ds13_data_decimal[5] < 0x20)){
            ds13_data_decimal[5] = ds13_data_decimal[5] - 6;
        }else if((ds13_data_decimal[5] >= 0x20) && (ds13_data_decimal[5] < 0x30)){
            ds13_data_decimal[5] = ds13_data_decimal[5] - 12;
        }else if((ds13_data_decimal[5] >= 0x30) && (ds13_data_decimal[5] < 0x40)){
            ds13_data_decimal[5] = ds13_data_decimal[5] - 18-48;
        }else if((ds13_data_decimal[5] >= 0x40) && (ds13_data_decimal[5] < 0x50)){
            ds13_data_decimal[5] = ds13_data_decimal[5] - 24;
        }else if((ds13_data_decimal[5] >= 0x50) && (ds13_data_decimal[5] < 0x60)){
            ds13_data_decimal[5] = ds13_data_decimal[5] - 30;
        }
        
        /* send string to lcd  */
        ret = convert_uint8_to_string(ds13_data_decimal[3],&addd);
        ret = lcd_4bit_send_string_pos(&lcd_4bit,2,8,&addd);
        ret = lcd_4bit_send_char_data_pos(&lcd_4bit,2,11,'/');
        ret = convert_uint8_to_string(ds13_data_decimal[4],&addd);
        ret = lcd_4bit_send_string_pos(&lcd_4bit,2,12,&addd);
        ret = lcd_4bit_send_char_data_pos(&lcd_4bit,2,14,'/');
        ret = convert_uint8_to_string(ds13_data_decimal[5],&addd);
        ret = lcd_4bit_send_string_pos(&lcd_4bit,2,15,&addd);
        
        
        
        ret = convert_uint8_to_string(ds13_data_decimal[0],&addd);
        ret = lcd_4bit_send_string_pos(&lcd_4bit,4,8,&addd);
        ret = lcd_4bit_send_char_data_pos(&lcd_4bit,4,11,'/');
        ret = convert_uint8_to_string(ds13_data_decimal[1],&addd);
        ret = lcd_4bit_send_string_pos(&lcd_4bit,4,12,&addd);
        ret = lcd_4bit_send_char_data_pos(&lcd_4bit,4,14,'/');
        ret = convert_uint8_to_string(ds13_data_decimal[2],&addd);
        ret = lcd_4bit_send_string_pos(&lcd_4bit,4,15,&addd);
        
        
        /* temperature and control  */
        tc74_read_value(&mssp_i2c,ADDRESS_A7,REDISTER_READ_0,&tc74,&i2c_ack);
        tc74_ref =tc74;
        ret = convert_uint8_to_string(tc74_ref,&addd);
        ret = lcd_4bit_send_string_pos(&lcd_4bit,3,15,&addd);
        if((tc74_ref > 25) && (tc74_ref <= 27)){
            ret = morot_turn_right(&motor_1);
            ret = morot_turn_left(&motor_2);
            ccp1_pwm_set_duty(80);
            ccp2_pwm_set_duty(20);
            CCP1_select_statues(ccp1_pwm_initialize);
            CCP2_select_statues(ccp2_pwm_initialize);
           }else if((tc74_ref > 27) && (tc74_ref <= 30)){
            ret = morot_turn_right(&motor_1);
            ret = morot_turn_left(&motor_2);
            ccp1_pwm_set_duty(50);
            ccp2_pwm_set_duty(50);
            CCP1_select_statues(ccp1_pwm_initialize);
            CCP2_select_statues(ccp2_pwm_initialize);
            } 
            else if((tc74_ref > 30) && (tc74_ref <= 33)){
            ret = morot_turn_right(&motor_1);
            ret = morot_turn_left(&motor_2);
            ccp1_pwm_set_duty(20);
            ccp2_pwm_set_duty(80);
            CCP1_select_statues(ccp1_pwm_initialize);
            CCP2_select_statues(ccp2_pwm_initialize);
            }
            else if(tc74_ref >= 34){
            ret = morot_turn_right(&motor_1);
            ret = morot_turn_left(&motor_2);
            ccp1_pwm_set_duty(1);
            ccp2_pwm_set_duty(100);
            CCP1_select_statues(ccp1_pwm_initialize);
            CCP2_select_statues(ccp2_pwm_initialize);
            }
           else{
            ret = morot_stop(&motor_1);
            ret = morot_stop(&motor_2);
            CCP1_select_statues(ccp1_module_disabled);
            CCP2_select_statues(ccp1_module_disabled);
            }
        /* usart and slave call */
        if(usart_data[6] != 0){
            if((usart_data[6] == 'd') && (usart_data[5] == 'e') && (usart_data[4] == 'm') && (usart_data[3] == 'a') && 
                    (usart_data[2] == 'h') && (usart_data[1] == 'o') && (usart_data[0] == 'm') ){
                ret = usart_ASYNC_write_byte_blocking('\r');
                ret = usart_ASYNC_write_byte_blocking('P');
                ret = usart_ASYNC_write_byte_blocking('A');
                ret = usart_ASYNC_write_byte_blocking('S');
                ret = usart_ASYNC_write_byte_blocking('S');
                ret = usart_ASYNC_write_byte_blocking('W');
                ret = usart_ASYNC_write_byte_blocking('O');
                ret = usart_ASYNC_write_byte_blocking('R');
                ret = usart_ASYNC_write_byte_blocking('D');
                ret = usart_ASYNC_write_byte_blocking(' ');
                ret = usart_ASYNC_write_byte_blocking('C');
                ret = usart_ASYNC_write_byte_blocking('O');
                ret = usart_ASYNC_write_byte_blocking('R');
                ret = usart_ASYNC_write_byte_blocking('R');
                ret = usart_ASYNC_write_byte_blocking('E');
                ret = usart_ASYNC_write_byte_blocking('C');
                ret = usart_ASYNC_write_byte_blocking('T');
                usart_data[6] = 0;
                MSSP_I2C_Master_call_slave(&mssp_i2c,0x60,'a',&i2c_ack);
            }else if((usart_data[6] == 'e') && (usart_data[5] == 'e') && (usart_data[4] == 'e') && (usart_data[3] == 't') && 
                    (usart_data[2] == 'i') && (usart_data[1] == 'x') && (usart_data[0] == 'e') ){
                ret = usart_ASYNC_write_byte_blocking('\r');
                    ret = usart_ASYNC_write_byte_blocking('E');
                ret = usart_ASYNC_write_byte_blocking('x');
                ret = usart_ASYNC_write_byte_blocking('i');
                ret = usart_ASYNC_write_byte_blocking('t');
                ret = usart_ASYNC_write_byte_blocking(' ');
                usart_data[6] = 0;
                MSSP_I2C_Master_call_slave(&mssp_i2c,0x60,'c',&i2c_ack);
            }else{
                ret = usart_ASYNC_write_byte_blocking('\r');
                ret = usart_ASYNC_write_byte_blocking('P');
                ret = usart_ASYNC_write_byte_blocking('A');
                ret = usart_ASYNC_write_byte_blocking('S');
                ret = usart_ASYNC_write_byte_blocking('S');
                ret = usart_ASYNC_write_byte_blocking('W');
                ret = usart_ASYNC_write_byte_blocking('O');
                ret = usart_ASYNC_write_byte_blocking('R');
                ret = usart_ASYNC_write_byte_blocking('D');
                ret = usart_ASYNC_write_byte_blocking(' ');
                ret = usart_ASYNC_write_byte_blocking('W');
                ret = usart_ASYNC_write_byte_blocking('R');
                ret = usart_ASYNC_write_byte_blocking('O');
                ret = usart_ASYNC_write_byte_blocking('N');
                ret = usart_ASYNC_write_byte_blocking('G');
                usart_data[6] = 0;
                MSSP_I2C_Master_call_slave(&mssp_i2c,0x60,'c',&i2c_ack);
            }
            
        } 
    }
    return (EXIT_SUCCESS);
} 

void intitialized_functions(void){
    ret = ADC_inti(&ADC_);
    ret = MSSP_I2C_Init(&mssp_i2c);
    ret = lcd_4bit_intialize(&lcd_4bit);
    ret = GPIO_pIN_INTIALIZE(&LED_ROOM_1);
    ret = GPIO_pIN_INTIALIZE(&leds_1);
    ret = Interrupt_Intx_Inti(&INT_0);
    ret = Interrupt_Intx_Inti(&INT_1);
    ret = Timer1_init(&Timer1);
    ret = motor_intialize(&motor_1);
    ret = motor_intialize(&motor_2);
    ret = usart_ASYNC_init(&usart_1);
    ret = Timer2_init(&Timer2);
    ret = ccp1_init(&ccp1_);
    ret = ccp2_init(&ccp2_);
}

void into_interrupt (void){
    ret = GPIO_pIN_WRITE_LOGIC(&leds_1,GPIO_HIGH);
    timer1_counter = 0;
}

void timer1_interrupt (void){
    timer1_counter++;
    counter++;
    if(timer1_counter == 6){
        timer1_counter = 0; 
        ret = GPIO_pIN_WRITE_LOGIC(&leds_1,GPIO_LOW);
    }
}

void int1_interrupt (void){
    ret = GPIO_pIN_TOGGLE_LOGIC(&LED_ROOM_1);
}

void interrupt_usart_rx(void){
    volatile uint8 ret_data = 0;
    ret = usart_ASYNC_read_byte_blocking (&usart_data[usart_counter]);
    usart_counter++;
    if(usart_counter == 7){
        usart_counter = 0;
    }
}

uint8 transfer_0x_d (uint8 num){
    uint8 l_bit = num%10;
    uint8 h_bit = num/10;
    l_bit = l_bit*1;
    h_bit = h_bit*16;
    return (l_bit+h_bit);
}