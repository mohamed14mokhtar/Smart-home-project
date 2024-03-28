#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/final_pro_2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/final_pro_2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=main_2.c ECU_layer/24c02c/ecu_24c02c.c ECU_layer/7_segment/ecu_seg.c ECU_layer/button/ecu_button.c ECU_layer/chr_lcd/ecu_chr_lcd.c ECU_layer/DC_MOTOR/ecu_dc_motor.c ECU_layer/DS1307/hal_ds1307.c ECU_layer/ecu_layer_init.c ECU_layer/LED/ecu_led.c ECU_layer/matrix_keypad/ecu_keymap.c ECU_layer/relay/ecu_relay.c ECU_layer/TC74/hal_tc74.c MKAL_layer/ADC/hal_adc.c MKAL_layer/CCP1/hal_ccp1.c MKAL_layer/CCP2/hal_ccp2.c MKAL_layer/device_confeg.c MKAL_layer/EEPROM/hal_eeprom.c MKAL_layer/gpio/pro_gpio.c MKAL_layer/I2C/hal_i2c.c MKAL_layer/Interrupt/mcal_external_interrupt.c MKAL_layer/Interrupt/mcal_internal_interrupt.c MKAL_layer/Interrupt/mcal_interrupt_manager.c MKAL_layer/timer/hal_timer0.c MKAL_layer/timer1/hal_timer1.c MKAL_layer/timer2/hal_timer2.c MKAL_layer/timer3/hal_timer3.c MKAL_layer/USART/hal_usart.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/main_2.p1 ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1 ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1 ${OBJECTDIR}/ECU_layer/button/ecu_button.p1 ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1 ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1 ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1 ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1 ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1 ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1 ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1 ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1 ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1 ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1 ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1 ${OBJECTDIR}/MKAL_layer/device_confeg.p1 ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1 ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1 ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1 ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1 ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1 ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1 ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1 ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1 ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1 ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1 ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1
POSSIBLE_DEPFILES=${OBJECTDIR}/main_2.p1.d ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1.d ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1.d ${OBJECTDIR}/ECU_layer/button/ecu_button.p1.d ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1.d ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1.d ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1.d ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1.d ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1.d ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1.d ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1.d ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1.d ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1.d ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1.d ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1.d ${OBJECTDIR}/MKAL_layer/device_confeg.p1.d ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1.d ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1.d ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1.d ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1.d ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1.d ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1.d ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1.d ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1.d ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1.d ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1.d ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1.d

# Object Files
OBJECTFILES=${OBJECTDIR}/main_2.p1 ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1 ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1 ${OBJECTDIR}/ECU_layer/button/ecu_button.p1 ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1 ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1 ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1 ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1 ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1 ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1 ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1 ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1 ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1 ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1 ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1 ${OBJECTDIR}/MKAL_layer/device_confeg.p1 ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1 ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1 ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1 ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1 ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1 ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1 ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1 ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1 ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1 ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1 ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1

# Source Files
SOURCEFILES=main_2.c ECU_layer/24c02c/ecu_24c02c.c ECU_layer/7_segment/ecu_seg.c ECU_layer/button/ecu_button.c ECU_layer/chr_lcd/ecu_chr_lcd.c ECU_layer/DC_MOTOR/ecu_dc_motor.c ECU_layer/DS1307/hal_ds1307.c ECU_layer/ecu_layer_init.c ECU_layer/LED/ecu_led.c ECU_layer/matrix_keypad/ecu_keymap.c ECU_layer/relay/ecu_relay.c ECU_layer/TC74/hal_tc74.c MKAL_layer/ADC/hal_adc.c MKAL_layer/CCP1/hal_ccp1.c MKAL_layer/CCP2/hal_ccp2.c MKAL_layer/device_confeg.c MKAL_layer/EEPROM/hal_eeprom.c MKAL_layer/gpio/pro_gpio.c MKAL_layer/I2C/hal_i2c.c MKAL_layer/Interrupt/mcal_external_interrupt.c MKAL_layer/Interrupt/mcal_internal_interrupt.c MKAL_layer/Interrupt/mcal_interrupt_manager.c MKAL_layer/timer/hal_timer0.c MKAL_layer/timer1/hal_timer1.c MKAL_layer/timer2/hal_timer2.c MKAL_layer/timer3/hal_timer3.c MKAL_layer/USART/hal_usart.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/final_pro_2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F4620
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/main_2.p1: main_2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_2.p1.d 
	@${RM} ${OBJECTDIR}/main_2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/main_2.p1 main_2.c 
	@-${MV} ${OBJECTDIR}/main_2.d ${OBJECTDIR}/main_2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main_2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1: ECU_layer/24c02c/ecu_24c02c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/24c02c" 
	@${RM} ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1 ECU_layer/24c02c/ecu_24c02c.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.d ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1: ECU_layer/7_segment/ecu_seg.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/7_segment" 
	@${RM} ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1 ECU_layer/7_segment/ecu_seg.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.d ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/button/ecu_button.p1: ECU_layer/button/ecu_button.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/button" 
	@${RM} ${OBJECTDIR}/ECU_layer/button/ecu_button.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/button/ecu_button.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/button/ecu_button.p1 ECU_layer/button/ecu_button.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/button/ecu_button.d ${OBJECTDIR}/ECU_layer/button/ecu_button.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/button/ecu_button.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1: ECU_layer/chr_lcd/ecu_chr_lcd.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/chr_lcd" 
	@${RM} ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1 ECU_layer/chr_lcd/ecu_chr_lcd.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.d ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1: ECU_layer/DC_MOTOR/ecu_dc_motor.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/DC_MOTOR" 
	@${RM} ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1 ECU_layer/DC_MOTOR/ecu_dc_motor.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.d ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1: ECU_layer/DS1307/hal_ds1307.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/DS1307" 
	@${RM} ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1 ECU_layer/DS1307/hal_ds1307.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.d ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/ecu_layer_init.p1: ECU_layer/ecu_layer_init.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer" 
	@${RM} ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1 ECU_layer/ecu_layer_init.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/ecu_layer_init.d ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/LED/ecu_led.p1: ECU_layer/LED/ecu_led.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/LED" 
	@${RM} ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1 ECU_layer/LED/ecu_led.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/LED/ecu_led.d ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1: ECU_layer/matrix_keypad/ecu_keymap.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/matrix_keypad" 
	@${RM} ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1 ECU_layer/matrix_keypad/ecu_keymap.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.d ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1: ECU_layer/relay/ecu_relay.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/relay" 
	@${RM} ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1 ECU_layer/relay/ecu_relay.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/relay/ecu_relay.d ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1: ECU_layer/TC74/hal_tc74.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/TC74" 
	@${RM} ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1 ECU_layer/TC74/hal_tc74.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.d ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1: MKAL_layer/ADC/hal_adc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/ADC" 
	@${RM} ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1 MKAL_layer/ADC/hal_adc.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.d ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1: MKAL_layer/CCP1/hal_ccp1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/CCP1" 
	@${RM} ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1 MKAL_layer/CCP1/hal_ccp1.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.d ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1: MKAL_layer/CCP2/hal_ccp2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/CCP2" 
	@${RM} ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1 MKAL_layer/CCP2/hal_ccp2.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.d ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/device_confeg.p1: MKAL_layer/device_confeg.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer" 
	@${RM} ${OBJECTDIR}/MKAL_layer/device_confeg.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/device_confeg.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/device_confeg.p1 MKAL_layer/device_confeg.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/device_confeg.d ${OBJECTDIR}/MKAL_layer/device_confeg.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/device_confeg.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1: MKAL_layer/EEPROM/hal_eeprom.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/EEPROM" 
	@${RM} ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1 MKAL_layer/EEPROM/hal_eeprom.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.d ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1: MKAL_layer/gpio/pro_gpio.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/gpio" 
	@${RM} ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1 MKAL_layer/gpio/pro_gpio.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.d ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1: MKAL_layer/I2C/hal_i2c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/I2C" 
	@${RM} ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1 MKAL_layer/I2C/hal_i2c.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.d ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1: MKAL_layer/Interrupt/mcal_external_interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1 MKAL_layer/Interrupt/mcal_external_interrupt.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.d ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1: MKAL_layer/Interrupt/mcal_internal_interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1 MKAL_layer/Interrupt/mcal_internal_interrupt.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.d ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1: MKAL_layer/Interrupt/mcal_interrupt_manager.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1 MKAL_layer/Interrupt/mcal_interrupt_manager.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.d ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1: MKAL_layer/timer/hal_timer0.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/timer" 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1 MKAL_layer/timer/hal_timer0.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.d ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1: MKAL_layer/timer1/hal_timer1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/timer1" 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1 MKAL_layer/timer1/hal_timer1.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.d ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1: MKAL_layer/timer2/hal_timer2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/timer2" 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1 MKAL_layer/timer2/hal_timer2.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.d ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1: MKAL_layer/timer3/hal_timer3.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/timer3" 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1 MKAL_layer/timer3/hal_timer3.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.d ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1: MKAL_layer/USART/hal_usart.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/USART" 
	@${RM} ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1 MKAL_layer/USART/hal_usart.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/USART/hal_usart.d ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/main_2.p1: main_2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_2.p1.d 
	@${RM} ${OBJECTDIR}/main_2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/main_2.p1 main_2.c 
	@-${MV} ${OBJECTDIR}/main_2.d ${OBJECTDIR}/main_2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main_2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1: ECU_layer/24c02c/ecu_24c02c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/24c02c" 
	@${RM} ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1 ECU_layer/24c02c/ecu_24c02c.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.d ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/24c02c/ecu_24c02c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1: ECU_layer/7_segment/ecu_seg.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/7_segment" 
	@${RM} ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1 ECU_layer/7_segment/ecu_seg.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.d ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/7_segment/ecu_seg.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/button/ecu_button.p1: ECU_layer/button/ecu_button.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/button" 
	@${RM} ${OBJECTDIR}/ECU_layer/button/ecu_button.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/button/ecu_button.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/button/ecu_button.p1 ECU_layer/button/ecu_button.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/button/ecu_button.d ${OBJECTDIR}/ECU_layer/button/ecu_button.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/button/ecu_button.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1: ECU_layer/chr_lcd/ecu_chr_lcd.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/chr_lcd" 
	@${RM} ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1 ECU_layer/chr_lcd/ecu_chr_lcd.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.d ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/chr_lcd/ecu_chr_lcd.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1: ECU_layer/DC_MOTOR/ecu_dc_motor.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/DC_MOTOR" 
	@${RM} ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1 ECU_layer/DC_MOTOR/ecu_dc_motor.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.d ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/DC_MOTOR/ecu_dc_motor.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1: ECU_layer/DS1307/hal_ds1307.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/DS1307" 
	@${RM} ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1 ECU_layer/DS1307/hal_ds1307.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.d ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/DS1307/hal_ds1307.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/ecu_layer_init.p1: ECU_layer/ecu_layer_init.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer" 
	@${RM} ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1 ECU_layer/ecu_layer_init.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/ecu_layer_init.d ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/ecu_layer_init.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/LED/ecu_led.p1: ECU_layer/LED/ecu_led.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/LED" 
	@${RM} ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1 ECU_layer/LED/ecu_led.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/LED/ecu_led.d ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/LED/ecu_led.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1: ECU_layer/matrix_keypad/ecu_keymap.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/matrix_keypad" 
	@${RM} ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1 ECU_layer/matrix_keypad/ecu_keymap.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.d ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/matrix_keypad/ecu_keymap.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1: ECU_layer/relay/ecu_relay.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/relay" 
	@${RM} ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1 ECU_layer/relay/ecu_relay.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/relay/ecu_relay.d ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/relay/ecu_relay.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1: ECU_layer/TC74/hal_tc74.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_layer/TC74" 
	@${RM} ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1.d 
	@${RM} ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1 ECU_layer/TC74/hal_tc74.c 
	@-${MV} ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.d ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_layer/TC74/hal_tc74.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1: MKAL_layer/ADC/hal_adc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/ADC" 
	@${RM} ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1 MKAL_layer/ADC/hal_adc.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.d ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/ADC/hal_adc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1: MKAL_layer/CCP1/hal_ccp1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/CCP1" 
	@${RM} ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1 MKAL_layer/CCP1/hal_ccp1.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.d ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/CCP1/hal_ccp1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1: MKAL_layer/CCP2/hal_ccp2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/CCP2" 
	@${RM} ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1 MKAL_layer/CCP2/hal_ccp2.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.d ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/CCP2/hal_ccp2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/device_confeg.p1: MKAL_layer/device_confeg.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer" 
	@${RM} ${OBJECTDIR}/MKAL_layer/device_confeg.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/device_confeg.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/device_confeg.p1 MKAL_layer/device_confeg.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/device_confeg.d ${OBJECTDIR}/MKAL_layer/device_confeg.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/device_confeg.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1: MKAL_layer/EEPROM/hal_eeprom.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/EEPROM" 
	@${RM} ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1 MKAL_layer/EEPROM/hal_eeprom.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.d ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/EEPROM/hal_eeprom.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1: MKAL_layer/gpio/pro_gpio.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/gpio" 
	@${RM} ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1 MKAL_layer/gpio/pro_gpio.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.d ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/gpio/pro_gpio.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1: MKAL_layer/I2C/hal_i2c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/I2C" 
	@${RM} ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1 MKAL_layer/I2C/hal_i2c.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.d ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/I2C/hal_i2c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1: MKAL_layer/Interrupt/mcal_external_interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1 MKAL_layer/Interrupt/mcal_external_interrupt.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.d ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_external_interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1: MKAL_layer/Interrupt/mcal_internal_interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1 MKAL_layer/Interrupt/mcal_internal_interrupt.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.d ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_internal_interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1: MKAL_layer/Interrupt/mcal_interrupt_manager.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1 MKAL_layer/Interrupt/mcal_interrupt_manager.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.d ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/Interrupt/mcal_interrupt_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1: MKAL_layer/timer/hal_timer0.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/timer" 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1 MKAL_layer/timer/hal_timer0.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.d ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/timer/hal_timer0.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1: MKAL_layer/timer1/hal_timer1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/timer1" 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1 MKAL_layer/timer1/hal_timer1.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.d ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/timer1/hal_timer1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1: MKAL_layer/timer2/hal_timer2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/timer2" 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1 MKAL_layer/timer2/hal_timer2.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.d ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/timer2/hal_timer2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1: MKAL_layer/timer3/hal_timer3.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/timer3" 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1 MKAL_layer/timer3/hal_timer3.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.d ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/timer3/hal_timer3.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1: MKAL_layer/USART/hal_usart.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MKAL_layer/USART" 
	@${RM} ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1.d 
	@${RM} ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1 MKAL_layer/USART/hal_usart.c 
	@-${MV} ${OBJECTDIR}/MKAL_layer/USART/hal_usart.d ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MKAL_layer/USART/hal_usart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/final_pro_2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/final_pro_2.X.${IMAGE_TYPE}.map  -D__DEBUG=1  -mdebugger=none  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits -std=c99 -gcoff -mstack=compiled:auto:auto:auto        $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/final_pro_2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	@${RM} ${DISTDIR}/final_pro_2.X.${IMAGE_TYPE}.hex 
	
else
${DISTDIR}/final_pro_2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/final_pro_2.X.${IMAGE_TYPE}.map  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits -std=c99 -gcoff -mstack=compiled:auto:auto:auto     $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/final_pro_2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
