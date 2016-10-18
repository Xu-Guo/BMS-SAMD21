/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <stdlib.h>
#include <asf.h>
#include "conf_usb.h"
#include "ui.h"
#include "uart.h"
#include "conf_qs_events_interrupt_hook.h"
#include "adc_temp.h"


//[definition_pwm]
/** PWM module to use */
#define CONF_PWM_MODULE      LED_0_PWM4CTRL_MODULE
/** PWM channel */
#define CONF_PWM_CHANNEL     LED_0_PWM4CTRL_CHANNEL
/** PWM output */
#define CONF_PWM_OUTPUT      LED_0_PWM4CTRL_OUTPUT
/** PWM output pin */
#define CONF_PWM_OUT_PIN     LED_0_PWM4CTRL_PIN
/** PWM output pinmux */
#define CONF_PWM_OUT_MUX     LED_0_PWM4CTRL_MUX
//[definition_pwm]



#define  ADC_REFERENCE_INT1V_VALUE    0X01
#define  ADC_8BIT_FULL_SCALE_VALUE    0XFF

//TX_DATA_TYPE
#define TX_TYPE_BATTERY_DATA		  0x10
#define TX_TYPE_TIME_DATA		      0x11

#define OPAMP_GAIN					  13




/*
|START_FLAG|TX_DATA_TYPE|DATA_LENGTH|CHARGE_CURRENT|DISCHARGE_CURRENT|TEMPERATURE_DATA|soc percentage|BATTERY_STATUS|CHARGER_STATUS|YEAR   |MONTH  |DAY    |HOUR   |MIN   |SEC   |END_FLAG|
|1 BYTE    |1 BYTE      |1 BYTE     |2 BYTES       |2 BYTES          |2 BYTES         |1 BYTE        |1 BYTE        |1 BYTE        |2 BYTES|1 BYTE |1 BYTE |1 BYTE |1 BYTE|1 BYTE|1 BYTE  |
*/
#define BATTERY_DATA_LENGTH   16
#define TIME_DATA_LENGTH       7



uint8_t battery_data[BATTERY_DATA_LENGTH] = {0};
uint8_t time_data[TIME_DATA_LENGTH]	= {0};
	
uint16_t temprerature_value;

//for check if battery charging status changes
uint8_t battery_status_new = 0;
uint8_t battery_status_old = 0;


//**************SYSTEM COMMAND RELATED VARIABLE****************
/*set this variable to 1 when executing command, 
set this variable back to 0 when command is done executing.*/

uint8_t system_busy_flag = 0;
uint8_t forceDataReportFlag = 0;
uint8_t dateReportFlag = 0;

uint8_t commandReady = 0;


static volatile bool main_b_cdc_enable = false;

volatile bool adc_read_done = false;

//charge current samples on the ain8>PB00
uint16_t charge_signal_adc_result;
//discharge current samples on the ain9>PB01
uint16_t discharge_signal_adc_result;
//temperature result
uint16_t temprerature_adc_result;

uint16_t avg_charge_current_reading = 0;
uint16_t avg_discharge_current_reading = 0;

float adc_charge_signal_voltage;
float adc_discharge_signal_voltage;

/* To store raw_result of ADC output */
uint16_t raw_result;

//1 for changer presents,0 for charger not presents
uint8_t charger_status = 0;

//1 for charging, 2 for discharging, 0 for no current in/out.
uint8_t battery_status = 0;

uint8_t commandType = 0; //default = 0
uint8_t commandIndex = 0;//default = 0
uint8_t commandDataLength = 0;
uint8_t commandData[20] = {0};
	
uint64_t total_charge_current = 0;
uint64_t total_discharge_current = 0;

uint32_t charge_sample_num = 0;
uint32_t discharge_sample_num = 0;

uint32_t battery_capcity = 540000; //listed total battery capacity in unit of mAsec
uint32_t calibrated_battery_capacity = 540000;

int8_t charge_remain_percentage = 0;    // ?% of total capacity x 100;

uint8_t charge_from_empty_flag = 0;
uint8_t	discharge_from_full_flag = 0;

static volatile uint32_t event_count = 0;

//! [adc_module_inst]
struct adc_module adc_instance;

//! [rtc_module_instance]
struct rtc_module rtc_instance;

//! [alarm_struct]
struct rtc_calendar_alarm_time alarm;

//! [GPIO_struct]
struct port_config config_port_pin;


void ADC_event(struct events_resource *resource);

void configure_port_pins(void);

void configure_adc(void);
void configure_adc_callbacks(void);
void adc_complete_callback(struct adc_module *const module);
void adc_sampling(void);

//digital read from GPIO PB06 to see if charger is connected
void charger_detection(void); 

//! [functions for setup rtc]
void rtc_match_callback(void);
void configure_rtc_callbacks(void);
void configure_rtc_calendar(void);
void rtc_event_init(void);


void processCommandMsg(void);
void execute_system_command(void);

void adc_get_temperature(void);
uint16_t adc_start_read_temp(void);

uint8_t battery_status_update(void);

//smart battery issues commands
void send_battery_data(void);
void send_board_time_data(void);


//local host issues commands
void setParameter(void);
void setTime(void);

void battery_charge_calculation(uint8_t time);

//functions for pwm
static void configure_tcc(void);
static void configure_tcc_callbacks(void);
static void tcc_callback_to_change_duty_cycle(struct tcc_module *const module_inst);

//! [module_inst]
struct tcc_module tcc_instance;


//! [callback_funcs]
static void tcc_callback_to_change_duty_cycle(struct tcc_module *const module_inst)
{
	static uint32_t delay = 100;
	static uint32_t i = 0;

	if (--delay) {
		return;
	}
	delay = 100;
	i = (i + 0x0800) & 0xFFFF;
	tcc_set_compare_value(module_inst,(enum tcc_match_capture_channel)(TCC_MATCH_CAPTURE_CHANNEL_0 + CONF_PWM_CHANNEL),i + 1);
	//try modify the duty_cycle to get adc reading*****
	//tcc_set_compare_value(module_inst,(enum tcc_match_capture_channel)(TCC_MATCH_CAPTURE_CHANNEL_0 + CONF_PWM_CHANNEL),0x04ff);
}


//! [setup]
static void configure_tcc(void)
{
	//! [setup_config]
	struct tcc_config config_tcc;
	
	//! [setup_config_defaults]
	tcc_get_config_defaults(&config_tcc, CONF_PWM_MODULE);


	//! [setup_change_config]
	config_tcc.counter.period = 0xFFFF;
	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	config_tcc.compare.match[CONF_PWM_CHANNEL] = 0xFFFF;
	

	//! [setup_change_config_pwm]
	config_tcc.pins.enable_wave_out_pin[CONF_PWM_OUTPUT] = true;
	config_tcc.pins.wave_out_pin[CONF_PWM_OUTPUT]        = CONF_PWM_OUT_PIN;
	config_tcc.pins.wave_out_pin_mux[CONF_PWM_OUTPUT]    = CONF_PWM_OUT_MUX;


	//! [setup_set_config]
	tcc_init(&tcc_instance, CONF_PWM_MODULE, &config_tcc);
	

	//! [setup_enable]
	tcc_enable(&tcc_instance);

}

static void configure_tcc_callbacks(void)
{
	//! [setup_register_callback]
	tcc_register_callback(
	&tcc_instance,
	tcc_callback_to_change_duty_cycle,
	(enum tcc_callback)(TCC_CALLBACK_CHANNEL_0 + CONF_PWM_CHANNEL));

	//! [setup_enable_callback]
	tcc_enable_callback(&tcc_instance,
	(enum tcc_callback)(TCC_CALLBACK_CHANNEL_0 + CONF_PWM_CHANNEL));
}



uint8_t battery_status_update(void)
{
	if (charger_status == 1)
	{
		if (charge_signal_adc_result > 0x00ff)
		{
			battery_status = 1; //charging, battery is not full
			
			//the followed discharge is not from 100% remain.
			discharge_from_full_flag = 0;
		}
		else if (charge_signal_adc_result < 0x20)
		{
			battery_status = 0;//no current in/out for battery, battery is full
			charge_signal_adc_result = 0;
			//the follow discharge will be from 100% remain, set the discharge_from_full_flag = 1
			discharge_from_full_flag = 1;
		}
	}
	//need to add in self-discharge control logic and self-discharge to empty function
	else if (discharge_signal_adc_result > 0xff)
	{
		battery_status = 2;    //discharging
		charge_from_empty_flag = 0;
	}
	else //battery is empty
	{
		battery_status = 0;
		discharge_signal_adc_result = 0;
		charge_from_empty_flag = 1;	
	}
	return battery_status;
}


uint16_t adc_start_read_temp(void)
{
	uint16_t adc_result = 0;
	adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_TEMP);
	adc_start_conversion(&adc_instance);
	while((adc_get_status(&adc_instance) & ADC_STATUS_RESULT_READY) != 1);
		
	adc_read(&adc_instance, &adc_result);
	
	return adc_result;
}

void adc_get_temperature(void)
{
	float temp;
	system_voltage_reference_enable(SYSTEM_VOLTAGE_REFERENCE_TEMPSENSE);
	load_calibration_data();
	raw_result = adc_start_read_temp();
	temp = calculate_temperature(raw_result);
	temprerature_value = (uint16_t)(temp * 1000);
	//udi_cdc_write_buf(&temprerature_value,2);
	
}


//! [rtc_alarm_callback]
void rtc_match_callback(void)
{
	static uint8_t second_count = 1;
	
	/* Do something on RTC alarm match here */
	port_pin_toggle_output_level(LED_0_PIN);
	adc_get_temperature();
	if (dateReportFlag == 1)
	{
		battery_charge_calculation(second_count);
		second_count = 1;
		send_battery_data();
		//reset flags
		dateReportFlag = 0;
	}
	
	/* Set new alarm in 1 seconds */
	//! [alarm_mask]
	alarm.mask = RTC_CALENDAR_ALARM_MASK_SEC;

	//! [set_alarm]
	alarm.time.second += 1;
	alarm.time.second = alarm.time.second % 60;
	
	//forced data reporting every 10 seconds
	if ((alarm.time.second % 50) == 0)
	{
		dateReportFlag = 1;
	}
	
	second_count++;
	
	rtc_calendar_set_alarm(&rtc_instance, &alarm, RTC_CALENDAR_ALARM_0);
}


//! [setup_rtc_alarm_callback]
void configure_rtc_callbacks(void)
{
	//! [reg_callback]
	rtc_calendar_register_callback(
	&rtc_instance, rtc_match_callback, RTC_CALENDAR_CALLBACK_ALARM_0);
	
	//! [en_callback]
	rtc_calendar_enable_callback(&rtc_instance, RTC_CALENDAR_CALLBACK_ALARM_0);
}


//! [initialize_rtc]
void configure_rtc_calendar(void)
{
	/* Initialize RTC in calendar mode. */
	//! [init_conf]
	struct rtc_calendar_config config_rtc_calendar;
	rtc_calendar_get_config_defaults(&config_rtc_calendar);

	//! [time_struct]
	alarm.time.year      = 2016;
	alarm.time.month     = 1;
	alarm.time.day       = 1;
	alarm.time.hour      = 0;
	alarm.time.minute    = 0;
	alarm.time.second    = 0;

	//! [set_config]
	config_rtc_calendar.clock_24h = true;
	config_rtc_calendar.alarm[0].time = alarm.time;
	config_rtc_calendar.alarm[0].mask = RTC_CALENDAR_ALARM_MASK_SEC;

	//! [init_rtc]
	rtc_calendar_init(&rtc_instance, RTC, &config_rtc_calendar);
	
	//[setup and init rtc event]
	struct rtc_calendar_events calendar_event;
	calendar_event.generate_event_on_periodic[4] = true;
	rtc_calendar_enable_events(&rtc_instance, &calendar_event);
	
	//! [enable]
	rtc_calendar_enable(&rtc_instance);
}

//setup event 
static void configure_event_channel(struct events_resource *resource)
{
	struct events_config config;

	events_get_config_defaults(&config);

	config.generator      = CONF_EVENT_GENERATOR;
	config.edge_detect    = EVENTS_EDGE_DETECT_RISING;
	config.path           = EVENTS_PATH_SYNCHRONOUS;
	config.clock_source   = GCLK_GENERATOR_0;

	events_allocate(resource, &config);
}

//setup the event user
static void configure_event_user(struct events_resource *resource)
{
	events_attach_user(resource, CONF_EVENT_USER);
}

//hook up the event function to event resource
static void configure_event_interrupt(struct events_resource *resource, struct events_hook *hook)
{
	events_create_hook(hook, ADC_event);

	events_add_hook(resource, hook);
	
	events_enable_interrupt_source(resource, EVENTS_INTERRUPT_DETECT);
}



//SYSTEM EVENT for ADC sampling and state detection
void ADC_event(struct events_resource *resource)
{

	if(events_is_interrupt_set(resource, EVENTS_INTERRUPT_DETECT)) {
		//port_pin_toggle_output_level(LED_0_PIN);
		charger_detection();
		adc_sampling();
		battery_status_new = battery_status_update();
		if (battery_status_new != battery_status_old) //if battery charging status changes, send data to pc to update
		{	
			dateReportFlag = 1;
			battery_status_old = battery_status_new;
		}
		events_ack_interrupt(resource, EVENTS_INTERRUPT_DETECT);
	}
}

//charger detect function
void charger_detection(void)
{
	charger_status = port_pin_get_input_level(EXT1_PIN_5);
	port_pin_set_output_level(EXT1_PIN_6,charger_status);
}


//GPIO setup
void configure_port_pins(void)
{
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_NONE;
	
	
	//set EXT1_PIN_5(PB06) as input for charger detect, 1 for charger present/0 for charge not present
	port_pin_set_config(EXT1_PIN_5, &config_port_pin);
	
	//set EXT1_PIN_6(PB07) as LED output, on for charger present/off for charger not present 
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(EXT1_PIN_6, &config_port_pin);
}


//! [setup ADC]
void configure_adc(void)
{
	//! [setup_config]
	struct adc_config config_adc;
	
	//! [setup_config_defaults]
	adc_get_config_defaults(&config_adc);

	config_adc.clock_source = GCLK_GENERATOR_1;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
	config_adc.reference = ADC_REFERENCE_INT1V;
	config_adc.positive_input = ADC_POSITIVE_INPUT_TEMP;
	config_adc.negative_input = ADC_NEGATIVE_INPUT_GND;
	config_adc.sample_length = ADC_TEMP_SAMPLE_LENGTH;

	//! [setup_set_config]
	adc_init(&adc_instance, ADC, &config_adc);

	ADC->AVGCTRL.reg = ADC_AVGCTRL_ADJRES(2) | ADC_AVGCTRL_SAMPLENUM_4;
	//! [setup_enable]
	adc_enable(&adc_instance);
}

void adc_sampling(void)
{
	//uint8_t string1 = 0x0c;
	//uint8_t string2 = 0x0d;

	//************************************************************************
	if (0 == charger_status)//charger not connected
	{	charge_signal_adc_result = 0;
		//**********************************for discharge signal*******************************
		adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN9);
		adc_start_conversion(&adc_instance);
		do {
		// Wait for conversion to be done and read out result
		} while (adc_read(&adc_instance, &discharge_signal_adc_result) == STATUS_BUSY);	
		

		total_discharge_current += discharge_signal_adc_result;
		discharge_sample_num ++;
		//udi_cdc_write_buf(&string2,1);				
		//udi_cdc_write_buf(&discharge_signal_adc_result,2);
		//int16_t discharge_raw_result_signed;
		//discharge_raw_result_signed = (int16_t)discharge_signal_adc_result;
		//adc_discharge_signal_voltage = ((float)discharge_raw_result_signed * (float)ADC_REFERENCE_INT1V_VALUE)/(float)ADC_8BIT_FULL_SCALE_VALUE;
	}
	else
	{
		discharge_signal_adc_result = 0;
		//*****************************for charge signal************************************
		adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN8);
		adc_start_conversion(&adc_instance);
		do {
			// Wait for conversion to be done and read out result 
		} while (adc_read(&adc_instance, &charge_signal_adc_result) == STATUS_BUSY);
		
		total_charge_current += charge_signal_adc_result;
		charge_sample_num++;
		//udi_cdc_write_buf(&string1,1);
		//udi_cdc_write_buf(&charge_signal_adc_result,2);
		//int16_t charge_raw_result_signed;
		//charge_raw_result_signed = (int16_t)charge_signal_adc_result;
		//adc_charge_signal_voltage = ((float)charge_raw_result_signed * (float)ADC_REFERENCE_INT1V_VALUE)/(float)ADC_8BIT_FULL_SCALE_VALUE;
	}
}



void send_battery_data(void)
{	
	struct rtc_calendar_time current_time;
	rtc_calendar_get_time(&rtc_instance, &current_time);
	battery_data[0] = (uint8_t)((avg_charge_current_reading >> 8) & 0xff);
	battery_data[1] = (uint8_t)(avg_charge_current_reading & 0xff);
 	battery_data[2] = (uint8_t)((avg_discharge_current_reading >> 8) & 0xff);
	battery_data[3] = (uint8_t)(avg_discharge_current_reading & 0xff);
	battery_data[4] = (uint8_t)((temprerature_value >> 8) & 0xff);
	battery_data[5] = (uint8_t)(temprerature_value & 0xff);
	battery_data[6] = charge_remain_percentage;
	battery_data[7] = battery_status;
	battery_data[8] = charger_status;
	battery_data[9] = (uint8_t)((current_time.year >> 8) & 0xff);
	battery_data[10] = (uint8_t)(current_time.year & 0xff);
	battery_data[11] = current_time.month;
	battery_data[12] = current_time.day;
	battery_data[13] = current_time.hour;
	battery_data[14] = current_time.minute;
	battery_data[15] = current_time.second;
	
	avg_charge_current_reading = 0;
	avg_discharge_current_reading = 0;
		
	uint8_t tx_data[20];
	tx_data[0] = START_FLAG;
	tx_data[1] = TX_TYPE_BATTERY_DATA;
	tx_data[2] = 16;
	for (uint8_t i = 0; i<sizeof(battery_data); i++)
	{
		tx_data[i+3] = battery_data[i];
	}
	tx_data[19] = END_FLAG;
	udi_cdc_write_buf(tx_data,20);
	
}

void send_board_time_data(void)
{
	struct rtc_calendar_time current_time;
	rtc_calendar_get_time(&rtc_instance, &current_time);
	time_data[0] = (uint8_t)((current_time.year >> 8) & 0xff);
	time_data[1] = (uint8_t)(current_time.year & 0xff);
	time_data[2] = current_time.month;
	time_data[3] = current_time.day;
	time_data[4] = current_time.hour;
	time_data[5] = current_time.minute;
	time_data[6] = current_time.second;
	
	uint8_t tx_data[11];
	tx_data[0] = START_FLAG;
	tx_data[1] = TX_TYPE_TIME_DATA;
	tx_data[2] = 7;
	for (uint8_t i=0;i<sizeof(time_data);i++)
	{
		tx_data[i+3] = time_data[i];
	}
	tx_data[10] = END_FLAG;
	udi_cdc_write_buf(tx_data,11);
}

void setParameter(void)
{
	//to be done..
}

void setTime(void)
{
	struct rtc_calendar_time time;
	rtc_calendar_get_time_defaults(&time);
	time.year   = (uint16_t)((commandData[0] << 8) + commandData[1]);
	time.month  = commandData[2];
	time.day    = commandData[3];
	time.hour   = commandData[4];
	time.minute = commandData[5];
	time.second = commandData[6];
	rtc_calendar_set_time(&rtc_instance, &time);
}

typedef void (*funcPtQuery)(void);
funcPtQuery requestAction[] = {&send_battery_data , &send_board_time_data};
const int requestActionNum = sizeof(requestAction)/sizeof(requestAction[0]);

typedef void (*funcPtUpdate)(void);
funcPtUpdate setAction[] = {&setParameter , &setTime};
const int setActionNum = sizeof(setAction)/sizeof(setAction[0]);


/*************************************************************
*	 commandMsg format:
*	|commandType|commandIndex  |commandDataLength|commandData
*	|1st Byte   |2nd Byte      |3rd Byte         | ..........
*   queryType command: commandDataLength and commandData are 0.
***************************************************************/
void processCommandMsg(void)
{	
	commandType = commandMsg[0]; //get commandType
	commandIndex = commandMsg[1];//get commandIndex
	commandDataLength = commandMsg[2];//get commandDataLength
		
	if (commandDataLength!= 0 )//if commandDataLength is not 0, get commandData.
	{
		for (int i=0;i<commandDataLength;i++)
		{
			commandData[i] = commandMsg[i+3];
		}
	}
	//reset commandMsg[] back to 0;
	for (uint8_t i = 0; i<sizeof(commandMsg); i++)
	{
		commandMsg[i] = 0;
	}
	usbMsgInFlag = 0;//reset MsgIn flag
	commandReady = 1;//set commandReady to be executed
}




void execute_system_command()
{	
	system_busy_flag = 1;
	commandReady = 0;
	
	if(commandType == CMD_TYPE_REQUEST)
	{	
		requestAction[commandIndex]();		
	}else if (commandType == CMD_TYPE_SET)
	{
		setAction[commandIndex]();
	}
	
	commandType = 0; //reset commandType to 0
	commandIndex = 0;//reset commandIndex to 0
	commandDataLength = 0;//reset commandDataLength to 0
	system_busy_flag = 0;
}

void battery_charge_calculation(uint8_t time)
{

	static uint16_t avg_charge_current_reading_old;
	static uint16_t avg_discharge_current_reading_old;
	static int32_t delta_charge; //total electrical charge that been charged into battery plus discharged from battery in unit of mAsec. 

	int16_t diff = 0;
	
	if (charge_sample_num >0)
	{
		avg_charge_current_reading = (uint16_t)(total_charge_current / charge_sample_num);
		total_charge_current = 0;
		charge_sample_num = 0;
		diff = (int16_t)(avg_charge_current_reading - avg_charge_current_reading_old);
		if (diff > 20 || diff < -20)//20 ADC reading unit = 0.005v
		{
			dateReportFlag = 1;
			avg_charge_current_reading_old = avg_charge_current_reading;
		}
		delta_charge += avg_charge_current_reading * 1000 * 10 / 4095 / 75 * time;
	}
	
	if (discharge_sample_num > 0)
	{	
		avg_discharge_current_reading = (uint16_t)(total_discharge_current / discharge_sample_num);
		total_discharge_current = 0;
		discharge_sample_num = 0;
		diff = (int16_t)(avg_discharge_current_reading - avg_discharge_current_reading_old);
		if (diff > 20 || diff < -20)//20 ADC reading unit = 0.005v
		{
			dateReportFlag = 1;
			avg_discharge_current_reading_old = avg_discharge_current_reading;
		}
		delta_charge -= avg_discharge_current_reading * 1000 / 4095 / 5 * time;
	}
		
	if ((((uint32_t)(delta_charge) > battery_capcity) && (charge_from_empty_flag == 1) ) ||
		(((uint32_t)(0 - delta_charge) > battery_capcity) && (discharge_from_full_flag == 1)))
	{
		calibrated_battery_capacity = (uint32_t)(abs(delta_charge));
	}
	
	if (delta_charge > 0)
	{
		charge_remain_percentage = delta_charge * 100 / calibrated_battery_capacity;
	}
	else
	{
		charge_remain_percentage = (calibrated_battery_capacity + delta_charge) * 100 / calibrated_battery_capacity;
		if (charge_remain_percentage < 1)
		{
			charge_remain_percentage = 1;
		}
	}
}





void main_suspend_action(void)
{
	ui_powerdown();
}

void main_resume_action(void)
{
	ui_wakeup();
}

void main_sof_action(void)
{
	if (!main_b_cdc_enable)
	return;
	ui_process(udd_get_frame_number());
}

#ifdef USB_DEVICE_LPM_SUPPORT
void main_suspend_lpm_action(void)
{
	ui_powerdown();
}

void main_remotewakeup_lpm_disable(void)
{
	ui_wakeup_disable();
}

void main_remotewakeup_lpm_enable(void)
{
	ui_wakeup_enable();
}
#endif

bool main_cdc_enable(uint8_t port)
{
	main_b_cdc_enable = true;
	// Open communication
	uart_open(port);
	return true;
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
	// Close communication
	uart_close(port);
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable) {
		// Host terminal has open COM
		ui_com_open(port);
		}else{
		// Host terminal has close COM
		ui_com_close(port);
	}
}


/*! \brief Main function. Execution starts here.
 */
int main(void)

{	struct events_resource example_event;
	struct events_hook hook;

	irq_initialize_vectors();
	cpu_irq_enable();

	// Initialize the sleep manager
	sleepmgr_init();

	
	system_init();
	system_interrupt_enable_global();
	
	//! [setup_tcc]
	configure_tcc();
	configure_tcc_callbacks();

	
	//setup GPIO
	configure_port_pins();
	
	//setup system event
	configure_event_channel(&example_event);
	configure_event_user(&example_event);
	configure_event_interrupt(&example_event, &hook);
	

	//setup rtc
	configure_rtc_calendar();
	configure_rtc_callbacks();
	rtc_calendar_enable(&rtc_instance);
	
	//! [set initial time]
	struct rtc_calendar_time time;
	rtc_calendar_get_time_defaults(&time);
	time.year = 2016;
	time.month = 10;
	time.day = 15;
	time.hour = 15;
	time.minute = 36;
	time.second = 59;
	rtc_calendar_set_time(&rtc_instance, &time);
	
	configure_adc();
	
	ui_init();
	ui_powerdown();
	
	// Start USB stack to authorize VBus monitoring
	udc_start();
	
	while (events_is_busy(&example_event)) {
		/* Wait for channel */
	};

	// The main loop manages only the power mode
	// because the USB management is done by interrupt
	while (true) {
		if (1==usbMsgInFlag){
			processCommandMsg();
		}
		if ((0 == system_busy_flag) && (1 == commandReady))
		{
			execute_system_command();
		}
			
	}
}
