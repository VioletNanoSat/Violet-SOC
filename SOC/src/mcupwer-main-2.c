//========================================================================
// Violet Satellite Power Board MCU
//========================================================================
// Adam Wrobel, Will Voge, John Fu, Vivek Gaddam
// Spring 2014
// Updated with COMM FIX, POWER UP SEQUENCE, SPOOF SOC, and TIMER CHECKING

#include "mcupwr-Defs.h"
#include "vcp_library.h"
#include "crclib.h"
#include "uart.h"
#include "math.h"
#include "battery_data.h"
#include "soc.h"

// UART file descriptor for debugging purposes
//FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void set_component( uint8_t svit_index, uint8_t name, uint8_t switch_num, uint8_t switch_state, uint8_t V_mux_num, uint8_t V_mux_sel, uint8_t V_upper_limit, uint8_t V_lower_limit, uint8_t I_mux_num, uint8_t I_mux_sel, uint8_t I_upper_limit, uint8_t I_lower_limit, uint8_t T_mux_num, uint8_t T_mux_sel )
{
  svit[svit_index].name = name;
  svit[svit_index].switch_num = switch_num;
  svit[svit_index].switch_state = switch_state;
  svit[svit_index].force_on = 0;

  svit[svit_index].V_mux_num = V_mux_num;
  svit[svit_index].V_mux_sel = V_mux_sel;
  svit[svit_index].V_upper_limit = V_upper_limit;
  svit[svit_index].V_lower_limit = V_lower_limit;
  svit[svit_index].V_sample_index = 0;
  svit[svit_index].V_critical_value = 0;

  svit[svit_index].I_mux_num = I_mux_num;
  svit[svit_index].I_mux_sel = I_mux_sel;
  svit[svit_index].I_upper_limit = I_upper_limit;
  svit[svit_index].I_lower_limit = I_lower_limit;
  svit[svit_index].I_sample_index = 0;
  svit[svit_index].I_critical_value = 0;
  svit[svit_index].T_mux_num = T_mux_num;
  svit[svit_index].T_mux_sel = T_mux_sel;
  svit[svit_index].T_sample_index = 0;
}

void initialize_svit( void )
{ 
  uint8_t svit_index;
  // ADC
  // Vref = AVCC = 5 V
  // ADC clock = 16 MHz / 128 = 125 kHz

  // set Vref to AVCC, left-adjust result into ADCH
  ADMUX = ( 1 << ADLAR ) | ( 1 << REFS0 );

  // enable ADC and set ADC division factor to 128
  ADCSRA = ( 1 << ADEN ) | ( 1 << ADIE ) | ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 );

  //sample_index = 0;
  svit_index = 0;

  //                index           name      switch_num  switch_state   V_mux_num   V_mux_sel     V_upper_limit		 I_mux_num  I_mux_sel     I_upper_limit      T_mux_num   T_mux_sel
  set_component( svit_index++,  SPECTROMETER,  SW_EN7 ,     SW_ON,          MUX0,       11,      V_THRESHOLD_VALUE, V_UNDER_VALUE,       MUX0,        6 ,    I_THRESHOLD_VALUE,   I_UNDER_VALUE,    MUX_NULL,       0     );
  set_component( svit_index++,  STAR_TRACKER,  SW_EN8 ,     SW_ON,          MUX0,       23,      V_THRESHOLD_VALUE,  V_UNDER_VALUE,     MUX0,        7 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,    MUX1    ,       7     );        
  set_component( svit_index++,  FC_5V       ,  SW_EN1 ,     SW_ON,          MUX1,       19,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,    MUX1,        0 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,    MUX_NULL,       0     );        
  set_component( svit_index++,  FC_3_3V     ,  SW_EN1 ,     SW_ON,          MUX2,       13,      V_THRESHOLD_VALUE,    V_UNDER_VALUE,   MUX2,        14,    I_THRESHOLD_VALUE, I_UNDER_VALUE,  MUX_NULL,       0     );        
  set_component( svit_index++,  GPS_1       ,  SW_EN2 ,     SW_ON,          MUX1,       21,      V_THRESHOLD_VALUE,  V_UNDER_VALUE,    MUX1,        1 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  
  set_component( svit_index++,  GPS_2       ,  SW_EN3 ,     SW_ON,          MUX1,       26,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,    MUX1,        2 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );    
  set_component( svit_index++,  CDH_IB      ,  SW_EN4 ,     SW_ON,          MUX1,       29,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,    MUX1,        3 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  
  set_component( svit_index++,  HEATER_1    ,  SW_EN5 ,     SW_ON,          MUX1,       14,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX1,        31,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     ); 
  set_component( svit_index++,  HEATER_2    ,  SW_EN6 ,     SW_ON,          MUX1,       27,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX1,        4 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  
  set_component( svit_index++,  CMG         ,  SW_EN9 ,     SW_ON,          MUX1,       15,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX1,        8 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     ); 
  set_component( svit_index++,  SUN_SENSOR  ,  SW_EN10,     SW_ON,          MUX1,       25,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX1,        9 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  
  set_component( svit_index++,  RADIO_1     ,  SW_EN11,     SW_OFF,          MUX1,       24,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX1,        10,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     ); 
  set_component( svit_index++,  RADIO_2     ,  SW_EN12,     SW_OFF,          MUX1,       17,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX1,        16,    I_THRESHOLD_VALUE, I_UNDER_VALUE,    MUX_NULL,       0     );   
  set_component( svit_index++,  MAESTRO     ,  SW_EN13,     SW_ON,          MUX1,       22,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX1,        12,    I_THRESHOLD_VALUE, I_UNDER_VALUE,    MUX2    ,       8     );  
  set_component( svit_index++,  MAGNETOM    ,  SW_EN14,     SW_ON,          MUX0,       4 ,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        6 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,  MUX_NULL,       0     );    
  set_component( svit_index++,  FOG_15V     ,  SW_EN16,     SW_ON,          MUX2,       21,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX_NULL,    0,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  // Fake Data changed mux 2 20 to mux null 0
  set_component( svit_index++,  FOG_5V      ,  SW_EN15,     SW_ON,          MUX1,       18,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX1,        13,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  // Fake Data
  set_component( svit_index++,  TORQUER_1   ,  SW_NULL,     SW_OFF,          MUX1,       28,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX1,        5 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  
  set_component( svit_index++,  TORQUER_2   ,  SW_NULL,     SW_OFF,          MUX2,       15,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX2,        16,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     ); 
  set_component( svit_index++,  TORQUER_3   ,  SW_NULL,     SW_OFF,          MUX2,       17,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX2,        18,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     ); 
  set_component( svit_index++,  BATTERY_1   ,  SW_NULL,     SW_ON,          MUX0,       1 ,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        2 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX2    ,       9     );  // Fake Data  
  //set_component( svit_index++,  BATTERY_1_b  ,  SW_NULL,     SW_ON,          MUX0,       1 ,      V_THRESHOLD_VALUE,      MUX1,        23 ,    I_THRESHOLD_VALUE,    MUX2    ,       9     );  // Fake Data  
  set_component( svit_index++,  BATTERY_2   ,  SW_NULL,     SW_ON,          MUX2,       20,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX2,        30,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX2    ,       11    );  // Fake Data  
  set_component( svit_index++,  SOLAR_FULL  ,  SW_NULL,     SW_ON,          MUX0,       7 ,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        3 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );
  set_component( svit_index++,  SOLAR_1     ,  SW_NULL,     SW_ON,          MUX0,       18,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        20,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX2    ,       0     );  
  set_component( svit_index++,  SOLAR_2     ,  SW_NULL,     SW_ON,          MUX0,       19,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        21,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX2    ,       1     ); 
  set_component( svit_index++,  SOLAR_3     ,  SW_NULL,     SW_ON,          MUX0,       10,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        22,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX2    ,       2     );  
  set_component( svit_index++,  SOLAR_4     ,  SW_NULL,     SW_ON,          MUX0,       15,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        23,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX2    ,       3     );  
  set_component( svit_index++,  SOLAR_5     ,  SW_NULL,     SW_ON,          MUX0,       9 ,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        24,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX2    ,       4     );  
  set_component( svit_index++,  SOLAR_6     ,  SW_NULL,     SW_ON,          MUX0,       14,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        25,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     ); 
  set_component( svit_index++,  SOLAR_7     ,  SW_NULL,     SW_ON,          MUX0,       16,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        26,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  
  set_component( svit_index++,  SOLAR_8     ,  SW_NULL,     SW_ON,          MUX0,       8 ,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        27,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  
  set_component( svit_index++,  SOLAR_9     ,  SW_NULL,     SW_ON,          MUX0,       13,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        28,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX2    ,       5     );  
  set_component( svit_index++,  SOLAR_10    ,  SW_NULL,     SW_ON,          MUX0,       12,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        29,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  
  set_component( svit_index++,  SOLAR_11    ,  SW_NULL,     SW_ON,          MUX0,       11,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        30,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  
  set_component( svit_index++,  SOLAR_12    ,  SW_NULL,     SW_ON,          MUX0,       17,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        31,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX_NULL,       0     );  
  set_component( svit_index++,  POWER_BOARD ,  SW_NULL,     SW_ON,          MUX0,       5 ,      V_THRESHOLD_VALUE,   V_UNDER_VALUE,      MUX0,        0 ,    I_THRESHOLD_VALUE, I_UNDER_VALUE,   MUX2    ,       6     );  // Fake Data
}

void initialize( void )
{
   // pin initialization
  DDRA  = 0b11111111;
  PORTA = 0b00000111;

  DDRB  = 0b11111111;
  PORTB = 0b11100000;

  DDRC  = 0b11111111;
  PORTC = 0b11111111;

  DDRD  = 0b11111011;
  PORTD = 0b11110000;

  DDRE  = 0b11111110;
  PORTE = 0b00000000;

  DDRF  = 0b11110000;
  //0b11111111;	//testing
  PORTF = 0b00000000;

  DDRG  = 0b00011111;
  PORTG = 0b00000100;

  //---------------------------------------------------------------------
  // timer 0
  //---------------------------------------------------------------------
  // used for transmitting telemetry packet at 1 Hz
  //
  // mode = clear on match
  // prescalar = 1024
  // compare value = 124
  // compare match interrupt freq = (14.7456 MHz / 1024 / 100) = 144 Hz
  // use timer0_counter to get 144 / 144 = 1 Hz

  // enable clear on match interrupt
  TIMSK = ( 1 << OCIE0 );
  OCR0 = 71;

  // enable clear on match mode, set prescalar to 1024
  TCCR0 = ( 1 << WGM01 ) | ( 1 << CS02 ) | ( 1 << CS01 ) | ( 1 << CS00 );

  timer0_counter[0] = 99;
  timer0_counter[1] = 199;
  
  //---------------------------------------------------------------------  
  // timer 1
  //---------------------------------------------------------------------
  /* used for waiting up to 24 hours if the power board does not receive
  a command
  
  Ex:
  mode = clear on match
  prescalar = 1024
  Timer counts up to OCR1B = 28800 and enters ISR(TIMER1_COMPB_vect)
  clock frequency after prescaling = (14.7456 MHz / 1024) = 14.4 kHz
  period of interrupts = 28800 / 14400 Hz = 2 s/interrupt
  use timer1_counter to get a total period of 2*65535 =  s (24 hours is 86400 seconds)
  */
  
  // Enable clear on match interrupt for the 16 bit timer/counter 1, register A
  OCR1A = 28800;		//28800 corresponds to 2 seconds

  // Enable clear on match mode, set prescalar to 1024.
  // CS[2:0] = 101 (1024 prescalar)
  // WGM[3:0] = 0100 (Clear Timer on Compare (CTC) when timer matches OCR1A)
  
  TCCR1B = ( 1 << WGM12 ) | ( 1 << CS12 ) | ( 1 << CS10 );
  // Explicit Default Defs
  // TCCR1A |= 0;
  // TCNT1 = 0;
  timer1_counter[0] = CYCLE_COUNTER;	//CYCLE_COUNTER   = 900 for 2 seconds interrupt handler = 1800 secs (30 minutes)
  timer1_counter[1] = CYCLE_COUNTER_2;	//CYCLE_COUNTER_2 = 300 for 2 seconds interrupt handler = 600 secs  (10 minutes)
  //---------------------------------------------------------------------  
  // End of timer 1 Setup
  //--------------------------------------------------------------------- 
  
  // communication
  uart_init();

  // for use in debugging
  // stdout = stdin = stderr = &uart_str;

  tel_packet_size[0] = 0;
  tel_packet_size[1] = 0;
  tel_packet_index[0] = 0;
  tel_packet_index[1] = 0;
  uart_vcp_buff[0] = (vcp_ptrbuffer*)malloc( sizeof( vcp_ptrbuffer ) );
  uart_vcp_buff[1] = (vcp_ptrbuffer*)malloc( sizeof( vcp_ptrbuffer ) );
  vcpptr_init( uart_vcp_buff[0], uart_message_buff[0], BUFFER_SIZE );
  vcpptr_init( uart_vcp_buff[1], uart_message_buff[1], BUFFER_SIZE );

  rx_flag[0] = 0;
  rx_flag[1] = 0;

  // svit
  initialize_svit();

  // rev up those interrupts
  sei();

  //ADC conversions
  adc_flag = 1;
	adc_component = 0;
	adc_sensor_type = ADC_INIT;//get default case on first interation as to not enter switch
  ADC_high = 0;

  V_upper_val_change = 0;
  I_upper_val_change = 0;

  //---------------------------------------------------------------------  
  // SOC Initializations
  //--------------------------------------------------------------------- 
  // and shunt and safe transmit flags
  safe_mode = 0;
  transmit_safe = 0;
  transmit_shunt = 0;
  been_to_safe = 0;
  been_to_shunt = 0;
  assign_charge_fit();
  assign_discharge_fit();
  
  
  // First get battery voltage so that the SoC can
  // accurately determine whether batteries are charging or discharging
  batt1_voltage = 0xff;
  batt2_voltage = 0xff;
  charging = 0xff;
  chargeforward = 0;
  chargebackward = 0;
  debug = 0;
  debug2 = 0;
  percent = 0;
  soc = 0;
  high = 0;
  low = 0;
  limit_check_overriden = 0; // Initially limit checking is NOT OVERRIDEN
  isCharging = 0;
  hasCheckedCurr = 0;
  
  // Sets SOC initial conditions
  SOC_init(state_of_charge,coloumb_count);
  //---------------------------------------------------------------------  
  // END SOC Initializations
  //--------------------------------------------------------------------- 

  //---------------------------------------------------------------------  
  // Timer Initializations
  //---------------------------------------------------------------------
  cdh_heartbeat_flag = 0;	//Default: do not have flag to restart components on
  rad_torq_flag = 1;		//Indicates need to delay radio/torquer on signals later
  SVIT_t *component; 	// Initialize Radios to be OFF
  component = &svit[components[RADIO_1]];
  switch_off( component->switch_num );
  component = &svit[components[RADIO_2]];
  switch_off( component->switch_num );
  component = &svit[TORQUER_1];	// Initialize Torque Coils to be OFF
  component->switch_state = SW_OFF;
  component = &svit[TORQUER_2];
  component->switch_state = SW_OFF;
  component = &svit[TORQUER_3];
  component->switch_state = SW_OFF;
  torquer_off(TORQUER_1);
  torquer_off(TORQUER_2);
  torquer_off(TORQUER_3);
  //---------------------------------------------------------------------  
  // END Timer Initializations
  //--------------------------------------------------------------------- 
}

inline void receive_message( uint8_t uart, uint8_t* message, uint8_t message_size )//was inline
{
  uint8_t command = message[VCP_COMMAND_FIELD];
  uint8_t payload = message[VCP_PAYLOAD_FIELD];

  switch(command)
  {
    case VCP_COMPONENT_ON:
	
	  // Reset CDH IB Heartbeat timer
	  timer1_counter[1] = CYCLE_COUNTER_2;
	  
      if ( svit[payload].switch_num != SW_NULL )
      {
        switch_on( svit[payload].switch_num );
        svit[payload].switch_state = 1;
      }
      transmit_packet( uart, VCP_ACK, command );
      break;
    case VCP_COMPONENT_OFF:
      if ( svit[payload].switch_num != SW_NULL )
      {
        switch_off( svit[payload].switch_num );
        svit[payload].switch_state = 0;
      }
      transmit_packet( uart, VCP_ACK, command );
      break;
    case VCP_POWER_CYCLE:
      if ( svit[payload].switch_num != SW_NULL )
      {
        switch_off( svit[payload].switch_num );
        svit[payload].switch_state = 0;
      }
      _delay_us(1);
      if ( svit[payload].switch_num != SW_NULL )
      {
        switch_on( svit[payload].switch_num );
        svit[payload].switch_state = 1;
      }
      transmit_packet( uart, VCP_ACK, command );
      break;
    case VCP_TORQ_CTRL:
      transmit_packet( uart, VCP_ACK, command );
      break;
    case VCP_GET_TELEMETRY:
      transmit_packet( uart, VCP_POWER_TELEMETRY, 0);
      break;
    case VCP_FORCE_ON:
      if ( svit[payload].switch_num != SW_NULL )
      {
        switch_on( svit[payload].switch_num );
        svit[payload].switch_state = 1;
        svit[payload].force_on = 1;  
      }
      transmit_packet( uart, VCP_ACK, command );                                                            
      break;
    case VCP_CRIT_V_CHANGE:
      if ( svit[payload].switch_num != SW_NULL )
      {
        svit[payload].V_upper_limit = message[VCP_PAYLOAD_FIELD + 1];
        V_upper_val_change= message[VCP_PAYLOAD_FIELD + 1];
      }
      //transmit_packet( uart, VCP_ACK, 0);
      break;
        case VCP_CRIT_I_CHANGE:
      if ( svit[payload].switch_num != SW_NULL )
      {
        svit[payload].I_upper_limit = message[VCP_PAYLOAD_FIELD + 1];
        I_upper_val_change= message[VCP_PAYLOAD_FIELD + 1];
      }
      //transmit_packet( uart, VCP_ACK, 0);
      break;
    default:
      transmit_packet( uart, VCP_INVALID_COMMAND, 0 );
      break;
  }
}

/*
SOC subfunctions
*/

int soc_init(state_of_charge,coloumb_count,h){
	
	int OCV;
	
	// Gathers a number of samples for Terminal Voltage and Net Current.
	// Net current out of the batteries should correspond with [+/-] input current to algorithm.
	
	while(sample_counter < sample_size){
		sampled_i[sample_counter] = input_current;
		sampled_v[sample_counter] = input_voltage;
		sample_counter++;
		delay(10);
	}
	
	sample_counter = 0;
	
	//First measurement for SOC
	
	algorithm_1(sampled_i,f);
	compute_vf(f,sampled_v,vf);
	compute_uf(f,uf);
	max=max_uf(uf,max_index);
	OCV=compute_OCV(vf,max,max_index);
	compute_h(OCV,vf,uf,h);
	// SOC_index=round(OCV*100);
	// SOC=SOC_lookup_table[SOC_index];
	// soc_look_up(OCV,SOCv);
	SOCv = 95.2;
	// coulomb_count=current_lookup_table[round(SOC*100)];
	compute_h(OCV,vf,uf,h)
	
}

void soc_data_gather(double sampled_i[],double sampled_v[],double h[]){
	int sample_counter = 0;
	int ixh = 0;
	while(sample_counter < sample_size){
		sampled_i[sample_counter] = input_current;
		sampled_v[sample_counter] = input_voltage;
		sampled_v[sample_counter] = compute_new_voltage(sampled_v[sample_counter],sampled_i[sample_counter],h,sample_counter,ixh);
		sample_counter++;
	}
}

// Calculates f(t) used in the SOC algorithm.
// Can add an extra input n for size of array since it will always be fixed.
void algorithm_1(double sampled_i[], double f[]){
	int n=ELEMENT_COUNT(sampled_i);
	double f_norm[n];
	double i_norm[n];
	for(int i=0;i<n;i++){
		f_norm[i]=(double) delta_function(i)/sampled_i[0];
		i_norm[i]=(double) sampled_i[i]/sampled_i[0];
		
		for(int i=1;i<n;i++){
			for(int j=n;j>i;j--){
				f[j]=f[j]-f_norm[j-i]*i_norm[i];
				i_norm[j]=i_norm[j]-i_norm[j-i]*i_norm[i];
			}
		}
	}
}

void compute_uf(double f[], double uf[]){
	int size=ELEMENT_COUNT(f);
	double u[size];
	for(int i=0;i<size;i++){
		u[i]=step_function(i);
	}
	convolve(f,ELEMENT_COUNT(f),u,ELEMENT_COUNT(u),uf);
}

void compute_vf(double f[], double sampled_v[], double vf[]){
	convolve(f,ELEMENT_COUNT(f),sampled_v,ELEMENT_COUNT(sampled_v),vf);
}

double max_uf(double uf[], int max_index){
	double max=0;
	for(int i=0;i<ELEMENT_COUNT(uf);i++){
		if(uf[i]>max){
			max=uf[i];
			max_index=i;
		}
	}
	return max;
}

double compute_OCV(double vf[],double max, int max_index){
	return (double) vf[max_index]/max;
}

void compute_h(double OCV, double vf[], double uf[], double h[]){
	int size=ELEMENT_COUNT(h);
	for int(i=0;i<size;i++){
		h[i]=vf[i]-OCV*uf[i];
	}
}

int delta_function(int i){
	return i == 0;
}

int step_function(int i){
	return i >= 0;
}

double compute_new_voltage(double sampled_v, double sampled_current, double h[], unsigned int sample_counter, double ixh){
	double term=h[sample_counter]*sampled_current;
	ixh=ixh+term;
	return sampled_v - ixh;
}


uint8_t antioptimizer = 0;
double StateofCharge(void){
	
	/*
	This marks the start of the SOC algorithm.
	For this we assume only that the battery has been off prior to this point.
	*/
  
	// Returns initial conditions for voltage-based and current-based algorithms
	
	
	soc_data_gather(sampled_i,sampled_v,h);
	algorithm_1(sampled_i,f);
	compute_vf(f,sampled_v,vf);
	compute_uf(f,uf);
	max=max_uf(uf,max_index);
	OCV=compute_OCV(vf,max,max_index);
	// soc_look_up(OCV,SOCv);
	SOCv = OCV;
	compute_h(OCV,vf,uf,h);
	
	
	
	return SOC;
	
	// Ignoring what is below this...
	
	
	
	if(batt1_voltage < 0xFF  &&  hasCheckedCurr){
		
		if(solar1_current > 0x00){
			isCharging = 1;
		}else{
			isCharging = 0;
		}
	
		
		if(  (isCharging && (batt1_voltage <= 0x7E) && (batt1_voltage > 0x64)) || (!isCharging && (batt1_voltage <= 0x77) && (batt1_voltage > 0x64)) ){
			antioptimizer++;
			if(isCharging){
				
				ltOffset =  (batt1_voltage <= 0x69) ? (batt1_voltageLow >> 6) :
				            ((batt1_voltage >= 0x6A) && (batt1_voltage <= 0x73)) ? (((batt1_voltageLow >> 6) + 1) % 0x04) :
							(batt1_voltage >= 0x74) ? (((batt1_voltageLow >> 6) + 2) % 0x04) :
							0x00; 
				percent = (charge20Pc[(batt1_voltage - 0x65)*4 + ltOffset] - 11) / (10.57);
				antioptimizer+=6;
				
			}else{
				
				ltOffset =  (batt1_voltage <= 0x69) ? (batt1_voltageLow >> 6) :
							((batt1_voltage >= 0x6A) && (batt1_voltage <= 0x73)) ? (((batt1_voltageLow >> 6) + 1) % 0x04) :
							((batt1_voltage >= 0x74) && (batt1_voltage <= 0x76)) ? (((batt1_voltageLow >> 6) + 2) % 0x04) :
							0x00;
				
				percent = 100 - ((discharge20Pc[(0x77 - batt1_voltage)*4 - ltOffset]) ) / (10.07);
			}
		}else if(batt1_voltage <= 0x64){
			percent = -1;
		}else if(  ((batt1_voltage > 0x7E)&&isCharging) || ((!isCharging)&&(batt1_voltage > 0x77)) ){
			percent = 108;
		}
	}
	
	if(solar1_current < 0xFF && solar1_current > 0x00){
		antioptimizer++;
	}
	
	
}



/* Assigns values to the Fourier coefficients of the charge approximation */
void assign_charge_fit( void ) {
  charge_max_time = 10188;
  // x is normalized by mean 13.13 and std 0.1754
  p1 = 18.57;
  p2 = 184.8;
  p3 = 663.6;
  p4 = 838.6;
  //where x is normalized by mean 13.7 and std 0.1558
  q1 = 78.49;
  q2 = 543.1;
  q3 = 2427;
  q4 = 5587;

}

/* Assigns values to the Fourier coefficients of the discharge approximation */

void assign_discharge_fit( void ) {
  discharge_max_time = 17547;
  f0 = -1.543291233254410E4;
  f1 = -1.819810644993805E4;
  b1 = -2.548949173344369E4;
  f2 = 1.141365113433748E4;
  b2 = -2.190399298389397E4;
  f3 = 1.453849538420288E4;
  b3 = 7.700208204268025E2;
  f4 = 1.965482651710955E3;
  b4 = 5.504490550919407E3;
  f5 = -9.466488439471518E2;
  b5 = 8.270458368650347E2;
  w = 2.725333178515558;
  
  h0 = 2.979485572689352E8;
  h1 = 2.088787459098652E8;
  g1 = 4.359717315569648E8;
  h2 =  -1.569778955752849E8;
  g2 =  1.969854815603661E8;
  h3 =  -7.513816845838763E7;
  g3 =  -1.633385950799686E7;
  h4 =   -2.642581886559125E6;
  g4 =   -1.016608352073227E7;
  w2 =  0.364797662747743;
}


void compareVoltage( void ) {
  SVIT_t *component;
  //component = &svit[BATTERY_1_b];
  //chargebackward = average_samples( component->I_samples );
  component = &svit[BATTERY_1];
  chargeforward= average_samples( component->I_samples );
  if (chargeforward > 2){
    charging = 0xdd;//discharging
    }
  else{
    charging = 0xcc;
  } 
/*	if (prev_batt1_voltage < batt1_voltage) {
		charging = 0xcc;
	}
	else if (prev_batt1_voltage > batt1_voltage) {
		charging = 0xdd;
	}
	else {} */
}


/*  
Spoof Limit Checking: There is a known issue with the Vsense values, but 
we should at least get the framework set up for limit checking on arbitrary values.
The power board should have upper and lower limits for vsense and csense data, 
and turn off components if their voltage/current is too high. ONLY CHECKS
BATTERY 1 VOLTAGE LINE RIGHT NOW.
*/
void limit_check( void ) {
	unsigned char sw;
	SVIT_t *component;

	// turn off all switches and send ack_command w/ value of SAFE_MODE
	if (percent < SAFE_MODE  && !((percent<-.56) && (percent>-.57) )   ) {
		safe_mode = 1;
		for (sw = 0; sw < sizeof(components); sw++) {
			component = &svit[components[sw]];
			switch_off( component->switch_num );
			component->switch_state = SW_OFF;
		}
		component = &svit[TORQUER_1];
		component->switch_state = SW_OFF;
		component = &svit[TORQUER_2];
		component->switch_state = SW_OFF;
		component = &svit[TORQUER_3];
		component->switch_state = SW_OFF;
		torquer_off(TORQUER_1);
		torquer_off(TORQUER_2);
		torquer_off(TORQUER_3);
		// Only transmit once
		if (!transmit_safe && !been_to_safe) { 
			transmit_packet( 0, VCP_ACK, SAFE_MODE);
			transmit_safe = 1;
		}
		else {}
		been_to_safe = 1;
		transmit_safe = 1;
	}
	else if (percent > SHUNT_MODE  && isCharging) {
	    safe_mode = 0;
		// turn on the maestro and send ack_command w/ value of SHUNT_MODE
		component = &svit[MAESTRO];
		switch_on( component->switch_num );
		component->switch_state = SW_ON;
		// Only transmit once
		if (!transmit_shunt && !been_to_shunt) {
			transmit_packet( 0, VCP_ACK, SHUNT_MODE);
		}
		else {}	
		been_to_shunt = 1;
		transmit_shunt = 1;	
	}
	else {} // To avoid annoying compile warning 
}




// Calculates percent state of charge
void calcSOC( void ) {
  uint16_t ADC_val = (high << 2) + (low >> 6);
  //10-bit resolution
  //real = (((batt1_voltage + 1)*5)/1024 - 0.0315)*5.5556645; // 16-bit resolution
  real = (ADC_val + 1)*0.02712726806640625-0.17500343175;
  //8-bit resolution
  //real = (batt1_voltage+1)*0.1085090723-0.66667974;
  
  // Since approximations are periodic, limit batt_time values within a certain voltage range 
  if (real > 14.2) {
	 real = 14.2;
  }
  else if (real < 11.2) {
	 real = 11.2;
  }
  
  //charging = 0xcc;
  if (charging == 0xdd) // change back to dd for tomorrow 
  {
  	if (real > 12.43) 
	{
      debug = 0x0A;
      batt_time = f0 + f1*cos(real*w) + b1*sin(real*w) +
                  f2*cos(2*real*w) + b2*sin(2*real*w) + 
				  f3*cos(3*real*w) + b3*sin(3*real*w) +
				  f4*cos(4*real*w) + b4*sin(4*real*w) +
				  f5*cos(5*real*w) + b5*sin(5*real*w);
	}
    else 
	{
	  debug = 0x0B;
	  batt_time = h0 + h1*cos(real*w2) + g1*sin(real*w2) +
                  h2*cos(2*real*w2) + g2*sin(2*real*w2) +
                  h3*cos(3*real*w2) + g3*sin(3*real*w2) +
                  h4*cos(4*real*w2) + g4*sin(4*real*w2);
	}
  	percent = 100*(1 - batt_time/discharge_max_time);
  }
  else 
  {
    float temp_real = 0;
    if (real < 13.35)
	{
	  debug = 0x0C;
	  // voltage is normalized by mean 13.13 and std 0.1754 (real - 13.13)/0.1754
	  temp_real = (real - 13.13)*5.701254;
	  batt_time = p1*pow(temp_real,3) + p2*pow(temp_real,2) + p3*temp_real + p4;
	}
	else 
	{
	  debug = 0x0D;
	  // voltage is normalized by mean 13.7 and std 0.1558 (real - 13.7)/0.1558;
	  temp_real = (real - 13.7)*6.418485;
      batt_time = q1*pow(temp_real,3) + q2*pow(temp_real,2) + q3*temp_real + q4;
	}
  	percent = 100*(batt_time/charge_max_time);
  }
  
  // Percent limiting for periodic function
  if (percent < 1) 
  { 
  	debug2 = 0xA0;
  	soc = 0; 
  }
  else if (percent > 99) 
  { 
  	debug2 = 0xB0;
  	soc = 255;
  }
  else { 
  	debug2 = 0xC0;
  	soc = (char)floor((int)(percent)); 
  }
}

/*
Using the SoC, the power board must autonomously turn on the CDH IB, 
Radio IBs, Flight Computer, in an arbitrary level at arbitrary charge levels. 
*/
void on_sequence( void ) {
// CDH IB: 75% charge
// Radio IBs: 80% charge
// Flight Computer : 85% charge
	SVIT_t *component;
	// FC_5V
	if (percent > 85) {
		component = &svit[FC_5V];
		switch_on( component->switch_num );
		component->switch_state = SW_ON;
	}
	// RADIOs 1 and 2
	if (percent > 80) {
		component = &svit[RADIO_1];		
		switch_on( component->switch_num );
		component->switch_state = SW_ON;
		component = &svit[RADIO_2];
		switch_on( component->switch_num );
		component->switch_state = SW_ON;
	}
	// CDH IB
	if (percent > 75) {
		component = &svit[CDH_IB];
		switch_on( component->switch_num );
		component->switch_state = SW_ON;
	}
}




// MAIN
int main( void ) 
{  
  initialize();
  //fprintf( stdout, "uart initialized\n" );
	
  //Testing
  cntr = 0;

  while(1)
  {
	cntr = 1 - cntr; //Testing
	
    if ( timer0_counter[1] == 0 )
    {
        timer0_counter[1] = 199;
        transmit_packet( 1, VCP_POWER_TELEMETRY, 0);
    }
	  if ( timer0_counter[0] == 0 )
    {
        timer0_counter[0] = 199;
        transmit_packet( 0, VCP_POWER_TELEMETRY, 0);
    }
	  if ( rx_flag[0] > 0 )
    {
        rx_flag[0]--;
        receive_message( 0, uart_vcp_buff[0]->message, uart_vcp_buff[0]->index );
        vcpptr_init( uart_vcp_buff[0], uart_message_buff[0], BUFFER_SIZE );
    }
	  if ( rx_flag[1] > 0 )
    {
        rx_flag[1]--;
        receive_message( 1, uart_vcp_buff[1]->message, uart_vcp_buff[1]->index );
        vcpptr_init( uart_vcp_buff[1], uart_message_buff[1], BUFFER_SIZE );
    }
    if (adc_flag == 1)
    {
        adc_flag = 0;
        read_VIT();
		//calcSOC();
		StateofCharge();

		/*
		Manual Override on Limit Checking: The power board must be able to receive a 
		command to disable and/or change the limits in the limit checking code
		*/
		if (!limit_check_overriden) {
			limit_check(); // First determine if voltage is within valid range, then switch
		}

		if ( adc_component == 23 ) 
      	{ //23ish anything after battery values are calculated
  			  compareVoltage();
	   	}
    }	
	
	//Restart all board components if CDH-IB heartbeat timeout
	if (cdh_heartbeat_flag == 1){
		//Restart all components
		//PORTC ^= 0x02;
		//Set flag off
		cdh_heartbeat_flag = 0;
		
		//reset appropriate timer counter to cycle_counter_2     //DOUBLECHECK that this adheres to the spec!!!
		timer1_counter[1] = CYCLE_COUNTER_2;
	}
  
    //End of While(1)
  }
//End of main
}
