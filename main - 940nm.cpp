#include <SPI.h>
#include <math.h>
#include <Arduino.h>
#include <stdio.h>

// stepper driver libraries
#include "DRV8834.h"
#include <BasicStepperDriver.h>
#include <SyncDriver.h>
#include <A4988.h>
#include <DRV8825.h>
#include <DRV8880.h>
#include <MultiDriver.h>
/*
 * Debugging messages over serial
 *
 * defined: Debug serial messages
 * commented out: no debug messages
 */
#define DEBUG
//#define DEBUG_SPI
#define DEBUG_STEPPER

// leave these 7 lines (don't comment them out)
#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINT2(x,y) Serial.print(x,y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINT2(x,y)
#endif

// leave these 7 lines (don't comment them out)
#ifdef DEBUG_SPI
  #define DEBUG_SPI_PRINT(x) Serial.print(x)
  #define DEBUG_SPI_PRINT2(x,y) Serial.print(x,y)
#else
  #define DEBUG_SPI_PRINT(x)
  #define DEBUG_SPI_PRINT2(x,y)
#endif

// leave these 7 lines (don't comment them out)
#ifdef DEBUG_STEPPER
  #define DEBUG_PRINT_STEPPER(x) Serial.print(x)
  #define DEBUG_PRINT_STEPPER2(x,y) Serial.print(x,y)
#else
  #define DEBUG_PRINT_STEPPER(x)
  #define DEBUG_PRINT_STEPPER2(x,y)
#endif

/*
 * Status codes, return from functions with datatype defined below
 */
typedef unsigned int STATUS;
#define SUCCESS              0
#define ERR_POWER_UP         1
#define ERR_SET_LED_MOD_FREQ 2
#define ERR_SET_INT_TIME     3
#define ERR_START_MEAS       4
#define ERR_FACTORY_SETTINGS 5
#define ERR_SPI_RECOVERY_SEQUENCE 6
#define ERR_UNKNOWN          7

typedef unsigned int QUALITY_INDICATOR;
#define WEAK_ILLUMINATION         0
#define SUFFICIENT_ILLUMINATION   1
#define EXCELLENT_ILLUMINATION    2
#define HIGH_ILLUMINATION         3

// typedef unsigned int VALIDITY_INDICATOR;
// #define VALID                 0
// #define SATURATED             1
// #define MAX_SIGNAL_LIMIT      2
// #define LOW_SIGNAL_LIMIT      3

/*
 * Serial Settings (for debugging messages)
 */
#define SERIAL_BAUD 115200

/*
 * SPI Settings for EPC610 ToF Camera
 */
//#define EPC_SS_PIN 0      // Teensy 2.0
//#define EPC_SS_PIN 20     // Teensy++ 2.0
#define EPC_SS_PIN 10       // Teensy LC/3.0/3.1/3.2/3.5/3.6
#define EPC_LED_HIGH_ENABLE 8
#define EPC_LED_LOW_ENABLE 7
#define EPC_SPI_CLK 10000000 // EPC Speed: 10 Mbit/s
//#define EPC_SPI_CLK 8000000 // EPC Speed: 8 Mbit/s
//#define EPC_SPI_CLK 100000  // EPC Speed: 1 Mbit/s
SPISettings settingsEPC(EPC_SPI_CLK, MSBFIRST, SPI_MODE0);

/*
 * Button 
 */
#define BUTTON1_PIN		5 
/*
 * Defines. See EPC610 datasheet
 */
#define POWERUP_MAX_TRIES 100
#define MAX_ROW_READY_TRIES 5000
#define MAX_SAT_PIXELS_TRIES 400

// write addresses
#define WRITE_ADDR_INT_TIME     0x00
#define WRITE_ADDR_MEAS_TYPE    0x02
#define WRITE_ADDR_LED_MOD_CLK  0x1A
#define WRITE_ADDR_SET_TRIGGER  0x3F
#define WRITE_ADDR_READDATA     0x7E

// writes
#define WRITE_NOP                0x0000
#define WRITE_QUIT               0xC000
#define WRITE_TRIGGER            0xBF19
#define WRITE_QUERY_STATUS       0x6E00
#define WRITE_QUERY_SAT_PIXELS   0x6F00
#define WRITE_QUERY_READDATA     0x7E00
#define WRITE_QUERY_READLASTDATA 0x0000
#define WRITE_DOWNLOAD_EEPROM    0xB204

// uknown writes after trigger that are found in logic capture
const word WRITE_READ_UNKNOWN[28][2] = {
                                {0xB310,0x0000},
                                {0xB212,0xB310},
                                {0x7400,0xB212},
                                {0x0000,0x7401},
                                {0xB310,0x0000},
                                {0xB212,0xB310},
                                {0x7400,0xB212},
                                {0x0000,0x7400},
                                {0xB310,0x0000},
                                {0xB212,0xB310},
                                {0x7400,0xB212},
                                {0x0000,0x7400},
                                {0xB310,0x0000},
                                {0xB212,0xB310},
                                {0x7400,0xB212},
                                {0x0000,0x7400},
                                {0xB310,0x0000},
                                {0xB212,0xB310},
                                {0x7400,0xB212},
                                {0x0000,0x7400},
                                {0xB310,0x0000},
                                {0xB212,0xB310},
                                {0x7400,0xB212},
                                {0x0000,0x7430},
                                {0xB310,0x0000},
                                {0xB212,0xB310},
                                {0x7400,0xB212},
                                {0x0000,0x7430}};





// reads
//#define READ_ROW_READY 0x6E10               //////////////////changed from 6e10
#define READ_ROW_READY           0x0010 //to mask out 0x0010  no mask 6E10
#define READ_IDLE                0x0000
#define READ_BUSY                0xCCCC
#define READ_SPI_ERROR           0xFFFF

// read addresses
#define READ_ADDR_ROW_READY 0x2E
#define READ_ADDR_SAT_PIXEL 0x2F
#define READ_ADDR_DATA      0x3E
#define READ_ADDR_STATUS    0x6E

// write addresses
#define WRITE_ADDR_MOD_CLK_DIV 0x1A

// Measurement type
#define DISTANCE  0
#define AMBIENT   1

// distance calculation constants
#define C_LIGHT 300000000 // Speed of Light m/s
#define F_LED   0x9A09    // LED Modulation Frequency 10MHz setting
#define F_LED_HZ 10000000  // LED Modulation Frequency 10MHz
#define MOD_CLK_DIV 0x09  //set to MOD_CLK_DIV=1 so this is set to 0x09 and sent to address 1A
#define PI_PRECISE 3.1415926
#define D_UNAMBIGUITY 15 // for LED mod freq 10 MHz it is 15m

// integration times for LED modulation freq of 10 MHz (see section 3.5.1)
#define T_INT_52600 0x8080 //  52.60 ms
#define T_INT_26300 0x8081 //  26.30 ms
#define T_INT_13200 0x8082 //  13.20 ms
#define T_INT_06560 0x8083 //   6.56 ms
#define T_INT_03280 0x8084 //   3.28 ms
#define T_INT_01640 0x8085 //   1.64 ms
#define T_INT_00818 0x8086 // 818.00 us
#define T_INT_00408 0x8087 // 408.00 us
#define T_INT_00205 0x80C0 // 205.00 us
#define T_INT_00103 0x80C1 // 103.00 us  init=9
#define T_INT_00051 0x80C2 //  51.20 us
#define T_INT_00025 0x80C3 //  25.60 us
#define T_INT_00012 0x80C4 //  12.80 us
#define T_INT_00006 0x80C5 //  6.40  us
#define T_INT_00003 0x80C6 //  3.20  us
#define T_INT_00001 0x80C7 //  1.60  us

const word t_int[16] = {
	T_INT_00001,
	T_INT_00003,
	T_INT_00006,
  T_INT_00012,
  T_INT_00025,
  T_INT_00051,
  T_INT_00103,
  T_INT_00205,
  T_INT_00408,
  T_INT_00818,
  T_INT_01640,
  T_INT_03280,
  T_INT_06560,
  T_INT_13200,
  T_INT_26300,
  T_INT_52600
};

byte t_int_current = 0; // initialize to 103 us

#define UP         1
#define DOWN       0
#define MAX_T_INT 14
#define MIN_T_INT  4

const word t_int_delay_us[16] = {
	  1,
	  3,
	  6,
     12,
     25,
     51,
    103,
    205,
    408,
    818,
   1640,
   3280,
   6560,
  13200,
  26300,
  52600
};

// delays
#define WAIT_POWERUP_DELAY_MS         delay(200)
#define WAIT_T_INT                    delayMicroseconds(t_int_delay_us[t_int_current])
#define WAIT_FIRST_SAMPLE             delayMicroseconds(82)
#define WAIT_NEXT_SAMPLES             delayMicroseconds(50)
#define WAIT_NEXT_ROW                 delayMicroseconds(16)
#define WAIT_CONVERSION               delayMicroseconds(175)
#define WAIT_LOAD_FACTORY_SETTINGS_MS delayMicroseconds(105)
#define WAIT_AFTER_START_MEASUREMENT  delayMicroseconds(100)


const float D_OFFSET_avg = -2.00;
const float temp_m = 0.0058;
const float temp_b = -6.3420;

/*float D_OFFSET[8][8] = {
  {-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9},
  {-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9},
  {-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9},
  {-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9},
  {-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9},
  {-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9},
  {-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9},
  {-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9,-27.9}
};*/


byte saturated_pixels[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

bool pixel_validity[8][8] = {0};

float avgDCS0, avgDCS1, avgDCS2, avgDCS3, avgDCS4;

#define RUN_AVG_LEN 5
float distance_avg,last_dist,arctan,dist_ravg[RUN_AVG_LEN],data_weight[5] = {0.1,0.2,0.3,0.4,4};
char dist_pt;
bool infinity_flag;
/* temperature polynoms */
const int32_t epc610_temp_dist_off = 1430;
const uint8_t epc610_temp_fract_bits[3] = { 22, 22, 12 };
const int32_t epc610_temp_corr_fact[3] = { 11563, -20829295, 9117951 };

int DCS[5][8][8];
int temperature[5][8][2];
uint32_t TEMPRT=0;

//float distance[8][8];
float quality[8][8];
float avg_quality;
unsigned char quality_thres=50;

byte num_dcs_frames;
bool measure_type;
word rawData[5][8][15];
byte satPixels[8]; // 8 bits in a byte correspond to 8 pixels

/** buffers to send from and to receive to **/
#define DMASIZE 2
uint8_t src[DMASIZE];
volatile uint8_t dest[DMASIZE];
volatile uint8_t dest1[DMASIZE];

float temp_comp_curve[100] = {0};
/*
 * Stepper Motor defines
 */
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 2715

// All the wires needed for full functionality
#define DIR 1
#define STEP 22
//Uncomment line to use enable/disable functionality
//#define ENBL 0

// Since microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 32

// 2-wire basic config, microstepping is hardwired on the driver
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

// microstep control for DRV8834
#define M0 18
#define M1 19
DRV8834 stepper(MOTOR_STEPS, DIR, STEP, M0, M1);

//Uncomment line to use enable/disable functionality
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENBL);
#define HOME_OFFSET 50
#define MAX_POS 1900
#define HOME_POS 2400
#define MIN_POS 0
#define BACKLASH 35
#define MV_TH		8
#define MV_TH500    10
#define MV_TH900    15
#define MV_TH1100   25
#define INF_POS	250
// #define BACKLASH 0

int stepper_current_pos = 0;
int stepper_last_dir = 0;
int stepper_input_num = 0;
int stepper_move_steps = 0;
unsigned long infinity_time;

void stepper_move_to(int pos, bool force);
/*
 * convert_int_12_16 - converts a 16 bit word containing a 12 bit int
 *                     to a 16 bit int.
 *
 *                     First captures the sign (bit 11).
 *                     If bit 11 == 1 (negative) then changes bits 11-15 to 1.
 *                     Else if bit 11 == 0 then does nothing.
 */
int convert_int_12_16(unsigned int q12) {
  int q16;
  q16 = (q12 ^ 1 << 11) - (1 << 11);
  return q16;
}





/*
 * sendEPC - directly sends a word (2 bytes) to the EPC610.
             NOTE: It is recommended to use sendEPC_cid_addr_data() to conform with manual
 *
 * @param data: a 16 bit word containing the cid, data, and addr
 * @return rcv: a word containing the response
 */
word sendEPC(word data) {
  word rcv;
  // send result to device EPC610
  SPI.beginTransaction(settingsEPC);
  digitalWriteFast (EPC_SS_PIN, LOW);
  rcv = SPI.transfer16(data);
  digitalWriteFast (EPC_SS_PIN, HIGH);
  SPI.endTransaction();

  DEBUG_SPI_PRINT("SPI Sent: ");
  DEBUG_SPI_PRINT2(data, HEX);
  DEBUG_SPI_PRINT("  Received: ");
  DEBUG_SPI_PRINT2(rcv, HEX);
  DEBUG_SPI_PRINT("\n");

  return rcv;
}




/*
 * send_EPC_addr - sends an addr and data byte to the EPC610
 *
 * @param cid:  2 bit command id
 * @param addr: 6 bit address
 * @param data: 8 bit data
 *
 * @return rcv: a word containing the response
 */
word sendEPC_cid_addr_data(byte cid, byte addr, byte data) {
  word data_word, rcv;

  // put each byte into a 2-byte word
  data_word = ((cid << 14) | (addr << 8)) | data;

  // send result to device EPC610
  SPI.beginTransaction(settingsEPC);
  digitalWriteFast (EPC_SS_PIN, LOW);
  rcv = SPI.transfer16(data_word);
  digitalWriteFast (EPC_SS_PIN, HIGH);
  SPI.endTransaction();

  DEBUG_SPI_PRINT("SPI Sent: ");
  DEBUG_SPI_PRINT2(data_word, HEX);
  DEBUG_SPI_PRINT("  Received: ");
  DEBUG_SPI_PRINT2(rcv, HEX);
  DEBUG_SPI_PRINT("\n");

  return rcv;
}


/*
 * setLEDModFreq - Sets the LED Modulation frequency with the #define data listed above
 *
 * @return: STATUS: success or error code
 */
STATUS setLedModFreq() {
  word rcv;
  rcv = sendEPC(F_LED);
  if(rcv == READ_IDLE) {
    rcv = sendEPC(WRITE_NOP);
    if(rcv == F_LED) {
      return SUCCESS;
    }
  }

  return ERR_SET_LED_MOD_FREQ;
}

/*
 * powerupEPC - power up sequence sends NOPs continuously until an IDLE response
 *              the delay and number of times to send NOP is configurable via
 *              defines above
 *
 * @return STATUS: success or error code
 */
STATUS powerUpEPC() {
  word rcv;
  int i=0;
  while(i<POWERUP_MAX_TRIES) {
    rcv = sendEPC(WRITE_NOP);
    if(rcv == READ_IDLE)
      return SUCCESS;
    i++;
    WAIT_POWERUP_DELAY_MS;
  }
  return ERR_POWER_UP;
}

/*
 * loadFactorySettings - Required every time the device is powered up
 *
 * @return STATUS: success or error code
 */
STATUS loadFactorySettings() {
  word rcv;
  rcv = sendEPC(WRITE_DOWNLOAD_EEPROM);
  if(rcv != READ_IDLE) {
    return ERR_FACTORY_SETTINGS;
  }
  if(sendEPC(WRITE_NOP) == WRITE_DOWNLOAD_EEPROM) {
    while(sendEPC(WRITE_NOP) != READ_IDLE) {
      WAIT_LOAD_FACTORY_SETTINGS_MS;
    }
    return SUCCESS;
  }

  return ERR_FACTORY_SETTINGS;
}


/*
 * setIntegrationTime - sets the integration time for the EPC610
 *
 * @return STATUS: success or error code
 */
STATUS setIntegrationTime(word t_int) {
  word rcv=0;
  rcv = sendEPC(t_int);
  if(rcv == READ_IDLE) {
    rcv = sendEPC(WRITE_NOP);
    if(rcv == t_int) {
      sendEPC(WRITE_NOP);   ////////////////////////////// in logic capture of eval sw
      return SUCCESS;
    }
  }
  //while(1){
  DEBUG_PRINT("int time write err:rcv=");
  DEBUG_PRINT(rcv);
  DEBUG_PRINT("\n");
  //}
  return ERR_SET_INT_TIME;
}


STATUS setMeasurementType(int type) {
	word rcv=0,data_word;
	
	
  if (type == DISTANCE) {
  	data_word = ((0x10 << 14) | (0x02 << 8)) | 0x08;
    rcv = sendEPC(data_word);
    if(rcv == READ_IDLE) {
    	sendEPC(WRITE_NOP);
    	if(rcv == data_word) {
      sendEPC(WRITE_NOP);   
      return SUCCESS;
      }
      	
		}
		/*
		while(1){
		DEBUG_PRINT("meas type write err:rcv=");
		DEBUG_PRINT(rcv);
  	DEBUG_PRINT("\n");
  	}*/
		return ERR_UNKNOWN;
  }
  else if (type == AMBIENT) {
    //Serial.println("Setting up Ambient measurement.");
    //sendEPC_cid_addr_data(0x10, 0x02, 0x38);
    //sendEPC(WRITE_NOP);
    //return SUCCESS;
    data_word = ((0x10 << 14) | (0x02 << 8)) | 0x38;
    rcv = sendEPC(data_word);
    if(rcv == READ_IDLE) {
    	sendEPC(WRITE_NOP);
    	if(rcv == data_word) {
      sendEPC(WRITE_NOP);   
      return SUCCESS;
      }
      
		}
		/*
		while(1){
		DEBUG_PRINT("meas type write err:rcv=");
		DEBUG_PRINT(rcv);
  	DEBUG_PRINT("\n");
  	}*/
		return ERR_UNKNOWN;
  }
}



/*
 * startMeasurement - initiates measurement
 *                    NOTE: a NOP is required after the trigger
 *
 * @return STATUS: success or error message
 */
STATUS startMeasurement() {
  if(sendEPC(WRITE_TRIGGER) == READ_IDLE) {
    if(sendEPC(WRITE_NOP) == WRITE_TRIGGER) { //required
      //WAIT_AFTER_START_MEASUREMENT;
      return SUCCESS;
    }
  }
  return ERR_START_MEAS;
}



STATUS unknown() {
  STATUS err_code = SUCCESS;
  for(int i=0; i<28; i++) {
    if(sendEPC(WRITE_READ_UNKNOWN[i][0] != WRITE_READ_UNKNOWN[i][1])) {
      DEBUG_PRINT("ERROR Unkown");
      DEBUG_PRINT2(i, DEC);
      err_code = ERR_UNKNOWN;
    }
  }
  return err_code;
}


/*
 * distance -  calculates the distance to the object of a particular pixel
 *
 * @return distance
 */
 /*
void calcDistance() {
  float dist;
  for(int row=0; row<8; row++) {
    for(int col=0; col<8; col++) {
    	if(pixel_validity[row][col] != 0){
    		
      	dist = (C_LIGHT / (4 * PI_PRECISE * F_LED_HZ)) *
      	                     (PI_PRECISE +
      	                      atan2(DCS[0][row][col]-DCS[2][row][col], DCS[3][row][col]-DCS[1][row][col])) +
      	                     D_OFFSET_avg;//D_OFFSET[row][col];
      	
      	if (dist > D_UNAMBIGUITY) {
      	  distance[row][col] = dist - D_UNAMBIGUITY;
      	} 
      	else {
      	  distance[row][col] = dist;
      	}
      // Serial.print(distance[row][col]);
      // if (col != 7) Serial.print(", ");
      }
    }
    // Serial.println("");
  }
  return;
}*/
/*
 * @reset_all_data -  reset all data structure and start next round of measuring
 * @input:none
 * @return:none
 */
void reset_all_data(void)
{
	for(int row=0; row<8; row++) {
    for(int col=0; col<8; col++) {
			pixel_validity[row][col] = 0;
		}	
	}	
}	
/*
 * @average_distance -  average all the valid distance
 * @input:none
 * @return distance_avg
 */
 /*
float average_distance()
{
	float dist=0;
	byte count=0;
	
	for(int row=0; row<8; row++) {
    for(int col=0; col<8; col++) {
    	if(pixel_validity[row][col] != 0){
    		
      	dist += distance[row][col];
      	count ++;
      }
    }
    // Serial.println("");
  }
  
  if(!count)
  	count = 1;
  
  dist /= count;
  
  return dist;
}*/


/*
 * distance -  calculates the distance to the object of a particular pixel
 *
 * @return distance
 */
void calcDistance_avg() {
  float dist;

  dist = (C_LIGHT / (4 * PI_PRECISE * F_LED_HZ)) *
                       (PI_PRECISE +
                        atan2(avgDCS0-avgDCS2, avgDCS3-avgDCS1)) +
                       D_OFFSET_avg;


  // dist = (C_LIGHT / (4 * PI_PRECISE * F_LED_HZ)) *
  //                      (PI_PRECISE +
  //                       atan2(avgDCS2-avgDCS0, avgDCS3-avgDCS1)) + 0;
                      //  D_OFFSET_avg;

	 arctan = atan2(avgDCS0-avgDCS2, avgDCS3-avgDCS1);
   distance_avg = dist;

  return;
}


void calcQuality() {
  for(int row=0; row<8; row++) {
    for(int col=0; col<8; col++) {
      quality[row][col] = sqrt( ((long)sq(DCS[0][row][col]-DCS[2][row][col]) / 4 ) + ((long)sq(DCS[3][row][col]-DCS[1][row][col]) / 4));
    }
  }
}


void calcTemperature() {
  // float temp;
  for(int row=0; row<8; row++) {
    Serial.print("Temp: ");
    Serial.println(temperature[0][row][0]);
  }
}

int epc610_temp_corr(uint16_t dist, uint16_t temp, uint16_t *res_dist,
                     uint16_t *dist_off)
{
    uint8_t fract_diff=epc610_temp_fract_bits[0]-epc610_temp_fract_bits[2];
    
    int32_t t_dist_off = (int32_t)(((((int64_t)temp*temp*epc610_temp_corr_fact[0] +
                                      (int64_t)temp*epc610_temp_corr_fact[1])>>fract_diff) +
                                    (int64_t)epc610_temp_corr_fact[2])>>epc610_temp_fract_bits[2]);
    
    if(t_dist_off > dist) {
        *res_dist = 3000 - (t_dist_off - dist);
    } else {
        *res_dist = dist - t_dist_off;
    }
    
    *dist_off = t_dist_off;
    
    return 0;
}

/* comp_temp: temperature compensation, general formula: D_error=p1*dT+p2
 * input: int temp:temperature reading from IC, float dist:raw distance value
 * output: float:distance after temperature compensation
 */
/* version 1
float comp_temp(int temp,float dist) {
	float dist_temp,offset_drift,p2=0.68;
	
	if(dist>6)
		dist = 6;
	else if(dist<0.15)
		dist = 0.15;
	dist_temp = -0.006*temp+p2+dist;
			
  if(dist_temp < 0.37) //real distance < 0.36m, use p2=0.64
  	dist_temp -= 0.04;
  else if(dist_temp>1.2){ //real distance > 1.2m, use p1=0.005
  	
  	dist_temp = -0.005*temp+p2+dist;
  	offset_drift = (dist_temp-1.5)/0.5 * 0.05; //offset_drift increases as real distance increases, but offset_drift seems to stable around 0.15;
  	if(offset_drift>0.15)
  		offset_drift = 0.15;
  	dist_temp += offset_drift;
  }
  
  //only operate between 0.15m~6.5m
  if(dist_temp<0.15)	
  	dist_temp = 0.15;
  else if(dist_temp > 6.5)
  	dist_temp = 6.5;
  	
  return dist_temp;
}*/


//!version 2
float comp_temp(int temp,float dist) {
	float dist_temp,offset_drift,p2=6.61;//others:6.57, num 3: 6.61,	silver case: 6.43
	
	//if(dist>6)
	//	dist = 6;
	//else if(dist<0)
	//	dist = 0;
	dist_temp = -0.0055*temp+p2+dist;
			
  /*
  if(dist_temp>1.5){ //real distance > 1.2m, use p1=-0.0034
  	
  	dist_temp = -0.0055*temp+p2+dist;
  	offset_drift = (dist_temp-1)/1 * 0.08; //offset_drift increases as real distance increases
  	if(offset_drift>0.22)
  		offset_drift = 0.22;
  	dist_temp += offset_drift;
  }*/
  
  //else if(dist_temp > 6)
  	//dist_temp = 6;
  	
  return dist_temp;
}

// General model Exp2:
//      f(x) = a*exp(b*x) + c*exp(d*x)
// Coefficients (with 95% confidence bounds):
//        a =      -738.1  (-6.301e+09, 6.301e+09)
//        b =     -0.1045  (-609, 608.8)
//        c =       738.1  (-6.301e+09, 6.301e+09)
//        d =     -0.1044  (-608.7, 608.5)

float comp_dist(float dist) {

  // Distance Compensation Curve
  float a =      -738.1;
  float b =     -0.1045;
  float c =       738.1;
  float d =     -0.1044;


  float dist_comp;

  if (dist > 6){
    dist = 6;
  }
  else if (dist < 0.2){
    dist = 0.2;
  }


  dist_comp = (a*expf(b*dist)) + (c*expf(d*dist));

  return dist_comp;

}
/*
int focus_curve_lut(float dist){
//   General model Exp2:
//      f(x) = a*exp(b*x) + c*exp(d*x)
// Coefficients (with 95% confidence bounds):
//        a =        1013  (876.3, 1151)
//        b =      -1.792  (-2.15, -1.434)
//        c =       390.6  (339.4, 441.8)
//        d =    -0.05465  (-0.085, -0.0243)


//! new coefficiency
  float a =        3856;
  float b =      -4.289;
  float c =       597;
  float d =    -0.1296;

  float focus_pos;

  if (dist > 6){
    dist = 6;
  }
 	else if (dist < 0.25){
    dist = 0.2;
  }

  // distn = (dist - xmean) / xstd;

  focus_pos = (a*expf(b*dist)) + (c*expf(d*dist));
	
	if(focus_pos<220)
		focus_pos = 220;
  // Serial.println();
  // Serial.print(dist);
  // Serial.print(", ");
  // Serial.println(focus_pos);

  return int(focus_pos);
}*/
int focus_curve_lut(float dist)
{
	float a =        292.5;
  	float b =      -0.9941;
  	float c =       253.5;//429.8
  	float focus_pos,temp_dist;
  	
  	temp_dist = dist;
  	if(temp_dist<0.1)
  		temp_dist = 0.1;
  	focus_pos = a*pow(temp_dist,b) + c;
/*  	
  	if((dist>6) && (dist<7))
  		focus_pos = 340;
  	else if((dist>=7) && (dist<8))
  		focus_pos = 325;
  	else if(dist>=8)
  		focus_pos = 310;
*/  		
	if(focus_pos > MAX_POS)
		focus_pos = MAX_POS;
  	return int(focus_pos);
}

/*
 * rowReady - waits until a row is ready
 *
 * @param: count - the number of times to wait for row ready (twice is typical)
 * @return ready or not ready boolean
 */
bool rowReady() {
  word rcv;
  while(1) {
    rcv = sendEPC(WRITE_QUERY_STATUS);
    rcv = 0x0010 & rcv;                                   // MASK
    if(rcv == READ_ROW_READY) {
      return false;
    } else if (rcv == READ_SPI_ERROR) {
      return true; // need to recover
    } else {
      continue;
    }
  }
  return true;
}


/*
 * satPixels - reads saturated pixels
 *
 * @param row: 0 through 7
 */
byte getSatPixels(int row) {
  saturated_pixels[row] = 0x00;
  byte sat_pix_row;
  word rcv;
  sendEPC(WRITE_QUERY_SAT_PIXELS);
  rcv = sendEPC(WRITE_NOP);
  sendEPC(WRITE_NOP);
  sat_pix_row = 0x00FF & rcv;
  return sat_pix_row;
}


/*
 * change_t_int - changes integration time up or down
 *
 * @param dir - UP or DOWN
 */
void change_t_int(bool dir) {
  if(dir == UP){
    if(t_int_current<MAX_T_INT) t_int_current++;
    else{ 
    	DEBUG_PRINT("maximum t_int reached: ");
    	DEBUG_PRINT(t_int_current);
    	DEBUG_PRINT("\n");
    	stepper_move_to(INF_POS,1);
    }
  } else if(dir == DOWN) {
    if(t_int_current>MIN_T_INT) t_int_current--;
    else {
    	DEBUG_PRINT("minimum t_int reached: ");
    	DEBUG_PRINT(t_int_current);
    	DEBUG_PRINT("\n");
    }
  }
  return;
}


/*
 * recovery_sequence - runs if there is bad data in the DCS0-3 readouts
 */
void recovery_sequence() {
  sendEPC_cid_addr_data(0x2, 0x33, 0x59);
  sendEPC_cid_addr_data(0x2, 0x34, 0x00);
  sendEPC_cid_addr_data(0x2, 0x32, 0x01);
  sendEPC_cid_addr_data(0x0, 0x00, 0x00);
  for(int i=0; i<4; i++)
    WAIT_T_INT;
  sendEPC_cid_addr_data(0x2, 0x33, 0x59);
  sendEPC_cid_addr_data(0x2, 0x34, 0x08);
  sendEPC_cid_addr_data(0x2, 0x32, 0x01);
  sendEPC_cid_addr_data(0x0, 0x00, 0x00);
  return;
}


/*
 * adjust_t_int_based_on_saturated - checks for saturated pixels; adjusts t_int down if pixels are saturated
 */
void adjust_t_int_based_on_saturated() {
  bool saturated = false;
  for(int i=0; i<8; i++) {
    if(saturated_pixels[i] != 0x00) {
      saturated = true;
    }
  }
  if(saturated) {
    DEBUG_PRINT("Some pixels are saturated, decreasting t_int\n");
    change_t_int(DOWN);
  } else if(!saturated) {
    DEBUG_PRINT("increasing t_int\n");
    change_t_int(UP);
  }
}


// // typedef unsigned int VALIDITY_INDICATOR;
// // #define SATURATED             0
// // #define MAX_SIGNAL_LIMIT      1
// // #define LOW_SIGNAL_LIMIT      2
// VALIDITY_INDICATOR calcValidity() {
//
//   for(int i=0; i<8; i++) {
//     for(int j=0; j<8; j++) {
//       // Serial.print("quality[i][j] = ");
//       // Serial.println(quality[i][j]);
//     }
//     // Serial.print("saturated_pixels[");
//     // Serial.print(i);
//     // Serial.print("] = ");
//     // Serial.println(10, HEX);
//   }
//
//   return VALID;
// }


// typedef unsigned int QUALITY_INDICATOR;
// #define WEAK_ILLUMINATION         0
// #define SUFFICIENT_ILLUMINATION   1
// #define EXCELLENT_ILLUMINATION    2
// #define HIGH_ILLUMINATION         3
QUALITY_INDICATOR evalQuality() {
  int low = 0;
  int sufficient = 0;
  int excellent = 0;
  int high = 0;
  int majority = 48;
  int ret = SUFFICIENT_ILLUMINATION;
  
  avg_quality = 0;
  for(int i=0; i<8; i++) {
    for(int j=0; j<8; j++) {
      if (quality[i][j] < quality_thres) {
        low++;
        pixel_validity[i][j] = 0;
      }
      else if ((quality[i][j] > quality_thres) && (quality[i][j] <= 100)) {
        sufficient++;
        pixel_validity[i][j] = 1;
      }
      else if ((quality[i][j] > 100) && (quality[i][j] <= 1000)) {
        excellent++;
        pixel_validity[i][j] = 1;
      }
      else if (quality[i][j] > 1000) {
        high++;
        pixel_validity[i][j] = 0;
      }
      // Serial.print(pixel_validity[i][j]);
      avg_quality += quality[i][j];
    }
    // Serial.print("\n");
  }
  
  avg_quality /= 64;


  Serial.print("avg_quality:");
  Serial.print(avg_quality);
  Serial.print(" integration time:");
  Serial.print(t_int_current);
  Serial.print("\n");
  
  //
  //Serial.print(", sufficient = ");
  //Serial.print(sufficient);
  //
  //Serial.print(", excellent = ");
  //Serial.print(excellent);
  //
  //Serial.print(", high = ");
  //Serial.println(high);

  if (low >= majority){
    // Serial.println("Increase integration time.");
    ret = WEAK_ILLUMINATION;
  }
  else if (high >= majority){
    // Serial.println("Decrease integration time.");
    ret = HIGH_ILLUMINATION;
  }
  else if (sufficient >= majority){
    // Serial.println("Increase integration time.");
    // ret = WEAK_ILLUMINATION;
    ret = SUFFICIENT_ILLUMINATION;
  }
  else if ((sufficient + excellent) >= majority){
    // Serial.println("Do not change integration time.");
    ret = EXCELLENT_ILLUMINATION;
  }
  else{
    // Serial.println("Unclear which way to change integration time.");
    ret = WEAK_ILLUMINATION;
  }

  return ret;
}

void averagePixelArray(){

  float DCS0=0, DCS1=0, DCS2=0, DCS3=0;
  int count = 0;

  for(int i=0; i<8; i++) {
    for(int j=0; j<8; j++) {
      if (pixel_validity[i][j]){
        DCS0 += DCS[0][i][j];
        DCS1 += DCS[1][i][j];
        DCS2 += DCS[2][i][j];
        DCS3 += DCS[3][i][j];
        count++;
      }
    }
  }
  
  
  for(int f=0;f<4;f++){ 
  	for(int i=0; i<8; i++) {
  	  for(int j=0; j<2; j++) {
  	    
  	      TEMPRT += temperature[f][i][j];        
  	  }
  	}
  }
  
  if (count == 0) {
    count = 1;
  }
  
  

  avgDCS0 = DCS0 / count;
  avgDCS1 = DCS1 / count;
  avgDCS2 = DCS2 / count;
  avgDCS3 = DCS3 / count;

	TEMPRT = TEMPRT/(4*8*2);
	
	/*
	DEBUG_PRINT("Average DCS0-3: ");
	DEBUG_PRINT(avgDCS0);
	DEBUG_PRINT(", ");
	DEBUG_PRINT(avgDCS1);
	DEBUG_PRINT(", ");
	DEBUG_PRINT(avgDCS2);
	DEBUG_PRINT(", ");
	DEBUG_PRINT(avgDCS3);
	//DEBUG_PRINT(", Tempt:");
	//DEBUG_PRINT(TEMPRT);
	//
	DEBUG_PRINT("\n");*/
	
  // Serial.print("Average DCS0-3: ");
  // Serial.print(avgDCS0);
  // Serial.print(", ");
  // Serial.print(avgDCS1);
  // Serial.print(", ");
  // Serial.print(avgDCS2);
  // Serial.print(", ");
  // Serial.println(avgDCS3);


  return;
}



/*
 * adjust_t_int_with_keys - manual adjustment up and down for t_int
 */
void adjust_t_int_with_keys() {
  int incomingByte = 0;
  if(Serial.available() > 0) {
    incomingByte = Serial.read();

    // 'd' key pressed
    if(incomingByte == 100) {
      DEBUG_PRINT("\n*\n*\nDecreasing t_int = ");
      change_t_int(DOWN);
      DEBUG_PRINT(t_int_current);
      DEBUG_PRINT("\n*\n*\n");
    }

    // 'u' key pressed
    if(incomingByte == 117) {
      DEBUG_PRINT("\n*\n*\nIncreasing t_int = ");
      change_t_int(UP);
      DEBUG_PRINT(t_int_current);
      DEBUG_PRINT("\n*\n*\n");
    }
  }
}

/*void adjust_D_OFFSET_with_keys() {
  int incomingByte = 0;
  if(Serial.available() > 0) {
    incomingByte = Serial.read();

    // 'q' key pressed
    if(incomingByte == 113) {
      DEBUG_PRINT("\n*\n*\nDecreasing D_OFFSET = ");
      D_OFFSET[3][3] -= .1;
      DEBUG_PRINT(D_OFFSET[3][3]);
      DEBUG_PRINT("\n*\n*\n");
      delay(500);
    }

    // 'w' key pressed
    if(incomingByte == 119) {
      DEBUG_PRINT("\n*\n*\nIncreasing D_OFFSET = ");
      D_OFFSET[3][3] += .1;
      DEBUG_PRINT(D_OFFSET[3][3]);
      DEBUG_PRINT("\n*\n*\n");
      delay(500);
    }
  }
}
*/

boolean rawMeasurement(int type) {
    // read dcs0-dcs3
  word rcv;
  boolean recovery = false;
  noInterrupts();

  if (type == DISTANCE) {
    num_dcs_frames = 4;
  }
  else if (type == AMBIENT) {
    num_dcs_frames = 5;
  }

  // these three commands must be done before every measurement sequence
  setIntegrationTime(t_int[t_int_current]);
  setMeasurementType(type);
  STATUS st = startMeasurement();
  if(st == ERR_START_MEAS) {
    recovery = true;
    return recovery;
  }


  for(int dcs=0; dcs<num_dcs_frames; dcs++) {
    //recovery = rowReady();
    WAIT_T_INT;
    WAIT_CONVERSION;
    for(int row=0; row<8; row++) {
      //recovery = rowReady();

      WAIT_NEXT_ROW;
      satPixels[row] = getSatPixels(row);

      word throwOut = sendEPC(WRITE_QUERY_READDATA);
      if(throwOut == READ_SPI_ERROR)
        recovery = true;
      for(int read=0; read<14; read++) {
        // READ data 1 through 14
        rcv = sendEPC(WRITE_QUERY_READDATA);
        rawData[dcs][row][read] = rcv;
        if (dcs == 4){
          // Serial.println(rawData[dcs][row][read]);
        }
        if(rcv == READ_SPI_ERROR) {
          recovery = true;
        }
      }
      // Read last data 15
      rcv = sendEPC(WRITE_QUERY_READLASTDATA);
      rawData[dcs][row][14] = rcv;
      if(rcv == READ_SPI_ERROR) {
        recovery = true;
      }

      // NOP seen in logic capture of evalboard
      rcv = sendEPC(WRITE_QUERY_READLASTDATA);
      if(rcv == READ_SPI_ERROR) {
        recovery = true;
      }
    }
  }

  interrupts();
  return recovery;
}


/*
 * convertToPixelData - converts read data to pixel int array (12 bit)
 */
void convertToPixelData() {

  for(int d=0; d<num_dcs_frames; d++) {
    for(int row=0; row<8; row++) {
      DCS[d][row][0] = convert_int_12_16((unsigned int)( rawData[d][row][0]  & 0x00FF)       | ((rawData[d][row][1]  & 0x000F) << 8));
      DCS[d][row][1] = convert_int_12_16((unsigned int)((rawData[d][row][1]  & 0x00F0) >> 4) | ((rawData[d][row][2]  & 0x00FF) << 4));
      DCS[d][row][2] = convert_int_12_16((unsigned int)( rawData[d][row][3]  & 0x00FF)       | ((rawData[d][row][4]  & 0x000F) << 8));
      DCS[d][row][3] = convert_int_12_16((unsigned int)((rawData[d][row][4]  & 0x00F0) >> 4) | ((rawData[d][row][5]  & 0x00FF) << 4));
      DCS[d][row][4] = convert_int_12_16((unsigned int)( rawData[d][row][6]  & 0x00FF)       | ((rawData[d][row][7]  & 0x000F) << 8));
      DCS[d][row][5] = convert_int_12_16((unsigned int)((rawData[d][row][7]  & 0x00F0) >> 4) | ((rawData[d][row][8]  & 0x00FF) << 4));
      DCS[d][row][6] = convert_int_12_16((unsigned int)( rawData[d][row][9]  & 0x00FF)       | ((rawData[d][row][10] & 0x000F) << 8));
      DCS[d][row][7] = convert_int_12_16((unsigned int)((rawData[d][row][10] & 0x00F0) >> 4) | ((rawData[d][row][11] & 0x00FF) << 4));

      temperature[d][row][0] = convert_int_12_16((unsigned int)( rawData[d][row][12] & 0x00FF)       | ((rawData[d][row][13] & 0x000F) << 8));
      temperature[d][row][1] = convert_int_12_16((unsigned int)((rawData[d][row][13] & 0x00F0) >> 4) | ((rawData[d][row][14] & 0x00FF) << 4));
    }
  }


  return;
}


/*
 * setStepperViaLookupTable - sets the stepper motor position which controls the lens
 *
 * @param: distance - one distance value. a separate function should take all pixel values and
 *                    come up with a single value for this function
 * @return: stepper position - to be fed into the stepper motor function
 */
int getStepperPositionViaLookupTable(float distance) {
  if (distance < 0.19)
    return 1800;
  else if (distance < 0.3) {
    double calc = -7500.0*distance + 3200.0;
    return (int)calc;
  }
  else if (distance < 1.5) {
    double calc = 275.0/distance;
    return (int)calc;
  }
  else return 0;
}

/*
 * stepper_home - sends the motor to the home position
 */
void stepper_home() {
  // analogWrite(A12, (int)896);
  stepper.move(HOME_POS * MICROSTEPS);
  // analogWrite(A12, (int)1280);
  stepper.move(-HOME_OFFSET * MICROSTEPS);
  stepper_last_dir = 1;
  stepper_current_pos = 0;
  return;
}

/*
 * stepper_move_to - moves the motor to a specified position
 */
void stepper_move_to(int pos, bool force) {
	char mv_thres;
	
	if((pos>900)&&(pos<1100))
		mv_thres = MV_TH900;
	else if(pos>=1100)
		mv_thres = MV_TH1100;
	else if((pos>500)&&(pos<900))
		mv_thres = MV_TH500;
	else 
		mv_thres = MV_TH;
  if(pos <= MIN_POS)
  {
  	DEBUG_PRINT_STEPPER("Move input out of range.\n");
    DEBUG_PRINT_STEPPER(pos);
    DEBUG_PRINT("\n");
  	pos = MIN_POS;
  }
  else if(pos >= MAX_POS) 
  {
  	DEBUG_PRINT_STEPPER("Move input out of range.\n");
    DEBUG_PRINT_STEPPER(pos);
    DEBUG_PRINT("\n");
  	pos = MAX_POS;
  }
    // DEBUG_PRINT_STEPPER("Moving from position ");
    // DEBUG_PRINT_STEPPER(stepper_current_pos);
    // DEBUG_PRINT_STEPPER(" to position: ");
    // DEBUG_PRINT_STEPPER(pos);
    // DEBUG_PRINT_STEPPER("\n");
	DEBUG_PRINT_STEPPER("target position: ");
    DEBUG_PRINT_STEPPER(pos);
    DEBUG_PRINT("\n");
    stepper_move_steps = stepper_current_pos - pos;
	
	//! change rpm for long distance moving
	/*
	if(stepper_move_steps > 200)
	{
		stepper.setRPM(60);
	}
	else
	{
		stepper.setRPM(35);
	}	*/
    // Serial.print("stepper_move_steps = ");
    // Serial.println(stepper_move_steps);
    if (stepper_move_steps > mv_thres) {
      if (stepper_last_dir == 0) {
        stepper_move_steps += BACKLASH;
        stepper_last_dir = 1;
        // Serial.print("new stepper_move_steps = ");
        // Serial.println(stepper_move_steps);
      }
    }
    else if (stepper_move_steps < -mv_thres) {
      if (stepper_last_dir == 1) {
        stepper_move_steps -= BACKLASH;
        stepper_last_dir = 0;
        // Serial.print("new stepper_move_steps = ");
        // Serial.println(stepper_move_steps);
      }
    }
    else {
      if(force)			//!if force set, move to target position, ignore threshold
      {
      	if ((stepper_last_dir == 0)&&(stepper_move_steps>0)) 
      	{
        	stepper_move_steps += BACKLASH;
        	stepper_last_dir = 1;
      	}
      	else if((stepper_last_dir == 1)&&(stepper_move_steps<0))
      	{
      		stepper_move_steps -= BACKLASH;
        	stepper_last_dir = 0;
      	}
      }
      else
      	return;
    }
    // if ((stepper_move_steps > 0) & stepper_last_dir){
    //   stepper_move_steps += BACKLASH;
    // }
    //
    // if ((stepper_move_steps < 0) & !stepper_last_dir){
    //   stepper_move_steps -= BACKLASH;
    // }

    stepper.move(stepper_move_steps * MICROSTEPS);

    // if(stepper_move_steps < 0){
    //   stepper_last_dir = 1;
    // }
    // else {
    //   stepper_last_dir = 0;
    // }

    stepper_current_pos = pos;

  return;
}

void usr_input_serial() {
  static char usr_data[20]={0};
  char *tok;
  unsigned char value;
  static unsigned char data_pt=0;
  
  if(Serial.available() > 0) {
    usr_data[data_pt] = Serial.read();
    Serial.print(usr_data[data_pt++]); 
    data_pt %= 20;   
	/*
    // 'z' key pressed
    if(incomingByte == 122) {
      stepper_desired_position = stepper_current_pos - 100;
    }
    // 'x' key pressed
    if(incomingByte == 120) {
      stepper_desired_position = stepper_current_pos + 100;
    }
    // 'c' key pressed
    if(incomingByte == 99) {
      stepper_desired_position = stepper_current_pos - 10;
    }
    // 'v' key pressed
    if(incomingByte == 118) {
      stepper_desired_position = stepper_current_pos + 10;
    }

    // 'b' key pressed
    if(incomingByte == 98) {
      stepper_desired_position = stepper_current_pos - 1;
    }
    // 'n' key pressed
    if(incomingByte == 110) {
      stepper_desired_position = stepper_current_pos + 1;
    }

    // 'p'
    if(incomingByte == 112) {
      DEBUG_PRINT("\n\n***CURRENT MOTOR POS = ");
      DEBUG_PRINT(stepper_current_pos);
      DEBUG_PRINT("\n");
    }
    //'h'
    if(incomingByte == 104) {
      DEBUG_PRINT("\n\n***HOME************************* ");
      stepper_home();
      DEBUG_PRINT(stepper_current_pos);
      DEBUG_PRINT("\n");
      return;
    }
	
    stepper_move_to(stepper_desired_position,1);*/
    //delay(2000);
  }
  Serial.print("\n"); 
  if(data_pt && ((usr_data[data_pt-1]=='\r') || (usr_data[data_pt-1]=='\n')))
  {
  	Serial.print("[USER_INPUT]Data input end, data len=");  	
  	Serial.print(data_pt);
  	Serial.print("\n");
  	usr_data[data_pt-1] = '\0';
  	tok = strtok(usr_data,"=");
  	Serial.print(tok);
  	Serial.print("\n");
  	if(!strcmp(tok,"TH"))
  	{
  		tok = strtok(NULL,";");
  		Serial.print(tok);
  		Serial.print("\n");
  		value = atoi(tok);
  		quality_thres = value;
  		Serial.print("[USER_INPUT]Set quality threshold to:");    
  		Serial.print(quality_thres);    
  		Serial.print("\n");    
  	}
  	data_pt = 0;
  }
}


/*
 * Setup - Initializes Serial (debugging) and SPI (EPC610 TOF Cam)
 */
void setup() {
  String status_msg;
  STATUS StatusPowerUp,StatusFactory,StatusIntTime,StatusLedModFreq;
	uint8_t fail_count;
  // initialize serial
  #ifdef DEBUG
  Serial.begin(SERIAL_BAUD);
  #endif
  DEBUG_PRINT("Initialized Serial...\n\n");

  // button pin init
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  
  // set the Slave Select Pins as outputs
  pinMode (EPC_SS_PIN, OUTPUT);
  pinMode (EPC_LED_LOW_ENABLE, OUTPUT);
  pinMode (EPC_LED_HIGH_ENABLE, OUTPUT);

  digitalWrite (EPC_SS_PIN, HIGH);
  digitalWrite (EPC_LED_LOW_ENABLE, LOW);   //HIGH = enable
  digitalWrite (EPC_LED_HIGH_ENABLE,HIGH);

  // delay(5000); // wait 1 sec for EPC610 to be fully powered up


  // initialize SPI
  DEBUG_PRINT("Initializing SPI...\n\n");
  SPI.begin();

  // set up stepper and go home
  
  analogWriteResolution(12);
  analogReference(EXTERNAL);
  analogWrite(A12, (int)2100);
  stepper.setMicrostep(MICROSTEPS);
  stepper.setRPM(35);
  stepper_home();


/*float dist = (C_LIGHT / (4 * PI_PRECISE * F_LED_HZ)) *
                           (PI_PRECISE +
                            atan2(-348+51, -279+108)) +
                           0;

  DEBUG_PRINT("Distance: ");
  DEBUG_PRINT(dist);
  DEBUG_PRINT("\n");
  while(1);
*/
  // power up...wait until ready for communication
  while(1) {
    DEBUG_PRINT("Running powering up sequence...\n");
    StatusPowerUp = powerUpEPC();
    status_msg = String("Status:" + String(StatusPowerUp, DEC) + "\n");
    DEBUG_PRINT(status_msg);
    if(StatusPowerUp == SUCCESS) {
      DEBUG_PRINT("...success!\n\n");
      fail_count = 0;
      break;
    }
    DEBUG_PRINT("...failed, retrying.\n");
    fail_count++;
    if(fail_count>=5)
    {
    	while(1);
    }
  }


  while(1) {
      DEBUG_PRINT("Setting factory settings...\n");
      StatusFactory = loadFactorySettings();
      status_msg = String("Status:" + String(StatusFactory, DEC) + "\n");
      DEBUG_PRINT(status_msg);
      if(StatusFactory == SUCCESS) {
        DEBUG_PRINT("...success!\n\n");
        fail_count = 0;
        break;
      }
      DEBUG_PRINT("...failed, retrying.\n");
      fail_count++;
    	if(fail_count>=5)
    	{
    		while(1);
    	}
  }


  // set LED Modulation Frequency
  while(1) {
    DEBUG_PRINT("Setting LED modulation frequency 10MHz...\n");
    StatusLedModFreq = setLedModFreq();
    status_msg = String("Status:" + String(StatusLedModFreq, DEC) + "\n");
    DEBUG_PRINT(status_msg);
    if(StatusLedModFreq == SUCCESS) {
      DEBUG_PRINT("success!\n\n");    
      fail_count = 0; 
      break;
    }
    DEBUG_PRINT("...failed, retrying.\n");
    fail_count++;
    if(fail_count>=5)
    {
    	while(1);
    }
  }

  // set Integration Time
  while(1) {
    DEBUG_PRINT("Setting integration time 103us...\n");
    StatusIntTime = setIntegrationTime(T_INT_00103);
    status_msg = String("Status:" + String(StatusIntTime, DEC) + "\n");
    DEBUG_PRINT(status_msg);
    if(StatusIntTime == SUCCESS) {
      DEBUG_PRINT("...success!\n\n");
      fail_count = 0; 
      break;
    }
    DEBUG_PRINT("...failed, retrying.\n");    
    fail_count++;        
		if(fail_count>=5)    
		{                    
			while(1);          
		}                    

  }


  if(StatusPowerUp == 0   && StatusFactory == 0 && StatusIntTime == 0 &&
    StatusLedModFreq == 0)
    DEBUG_PRINT("setup success\n");

}

/*
void print_distance() {
  DEBUG_PRINT("Distance:\n");
  for(int j=0; j<8; j++) {
    DEBUG_PRINT("{");
    for(int k=0; k<8; k++) {
      DEBUG_PRINT2(distance[j][k], DEC);
      if(k!=7)DEBUG_PRINT(", ");
    }
    DEBUG_PRINT("}\n");
  }
  DEBUG_PRINT("\n\n");
}

void print_distance_row_col(int row, int col) {
  DEBUG_PRINT("Distance[");
  DEBUG_PRINT(row);
  DEBUG_PRINT("][");
  DEBUG_PRINT(col);
  DEBUG_PRINT("]: ");
  DEBUG_PRINT2(distance[row][col], 2);
  DEBUG_PRINT("\n\n");
}*/

void print_DCS() {
  for(int d=0; d<num_dcs_frames; d++) {
    DEBUG_PRINT("DCS ");
    DEBUG_PRINT(d);
    DEBUG_PRINT(":\n{");
    for(int row=0; row<8; row++) {
      DEBUG_PRINT("[");
      for(int col=0; col<8; col++) {
        DEBUG_PRINT2(DCS[d][row][col], 10);
        //DEBUG_PRINT2(rawData[d][row][col], 10);
        if(col != 7)
          DEBUG_PRINT(", ");
      }
      DEBUG_PRINT("]\n");
    }
    DEBUG_PRINT("}\n\n");
  }
  return;
}

void print_quality()
{
	DEBUG_PRINT("QUALITY: \n");
	for(int row=0; row<8; row++) {
      DEBUG_PRINT("[");
      for(int col=0; col<8; col++) {
        DEBUG_PRINT2(quality[row][col], 10);
        //DEBUG_PRINT2(rawData[d][row][col], 10);
        if(col != 7)
          DEBUG_PRINT(", ");
      }
      DEBUG_PRINT("]\n");
    }
}

void print_temperature() {
  for(int d=0; d<num_dcs_frames; d++) {
    DEBUG_PRINT("TEMP ");
    DEBUG_PRINT(d);
    DEBUG_PRINT(":\n{");
    for(int row=0; row<8; row++) {
      DEBUG_PRINT("[");
      for(int col=0; col<2; col++) {
        DEBUG_PRINT2(temperature[d][row][col], 10);
        //DEBUG_PRINT2(rawData[d][row][col], 10);
        if(col != 1)
          DEBUG_PRINT(", ");
      }
      DEBUG_PRINT("]\n");
    }
    DEBUG_PRINT("}\n\n");
  }
  return;
}

void print_DCS_row_col(int row, int col) {
  for(int d=0; d<4; d++) {
    DEBUG_PRINT("DCS ");
    DEBUG_PRINT(d);
    DEBUG_PRINT(" [");
    DEBUG_PRINT(row);
    DEBUG_PRINT("][");
    DEBUG_PRINT(col);
    DEBUG_PRINT("]: ");
    DEBUG_PRINT2(DCS[d][row][col], DEC);
    DEBUG_PRINT("\n");
  }
  DEBUG_PRINT("\n");
}


void record_temp_curve(int temp,float dist)
{
	int temp_bak;
	
	temp_bak = temp-1100;		//working temperature should be above 1100
	if(temp_bak<0)
		temp_bak = 0;
	temp_comp_curve[temp_bak] = dist;
	DEBUG_PRINT("TEMP_DIST_CURVE:\n");
	for(int i=0; i<10; i++) {
    DEBUG_PRINT("[");
    for(int j=0; j<10; j++) {
      		DEBUG_PRINT(temp_comp_curve[i*10 + j]);
          DEBUG_PRINT(", ");
      }
    DEBUG_PRINT("]\n");  
    }      
}

int sweep_int_analysis(void)
{
	bool recovery = false;
	int best_int=MAX_T_INT;
	float best_result_quality=0;
	
	for(int i=MIN_T_INT; i<= MAX_T_INT; i++){
		
		t_int_current = i;
		measure_type = DISTANCE;
  	recovery = rawMeasurement(measure_type);
  	
  	if(!recovery) {
  	  // we have good data, so process it
  	  convertToPixelData();
  	
  	  //print_DCS();
			//print_temperature();
			
  	  // calcDistance(); // full array distance calculation
  	  
  	  //this is where pixel validity starts to make difference in calculation
  	  calcQuality();
  	  // calcTemperature();
  	
  	  evalQuality();
  	  
  	  if(avg_quality>best_result_quality){
  	  	best_result_quality = avg_quality;
  	  	best_int = t_int_current;
  	  }	
  	  Serial.print("avg_quality:");
  	  Serial.print(avg_quality);
  	  Serial.print(";");
  	  Serial.print("int:");
  	  Serial.print(t_int_current);
  	  Serial.print("\n");
  	  delay(50);
  	}	
  	if(best_result_quality>quality_thres)	//good enough, no need to continue.
  		break;
  }
  
  if(best_result_quality<quality_thres)   //if quality is below 75, high chance it's point to far away, change the focus to infinity
  {	
  	Distance_running_avg(1000);  //! make infinity 1000
  	infinity_time = millis();
  	stepper_move_to(INF_POS,1);
  	infinity_flag = 1;
  	Serial.print("[INTEG_SWEEP]:move to infinity!\n");
  }	
  Serial.print("best_result_quality:");
  Serial.print(best_result_quality);
  Serial.print(";");
  Serial.print("best_int:");
  Serial.print(best_int);
  Serial.print("\n");
  return best_int;
}	
/*
 * Distance_running_avg - doing a running avg for dist, make the measurement smoother
 * @input: 
 * 		float dist: current measured distance 
 * @return:
 *		float: result of running avg
 */
float Distance_running_avg(float dist)
{
	char count=0;
	float dist_sum=0;
	
	dist_ravg[dist_pt] = dist;  //! save distance result in a global array
	dist_pt++;
	dist_pt %= RUN_AVG_LEN;
	//Serial.print("dist0~4:");
	for(char i=0; i<RUN_AVG_LEN; i++)     //! calculate running avg
	{
		count++;
		dist_sum += dist_ravg[(dist_pt + i)%5]*data_weight[i];	
  		//Serial.print(dist_ravg[i]);  
	}
	//Serial.print(" avg_dist:");
	//Serial.print(dist_sum/count);
	//Serial.print("\n");
	return dist_sum/count;
}	
/*
 * state machine for button detection
 * input: button_proc, 0:not processed yet, 1:already processed
 * output: 1:pressed, 0:unpressed
 */
unsigned char check_button(bool *button_proc)
{
	static unsigned char state_hold,button_state,special_key;
	static unsigned long button_delay,button_time,dclick_delay,lpress_delay; 
	unsigned char key_func = 0;
	
	button_time = millis();
	if(button_time < (button_delay + 10))
		return 0;
	button_delay = button_time;
	
	switch(button_state)
	{
		case 0:		//!idle state
  				if(!digitalRead(BUTTON1_PIN))
  				{
  					state_hold++;
  					
  				}
  				else
  				{
  					state_hold = 0;
  				}
  			
  			if(state_hold>=2)
  			{
  				button_state = 1;		//! enter pressed state
  				state_hold = 0;
  				lpress_delay = button_time;
  			}
			break;
			
		case 1:		//! pressed state
				Serial.print("key pressed!\n");
  				if(!digitalRead(BUTTON1_PIN))
  				{
  					state_hold = 0;
  				}
  				else
  				{
  					state_hold++;
  				}
  			
  			if((button_time > (lpress_delay+500)) && !special_key)
  			{
  				special_key = 2;
  				key_func = 3;  //! long press
  				Serial.print("RESULT:Long Press Detected!\n");	
  			}
  				
  			if(state_hold>=2)
  			{
  				state_hold = 0;
  				button_state = 2;		//! enter released state
  				
  				if(!special_key)				//! if double click already detected, no need to track time
  					dclick_delay = button_time;
  			}
			break;
		case 2:		//! release state for double click detect
			Serial.print("delay for double click detect!\n");				
			if(special_key==2)   //! if long press detected, release part should not trigger function again
			{
				special_key = 0;
				button_state = 3;
			}	
			else if(button_time > (dclick_delay+400))	//! after 400ms without clicking, no double click
			{
				button_state = 3;
				key_func = 1;				//! single press
				Serial.print("RESULT:Single Click Detected!\n");	
			}
			else	//! for double click
			{
				if(special_key==1)	//! if it's double click
				{
					special_key = 0;
  					button_state = 3;
					key_func = 2;				//! double click
					Serial.print("RESULT:Double Click Detected!\n");	
				}	
				else
				{
					if(!digitalRead(BUTTON1_PIN))
  					{
  						state_hold++;
  						
  					}
  					else
  					{
  						state_hold = 0;
  					}
  			    	
  					if(state_hold>=2)		//detect button press within double click delay
  					{
  						Serial.print("RESULT:second press Detected!\n");
  						state_hold = 0;
  						button_state = 1;
  						special_key = 1;  //! double click
					}
				}				
			}
			break;
		case 3:		//! wait for key func processing
			Serial.print("wait for key process!\n");
			if(button_proc)
			{
				button_state = 0;		//! enter idle again
				button_proc = 0;
				Serial.print("key idle!\n");
			}
			break;
		default:break;
	}
	
  	
  	return key_func;
}

/* process button
 * input: lock_motor_pos; key_val: 1:single press, 2:double press;
 * output: lock_motor_pos, 1:stay, 0:leave; last_dist, 1000:infinity; 
 * return: 1:processed
 */
bool button_func(bool *lock_motor_pos,unsigned char key_val)
{
	if(!(*lock_motor_pos))
	{
		if(key_val == 1)	//! single press, lock at current position
		{
			*lock_motor_pos = 1;
		}
		else if(key_val == 2)  //! double press, lock at infinity
		{
			*lock_motor_pos = 1;
			last_dist = 1000;
			stepper_move_to(INF_POS,1);
		}
		else if(key_val == 3)  //! long press, lock at infinity
		{
			*lock_motor_pos = 1;
			last_dist = 1000;
			stepper_move_to(INF_POS,1);
		}
	}
	else
		*lock_motor_pos = 0;
	return 1;
}
/*
 * loop - Arduino req'd function. Main functionality after setup
 */
void loop() {
  bool recovery = false;
  // char buffer[50];
  int meas_quality;
  float temp_comp, dist_comp;
  static int pos_lookup=0;
  static unsigned long cur_time,measure_time;
  static uint8_t count=0,weak_sig_count=0;
  static bool button_proc = 0,lock_motor_pos = 0;
  unsigned char key_value=0;
  
  //adjust_t_int_with_keys();
  //check button state
  if(key_value = check_button(&button_proc))
  {
  	button_proc = button_func(&lock_motor_pos,key_value);
  }
  if(lock_motor_pos)
  	return;			//lock position until next button press
  	
  cur_time = millis();	
  if((cur_time - measure_time) < 100)	//! limit sampling rate
  	return;
    
  reset_all_data();		//clear data arrays before every measurement
  // Take the measurement
	 //measure_type = AMBIENT;
	 //t_int_current = 10;
   measure_type = DISTANCE;
   recovery = rawMeasurement(measure_type);
   measure_time = millis();
   
  if(!recovery) {
    // we have good data, so process it
    usr_input_serial();
    convertToPixelData();

    //print_DCS();
		//print_temperature();
		
    // calcDistance(); // full array distance calculation
    
    //this is where pixel validity starts to make difference in calculation
    calcQuality();
    // calcTemperature();

    meas_quality = evalQuality();
    	
		switch(meas_quality)
		{
			
    	case WEAK_ILLUMINATION:
    		//! if getting 3 consecutive weak signal, do sweep, otherwise consider it as random interference
      		if(weak_sig_count<4)
      		{
      			weak_sig_count++;
      		}
      		else
      		{
      			if((measure_time - infinity_time)> 400)
      			{
      				t_int_current = sweep_int_analysis();
      				weak_sig_count = 0;
      				Serial.print("WEAK ILLUMI\n");
      			}
      		}      					
      		break;
    	    
    case HIGH_ILLUMINATION:
    
      // Serial.println("Decreasing integration time");
      change_t_int(DOWN);
      Serial.print("HIGH ILLUMI\n");

      break;
    
    case SUFFICIENT_ILLUMINATION:
    	//t_int_current = MIN_T_INT-1;
    	//change_t_int(UP);
    	//t_int_current = sweep_int_analysis();
      Serial.print("SUFFICIENT ILLUMI\n");
      //delay(250);
    default:
      
      infinity_flag = 0;
      weak_sig_count = 0;
      
      averagePixelArray();
      calcDistance_avg();
	  
	  distance_avg = Distance_running_avg(distance_avg);  //! doing a running avg on distance, make it smoother
	  
      count++;
	  if(count>=3)      //! sample 3 times before moving the motor
	  {
	  	count = 0;
	  	if((abs(last_dist - distance_avg) >= distance_avg/50)&&(distance_avg<15))//&&((measure_time - infinity_time)>200))
	  	{
	  		Serial.print("needs to move motor!\n");
	  		last_dist = distance_avg;
	  		dist_comp = comp_temp(TEMPRT,distance_avg); 
      		pos_lookup = focus_curve_lut(dist_comp);
      		stepper_move_to(pos_lookup,0);
      		print_quality();
      		Serial.print("t_int: ");
      		Serial.print(t_int_current);
      		Serial.print(", dist:");
      		
      		Serial.print(dist_comp);
      		
      		Serial.print(", temp:");
      		
      		Serial.print(TEMPRT);
      
			Serial.print(", position:");
      		Serial.print(pos_lookup);
      
      		Serial.print(", non_comp:");
      		Serial.print(distance_avg);
      		Serial.print(", last_dist:");
      		Serial.print(last_dist);
			Serial.print("\n");       		
      	}	       
	  }     
      // Uncomment to move with keys...
      
      
      for(int i=0; i<8; i++) {
        if(satPixels[i] != 0x00)
          DEBUG_PRINT("Some pixels are saturated\n");
      }
			break;
    }
  }  
  else {
    DEBUG_PRINT("Bad data.  Recovering...\n");
    recovery_sequence();
    DEBUG_PRINT("Recovered.\n\n");
  }

}
