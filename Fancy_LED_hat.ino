/*
 * Fancy LED hat
 * by: Shawn Parr (parr@shawnparr.com)
 * May 6th 2018
 * 
 * License: The Unlicense (https://choosealicense.com/licenses/unlicense/) for any code except the BNO055 functions and definitions
 * BNO055 functions and definitions:
 *     by: Kris Winer
 *     date: April 25, 2015
 *     license: Beerware - Use this code however you'd like. If you 
 *     find it useful you can buy me a beer some time.
 *     (https://github.com/kriswiner/BNO055)
 * 
 * 
 * This silly code is to mount addressable LEDs as a single strip into a hat, and have the color, speed, and direction of 
 * "particles" of light controlled by the positioning.
 * 
 * Includes impact detection and spin detection for some extra flair.
 */

#include <FastLED.h>
#include <i2c_t3.h>
#include <SPI.h>
#define NUM_LEDS 89
#define DATA_PIN 17
#define MAX_LED_DELAY 35
#define MIN_LED_DELAY 10
#define ROLLOFFSET 20
#define MAXROLL 35
#define MINROLL -35
#define MAXPITCH 40 
#define MINPITCH -5 
#define PITCHCENTER 15
#define SERIAL_DEBUG true

/*
 * BNO055 Setup
 */
#define BNO055_ADDRESS 0x29

// BNO055 Page 0
#define BNO055_CHIP_ID          0x00    // should be 0xA0              
#define BNO055_ACC_ID           0x01    // should be 0xFB              
#define BNO055_MAG_ID           0x02    // should be 0x32              
#define BNO055_GYRO_ID          0x03    // should be 0x0F              
#define BNO055_SW_REV_ID_LSB    0x04                                                                          
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_ACC_DATA_X_MSB   0x09
#define BNO055_ACC_DATA_Y_LSB   0x0A
#define BNO055_ACC_DATA_Y_MSB   0x0B
#define BNO055_ACC_DATA_Z_LSB   0x0C
#define BNO055_ACC_DATA_Z_MSB   0x0D
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_MAG_DATA_X_MSB   0x0F
#define BNO055_MAG_DATA_Y_LSB   0x10
#define BNO055_MAG_DATA_Y_MSB   0x11
#define BNO055_MAG_DATA_Z_LSB   0x12
#define BNO055_MAG_DATA_Z_MSB   0x13
#define BNO055_GYR_DATA_X_LSB   0x14
#define BNO055_GYR_DATA_X_MSB   0x15
#define BNO055_GYR_DATA_Y_LSB   0x16
#define BNO055_GYR_DATA_Y_MSB   0x17
#define BNO055_GYR_DATA_Z_LSB   0x18
#define BNO055_GYR_DATA_Z_MSB   0x19
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F
#define BNO055_QUA_DATA_W_LSB   0x20
#define BNO055_QUA_DATA_W_MSB   0x21
#define BNO055_QUA_DATA_X_LSB   0x22
#define BNO055_QUA_DATA_X_MSB   0x23
#define BNO055_QUA_DATA_Y_LSB   0x24
#define BNO055_QUA_DATA_Y_MSB   0x25
#define BNO055_QUA_DATA_Z_LSB   0x26
#define BNO055_QUA_DATA_Z_MSB   0x27
#define BNO055_LIA_DATA_X_LSB   0x28
#define BNO055_LIA_DATA_X_MSB   0x29
#define BNO055_LIA_DATA_Y_LSB   0x2A
#define BNO055_LIA_DATA_Y_MSB   0x2B
#define BNO055_LIA_DATA_Z_LSB   0x2C
#define BNO055_LIA_DATA_Z_MSB   0x2D
#define BNO055_GRV_DATA_X_LSB   0x2E
#define BNO055_GRV_DATA_X_MSB   0x2F
#define BNO055_GRV_DATA_Y_LSB   0x30
#define BNO055_GRV_DATA_Y_MSB   0x31
#define BNO055_GRV_DATA_Z_LSB   0x32
#define BNO055_GRV_DATA_Z_MSB   0x33
#define BNO055_TEMP             0x34
#define BNO055_CALIB_STAT       0x35
#define BNO055_ST_RESULT        0x36
#define BNO055_INT_STATUS       0x37
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_STATUS       0x39
#define BNO055_SYS_ERR          0x3A
#define BNO055_UNIT_SEL         0x3B
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_TEMP_SOURCE      0x40
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB   0x67
#define BNO055_ACC_RADIUS_MSB   0x68
#define BNO055_MAG_RADIUS_LSB   0x69
#define BNO055_MAG_RADIUS_MSB   0x6A
//
// BNO055 Page 1
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_CONFIG       0x08
#define BNO055_MAG_CONFIG       0x09
#define BNO055_GYRO_CONFIG_0    0x0A
#define BNO055_GYRO_CONFIG_1    0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK          0x0F
#define BNO055_INT_EN           0x10
#define BNO055_ACC_AM_THRES     0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION  0x13
#define BNO055_ACC_HG_THRESH    0x14
#define BNO055_ACC_NM_THRESH    0x15
#define BNO055_ACC_NM_SET       0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET     0x18
#define BNO055_GYR_DUR_X        0x19
#define BNO055_GYR_HR_Y_SET     0x1A
#define BNO055_GYR_DUR_Y        0x1B
#define BNO055_GYR_HR_Z_SET     0x1C
#define BNO055_GYR_DUR_Z        0x1D
#define BNO055_GYR_AM_THRESH    0x1E
#define BNO055_GYR_AM_SET       0x1F

// Set initial input parameters
enum Ascale {  // ACC Full Scale
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_18G
};

enum Abw { // ACC Bandwidth
  ABW_7_81Hz = 0,
  ABW_15_63Hz,
  ABW_31_25Hz,
  ABW_62_5Hz,
  ABW_125Hz,    
  ABW_250Hz,
  ABW_500Hz,     
  ABW_1000Hz,    //0x07
};

enum APwrMode { // ACC Pwr Mode
  NormalA = 0,  
  SuspendA,
  LowPower1A,
  StandbyA,        
  LowPower2A,
  DeepSuspendA
};

enum Gscale {  // gyro full scale
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS    // 0x04
};

enum GPwrMode { // GYR Pwr Mode
  NormalG = 0,
  FastPowerUpG,
  DeepSuspendedG,
  SuspendG,
  AdvancedPowerSaveG
};

enum Gbw { // gyro bandwidth
  GBW_523Hz = 0,
  GBW_230Hz,
  GBW_116Hz,
  GBW_47Hz,
  GBW_23Hz,
  GBW_12Hz,
  GBW_64Hz,
  GBW_32Hz
};

enum OPRMode {  // BNO-55 operation modes
  CONFIGMODE = 0x00,
// Sensor Mode
  ACCONLY,
  MAGONLY,
  GYROONLY,
  ACCMAG,
  ACCGYRO,
  MAGGYRO,
  AMG,            // 0x07
// Fusion Mode
  IMU,
  COMPASS,
  M4G,
  NDOF_FMC_OFF,
  NDOF            // 0x0C
};

enum PWRMode {
  Normalpwr = 0,   
  Lowpower,       
  Suspendpwr       
};

enum Modr {         // magnetometer output data rate  
  MODR_2Hz = 0,     
  MODR_6Hz,
  MODR_8Hz,
  MODR_10Hz,  
  MODR_15Hz,
  MODR_20Hz,
  MODR_25Hz, 
  MODR_30Hz 
};

enum MOpMode { // MAG Op Mode
  LowPower = 0,
  Regular,
  EnhancedRegular,
  HighAccuracy
};

enum MPwrMode { // MAG power mode
  Normal = 0,   
  Sleep,     
  Suspend,
  ForceMode  
};

uint8_t GPwrMode = NormalG;    // Gyro power mode
uint8_t Gscale = GFS_250DPS;  // Gyro full scale
//uint8_t Godr = GODR_250Hz;    // Gyro sample rate
uint8_t Gbw = GBW_23Hz;       // Gyro bandwidth
//
uint8_t Ascale = AFS_2G;      // Accel full scale
//uint8_t Aodr = AODR_250Hz;    // Accel sample rate
uint8_t APwrMode = NormalA;    // Accel power mode
uint8_t Abw = ABW_31_25Hz;    // Accel bandwidth, accel sample rate divided by ABW_divx
//
//uint8_t Mscale = MFS_4Gauss;  // Select magnetometer full-scale resolution
uint8_t MOpMode = Regular;    // Select magnetometer perfomance mode
uint8_t MPwrMode = Normal;    // Select magnetometer power mode
uint8_t Modr = MODR_10Hz;     // Select magnetometer ODR when in BNO055 bypass mode

uint8_t PWRMode = Normalpwr;    // Select BNO055 power mode
uint8_t OPRMode = NDOF;       // specify operation mode for sensors
uint8_t status;               // BNO055 data status register
float aRes, gRes, mRes;       // scale resolutions per LSB for the sensors

static bool bno055 = 1;

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
int16_t quatCount[4];   // Stores the 16-bit signed quaternion output
int16_t EulCount[3];    // Stores the 16-bit signed Euler angle output
int16_t LIACount[3];    // Stores the 16-bit signed linear acceleration output
int16_t GRVCount[3];    // Stores the 16-bit signed gravity vector output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, and magnetometer
int16_t tempGCount, tempMCount;      // temperature raw count output of mag and gyro
float   Gtemperature, Mtemperature;  // Stores the BNO055 gyro and mag internal chip temperatures in degrees Celsius

float pitch, yaw, roll;
float Pitch, Yaw, Roll;

float LIAx, LIAy, LIAz, GRVx, GRVy, GRVz;

/* 
 * Helper functions for BNO055 
 */
// I2C read/write functions for the BNO055 sensor

void initBNO055() {
   // Select BNO055 config mode
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
   delay(25);
   // Select page 1 to configure sensors
   writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x01);
   // Configure ACC
   writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | Ascale );
   // Set accelerometer high G detection to interrupt, and high gyro detection on z-axis
   writeByte(BNO055_ADDRESS, BNO055_INT_MSK, B00101000);
   writeByte(BNO055_ADDRESS, BNO055_INT_EN, B00101000);
   writeByte(BNO055_ADDRESS, BNO055_ACC_INT_SETTINGS, B11100000);
   writeByte(BNO055_ADDRESS, BNO055_ACC_HG_DURATION, B00000001);
   writeByte(BNO055_ADDRESS, BNO055_ACC_HG_THRESH, B11000000);
   writeByte(BNO055_ADDRESS, BNO055_GYR_INT_SETTINGS, B00100000);
   writeByte(BNO055_ADDRESS, BNO055_GYR_HR_Z_SET, B00000100);
   writeByte(BNO055_ADDRESS, BNO055_GYR_DUR_Z, B00000010);
   // Configure GYR
   writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, Gbw << 3 | Gscale );
   writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, GPwrMode);
   // Configure MAG
   writeByte(BNO055_ADDRESS, BNO055_MAG_CONFIG, MPwrMode << 5 | MOpMode << 3 | Modr );
   
   // Select page 0 to read sensors
   writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);

   // Select BNO055 gyro temperature source 
   writeByte(BNO055_ADDRESS, BNO055_TEMP_SOURCE, 0x01 );

   // Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
   writeByte(BNO055_ADDRESS, BNO055_UNIT_SEL, 0x01 );
   
   // Select BNO055 system power mode
   writeByte(BNO055_ADDRESS, BNO055_PWR_MODE, PWRMode );
 
   // Select BNO055 system operation mode
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );

   delay(25);
  }

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.requestFrom(address, 1);  // Read one byte from slave register address 
  Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

void readEulData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_EUL_HEADING_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readLIAData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_LIA_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

/*
 * Variable and containers for LEDs
 */
CRGB leds[NUM_LEDS];

static unsigned int curLedDelay = MAX_LED_DELAY;
static int centerLed = NUM_LEDS / 2;
static int maxLedPos = NUM_LEDS / 2;

static bool oddLeds = 0;
static bool particleDir = 1;
static bool speedDir = 1;

unsigned long dirCount;
unsigned long hueCount;

void setAllLeds(CRGB color) {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }

  FastLED.show();
}

void displayStatus(int statusNum, bool statusVal) {
  int Pos1 = 0;
  int Pos2 = 0;
  CRGB color;

  if(oddLeds) {
    Pos1 = centerLed + statusNum;
    Pos2 = centerLed - statusNum;
  } else {
    Pos1 = centerLed + statusNum;
    Pos2 = (centerLed -1) - statusNum;
  }

  if(statusVal) {
    color = CRGB::Green;
  } else {
    color = CRGB::Red;
  }
  
  leds[Pos1] = color;
  leds[Pos2] = color;

  FastLED.show();
}

class Particle {
  int currPos;
  int lastPos;
  unsigned long previousMillis;

  public:
  uint8_t currHue;
  uint16_t currDelay;
  
  Particle() {
    currPos = 0;
    previousMillis = 0;
    currHue = 0;

    if(particleDir) {
      currPos = 0;
    } else {
      if(oddLeds) {
        currPos = NUM_LEDS/2;
      } else {
        currPos = NUM_LEDS/2 -1;
      }
    }
  }

  int getPos() {
    return currPos;
  }

  int getLastPos() {
    return lastPos;
  }

  void Update() {
    unsigned long currentMillis = millis();

    int Pos1 = 0;
    int Pos2 = 0;

    lastPos = currPos;

    if(currentMillis - previousMillis >= currDelay) {

      if(oddLeds) {
        Pos1 = centerLed + currPos;
        Pos2 = centerLed - currPos;
      } else {
        Pos1 = centerLed + currPos;
        Pos2 = (centerLed -1) - currPos;
      }
      
      leds[Pos1] = CRGB::Black;
      leds[Pos2] = CRGB::Black;

      if(particleDir) {
        if((currPos == NUM_LEDS/2) && oddLeds){
          currPos = 0;
        } else if((currPos == NUM_LEDS/2 - 1) && (!oddLeds)){
          currPos = 0;
        } else {
          currPos++;
        }
      } else {
        if((currPos == 0) && oddLeds){
          currPos = centerLed;
        } else if((currPos == 0) && (!oddLeds)){
          currPos = centerLed - 1;
        } else {
          currPos--;
        }
      }
            
      previousMillis = currentMillis;

      if(particleDir) {
        if(oddLeds) {
          Pos1 = centerLed + currPos;
          Pos2 = centerLed - currPos;
        } else {
          Pos1 = centerLed + currPos;
          Pos2 = (centerLed -1) - currPos;
        }
      } else {
        if(oddLeds) {
          Pos1 = centerLed - currPos;
          Pos2 = centerLed + currPos;
        } else {
          Pos1 = centerLed - currPos;
          Pos2 = (centerLed -1) + currPos;
        }
      }
      leds[Pos1] = CHSV(currHue, 255,255);
      leds[Pos2] = CHSV(currHue, 255,255);

      FastLED.show();
    }
  }
};

Particle particle1;

/*
 * special effects for impact detection
 */
void impact() {
  setAllLeds(CRGB::Black);
  for(int i = 0; i < 10; i++) {
    setAllLeds(CRGB::Red);
    delay(30);
    setAllLeds(CRGB::Black);
    delay(30);
  }

  writeByte(BNO055_ADDRESS, BNO055_SYS_TRIGGER, B01000000);
}

void spin() {
  CRGB colors[]={CRGB::Purple,CRGB::Red,CRGB::White,CRGB::Blue,CRGB::Purple};
  
  setAllLeds(CRGB::Black);

  for(unsigned int i = 0; i < sizeof(colors); i++) {
    setAllLeds(colors[i]);
    delay(100);
    setAllLeds(CRGB::Black);
    delay(50);
  }

  writeByte(BNO055_ADDRESS, BNO055_SYS_TRIGGER, B01000000);
}

/*
 * Main Setup for project
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);

  /*
   * LED Configuration
   */
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // reset all LEDS to off
  setAllLeds(CRGB::Black);

  // flash all leds to indicate code booting
  for(int i = 1; i < 3; i++) {
    setAllLeds(CRGB::Indigo);
    delay(200);
    setAllLeds(CRGB::Black);
    delay(200);
  }

  if(NUM_LEDS % 2 == 1) {
     oddLeds = 1;
     maxLedPos = NUM_LEDS/2;
  } else {
     oddLeds = 0;
     maxLedPos = NUM_LEDS/2 - 1;
  }

  particle1.currHue = 0;

  /*
   * Motion configuration
   */
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(1000);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(BNO055_ADDRESS, BNO055_CHIP_ID);  // Read WHO_AM_I register for BNO055
  if(SERIAL_DEBUG) {
    Serial.println("BNO055 9-axis motion sensor...");
    Serial.print("BNO055 Address = 0x"); Serial.println(BNO055_ADDRESS, HEX);
    Serial.print("BNO055 WHO_AM_I = 0x"); Serial.println(BNO055_CHIP_ID, HEX);
    Serial.print("BNO055 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.println(" I should be 0xA0"); 
    
    // Read the WHO_AM_I register of the accelerometer, this is a good test of communication
    byte d = readByte(BNO055_ADDRESS, BNO055_ACC_ID);  // Read WHO_AM_I register for accelerometer
    Serial.print("BNO055 ACC "); Serial.print("I AM "); Serial.print(d, HEX); Serial.println(" I should be 0xFB"); 
    
    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte e = readByte(BNO055_ADDRESS, BNO055_MAG_ID);  // Read WHO_AM_I register for magnetometer
    Serial.print("BNO055 MAG "); Serial.print("I AM "); Serial.print(e, HEX); Serial.println(" I should be 0x32");  
    
    // Read the WHO_AM_I register of the gyroscope, this is a good test of communication
    byte f = readByte(BNO055_ADDRESS, BNO055_GYRO_ID);  // Read WHO_AM_I register for LIS3MDL
    Serial.print("BNO055 GYRO "); Serial.print("I AM "); Serial.print(f, HEX); Serial.println(" I should be 0x0F");
  }

  if (c == 0xA0) // BNO055 WHO_AM_I should always be 0xA0
  {  
    if(SERIAL_DEBUG) {
      Serial.println("BNO055 is online...");
      
      // Check software revision ID
      byte swlsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_LSB);
      byte swmsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_MSB);
      Serial.print("BNO055 SW Revision ID: "); Serial.print(swmsb, HEX); Serial.print("."); Serial.println(swlsb, HEX); 
      Serial.println("Should be 03.04");
      
      // Check bootloader version
      byte blid = readByte(BNO055_ADDRESS, BNO055_BL_REV_ID);
      Serial.print("BNO055 bootloader Version: "); Serial.println(blid); 
      }
    
    // Check self-test results
    byte selftest = readByte(BNO055_ADDRESS, BNO055_ST_RESULT);
    
    if(selftest & 0x01) {
      if(SERIAL_DEBUG) {
        Serial.println("accelerometer passed selftest"); 
      }
      displayStatus(1, 1);
    } else {
      if(SERIAL_DEBUG) {
        Serial.println("accelerometer failed selftest");
      }
      displayStatus(1, 0); 
    }
    delay(500);
    if(selftest & 0x02) {
      if(SERIAL_DEBUG) {
        Serial.println("magnetometer passed selftest"); 
      }
      displayStatus(2, 1);
    } else {
      if(SERIAL_DEBUG) {
        Serial.println("magnetometer failed selftest"); 
      }
      displayStatus(2, 0);
    }
    delay(500);  
    if(selftest & 0x04) {
      if(SERIAL_DEBUG) {
        Serial.println("gyroscope passed selftest"); 
      }
      displayStatus(3, 1);
    } else {
      if(SERIAL_DEBUG) {
        Serial.println("gyroscope failed selftest"); 
      }
      displayStatus(3, 0);
    }
    delay(500);      
    if(selftest & 0x08) {
      if(SERIAL_DEBUG) {
        Serial.println("MCU passed selftest"); 
      }
      displayStatus(4, 1);
    } else {
      if(SERIAL_DEBUG) {
        Serial.println("MCU failed selftest"); 
      }
      displayStatus(4, 0);
    }
      
    delay(3000);
    setAllLeds(CRGB::Black);

    initBNO055(); // Initialize the BNO055
    if(SERIAL_DEBUG) {
      Serial.println("BNO055 initialized for sensor mode...."); // Initialize BNO055 for sensor read 
    }
  } else {
    bno055 = 0;
    for(int i = 0; i < 10; i++) {
      displayStatus(1, 0);
      displayStatus(2, 0);
      displayStatus(3, 0);
      displayStatus(4, 0);
      delay(500);
      setAllLeds(CRGB::Black);
    }
  }
}

void loop() {
  
  if(bno055) {
    /*
     * Accelerometer is there, let's do some magic
     */

    // Detect any interrupts triggered, i.e. due to high G
    byte intStatus = readByte(BNO055_ADDRESS, BNO055_INT_STATUS);  
    if(intStatus > 8) {
      impact();
    } else if(intStatus > 0) {
      spin();
    }
     
    readEulData(EulCount);  // Read the x/y/z adc values   
    // Calculate the Euler angles values in degrees
    Yaw = (float)EulCount[0]/16.;  
    Roll = (float)EulCount[1]/16.;  
    Pitch = (float)EulCount[2]/16.; 

    if(SERIAL_DEBUG) {
      Serial.println((String)"Pitch: " + Pitch + " Roll: " + Roll + " Yaw: " + Yaw);
    }

    /*
     * Use Roll values for LED hues
     */

    uint8_t hue = 0;

    // My roll value was off by about -20 from truly centered, so you can adjust the following variable to try to center your values
    float rollCal = Roll + ROLLOFFSET;

    // Map "sane" values since hat will be on head and should only have limited motion
    if(rollCal < MINROLL) {
      hue = 0;
    } else if(rollCal > MAXROLL) {
      hue = 255;
    } else {
      hue = map(rollCal, MINROLL, MAXROLL, 0, 255);
    }

    // Set the hue for our particle
    particle1.currHue = hue;

    /*
     * Use Pitch values for speed LEDS move
     */
    
    if(Pitch < MINPITCH) {
      particleDir = 0;
      curLedDelay = MIN_LED_DELAY;
    } else if(Pitch < PITCHCENTER) {
      particleDir = 0;
      curLedDelay = map(Pitch, MINPITCH, PITCHCENTER, MIN_LED_DELAY, MAX_LED_DELAY);
    } else if(Pitch > MAXPITCH) {
      particleDir = 1;
      curLedDelay = MIN_LED_DELAY;
    } else {
      particleDir = 1;
      curLedDelay = map(Pitch, PITCHCENTER, MAXPITCH, MAX_LED_DELAY, MIN_LED_DELAY);
    }

    particle1.currDelay = curLedDelay;
    
    
    // Now that everything is set, update the particle
    particle1.Update();
     
  } else {
    /* 
     *  Accelerometer missing, go into demo mode
     */
    
    if(particle1.currHue > 255) {
      particle1.currHue = 0;
    }
  
    if(particle1.getPos() != particle1.getLastPos()) {
      if((particle1.getPos() == maxLedPos) && speedDir) {
        curLedDelay++;
      } else if ((particle1.getPos() == 0) && !(speedDir)) {
        curLedDelay--;
      }
    }
  
    if(curLedDelay == MAX_LED_DELAY) {
      speedDir = 0;
    } else if (curLedDelay == MIN_LED_DELAY) {
      speedDir = 1;
    }
    
    particle1.Update();
  
    unsigned long millsNow = millis();
    
    if(millsNow - hueCount > 500) {
      particle1.currHue += 10;
      hueCount = millsNow;
    }
    
    if(millsNow - dirCount > 60000) {
      if(particleDir) {
        particleDir = 0;
      } else {
        particleDir = 1;
      }
  
      dirCount = millsNow;
    }
  }
}
