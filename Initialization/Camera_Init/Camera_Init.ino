/**************************************************************************************************
* Ov7670 Camera I2C Initialization
* by Dirk Kaiser
* 5/12/2023
*
*   This intitializes the popular ov7670 camera module for use with a seperate FPGA board.This 
* allows for an already created i2c library to be used for setup of the camera and the changing of
* setting while an FPGA board is used for data acquisition and processing. This is based off of the
* arduino ov767x library (https://github.com/arduino-libraries/Arduino_OV767X), but with changes to
* make it more understandable to what is happening on the hardware level as well as cutting away 
* the code that is not needed when capturing images on an FPGA instead of solely on an arduino.
**************************************************************************************************/

#include <Wire.h> // arduino library for i2c

/*
this does not work because the wire library takes 7 bit addresses and does the last bit for read and write automatically
// The ov7670 i2c addresses are 42 for write and 43 for read
#define ov_W 0x42
#define ov_R 0x43
*/
// 7 bit ov7670 i2c address
#define ovAddr 0x21

//--------------------------ov7670 register address table------------------------------------------
// this is a working list of registers for the ov7670 holding the first 32 registers + any other registers that are found to be needed
// descriptions are taken from the ov7670 data sheet, more information can be found here http://web.mit.edu/6.111/www/f2016/tools/OV7670_2006.pdf
#define GAIN 0x00 // gain control range AGC(Automatic Gain Control)[7:0] 00 to FF defaults 00
#define BLUE 0x01 // blue channel gain control range 00 to FF defaults 80
#define RED 0x02 // red channel gain control range 00 to FF defaults 80
#define VREF 0x03 // vertical frame control defaults 00 (bit[7:6] is gain control AGC[9:8]) bit[3:2] VREF end low 2 bits (high 8 bits at VSTOP[7:0]) bit[1:0] VREF star low 2 bits (high 8 bits at VSTRT[7:0])
#define COM1 0x04 // common control 1 bit[6]: CCIR656 format, Bit[1:0] AEC(Automatic Exposure Control) low 2 LSB defaults 00
#define BAVE 0x05 // U/B average level, automatically updated based on chip output format defaults 00
#define GbAVE 0x06 // Y/Gb average level, automatically updated based on chip output format defaults 00
#define AECHH 0x07 // exposure value 5 MSB AEC[15:10] bit[5:0] defaults 00
#define RAVE 0x08 // V/R average level, automatically updated based on chip output format defauts 00
#define COM2 0x09 // common control 2 bit[4]: soft sleep, bit[1:0]: output drive capability defaults 01
#define PID 0x0A // product ID Number MSB defaults 76
#define VER 0x0B // product ID Number LSB defaults 73 
#define COM3 0x0C // common control 3 defaults 00 bit[6]: output data MSB and LSB swap, bit[5]: tri-state clock option, bit[4]: tri-state data ouption, bit[3]: scale enable, bit[2]: DCW enable
#define COM4 0x0D // common control 4 bit[5:4]: average option defaults 00
#define COM5 0x0E // common control 5 all reserved defaults 01
#define COM6 0x0F // common control 6 defaults 43 bit[7]: output of optical line, bit[1]: reset all timin when format changes
#define AECH 0x10 // exposure value AEC[9:2] bit[7:0]: defaults 40
#define CLKRC 0x11 // internal clock bit[6]: use external clock directly, bit[5:0]: internal clock pre-scalar defaults 80
#define COM7 0x12 // common control 7 defaults 00 bit[7]: SCCB register reset, bit[5]: CIF output format, bit[4]: QVGA output format, bit[3]: QCIF output format, bit[2]: RGB selection, bit[1]: color bar, bit[0]: Raw RGB 
#define COM8 0x13 // common control 8 defaults 8F bit[7]: enable fast AGC/AEC algorithm, bit[6]: AEC step sizw limit, bit[5]: banding filter on/off, bit[2]: AGC enable, bit[1]: awb enable, bit[0]: AEC enable
#define COM9 0x14 // common control 9 defaults 4A bit[6:4]: automatic gain ceiling, bit[0]: freeze AGC/AEC
#define COM10 0x15 // common control 10 defaults bit[6]: HREF changes to HSYNC, bit[5]: PCLK output option, bit[4]: PCLK reverse, bit[3] HREF reverse, bit[2] VSYNC option, bit[1] VSYNC negative, bit[0] HSYNC negative
#define RSVD 0x16 // NO R/W defaults XX
#define HSTART 0x17 // output format horizontal frame start high 8bit defaults 11
#define HSTOP 0x18 // output format horizontal frame end high 8bit defaults 61
#define VSTRT 0x19 // output format vertical frame start high 8bit defaults 03
#define VSTOP 0x1A // output format vertical frame end high 8bit defaults 7B
#define PSHIFT 0x1B // data format pixel delat select defaults 00
#define MIDH 0x1C // manufacturer ID byte high defaults 7F
#define MIDL 0x1D // manufacturer ID byte low defauts A2
#define MVFP 0x1E // mirror/vflip enable defaults 01
#define LAEC 0x1F // Reserved defaults 00
#define HREF 0x32 // HREF control defaults 80
#define TSLB 0x3A // Line Buffer Test Option defaults 0D
#define COM14 0x3E // common control 14 DCW and PCLK enable/divider/manual scaling defaults 00
#define SCALING_XSC 0x70 // horizontal scale factor
#define SCALING_YSC 0x71 // vertical scale factor
#define BD50MAX 0xA5 // 50Hz banding step limit
#define BD60MAX 0xAB // 60Hz banding step limit
//-------------------------------------------------------------------------------------------------

//-----------------------------function declarations-----------------------------------------------
int readReg(int x); // function to read camera register 'x' over i2c 
void writeReg(int x, int data); // function to write data to camera register 'x' over i2c
//-------------------------------------------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // start serial for output
  Serial.println("Starting intitialization....");

  Wire.begin(); // join i2c bus (address optional for master)

  int id;
  id = readReg(PID);
  Serial.print("Device ID = ");
  Serial.println(id, HEX);

  if(id == 0x76) { // checking if camera is there and can be read from, data sheet shows that the id of the camera is set to 0x76
    Serial.println("ov7670 camera found!");

    writeReg(COM7, 0x80); // reseting all camera registers
    writeReg(CLKRC, 0x1); // setting camera clock scale to 30 fps, change this value for different fps
    /*
     * Clock scale: 3 = 15fps
     *              2 = 20fps
     *              1 = 30fps
     */

    // values for the following registers were taken from the arduino ov7670 library
    // orginal comments are kept as well as my added comments to try to help figure out what is going on
    writeReg(TSLB, 0x04); // ensures camera uses normal UV output
    writeReg(COM7, 0x00); // sets format to VGA, output data as YUV
  	/*
	   * Set the hardware window.  These values from OV don't entirely
	   * make sense - hstop is less than hstart.  But they work...
	   */
    // many of the camera registers have reserved bits that need to stay at either 0 or 1
    writeReg(HSTART, 0x13);
    writeReg(HSTOP, 0x01);
    writeReg(HREF, 0xB6);
    writeReg(VSTRT, 0x02);
    writeReg(VSTOP, 0x7A);
    writeReg(VREF, 0x0A);
    writeReg(COM3, 0x00);
    writeReg(COM14, 0x00);

    /* Mystery scaling numbers */
    // this is updating the registers that holds scaling information with values that are known to produce an acceptable image
    writeReg(SCALING_XSC, 0x3A);
    writeReg(SCALING_YSC, 0x35);
    writeReg(0x72, 0x11); // a lot of the registers going forward are not labeled. While they have names in the data sheet, 
    writeReg(0x73, 0xf0);
    writeReg(0xA2, 0x02);
    writeReg(COM10, 0x00);

    /* Gamma curve values */
    // setting the gamma correction registers with values that are known to produce an acceptable image most of them are unhelpful and it is quicker just to leave them as hex values, especially because that is how it is label in the arduino library
    writeReg(0x7A, 0x20);
    writeReg(0x7B, 0x10);
    writeReg(0x7C, 0x1E);
    writeReg(0x7D, 0x35);
    writeReg(0x7E, 0x5A);
    writeReg(0x7F, 0x69);
    writeReg(0x80, 0x76);
    writeReg(0x81, 0x80);
    writeReg(0x82, 0x88);
    writeReg(0x83, 0x8f);
    writeReg(0x84, 0x96);
    writeReg(0x85, 0xA3);
    writeReg(0x86, 0xAF);
    writeReg(0x87, 0xC4);
    writeReg(0x88, 0xD7);
    writeReg(0x89, 0xE8);

    /* AGC(Automatic Gain Control) and AEC(Automatic Exposure Control) parameters.  Note we start by disabling those features,
	   then turn them only after tweaking the values. */
    writeReg(COM8, 0xE0); // enables fast exposure, unlimited stepsize, and band filter
    writeReg(GAIN, 0x00);
    writeReg(AECH, 0x00);
    writeReg(COM4, 0x40); /* magic reserved bit */
    writeReg(COM9, 0x18); /* 4x gain + magic rsvd bit */
    writeReg(BD50MAX, 0x05);
    writeReg(BD60MAX, 0x07);
  }
}

//-------------------------------functions---------------------------------------------------------
int readReg(int x) {

  Wire.beginTransmission(ovAddr); // begin i2c transmission
  Wire.write(byte(x)); // point to register
  Wire.endTransmission;
  Wire.requestFrom(ovAddr, 1); // regquest 1 byte from camera

  do { // do while used to ensure something is returned each time function is called
  return Wire.read();
  }
  while (Wire.available());
}

void writeReg(int x, int data) {
  Wire.beginTransmission(ovAddr); // begin i2c transmission
  Wire.write(byte(x)); // point to register
  Wire.write(byte(data)); // send data to register
  Wire.endTransmission;
}
