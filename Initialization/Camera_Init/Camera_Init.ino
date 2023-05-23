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
#define AEW 0x24 // AGC/AEC stable operating region upper limit defaults 75
#define AEB 0x25 // AGC/AEC stable operating region lower limit defaults 63
#define VPT 0x26 // fast mode operating region defaults D4
#define HREF 0x32 // HREF control defaults 80
#define TSLB 0x3A // Line Buffer Test Option defaults 0D
#define COM11 0x3B // common control 11 defaults 3B bit[7]: night mode, bit[6:5]: minimum frame rate of nightmode, bit[4]: D56_auto, bit[3]: banding filter value select, bit[2]: reserved, bit[1]: exposure timeing can be less than limit, bit[0]: reserved
#define COM12 0x3C // common control 12 href option defaults 68
#define COM13 0x3D // common control 13 defaults 88 bit[7]: gamma enable, bit[6]: uv saturation level, bit[5:1]: reserved, bit[0]: uv swap
#define COM14 0x3E // common control 14 DCW and PCLK enable/divider/manual scaling defaults 00
#define EDGE 0x3F // edge enhancement adjustment defaults 00
#define COM15 0x40 // common control 15 defaults C0 bit[7:6]: data format, bit[5:4]: RGB 555/565 option, bit[3:0] reserved
#define COM16 0x41 // common control 16 defaults 08 bit[7:6]: reserved, bit[5]: enable edge enhancement threshold auto-adjustment for YUV output, bit[4]: denoise threshold auto adjustment, bit[3]: AWB gain enable, bit[2]: reserved, bit[1]: color matrix coefficient double, bit[0]: reserved
#define COM17 0x42 // common control 17 defaults 00 bit[7:6]: AEC window, bit[5:4]: reserved, bit[3]: DSP color bar enable, bit[2:0]: reserved
#define GFIX 0x68 // fix gain control defaults 00
#define SCALING_XSC 0x70 // horizontal scale factor
#define SCALING_YSC 0x71 // vertical scale factor
#define HAECC1 0x9F // histogram based AEC/AGC control 1 defaults C0
#define HAECC2 0xA0 // histogram based AEC/AGC control 2 defaults 90
#define BD50MAX 0xA5 // 50Hz banding step limit
#define HAECC3 0xA6 // histogram based AEC/AGC control 3 defaults F0
#define HAECC4 0xA7 // histogram based AEC/AGC control 4 defaults C1
#define HAECC5 0xA8 // histogram based AEC/AGC control 5 defaults F0
#define HAECC6 0xA9 // histogram based AEC/AGC control 6 defaults C1
#define HAECC7 0xAA // AEC algorithm selection defaults 14
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
  Wire.setClock(100000); // eventhough the ov7670 datasheet says 400kHz, only 100kHz seems to work

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
    // I realize that writing consecutive bits like this is no optimized, but it allows me to only have one function to do all of it
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
    writeReg(AEW, 0x95);
    writeReg(AEB, 0x33);
    writeReg(VPT, 0xE3);
    writeReg(HAECC1, 0x78);
    writeReg(HAECC2, 0x68);
    writeReg(0xA1, 0x03); /* magic reserved bit*/
    writeReg(HAECC3, 0xD8);
    writeReg(HAECC4, 0xD8);
    writeReg(HAECC5, 0xF0);
    writeReg(HAECC6, 0x90);
    writeReg(HAECC7, 0x94);
    writeReg(COM8, 0xE7);

    /* Almost all of these are magic "reserved" values.  */
    writeReg(COM5, 0x61);
    writeReg(COM6, 0x4B);
    writeReg(0x16, 0x02);
    writeReg(0x1E, 0x07);
    writeReg(0x21, 0x02);
    writeReg(0x22, 0x91);
    writeReg(0x29, 0x07);
    writeReg(0x33, 0x0B);
    writeReg(0x35, 0x0B);
    writeReg(0x37, 0x1D);
    writeReg(0x38, 0x71);
    writeReg(0x39, 0x2A);
    writeReg(COM12, 0x78);
    writeReg(0x4D, 0x40);
    writeReg(0x4E, 0x20);
    writeReg(GFIX, 0x00);
    writeReg(0x6B, 0x4A);
    writeReg(0x74, 0x10);
    writeReg(0x8D, 0x4F);
    writeReg(0x8E, 0x00);
    writeReg(0x8F, 0x00);
    writeReg(0x90, 0x00);
    writeReg(0x91, 0x00);
    writeReg(0x96, 0x00);
    writeReg(0x9A, 0x00);
    writeReg(0xB0, 0x84);
    writeReg(0xB1, 0x0C);
    writeReg(0xB2, 0x0E);
    writeReg(0xB3, 0x82);
    writeReg(0xB8, 0x0A);

    /* More reserved magic, some of which tweaks white balance */
    writeReg(0x43, 0x0A);
    writeReg(0x44, 0xF0);
    writeReg(0x45, 0x34);
    writeReg(0x46, 0x58);
    writeReg(0x47, 0x28);
    writeReg(0x48, 0x3A);
    writeReg(0x59, 0x88);
    writeReg(0x5A, 0x88);
    writeReg(0x5B, 0x44);
    writeReg(0x5C, 0x67);
    writeReg(0x5D, 0x44);
    writeReg(0x5E, 0x0E);
    writeReg(0x6C, 0x0A);
    writeReg(0x6D, 0x55);
    writeReg(0x6E, 0x11);
    writeReg(0x6F, 0x9F); /* "9e for advance AWB" */
    writeReg(0x6A, 0x40);
    writeReg(BLUE, 0x40);
    writeReg(RED, 0x60);
    writeReg(COM8, 0xE7);


	  /* Matrix coefficients */
    writeReg(0x4F, 0x80);
    writeReg(0x50, 0x80);
    writeReg(0x51, 0x00);
    writeReg(0x52, 0x22);
    writeReg(0x53, 0x5E);
    writeReg(0x54, 0x54);
    writeReg(0x58, 0x9E);

    /* de-noise and more reserved registers */
    writeReg(COM16, 0x08);
    writeReg(EDGE, 0x00);
    writeReg(0x75, 0x05);
    writeReg(0x76, 0xE1);
    writeReg(0x4C, 0x00);
    writeReg(0x77, 0x01);
    writeReg(COM13, 0xC3);
    writeReg(0x4B, 0x09);
    writeReg(0xC9, 0x60);
    writeReg(COM16, 0x38);
    writeReg(0x56, 0x40); // contrast control
    writeReg(0x34, 0x11);
    writeReg(COM11, 0x12);
    writeReg(0xA4, 0x88);
    writeReg(0x97, 0x30);
    writeReg(0x98, 0x20);
    writeReg(0x99, 0x30);
    writeReg(0x9A, 0x84);
    writeReg(0x9B, 0x29);
    writeReg(0x9C, 0x03);
    writeReg(0x9D, 0x4C);
    writeReg(0x9E, 0x3F);
    writeReg(0x78, 0x04);

    /* Extra-weird stuff.  Some sort of multiplexor register */
    writeReg(0x79, 0x01);
    writeReg(0xC8, 0xF0);
    writeReg(0x79, 0x0F);
    writeReg(0xC8, 0x00);
    writeReg(0x79, 0x10);
    writeReg(0xC8, 0x7E);
    writeReg(0x79, 0x0A);
    writeReg(0xC8, 0x80);
    writeReg(0x79, 0x0B);
    writeReg(0xC8, 0x01);
    writeReg(0x79, 0x0C);
    writeReg(0xC8, 0x0F);
    writeReg(0x79, 0x0D);
    writeReg(0xC8, 0x20);
    writeReg(0x79, 0x09);
    writeReg(0xC8, 0x80);
    writeReg(0x79, 0x02);
    writeReg(0xC8, 0xC0);
    writeReg(0x79, 0x03);
    writeReg(0xC8, 0x40);
    writeReg(0x79, 0x05);
    writeReg(0xC8, 0x30);
    writeReg(0x79, 0x26);
  }
  else {
    Serial.println("Camera not found, please reset arduino to try again");
  }
}

void loop() {
  // turns out loop is still needed to run arduino
}

//-------------------------------functions---------------------------------------------------------
int readReg(int x) {

  Wire.beginTransmission(ovAddr); // begin i2c transmission
  Wire.write(byte(x)); // point to register
  Wire.endTransmission();
  Wire.requestFrom(ovAddr, 1); // regquest 1 byte from camera

  do { // do while used to ensure something is returned each time function is called
  byte y = Wire.read();
  return y;
  }
  while (Wire.available());
}

void writeReg(int x, int data) {
  Wire.beginTransmission(ovAddr); // begin i2c transmission
  Wire.write(byte(x)); // point to register
  Wire.write(byte(data)); // send data to register
  Wire.endTransmission();
}
