#include "legato.h"
#include "interfaces.h"

	uint8_t write_data2[]={0,0};
	//size_t size= 1;
static	le_spi_DeviceHandleRef_t spiHandle;

#define ILI9341_TFTWIDTH   240      ///< ILI9341 max TFT width
#define ILI9341_TFTHEIGHT  320      ///< ILI9341 max TFT height

#define ILI9341_NOP        0x00     ///< No-op register
#define ILI9341_SWRESET    0x01     ///< Software reset register
#define ILI9341_RDDID      0x04     ///< Read display identification information
#define ILI9341_RDDST      0x09     ///< Read Display Status

#define ILI9341_SLPIN      0x10     ///< Enter Sleep Mode
#define ILI9341_SLPOUT     0x11     ///< Sleep Out
#define ILI9341_PTLON      0x12     ///< Partial Mode ON
#define ILI9341_NORON      0x13     ///< Normal Display Mode ON

#define ILI9341_RDMODE     0x0A     ///< Read Display Power Mode
#define ILI9341_RDMADCTL   0x0B     ///< Read Display MADCTL
#define ILI9341_RDPIXFMT   0x0C     ///< Read Display Pixel Format
#define ILI9341_RDIMGFMT   0x0D     ///< Read Display Image Format
#define ILI9341_RDSELFDIAG 0x0F     ///< Read Display Self-Diagnostic Result

#define ILI9341_INVOFF     0x20     ///< Display Inversion OFF
#define ILI9341_INVON      0x21     ///< Display Inversion ON
#define ILI9341_GAMMASET   0x26     ///< Gamma Set
#define ILI9341_DISPOFF    0x28     ///< Display OFF
#define ILI9341_DISPON     0x29     ///< Display ON

#define ILI9341_CASET      0x2A     ///< Column Address Set
#define ILI9341_PASET      0x2B     ///< Page Address Set
#define ILI9341_RAMWR      0x2C     ///< Memory Write
#define ILI9341_RAMRD      0x2E     ///< Memory Read

#define ILI9341_PTLAR      0x30     ///< Partial Area
#define ILI9341_VSCRDEF    0x33     ///< Vertical Scrolling Definition
#define ILI9341_MADCTL     0x36     ///< Memory Access Control
#define ILI9341_VSCRSADD   0x37     ///< Vertical Scrolling Start Address
#define ILI9341_PIXFMT     0x3A     ///< COLMOD: Pixel Format Set

#define ILI9341_FRMCTR1    0xB1     ///< Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9341_FRMCTR2    0xB2     ///< Frame Rate Control (In Idle Mode/8 colors)
#define ILI9341_FRMCTR3    0xB3     ///< Frame Rate control (In Partial Mode/Full Colors)
#define ILI9341_INVCTR     0xB4     ///< Display Inversion Control
#define ILI9341_DFUNCTR    0xB6     ///< Display Function Control

#define ILI9341_PWCTR1     0xC0     ///< Power Control 1
#define ILI9341_PWCTR2     0xC1     ///< Power Control 2
#define ILI9341_PWCTR3     0xC2     ///< Power Control 3
#define ILI9341_PWCTR4     0xC3     ///< Power Control 4
#define ILI9341_PWCTR5     0xC4     ///< Power Control 5
#define ILI9341_VMCTR1     0xC5     ///< VCOM Control 1
#define ILI9341_VMCTR2     0xC7     ///< VCOM Control 2

#define ILI9341_RDID1      0xDA     ///< Read ID 1
#define ILI9341_RDID2      0xDB     ///< Read ID 2
#define ILI9341_RDID3      0xDC     ///< Read ID 3
#define ILI9341_RDID4      0xDD     ///< Read ID 4

#define ILI9341_GMCTRP1    0xE0     ///< Positive Gamma Correction
#define ILI9341_GMCTRN1    0xE1     ///< Negative Gamma Correction
//#define ILI9341_PWCTR6     0xFC

// Color definitions
#define ILI9341_BLACK       0x0000  ///<   0,   0,   0
#define ILI9341_NAVY        0x000F  ///<   0,   0, 123
#define ILI9341_DARKGREEN   0x03E0  ///<   0, 125,   0
#define ILI9341_DARKCYAN    0x03EF  ///<   0, 125, 123
#define ILI9341_MAROON      0x7800  ///< 123,   0,   0
#define ILI9341_PURPLE      0x780F  ///< 123,   0, 123
#define ILI9341_OLIVE       0x7BE0  ///< 123, 125,   0
#define ILI9341_LIGHTGREY   0xC618  ///< 198, 195, 198
#define ILI9341_DARKGREY    0x7BEF  ///< 123, 125, 123
#define ILI9341_BLUE        0x001F  ///<   0,   0, 255
#define ILI9341_GREEN       0x07E0  ///<   0, 255,   0
#define ILI9341_CYAN        0x07FF  ///<   0, 255, 255
#define ILI9341_RED         0xF800  ///< 255,   0,   0
#define ILI9341_MAGENTA     0xF81F  ///< 255,   0, 255
#define ILI9341_YELLOW      0xFFE0  ///< 255, 255,   0
#define ILI9341_WHITE       0xFFFF  ///< 255, 255, 255
#define ILI9341_ORANGE      0xFD20  ///< 255, 165,   0
#define ILI9341_GREENYELLOW 0xAFE5  ///< 173, 255,  41
#define ILI9341_PINK        0xFC18  ///< 255, 130, 198

//-----------------------------------------------------------------
	const uint8_t init1[] = { 0x03, 0x80, 0x02 };
		const uint8_t init2[] = { 0x00, 0xC1, 0x30 };
		const uint8_t init3[] = { 0x64, 0x03, 0x12, 0x81 };
		const uint8_t init4[] = { 0x85, 0x00, 0x78 };
		const uint8_t init5[] = { 0x39, 0x2C, 0x00, 0x34, 0x02 };
		const uint8_t init6[] = { 0x20 };
		const uint8_t init7[] = { 0x00, 0x00 };
		const uint8_t Power_control_VRH[] = { 0x23 };
		const uint8_t Power_control_SAP[] = { 0x10 };
		const uint8_t VCM_control[] = { 0x3e, 0x28 };
		const uint8_t VCM_control2[] = { 0x86 };
		const uint8_t Memory_access_control[] = { 0x48 };
		// const u8 Vertical_scroll_zero[] = { 0x00 };//
		const uint8_t PIXFMT[] = { 0x55 };
		const uint8_t FRMCTR1[] = { 0x00, 0x18 };
		const uint8_t Display_function_control[] = { 0x08, 0x82, 0x27 };
		const uint8_t Gamma_function_disable[] = { 0x00 };
		const uint8_t Gamma_curve_selected[] = { 0x01 };
		const uint8_t Set_gamma[] = { 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
			0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00 };
		const uint8_t Set_gamma2[] = { 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
	    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F };
//-----------------------------------------------------------------
//-----------------------------------------------------------------
void setAddrWindow(uint16_t x1,uint16_t y1,uint16_t w,uint16_t h)
{
    uint16_t x2 = (x1 + w - 1), y2 = (y1 + h - 1);
	 uint8_t write_data=0x2A;
	 le_spi_WriteHD(spiHandle,&write_data,1);
	 le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_HIGH,true);
	 write_data2[0]= x1>>8;
	 write_data2[1]= x1& 0xFF;
	 le_spi_WriteHD(spiHandle,write_data2,2);
	 le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_HIGH,true);
	 write_data2[0]= x2>>8;
	 write_data2[1]= x2& 0xFF;
	 le_spi_WriteHD(spiHandle,write_data2,2);
	 le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_LOW,true);
	 write_data=0x2B;
	 le_spi_WriteHD(spiHandle,&write_data,1);
	 le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_HIGH,true);
	 write_data2[0]= y1>>8;
	 write_data2[1]= y1& 0xFF;
	 le_spi_WriteHD(spiHandle,write_data2,2);
	 le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_HIGH,true);
	 write_data2[0]= y2>>8;
	 write_data2[1]= y2& 0xFF;
	 le_spi_WriteHD(spiHandle,write_data2,2);
	 le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_LOW,true);
	 write_data=0x2C;
	 le_spi_WriteHD(spiHandle,&write_data,1);
}

void sendCommand (uint8_t commandByte,const uint8_t *dataBytes, uint8_t numDataBytes)
{
	le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_LOW,true);
	le_spi_WriteHD(spiHandle,&commandByte,1);
	le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_HIGH,true);
	le_spi_WriteHD(spiHandle,dataBytes,numDataBytes);

}
void reset_rst()
{
	le_gpioPin8_SetPushPullOutput(LE_GPIOPIN8_ACTIVE_LOW,true);
	sleep(1);
	le_gpioPin8_SetPushPullOutput(LE_GPIOPIN8_ACTIVE_HIGH,true);
	sleep(1);

}

/*void writeChar(int16_t x, int16_t y, unsigned char c,
		  uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y)
{
    for(int8_t i=0; i<5; i++ ) { // Char bitmap = 5 columns
         uint8_t line = (font[c * 5 + i]);
         for(int8_t j=0; j<8; j++, line >>= 1) {
             if(line & 1) {
                 if(size_x == 1 && size_y == 1)
                 {
                	 LE_INFO("HELLO %d",i);
                	 setAddrWindow(x+i,y+j,1,1);
                	 write_data2[0]=0x00;
                	 write_data2[1]=0x1F;
                     le_spi_WriteHD(spiHandle,write_data2,2);
                 }
             }
         }
    }
}*/

void begin()
{
	reset_rst();
	sendCommand(0xEF,init1,NUM_ARRAY_MEMBERS(init1));
	sendCommand(0xCF,init2,NUM_ARRAY_MEMBERS(init2));
	sendCommand(0xED,init3,NUM_ARRAY_MEMBERS(init3));
	sendCommand(0xE8,init4,NUM_ARRAY_MEMBERS(init4));
	sendCommand(0xCB,init5,NUM_ARRAY_MEMBERS(init5));
	sendCommand(0xF7,init6,NUM_ARRAY_MEMBERS(init6));
	sendCommand(0xEA,init7,NUM_ARRAY_MEMBERS(init7));
	sendCommand(ILI9341_PWCTR1,Power_control_VRH,NUM_ARRAY_MEMBERS(Power_control_VRH));
	sendCommand(ILI9341_PWCTR2,Power_control_SAP,NUM_ARRAY_MEMBERS(Power_control_SAP));
	sendCommand(ILI9341_VMCTR1,VCM_control,NUM_ARRAY_MEMBERS(VCM_control));
	sendCommand(ILI9341_VMCTR2,VCM_control2,NUM_ARRAY_MEMBERS(VCM_control2));
	sendCommand(ILI9341_MADCTL,Memory_access_control,NUM_ARRAY_MEMBERS(Memory_access_control));
	sendCommand(ILI9341_PIXFMT,PIXFMT,NUM_ARRAY_MEMBERS(PIXFMT));
	sendCommand(ILI9341_FRMCTR1,FRMCTR1,NUM_ARRAY_MEMBERS(FRMCTR1));
	sendCommand(ILI9341_DFUNCTR,Display_function_control,NUM_ARRAY_MEMBERS(Display_function_control));
	sendCommand(0xF2,Gamma_function_disable,NUM_ARRAY_MEMBERS(Gamma_function_disable));
	sendCommand(ILI9341_GAMMASET,Gamma_curve_selected,NUM_ARRAY_MEMBERS(Gamma_curve_selected));
	sendCommand(ILI9341_GMCTRP1,Set_gamma,NUM_ARRAY_MEMBERS(Set_gamma));
	sendCommand(ILI9341_GMCTRN1,Set_gamma2,NUM_ARRAY_MEMBERS(Set_gamma2));
	le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_LOW,true);
	uint8_t write_cmd=ILI9341_SLPOUT;
	le_spi_WriteHD(spiHandle,&write_cmd,1);
	write_cmd=ILI9341_DISPON;
	le_spi_WriteHD(spiHandle,&write_cmd,1);


}



COMPONENT_INIT
{
	uint8_t write_cmd= 0x01;
	//SPI config

	le_result_t res = le_spi_Open("spidev1.0", &spiHandle);
	LE_FATAL_IF(res != LE_OK, "le_spi_Open failed with result=%s", LE_RESULT_TXT(res));
	le_spi_Configure(spiHandle, 0, 8, 4800000, 0);
	//GPIO config
	le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_LOW,true);
	if ( le_gpioPin7_GetPolarity()==LE_GPIOPIN7_ACTIVE_LOW) LE_INFO("LOW");
	//Command software reset
	le_spi_WriteHD(spiHandle,&write_cmd,1);
	begin();
	//Write Pixel
	/*setAddrWindow(0,0,240,320);
	for(int i=0;i<240*320;i++)
	{
	le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_HIGH,true);
	for(int j=0;j<1024;j++){
	write_data2[0]=0xFF;
	write_data2[1]=0xFF;
	le_spi_WriteHD(spiHandle,write_data2,2);
	}
	le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_LOW,true);*/
	//---
	setAddrWindow(0,0,240,320);
	uint8_t write_data3[1024];
	for(int i=0;i<300;i++)
	{
	le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_HIGH,true);
	for(int j=0;j<1024;j++){
	write_data3[j]=0x11;
	write_data3[++j]=0x22;
	}
	le_spi_WriteHD(spiHandle,write_data3,1024);
	}
	le_gpioPin7_SetPushPullOutput(LE_GPIOPIN7_ACTIVE_LOW,true);
	//---
	//writeChar(1,1,'h',0x0000,0xFFFF,1,1);


	LE_INFO("DONE2");
}
