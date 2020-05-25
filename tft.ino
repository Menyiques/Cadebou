#include <SPI.h>

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif

#define TFT_MOSI            19
#define TFT_SCLK            18
#define TFT_CS              5
#define TFT_DC              16
#define TFT_RST             23
#define ADC_PIN             34
#define TFT_BL              4  // Display backlight control pin


void setup_tft()
{
    tft.init();

    if (TFT_BL > 0) { // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
         pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode
         digitalWrite(TFT_BL, HIGH); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    }
    tft.setRotation(3);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(4);
    //tft.drawString("CADEBOU", tft.width() / 2, tft.height() / 2+10);
}

void tft_print(String s, int x, int y, int siz, int colFore){

    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(siz);
    tft.setTextColor(colFore, TFT_BLACK);
 
  }

  void printAtG(String s, int x, int y, int siz, int color){
    printAt(s,x*5,y,siz,color);
    }

 void print3AtG(String s1,String s2,String s3, int y,int color){
    tft.setTextDatum(TL_DATUM);
    printAt(s1,0,y*28,3,color);
    tft.setTextDatum(TC_DATUM);
    printAt(s2,120,y*28,3,color);
    tft.setTextDatum(TR_DATUM);
    printAt(s3,240,y*28,3,color);
    }

 void print2AtG(String s1,String s2, int y,int color){
    tft.setTextDatum(TL_DATUM);
    printAt(s1,0,y*28,3,color);
    tft.setTextDatum(TR_DATUM);
    printAt(s2,240,y*28,3,color);
    }
    void print1AtG(String s1, int y,int color){
    tft.setTextDatum(TC_DATUM);
    printAt(s1,120,y*28,3,color);
    }
     
  void printAt(String s, int x, int y, int siz, int color){
      tft.setTextSize(siz);
      tft.setTextColor(color);
      tft.drawString(s, x, y);
    }

    
