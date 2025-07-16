
#include "setup.h"

const __u8 max_size = 7;
const __u8 factor2 = 3;
__u8 actual_screen;

TFT_eSPI tft = TFT_eSPI();

__u8 screen1(bool clear){

    const char* text1 = "12000";
    __u8 size_1 = 7;
    __u8 font_1 = 1;
    __u16 pos_x_1 = tft.width()/2;
    __u16 pos_y_1 = 220;
    __u16 color_1 = displayRGB(255,255,255);

    const char* text2 = "RPM";
    __u8 size_2 = 2;
    __u8 font_2 = 1;
    __u16 pos_x_2 = tft.width()/2;
    __u16 pos_y_2 = tft.height();
    __u16 color_2 = displayRGB(255,255,255);

    if (!clear){
        writeBottomCenterText(text1,size_1,font_1,color_1,pos_x_1,pos_y_1);
        writeBottomCenterText(text2,size_2,font_2,color_2,pos_x_2,pos_y_2);
    }
    else{
        writeBottomCenterText(text1,size_1,font_1,bg_color,pos_x_1,pos_y_1);
        writeBottomCenterText(text2,size_2,font_2,bg_color,pos_x_2,pos_y_2);
    }
    return 1;
}

__u8 screen2(bool clear){
    
    const char* text1 = "Test 2";
    __u8 size_1 = 7;
    __u8 font_1 = 1;
    __u16 pos_x_1 = tft.width()/2;
    __u16 pos_y_1 = 220;
    __u16 color_1 = displayRGB(255,255,255);
 
    if (!clear){
        writeBottomCenterText(text1,size_1,font_1,color_1,pos_x_1,pos_y_1);
    }
    else{
        writeBottomCenterText(text1,size_1,font_1,bg_color,pos_x_1,pos_y_1);
    }
 

return 2;
}
__u8 screen3(bool clear){

    const char* text1 = "AAAAA";
    __u8 size_1 = 7;
    __u8 font_1 = 1;
    __u16 pos_x_1 = tft.width()/2;
    __u16 pos_y_1 = 220;
    __u16 color_1 = displayRGB(255,255,255);
 
    if (!clear){
        writeBottomCenterText(text1,size_1,font_1,color_1,pos_x_1,pos_y_1);
    }
    else{
        writeBottomCenterText(text1,size_1,font_1,bg_color,pos_x_1,pos_y_1);
    }

return 3;
}


void
 switchScreen(bool direction, __u16 bg_color){
    switch (actual_screen){
        case 1:
            screen1(1);
            break;
        case 2:
            screen2(1);
            break;
        case 3:
            screen3(1);
            break;
    };

    if (direction){
        actual_screen++;
        if (actual_screen==4){
            actual_screen=1;
        }
    }
    else{
        actual_screen--;
        if (actual_screen==0){
            actual_screen=3;
        }
    }

    switch (actual_screen){
        case 1:
            screen1(0);
            break;
        case 2:
            screen2(0);
            break;
        case 3:
            screen3(0);
            break;
    };


}








// WRITE FUNCTIONS

__u16 displayRGB(__u8 r, __u8 g, __u8 b){
    __u16 red = (r >> 3) & 0x1F;   // Red: Shift by 3 to fit in 5 bits
    __u16 green = (g >> 2) & 0x3F; // Green: Shift by 2 to fit in 6 bits
    __u16 blue = (b >> 3) & 0x1F;  // Blue: Shift by 3 to fit in 5 bits
    return (blue << 11) | (green << 5) | red;
}

__u16 displayHEX(const char* hexcode) {
    if (hexcode[0] == '#') hexcode++;
    __u32 rgb = (uint32_t)strtol(hexcode, NULL, 16);
    return (((rgb >> 19) & 0x1F)) |(((rgb >> 10) & 0x3F) << 5)|(((rgb >> 3)  & 0x1F) << 11);
}

void writeCenterText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize/2;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
    
}

void writeTopCenterText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize/2;
        factor_y=(factor2*textsize)-2;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y-factor_y); // (text, x, y, font)
}

void writeBottomCenterText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize/2;
        factor_y=(factor2*textsize)-1;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(BC_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
}


void writeLeftText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= 2;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
}

void writeTopLeftText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= 2;
        factor_y=(factor2*textsize)-2;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y-factor_y); // (text, x, y, font)
}

void writeBottomLeftText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= 1;
        factor_y=(factor2*textsize)-1;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(BL_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
}


void writeRightText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize-1;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
}

void writeTopRightText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize-1;
        factor_y=(factor2*textsize)-2;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y-factor_y); // (text, x, y, font)
}

void writeBottomRightText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize-1;
        factor_y=(factor2*textsize)-1;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(BR_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
}