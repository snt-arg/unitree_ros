#ifndef LEDPIXEL_H
#define LEDPIXEL_H

#include <stdint.h>

class LEDPixel{
public:
    LEDPixel(){}
    ~LEDPixel(){}
    void setPixelCmd(const uint8_t *rgb, uint8_t *cmd);
    void getPixelColor(uint8_t *rgb);
private:
    uint8_t _rgb[3] = {255, 255, 255};
    const uint8_t _rgb2grb[3] = {1, 0, 2};
};

#endif  // LEDPIXEL_H