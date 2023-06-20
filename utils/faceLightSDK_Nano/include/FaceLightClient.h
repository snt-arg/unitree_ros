#ifndef FACELIGHTCLIENT_H
#define FACELIGHTCLIENT_H

#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include "LEDPixel.h"

class FaceLightClient{
public:
    FaceLightClient();
    ~FaceLightClient();

    void sendCmd();
    void setLedColor(uint32_t id, const uint8_t *rgb);
    void setAllLed(const uint8_t *rgb);

    const uint8_t red[3]   = {255, 0, 0};
    const uint8_t green[3] = {0, 255, 0};
    const uint8_t blue[3]  = {0, 0, 255};
    const uint8_t yellow[3]= {255, 255, 0};
    const uint8_t black[3] = {0, 0, 0};
    const uint8_t white[3] = {255, 255, 255};
private:
    // bool checkColorAvailable(uint8_t *color);
    void _setUDP();

    LEDPixel _led;
    uint32_t _ledNum;
    uint32_t _cmdLength;
    uint8_t *_stripCmd;

    /* UDP */
    int _sockfd;
    int _udpAddrSize;
    int _on = -1;
    struct sockaddr_in _saddr;
    int _ret;
};

#endif  // FACELIGHTCLIENT_H