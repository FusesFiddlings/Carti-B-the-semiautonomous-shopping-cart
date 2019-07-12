#ifndef _PIXY2I2C_H
#define _PIXY2I2C_H

#include "TPixy2.h"

#define PIXY_I2C_DEFAULT_ADDR           0x54  
#define PIXY_I2C_MAX_SEND               16 // don't send any more than 16 bytes at a time

class Link2I2C
{
public:
  int8_t open(uint32_t arg) // take I2C address as argument to open
  {
    if (arg==PIXY_DEFAULT_ARGVAL)
      m_addr = PIXY_I2C_DEFAULT_ADDR;
    else
      m_addr = arg;
    Wire.begin();
	return 0;
  }
	
  void close()
  {
  }
    
  int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs=NULL)
  {
    uint8_t i, j, n;
    if (cs)
      *cs = 0;
    for (i=0; i<len; i+=n)
    {
      // n is the number read -- it most likely won't be equal to len
      n = Wire.requestFrom((uint8_t)m_addr, (uint8_t)(len-i));
      for (j=0; j<n; j++)
      {		  
        buf[j+i] = Wire.read();
        if (cs)
          *cs += buf[j+i];
      }	  
    }
    return len;
  }
    
  int16_t send(uint8_t *buf, uint8_t len)
  {
    int8_t i, packet;
	for (i=0; i<len; i+=PIXY_I2C_MAX_SEND)
    {
      if (len-i<PIXY_I2C_MAX_SEND)
		  packet = len-i;
	  else 
		  packet = PIXY_I2C_MAX_SEND;
      Wire.beginTransmission(m_addr);
      Wire.write(buf+i, packet);
      Wire.endTransmission();
    }
    return len;
  }
	
private:
  uint8_t m_addr;	
};


typedef TPixy2<Link2I2C> Pixy2I2C;


#endif
