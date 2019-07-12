#ifndef _PIXY2VIDEO_H
#define _PIXY2VIDEO_H

#define VIDEO_REQUEST_GET_RGB   0x70

template <class LinkType> class TPixy2;

template <class LinkType> class Pixy2Video
{
public:
  Pixy2Video(TPixy2<LinkType> *pixy)
  {
    m_pixy = pixy;
  }	  
 
  int8_t getRGB(uint16_t x, uint16_t y, uint8_t *r, uint8_t *g, uint8_t *b, bool saturate=true);
  
private:
  TPixy2<LinkType> *m_pixy;
  
};

template <class LinkType> int8_t Pixy2Video<LinkType>::getRGB(uint16_t x, uint16_t y, uint8_t *r, uint8_t *g, uint8_t *b, bool saturate)
{
  while(1)
  {
    *(int16_t *)(m_pixy->m_bufPayload + 0) = x;
    *(int16_t *)(m_pixy->m_bufPayload + 2) = y;
    *(m_pixy->m_bufPayload + 4) = saturate;
    m_pixy->m_length = 5;
    m_pixy->m_type = VIDEO_REQUEST_GET_RGB;
    m_pixy->sendPacket();
    if (m_pixy->recvPacket()==0)
    {
      if (m_pixy->m_type==PIXY_TYPE_RESPONSE_RESULT && m_pixy->m_length==4)
      {
	    *b = *(m_pixy->m_buf+0);
	    *g = *(m_pixy->m_buf+1);
	    *r = *(m_pixy->m_buf+2);
        return 0;
	  }
	  // deal with program changing 
      else if (m_pixy->m_type==PIXY_TYPE_RESPONSE_ERROR && (int8_t)m_pixy->m_buf[0]==PIXY_RESULT_PROG_CHANGING)
      {
        delayMicroseconds(500); // don't be a drag
        continue;
      }
    }
    return PIXY_RESULT_ERROR;     
  }
}

#endif