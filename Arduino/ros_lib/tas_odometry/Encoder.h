#ifndef _ROS_tas_odometry_Encoder_h
#define _ROS_tas_odometry_Encoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tas_odometry
{

  class Encoder : public ros::Msg
  {
    public:
      int32_t encoder_ticks;
      uint32_t duration;

    Encoder():
      encoder_ticks(0),
      duration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_encoder_ticks;
      u_encoder_ticks.real = this->encoder_ticks;
      *(outbuffer + offset + 0) = (u_encoder_ticks.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder_ticks.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_encoder_ticks.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_encoder_ticks.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoder_ticks);
      *(outbuffer + offset + 0) = (this->duration >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->duration >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->duration >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->duration >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_encoder_ticks;
      u_encoder_ticks.base = 0;
      u_encoder_ticks.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder_ticks.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_encoder_ticks.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_encoder_ticks.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->encoder_ticks = u_encoder_ticks.real;
      offset += sizeof(this->encoder_ticks);
      this->duration =  ((uint32_t) (*(inbuffer + offset)));
      this->duration |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->duration |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->duration |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->duration);
     return offset;
    }

    const char * getType(){ return "tas_odometry/Encoder"; };
    const char * getMD5(){ return "bd47b8eb0e5fbce41a6c91323c6d0c40"; };

  };

}
#endif