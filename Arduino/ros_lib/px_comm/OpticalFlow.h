#ifndef _ROS_px_comm_OpticalFlow_h
#define _ROS_px_comm_OpticalFlow_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace px_comm
{

  class OpticalFlow : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float ground_distance;
      int16_t flow_x;
      int16_t flow_y;
      float velocity_x_raw;
      float velocity_y_raw;
      float velocity_x;
      float velocity_y;
      uint8_t quality;

    OpticalFlow():
      header(),
      ground_distance(0),
      flow_x(0),
      flow_y(0),
      velocity_x_raw(0),
      velocity_y_raw(0),
      velocity_x(0),
      velocity_y(0),
      quality(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_ground_distance;
      u_ground_distance.real = this->ground_distance;
      *(outbuffer + offset + 0) = (u_ground_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ground_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ground_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ground_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ground_distance);
      union {
        int16_t real;
        uint16_t base;
      } u_flow_x;
      u_flow_x.real = this->flow_x;
      *(outbuffer + offset + 0) = (u_flow_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_flow_x.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->flow_x);
      union {
        int16_t real;
        uint16_t base;
      } u_flow_y;
      u_flow_y.real = this->flow_y;
      *(outbuffer + offset + 0) = (u_flow_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_flow_y.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->flow_y);
      union {
        float real;
        uint32_t base;
      } u_velocity_x_raw;
      u_velocity_x_raw.real = this->velocity_x_raw;
      *(outbuffer + offset + 0) = (u_velocity_x_raw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_x_raw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_x_raw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_x_raw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_x_raw);
      union {
        float real;
        uint32_t base;
      } u_velocity_y_raw;
      u_velocity_y_raw.real = this->velocity_y_raw;
      *(outbuffer + offset + 0) = (u_velocity_y_raw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_y_raw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_y_raw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_y_raw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_y_raw);
      union {
        float real;
        uint32_t base;
      } u_velocity_x;
      u_velocity_x.real = this->velocity_x;
      *(outbuffer + offset + 0) = (u_velocity_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_x);
      union {
        float real;
        uint32_t base;
      } u_velocity_y;
      u_velocity_y.real = this->velocity_y;
      *(outbuffer + offset + 0) = (u_velocity_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_y);
      *(outbuffer + offset + 0) = (this->quality >> (8 * 0)) & 0xFF;
      offset += sizeof(this->quality);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_ground_distance;
      u_ground_distance.base = 0;
      u_ground_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ground_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ground_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ground_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ground_distance = u_ground_distance.real;
      offset += sizeof(this->ground_distance);
      union {
        int16_t real;
        uint16_t base;
      } u_flow_x;
      u_flow_x.base = 0;
      u_flow_x.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_flow_x.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->flow_x = u_flow_x.real;
      offset += sizeof(this->flow_x);
      union {
        int16_t real;
        uint16_t base;
      } u_flow_y;
      u_flow_y.base = 0;
      u_flow_y.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_flow_y.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->flow_y = u_flow_y.real;
      offset += sizeof(this->flow_y);
      union {
        float real;
        uint32_t base;
      } u_velocity_x_raw;
      u_velocity_x_raw.base = 0;
      u_velocity_x_raw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_x_raw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_x_raw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_x_raw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_x_raw = u_velocity_x_raw.real;
      offset += sizeof(this->velocity_x_raw);
      union {
        float real;
        uint32_t base;
      } u_velocity_y_raw;
      u_velocity_y_raw.base = 0;
      u_velocity_y_raw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_y_raw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_y_raw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_y_raw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_y_raw = u_velocity_y_raw.real;
      offset += sizeof(this->velocity_y_raw);
      union {
        float real;
        uint32_t base;
      } u_velocity_x;
      u_velocity_x.base = 0;
      u_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_x = u_velocity_x.real;
      offset += sizeof(this->velocity_x);
      union {
        float real;
        uint32_t base;
      } u_velocity_y;
      u_velocity_y.base = 0;
      u_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_y = u_velocity_y.real;
      offset += sizeof(this->velocity_y);
      this->quality =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->quality);
     return offset;
    }

    const char * getType(){ return "px_comm/OpticalFlow"; };
    const char * getMD5(){ return "7e77c3ff7bd4b447cb8601535dcaa7b3"; };

  };

}
#endif