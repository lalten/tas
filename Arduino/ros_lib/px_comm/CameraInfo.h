#ifndef _ROS_px_comm_CameraInfo_h
#define _ROS_px_comm_CameraInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace px_comm
{

  class CameraInfo : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* camera_model;
      const char* camera_name;
      const char* camera_type;
      uint32_t image_width;
      uint32_t image_height;
      uint8_t D_length;
      float st_D;
      float * D;
      uint8_t P_length;
      float st_P;
      float * P;
      uint8_t M_length;
      float st_M;
      float * M;
      geometry_msgs::Pose pose;

    CameraInfo():
      header(),
      camera_model(""),
      camera_name(""),
      camera_type(""),
      image_width(0),
      image_height(0),
      D_length(0), D(NULL),
      P_length(0), P(NULL),
      M_length(0), M(NULL),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_camera_model = strlen(this->camera_model);
      memcpy(outbuffer + offset, &length_camera_model, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->camera_model, length_camera_model);
      offset += length_camera_model;
      uint32_t length_camera_name = strlen(this->camera_name);
      memcpy(outbuffer + offset, &length_camera_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->camera_name, length_camera_name);
      offset += length_camera_name;
      uint32_t length_camera_type = strlen(this->camera_type);
      memcpy(outbuffer + offset, &length_camera_type, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->camera_type, length_camera_type);
      offset += length_camera_type;
      *(outbuffer + offset + 0) = (this->image_width >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->image_width >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->image_width >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->image_width >> (8 * 3)) & 0xFF;
      offset += sizeof(this->image_width);
      *(outbuffer + offset + 0) = (this->image_height >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->image_height >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->image_height >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->image_height >> (8 * 3)) & 0xFF;
      offset += sizeof(this->image_height);
      *(outbuffer + offset++) = D_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < D_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->D[i]);
      }
      *(outbuffer + offset++) = P_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < P_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->P[i]);
      }
      *(outbuffer + offset++) = M_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < M_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->M[i]);
      }
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_camera_model;
      memcpy(&length_camera_model, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_camera_model; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_camera_model-1]=0;
      this->camera_model = (char *)(inbuffer + offset-1);
      offset += length_camera_model;
      uint32_t length_camera_name;
      memcpy(&length_camera_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_camera_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_camera_name-1]=0;
      this->camera_name = (char *)(inbuffer + offset-1);
      offset += length_camera_name;
      uint32_t length_camera_type;
      memcpy(&length_camera_type, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_camera_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_camera_type-1]=0;
      this->camera_type = (char *)(inbuffer + offset-1);
      offset += length_camera_type;
      this->image_width =  ((uint32_t) (*(inbuffer + offset)));
      this->image_width |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->image_width |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->image_width |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->image_width);
      this->image_height =  ((uint32_t) (*(inbuffer + offset)));
      this->image_height |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->image_height |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->image_height |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->image_height);
      uint8_t D_lengthT = *(inbuffer + offset++);
      if(D_lengthT > D_length)
        this->D = (float*)realloc(this->D, D_lengthT * sizeof(float));
      offset += 3;
      D_length = D_lengthT;
      for( uint8_t i = 0; i < D_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_D));
        memcpy( &(this->D[i]), &(this->st_D), sizeof(float));
      }
      uint8_t P_lengthT = *(inbuffer + offset++);
      if(P_lengthT > P_length)
        this->P = (float*)realloc(this->P, P_lengthT * sizeof(float));
      offset += 3;
      P_length = P_lengthT;
      for( uint8_t i = 0; i < P_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_P));
        memcpy( &(this->P[i]), &(this->st_P), sizeof(float));
      }
      uint8_t M_lengthT = *(inbuffer + offset++);
      if(M_lengthT > M_length)
        this->M = (float*)realloc(this->M, M_lengthT * sizeof(float));
      offset += 3;
      M_length = M_lengthT;
      for( uint8_t i = 0; i < M_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_M));
        memcpy( &(this->M[i]), &(this->st_M), sizeof(float));
      }
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "px_comm/CameraInfo"; };
    const char * getMD5(){ return "014513fdee9cefabe3cec97bca5e5c57"; };

  };

}
#endif