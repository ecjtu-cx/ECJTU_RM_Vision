#ifndef SERIALPORT_H
#define SERIALPORT_H

// 字节数为4的结构体
typedef union {
  float f;
  unsigned char c[4];
} float2uchar;

// 用于保存目标相关角度和距离信息及瞄准情况
typedef struct {
  float2uchar pitch_angle; // 偏航角
  float2uchar yaw_angle;   // 俯仰角
  float2uchar dis;         // 目标距离
  int isSwitched;          // 目标是否发生切换
  int isFindTarget; // 当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
  int isSpinning;   // 目标是否处于陀螺状态
  int ismiddle; // 设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，默认置0
} VisionSendData;

#endif // SERIALPORT_H