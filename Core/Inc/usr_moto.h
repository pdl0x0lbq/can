#ifndef __USR_moto_H__
#define __USR_moto_H__

/*******MOTO ID******/
#define MOTO1 0x01
#define MOTO2 0x02

/*****GROUP ID******/
#define GROUP0	0x00

/*****CMD CODE******/
#define READ			0x2a
#define READRET		0x2b
#define WRITE			0x1a
#define WRITERET	0x1b
#define REPORT		0x88

/*****REG ADDR****/
#define MOTOSTARTSTOP	0x00
#define SPEEDSET	0x06
#define POSFEEDBACKL 0xe9
#define POSFEEDBACKH 0xe8
#define SPEEDFEEDBACK 0xe4
#define AMPFEEDBACK	0xe2
#define CANREPORTSAMPLE	0x0c
#define CANREPORTCONTENT	0x2e

/*****START STOP******/
#define START	0x01
#define STOP	0x00

/*****MOTO ID******/
#define LEFT	0x01
#define RIGHT	0x02

/*****REPORT SWITCH*******/
#define ALL	0
#define POSITION_REPORT	1
#define CURRENT_REPORT	2

void MotoSpeedSet(unsigned char moto_id, unsigned int speed);
void MotoPositionRead(unsigned char moto_id);
void MotoCANReportSet(unsigned char moto_id, unsigned int sample_time);
#endif /* __USR_moto_H__ */
