#include "usr_moto.h"
#include "can.h"
#include "main.h"

unsigned long int EncoderValues_L = 0, EncoderValues_R = 0;
unsigned char can_data[8] = {0};

/**********
	@bief moto speed set.
	@retval	void
	@param moto_id: moto id, speed:moto speed.
**/
void MotoSpeedSet(unsigned char moto_id, unsigned int Mspeed){
#ifdef MOTO_DIRECTION_F
    Mspeed = 0- Mspeed;
		can_data[0] = GROUP0;
		can_data[1] = WRITE;
		can_data[2] = MOTOSTARTSTOP;
		can_data[3] = 0x00;
		can_data[4] = START;
		can_data[5] = SPEEDSET;
		can_data[6] = (Mspeed >> 8) & 0xff;
		can_data[7] = Mspeed & 0xff;
		usr_can_Tx(moto_id, can_data, 8);
#elif MOTO_DIRECTION_B
    can_data[0] = GROUP0;
		can_data[1] = WRITE;
		can_data[2] = MOTOSTARTSTOP;
		can_data[3] = 0x00;
		can_data[4] = START;
		can_data[5] = SPEEDSET;
		can_data[6] = (Mspeed >> 8) & 0xff;
		can_data[7] = Mspeed & 0xff;
		usr_can_Tx(moto_id, can_data, 8);
#endif
}

/**********
	@bief moto speed set.
	@retval	void.
	@param moto_id: moto id, speed:moto speed.
**/
void MotoCANReportSet(unsigned char moto_id, unsigned int sample_time){
    can_data[0] = GROUP0;
    can_data[1] = WRITE;
    can_data[2] = CANREPORTCONTENT;
    can_data[3] = 0x00;
    can_data[4] = ALL;
    can_data[5] = CANREPORTSAMPLE;
    can_data[6] = (sample_time >> 8) & 0xff;
    can_data[7] = sample_time & 0xff;
    usr_can_Tx(moto_id, can_data, 8);
}

/**********
	@bief read moto position.
	@retval	void
	@param moto_id: moto id.
**/
void MotoPositionRead(unsigned char moto_id){
    can_data[0] = GROUP0;
    can_data[1] = READ;
    can_data[2] = POSFEEDBACKH;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = POSFEEDBACKL;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    usr_can_Tx(moto_id, can_data, 8);
}

