#include "remote.h"

#include "usart2.h"
#include "led.h"
#include "iwdg.h"


#define    REMOTE_LOST_TIME    ((uint32_t)50)   //50ms


RC_Ctl_t RC_Ctl;


portTickType ulRemoteLostTime = 0;



uint32_t REMOTE_ulGetLostTime( void )
{
	/* */
	return  ulRemoteLostTime;
}



void REMOTE_vResetData( void )
{
	/* Channel 0, 1, 2, 3 */
	RC_Ctl.rc.ch0 = RC_CH_VALUE_OFFSET;
	RC_Ctl.rc.ch1 = RC_CH_VALUE_OFFSET;
	RC_Ctl.rc.ch2 = RC_CH_VALUE_OFFSET;
	RC_Ctl.rc.ch3 = RC_CH_VALUE_OFFSET;

	/* Switch left, right */
	RC_Ctl.rc.s1  = RC_SW_MID;
	RC_Ctl.rc.s2  = RC_SW_MID;

	/* Mouse axis: X, Y, Z */
	RC_Ctl.mouse.x = 0;
	RC_Ctl.mouse.y = 0;
	RC_Ctl.mouse.z = 0;

	/* Mouse Left, Right Is Press ? */
	RC_Ctl.mouse.press_l = 0;
	RC_Ctl.mouse.press_r = 0;

	/* KeyBoard value */
	RC_Ctl.key.v = 0;
}



void REMOTE_vUpdateLostTime( void )
{

	ulRemoteLostTime = xTaskGetTickCount( ) + REMOTE_LOST_TIME;
}



bool_t REMOTE_IfDataError( void )
{
	if ( (RC_Ctl.rc.s1 != RC_SW_UP && RC_Ctl.rc.s1 != RC_SW_MID && RC_Ctl.rc.s1 != RC_SW_DOWN)
		|| (RC_Ctl.rc.s2 != RC_SW_UP && RC_Ctl.rc.s2 != RC_SW_MID && RC_Ctl.rc.s2 != RC_SW_DOWN)
		|| (RC_Ctl.rc.ch0 > RC_CH_VALUE_MAX || RC_Ctl.rc.ch0 < RC_CH_VALUE_MIN)
		|| (RC_Ctl.rc.ch1 > RC_CH_VALUE_MAX || RC_Ctl.rc.ch1 < RC_CH_VALUE_MIN)
		|| (RC_Ctl.rc.ch2 > RC_CH_VALUE_MAX || RC_Ctl.rc.ch2 < RC_CH_VALUE_MIN)
		|| (RC_Ctl.rc.ch3 > RC_CH_VALUE_MAX || RC_Ctl.rc.ch3 < RC_CH_VALUE_MIN) )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


void REMOTE_vReadData(char *pucRxBuffer )
{
	if(pucRxBuffer == NULL)
	{
	return;
	}

	/* Channel 0, 1, 2, 3 */
	RC_Ctl.rc.ch0 = (  pucRxBuffer[0]       | (pucRxBuffer[1] << 8 ) ) & 0x07ff;
	RC_Ctl.rc.ch1 = ( (pucRxBuffer[1] >> 3) | (pucRxBuffer[2] << 5 ) ) & 0x07ff;
	RC_Ctl.rc.ch2 = ( (pucRxBuffer[2] >> 6) | (pucRxBuffer[3] << 2 ) | (pucRxBuffer[4] << 10) ) & 0x07ff;
	RC_Ctl.rc.ch3 = ( (pucRxBuffer[4] >> 1) | (pucRxBuffer[5] << 7 ) ) & 0x07ff;

	

	/* Switch left, right */
	RC_Ctl.rc.s1  = ( (pucRxBuffer[5] >> 4) & 0x000C ) >> 2;
	RC_Ctl.rc.s2  = ( (pucRxBuffer[5] >> 4) & 0x0003 );

	/* Mouse axis: X, Y, Z */
	RC_Ctl.mouse.x = pucRxBuffer[6]  | (pucRxBuffer[7 ] << 8);
	RC_Ctl.mouse.y = pucRxBuffer[8]  | (pucRxBuffer[9 ] << 8);
	RC_Ctl.mouse.z = pucRxBuffer[10] | (pucRxBuffer[11] << 8);

	/* Mouse Left, Right Is Press ? */
	RC_Ctl.mouse.press_l = pucRxBuffer[12];
	RC_Ctl.mouse.press_r = pucRxBuffer[13];

	/* KeyBoard value */
	RC_Ctl.key.v = pucRxBuffer[14] | (pucRxBuffer[15] << 8);
	

}
