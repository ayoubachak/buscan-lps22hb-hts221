#include "main.h"

#include "hts221.h"
#include "lps22hb.h"

// 6 TEMP
// 7 PRESSION
// 8 HUMIDI
//###################################################################
#define VL6180X	1
#define MPU9250	0
#define DYN_ANEMO_PRESS 0
//###################################################################
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
//====================================================================
//			CAN ACCEPTANCE FILTER
//====================================================================
#define USE_FILTER	1
// Can accept until 4 Standard IDs
#define ID_1	0x10
#define ID_2	0x02
#define ID_3	0x03
#define ID_4	0x04
//====================================================================
extern void systemClock_Config(void);

void (*rxCompleteCallback) (void);
void can_callback(void);

CAN_Message      rxMsg;
CAN_Message      txMsg;
long int        counter = 0;

uint8_t* aTxBuffer[2];

uint8_t rx8buffer[11];

int count;
//count= 0;
extern float magCalibration[3];

void VL6180x_Init(void);
void VL6180x_Step(void);
void send_lum_dist(void);

int status;
int new_switch_state;
int switch_state = -1;

//====================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>> MAIN <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//====================================================================
int main(void)
{
	HAL_Init();
	systemClock_Config();
    SysTick_Config(HAL_RCC_GetHCLKFreq() / 1000); //SysTick end of count event each 1ms
	uart2_Init();
	uart3_Init();
	i2c1_Init();
	int i;
	i = lps22hb_whoAmI();
	  lps22hb_setup();
	  //--------------------------------------------------------------------

	  hts221_activate();
	  i = hts221_whoAmI();
	  hts221_storeCalibration();

#if DYN_ANEMO_PRESS
    anemo_Timer1Init();
#endif

	HAL_Delay(1000); // Wait

#if VL6180X
    VL6180x_Init();
#endif

#if MPU9250
    mpu9250_InitMPU9250();
    mpu9250_CalibrateMPU9250();
#if USE_MAGNETOMETER
    mpu9250_InitAK8963(magCalibration);
#endif
    uint8_t response=0;
	response =  mpu9250_WhoAmI();
	//term_printf("%d",response);
#endif


    can_Init();
    can_SetFreq(CAN_BAUDRATE); // CAN BAUDRATE : 500 MHz -- cf Inc/config.h
#if USE_FILTER
    can_Filter_list((ID_1<<21)|(ID_2<<5) , (ID_3<<21)|(ID_4<<5) , CANStandard, 0); // Accept until 4 Standard IDs
#else
    can_Filter_disable(); // Accept everybody
#endif
    can_IrqInit();
    can_IrqSet(&can_callback);
/*
    txMsg.id=0x55;
    txMsg.data[0]=1;
    txMsg.data[1]=2;
    txMsg.len=2;
    txMsg.format=CANStandard;
    txMsg.type=CANData;

    can_Write(txMsg);*/

    // Décommenter pour utiliser ce Timer ; permet de déclencher une interruption toutes les N ms
    // tickTimer_Init(200); // period in ms

#if DYN_ANEMO_PRESS
   // TEST MOTEUR
    dxl_LED(1, LED_ON);
    HAL_Delay(500);
    dxl_LED(1, LED_OFF);
    HAL_Delay(500);

    dxl_torque(1, TORQUE_OFF);
    dxl_setOperatingMode(1, VELOCITY_MODE);
    dxl_torque(1, TORQUE_ON);
    dxl_setGoalVelocity(1, 140);
    HAL_Delay(5000);
    dxl_setGoalVelocity(1, 0);
#endif


    while (1) {

#if DYN_ANEMO_PRESS

#endif

#if VL6180X
    	/*
    	HAL_UART_Receive(&Uart2Handle, rx8buffer, 11,100);
    	    if(rx8buffer[0]=='h'){
    	    	term_printf("hello");
    	        	  }
    	 */
    //VL6180x_Step();

    send_lum_dist();

    SendEnvironmentalData();

#endif



#if MPU9250

#endif

}
	return 0;
}


//====================================================================
//			CAN CALLBACK RECEPT
//====================================================================

void can_callback(void)
{
	CAN_Message msg_rcv;
	can_Read(&msg_rcv);
	rx8buffer[0]=msg_rcv.data[0];
	if (msg_rcv.data[0] == 'h'){
		new_switch_state =!new_switch_state;
	}
}
//====================================================================
//			TIMER CALLBACK PERIOD
//====================================================================

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//term_printf("from timer interrupt\n\r");
}
//====================================================================

#if VL6180X
void VL6180x_Init(void)
{
	uint8_t id;
	State.mode = 1;

    XNUCLEO6180XA1_Init();
    HAL_Delay(500); // Wait
    // RESET
    XNUCLEO6180XA1_Reset(0);
    HAL_Delay(10);
    XNUCLEO6180XA1_Reset(1);
    HAL_Delay(1);

    HAL_Delay(10);
    VL6180x_WaitDeviceBooted(theVL6180xDev);
    id=VL6180x_Identification(theVL6180xDev);
    term_printf("id=%d, should be 180 (0xB4) \n\r", id);
    VL6180x_InitData(theVL6180xDev);

    State.InitScale=VL6180x_UpscaleGetScaling(theVL6180xDev);
    State.FilterEn=VL6180x_FilterGetState(theVL6180xDev);

     // Enable Dmax calculation only if value is displayed (to save computation power)
    VL6180x_DMaxSetState(theVL6180xDev, DMaxDispTime>0);

    switch_state=-1 ; // force what read from switch to set new working mode
    State.mode = AlrmStart;
}
//====================================================================
void VL6180x_Step(void)
{
    DISP_ExecLoopBody();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
    HAL_UART_Receive(&Uart2Handle, rx8buffer, 11,100);

    new_switch_state = XNUCLEO6180XA1_GetSwitch();
    if (new_switch_state != switch_state) {
        switch_state=new_switch_state;
        status = VL6180x_Prepare(theVL6180xDev);
        // Increase convergence time to the max (this is because proximity config of API is used)
        VL6180x_RangeSetMaxConvergenceTime(theVL6180xDev, 63);
        if (status) {
            AbortErr("ErIn");
        }
        else{
            if (switch_state == SWITCH_VAL_RANGING) {
                VL6180x_SetupGPIO1(theVL6180xDev, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_HIGH);
                VL6180x_ClearAllInterrupt(theVL6180xDev);
                State.ScaleSwapCnt=0;
                DoScalingSwap( State.InitScale);
            } else {
                 State.mode = RunAlsPoll;
                 InitAlsMode();
            }
        }
    }

    switch (State.mode) {
    case RunRangePoll:
        RangeState();
        break;

    case RunAlsPoll:
        AlsState();
        break;

    case InitErr:
        TimeStarted = g_TickCnt;
        State.mode = WaitForReset;
        break;

    case AlrmStart:
       GoToAlaramState();
       break;

    case AlrmRun:
        AlarmState();
        break;

    case FromSwitch:
        // force reading swicth as re-init selected mode
        switch_state=!XNUCLEO6180XA1_GetSwitch();
        break;

    case ScaleSwap:

        if (g_TickCnt - TimeStarted >= ScaleDispTime) {
            State.mode = RunRangePoll;
            TimeStarted=g_TickCnt; /* reset as used for --- to er display */
        }
        else
        {
        	DISP_ExecLoopBody();
        }
        break;

    default: {
    	 DISP_ExecLoopBody();
          if (g_TickCnt - TimeStarted >= 5000) {
              NVIC_SystemReset();
          }
    }
    }
}

void SendEnvironmentalData(void)
{
    static float pres_value, temp_value,hum_value;
    int32_t PressToSend=0;
    int16_t TempToSend=0;
    int16_t HumToSend=0;
    int32_t decPart, intPart;

    lps22hb_getPressure(&pres_value);
    MCR_BLUEMS_F2I_2D(pres_value, intPart, decPart);
   	PressToSend=intPart*100+decPart;

   	hts221_getTemperature(&temp_value);
   	MCR_BLUEMS_F2I_1D(temp_value, intPart, decPart);
   	TempToSend = intPart*10+decPart;

   	hts221_getHumidity(&hum_value);
   	MCR_BLUEMS_F2I_1D(hum_value, intPart, decPart);
   	HumToSend = intPart*10+decPart;

   	CAN_Message msg_test;
	msg_test.id =0x06;
	msg_test.data[0] = (uint8_t)((TempToSend & 0xFF) );
	msg_test.data[1] = (uint8_t)(TempToSend >> 8);
	msg_test.len=2;			// Nombre d'octets à envoyer
	msg_test.format=CANStandard;
	msg_test.type=CANData;
	can_Write(msg_test);



	CAN_Message msg_test2;
	msg_test2.id =0x07;
	msg_test2.data[0] = (PressToSend & 0xFF000000) >> 24;
	msg_test2.data[1] = (PressToSend & 0x00FF0000) >> 16;
	msg_test2.data[2] = (PressToSend & 0x0000FF00) >> 8;
	msg_test2.data[3] = (PressToSend & 0x000000FF);
	msg_test2.len=4;			// Nombre d'octets à envoyer
	msg_test2.format=CANStandard;
	msg_test2.type=CANData;
	can_Write(msg_test2);


	CAN_Message msg_test3;
		msg_test3.id =0x08;
		msg_test3.data[0] = (uint8_t)((HumToSend & 0xFF) );
		msg_test3.data[1] = (uint8_t)(HumToSend >> 8);
		msg_test3.len=2;			// Nombre d'octets à envoyer
		msg_test3.format=CANStandard;
		msg_test3.type=CANData;
		can_Write(msg_test3);
}


void send_lum_dist(void)
{
    DISP_ExecLoopBody();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);



    if (new_switch_state != switch_state) {
        switch_state=new_switch_state;
        status = VL6180x_Prepare(theVL6180xDev);
        // Increase convergence time to the max (this is because proximity config of API is used)
        VL6180x_RangeSetMaxConvergenceTime(theVL6180xDev, 63);
        if (status) {
            AbortErr("ErIn");
        }
        else{
            if (switch_state == SWITCH_VAL_RANGING) {
                VL6180x_SetupGPIO1(theVL6180xDev, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_HIGH);
                VL6180x_ClearAllInterrupt(theVL6180xDev);
                State.ScaleSwapCnt=0;
                DoScalingSwap( State.InitScale);
            } else {
                 State.mode = RunAlsPoll;
                 InitAlsMode();
            }
        }
    }

    switch (State.mode) {
    case RunRangePoll:
        RangeState();
        break;

    case RunAlsPoll:
        AlsState();
        send_lum_can();
        break;

    case InitErr:
        TimeStarted = g_TickCnt;
        State.mode = WaitForReset;
        break;

    case AlrmStart:
       GoToAlaramState();
       break;

    case AlrmRun:
        AlarmState();
        break;

    case FromSwitch:
        // force reading swicth as re-init selected mode
        switch_state=!XNUCLEO6180XA1_GetSwitch();
        break;

    case ScaleSwap:

        if (g_TickCnt - TimeStarted >= ScaleDispTime) {
            State.mode = RunRangePoll;
            TimeStarted=g_TickCnt; /* reset as used for --- to er display */
        }
        else
        {
        	DISP_ExecLoopBody();
        }
        break;

    default: {
    	 DISP_ExecLoopBody();
          if (g_TickCnt - TimeStarted >= 5000) {
              NVIC_SystemReset();
          }
    }
    }
}

#endif
//====================================================================

