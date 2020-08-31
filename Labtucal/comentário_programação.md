
![logo](https://spacelab.ufsc.br/wp-content/uploads/2019/07/g3920-3-2.png)
# `MPMA`

Cada ariquvo que compõe a programacao do sdk será comentado aqui para aprendizado e documentação,lembro que esses comentarios não serão feitos pelo criador da programação.
---
## `Arquivo principal`
--- 
### *_Acq_mpma.c_*
```
#include <stdio.h>
#include <stdlib.h>
#include <sleep.h>

#include <xparameters.h>
#include <xsysmon.h>
#include <xgpiops.h>
#include <xtime_l.h>
#include <xil_cache.h>

#include <communicator.h>
#include <fsatcomm.h>
#include <ds3231.h>
#include <max31855.h>
#include <ads1248.h>
#include <ltc2943.h>
#include <vnh5180.h>
#include <ad5726.h>
#include <pid.h>
#include <util.h>

#include <m1.h>
#include <mpma.h>
#include <acq.h>

void ExecutePid0(Pid *PidController, VNH5180 *Vnh, int ProcessValue);
void ExecutePid1(Pid *PidController, VNH5180 *Vnh, int ProcessValue);

void ExecutePid0(Pid *PidController, VNH5180 *Vnh, int ProcessValue)
{
	int DutyCycle;

	if (ProcessValue < PID0_SET_POINT) {
		return;
	}

	DutyCycle = -Pid_Control(PID0_SET_POINT, ProcessValue, PidController);

	if (DutyCycle > 100) {
		DutyCycle = 100;
	} else if (DutyCycle < 0) {
		DutyCycle = 0;
	}

	VNH5180_SetDC(Vnh, abs(DutyCycle));
}

void ExecutePid1(Pid *PidController, VNH5180 *Vnh, int ProcessValue)
{
	int DutyCycle;

	if (ProcessValue < PID1_SET_POINT) {
		return;
	}

	DutyCycle = -Pid_Control(PID1_SET_POINT, ProcessValue, PidController);

	if (DutyCycle > 100) {
		DutyCycle = 100;
	} else if (DutyCycle < -100) {
		DutyCycle = -100;
	}

	if (DutyCycle > 0) {
		VNH5180_SetMode(Vnh, VNH5180_CCW);
	} else {
		VNH5180_SetMode(Vnh, VNH5180_CW);
	}

	VNH5180_SetDC(Vnh, abs(DutyCycle));
}

int main()
{
	const XTime END_TIME = ((XTime) N_SECS) * COUNTS_PER_SECOND,
			T_PLATEAU1 = ((XTime) N_SECS_PLATEAU1) * COUNTS_PER_SECOND;
	const int TUBE1_MON_THERMOS[] = { TUBE1_MON_THERMO0, TUBE1_MON_THERMO1,
			TUBE1_MON_THERMO2, TUBE1_MON_THERMO3 };
	int TempThermos[TUBE1_N_MON_THERMO];
	FATFS FatFs;
	FIL File;
	XGpioPs Gpio;
	DS3231 Ds;
	MAX31855 Max;
	ADS1248 Ads;
	LTC2943 Ltc;
	XSysMon SysMon;
	VNH5180 Vnh[N_VNH5180];
	AD5726 Ad;
	FsatComm FsatCommunicator;
	Pid PidController[2];
	Communicator Comm;
	Communicator_TxPacket Packet[N_PACKET];
	Communicator_RxPacket *RxPacket = NULL;
	XTime T, T0, TUg;
	int i, Plateau1 = 0, Ug = 0, PowerOn = GPIO_POWER_ON_DEASSERT;

	printf("Microgravity experiment MPM-A\n\r");

	/*
	 * Initialize all peripherals and the FAT file system.
	 */
	Fs_Init(&FatFs, &File);
	PeripheralsInit(&Comm, &FsatCommunicator, &Gpio, &Ds, &Max, &Ads, &Ltc,
			&SysMon, Vnh, &Ad, &PidController[0]);

	/*
	 * Sample and transmit data through the Communicator while waiting for
	 * lift off.
	 */
	while (1) {
		Sample(Packet, &FsatCommunicator, &Gpio, &Max, &Ads, &Ltc, &SysMon,
				&PowerOn, &Ug);
		Transmit(&Comm, Packet);
		ExecutePid0(&PidController[0], &Vnh[0],
				Communicator_TxUnpack(&Packet[PID0_THERMOCOUPLE_CHANNEL]));
		ExecutePid1(&PidController[1], &Vnh[1],
				MAX31855_Raw2Celsius(Communicator_TxUnpack(&Packet[PID1_THERMOCOUPLE_CHANNELP])) -
				MAX31855_Raw2Celsius(Communicator_TxUnpack(&Packet[PID1_THERMOCOUPLE_CHANNELN])));

		/*
		 * Try to receive the whole packet and execute the received command.
		 */
		for (i = 0; i < COMMUNICATOR_RX_N_BYTES; i++) {
			Communicator_Recv(&Comm, &RxPacket);
		}

		if (RxPacket != NULL) {
			ExecCmd(RxPacket, &Vnh[0], &Ad, &Gpio, &FsatCommunicator, &PowerOn);
			RxPacket = NULL;
		}

		/*
		 * Wait for lift off.
		 */
		if (XGpioPs_ReadPin(&Gpio, GPIO_LIFT_OFF_PIN) == GPIO_LIFT_OFF_ASSERT) {
			break;
		}

		usleep(150000);
	}

	/*
	 * Enable FSatComm and sample the flight timer on lift off.
	 */
	XTime_GetTime(&T0);
	T = T0;
	FsatComm_Enable(&FsatCommunicator);

	/*
	 * Enable VNH5180 0 on lift off.
	 */
	VNH5180_SetMode(&Vnh[0], VNH5180_CCW);
	AD5726_SetVoltage(&Ad, VNH5180_DAC_CHANNEL, VNH5180_LEVEL_MAX);

	/*
	 * TODO: There is a possible overflow here if the lift off takes too long.
	 */
	while (T < (T0 + END_TIME)) {
		Sample(Packet, &FsatCommunicator, &Gpio, &Max, &Ads, &Ltc, &SysMon,
				&PowerOn, &Ug);
		Transmit(&Comm, Packet);
		Fs_Write(Packet, &FatFs, &File);
		ExecutePid0(&PidController[0], &Vnh[0],
				Communicator_TxUnpack(&Packet[PID0_THERMOCOUPLE_CHANNEL]));
		ExecutePid1(&PidController[1], &Vnh[1],
				MAX31855_Raw2Celsius(Communicator_TxUnpack(&Packet[PID1_THERMOCOUPLE_CHANNELP])) -
				MAX31855_Raw2Celsius(Communicator_TxUnpack(&Packet[PID1_THERMOCOUPLE_CHANNELN])));

		/*
		 * Wait for uG, plateau 0.
		 */
		if ((XGpioPs_ReadPin(&Gpio, GPIO_UG_PIN) == GPIO_UG_ASSERT) &&
				(Ug == 0)) {
			Ug = 1;
			AD5726_SetVoltage(&Ad, AD5726_TUBE, AD5726_LEVEL_PLATEAU0);
			XGpioPs_WritePin(&Gpio, GPIO_POWER_ON_PIN, GPIO_POWER_ON_ASSERT);
			PowerOn = GPIO_POWER_ON_ASSERT;
			XTime_GetTime(&TUg);
		}

		/*
		 * Plateau 1.
		 */
		if ((Plateau1 == 0) && (Ug == 1) && (T > (TUg + T_PLATEAU1))) {
			Plateau1 = 1;
			AD5726_SetVoltage(&Ad, AD5726_TUBE, AD5726_LEVEL_PLATEAU1);
		}

		/*
		 * Monitor tubes temperature.
		 */
		for (i = 0; i < TUBE1_N_MON_THERMO; i++) {
			TempThermos[i] = Communicator_TxUnpack(&Packet[TUBE1_MON_THERMOS[i]]);
		}

		MonitorTube(&Ad, TempThermos, TUBE1_N_MON_THERMO, AD5726_TUBE,
				TUBE1_N_MON_THERMO_MAX, TUBE1_MON_TEMP_MAX);

		usleep(150000);

		XTime_GetTime(&T);
	}

	/*
	 * SD card file system cleanup.
	 */
	Fs_Terminate(&FatFs, &File);

	/*
	 * Shut down the experiment.
	 */
	XGpioPs_WritePin(&Gpio, GPIO_POWER_ON_PIN, GPIO_POWER_ON_DEASSERT);
	XGpioPs_WritePin(&Gpio, GPIO_KILL_PIN, GPIO_KILL_ASSERT);

	return XST_SUCCESS;
}

```
Este é o arquivo que contém  a função principál da programação.
No setup ele cria as variáveis globais e configura o comando PID para o peltier.Depois inicia inicia um loop infinito para enviar e receber os dados, via interface UART.Ele sairá do loop quando receber o sinal de lift off.
Criado um loop para esperar ate o tempo limite configurado
Aguarda o sinal de micro g,platô 0,caso acionado envia as tensões para resistências e o peltier.
No segundo platô envia uma nova potencia para as resistências.
Na sequencia um loop armazena os valores recebidos dos termopares e os armazena em uma variável do tipo struct.
## `Aquisição de dados`
---
### *_Acq.c_*

```
#include <stdio.h>

#include <xtime_l.h>

#include <acq.h>
#ifdef MPMA
#include <mpma.h>
#endif
#ifdef MPMB
#include <mpmb.h>
#endif

#include <m1.h>

void ExecCmd(Communicator_RxPacket *Packet, VNH5180 *Vnh, AD5726 *Ad,
		XGpioPs *Gpio, FsatComm *FsatCommunicator, int *PowerOn)
{
#ifdef MPMA
	const int AD5726_TUBES[] = { AD5726_TUBE };
#endif
#ifdef MPMB
	const int AD5726_TUBES[] = {AD5726_TUBE0, AD5726_TUBE1};
#endif
	u32 Data;
	int i;

	Data = Communicator_Unpack(Packet);

	switch (Packet->Command) {
	case CMD_PELTIER1_SWITCH:
		if (Data == SWITCH_ON) {
			VNH5180_SetMode(&Vnh[0], VNH5180_CCW);
		} else if (Data == SWITCH_OFF) {
			VNH5180_SetMode(&Vnh[0], VNH5180_BRAKE_GND);
		}

		break;
	case CMD_PELTIER1_SET_POWER:
		VNH5180_SetDC(&Vnh[0], Data);

		break;
	case CMD_PELTIER2_SWITCH:
		if (Data == SWITCH_ON) {
			VNH5180_SetMode(&Vnh[1], VNH5180_CCW);
		} else if (Data == SWITCH_OFF) {
			VNH5180_SetMode(&Vnh[1], VNH5180_BRAKE_GND);
		}

		break;
	case CMD_PELTIER2_SET_POWER:
		VNH5180_SetDC(&Vnh[1], Data);

		break;
	case CMD_SET_POWER1:
		AD5726_SetVoltage(Ad, AD5726_TUBES[0], Data);

		break;
	case CMD_SET_POWER2:
		AD5726_SetVoltage(Ad, AD5726_TUBES[1], Data);

		break;
	case CMD_BATTERY_SWITCH:
		if (Data == SWITCH_ON) {
			XGpioPs_WritePin(Gpio, GPIO_POWER_ON_PIN, GPIO_POWER_ON_ASSERT);
			*PowerOn = GPIO_POWER_ON_ASSERT;
		} else if (Data == SWITCH_OFF) {
			XGpioPs_WritePin(Gpio, GPIO_POWER_ON_PIN, GPIO_POWER_ON_DEASSERT);
			*PowerOn = GPIO_POWER_ON_DEASSERT;
		}

		break;
	case CMD_KILL:
		XGpioPs_WritePin(Gpio, GPIO_KILL_PIN, GPIO_KILL_ASSERT);

		break;
	case CMD_FSAT_SWITCH:
		if (Data == SWITCH_ON) {
			FsatComm_Enable(FsatCommunicator);
		} else if (Data == SWITCH_OFF) {
			FsatComm_Disable(FsatCommunicator);
		}

		break;
	default:
		break;
	}
}

int PeripheralsInit(Communicator *Comm, FsatComm *FsatCommunicator,
		XGpioPs *Gpio, DS3231 *Ds, MAX31855 *Max, ADS1248 *Ads, LTC2943 *Ltc,
		XSysMon *SysMon, VNH5180 *Vnh, AD5726 *Ad, Pid *PidController)
{
	const int MAX31855_MUX_SEL_PIN[] = { MAX31855_MUX_SEL_PIN0,
			MAX31855_MUX_SEL_PIN1, MAX31855_MUX_SEL_PIN2, MAX31855_MUX_SEL_PIN3,
			MAX31855_MUX_SEL_PIN4 };
	XGpioPs_Config *GpioConfig;
	XSysMon_Config *SysMonConfig;

	/*
	 * Reset Flight Timer.
	 */
	XTime_SetTime(0);

	/*
	 * XGpioPs must be initialized before the other peripherals as they use it.
	 */
	if ((GpioConfig = XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID)) == NULL ) {
		DEBUG_PRINT("XGpioPs_LookupConfig() failed\n\r");
		return XST_FAILURE;
	}

	if (XGpioPs_CfgInitialize(Gpio, GpioConfig,
			GpioConfig->BaseAddr) != XST_SUCCESS) {
		DEBUG_PRINT("XGpioPs_CfgInitialize() failed\n\r");
		return XST_FAILURE;
	}

	if (XGpioPs_SelfTest(Gpio)) {
		DEBUG_PRINT("XGpioPs_SelfTest() failed\n\r");
		return XST_FAILURE;
	}

	/*
	 * Set POWER_ON and KILL pins as outputs
	 */
	XGpioPs_SetDirectionPin(Gpio, GPIO_POWER_ON_PIN, 1);
	XGpioPs_SetOutputEnablePin(Gpio, GPIO_POWER_ON_PIN, 1);
	XGpioPs_SetDirectionPin(Gpio, GPIO_KILL_PIN, 1);
	XGpioPs_SetOutputEnablePin(Gpio, GPIO_KILL_PIN, 1);

	/*
	 * Set LIFT_OFF and UG pins as inputs.
	 */
	XGpioPs_SetDirectionPin(Gpio, GPIO_LIFT_OFF_PIN, 0);
	XGpioPs_SetDirectionPin(Gpio, GPIO_UG_PIN, 0);

	/*
	 * Deassert output GPIOs.
	 */
	XGpioPs_WritePin(Gpio, GPIO_POWER_ON_PIN, GPIO_POWER_ON_DEASSERT);
	XGpioPs_WritePin(Gpio, GPIO_KILL_PIN, GPIO_KILL_DEASSERT);

	/*
	 * Initialize Communicator.
	 */
	if (Communicator_Init(Comm, COMMUNICATOR_UARTID, Gpio, COMMUNICATOR_DE_PIN,
			COMMUNICATOR_RE_PIN) != XST_SUCCESS) {
		DEBUG_PRINT("Communicator_Init() failed\n\r");
		return XST_FAILURE;
	};

	/*
	 * Initialize DS3231.
	 */
//	if (DS3231_Init(Ds, DS3231_IICID) != XST_SUCCESS) {
//		return XST_FAILURE;
//	};
	/*
	 * Initialize MAX31855.
	 */
	if (MAX31855_Init(Max, MAX31855_SPIID, Gpio, MAX31855_MUX_SEL_PIN,
			MAX31855_MUX_CS_PIN, MAX31855_MUX_WR_PIN,
			MAX31855_MUX_EN_PIN) != XST_SUCCESS) {
		return XST_FAILURE;
	};

	/*
	 * Initialize ADS1248.
	 */
	if (ADS1248_Init(Ads, ADS1248_SPIID, Gpio) != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Initialize LTC2943.
	 */
//	if (LTC2943_Init(Ltc, LTC2943_IICID) != XST_SUCCESS) {
//		return XST_FAILURE;
//	}
	/*
	 * Initialize XSysMon.
	 */
	if ((SysMonConfig = XSysMon_LookupConfig(XPAR_PS7_XADC_0_DEVICE_ID))
			== NULL ) {
		DEBUG_PRINT("XSysMon_LookupConfig() failed\n\r");
		return XST_FAILURE;
	}

	if (XSysMon_CfgInitialize(SysMon, SysMonConfig,
			SysMonConfig->BaseAddress) != XST_SUCCESS) {
		DEBUG_PRINT("XSysMon_CfgInitialize() failed\n\r");
		return XST_FAILURE;
	}

	XSysMon_SetAvg(SysMon, XSM_AVG_16_SAMPLES);

	/*
	 * Initialize AD5726.
	 */
	if (AD5726_Init(Ad, AD5726_SPIID, Gpio) != XST_SUCCESS) {
		DEBUG_PRINT("AD5726_Init() failed\n\r");
	}

#ifdef MPMA
	/*
	 * Initialize VNH5180s.
	 */
	if (VNH5180_Init(&Vnh[0], VNH5180_1_TTCID, VNH5180_1_FREQ, Gpio,
			VNH5180_1_INA_PIN, VNH5180_1_INB_PIN, VNH5180_1_ENA_PIN,
			VNH5180_1_ENB_PIN) != XST_SUCCESS) {
		return XST_FAILURE;
	}

	if (VNH5180_Init(&Vnh[1], VNH5180_2_TTCID, VNH5180_2_FREQ, Gpio,
			VNH5180_2_INA_PIN, VNH5180_2_INB_PIN, VNH5180_2_ENA_PIN,
			VNH5180_2_ENB_PIN) != XST_SUCCESS) {
		return XST_FAILURE;
	}

	AD5726_SetVoltage(Ad, VNH5180_DAC_CHANNEL, VNH5180_LEVEL_MAX);

	/*
	 * Initialize PID 0.
	 */
	Pid_Init(&PidController[0], PID_P, PID_I, PID_D, PID_SCALING_FACTOR);

	/*
	 * Initialize PID 1.
	 */
	Pid_Init(&PidController[1], PID_P, PID_I, PID_D, PID_SCALING_FACTOR);

	/*
	 * Initialize FloripaSat.
	 */
	if (FsatComm_Init(FsatCommunicator, Gpio, FSATCOMM_UARTID,
			FSATCOMM_ENABLE_PIN) != XST_SUCCESS) {
		return XST_FAILURE;
	};
#endif

	return XST_SUCCESS;
}

void Sample(Communicator_TxPacket *Packet, FsatComm *FsatCommunicator,
		XGpioPs *Gpio, MAX31855 *Max, ADS1248 *Ads, LTC2943 *Ltc,
		XSysMon *SysMon, int *PowerOn, int *Ug)
{
	const int ADC_CHAN_VOLT_RES[] = { ADC_CHAN_VOLT_RES1, ADC_CHAN_VOLT_RES2,
			ADC_CHAN_VOLT_RES3, ADC_CHAN_VOLT_RES4 };
	const int ADC_CHAN_CURR_RES[] = { ADC_CHAN_CURR_RES1, ADC_CHAN_CURR_RES2,
			ADC_CHAN_CURR_RES3, ADC_CHAN_CURR_RES4 };
	const int ADC_CHAN_PRESSURE_TRANSD[] = { ADC_CHAN_PRESSURE_TRANSD0,
			ADC_CHAN_PRESSURE_TRANSD1, ADC_CHAN_PRESSURE_TRANSD2,
			ADC_CHAN_PRESSURE_TRANSD3, ADC_CHAN_PRESSURE_TRANSD4 };
	const int ADS1248_CHAN[] = { ADS1248_CHAN0, ADS1248_CHAN1, ADS1248_CHAN2,
			ADS1248_CHAN3, ADS1248_CHAN4 };
	FsatComm_RxPacket *FsatPacket = NULL;
	XTime Time;
	u32 Data, DataTime, Buffer[COMMUNICATOR_N_DATA];
	s32 Temperature;
	int i, j;

	/*
	 * Channel 0: Time in seconds.
	 */
	XTime_GetTime(&Time);
	Time /= (XTime) (COUNTS_PER_SECOND / 1000);
	DataTime = (u32) Time;

	if (DataTime > TIMER_MAX) {
		DataTime = (u32) 0;
	}

	Communicator_Pack(&Packet[CHAN_TIME_1], DataTime, CHAN_TIME_1);

	/*
	 * Channel 1: Status.
	 */
	Data = *Ug | (XGpioPs_ReadPin(Gpio, GPIO_LIFT_OFF_PIN) << 1)
			| (*PowerOn << 3);
	Communicator_Pack(&Packet[CHAN_STATUS], Data, CHAN_STATUS);

	/*
	 * Channels 2-33: Thermocouple (MAX31855).
	 */
	for (i = 0; i < N_MAX31855; i++) {
		/*
		 * Num starts at 1.
		 */
		switch (MAX31855_GetRaw(Max, &Temperature, i + 1)) {
		case (XST_SUCCESS):
			DEBUG_PRINT("Thermocouple %d temperature: %f C\n\r", i + 1,
					MAX31855_Raw2Celsius(Temperature));
		case (MAX31855_ERR_OC):
			DEBUG_PRINT("Thermocouple %d open\n\r", i + 1);
		case (MAX31855_ERR_SCG):
			DEBUG_PRINT("Thermocouple %d connected to GND\n\r", i + 1);
		case (MAX31855_ERR_SCV):
			DEBUG_PRINT("Thermocouple %d connected to VCC\n\r", i + 1);
		}

		Communicator_Pack(&Packet[CHAN_MAX31855 + i], Temperature,
				CHAN_MAX31855 + i);
	}

	/*
	 * Channels 34-38: RTD (ADS1248).
	 */
	for (i = 0; i < N_ADS1248; i++) {
		Temperature = ADS1248_GetRaw(Ads, ADS1248_CHAN[i]);
		DEBUG_PRINT("RTD %d temperature (%d): %f C\n\r", i, (int) Data,
				ADS1248_Raw2Temp(Temperature));
		Communicator_Pack(&Packet[CHAN_ADS1248 + i], Temperature,
				CHAN_ADS1248 + i);
	}

	/*
	 * Channel 39: Time.
	 */
	Communicator_Pack(&Packet[CHAN_TIME_2], DataTime, CHAN_TIME_2);

	/*
	 * Channel 40: Board battery voltage (LTC2943).
	 */
//	if (LTC2943_GetRawVoltage(Ltc, &Temp) == XST_SUCCESS) {
//		Data = (u32) Temp;
//		Communicator_Pack(&Packet[CHAN_LTC2943_VOLT_BOARD], Data,
//				CHAN_LTC2943_VOLT_BOARD);
//	} else {
//		Communicator_Pack(&Packet[CHAN_LTC2943_VOLT_BOARD], (u32) 0,
//				CHAN_LTC2943_VOLT_BOARD);
//	}
	/*
	 * Channel 41: Board battery charge (LTC2943).
	 */
//	if (LTC2943_GetRawCharge(Ltc, &Temp) == XST_SUCCESS) {
//		Data = (u32) Temp;
//		Communicator_Pack(&Packet[CHAN_LTC2943_CHARGE_BOARD], Data,
//				CHAN_LTC2943_CHARGE_BOARD);
//	} else {
//		Communicator_Pack(&Packet[CHAN_LTC2943_CHARGE_BOARD], (u32) 0,
//				CHAN_LTC2943_CHARGE_BOARD);
//	}
	/*
	 * Channel 42: Board battery current (LTC2943).
	 */
//	if (LTC2943_GetRawCurrent(Ltc, &Temp) == XST_SUCCESS) {
//		Data = (u32) Temp;
//		Communicator_Pack(&Packet[CHAN_LTC2943_CURR_BOARD], Data,
//				CHAN_LTC2943_CURR_BOARD);
//	} else {
//		Communicator_Pack(&Packet[CHAN_LTC2943_CURR_BOARD], (u32) 0,
//				CHAN_LTC2943_CURR_BOARD);
//	}
	/*
	 * Channel 43: Board battery temperature (LTC2943).
	 */
//	if (LTC2943_GetRawTemp(Ltc, &Temp) == XST_SUCCESS) {
//		Data = (u32) Temp;
//		Communicator_Pack(&Packet[CHAN_LTC2943_TEMP_BOARD], Data,
//				CHAN_LTC2943_TEMP_BOARD);
//	} else {
//		Communicator_Pack(&Packet[CHAN_LTC2943_TEMP_BOARD], (u32) 0,
//				CHAN_LTC2943_TEMP_BOARD);
//	}
	/*
	 * Channels 44-47: Resistors voltage.
	 */
	for (i = 0; i < NELEMS(ADC_CHAN_VOLT_RES); i++) {
		Data = ReadAdc(SysMon, ADC_CHAN_VOLT_RES[i]);
		Communicator_Pack(&Packet[CHAN_VOLT_RES + i], Data, CHAN_VOLT_RES + i);
	}

	/*
	 * Channel 48: Board battery voltage.
	 */
	Data = ReadAdc(SysMon, ADC_CHAN_VOLT_BATT_BOARD);
	Communicator_Pack(&Packet[CHAN_VOLT_BATT_BOARD], Data,
			CHAN_VOLT_BATT_BOARD);

	/*
	 * Channel 49: Experiment battery voltage.
	 */
	Data = ReadAdc(SysMon, ADC_CHAN_VOLT_BATT_EXP);
	Communicator_Pack(&Packet[CHAN_VOLT_BATT_EXP], Data, CHAN_VOLT_BATT_EXP);

	/*
	 * Channels 50-53: Resistors current.
	 */
	for (i = 0; i < NELEMS(ADC_CHAN_CURR_RES); i++) {
		Data = ReadAdc(SysMon, ADC_CHAN_CURR_RES[i]);
		Communicator_Pack(&Packet[CHAN_CURR_RES + i], Data, CHAN_CURR_RES + i);
	}

	/*
	 * Channels 54-58: Pressure transducers.
	 */
	for (i = 0; i < NELEMS(ADC_CHAN_PRESSURE_TRANSD); i++) {
		Data = ReadAdc(SysMon, ADC_CHAN_PRESSURE_TRANSD[i]);
		Communicator_Pack(&Packet[CHAN_PRESSURE_TRANSD + i], Data,
				CHAN_PRESSURE_TRANSD + i);
	}

#ifdef MPMA
	/*
	 * Channels 59-72: FloripaSat Data.
	 */
	for (i = 0; i < FSATCOMM_N_SOFRAME + FSATCOMM_N_DATA + FSATCOMM_N_EOFRAME;
			i++) {
		FsatComm_Recv(FsatCommunicator, &FsatPacket);

		if (FsatPacket != NULL) {
			break;
		}
	}

	if (FsatPacket != NULL ) {
		/*
		 * SOF.
		 */
		for (i = 0; i < COMMUNICATOR_N_DATA; i++) {
			Buffer[i] = FsatPacket->SOFrame[i];
		}

		Data = ((u32) (Buffer[0]) << 16) & 0xFF0000;
		Data |= ((u32) (Buffer[1]) << 8) & 0xFF00;
		Data |= ((u32) (Buffer[2])) & 0xFF;

		Communicator_Pack(&Packet[CHAN_FSAT + 0], Data, CHAN_FSAT + 0);

		/*
		 * Data.
		 */
		for (i = 0; i < FSAT_N_PACKET - 2; i++) {
			for (j = 0; j < COMMUNICATOR_N_DATA; j++) {
				if (i * COMMUNICATOR_N_DATA + j > FSATCOMM_N_DATA - 1) {
					Buffer[j] = 0;
				} else {
					Buffer[j] = FsatPacket->Data[i * COMMUNICATOR_N_DATA + j];
				}
			}

			Data = ((u32) (Buffer[0]) << 16) & 0xFF0000;
			Data |= ((u32) (Buffer[1]) << 8) & 0xFF00;
			Data |= ((u32) (Buffer[2])) & 0xFF;

			Communicator_Pack(&Packet[CHAN_FSAT + 1 + i], Data,
					CHAN_FSAT + 1 + i);
		}

		/*
		 * EOF.
		 */
		for (i = 0; i < COMMUNICATOR_N_DATA; i++) {
			Buffer[i] = FsatPacket->EOFrame[i];
		}

		Data = ((u32) (Buffer[0]) << 16) & 0xFF0000;
		Data |= ((u32) (Buffer[1]) << 8) & 0xFF00;
		Data |= ((u32) (Buffer[2])) & 0xFF;

		Communicator_Pack(&Packet[CHAN_FSAT + FSAT_N_PACKET - 1], Data,
				CHAN_FSAT + FSAT_N_PACKET - 1);
	} else {
		for (i = 0; i < FSAT_N_PACKET; i++) {
			Communicator_Pack(&Packet[CHAN_FSAT + i], 0, CHAN_FSAT + i);
		}
	}
#else
	for (i = 0; i < FSAT_N_PACKET; i++) {
		Communicator_Pack(&Packet[CHAN_FSAT + i], Data, CHAN_FSAT + i);
	}
#endif
}

void Transmit(Communicator *Comm, Communicator_TxPacket *Packet)
{
	int i;

	for (i = 0; i < N_PACKET; i++) {
		Communicator_Send(Comm, &Packet[i]);
	}
}

/*
 * Set tube power to the minimum to avoid over heating.
 */
void MonitorTube(AD5726 *Ad, const int *Thermos, int NThermos, int Tube,
		int MaxThermos, int MaxTemp)
{
	int i, NHigh = 0;

	for (i = 0; i < NThermos; i++) {
		if (Thermos[i] > MaxTemp) {
			NHigh++;
		}
	}

	if (NHigh >= MaxThermos) {
		AD5726_SetVoltage(Ad, Tube, 0);
	}
}

```

Possui as funções:
Executa os comandos indicados nos parâmetros.
Inicializa os periféricos indicados nos parametros.
Funcao que envia dados pelo protocolo rs422
Possui um tempo limite ,quando atingido zera o tempo de transmissão .
Transmite o comando enviado via labview.
 

### *_Acq.h_*


```
#ifndef ACQ_H_
#define ACQ_H_

#include <communicator.h>
#include <fsatcomm.h>
#include <ds3231.h>
#include <max31855.h>
#include <ads1248.h>
#include <ltc2943.h>
#include <vnh5180.h>
#include <ad5726.h>
#include <util.h>
#include <pid.h>

/*
 * The maximum timer value can't be bigger than 2^24 - 1 which is the biggest
 * number that can be represented by 24 bits.
 */
enum {
	TIMER_MAX = 16777215,
};

/*
 * FloripaSat related constants.
 */
enum {
	FSAT_N_PACKET = 14
};

enum {
	N_PACKET = 73
};

enum {
	CHAN_TIME_1 = 0,
	CHAN_STATUS = 1,
	CHAN_MAX31855 = 2,
	CHAN_ADS1248 = 34,
	CHAN_TIME_2 = 39,
	CHAN_LTC2943_VOLT_BOARD = 40,
	CHAN_LTC2943_CHARGE_BOARD = 41,
	CHAN_LTC2943_CURR_BOARD = 42,
	CHAN_LTC2943_TEMP_BOARD = 43,
	CHAN_VOLT_RES = 44,
	CHAN_VOLT_BATT_BOARD = 48,
	CHAN_VOLT_BATT_EXP = 49,
	CHAN_CURR_RES = 50,
	CHAN_PRESSURE_TRANSD = 54,
	CHAN_FSAT = 59
};

enum {
	CMD_PELTIER1_SWITCH = 0xC4,
	CMD_PELTIER1_SET_POWER = 0xF5,
	CMD_PELTIER2_SWITCH = 0xC6,
	CMD_PELTIER2_SET_POWER = 0xF3,
	CMD_SET_POWER1 = 0xFA,
	CMD_SET_POWER2 = 0xFB,
	CMD_BATTERY_SWITCH = 0xCE,
	CMD_FSAT_SWITCH = 0xC9,
	CMD_KILL = 0xCB
};

enum {
	SWITCH_ON = 0x2A2A2A,
	SWITCH_OFF = 0x151515
};

void ExecCmd(Communicator_RxPacket *Packet, VNH5180 *Vnh, AD5726 *Ad,
		XGpioPs *Gpio, FsatComm *FsatCommunicator, int *PowerOn);
int PeripheralsInit(Communicator *Comm, FsatComm *FsatCommunicator,
		XGpioPs *Gpio, DS3231 *Ds, MAX31855 *Max, ADS1248 *Ads, LTC2943 *Ltc,
		XSysMon *SysMon, VNH5180 *Vnh, AD5726 *Ad, Pid *PidController);
void Sample(Communicator_TxPacket *Packet, FsatComm *FsatCommunicator,
		XGpioPs *Gpio, MAX31855 *Max, ADS1248 *Ads, LTC2943 *Ltc,
		XSysMon *SysMon, int *PowerOn, int *uG);
void Transmit(Communicator *Comm, Communicator_TxPacket *Packet);
void MonitorTube(AD5726 *Ad, const int *Thermos, int NThermos, int Tube,
		int MaxThermos, int MaxTemp);

#endif

```

Contém o tempo máximo e o número de pacotes a serem enviados pelo protocolo rs 422.
Enumera os canais de comunicação dos periféricos
Endereça os sinais de comando para as resistências e bateria
Endereça o chaveamento On Off
Cria as funções usadas no acq.c :
* **Transmit** 
* **MonitorTube** 
* **ExecCmd**
* **PeripheralsInit**
* **Sample**



## `Controle de potência`
---
Primeiramente o nome do arquivo foi dado pelo circuito intergrado que ele programa o [ad5726](https://www.analog.com/media/en/technical-documentation/data-sheets/AD5726.pdf),ele é usado para controlar as tensões enviadas ,por binário,via labview , para as resistências 
---
### *_ad5726.h_*


```
#ifndef AD5726_H_
#define AD5726_H_

#include "xspips.h"
#include "xgpiops.h"

#define AD5726_CLRSEL_PIN	70
#define AD5726_CLR_PIN		71
#define AD5726_LDAC_PIN	72

#define AD5726_SPI_OPTIONS	XSPIPS_MASTER_OPTION | \
							XSPIPS_CLK_ACTIVE_LOW_OPTION | \
							XSPIPS_CLK_PHASE_1_OPTION | \
							!XSPIPS_DECODE_SSELECT_OPTION | \
							!XSPIPS_FORCE_SSELECT_OPTION | \
							XSPIPS_MANUAL_START_OPTION

typedef struct {
	XSpiPs Spi;
	XGpioPs *Gpio;
} AD5726;

int AD5726_Init(AD5726 *Ad, u16 SpiId, XGpioPs *Gpio);
void AD5726_SetVoltage(AD5726 *Ad, unsigned int Channel, u16 Code);

#endif

```
* Elenca as funções para controlar o CI ad5726 pelas interfaces SPI e GPIO
* Define sinais para limpar comandos enviados via labview e seus pinos
* Criada a função que envia potência para as resistências


## *_ad5726.c_*
```
#include <sleep.h>

#include <ad5726.h>

int AD5726_Init(AD5726 *Ad, u16 SpiId, XGpioPs *Gpio)
{
	XSpiPs_Config *SpiConfig;

	/*
	 * Initialize the SPI device.
	 */
	if ((SpiConfig = XSpiPs_LookupConfig(SpiId)) == NULL) {
		return XST_FAILURE;
	}

	if (XSpiPs_CfgInitialize(&(Ad->Spi), SpiConfig, SpiConfig->BaseAddress) != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to check hardware build.
	 */
	if (XSpiPs_SelfTest(&(Ad->Spi)) != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Configure the SPI device.
	 */
	if (XSpiPs_SetOptions(&(Ad->Spi), AD5726_SPI_OPTIONS) != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Set the SPI device prescaler to divide by 256.
	 */
	if (XSpiPs_SetClkPrescaler(&(Ad->Spi), XSPIPS_CLK_PRESCALE_256) != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Using slave 0.
	 */
	if (XSpiPs_SetSlaveSelect(&(Ad->Spi), 0) != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Set LDAC, CLRSEL and CLR as outputs.
	 */
	Ad->Gpio = Gpio;

	XGpioPs_SetDirectionPin(Ad->Gpio, AD5726_LDAC_PIN, 1);
	XGpioPs_SetOutputEnablePin(Ad->Gpio, AD5726_LDAC_PIN, 1);

	XGpioPs_SetDirectionPin(Ad->Gpio, AD5726_CLRSEL_PIN, 1);
	XGpioPs_SetOutputEnablePin(Ad->Gpio, AD5726_CLRSEL_PIN, 1);

	XGpioPs_SetDirectionPin(Ad->Gpio, AD5726_CLR_PIN, 1);
	XGpioPs_SetOutputEnablePin(Ad->Gpio, AD5726_CLR_PIN, 1);

	/*
	 * Disable LDAC and put DACs in zero scale.
	 */
	XGpioPs_WritePin(Ad->Gpio, AD5726_LDAC_PIN, 1);

	XGpioPs_WritePin(Ad->Gpio, AD5726_CLRSEL_PIN, 0);

	XGpioPs_WritePin(Ad->Gpio, AD5726_CLR_PIN, 0);
	usleep(1);
	XGpioPs_WritePin(Ad->Gpio, AD5726_CLR_PIN, 1);

	return XST_SUCCESS;
}

void AD5726_SetVoltage(AD5726 *Ad, unsigned int Channel, u16 Code)
{
	u16 Reg;
	u8 SendBuf[2];

	Reg = ((u16) Channel << 14) | Code;

	SendBuf[0] = (u8) (Reg >> 8);
	SendBuf[1] = (u8) (Reg & 0xFF);

	XSpiPs_PolledTransfer(&(Ad->Spi), SendBuf, NULL, (unsigned) 2);

	/*
	 * Update DAC output.
	 */
	XGpioPs_WritePin(Ad->Gpio, AD5726_LDAC_PIN, 0);
	usleep(1);
	XGpioPs_WritePin(Ad->Gpio, AD5726_LDAC_PIN, 1);
}

```

#### 1. Função que inicializa comunicação serial `SPI`:
* Seta as configurações da interface 
* Verifica se foi iniciada corretamente 
* Seta um preescaler para dividir por 256 
* Usa o escravo `zero`
* Configura `LADAC, CLRSEL e CLR como saidas` 
* Desabilita LDAC e zera a escala 
* Retorna sinal de ação bem sucedida
#### 2. Função que envia as potências para resistências:
* Cria os buffers para enviar os bytes de comando
* Envia os bytes de comando 

## `Dados de temperatura e pressão`
---
As medidas de temperatura e pressão adquiridas pelos termopares e transdutor de pressão são processadas e adquiridas pelo [ads1248](https://www.ti.com/lit/ds/symlink/ads1248.pdf?ts=1598885620319&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS1248) e a programação converte os dados.
---

### *_ads1428.h_*
```
#ifndef ADS1248_H_
#define ADS1248_H_

#include "xspi.h"
#include "xgpiops.h"

/*
 * Digital interface pins.
 */
#define ADS1248_DRDY_PIN	67
#define ADS1248_START_PIN	68
#define ADS1248_RESET_PIN	69

#define ADS1248_SPI_OPTIONS	XSP_MASTER_OPTION | \
								!XSP_CLK_ACTIVE_LOW_OPTION | \
								XSP_CLK_PHASE_1_OPTION | \
								!XSP_LOOPBACK_OPTION | \
								XSP_MANUAL_SSELECT_OPTION

#define ADS1248_GAIN	1
#define ADS1248_VREF	1.5
#define ADS1248_IMAG	0.001

/*
 * Negative ADC channel.
 */
#define ADS1248_NEG_CHAN	7

/*
 * Register addresses.
 */
#define ADS1248_MUX0	0x00
#define ADS1248_MUX1	0x02
#define ADS1248_SYS0	0x03
#define ADS1248_IDAC0	0x0A
#define ADS1248_IDAC1	0x0B

/*
 * AD1248_MUX1 fields.
 */
#define DS3231_MUX1_CLKSTAT	7
#define DS3231_MUX1_VREFCON	5
#define DS3231_MUX1_REFSELT	3
#define DS3231_MUX1_MUXCAL		0

/*
 * AD1248_SYS0 fields.
 */
#define DS3231_SYS1_DR	0
#define DS3231_SYS1_PGA	4

/*
 * AD1248_IDAC0 fields.
 */
#define DS3231_IDAC0_DRDY_MODE	3
#define DS3231_IDAC0_IMAG		0

/*
 * AD1248_IDAC1 fields.
 */
#define DS3231_IDAC1_I1DIR	4
#define DS3231_IDAC1_I2DIR	0

/*
 * Commands.
 */
#define ADS1248_SYNC	0x04
#define ADS1248_RDATA	0x12
#define ADS1248_SDATAC	0x16
#define ADS1248_RREG	0x20
#define ADS1248_WREG	0x40
#define ADS1248_NOP	0xFF

typedef struct {
	XSpi_Config *SpiConfig;
	XSpi Spi;

	XGpioPs_Config *GpioConfig;
	XGpioPs *Gpio;
} ADS1248;

int ADS1248_Init(ADS1248 *Ads, u16 SpiId, XGpioPs *Gpio);
s32 ADS1248_GetRaw(ADS1248 *Ad, u8 Chan);
float ADS1248_Raw2Temp(s32 Data);

#endif

```

O programa controla o CI pela interface SPI