/*
 * functions.c
 *
 *  Created on: Nov 30, 2022
 *      Author: noahd
 */


//Includes
#include "functions.h"
#include "main.h"
#include "Const.h"


//Private Defines

#define MPU_ADDRESS 0xD0
#define MPU_RESET_ADDRESS 0x6B
#define MPU_RESET_VALUE 0x80
#define MPU_CLOCK_SELECT_VALUE 0x03
#define MPU_TEMP_H_ADDRESS 0x41
#define MPU_TEMP_L_ADDRESS 0x42

#define MPU_ACCEL_SCALE 16
#define MPU_ACCEL_SENS 32767.0

#define MPU_SPEED_SCALE 2000.0
#define MPU_SPEED_SENS 32767.0

#define MPU_XSPEED_OFFSET -18.0



//Private Variables
uint8_t MPU_reset;
uint8_t MPU_clockSelect;

uint8_t accelConfigReceived;
uint8_t newAccelConfig;

uint8_t speedConfigReceived;
uint8_t newSpeedConfig;

uint8_t bypassConfigReceived;
uint8_t newBypassConfig;

uint8_t MagConfigReceived;
uint8_t newMagConfig;

uint8_t newConfig;
uint8_t newSampleRate;


uint8_t TempReceivedRaw[2];
uint16_t TempCalculated;

uint8_t accelXoffsetRaw[2];
uint8_t accelYoffsetRaw[2];
uint8_t accelZoffsetRaw[2];
uint16_t AccelXOffset;
uint16_t AccelYOffset;
uint16_t AccelZOffset;

//acc

double accelXOffsetMem;
double accelYOffsetMem;
double accelZOffsetMem;

uint8_t AccelXReceivedRaw[2];
uint8_t AccelYReceivedRaw[2];
uint8_t AccelZReceivedRaw[2];
int16_t AccelXCalculated;
int16_t AccelYCalculated;
int16_t AccelZCalculated;

//gyro

uint8_t SpeedXReceivedRaw[2];
uint8_t SpeedYReceivedRaw[2];
uint8_t SpeedZReceivedRaw[2];
int16_t SpeedXCalculated;
int16_t SpeedYCalculated;
int16_t SpeedZCalculated;


//Mag

uint8_t DRDY[1];

uint8_t MagneticXReceivedRaw[2];
uint8_t MagneticYReceivedRaw[2];
uint8_t MagneticZReceivedRaw[2];

uint8_t MagneticSensibilityX;
uint8_t MagneticSensibilityY;
uint8_t MagneticSensibilityZ;

int16_t MagneticXCalculated;
int16_t MagneticYCalculated;
int16_t MagneticZCalculated;

double MagneticXMem;
double MagneticYMem;
double MagneticZMem;

//Functions

/**
 * @brief Initialisation
 * @Note Cette fonction permet de remettre tous les registres du capteur à leur valeur par défaut
 * @param I2C_HandleTypeDef* pStruct : structure qui contient l’ensemble des informations de configuration du périphérique utilisé
 * @retval None
 */
void Init(I2C_HandleTypeDef* pStruct)
{
	//	Hardware Reset : mise à 1 du bit n°7(H_RESET) du registre PWR_MGMT_1(addr : 0x6B)
	MPU_reset = MPU_RESET_VALUE;
	if(HAL_I2C_Mem_Write(pStruct,MPU_ADDRESS ,MPU_RESET_ADDRESS,1,&MPU_reset,1,10)!=HAL_OK)
	{
		printf("Error : Cannot reset the registers \r\n");
		Error_Handler();
	}

	HAL_Delay(100);

	//Bypass Enable
	if(HAL_I2C_Mem_Read(pStruct,MPU_ADDRESS ,INT_PIN_CFG,1,&bypassConfigReceived,1,10)!=HAL_OK)
	{
		printf("Error : Cannot read bypass config \r\n");
		Error_Handler();
	}

	newBypassConfig = bypassConfigReceived | 0x2; //0x18 pour une échelle de vitesse de 2000dps

	if(HAL_I2C_Mem_Write(pStruct,MPU_ADDRESS ,INT_PIN_CFG,1, &newBypassConfig,1,10)!=HAL_OK)
	{
		printf("Error : Cannot select new bypass config \r\n");
		Error_Handler();
	}
	printf("New bypass config : %d \r\n", (int)&newSpeedConfig);

	//Magneto config

	if(HAL_I2C_Mem_Read(pStruct,MAGNETO_ADD ,0x0A,1,&MagConfigReceived,1,10)!=HAL_OK)
	{
		printf("Error : Cannot read Mag config \r\n");
		Error_Handler();
	}

	newMagConfig = 0x16;

	if(HAL_I2C_Mem_Write(pStruct,MAGNETO_ADD ,AK8963_CNTL,1, &newMagConfig,1,10)!=HAL_OK)
	{
		printf("Error : Cannot select new Mag config \r\n");
		Error_Handler();
	}
	printf("New Mag config : %d \r\n", (int)&newMagConfig);


	//config echantillonage

	newConfig = 0x3;
	if(HAL_I2C_Mem_Write(pStruct,MPU_ADD,CONFIG,1,newConfig,1,10)!=HAL_OK)
	{
		printf("Error : Cannot select new fchoice config \r\n");
		Error_Handler();
	}

	newSampleRate = 0xFF;
	if(HAL_I2C_Mem_Write(pStruct,MPU_ADD,SMPLRT_DIV,1,newSampleRate,1,10)!=HAL_OK)
	{
		printf("Error : Cannot select new sample rate divider \r\n");
		Error_Handler();
	}

	//	Clock Selection
	MPU_clockSelect = MPU_CLOCK_SELECT_VALUE;
	if(HAL_I2C_Mem_Write(pStruct,MPU_ADDRESS ,MPU_RESET_ADDRESS,1,&MPU_clockSelect,1,10)!=HAL_OK)
	{
		printf("Error : Cannot select clock \r\n");
		Error_Handler();
	}

	//Configuration de l'accelerometre
	if(HAL_I2C_Mem_Read(pStruct,MPU_ADDRESS ,ACCEL_CONFIG,1,&accelConfigReceived,1,10)!=HAL_OK)
	{
		printf("Error : Cannot read accel config \r\n");
		Error_Handler();
	}

	newAccelConfig = accelConfigReceived | 0x18; //0x18 pour une échelle d'acceleration de 16g

	if(HAL_I2C_Mem_Write(pStruct,MPU_ADDRESS ,ACCEL_CONFIG,1, &newAccelConfig,1,10)!=HAL_OK)
	{
		printf("Error : Cannot select new accel config \r\n");
		Error_Handler();
	}
	printf("New accel config : %d \r\n", newAccelConfig);

	//Configuration du gyroscope
	if(HAL_I2C_Mem_Read(pStruct,MPU_ADDRESS ,GYRO_CONFIG,1,&speedConfigReceived,1,10)!=HAL_OK)
	{
		printf("Error : Cannot read speed config \r\n");
		Error_Handler();
	}

	newSpeedConfig = speedConfigReceived | 0x18; //0x18 pour une échelle de vitesse de 2000dps

	if(HAL_I2C_Mem_Write(pStruct,MPU_ADDRESS ,GYRO_CONFIG,1, &newSpeedConfig,1,10)!=HAL_OK)
	{
		printf("Error : Cannot select new speed config \r\n");
		Error_Handler();
	}
	printf("New speed config : %d \r\n", newSpeedConfig);


}


//==================================================
//Version fonctionnelle de Init() pour la mesure de vitesse, d'acceleration et de température
//==================================================
/*
void Init(I2C_HandleTypeDef* pStruct)
{
	//	Hardware Reset : mise à 1 du bit n°7(H_RESET) du registre PWR_MGMT_1(addr : 0x6B)
	MPU_reset = MPU_RESET_VALUE;
	if(HAL_I2C_Mem_Write(pStruct,MPU_ADDRESS ,MPU_RESET_ADDRESS,1,&MPU_reset,1,10)!=HAL_OK)
	{
		printf("Error : Cannot reset the registers \r\n");
		Error_Handler();
	}

	HAL_Delay(100);

	//	Clock Selection
	MPU_clockSelect = MPU_CLOCK_SELECT_VALUE;
	if(HAL_I2C_Mem_Write(pStruct,MPU_ADDRESS ,MPU_RESET_ADDRESS,1,&MPU_clockSelect,1,10)!=HAL_OK)
	{
		printf("Error : Cannot select clock \r\n");
		Error_Handler();
	}

	//Configuration de l'accelerometre
	if(HAL_I2C_Mem_Read(pStruct,MPU_ADDRESS ,ACCEL_CONFIG,1,&accelConfigReceived,1,10)!=HAL_OK)
	{
		printf("Error : Cannot read accel config \r\n");
		Error_Handler();
	}

	newAccelConfig = accelConfigReceived | 0x18; //0x18 pour une échelle d'acceleration de 16g

	if(HAL_I2C_Mem_Write(pStruct,MPU_ADDRESS ,ACCEL_CONFIG,1, &newAccelConfig,1,10)!=HAL_OK)
	{
		printf("Error : Cannot select new accel config \r\n");
		Error_Handler();
	}
	printf("New accel config : %d \r\n", newAccelConfig);

	//Configuration du gyroscope
	if(HAL_I2C_Mem_Read(pStruct,MPU_ADDRESS ,GYRO_CONFIG,1,&speedConfigReceived,1,10)!=HAL_OK)
	{
		printf("Error : Cannot read speed config \r\n");
		Error_Handler();
	}

	newSpeedConfig = speedConfigReceived | 0x18; //0x18 pour une échelle de vitesse de 2000dps

	if(HAL_I2C_Mem_Write(pStruct,MPU_ADDRESS ,GYRO_CONFIG,1, &newSpeedConfig,1,10)!=HAL_OK)
	{
		printf("Error : Cannot select new speed config \r\n");
		Error_Handler();
	}
	printf("New speed config : %d \r\n", newSpeedConfig);


}*/




/**
 * @brief Mesure Temperature
 * @Note Cette fonction permet d'envoyer dans le terminal les valeurs de temperature mesurée chaque seconde
 * @param I2C_HandleTypeDef* pStruct : structure qui contient l’ensemble des informations de configuration du périphérique utilisé
 * @param double* tempMem : buffer où sera stockée la valeur de température mesurée
 * @retval None
 */
void Measure_T(I2C_HandleTypeDef* pStruct, double* tempMem )
{
	if(HAL_I2C_Mem_Read(pStruct,MPU_ADDRESS ,MPU_TEMP_H_ADDRESS,1,TempReceivedRaw,2,10)!=HAL_OK)
	{
		printf("Error : Cannot read temperature \r\n");
		Error_Handler();
	}
	TempCalculated = TempReceivedRaw[0];
	TempCalculated = TempCalculated<<8;
	TempCalculated = TempCalculated + TempReceivedRaw[1];
	*tempMem = ((TempCalculated-ROOM_TEMP_OFFSET)/TEMP_SENS)+21;
	printf("Temp : %f\r\n\r\n",*tempMem);

}

/*
void GetAccelOffset(I2C_HandleTypeDef* pStruct)
{
	if(HAL_I2C_Mem_Read(pStruct,MPU_ADDRESS,XA_OFFSET_H,1,&accelXoffsetRaw,2,10)!=HAL_OK)
	{
		printf("Error : can't read accel offset\r\n");
		Error_Handler();
	}

	if(HAL_I2C_Mem_Read(pStruct,MPU_ADDRESS,YA_OFFSET_H,1,&accelYoffsetRaw,2,10)!=HAL_OK)
	{
		printf("Error : can't read accel offset\r\n");
		Error_Handler();
	}

	if(HAL_I2C_Mem_Read(pStruct,MPU_ADDRESS,XA_OFFSET_H,1,&accelZoffsetRaw,2,10)!=HAL_OK)
	{
		printf("Error : can't read accel offset\r\n");
		Error_Handler();
	}


	AccelXOffset = accelXoffsetRaw[0];
	AccelXOffset = AccelXOffset<<7;
	AccelXOffset = AccelXOffset + accelXoffsetRaw[1];

	AccelXOffset = accelYoffsetRaw[0];
	AccelXOffset = AccelXOffset<<7;
	AccelXOffset = AccelXOffset + accelYoffsetRaw[1];

	AccelXOffset = accelZoffsetRaw[0];
	AccelXOffset = AccelXOffset<<7;
	AccelXOffset = AccelXOffset + accelZoffsetRaw[1];

	accelXOffsetMem = AccelXOffset;
	accelYOffsetMem = AccelYOffset;
	accelZOffsetMem = AccelZOffset;

	printf(" AccelX Offset : %f\r\n AccelY Offset : %f\r\n AccelZ Offset : %f\r\n ", accelXOffsetMem,accelYOffsetMem,accelZOffsetMem);
}*/




/**
 * @brief Mesure Acceleration
 * @Note Cette fonction permet d'envoyer dans le terminal les valeurs d'accélératiion mesurées sur chque axe
 * @param I2C_HandleTypeDef* pStruct : structure qui contient l’ensemble des informations de configuration du périphérique utilisé
 * @param double* tempMem : buffer où sera stockée la valeur d'accélération mesurée (en valeur absolue)
 * @retval None
 */
void Measure_A(I2C_HandleTypeDef* pStruct, double* accelMem )
{
	if(HAL_I2C_Mem_Read(pStruct, MPU_ADDRESS ,ACCEL_XOUT_H,1,AccelXReceivedRaw,2,10)!=HAL_OK)
	{
		printf("Error : Cannot read accelX \r\n");
		Error_Handler();
	}
	if(HAL_I2C_Mem_Read(pStruct, MPU_ADDRESS ,ACCEL_YOUT_H,1,AccelYReceivedRaw,2,10)!=HAL_OK)
	{
		printf("Error : Cannot read accelY \r\n");
		Error_Handler();
	}
	if(HAL_I2C_Mem_Read(pStruct, MPU_ADDRESS ,ACCEL_ZOUT_H,1,AccelZReceivedRaw,2,10)!=HAL_OK)
	{
		printf("Error : Cannot read accelZ \r\n");
		Error_Handler();
	}

	AccelXCalculated = AccelXReceivedRaw[0];
	AccelXCalculated = AccelXCalculated<<8;
	AccelXCalculated = AccelXCalculated + AccelXReceivedRaw[1];
	AccelYCalculated = AccelYReceivedRaw[0];
	AccelYCalculated = AccelYCalculated<<8;
	AccelYCalculated = AccelYCalculated + AccelYReceivedRaw[1];
	AccelZCalculated = AccelZReceivedRaw[0];
	AccelZCalculated = AccelZCalculated<<8;
	AccelZCalculated = AccelZCalculated + AccelZReceivedRaw[1];

	//On a choisi 16g
	*accelMem = (AccelXCalculated)* MPU_ACCEL_SCALE/MPU_ACCEL_SENS;
	*(accelMem+1) = (AccelYCalculated)* MPU_ACCEL_SCALE/MPU_ACCEL_SENS;
	*(accelMem+2) = (AccelZCalculated)* MPU_ACCEL_SCALE/MPU_ACCEL_SENS;

	printf(" AccelX : %f\r\n AccelY : %f\r\n AccelZ : %f\r\n ", *accelMem,*(accelMem+1),*(accelMem+2));

}


void Measure_S(I2C_HandleTypeDef* pStruct, double* speedMem )
{
	if(HAL_I2C_Mem_Read(pStruct, MPU_ADDRESS ,GYRO_XOUT_H,1,SpeedXReceivedRaw,2,10)!=HAL_OK)
	{
		printf("Error : Cannot read SpeedX \r\n");
		Error_Handler();
	}
	if(HAL_I2C_Mem_Read(pStruct, MPU_ADDRESS ,GYRO_YOUT_H,1,SpeedYReceivedRaw,2,10)!=HAL_OK)
	{
		printf("Error : Cannot read SpeedY \r\n");
		Error_Handler();
	}
	if(HAL_I2C_Mem_Read(pStruct, MPU_ADDRESS ,GYRO_ZOUT_H,1,SpeedZReceivedRaw,2,10)!=HAL_OK)
	{
		printf("Error : Cannot read SpeedZ \r\n");
		Error_Handler();
	}

	SpeedXCalculated = SpeedXReceivedRaw[0];
	SpeedXCalculated = SpeedXCalculated<<8;
	SpeedXCalculated = SpeedXCalculated + SpeedXReceivedRaw[1];
	SpeedYCalculated = SpeedYReceivedRaw[0];
	SpeedYCalculated = SpeedYCalculated<<8;
	SpeedYCalculated = SpeedYCalculated + SpeedYReceivedRaw[1];
	SpeedZCalculated = SpeedZReceivedRaw[0];
	SpeedZCalculated = SpeedZCalculated<<8;
	SpeedZCalculated = SpeedZCalculated + SpeedZReceivedRaw[1];

	//On a choisi 2000 dps
	*speedMem = (SpeedXCalculated)* MPU_SPEED_SCALE/MPU_SPEED_SENS - MPU_XSPEED_OFFSET;
	*(speedMem+1)= (SpeedYCalculated)* MPU_SPEED_SCALE/MPU_SPEED_SENS;
	*(speedMem+2) = (SpeedZCalculated)* MPU_SPEED_SCALE/MPU_SPEED_SENS;

	printf(" SpeedX : %f\r\n SpeedY : %f\r\n SpeedZ : %f\r\n ", *speedMem,*(speedMem+1),*(speedMem+2));

}

/**
* \fn Measure_M(I2C_HandleTypeDef* p_hi2c1,double* mag)
 * \brief Cette fonction récupére les données de champ magnétique et les transforme en
	 une valeur décimale
	 * \param p_hi2c1
	 * \param  mag
	 */
void Measure_M(I2C_HandleTypeDef* pStruct,double* mag){

	int16_t x;
	int16_t y;
	int16_t z;

	if((HAL_I2C_Mem_Read(pStruct,MAGNETO_ADD,AK8963_XOUT_L,1,MagneticXReceivedRaw,2,10)!=HAL_OK)
			&(HAL_I2C_Mem_Read(pStruct,MAGNETO_ADD,AK8963_ASAX,1,MagneticSensibilityX,1,10)!=HAL_OK))
	{
		printf("Error : Cannot read MagneticX \r\n");
		Error_Handler();
	}
	if((HAL_I2C_Mem_Read(pStruct,MAGNETO_ADD,AK8963_YOUT_L,1,MagneticYReceivedRaw,2,10)!=HAL_OK)
			&(HAL_I2C_Mem_Read(pStruct,MAGNETO_ADD,AK8963_ASAY,1,MagneticSensibilityY,1,10)!=HAL_OK))
	{
		printf("Error : Cannot read MagneticY \r\n");
		Error_Handler();
	}
	if((HAL_I2C_Mem_Read(pStruct,MAGNETO_ADD,AK8963_ZOUT_L,1,MagneticZReceivedRaw,2,10)!=HAL_OK)
			&(HAL_I2C_Mem_Read(pStruct,MAGNETO_ADD,AK8963_ASAZ,1,MagneticSensibilityZ,1,10)!=HAL_OK))
	{
		printf("Error : Cannot read MagneticZ \r\n");
		Error_Handler();
	}

			MagneticXCalculated = MagneticXReceivedRaw[1];
			MagneticXCalculated = MagneticXCalculated<<8;
			MagneticXCalculated = MagneticXCalculated + MagneticXReceivedRaw[0];

			MagneticYCalculated = 256*MagneticXReceivedRaw[1] + MagneticXReceivedRaw[0];

			MagneticYCalculated = MagneticYReceivedRaw[1];
			MagneticYCalculated = MagneticYCalculated<<8;
			MagneticYCalculated = MagneticYCalculated + MagneticYReceivedRaw[0];


			MagneticZCalculated = MagneticZReceivedRaw[1];
			MagneticZCalculated = MagneticZCalculated<<8;
			MagneticZCalculated = MagneticZCalculated + MagneticZReceivedRaw[0];

			x = MagneticSensibilityX;
			y = MagneticSensibilityY;
			z = MagneticSensibilityZ;

			*mag=(MagneticXCalculated*(((( x - 128)*0.5)/128)+1)*4912)/32760;
			*(mag+1)=(MagneticYCalculated *(((( y - 128)*0.5)/128)+1)*4912)/32760;
			*(mag+2)=(MagneticZCalculated*(((( z - 128)*0.5)/128)+1)*4912)/32760;

}

void Calib_Gyro(I2C_HandleTypeDef* pStruct,double* offset){

	double newoffset[3];
	double Sensibility;

	uint8_t offset_x;
	uint8_t offset_y;
	uint8_t offset_z;

	Sensibility = MPU_SPEED_SCALE/MPU_SPEED_SENS;

	Average(pStruct, newoffset);

	newoffset[0] = newoffset[0]/Sensibility;
	newoffset[1] = newoffset[1]/Sensibility;
	newoffset[2] = newoffset[2]/Sensibility;

	*offset = newoffset[0];
	*(offset+1)= newoffset[1];
	*(offset+2)= newoffset[2];

	//conversion afin de pouvoir écrire en uint8_t dans HAL_I2C_Mem_Write

	offset_x	= (uint8_t) newoffset[0];
	offset_y	= (uint8_t) newoffset[1];
	offset_z	= (uint8_t) newoffset[2];

	if(HAL_I2C_Mem_Write(pStruct,MPU_ADD,XG_OFFSET_L,1,offset_x,2,10)!=HAL_OK)
	{
		printf("Error : Cannot calibrate gyro \r\n");
		Error_Handler();
	}

	if(HAL_I2C_Mem_Write(pStruct,MPU_ADD,YG_OFFSET_L,1,offset_y,2,10)!=HAL_OK)
		{
			printf("Error : Cannot calibrate gyro \r\n");
			Error_Handler();
		}
	if(HAL_I2C_Mem_Write(pStruct,MPU_ADD,ZG_OFFSET_L,1,offset_z,2,10)!=HAL_OK)
		{
			printf("Error : Cannot calibrate gyro \r\n");
			Error_Handler();
		}
}

void Average(I2C_HandleTypeDef* pStruct,double* offset) {

	int i=0;
	double newoffset[3];
	double newoffset_x;
	double newoffset_y;
	double newoffset_z;

	for(i=0;i<=250;i++){

		Measure_S(pStruct, newoffset);
		newoffset_x= newoffset[0];
		newoffset_y= newoffset[1];
		newoffset_z= newoffset[2];

		*offset = *offset + newoffset_x;

		*(offset+1) = *(offset+1) + newoffset_y;

		*(offset+2) = *(offset+1) + newoffset_z;
	}

	*offset = *offset/250;
	*(offset+1) = *(offset+1)/250;
	*(offset+2) = *(offset+2)/250;
}

