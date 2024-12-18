/*
 * pMMG.h
 *
 *  Created on: Jul 5, 2024
 *      Author: INVINCIBLE
 */

#ifndef _PMMG_H_
#define _PMMG_H_


#include "stm32g4xx_hal.h"

/* SPI Commands */
#define RESET_CMD							0x1E
#define PROM_READ(address)          		(0xA0 | ((address) << 1)) 		// Macro to change values for the 8 PROM addresses

#define CONVERT_D1_OSR_DEFAULT_CMD			0x40
#define CONVERT_D1_OSR256_CMD            	0x40
#define CONVERT_D1_OSR512_CMD            	0x42
#define CONVERT_D1_OSR1024_CMD            	0x44
#define CONVERT_D1_OSR2048_CMD            	0x46
#define CONVERT_D1_OSR4096_CMD            	0x48

#define CONVERT_D2_OSR_DEFAULT_CMD			0x50
#define CONVERT_D2_OSR256_CMD            	0x50
#define CONVERT_D2_OSR512_CMD            	0x52
#define CONVERT_D2_OSR1024_CMD            	0x54
#define CONVERT_D2_OSR2048_CMD            	0x56
#define CONVERT_D2_OSR4096_CMD            	0x58

#define READ_ADC_CMD              			0x00


/* Oversampling ratio */
typedef enum _pMMG_OSR_t {
	OSR_256  = 0x00,
	OSR_512  = 0x02,
	OSR_1024 = 0x04,
	OSR_2048 = 0x06,
	OSR_4096 = 0x08
} pMMG_OSR_t;


/* System states */
typedef enum _pMMG_State_t {
	pMMG_STATE_OK,
	pMMG_STATE_ERROR,
	pMMG_STATE_ERROR_1,
	pMMG_STATE_ERROR_2,
	pMMG_STATE_ERROR_3
} pMMG_State_t;


/* PROM data structure */
typedef struct _pMMG_PROMData_t {
  uint16_t reserved;
  uint16_t sens;		// Typ : 46372
  uint16_t off;			// Typ : 43981
  uint16_t tcs;			// Typ : 29059
  uint16_t tco;			// Typ : 27842
  uint16_t tref;		// Typ : 31553
  uint16_t tempsens;	// Typ : 28165
  uint16_t crc;
} pMMG_PROMData_t;

typedef struct _pMMG_UncompData_t {
	uint32_t uncompPressure;		// Typ : 6465444
	uint32_t uncompTemperature;		// Typ : 8077636
} pMMG_UncompData_t;

typedef struct _pMMG_Data_t {
	int32_t pressure;
	int32_t temperature;

	double pressureKPa;
	double temperatureC;
} pMMG_Data_t;

/* pMMG data structure */
typedef struct _pMMG_Obj_t {
	SPI_HandleTypeDef* pMMG_hspi;
	GPIO_TypeDef* pMMG_CS_GPIO_Port;
	uint16_t pMMG_CS_Pin;
	pMMG_PROMData_t promData;
	pMMG_UncompData_t uncompData;
	pMMG_Data_t pMMGData;
} pMMG_Obj_t;



/* Declaration of functions */
pMMG_State_t pMMG_Init(pMMG_Obj_t* pMMG_Obj, SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void pMMG_ReadPROM(pMMG_Obj_t* pMMG_Obj);
void pMMG_ReadUncompValue(pMMG_Obj_t* pMMG_Obj);
void pMMG_Convert(pMMG_Obj_t* pMMG_Obj);
void pMMG_Update(pMMG_Obj_t* pMMG_Obj);
void pMMG_EnableCS(pMMG_Obj_t* pMMG_Obj);
void pMMG_DisableCS(pMMG_Obj_t* pMMG_Obj);
void us_Delay(uint32_t us_delay);

void pMMG_ReadUncompValue_multiple_3(pMMG_Obj_t* pMMG_Obj1, pMMG_Obj_t* pMMG_Obj2, pMMG_Obj_t* pMMG_Obj3);
void pMMG_Update_multiple_3(pMMG_Obj_t* pMMG_Obj1, pMMG_Obj_t* pMMG_Obj2, pMMG_Obj_t* pMMG_Obj3);
void pMMG_ReadUncompValue_multiple_2(pMMG_Obj_t* pMMG_Obj1, pMMG_Obj_t* pMMG_Obj2);
void pMMG_Update_multiple_2(pMMG_Obj_t* pMMG_Obj1, pMMG_Obj_t* pMMG_Obj2);

#endif /* _PMMG_H_ */
