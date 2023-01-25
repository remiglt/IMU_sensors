/*
 * functions.h
 *
 *  Created on: Nov 30, 2022
 *      Author: noahd
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_


#endif /* INC_FUNCTIONS_H_ */

//Includes
#include "main.h"


//Functions
void Init(I2C_HandleTypeDef*);
void Measure_T(I2C_HandleTypeDef*,double*);
void Measure_A(I2C_HandleTypeDef*,double*);
void GetAccelOffset(I2C_HandleTypeDef*);


