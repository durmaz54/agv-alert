/*
 * ekran.h
 *
 *  Created on: 14 Ara 2022
 *      Author: besik
 */

#ifndef INC_EKRAN_H_
#define INC_EKRAN_H_

#include "stm32f1xx.h"
#include "string.h"

struct ekran {
	//int V1;
	//int V2;
	int temp1;
	int temp2;
	int pil;
	float I1;
	float I2;
	char konumbilgisi1;
	char konumbilgisi2;
	char konumbilgisi3;
	int qrbilgisi1;
	int qrbilgisi2;
	int qrbilgisi3;
	};


void NEXTION_SendString(UART_HandleTypeDef* uartx,char *ekran_konum, char *ekrana_yazilan_veri);
void NEXTION_SendAllData(UART_HandleTypeDef* uartx,struct ekran ekran1);

#endif /* INC_EKRAN_H_ */
