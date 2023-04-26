/*
 * ekran.c
 *
 *  Created on: 14 Ara 2022
 *      Author: besik
 */


#include "ekran.h"



static uint8_t Cmd_End[3] = {0xFF, 0xFF, 0xFF}; //Commend end sequence.
void NEXTION_SendString(UART_HandleTypeDef *uartx,char *ekran_konum, char *ekrana_yazilan_veri)
{
	char buf[20];  // char array (8-byte) (1 char = 8-bit = 1-byte)
	memset(buf, 0x00 , 20);
	int8_t len = sprintf (buf, "%s.txt=\"%s\"", ekran_konum, ekrana_yazilan_veri ); //(1:char array-2:ekran konumu ve ekrana yazılan veri birleşimi-3:çevrilecek verinin konumu-4:ekrana yazılacak veri) sprintf=hem veriyi stringe çeviriyor hem de boyutunu (kaç byte olduğu) söylüyor.
	HAL_UART_Transmit(uartx, (uint8_t*)buf, len, 1000); // buf 8bitlere ayırıyor burada.
	HAL_UART_Transmit(uartx, Cmd_End, 3, 100);
}

void NEXTION_SendAllData(UART_HandleTypeDef* uartx,struct ekran ekran1)   // Ekrana string göndermek için fonksiyon (1.nesne adı 2.göndermek istediğimiz string)
{
	//char str_V1[4];
	//char str_V2[4];
	char str_temp1[4];
	char str_temp2[4];
	char str_pil[4];
	char str_I1[8];
	char str_I2[8];
    char str_qrbilgisi1[4];
    char str_qrbilgisi2[4];
    char str_qrbilgisi3[4];

    char cr_konumbilgisi1[4];
	char cr_konumbilgisi2[4];
	char cr_konumbilgisi3[4];
	sprintf(cr_konumbilgisi1,"%s.",ekran1.konumbilgisi1);


	//sprintf(str_V1, "%d", ekran1.V1);
	//sprintf(str_V2, "%d", ekran1.V2);

	sprintf(str_temp1, "%d", ekran1.temp1);
	sprintf(str_temp2, "%d", ekran1.temp2);
	sprintf(str_pil, "%d", ekran1.pil);
	sprintf(str_I1, "%.2f", ekran1.I1);
	sprintf(str_I2, "%.2f", ekran1.I2);
	sprintf(str_qrbilgisi1, "%d", ekran1.qrbilgisi1);
	sprintf(str_qrbilgisi2, "%d", ekran1.qrbilgisi2);
	sprintf(str_qrbilgisi3, "%d", ekran1.qrbilgisi3);


    //NEXTION_SendString(uartx,"x0",str_V1);
    //NEXTION_SendString(uartx,"x1",str_V2);
    NEXTION_SendString(uartx,"t12",str_temp1);
    NEXTION_SendString(uartx,"t13",str_temp2);
    NEXTION_SendString(uartx,"t22",str_pil);
    NEXTION_SendString(uartx,"t2",str_I1);
    NEXTION_SendString(uartx,"t3",str_I2);
    NEXTION_SendString(uartx,"t17",str_qrbilgisi1);
    NEXTION_SendString(uartx,"t18",str_qrbilgisi2);
    NEXTION_SendString(uartx,"t19",str_qrbilgisi3);

    NEXTION_SendString(uartx,"t7",cr_konumbilgisi1);
    NEXTION_SendString(uartx,"t8",cr_konumbilgisi2);
   NEXTION_SendString(uartx,"t9",cr_konumbilgisi3);
}
