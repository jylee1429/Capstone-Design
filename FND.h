#ifndef _FND_SPI_H_
#define _FND_SPI_H_

#include <SPI.h>

#define TRUE 1
#define FALSE 0
#define RCLK_PIN 6

typedef struct _fnd {
	int RCLK;
}FND;

void SPI_Init(void);
void FND_Init(FND* fnd, int rclk);
void Send(FND* fnd, unsigned char X);
void SendPort(FND* fnd, unsigned char X, unsigned char port);
void Digit4(FND* fnd, int n, int replay, int showZero);
void Digit4_Zero(FND* fnd, int n, int replay);
void Digit4_ShowZero(FND* fnd, int n, int replay);

#endif