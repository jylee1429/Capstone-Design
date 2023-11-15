#include "FND.h"
#include <Arduino.h>
FND fnd;
unsigned char FND_Number[29];

void SPI_Init(void){
  SPI.begin();
  digitalWrite(SS, HIGH);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
}

void FND_Init(FND* fnd, int rclk) {
	fnd->RCLK = rclk;
  pinMode(fnd->RCLK, OUTPUT);

  FND_Number[0] = 0xC0; //0
  FND_Number[1] = 0xF9; //1
  FND_Number[2] = 0xA4; //2
  FND_Number[3] = 0xB0; //3
  FND_Number[4] = 0x99; //4
  FND_Number[5] = 0x92; //5
  FND_Number[6] = 0x82; //6
  FND_Number[7] = 0xF8; //7
  FND_Number[8] = 0x80; //8
  FND_Number[9] = 0x90; //9
  FND_Number[10] = 0x88; //A
  FND_Number[11] = 0x83; //b
  FND_Number[12] = 0xC6; //C
  FND_Number[13] = 0xA1; //d
  FND_Number[14] = 0x86; //E
  FND_Number[15] = 0x8E; //F
  FND_Number[16] = 0xC2; //G
  FND_Number[17] = 0x89; //H
  FND_Number[18] = 0xF9; //I
  FND_Number[19] = 0xF1; //J
  FND_Number[20] = 0xC3; //L
  FND_Number[21] = 0xA9; //n
  FND_Number[22] = 0xC0; //O
  FND_Number[23] = 0x8C; //P
  FND_Number[24] = 0x98; //q
  FND_Number[25] = 0x92; //S
  FND_Number[26] = 0xC1; //U
  FND_Number[27] = 0x91; //Y
  FND_Number[28] = 0xFE; //hight -
}

void Send(FND* fnd, unsigned char x) {
  int data = SPI.transfer(x);
}

void SendPort(FND* fnd, unsigned char x, unsigned char port){
    Send(fnd, x);
    Send(fnd, port);
    digitalWrite(fnd->RCLK, LOW);
    digitalWrite(fnd->RCLK, HIGH);
}
void Digit4(FND* fnd, int n, int replay, int showZero) {
  int n1, n2, n3, n4;
  n1 = (int)n % 10;
  n2 = (int)(n % 100) / 10;
  n3 = (int)(n % 1000) / 100;
  n4 = (int)(n % 10000) / 1000;

  for (int i = 0; i <= replay; i++) {
    SendPort(fnd, FND_Number[n1], 0b0001);
    if (showZero | n > 9)
      SendPort(fnd, FND_Number[n2], 0b0010);
    if (showZero | n > 99)
      SendPort(fnd, FND_Number[n3], 0b0100);
    if (showZero | n > 999)
      SendPort(fnd, FND_Number[n4], 0b1000);
  }
}
void Digit4_Zero(FND* fnd, int n, int replay) {
  Digit4(fnd, n, replay, false);
}

void Digit4_ShowZero(FND* fnd, int n, int replay) {
  Digit4(fnd, n, replay, true);
}

void Display_FND(int val){
  digitalWrite(SS, LOW);
  Digit4_Zero(&fnd, val, 10);
}

void FND_Setting(void){
  SPI_Init();
  FND_Init(&fnd, RCLK_PIN);
}

