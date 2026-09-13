#ifndef REMOTER_H
#define REMOTER_H

#include "key.h"
#include "nrf24l01_recive.h"

void Remote_Init(void);
void Remote_Task(void);
void Remote_Send(void);
void date_pack_process(void);

#endif
