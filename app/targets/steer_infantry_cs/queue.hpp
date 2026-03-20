#ifndef MY_QUEUE_H
#define MY_QUEUE_H

#include "librm.hpp"

#define QSIZE 517

typedef struct {
  uint8_t RW_Lock;
  uint8_t arr[QSIZE];
  uint32_t front;
  int32_t rear;
  uint32_t counter;
} Queue_t;

void QueueInit(Queue_t *q);
uint8_t IsFull(Queue_t *q);
uint8_t IsEmpty(Queue_t *q);
void EnQueue(Queue_t *q, uint8_t *val, uint8_t lenth);
int Pop(Queue_t *buffer1, Queue_t *buffer2, uint8_t data[11]);
void UI_EnQueue(Queue_t *q, uint8_t *val, uint8_t lenth);
int UI_Pop(Queue_t *buffer1, uint8_t *data);

#endif  // MY_QUEUE_H