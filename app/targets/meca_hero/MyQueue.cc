#include "MyQueue.h"

void QueueInit(Queue_t *q) {
  q->RW_Lock = 0;
  q->front = 0;
  q->rear = -1;
  q->counter = 0;
}

uint8_t IsFull(Queue_t *q) {
  if (q->counter >= QSIZE) return 1;
  return 0;
}

uint8_t IsEmpty(Queue_t *q) {
  if (q->counter <= 0) return 1;
  return 0;
}

void EnQueue(Queue_t *q, uint8_t *val, uint8_t lenth) {
  uint8_t i = 0;
  if (q->counter > (QSIZE - lenth)) return;

  q->RW_Lock = 1;
  for (; i < lenth; i++) {
    q->rear = (q->rear + 1) % QSIZE;
    q->arr[q->rear] = *(val + i);
    q->counter++;
  }
  q->RW_Lock = 0;
}

// 0代表空 -1代表出错
int Pop(Queue_t *buffer1, Queue_t *buffer2, uint8_t data[11]) {
  int i = 0;
  if (IsEmpty(buffer1)) {
    if (IsEmpty(buffer2))
      return 0;
    else if (buffer2->RW_Lock == 0 && buffer2->counter % 11 == 0) {
      for (; i < 11; i++) {
        data[i] = buffer2->arr[buffer2->front];
        buffer2->front = (buffer2->front + 1) % QSIZE;
        buffer2->counter--;
      }
    } else {
      return -1;
    }

  } else if (buffer1->RW_Lock == 0 && buffer1->counter % 11 == 0) {
    for (; i < 11; i++) {
      data[i] = buffer1->arr[buffer1->front];
      buffer1->front = (buffer1->front + 1) % QSIZE;
      buffer1->counter--;
    }
  } else {
    return -1;
  }
  return 1;
}

void UI_EnQueue(Queue_t *q, uint8_t *val, uint8_t lenth) {
  if (q->counter > (QSIZE - lenth)) return;

  q->RW_Lock = 1;
  for (uint8_t i = 0; i < lenth; i++) {
    q->rear = (q->rear + 1) % QSIZE;
    q->arr[q->rear] = *(val + i);
    q->counter++;
  }
  q->RW_Lock = 0;
}

int UI_Pop(Queue_t *buffer1, uint8_t *data) {
  if (IsEmpty(buffer1)) return 0;
  if (buffer1->RW_Lock == 0) {
    for (uint8_t i = 0; i < 4; i++) {
      memcpy((char *)data + i, &buffer1->arr[buffer1->front], 1);
      buffer1->front = (buffer1->front + 1) % QSIZE;
      buffer1->counter--;
    }
  } else {
    return -1;
  }
  return 1;
}