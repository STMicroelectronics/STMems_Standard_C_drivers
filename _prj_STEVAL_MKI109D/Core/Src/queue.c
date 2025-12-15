/**
  ******************************************************************************
  * @file    queue.c
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <stdint.h>
#include <stdio.h>
#include "queue.h"

static inline msg_queue_status msg_enqueue_one(msg_queue *queue, uint8_t element);
static inline msg_queue_status msg_dequeue_one(msg_queue *queue, uint8_t *element);

uint32_t msg_queue_get_size(msg_queue *queue)
{
  uint32_t size;
  if (queue->_tail >= queue->_head) {
    size = queue->_tail - queue->_head;
  } else {
    size = MSG_QUEUE_ALLOCATED_SIZE - queue->_head + queue->_tail;
  }

  return size;
}

uint8_t msg_queue_is_empty(msg_queue *queue)
{
  return msg_queue_get_size(queue) == 0;
}

uint8_t msg_queue_is_full(msg_queue *queue)
{
  return msg_queue_get_size(queue) == MSG_QUEUE_MAX_CAPACITY;
}

static inline msg_queue_status msg_enqueue_one(msg_queue *queue, uint8_t element)
{
  msg_queue_status error_code = MSG_QUEUE_OK;

  if (msg_queue_is_full(queue)) {
    error_code = MSG_QUEUE_ERROR;
  } else {
    if (queue->_tail >= MSG_QUEUE_MAX_CAPACITY) {
      queue->_msg_queue[queue->_tail] = element;
      queue->_tail = 0;
    } else {
      queue->_msg_queue[queue->_tail++] = element;
    }
  }

  return error_code;
}

uint32_t msg_enqueue(msg_queue *queue, uint8_t *buf, uint32_t size)
{
  uint32_t count = 0;

  for (uint32_t idx = 0; idx < size; idx++) {
    if (msg_enqueue_one(queue, buf[idx]) == MSG_QUEUE_OK) {
      count++;
    } else {
      break;
    }
  }

  return count;
}

static inline msg_queue_status msg_dequeue_one(msg_queue *queue, uint8_t *element)
{
  msg_queue_status error_code = MSG_QUEUE_OK;

  if (msg_queue_is_empty(queue)) {
    error_code = MSG_QUEUE_ERROR;
  } else {
    *element = queue->_msg_queue[queue->_head];
    if (queue->_head >= MSG_QUEUE_MAX_CAPACITY) {
      queue->_head = 0;
    } else {
      queue->_head++;
    }
  }

  return error_code;
}

uint32_t msg_dequeue(msg_queue *queue, uint8_t *buf, uint32_t size)
{
  uint32_t count = 0;

  for (uint32_t idx = 0; idx < size; idx++) {
    if (msg_dequeue_one(queue, &buf[idx]) == MSG_QUEUE_OK) {
      count++;
    } else {
      break;
    }
  }

  return count;
}

uint32_t str_msg_dequeue(msg_queue *queue, uint8_t *str_buf, uint32_t str_buf_max_size)
{
  uint32_t count = 0;

  for (uint32_t idx = 0; idx < str_buf_max_size - 1; idx++) {
    if (msg_dequeue_one(queue, &str_buf[idx]) == MSG_QUEUE_OK && str_buf[idx] != '\0') {
      count++;
    } else {
      break;
    }
  }
  if (str_buf[count] != '\0')
    str_buf[count] = '\0';
  count++;

  return count;
}

msg_queue_status msg_queue_get(msg_queue *queue, uint8_t * dst, uint32_t idx)
{
  msg_queue_status err_code = MSG_QUEUE_OK;
  uint32_t phy_idx = 0;

  if (msg_queue_is_empty(queue) || idx > (msg_queue_get_size(queue)  - 1)) {
    err_code = MSG_QUEUE_ERROR;
  } else {
    phy_idx = (queue->_head + idx) % MSG_QUEUE_ALLOCATED_SIZE;
    *dst = queue->_msg_queue[phy_idx];
  }

  return err_code;
}

void msg_queue_to_str(msg_queue *queue, char *str_buf, uint32_t str_buf_max_size)
{
  snprintf(str_buf,
          str_buf_max_size,
          "msg_queue:\n\thead: %d\n\ttail: %d\n\tsize: %d\n",
          queue->_head, queue->_tail, msg_queue_get_size(queue));
}

uint32_t msg_queue_to_arr(msg_queue *queue, uint8_t *arr, uint32_t arr_max_size)
{
  uint32_t size = msg_queue_get_size(queue);
  size = arr_max_size >= size ? size : arr_max_size;
  for (int i = 0; i < size; i++) {
    msg_queue_get(queue, &arr[i], i);
  }

  return size;
}

void msg_queue_flush(msg_queue *queue)
{
  queue->_head = 0;
  queue->_tail = 0;
}
