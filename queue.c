#include <linux/kernel.h>
#include "module.h"
#include "queue.h"

int queue_get_room(queue_t* queue)
{
  /* cache read / write for thread safety as they might be modified during the evaluation */
  int read, write;
  read = queue->read;
  write = queue->write;

  return (read > write) ? (read - write - 1) : (QUEUE_SIZE - 1 - write + read);
}

int queue_is_full(queue_t* queue)
{
  return (((queue->write + 1) % QUEUE_SIZE) == queue->read);
}

int queue_is_empty(queue_t* queue)
{
  return (queue->write == queue->read);
}

int queue_enqueue(queue_t* queue, int item)
{
  if (queue_is_full(queue)) {
    return 0;
  }
  else {
    queue->arr[queue->write] = item;
    queue->write = ((queue->write + 1) == QUEUE_SIZE) ? 0 : queue->write + 1;
    return 1;
  }
}

int queue_dequeue(queue_t* queue, int* item)
{
  if (queue_is_empty(queue)) {
    return 0;
  } 
  else {
    *item = queue->arr[queue->read];
    queue->read = ((queue->read + 1) == QUEUE_SIZE) ? 0 : queue->read + 1;
    return 1;
  }
}
