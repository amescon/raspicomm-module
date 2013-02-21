#include <linux/kernel.h>
#include "module.h"
#include "queue.h"

int queue_get_room(struct queue* q)
{
  if (q->rear >= q->front)
    return QUEUE_SIZE - (q->rear - q->front) - 1;
  else
    return q->front - q->rear - 1;
}

int queue_is_empty(struct queue* q)
{
  if (q->front == q->rear)
    return 1;
  else
    return 0;
}

int queue_enqueue(struct queue* q, int v)
{
  int t;

  t = (q->rear+1) % QUEUE_SIZE;

  if (t == q->front)
  {
    return -1;  
  }
  else
  {
    q->rear = t;
    q->arr[q->rear] = v;

    // #if DEBUG
    //   printk( KERN_INFO "raspicomm: queue_enqueue(): %X", v);
    // #endif

    return 0;
  }
}

int queue_dequeue(struct queue* q)
{
  if (queue_is_empty(q))
  {
    printk( KERN_ERR "raspicomm: error - underflow in queue_dequeue()" );
    return 0;
  }
  else
  {
    q->front = (q->front + 1) % QUEUE_SIZE;

    // #if DEBUG
    //   printk( KERN_INFO "raspicomm: queue_dequeue(): %X", q->arr[q->front] );
    // #endif

    return (q->arr[q->front]);
  }
}