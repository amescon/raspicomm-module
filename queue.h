
#define QUEUE_SIZE 256

struct queue
{
  int arr[QUEUE_SIZE];
  int rear, front;
};

int queue_get_room(struct queue* q);

int queue_is_empty(struct queue* q);

int queue_enqueue(struct queue* q, int v);

int queue_dequeue(struct queue* q);