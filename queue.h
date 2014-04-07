#define QUEUE_SIZE 256

typedef struct
{
  int arr[QUEUE_SIZE];
  int read, write;
} queue_t;

int queue_get_room(queue_t* queue);
int queue_enqueue(queue_t* queue, int item);
int queue_dequeue(queue_t* queue, int* item);
int queue_is_empty(queue_t* queue);
int queue_is_full(queue_t* queue);
