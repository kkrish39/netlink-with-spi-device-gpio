#include<linux/slab.h>

/*Node of the doubly-linked list*/
typedef struct circular_buf{
    long distance;
    long long timestamp;
    struct circular_buf *prev;
    struct circular_buf *next;
} circular_buf;

/*Max buffer size*/
static int bufferSize = 0;

/*Current size of the buffer */
static int numMeasurementsTaken = 0;


/*Resetting the buffer if the number of elements*/
void resetBuffer(void){
    numMeasurementsTaken = 0;
}

/*Function to delete the first node from the list */
circular_buf * deleteHead(circular_buf *list){
    circular_buf *temp = list;

    if(list == NULL){
        return NULL;
    }
    if(list->next == list){
        list = NULL;
        kfree(temp);
        numMeasurementsTaken --;
        return list;
    }
    list->next->prev = list->prev;
    list->prev->next = list->next;
    list = list->next;
    numMeasurementsTaken --;
    kfree(temp);
    return list;
}

/*Function to delete the last node from the list */
circular_buf * deleteTail(circular_buf *list){
    circular_buf *temp;

    if(list == NULL){
        return NULL;
    }
    temp = list->prev;
    if(list->prev == list){
        list = NULL;
        kfree(temp);
        numMeasurementsTaken --;
        return list;
    }

    list->prev->prev->next = list;
    list->prev = list->prev->prev;
    numMeasurementsTaken --;
    kfree(temp);
    return list;
}


/*Adding nodes to the FIFO Buffer. Add to the end of the list*/
circular_buf * addNode(circular_buf *list, circular_buf *node){
    if(numMeasurementsTaken == bufferSize){
        list = deleteHead(list);
    }
    if(list == NULL){
        list = node;
        list->prev = list;
        list->next = list;
    }
    else {
        node->next = list;
        node->prev = list->prev;
        list->prev->next = node;
        list->prev = node;
    }
    numMeasurementsTaken++;
    return list;
}

/*Initializing the node */
circular_buf * initNode(long distance, long long timestamp){
    circular_buf * node = kmalloc(sizeof(circular_buf), GFP_KERNEL);
    node->distance = distance;
    node->timestamp = timestamp;
    node->next=node->prev = NULL;
    return node;
}