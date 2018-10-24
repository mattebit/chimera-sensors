#ifndef QUEUE_H
#define QUEUE_H

// Implementazione dinamica
typedef struct
{
  int head, tail;
  int dim;
  char * elem[40];
  char stringa[50];
}queue;



enum retval { FAIL, OK };


void init (queue *);
int push(char *,queue *);
int pop(char  *,queue *);


#endif
