//minus_os.c

#include<stdio.h>
#include "minus_os.h"

#define TIMER_SUPPORTED 1
#define TASKS_TOTAL 16
#define TIMERS_TOTAL 32


int idle_func(const void *para) {
  
  //  printf("idle_func\n");
    return 0;
}

struct minus_task idle_task=
{
	.task=&idle_func,
	.priority = 100,
	.para = (void*)&idle_task,
	.sched_count =-1,
	
};

#if TIMER_SUPPORTED
extern struct minus_task minus_timer_task;
#endif




//extern struct minus_task console_task;
extern struct minus_task minus_timer_task;

struct minus_task *minus_os_tasks[TASKS_TOTAL]=
{
 &idle_task,
 &minus_timer_task,
//	&console_task,
   
};

//need be called before minus_sched()
void minus_init()
{
	/*
	int i=0;
	for(i=0;i<sizeof(minus_os_tasks)/sizeof(struct minus_task*);i++)
	{
		if(minus_os_tasks[i] != 0)
		{
			if(minus_os_tasks[i]->init) (minus_os_tasks[i]->init)(minus_os_tasks[i]->init_data);
		}
	}
	*/
}

void minus_add_task(struct minus_task *task)
{
	int i=0;
	for(i=0;i<sizeof(minus_os_tasks)/sizeof(struct minus_task*);i++)
	{
		if(minus_os_tasks[i] == 0)
		{
			minus_os_tasks[i]=task;
			if(task->init) (task->init)(task->init_data);
			return;
		}
	}
	
	DEAD_LOOP;
}

struct minus_task *current;
struct minus_task *immediate =0;

void minus_sched()
{
	int i=0;
        
      //  printf("array size %d\n\r",sizeof(minus_os_tasks)/sizeof(struct minus_task*));

        while(1)
	{
		
                if(i == sizeof(minus_os_tasks)/sizeof(struct minus_task*)) i =0;
                
		if(immediate) { 
			current =immediate; immediate = 0;
			current->priority_count=0;

			//Restore the next entry or else it will be skipped
			if(i>0) i--; else i = sizeof(minus_os_tasks)/sizeof(struct minus_task*) -1;
                        
		} else
			current = minus_os_tasks[i];
	    i++;

	    if(current == 0) continue;
		
	    if(current->priority_count == 0) 
		{	
			current->priority_count=current->priority;

			if(current->sched_count>0)
			{
				current->status = TASK_RUNNING;
				(*(current->task))(current->para);
				current->sched_count --;
				if(current->sched_count>0) 
					current->status = TASK_WAITING; 
				else 
					current->status = TASK_STOPPED;
			}else if(current->sched_count<0)
			{
				current->status = TASK_RUNNING;
				(*(current->task))(current->para);
				current->status = TASK_WAITING; 
			}
		}else
		{
			current->priority_count --;
		}
            
           
	}
	
}




void minus_immediate(struct minus_task *task, void *msg) {
	if(immediate == 0){
		task->para = msg;
		immediate = task;
	}else if(task->sched_count>=0) {
		task->sched_count++;
	}
}




//Used for interrupt handler to notify to schedule the task 
void minus_sched_task(struct minus_task *task,void *msg) {
	
	if(task->sched_count>=0){
		task->para = msg;
		task->sched_count++;
	}		
	
}



/************************************************************/
//timer module
#if TIMER_SUPPORTED
volatile unsigned int high_jiffies=0;
volatile unsigned int jiffies=0;


struct minus_timer *minus_os_timers[TIMERS_TOTAL]=
{
   0
};



int minus_timer_callback(const void *para)
{
	
	
	struct minus_timer *entry;

	int i=0;

	for(i=0;i<sizeof(minus_os_timers)/sizeof(struct minus_timer*);i++)
	{
		entry = minus_os_timers[i];

		if(entry == 0) continue;

		if(entry->expires != 0)
		{
			if(entry->ticks>0) entry->ticks--;
			else
			{	
				
				//For one time timer
				if(entry->expires>0) {
					//stop the timer
					entry->expires =0;
		
					entry->ticks =0;
					}
				else {
				//Reenable the timer
				entry->ticks = 0-entry->expires;
				}
	
				//time out, now call the time out call back
				(*(entry->callback))(entry->data);
				
			}		
		}	
	
	}
	return 0;
}

struct minus_task minus_timer_task=
{
	.task=&minus_timer_callback,
	.priority = 1,
	.para = 0,
	.sched_count =1
};


void minus_add_timer(struct minus_timer *timer)
{
	int i=0;
	for(i=0;i<sizeof(minus_os_timers)/sizeof(struct minus_timer*);i++)
	{
		if(minus_os_timers[i] == 0)
		{
			minus_os_timers[i]=timer;
			return;
		}
	}
	
	DEAD_LOOP;
}

void minus_start_timer(struct minus_timer *timer, int expires) {
	
	if(expires>0)
		timer->ticks = expires;
	else
		timer->ticks = 0-expires;

	timer->expires = expires;
}

void minus_stop_timer(struct minus_timer *timer) {
	timer->expires = 0;	
}



//This function need be called by application's timer interrupt isr
void minus_tick(unsigned int data){

	jiffies++;
        
 	minus_sched_task(&minus_timer_task,(void*)data);
}

#endif









	









