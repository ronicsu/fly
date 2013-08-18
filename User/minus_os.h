#ifndef _MINUS_OS_H_
#define _MINUS_OS_H_



//task module code



#define	TASK_RUNNING 1 
#define TASK_WAITING 2
#define TASK_STOPPED 3
#define TASK_DEAD 4

#define POOLING_ENABLE 1

#define ERROR(ERROR_STR,ERROR_ID)	
#define ALERT(ALERT_STR,ALERT_ID)
#define DEAD_LOOP while(1)


typedef int (*minus_task_callback)(const void*);

struct minus_task
{
	minus_task_callback init;
	void *init_data;
	minus_task_callback task;
	void *para;
	unsigned int priority;/* user can use the  priority should > 1, the bigger value, the lower priority*/
	unsigned int priority_count;

	volatile int sched_count; /*if shed_count<0, it is a loop task. else it will be scheduled shed_count times*/

	unsigned int status;
	 
};

extern struct minus_task *current;

#define ENTER_CRITICAL
#define EXIT_CRITICAL

#define ISRABLE

void minus_init(void);

void minus_sched(void);

void minus_add_task(struct minus_task *task);
//ISRABLE


void minus_immediate(struct minus_task *task, void *msg) ;

//ISRABLE 
void minus_sched_task(struct minus_task *task, void *msg);



extern volatile unsigned int jiffies;
static const unsigned int HZ = 1000;

//This function need be called by app's timer isr
void minus_tick( unsigned int data);

struct minus_timer
{
	int expires;
	void (*callback)(unsigned long);
	unsigned long data;
	unsigned int ticks;
};


void minus_add_timer(struct minus_timer *timer);

//ISRABLE
void minus_start_timer(struct minus_timer *timer, int expires) ;

//ISRABLE
void minus_stop_timer(struct minus_timer *timer) ;





#endif

