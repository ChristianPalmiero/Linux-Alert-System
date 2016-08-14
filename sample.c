/*
 * sample.c - The simplest loadable kernel module.
 * Intended as a template for development of more
 * meaningful kernel modules.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/ktime.h>
#include <asm/uaccess.h>

/*
 * Definition of the GPIOs we want to use
*/
#define	TRIG	131		// HCSR04 Trig, D7: PI.3
#define	ECHO	130		// HCSR04 Echo, D8: PI.2
#define LED	129		// User led, PI.1
/*
 * Definition of ioctl commands
*/
#define INVALID         -1
#define WRITE_TRIG       0
#define WRITE_LED        1
/*
 * Device major number
 */
static uint module_major = 166;
/*
 * Device name
 */
static char * module_name = "sample";
/*
 * Device semaphore
 */
static struct semaphore sem;
/*
 * Device access lock. Only one process can access the driver at a time
 */
static int sample_lock = 0;
/*
 * Device variables
*/
static int	trig_period; 	// TRIG period in milliseconds
static int	blk_period; 	// LED blinking period in milliseconds
ktime_t 	before, after;
s64 		actual_time;
int 		flag = 0;
int		mode = INVALID;	// device mode: INVALID = do nothing
				// WRITE_LED = next write sets the blinking period
				// WRITE_TRIG = next write handles the trigger signal behaviour
/*
 * Declare the workqueues
 */
static struct workqueue_struct *my_wq;
static struct workqueue_struct *trig_wq;

typedef struct {
	struct work_struct my_work;
} my_work_t;

typedef struct {
	struct work_struct trig_work;
	int	mshigh, mslow;
} trig_work_t;

static my_work_t work;
static trig_work_t t_work;

/*
 * Led Work function
 */
static void my_wq_function( struct work_struct *work )
{
	my_work_t *my_work;

	my_work = (my_work_t *)work;
	
	gpio_set_value( LED, 0 );
	while(1)
	{
		if(blk_period==0){
			gpio_set_value( LED, 1 );   //LED on
			msleep(1);	//Without this msleep, the CPU resources would be consumed by this work
		}
		else{
			gpio_set_value( LED, 1 );   //LED on
			msleep( blk_period/2);
			gpio_set_value( LED, 0 );   //LED off
			msleep( blk_period/2);
		}		
	}
}

/*
 * Trigger work function
 */
static void trig_wq_function( struct work_struct *work )
{
	int	mshigh, mslow;
	trig_work_t *trig_work;

	trig_work = (trig_work_t *)work;
	mshigh = trig_work->mshigh;
	mslow = trig_work->mslow;	

	//The trigger signal is a square wave with a fixed period, equal to mshigh+mslow
	gpio_set_value( TRIG, 0 );
	while(1)
	{
		gpio_set_value( TRIG, 1 );
		msleep(mshigh);
		gpio_set_value( TRIG, 0 );
		msleep(mslow);
	}
}

/*
 * ECHO interrupt handler
*/

static irq_handler_t ECHO_handler( unsigned int irq, struct pt_regs *regs )
{
	int ECHO_value;

	ECHO_value = gpio_get_value( ECHO );
	if(ECHO_value)
		before=ktime_get();	//Rising edge
	else{
		after=ktime_get();	//Falling edge
		actual_time=ktime_to_us(ktime_sub(after,before));	//The width of the Echo pulse in microseconds is collected
		up(&sem);		//The read function is now ready to run
	}
	return (irq_handler_t)IRQ_HANDLED;
}

/*
 * Device open
 */
static int sample_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	/*
	 * One process at a time
	 */
	if (sample_lock > 0) 
	{
		ret = -EBUSY;
	}
	else
	{
		sample_lock++;

		/*
 	 	* Increment the module use counter
 	 	*/
		try_module_get(THIS_MODULE);

		#ifdef SAMPLE_DEBUG 
			printk( KERN_INFO "%s: %s\n", module_name, __func__ ); 
		#endif
	}

	return( ret );
}

/*
 * Device close
 */
static int sample_release(struct inode *inode, struct file *file)
{
	/*
 	 * Release device
 	 */
	sample_lock = 0;

	/*
 	 * Decrement module use counter
 	 */
	module_put(THIS_MODULE);

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( 0 );
}

/* 
 * Device read
 */
static ssize_t sample_read(struct file *filp, char *buffer,
			 size_t length, loff_t * offset)
{
	int ret = 1;
	down(&sem);
	sprintf(buffer,"%d",(int)actual_time);	//The width of the Echo pulse is moved to the user space

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( ret );
}

/* 
 * Device write
 */
static ssize_t sample_write(struct file *filp, const char *buffer,
			  size_t length, loff_t * offset)
{
	int ret = 0;

	switch( mode )
	{
		case WRITE_TRIG:
			trig_period=(int)*buffer;
			//Thigh and Tlow for the trigger signal are set
			t_work.mshigh = trig_period/100;
			if(t_work.mshigh<1)
				t_work.mshigh=1;
			t_work.mslow = trig_period-t_work.mshigh;
			
			queue_work( trig_wq, (struct work_struct *)&t_work );
			
			ret = trig_period;
			break;
		case WRITE_LED:
			memcpy(&blk_period,buffer,sizeof(blk_period));	//The user led blinking period is updated
			
			if(flag==0){
				queue_work( my_wq, (struct work_struct *)&work );
				flag=1;	//The work is queued only the first time we are in the WRITE_LED case
			}

			ret = blk_period;
			break;
		default:
			ret = 0;
			break;
	}

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( ret );
}

static ssize_t sample_ioctl(struct inode *inode, struct file *filep, 
			    const unsigned int cmd, const unsigned long arg)
{
	int ret = 0;
#define WRITE_TRIG	0
#define	WRITE_LED	1

	switch( cmd )
	{
		case WRITE_TRIG:
			mode = WRITE_TRIG;	// The next write will handle the trig signal
			break;
		case WRITE_LED:
			mode = WRITE_LED;	// The next write will start blinking the led with a fixed period
			break;
		default:
			mode = INVALID;
			break;
	}

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( ret );
}


/*
 * Device operations
 */
static struct file_operations sample_fops = {
	.read = sample_read,
	.write = sample_write,
	.open = sample_open,
	.release = sample_release,
	.ioctl = sample_ioctl
};

static int __init sample_init_module(void)
{
	/*
 	 * Register device
 	 */
	int	ret;

	ret = register_chrdev(module_major, module_name, &sample_fops);
	if (ret < 0) {
		printk(KERN_INFO "%s: registering device %s with major %d failed with %d\n",
		       __func__, module_name, module_major, module_major );
		return( ret );
	}
	else
	{
		printk(KERN_INFO "%s: registering device %s with major %d\n",
		       __func__, module_name, module_major );

		/*
 		 * Reserve gpios TRIG (as output, with default output value set to 0), ECHO (as input) and LED (as output, with default output value set to 0)
		*/
		if( gpio_request( TRIG, module_name ) )	// Check if TRIG is available
		{
			printk( KERN_INFO "%s: %s unable to get TRIG gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( gpio_request( ECHO, module_name ) )	// Check if ECHO is available
		{
			printk( KERN_INFO "%s: %s unable to get ECHO gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( gpio_request( LED, module_name ) )	// Check if LED is available
		{
			printk( KERN_INFO "%s: %s unable to get LED gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( gpio_direction_output( TRIG, 0 ) < 0 )	// Set TRIG gpio as output with default value 0
		{
			printk( KERN_INFO "%s: %s unable to set TRIG gpio as output\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( gpio_direction_input( ECHO ) < 0 )		// Set ECHO gpio as input
		{
			printk( KERN_INFO "%s: %s unable to set ECHO gpio as input\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( gpio_direction_output( LED, 0 ) < 0 )	// Set LED gpio as output with default value 0
		{
			printk( KERN_INFO "%s: %s unable to set LED gpio as output\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( request_irq( gpio_to_irq( ECHO ),       // Register the interrupt handler for the ECHO pin
                                 (irq_handler_t) ECHO_handler,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING ,
				  module_name,
				  NULL ) < 0 )
		{
			printk( KERN_INFO "%s: %s unable to register gpio irq for ECHO\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		my_wq = create_workqueue( "led_queue" );    // Create the LED workqueue
		if( my_wq )
		{
			INIT_WORK( (struct work_struct *)&work, my_wq_function );
		}

		trig_wq = create_workqueue( "trig_queue" ); // Create the TRIGGER workqueue
		if( trig_wq )
		{
			INIT_WORK( (struct work_struct *)&t_work, trig_wq_function );
		}
		
		sema_init(&sem,0);
	}
	
	return( ret );
}

static void __exit sample_cleanup_module(void)
{
	/*
	 * Free irq
	 */
	free_irq( gpio_to_irq( ECHO ), NULL );

	/*
	 * Release the gpios
	 */
	gpio_free( TRIG );
	gpio_free( ECHO );
	gpio_free( LED );

	/*
	 * Unregister device
	 */
	unregister_chrdev(module_major, module_name);

	printk(KERN_INFO "%s: unregistering %s done\n", __func__, module_name );
}

module_init(sample_init_module);
module_exit(sample_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Christian Palmiero, christian.palmiero@studenti.polito.it");
MODULE_DESCRIPTION("Device Driver Example 1");

