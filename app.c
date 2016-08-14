/*
 * This is a user-space application that implements a
 * proximity alert system
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define INVALID         -1
#define WRITE_TRIG       0
#define WRITE_LED        1
#define N		 6

int main(int argc, char **argv)
{
	char *app_name = argv[0];
	char *dev_name = "/dev/sample";
	int fd = -1;
	char c[15];
	int x, measurement_index=0;
	int blk_period=-1, period=-1, trig_period=67;	//trigger period in ms
	float distance, first_distance, second_distance, avg;

	if( argc != 1 )
	{
		printf( "Usage: launch %s and observe how the alert system works\n", argv[0] );
		return 1;
	}

	/*
 	 * Open the sample device RD | WR
 	 */
	if ((fd = open(dev_name, O_RDWR)) < 0) {
		fprintf(stderr, "%s: unable to open %s: %s\n", 
			app_name, dev_name, strerror(errno));
		return( 1 );
	}

	/*
	 * Set the trig period
	*/
	ioctl( fd, WRITE_TRIG, NULL );	        
	x = write(fd, &trig_period, sizeof(trig_period));
	
	while(1)
	{
		/*
		 * Read the distance
		*/
		x = read( fd, c, 1 );
		distance = (float)(1.0*atoi(c)/58);
		
		/*
		 * Round the distance if it exceeds the sensor ranging distance, that goes from
		 * 2 cm to 400 cm
		*/				
		if(distance<2)
			distance=2;
		else if(distance>400)
			distance=400;

		/*
		 * Two measurements must be computed and, then, their average is calculated,
		 * in order to reach a better accuracy
		*/		
		if(measurement_index++==0)
			first_distance=distance;
		else{
			second_distance=distance;
			measurement_index=0;

			avg=(first_distance+second_distance)/2;
			printf("Avg distance: %f [cm]\n", avg);
			
			/*
		 	* The next blinking period in ms is computed, according to the assignment constraints
			*/
			if(avg<10)
				period=0;
			else if(avg>=10 && avg<25)
				period=400;
			else if(avg>=25 && avg<50)
				period=600;
			else if(avg>=50 && avg<75)
				period=800;
			else if(avg>=75 && avg<=100)
				period=1000;
			else if(avg>100)
				period=2000;
			else
				period=-1;
			
			/*
			 * If the next blinking period is different from the current one, the user
			 * LED blinking period is updated through ioctl+write
			*/
			if(blk_period!=period){
				blk_period=period;
				printf("Period: %d [ms]\n", blk_period);
				ioctl( fd, WRITE_LED, NULL );	        
				x = write(fd, &blk_period, sizeof(blk_period));
			}
		}

	};

	if (fd >= 0) {
		close(fd);
	}
	return( 0 );
}
