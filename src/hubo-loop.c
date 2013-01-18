/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2012, Daniel M. Lofaro
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may
      be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <time.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

// for RT
#include <stdlib.h>
#include <sys/mman.h>

// for hubo
#include <hubo.h>

// for ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"

// For UDP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#define BUFLEN 512
#define NPACK 10
#define PORT 11000

/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

/* ... */

/* Somewhere in your app */

// Priority
#define MY_PRIORITY (49)/* we use 49 as the PRREMPT_RT use 50
			    as the priority of kernel tasklets
			    and interrupt handler by default */

#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
				   guaranteed safe to access without
				   faulting */


// Timing info
#define NSEC_PER_SEC    1000000000


struct timeb {
	time_t   time;
	unsigned short millitm;
	short    timezone;
	short    dstflag;
};



/* functions */
void stack_prefault(void);
static inline void tsnorm(struct timespec *ts);
void getMotorPosFrame(int motor, struct can_frame *frame);
//void huboLoop(struct hubo_param *h);
void huboLoop();
int ftime(struct timeb *tp);

int udp(void);
void diep(char *s);
void combine(double* absoluteJointValues, double* relativeJointValues, double* jointValues);







// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_hubo_ref;      // hubo-ach
ach_channel_t chan_hubo_init_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state
ach_channel_t chan_hubo_param;    // hubo-ach-param

int debug = 0;
int hubo_debug = 0;

/* Deg 2 rad */
double d2r(double rad) { return M_PI*rad/180.0; }


void diep(char *s){
   perror(s);
   exit(1);
}//void huboLoop(struct hubo_param *H_param) {



void huboLoop() {
	// get initial values for hubo
	struct hubo_ref H_ref;
	struct hubo_state H_state;
	memset( &H_ref,   0, sizeof(H_ref));
	memset( &H_state, 0, sizeof(H_state));

	size_t fs;
	//int r = ach_get( &chan_hubo_ref, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	//assert( sizeof(H) == fs );
	int r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_COPY );
	if(ACH_OK != r) {
		if(hubo_debug) {
			printf("Ref ini r = %s\n",ach_result_to_string(r));}
		}
	else{   assert( sizeof(H_ref) == fs ); }

	r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_COPY );
	if(ACH_OK != r) {
		if(hubo_debug) {
			printf("State ini r = %s\n",ach_result_to_string(r));}
		}
	else{
		assert( sizeof(H_state) == fs );
	 }


/* Open UPD */
	struct sockaddr_in si_me, si_other;
	    int s, i, slen=sizeof(si_other);
	    char buf[BUFLEN];

	    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
	      diep("socket");

	    memset((char *) &si_me, 0, sizeof(si_me));
	    si_me.sin_family = AF_INET;
	    si_me.sin_port = htons(PORT);
	    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	    if (bind(s, &si_me, sizeof(si_me))==-1)
	        diep("bind");
	printf("\n just made UDP \n");


	// time info
	struct timespec t;
	int interval = 1000000000; // 1hz (1.0 sec)
	//int interval = 500000000; // 2hz (0.5 sec)
	//int interval = 10000000; // 100 hz (0.01 sec)
	//int interval = 5000000; // 200 hz (0.005 sec)
	//int interval = 2000000; // 500 hz (0.002 sec)


	/* Sampling Period */
	double T = (double)interval/(double)NSEC_PER_SEC; // (sec)

	// get current time
	//clock_gettime( CLOCK_MONOTONIC,&t);
	clock_gettime( 0,&t);
	double theVal = 0.0;
	double theTarget = -0.3;

	double jntRef[HUBO_JOINT_COUNT];
	int mode[HUBO_JOINT_COUNT];
	memset( &jntRef, 0, sizeof(jntRef));
	memset( &mode, 0, sizeof(mode));
        
        // Added 1/17/13 Alyssa Batula
        double pipeBaseRightJointRef[6];        // Base position for right arm [RSP, RSR, RSY, REB, RWY, RWP]
        double pipeBaseLeftJointRef[6];         // Base position for left arm [LSP, LSR, LSY, LEB, LWY, LWP]
        double pipe0JointRef[6];                // Pipe left out relative joint movement (degrees)
        double pipe1JointRef[6];                // Pipe left in relative joint movement (degrees)
        double pipe2JointRef[6];                // Pipe right in relative joint movement (degrees)
        double pipe3JointRef[6];                // Pipe right out relative joint movement (degrees)
        
        //Set the default values for pipe positions
        pipeBaseRightJointRef[0] = d2r(-25.0);
        pipeBaseRightJointRef[1] = d2r(-8.0);
        pipeBaseRightJointRef[2] = d2r(-1.0);
        pipeBaseRightJointRef[3] = d2r(-40.0);
        pipeBaseRightJointRef[4] = d2r(22.0);
        pipeBaseRightJointRef[5] = d2r(-58.0);
        
        pipeBaseLeftJointRef[0] = d2r(-25.0);
        pipeBaseLeftJointRef[1] = d2r(8.0);
        pipeBaseLeftJointRef[2] = d2r(1.0);
        pipeBaseLeftJointRef[3] = d2r(-40.0);
        pipeBaseLeftJointRef[4] = d2r(-22.0);
        pipeBaseLeftJointRef[5] = d2r(58.0);
        
        // Left out
        pipe0JointRef[0] = d2r(6.0);
        pipe0JointRef[1] = d2r(4.0);
        pipe0JointRef[2] = d2r(4.0);
        pipe0JointRef[3] = d2r(5.0);
        pipe0JointRef[4] = d2r(0.0);
        pipe0JointRef[5] = d2r(0.0);
        
        // Left in
        pipe1JointRef[0] = d2r(3.5);
        pipe1JointRef[1] = d2r(-6.0);
        pipe1JointRef[2] = d2r(-5.0);
        pipe1JointRef[3] = d2r(3.5);
        pipe1JointRef[4] = d2r(0.0);
        pipe1JointRef[5] = d2r(0.0);
        
        // Right in
        pipe2JointRef[0] = d2r(3.5);
        pipe2JointRef[1] = d2r(6.0);
        pipe2JointRef[2] = d2r(5.0);
        pipe2JointRef[3] = d2r(3.5);
        pipe2JointRef[4] = d2r(0.0);
        pipe2JointRef[5] = d2r(0.0);
        
        // Right Out
        pipe3JointRef[0] = d2r(6.0);
        pipe3JointRef[1] = d2r(-4.0);
        pipe3JointRef[2] = d2r(-4.0);
        pipe3JointRef[3] = d2r(5.0);
        pipe3JointRef[4] = d2r(0.0);
        pipe3JointRef[5] = d2r(0.0);
        
	H_ref.ref[RSP] = pipeBaseRightJointRef[0];
        H_ref.ref[RSR] = pipeBaseRightJointRef[1];
        H_ref.ref[RSY] = pipeBaseRightJointRef[2];
        H_ref.ref[REB] = pipeBaseRightJointRef[3];
        H_ref.ref[RWY] = pipeBaseRightJointRef[4];
        H_ref.ref[RWP] = pipeBaseRightJointRef[5];
        
        H_ref.ref[LSP] = pipeBaseLeftJointRef[0];
        H_ref.ref[LSR] = pipeBaseLeftJointRef[1];
        H_ref.ref[LSY] = pipeBaseLeftJointRef[2];
        H_ref.ref[LEB] = pipeBaseLeftJointRef[3];
        H_ref.ref[LWY] = pipeBaseLeftJointRef[4];
        H_ref.ref[LWP] = pipeBaseLeftJointRef[5];


	memcpy(&H_ref.mode, &mode, sizeof(mode));
	ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));

	sleep(1.0);

	int state = 0;

	while(1) {
		// wait until next shot
//		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

		/* Get latest ACH message */
		r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
			if(hubo_debug) {
				printf("Ref r = %s\n",ach_result_to_string(r));}
			}
		else{   assert( sizeof(H_ref) == fs ); }
		r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_COPY );
		if(ACH_OK != r) {
			if(hubo_debug) {
				printf("State r = %s\n",ach_result_to_string(r));}
			}
		else{   assert( sizeof(H_state) == fs ); }

	double tmpRefR[6];
	double tmpRefL[6];
	memset(&tmpRefR, 0, sizeof(tmpRefR));
	memset(&tmpRefL, 0, sizeof(tmpRefL));



// ------------------------------------------------------------------------------
// ---------------[ DO NOT EDIT AVBOE THIS LINE]---------------------------------
// ------------------------------------------------------------------------------
	/* Wait for UPD */
	printf("\n Wait for UDP \n");
	if (recvfrom(s, buf, BUFLEN, 0, &si_other, &slen)==-1)
        	diep("recvfrom()");
        printf("\n%u %u %u\n",buf[0], buf[1], buf[2]);
//------------------------------------------------------------------------------
//+++++++++++[Will run after UDP is received]+++++++++++++++++++++++++++++++++++
//------------------------------------------------------------------------------
	/* state machine */
	if( 1 == buf[0] ) {	// Hit a pipe
		if( 0 == buf[1] ) { // down pipe 0
			
			combine( &pipeBaseLeftJointRef, &pipe0JointRef, &tmpRefL);

			H_ref.ref[LSP] = tmpRefL[0];
			H_ref.ref[LSR] = tmpRefL[1];
			H_ref.ref[LSY] = tmpRefL[2];
			H_ref.ref[LEB] = tmpRefL[3];
			H_ref.ref[LWY] = tmpRefL[4];
			H_ref.ref[LWP] = tmpRefL[5];
		}
		else if( 1 == buf[1] ) { // down pipe 1
			
			combine( &pipeBaseLeftJointRef, &pipe1JointRef, &tmpRefL);

			H_ref.ref[LSP] = tmpRefL[0];
			H_ref.ref[LSR] = tmpRefL[1];
			H_ref.ref[LSY] = tmpRefL[2];
			H_ref.ref[LEB] = tmpRefL[3];
			H_ref.ref[LWY] = tmpRefL[4];
			H_ref.ref[LWP] = tmpRefL[5];
		} 

		else if( 2 == buf[1] ) { // down pipe 2
			
			combine( &pipeBaseRightJointRef, &pipe2JointRef, &tmpRefR);

			H_ref.ref[RSP] = tmpRefR[0];
			H_ref.ref[RSR] = tmpRefR[1];
			H_ref.ref[RSY] = tmpRefR[2];
			H_ref.ref[REB] = tmpRefR[3];
			H_ref.ref[RWY] = tmpRefR[4];
			H_ref.ref[RWP] = tmpRefR[5];
		} 
		else if( 3 == buf[1] ) { // down pipe 2
			
			combine( &pipeBaseRightJointRef, &pipe3JointRef, &tmpRefR);

			H_ref.ref[RSP] = tmpRefR[0];
			H_ref.ref[RSR] = tmpRefR[1];
			H_ref.ref[RSY] = tmpRefR[2];
			H_ref.ref[REB] = tmpRefR[3];
			H_ref.ref[RWY] = tmpRefR[4];
			H_ref.ref[RWP] = tmpRefR[5];
		} 

        	
		ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));
		usleep(50000);		// sleep for x microseconds 1,000,000 per sec wait 0.05sec

                
		H_ref.ref[RSP] = pipeBaseRightJointRef[0];
		H_ref.ref[RSR] = pipeBaseRightJointRef[1];
		H_ref.ref[RSY] = pipeBaseRightJointRef[2];
		H_ref.ref[REB] = pipeBaseRightJointRef[3];
		H_ref.ref[RWY] = pipeBaseRightJointRef[4];
		H_ref.ref[RWP] = pipeBaseRightJointRef[5];
		
		H_ref.ref[LSP] = pipeBaseLeftJointRef[0];
                H_ref.ref[LSR] = pipeBaseLeftJointRef[1];
                H_ref.ref[LSY] = pipeBaseLeftJointRef[2];
                H_ref.ref[LEB] = pipeBaseLeftJointRef[3];
                H_ref.ref[LWY] = pipeBaseLeftJointRef[4];
                H_ref.ref[LWP] = pipeBaseLeftJointRef[5];

	}

	

//[RSP, RSR, RSY, REB, RWY, RWP

// ------------------------------------------------------------------------------
// ---------------[ DO NOT EDIT BELOW THIS LINE]---------------------------------
// ------------------------------------------------------------------------------
		ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));
//		t.tv_nsec+=interval;
//		tsnorm(&t);
	}


}






void stack_prefault(void) {
	unsigned char dummy[MAX_SAFE_STACK];
	memset( dummy, 0, MAX_SAFE_STACK );
}



static inline void tsnorm(struct timespec *ts){

//	clock_nanosleep( NSEC_PER_SEC, TIMER_ABSTIME, ts, NULL);
	// calculates the next shot
	while (ts->tv_nsec >= NSEC_PER_SEC) {
		//usleep(100);	// sleep for 100us (1us = 1/1,000,000 sec)
		ts->tv_nsec -= NSEC_PER_SEC;
		ts->tv_sec++;
	}
}


void combine(double* absoluteJointValues, double* relativeJointValues, double* jointValues)
{
    int i;

    for (i = 0; i < 6; i++)
        jointValues[i] = absoluteJointValues[i] + absoluteJointValues[i];
}


int udp(void) {
    struct sockaddr_in si_me, si_other;
    int s, i, slen=sizeof(si_other);
    char buf[BUFLEN];

    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
      diep("socket");

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, &si_me, sizeof(si_me))==-1)
        diep("bind");
while(1){
//    for (i=0; i<NPACK; i++) {
      if (recvfrom(s, buf, BUFLEN, 0, &si_other, &slen)==-1)
        diep("recvfrom()");
//      printf("Received packet from %s:%d\nData: %s\n\n", 
//             inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port), buf);
		printf("\n%d %d %d\n",buf[0], buf[1], buf[2]);
//	}	
}

    close(s);
    return 0;

}



int main(int argc, char **argv) {

	int vflag = 0;
	int c;


	char* ach_chan = HUBO_CHAN_REF_FILTER_NAME;
	int i = 1;
	while(argc > i) {
		if(strcmp(argv[i], "-d") == 0) {
			debug = 1;
		}
		if(strcmp(argv[i], "-r") == 0) {
			ach_chan = HUBO_CHAN_REF_NAME;
		}
		i++;
	}

	/* RT */
	struct sched_param param;
	/* Declare ourself as a real time task */
	param.sched_priority = MY_PRIORITY;
	if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
		perror("sched_setscheduler failed");
		exit(-1);
	}

	/* Lock memory */
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
		perror("mlockall failed");
		exit(-2);
	}

	/* Pre-fault our stack */
	stack_prefault();


	/* open ach channel */ 
	//int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
	int r = ach_open(&chan_hubo_ref, ach_chan , NULL);
	assert( ACH_OK == r );

	r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
	assert( ACH_OK == r );


        // get initial values for hubo
        struct hubo_ref H_ref;
        struct hubo_init_cmd H_init;
        struct hubo_state H_state;
        struct hubo_param H_param;
        memset( &H_ref,   0, sizeof(H_ref));
        memset( &H_init,  0, sizeof(H_init));
        memset( &H_state, 0, sizeof(H_state));
        memset( &H_param, 0, sizeof(H_param));

        usleep(250000);

        // set default values for Hubo
        //setJointParams(&H_param, &H_state);

        //huboLoop(&H_param);
//	while(1) { udp();}
//	udp();
        huboLoop();

	pause();
	return 0;

}
