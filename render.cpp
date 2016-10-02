/*
 ____  _____ _        _    
| __ )| ____| |      / \   
|  _ \|  _| | |     / _ \  
| |_) | |___| |___ / ___ \ 
|____/|_____|_____/_/   \_\

The platform for ultra-low latency audio and sensor processing

http://bela.io

A project of the Augmented Instruments Laboratory within the
Centre for Digital Music at Queen Mary University of London.
http://www.eecs.qmul.ac.uk/~andrewm

(c) 2016 Augmented Instruments Laboratory: Andrew McPherson,
  Astrid Bin, Liam Donovan, Christian Heinrichs, Robert Jack,
  Giulio Moro, Laurel Pardue, Victor Zappi. All rights reserved.

The Bela software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt
*/


#include <Bela.h>
#include <cmath>
#include <rtdk.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_BUFFER_SIZE 8

int _handle;
unsigned char serialBuffer[SERIAL_BUFFER_SIZE];

float freq, amp, phase, azimuth;

float left_gain(float azi)
{
    // Assuming an azimuth measured in 256 units around the full circle
    return sinf(2.0f * M_PI * azi / 256.0f);
}


float right_gain(float azi)
{
    // Assuming an azimuth measured in 256 units around the full circle
    return cosf(2.0f * M_PI * azi / 256.0f);
}


int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
			return -1;
    }

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}


void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		printf("Error tcsetattr: %s\n", strerror(errno));
	}


int initSerial(const char *portname, int speed)
{
    printf ("Attempting to connect to %s\n", portname);
	_handle = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (_handle < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	} else {
		printf ("Successfully opened %s with file descriptor %d\n", portname, _handle);
	}

	set_interface_attribs(_handle, speed);
	set_mincount(_handle, 0);                /* set to pure timed read */
    return 0;
}

void readSerialPort() 
{
    int rdlen;
	rdlen = read(_handle, serialBuffer, SERIAL_BUFFER_SIZE);
	if (rdlen > 0) {
		azimuth = (float)serialBuffer[rdlen-1];
	} 
	else if (rdlen < 0) 
        printf("Error from read: %d: %s\n", rdlen, strerror(errno));
}



AuxiliaryTask ttyTask;		// Auxiliary task to read serial data

bool setup(BelaContext *context, void *userData)
{
    freq = 256.0f;
    amp = 0.1f;
    phase = 0.0f;
    
    initSerial("/dev/ttyACM0", B9600);
    ttyTask = Bela_createAuxiliaryTask(readSerialPort, 50, "bela-arduino-comms");
	return true;
}


void render(BelaContext *context, void *userData)
{
	// loop through frames in block
	for(unsigned n = 0; n < context->audioFrames; n++) {

		if (n == 0)
		    Bela_scheduleAuxiliaryTask(ttyTask);
		
        float sample = 0.1f * ((float)rand()) / ((float)RAND_MAX);
        
        if (context->audioOutChannels == 2) {
            context->audioOut[context->audioOutChannels * n] = left_gain(azimuth) * sample;
            context->audioOut[context->audioOutChannels * n + 1] = right_gain(azimuth) * sample;
        } else {
    		for(unsigned ch = 0; ch < context->audioOutChannels; ch++)
	    		context->audioOut[context->audioOutChannels * n + ch] = sample;
        }
	}
}


void cleanup(BelaContext *context, void *userData)
{
    close(_handle);
}
