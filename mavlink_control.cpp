
/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Peter XU,  <peterxu9710@outlook.com>

 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Peter XU,  <peterxu9710@outlook.com>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
#include <fstream>

#define MAX_THREADS 5
#define

int Marker_map[10][10] = {{233,269,593,661,799,729,775,625,537,434},
                          {808,486,675,805,321,135,965,698,45,706},
                          {318,70,788,724,765,504,840,87,189,37},
                          {609,436,528,577,477,147,149,19,632,32},
                          {797,380,739,411,723,657,560,141,374,248},
                          {160,875,181,195,818,668,542,71,34,987},
                          {954,176,9,682,226,992,703,733,82,460},
                          {463,17,651,574,431,60,966,423,401,935},
                          {144,341,893,330,208,270,286,795,24,634},
                          {382,175,275,991,958,783,184,746,540,261}};

int route[10] = {783,270,60,992,668,657,147,504,135,729};

bool takeoff = 0;
bool mission_status = 0;
bool get_ball = 0;
bool throw_done = 0;
bool throwing = 0;
float H_init = 20;
int servopin = 25;
bool servo_on_control = 0;
bool servo_off_control = 0;
double rel_distance_x = 0;
double rel_distance_y = 0;
double rel_distance_z = 0;

pthread_mutex_t servo_lock;
pthread_cond_t servo_cond;
pthread_mutex_t detect_color_lock;
pthread_mutex_t detect_marker_lock;

Mat_<double>  camMatrix;
Mat_<double>  distCoeff;
Matrix44 projectionMatrix;
vector<Marker> m_detectedMarkers;
MarkerDetector markerDetector;

int top (int argc, char **argv)
{
    Init_all();
    char *uart_name = (char*)"/dev/ttyS0";
	int baudrate = 57600;

	parse_commandline(argc, argv, uart_name, baudrate);

	Serial_Port serial_port(uart_name, baudrate);
	Autopilot_Interface autopilot_interface(&serial_port);

	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	serial_port.start();
	autopilot_interface.start();
    printf("Interface.start!\n");

	commands(autopilot_interface);

	autopilot_interface.stop();
	serial_port.stop();

	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api)
{   
    pthread_cond_init(&servo_cond, NULL);
    pthread_mutex_init(&servo_lock, NULL);
    pthread_t ids[MAX_THREADS];
    
    int ret = pthread_create(&ids[0],NULL,task_thread,(void*)&api);
    if(ret!=0)
    {
        printf("Create move pthread error!\n");
    }

    ret=pthread_create(&ids[1],NULL,detect_thread,(void*)&api);
    if(ret!=0)
    {
        printf("Create detect pthread error!\n");
    }

    ret=pthread_create(&ids[2],NULL,servo_on_thread,NULL);
    if(ret!=0)
    {
        printf("Create servo on pthread error!\n");
    }

    ret=pthread_create(&ids[2],NULL,servo_off_thread,NULL);
    if(ret!=0)
    {
        printf("Create servo off pthread error!\n");
    }

	return;
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}

void* servo_on_thread(void*)
{
    while(servo_on_control)
    {
        if(throwing)
        {
            digitalWrite(servopin,HIGH);
            delayMicroseconds(1000);
            digitalWrite(servopin,LOW);
            delayMicroseconds(19000);
        }
    }
}

void* servo_off_thread(void*)
{
    while(servo_off_control)
    {
        if(get_ball)
        {
            digitalWrite(servopin,HIGH);
            delayMicroseconds(2000);
            digitalWrite(servopin,LOW);
            delayMicroseconds(18000);
        }
    }
}


void
Init_all()
{
    readCameraParameter();
    if(wiringPiSetup()== -1)
    {
        exit(1);
        printf("unable to setup pin %d",servopin);
    }
    pinMode(servopin,OUTPUT);
}


void readCameraParameter()
{
    camMatrix = Mat::eye(3, 3, CV_64F);
    distCoeff = Mat::zeros(8, 1, CV_64F);

    camMatrix(0,0) = 6.24860291e+02 * (640./352.);
    camMatrix(1,1) = 6.24860291e+02 * (480./288.);
    camMatrix(0,2) = 640 * 0.5f; //640
    camMatrix(1,2) = 480 * 0.5f; //480

    for (int i=0; i<4; i++)
        distCoeff(i,0) = 0;
}

int detect_marker(Mat frame)
{

}

void detect_color(Autopilot_Interface api,const Mat frame)
{
    mavlink_set_position_target_local_ned_t sp;
    Mavlink_Messages messages = api.current_messages;
    mavlink_set_position_target_local_ned_t ip = api.initial_position;
    mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;

    int target_rgb[3]={50,200,50};
    firedetect first(frame, target_rgb);
    first.CheckColor();

    if (first.firedetected() && !throw_done)
    {
        cout << "fire detected!" << endl;

        // copy current messages
        float f = 0.00304, H;
        double pix_x, pix_y;
        double square0 = 640 * 480;

        //calculate related parameters
        pix_x = (first.get_desx()-320) * 0.000005;
        pix_y = (first.get_desy()-240) * 0.000005;
        cout << "origin pix_x:" << first.get_desx() << endl;

        H = messages.range_finder.current_distance - H_init;
        H = H / (float) 100;

        pthread_mutex_lock(&detect_color_lock);
        rel_distance_x = pix_x * H / f;
        rel_distance_y = pix_y * H / f;

        //debug
/*          cout<<"pix_x "<< pix_x <<endl;
            cout<<"pix_y "<< pix_y <<endl;
            cout<<"H: "<< H <<endl;
            outfile << "H: "<< H <<endl;
            cout << "rel_distance_x = " << rel_distance_x << endl;
            cout << "rel_distance_y = " << rel_distance_y << endl;
            cout << "rel_distance_z = " << rel_distance_z << endl;
            outfile << "rel_distance_x = " << rel_distance_x << endl;
            outfile << "rel_distance_y = " << rel_distance_y << endl;
            outfile << "rel_distance_z = " << rel_distance_z << endl;
*/

        if (first.get_square() / square0 < 0.01 && first.get_square() / square0 != 0 && H != 0) {
            rel_distance_z = 0.1;
        } else {
            rel_distance_z = 0;
        }
        pthread_mutex_unlock(&detect_color_lock);
    }
}

void rtl(Autopilot_Interface &api, Mat frame)
{

}


void* task_thread(void* api)
{
    Autopilot_Interface api2 = *(Autopilot_Interface*)api;
    api2.enable_offboard_control();


}

void* detect_thread(void* api)
{
    int i = 0;
    double rate = 25;

    Mat frame;
    ofstream outfile;
    VideoCapture capture(0);
    VideoWriter writer;
    Size videoSize(capture.get(CV_CAP_PROP_FRAME_WIDTH),capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    outfile.open("/home/pi/result.txt");
    writer.open("/home/pi/result.avi",CV_FOURCC('P','I','M','1'),rate,videoSize);

    Autopilot_Interface api1 = *(Autopilot_Interface*)api;

    mavlink_set_position_target_local_ned_t sp;
    Mavlink_Messages messages = api1.current_messages;
    mavlink_set_position_target_local_ned_t ip = api1.initial_position;
    mavlink_local_position_ned_t pos = api1.current_messages.local_position_ned;

    while(mission_status != 6)
    {
        // initialize command data strtuctures
        capture >> frame;
        writer << frame;


        i++;
        outfile<< i <<" CURRENT POSITION XYZ = [ "<< pos.x << ", "<< pos.y << ", " << pos.z << " ]" << endl;
    }

    outfile.close();
    api1.disable_offboard_control();
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}
