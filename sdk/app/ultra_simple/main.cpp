/*
 * Copyright (C) 2014  RoboPeak
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 *  RoboPeak Lidar System
 *  Simple Data Grabber Demo App
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *
 *  An ultra simple app to fetech RPLIDAR data continuously....
 *
 */




#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <errno.h>
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "ObstacleAvoidance.hh"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;


bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

//request pipe name.request the direction
#define REQUESTPIPE "./Request"
//reply pipe name. 
#define REPLYPIPE "./Reply"

unsigned int GetFifo()
{
	unsigned int pipeFd = 0;
	if(access(REQUESTPIPE,F_OK | R_OK) == -1)
	{
		if(mkfifo(REQUESTPIPE,0777))
		{
			fprintf(stderr,"Could not create fifo %s \n",REQUESTPIPE);
			exit(EXIT_FAILURE);
		}
	}

	if(access(REPLYPIPE,F_OK | R_OK) == -1)
	{
		if(mkfifo(REPLYPIPE,0777))
		{
			fprintf(stderr,"Could not create fifo %s \n",REPLYPIPE);
			exit(EXIT_FAILURE);
		}
	}
	
	//high-order restore the request pipe file descriptor
RequestFdInit:
	int fd = open(REQUESTPIPE,O_RDONLY | O_NONBLOCK);
	if(fd == -1)
	{
		goto RequestFdInit;
		exit(EXIT_FAILURE);
	}
	pipeFd |= fd;
	pipeFd <<= (sizeof(unsigned int) * 8 / 2);
	
	//low-order restore the reply pipe file descriptor.
ReplyInit:
	fd = open(REPLYPIPE, O_WRONLY | O_NONBLOCK);
	if(fd == -1)
	{
		goto ReplyInit;
		exit(EXIT_FAILURE);
	}
	pipeFd |= fd;
	return pipeFd;		
}


//receive the message
int Request(size_t fd,Object &obj)
{
	Angle buffer[2];
      	int ret = read(fd,buffer,sizeof(Angle) * 2);
	if(ret > 0)//(int)sizeof(Angle )* 2)
	{
		if(buffer[0] <= 360 && buffer[1] <= 360)
		{
			obj.m_targetAngle = buffer[0];
			obj.m_currentAngle = buffer[1];
			return ret;
		}
	}
	return 0;	
}

//reply the message
void Reply(size_t fd,void *buffer,size_t size)
{
	while(1)
	{
		int ret = write(fd,buffer,size);
		if(ret >= 0)
			break;
		if(errno != EAGAIN)
			exit(EXIT_FAILURE);

	}
}

int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;

    //insert by gz
   // Object obj(1500);
    Object obj(argc < 2 ? 50 : atoi(argv[1]));
    DetectStrategy stt(obj,argc < 3 ? 500 : atoi(argv[2]));
    DecisionStrategy ds;
    vector<MyPoint> map(360);
	
    unsigned int pipeFds = GetFifo();
    int replyFd = pipeFds & ((1 << (sizeof(unsigned int) * 8 / 2)) - 1);
    int requestFd = pipeFds >> (sizeof(unsigned int) * 8/ 2);


    // read serial port from the command line...
    //if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    //if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);


    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }


    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }


    // start scan...
    drv->startScan();


    // fetech result and print it out...
    while (1) {
        rplidar_response_measurement_node_t nodes[360*2];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);

	    if(Request(requestFd,obj) != 0)
	    {
    		for(unsigned short i = 0;i < map.size();++i)
    		{
	    		map[i].angle = i;
	    		map[i].distance = 0xffff;
    		}
            	for (int pos = 0; pos < (int)count ; ++pos) {
			/*
                	 printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    	(nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
                    	(nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                    	nodes[pos].distance_q2/4.0f,
                    	nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
			*/
			if((nodes[pos].distance_q2 / 4.0f) != 0.0f)
			{
				map[(unsigned short)((nodes[pos].angle_q6_checkbit>>RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f) % 360].distance = (float)(nodes[pos].distance_q2 / 4.0f);
			}
            	}

		for(int i = 0;i < 360;++i)
		{
//			printf("angle %d,distance %d\n",map[i].angle,map[i].distance);
		}

	    	try
	    	{
		    MyPoint p = ds.Strategy(map,stt);
		    Reply(replyFd,&p,sizeof(MyPoint));
	    	}
	    	catch(CannotDecide &e)
	    	{
		    //...
		    MyPoint p;
		    Reply(replyFd,&p,sizeof(MyPoint));
		    printf("CRASH\n");
		}	    
	    }   
        }

    }

    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}
