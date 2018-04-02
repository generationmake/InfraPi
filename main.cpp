#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <thread>
#include <ctime>
#include <iostream>
#include <fstream>

// global variables for exchange between threads
cv::VideoCapture cap;	// create camera input
cv::Mat cameraImage;  // create opencv mat for camera

void cameraThread(void)	// function for the camera thread
{
	while(1)	// loop forever
	{
		cap >> cameraImage;	// copy camera input to opencv mat
	}
}
void usage(void)
{
	printf("usage of ThermoCam\n");
	printf("-q           quiet mode, no live output of image\n");
	printf("-h           print this help\n");
	printf("-nocam       don't use camera input, just temperature sensor\n");
	printf("-i           use interpolation on thermal data\n");
	printf("-f <file>    generate image file from each frame\n");
	printf("-s <file>    generate file with temperature data from each frame\n");
	printf("-l <file>    generate temperature log file\n");
	printf("-v <file>    generate video file\n");
}

int main(int argv, char **argc)
{
	int file;
	int addr=0x68;		// adress of AMG88xx
	int x,y,i; 	// variables to got through the array
	signed short int internal_temp;
	signed short int pixel_temp[64];	// array for pixel temperatures
	float temp_max=0;	// max. temperature of sensor
	char stringBuf[10];	//buffer for strings
	std::time_t timestamp, timestamplast=0;
	std::ofstream logfile;
	std::ofstream tempfile;
	char *name_video="output.avi";
	char *name_file="/dev/shm/image/opencv_stream.jpg";
	char *name_tempfile="/dev/shm/temp.dat";
	char *name_log="temp.log";
	std::thread tcam;	// create thread pointer

// flags
	bool mode_quiet=0;
	bool mode_interpolation=0;
	bool mode_video=0;
	bool mode_file=0;
	bool mode_tempfile=0;
	bool mode_log=0;
	bool mode_camera=1;

	if(argv<2)
	{
		usage();
		exit(1);
	}
	for(i=1;i<argv;i++)
	{
		printf("%i: %s\n",i,argc[i]);
		if(strncmp(argc[i],"-h",2)==0)
		{
			usage();
			exit(1);
		}
		if(strncmp(argc[i],"-q",2)==0) mode_quiet=1;
		if(strncmp(argc[i],"-i",2)==0) mode_interpolation=1;
		if(strncmp(argc[i],"-nocam",6)==0) mode_camera=0;
		if(strncmp(argc[i],"-v",2)==0)
		{
			mode_video=1;
			if(i<argv-1)
			{
				if(argc[i+1][0]!='-')
				{
					i++;
					name_video=argc[i];
				}
			}
		}
		if(strncmp(argc[i],"-f",2)==0)
		{
			mode_file=1;
			if(i<argv-1)
			{
				if(argc[i+1][0]!='-')
				{
					i++;
					name_file=argc[i];
				}
			}
		}
		if(strncmp(argc[i],"-s",2)==0)
		{
			mode_tempfile=1;
			if(i<argv-1)
			{
				if(argc[i+1][0]!='-')
				{
					i++;
					name_tempfile=argc[i];
				}
			}
		}
		if(strncmp(argc[i],"-l",2)==0) 
		{
			mode_log=1;
			if(i<argv-1)
			{
				if(argc[i+1][0]!='-')
				{
					i++;
					name_log=argc[i];
				}
			}
		}
	}
	printf("mode_quiet = %i\n", mode_quiet);
	printf("mode_interpolation = %i\n", mode_interpolation);
	printf("mode_camera = %i\n", mode_camera);
	printf("mode_video = %i\n", mode_video);
	printf("mode_file  = %i\n", mode_file);
	printf("mode_tempfile  = %i\n", mode_tempfile);
	printf("mode_log   = %i\n", mode_log);
	printf("name_video = %s\n", name_video);
	printf("name_file  = %s\n", name_file);
	printf("name_tempfile  = %s\n", name_tempfile);
	printf("name_log   = %s\n", name_log);

	int end=1;  // variable to end program
	printf("ThermoCam\n");  // print start message
	if((file=open("/dev/i2c-1",O_RDWR))<0)	// open i2c-bus
	{
		perror("cannot open i2c-1");
		exit(1);
	}
	if(ioctl(file, I2C_SLAVE, addr)<0)	// open slave
	{
		perror("cannot open slave address");
		exit(1);
	}
	if(mode_log) logfile.open(name_log, std::ios::trunc);

	internal_temp=(signed short)(i2c_smbus_read_word_data(file, 0x0e)<<4);	// read internal temperature from sensor
	internal_temp=internal_temp/16;		// recalculate correct value
	printf("Internal Temp: %f C (0x%04X = %i)\n",(float)internal_temp*0.0625,internal_temp,internal_temp);	// print internal temperature

	cv::Mat cameraImageGray; // create opencv mat for grayscale camera image
	cv::Mat cameraImageBig(320,320,CV_8UC3);  // create opencv mat for camera with final resolution
	cv::Mat cameraImageBigOutput(320,380,CV_8UC3);  // create opencv mat for output with space for tempBar

	if(mode_camera==1)
	{
		cap.open(0);		// open camera
		cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);	// change camera width to 320 - we do not need more
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);	// change camera height to 240
	}

	cv::VideoWriter outputVideo;	// create video output
	if(mode_video)
	{
		outputVideo.open(name_video, CV_FOURCC('M','J','P','G'), 15, cv::Size(380,320), true);	// set video output to 15 fps and MJPG
		if (!outputVideo.isOpened())	// check if generation of video output was successful
		{
			perror("Could not open the output video for write\n");
			exit(1);
		}
	}

	cv::Mat outSmall(8,8,CV_8UC1);		// create opencv mat for sensor data
	cv::Mat outSmallnorm(8,8,CV_8UC1);	// create opencv mat for normalized data
	cv::Mat outColor;	// create opencv mat for color output
	cv::Mat combined;	// create opencv mat for combined output

	cv::Mat tempBar(256,5,CV_8UC1);	// create opencv mat for temperature bar

	for(x=0;x<256;x++)
	{
		tempBar.at<unsigned char>(x,1)=255-x;	// fill tempBar with decreasing values
		tempBar.at<unsigned char>(x,1)=255-x;	// fill tempBar with decreasing values
		tempBar.at<unsigned char>(x,2)=255-x;	// fill tempBar with decreasing values
		tempBar.at<unsigned char>(x,3)=255-x;	// fill tempBar with decreasing values
		tempBar.at<unsigned char>(x,4)=255-x;	// fill tempBar with decreasing values
	}
	cv::applyColorMap(tempBar,tempBar,cv::COLORMAP_JET);  // generate colored output with colormap
	tempBar.copyTo(cameraImageBigOutput(cv::Rect(330, 32, 5, 256)));	// copy tempBar to big image
	cv::putText(cameraImageBigOutput, "100", cv::Point(345,25),1,1,cv::Scalar(255,255,255));
	cv::putText(cameraImageBigOutput, "0", cv::Point(345,305),1,1,cv::Scalar(255,255,255));
	if(mode_camera==1)
	{
		cap >> cameraImage;	// copy camera input to opencv mat to get data to startup
		tcam=std::thread(cameraThread);	// start extra thread to get camera input
	}
	while(end==1)  // check end variable
	{
		timestamp=std::time(NULL);
		if(timestamp>timestamplast)
		{
			timestamplast=timestamp;
			printf("Timestamp: %i\n", (int)timestamp);
	        cv::TickMeter t;  
	        t.start();    // start timer  
		temp_max=0;

		x=i2c_smbus_read_i2c_block_data(file,0x80,32,(__u8*)pixel_temp);	// read first 32 byte / 16 temperature pixels from sensor
		x=i2c_smbus_read_i2c_block_data(file,0xa0,32,(__u8*)pixel_temp+32);       // read next 32 byte / 16 temperature pixels from sensor
		x=i2c_smbus_read_i2c_block_data(file,0xc0,32,(__u8*)pixel_temp+64);       // read next 32 byte / 16 temperature pixels from sensor
		x=i2c_smbus_read_i2c_block_data(file,0xe0,32,(__u8*)pixel_temp+96);       // read last 32 byte / 16 temperature pixels from sensor

		for(x=0;x<64;x++)
		{
			pixel_temp[x]=(signed short)(pixel_temp[x]<<4)/16;	// set pixel_temp to original value
			if(pixel_temp[x]>temp_max) temp_max=pixel_temp[x];	// find max. temp of scan
		}
		temp_max=temp_max/4.0;

		for(x=0;x<8;x++)
		{
			for(y=0;y<8;y++)
			{
				outSmall.at<char>(7-x,7-y)=(pixel_temp[x*8+y]*64)/100;	// save data to opencv mat and rotate it
			}
		}

//		cv::normalize(outSmall,outSmallnorm,255,0,cv::NORM_MINMAX);	// normalize Mat to values between 0 and 255
		if(mode_interpolation) cv::resize(outSmall,outSmallnorm,cv::Size(320,320));	// resize Mat to 320 x 320 pixel
		else cv::resize(outSmall,outSmallnorm,cv::Size(320,320),0,0,cv::INTER_NEAREST);	// resize Mat to 320 x 320 pixel
		cv::applyColorMap(outSmallnorm,outColor,cv::COLORMAP_JET);  // generate colored output with colormap
		if(mode_camera==1)
		{
			cv::cvtColor(cameraImage,cameraImageGray,CV_RGB2GRAY);	// convert camera image to grayscale
			cv::cvtColor(cameraImageGray,cameraImageGray,CV_GRAY2RGB);	// make cameraImage 3 channels again
			cameraImageGray.copyTo(cameraImageBig(cv::Rect(0,40,320,240)));	// copy camera ingae to mat with same resolution as temperature mat
			cv::addWeighted(cameraImageBig,0.5,outColor,0.5,0.0,combined);	// combine camera mat and temperature mat into one single image
			combined.copyTo(cameraImageBigOutput(cv::Rect(0,0,320,320)));
		}
		else outColor.copyTo(cameraImageBigOutput(cv::Rect(0,0,320,320)));	// copy sensor image to output mat
		cv::rectangle(cameraImageBigOutput, cv::Point(345,30), cv::Point(380,290), cv::Scalar(0), -1);	// clear area for temp display
		sprintf(stringBuf,"%.1f",temp_max);
		cv::putText(cameraImageBigOutput, stringBuf, cv::Point(345,295-(temp_max*2.56)),1,1,cv::Scalar(255,255,255));

		if(mode_log) logfile<<temp_max<<std::endl;
		if(!mode_quiet) cv::imshow("combined",cameraImageBigOutput);  // display mat on screen
 		if(mode_video) outputVideo << cameraImageBigOutput;	// add frame to video
		if(mode_file) cv::imwrite(name_file,cameraImageBigOutput);	// write file to disk
		if(mode_tempfile) 	// store temperature data to file
		{
			tempfile.open(name_tempfile, std::ios::trunc);
			tempfile<<temp_max<<std::endl;
			tempfile.close();
		}

		t.stop();    // stop timer  
		printf("max. Temp: %f\n",temp_max);
	        printf("Time: %f ms\n", (double)t.getTimeMilli() / t.getCounter());  // print result of timer  
		
		char key = cv::waitKey(1);  // check keys for input
		if(key=='e') end=0;  // end if e was pressed
		}
	}
	printf("ended regularly!\n");  // print end message
	close(file);
	if(mode_log) logfile.close();
	return 0;
}
