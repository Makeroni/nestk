#include <ntk/ntk.h>
#include <ntk/utils/debug.h>
#include <ntk/camera/openni_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.cpp>
#include <QApplication>
#include <QDir>
#include <QMutex>

#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <string.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"
using namespace std;
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace cv;
using namespace ntk;
namespace opt
{
ntk::arg<bool> high_resolution("--highres", "High resolution color image.", 0);
ntk::arg<int> kinect_id("--kinect-id", "Kinect id", 0);
}


int colorTest = 0;

int sliderPosition = 5; // Initial slider position
int divisionsX = 16;	// Initial number of divisions en el eje X
int divisionsY = 8;     // Initial number of divisions en el eje Y

float windowXStart = 50.0/100.0; // % de la imagen en el que se situa el centro eje X
float windowXSize = 100.0/100.0; // % de la imagen que se emuestra en la salida de los leds en X
float windowYStart = 50.0/100.0; // % de la imagen en el que se situa el centro eje Y
float windowYSize = 100.0/100.0; // % de la imagen que se emuestra en la salida de los leds en Y

int blockXSize;         // Horizontal block size
int blockYSize;         // Vertical block size
 
int pixelCount;         // The number of pixels in a block (blockXSize multiplied by blockYSize)
 
int width;              // The width  of the input stream
int height;             // The height of the input stream

 
// // Callback function to adjust number of divisions when we move the slider
// void onDivisionSlide(int theSliderValue)
// {
//     // Put a lower limit of 1 on our slider position
//     if (sliderPosition < 0)
//     {
//         sliderPosition = 1;
//     }
 
//     // Set the number of divisions depending on the slider location
//     // Factors of both 640 and 480: 1, 2, 4, 5, 8, 10, 16, 20, 32, 40, 160
//     switch (theSliderValue)
//     {
//     case 1:
//         divisionsX = 1;
//         break;
 
//     case 2:
//         divisionsX = 2;
//         break;
 
//     case 3:
//         divisionsX = 4;
//         break;
 
//     case 4:
//         divisionsX = 5;
//         break;
 
//     case 5:
//         divisionsX = 8;
//         break;
 
//     case 6:
//         divisionsX = 10;
//         break;
 
//     case 7:
//         divisionsX = 16;
//         break;
 
//     case 8:
//         divisionsX = 20;
//         break;
 
//     case 9:
//         divisionsX = 32;
//         break;
 
//     case 10:
//         divisionsX = 40;
//         break;
 
//     case 11:
//         divisionsX = 160;
//         break;
 
//     default:
//         break;
//     }
 
//     // Recalculate our block sizes and pixelCount for the new number of divisions
//     blockXSize = width  / divisionsX;
//     blockYSize = height / divisionsY;
 
//     pixelCount = blockXSize * blockYSize;
// }

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0){}
                //error_message ("error %d setting term attributes", errno);
}

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                //error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

/**
Recibe el entero queidentifica al dispositivo en /dev/ttyUSB*
Lo incializa y devuelve su identificador
**/
int
start_led_dev (int dev)
{
    char buffer[200];
    int n;

    n=sprintf (buffer, "stty -F /dev/ttyUSB%d cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts", dev);
    system(buffer); 

    n=sprintf (buffer, "/dev/ttyUSB%d", dev);
    int fd0 = open (buffer, O_RDWR | O_NOCTTY | O_SYNC);
    set_interface_attribs (fd0, B115200, 0);
    set_blocking (fd0, 0);

    return fd0;
}

/**
Función principal
Lee la información 3D y la muestra en pantalla y en los dispositivos led conectados
**/
int main(int argc, char **argv)
{

	//color blanco
	unsigned char c = (unsigned char) (254);

    // Parse command line options.
    arg_base::set_help_option("-h");
    arg_parse(argc, argv);
    // Set debug level to 1.
    ntk::ntk_debug_level = 1;
    // Set current directory to application directory.
    // This is to find Nite config in config/ directory.
    QApplication app (argc, argv);
    QDir::setCurrent(QApplication::applicationDirPath());
    // Declare the global OpenNI driver. Only one can be instantiated in a program.
    OpenniDriver ni_driver;
    // Declare the frame grabber.
    OpenniGrabber grabber(ni_driver, opt::kinect_id());
    // High resolution 1280x1024 RGB Image.
    if (opt::high_resolution())
        grabber.setHighRgbResolution(true);
    // Start the grabber.
    grabber.connectToDevice();
    grabber.start();
    // Holder for the current image.
    RGBDImage image;
    // Image post processor. Compute mappings when RGB resolution is 1280x1024.
    OpenniRGBDProcessor post_processor;
    namedWindow("depth");
    namedWindow("color");
    //namedWindow("users");
    RGBDFrameRecorder record("base");

    cv::Mat3b depth_as_color;

    double zmin, zmax;
    zmin = 0.25;
    zmax = 1.75; //5

    // Create two windows
    cvNamedWindow("WebCam", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Low Rez Stream", CV_WINDOW_AUTOSIZE);
 
    int maxSliderValue = 11;
 
    // Create the divisions slider lider
    //cvCreateTrackbar("Divisions", "Low Rez Stream", &sliderPosition, maxSliderValue, onDivisionSlide);
 
    // Get an initial frame so we know the size of things (cvQueryFrame is a combination of cvGrabFrame and cvRetrieveFrame)
    IplImage* pFrame = NULL;

	grabber.waitForNextFrame();
	grabber.copyImageTo(image);
	post_processor.processImage(image);
	compute_color_encoded_depth(image.depth(), depth_as_color, &zmin, &zmax);
	pFrame = new IplImage(depth_as_color);

 
    // Create an image the same size and colour-depth as our input stream
    IplImage* pLowRezFrame = cvCreateImage(cvSize(pFrame->width, pFrame->height), IPL_DEPTH_8U, 3);
 
    uchar *ptr; // Pointer to our pixel
 
    int red, green, blue; // Integers to hold our pixel values
 
    // Get the width and height of our webcam input stream
    int width  = pFrame->width ;
    int height = pFrame->height;
 
    // Integers to hold our total colour values (used to find the average)
    int redSum     = 0;
    int greenSum   = 0;
    int blueSum    = 0;
 
    // Loop controling vars
    char keypress;
    bool quit = false;

	FILE *fp;
	unsigned char iimg0[198],iimg1[198];
	int iiimg0, iiimg1;
	int ii, ij;

	iimg0[  0] = 'S';
	iimg0[  1] = 'T';
	iimg0[  2] = 'A';
	iimg0[195] = 'E';
	iimg0[196] = 'N';
	iimg0[197] = 'D';

	iimg1[  0] = 'S';
	iimg1[  1] = 'T';
	iimg1[  2] = 'A';
	iimg1[195] = 'E';
	iimg1[196] = 'N';
	iimg1[197] = 'D';

    //dev es el valor que identifica a los dispositivos de leds /dev/ttyUSB[dev]
    int dev[]={3,2};

    int fd0 = start_led_dev(dev[0]);
    int fd1 = start_led_dev(dev[1]);

	//Igualar los tonos de color
	char str[10];
	char r[2];
	r[0]=(unsigned char)13;
	r[1]='\0';
	char g[2];
	g[0]=(unsigned char)53;
	g[1]='\0';
	char b[2];
	b[0]=(unsigned char)28;
	b[1]='\0';

	strcpy (str, "STA");
	strcat (str, r);
	strcat (str, g);
	strcat (str, b);
	strcat (str, "COL");

	write(fd0, str, 198);

	keypress = cvWaitKey(1000);

    while (quit == false)
    {
        // Grab a frame from the webcam 
        grabber.waitForNextFrame();
		grabber.copyImageTo(image);
		post_processor.processImage(image);
		compute_color_encoded_depth(image.depth(), depth_as_color, &zmin, &zmax);
		pFrame = new IplImage(depth_as_color);

        // Draw the original frame and low resolution version
        cvShowImage("WebCam", pFrame);
        cvShowImage("Low Rez Stream", pLowRezFrame);

        // Calculate our blocksize per frame to cater for slider
        blockXSize = width  * windowXSize / divisionsX; //width  / divisionsX / 2;
        blockYSize = height * windowYSize / divisionsY; //height / divisionsY / 2;

        pixelCount = blockXSize * blockYSize; // How many pixels we'll read per block - used to find the average colour

        cout << "At " << divisionsX << " divisionsY " << divisionsY << " divisions (Block size " << blockXSize << "x" << blockYSize << ", so " << pixelCount << " pixels per block)" << endl;
       // 16*8, 20*30, 600 pixeles por bloque
       //cout << "width " << width << " height " << height << endl; 640*480

		iiimg0 = 3;
		iiimg1 = 3;

		ij = 0;
        // Loop through each block vertically
        for (int yLoop = (height * windowYStart) - (height * windowYSize  / 2); yLoop < (height * windowYStart) + (height * windowYSize  / 2); yLoop += blockYSize)
        {
			ii = 0;
	        // Loop through each block horizontally
	        for (int xLoop = (width * windowXStart) - (width * windowXSize  / 2); xLoop < (width * windowXStart) + (width * windowXSize  / 2); xLoop += blockXSize)
	        {
 
                // Reset our colour counters for each block
                int redSum[blockXSize] [blockYSize];
                int greenSum[blockXSize] [blockYSize];
                int blueSum[blockXSize] [blockYSize];
 
                // Read every pixel in the block and calculate the average colour
                for (int pixXLoop = 0; pixXLoop < blockXSize; pixXLoop++)
                {
 
                    for (int pixYLoop = 0; pixYLoop < blockYSize; pixYLoop++)
                    { 
                        //sort(pFrame[0], pFrame[0] + blockXSize + blockYSize);
 
                        // Get the pixel colour from the webcam stream
                        ptr = cvPtr2D(pFrame, yLoop + pixYLoop, xLoop + pixXLoop, NULL);
 
                        // Add each component to its sum
                        redSum[pixXLoop] [pixYLoop] = ptr[2];
                        greenSum[pixXLoop] [pixYLoop] = ptr[1];
                        blueSum[pixXLoop] [pixYLoop] = ptr[0];
 
                    } // End of inner y pixel counting loop
 
                } // End of outer x pixel countier loop

				sort(redSum[0], redSum[0] + blockXSize + blockYSize);
				sort(greenSum[0], greenSum[0] + blockXSize + blockYSize);
				sort(blueSum[0], blueSum[0] + blockXSize + blockYSize);

                // Calculate the average colour of the block
                red   = redSum[blockXSize/10][blockYSize/10];
                green = greenSum[blockXSize/10][blockYSize/10];
                blue  = blueSum[blockXSize/10][blockYSize/10];
 
				unsigned char redcolor = c;
				unsigned char greencolor = c;
				unsigned char bluecolor = c;
				if(!colorTest)
				{
					redcolor=(unsigned char)(red*red*red/255/255);
					greencolor=(unsigned char)(green*green*green/255/255);
					bluecolor=(unsigned char)(blue*blue*blue/255/255);
				}

				if( ii<8 && ij<8 )
				{
					iimg0[3+(7-ij+8*(7-ii))*3+0] = redcolor;
					iimg0[3+(7-ij+8*(7-ii))*3+1] = greencolor;
					iimg0[3+(7-ij+8*(7-ii))*3+2] = bluecolor;
				}

				if( ii>=8 && ij<8 )
				{
					iimg1[3+(7-ij+8*(7-(ii-8)))*3+0] = redcolor;
					iimg1[3+(7-ij+8*(7-(ii-8)))*3+1] = greencolor;
					iimg1[3+(7-ij+8*(7-(ii-8)))*3+2] = bluecolor;
				}

                // Draw a rectangle of the average colour
                cvRectangle(
                    pLowRezFrame,
                    cvPoint(xLoop, yLoop),
                    cvPoint(xLoop + blockXSize, yLoop + blockYSize),
                    CV_RGB(red, green, blue),
                    CV_FILLED,
                    8,
                    0
                );
 				ii++;
            } // End of inner y loop
 			ij++;
        } // End of outer x loop

		write (fd0, iimg1, 198);
		write (fd1, iimg0, 198);
 
        // Wait 5 millisecond
        keypress = cvWaitKey(1);
 
        // Set the flag to quit if the key pressed was escape
        if (keypress == 27)
        {
            quit = true;
        }
 
    } // End of while loop
 
 	grabber.stop();

    // Release our images & destroy all windows
    cvReleaseImage(&pFrame);
    cvReleaseImage(&pLowRezFrame);
    cvDestroyAllWindows();
}

