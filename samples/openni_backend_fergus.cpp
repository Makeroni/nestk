#include <ntk/ntk.h>
#include <ntk/utils/debug.h>
#include <ntk/camera/openni_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.cpp>
#include <QApplication>
#include <QDir>
#include <QMutex>

#include <iostream>
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

int sliderPosition = 5; // Initial slider position
int divisions = 16;	// Initial number of divisions
 
int blockXSize;         // Horizontal block size
int blockYSize;         // Vertical block size
 
int pixelCount;         // The number of pixels in a block (blockXSize multiplied by blockYSize)
 
int width;              // The width  of the input stream
int height;             // The height of the input stream

 
// Callback function to adjust number of divisions when we move the slider
void onDivisionSlide(int theSliderValue)
{
    // Put a lower limit of 1 on our slider position
    if (sliderPosition < 0)
    {
        sliderPosition = 1;
    }
 
    // Set the number of divisions depending on the slider location
    // Factors of both 640 and 480: 1, 2, 4, 5, 8, 10, 16, 20, 32, 40, 160
    switch (theSliderValue)
    {
    case 1:
        divisions = 1;
        break;
 
    case 2:
        divisions = 2;
        break;
 
    case 3:
        divisions = 4;
        break;
 
    case 4:
        divisions = 5;
        break;
 
    case 5:
        divisions = 8;
        break;
 
    case 6:
        divisions = 10;
        break;
 
    case 7:
        divisions = 16;
        break;
 
    case 8:
        divisions = 20;
        break;
 
    case 9:
        divisions = 32;
        break;
 
    case 10:
        divisions = 40;
        break;
 
    case 11:
        divisions = 160;
        break;
 
    default:
        break;
    }
 
    // Recalculate our block sizes and pixelCount for the new number of divisions
    blockXSize = width  / divisions;
    blockYSize = height / divisions;
 
    pixelCount = blockXSize * blockYSize;
}

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

int main(int argc, char **argv)
{

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
    zmax = 1.75;

    // Create two windows
    cvNamedWindow("WebCam", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Low Rez Stream", CV_WINDOW_AUTOSIZE);
 
    int maxSliderValue = 11;
 
    // Create the divisions slider lider
    cvCreateTrackbar("Divisions", "Low Rez Stream", &sliderPosition, maxSliderValue, onDivisionSlide);
 
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
	unsigned char iimg0[198],iimg1[198],iimg2[198],iimg3[198];
	int iiimg0, iiimg1, iiimg2, iiimg3;
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

	iimg2[  0] = 'S';
	iimg2[  1] = 'T';
	iimg2[  2] = 'A';
	iimg2[195] = 'E';
	iimg2[196] = 'N';
	iimg2[197] = 'D';

	iimg3[  0] = 'S';
	iimg3[  1] = 'T';
	iimg3[  2] = 'A';
	iimg3[195] = 'E';
	iimg3[196] = 'N';
	iimg3[197] = 'D';

	system("stty -F /dev/ttyUSB0 cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts"); 
	system("stty -F /dev/ttyUSB1 cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts");

	int fd0 = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
	set_interface_attribs (fd0, B115200, 0);
	set_blocking (fd0, 0); 

	int fd1 = open ("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_SYNC);
	set_interface_attribs (fd1, B115200, 0);
	set_blocking (fd1, 0); 

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
        blockXSize = width  / divisions / 2;
        blockYSize = height / divisions / 2;
 
        pixelCount = blockXSize * blockYSize; // How many pixels we'll read per block - used to find the average colour
 
        cout << "At " << divisions << " divisions (Block size " << blockXSize << "x" << blockYSize << ", so " << pixelCount << " pixels per block)" << endl;
 
		iiimg0 = 3;
		iiimg1 = 3;

		ij = 0;
        // Loop through each block vertically
        for (int yLoop = height*1/8; yLoop < height*5/8; yLoop += blockYSize)
        {
			ii = 0;
	        // Loop through each block horizontally
	        for (int xLoop = width*2/8; xLoop < width*6/8; xLoop += blockXSize)
	        {
 
                // Reset our colour counters for each block
                redSum     = 0;
                greenSum   = 0;
                blueSum    = 0;
 
                // Read every pixel in the block and calculate the average colour
                for (int pixXLoop = 0; pixXLoop < blockXSize; pixXLoop++)
                {
 
                    for (int pixYLoop = 0; pixYLoop < blockYSize; pixYLoop++)
                    {
 
                        // Get the pixel colour from the webcam stream
                        ptr = cvPtr2D(pFrame, yLoop + pixYLoop, xLoop + pixXLoop, NULL);
 
                        // Add each component to its sum
                        redSum   += ptr[2];
                        greenSum += ptr[1];
                        blueSum  += ptr[0];
 
                    } // End of inner y pixel counting loop
 
                } // End of outer x pixel countier loop
 
                // Calculate the average colour of the block
                red   = redSum   / pixelCount;
                green = greenSum / pixelCount;
                blue  = blueSum  / pixelCount;
 
				if( ii<8 && ij<8 )
				{
					iimg0[3+(7-ij+8*(7-ii))*3+0] = (unsigned char)(red*red*red/255/255);
					iimg0[3+(7-ij+8*(7-ii))*3+1] = (unsigned char)(green*green*green/255/255);
					iimg0[3+(7-ij+8*(7-ii))*3+2] = (unsigned char)(blue*blue*blue/255/255);
				}

				if( ii>=8 && ij<8 )
				{
					iimg1[3+(7-ij+8*(7-(ii-8)))*3+0] = (unsigned char)(red*red*red/255/255);
					iimg1[3+(7-ij+8*(7-(ii-8)))*3+1] = (unsigned char)(green*green*green/255/255);
					iimg1[3+(7-ij+8*(7-(ii-8)))*3+2] = (unsigned char)(blue*blue*blue/255/255);
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

