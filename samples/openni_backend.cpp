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
#include <fcntl.h>

#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <typeinfo>

namespace opt
{
	ntk::arg<bool> high_resolution("--highres", "High resolution color image.", 0);
	ntk::arg<int> kinect_id("--kinect-id", "Kinect id", 0);
}

//OPCIONES
int colorTest = 0;     // 1 para hacer el test de como se ven los colores
int useBlackWhite = 1; // 1 para ver como veríamos la profundidad usando solo un color

//TAMAÑO LEDS
int divisionsX = 16;	// Initial number of divisions en el eje X
int divisionsY = 8;     // Initial number of divisions en el eje Y

//TAMAÑO VENTANA (%)
float totalWindowXStart = 40.0/100.0; // % de la imagen en el que se situa el centro eje X
float totalWindowXSize = 39.9/100.0; // % de la imagen que se emuestra en la salida de los leds en X
float totalWindowYStart = 40.0/100.0; // % de la imagen en el que se situa el centro eje Y
float totalWindowYSize = 39.9/100.0; // % de la imagen que se emuestra en la salida de los leds en Y

int blockXSize;         // Horizontal block size
int blockYSize;         // Vertical block size

int width;              // The width  of the input stream 640
int height;             // The height of the input stream 480

//Si es mayor que 0, indica que hemos parpadeado
int eye = 0;
//Si es mayor que 0, indica que tenemos los ojos cerrados
bool close_eye = false;
//Si es mayor que 0, indica que hemos movido los ojos
bool move = false;

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0){}
                printf ("error %d setting term attributes", errno);
}

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
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
int start_led_dev (int dev)
{
    char buffer[200];
    int n;

    n=sprintf (buffer, "stty -F /dev/ttyUSB%d cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts", dev);
    int returnValue = system(buffer); 

    n=sprintf (buffer, "/dev/ttyUSB%d", dev);
    int fd0 = open (buffer, O_RDWR | O_NOCTTY | O_SYNC);
    set_interface_attribs (fd0, B115200, 0);
    set_blocking (fd0, 0);

    return fd0;
}

/**
	Recibe los eventos del ratón, para emular el eye tracking
**/
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if  ( event == EVENT_LBUTTONDOWN )
	{
		close_eye = true;
	}
	else if ( event == EVENT_LBUTTONUP )
     {
		close_eye = false;
		eye = -30;
     }
	else if ( event == EVENT_MOUSEMOVE )
     {
		totalWindowXStart = (float)x / (float)width;
		totalWindowYStart = (float)y / (float)height;
		move=true;
     }
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
    ntk::arg_base::set_help_option("-h");
    ntk::arg_parse(argc, argv);

//    // Set debug level to 1.
//    ntk::ntk_debug_level = 100;
    
	// Set current directory to application directory.
    // This is to find Nite config in config/ directory.
    QApplication app (argc, argv);
    QDir::setCurrent(QApplication::applicationDirPath());
    // Declare the global OpenNI driver. Only one can be instantiated in a program.
    ntk::OpenniDriver ni_driver;
    // Declare the frame grabber.
    ntk::OpenniGrabber grabber(ni_driver, opt::kinect_id());
    // High resolution 1280x1024 RGB Image.
    if (opt::high_resolution())
        grabber.setHighRgbResolution(true);
    // Start the grabber.
    grabber.connectToDevice();
    grabber.start();
    // Holder for the current image.
    ntk::RGBDImage image;
    // Image post processor. Compute mappings when RGB resolution is 1280x1024.
    ntk::OpenniRGBDProcessor post_processor;

    cv::Mat3b depth_as_color;

    double zmin, zmax, zmed;
    zmin = 0.25;
    zmed = 1.75;
    zmax = 5.0;

    // Create two windows
    cvNamedWindow("WebCam", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Low Rez Stream", CV_WINDOW_AUTOSIZE);
	// Establecemos que hacer con las señales del ratón al estar sobre la ventana
	cvSetMouseCallback("WebCam", CallBackFunc, NULL);

    // Get an initial frame so we know the size of things (cvQueryFrame is a combination of cvGrabFrame and cvRetrieveFrame)
    IplImage* pFrame = NULL;
	IplImage pFrameAux;

	grabber.waitForNextFrame();
	grabber.copyImageTo(image);
	post_processor.processImage(image);

    int** blackWhite_im;     // Imagen en blanco y negro
	blackWhite_im = new int*[image.depth().rows];
	for (int r = 0; r < image.depth().rows; ++r)
    {
		blackWhite_im[r] = new int[image.depth().cols];
	}
	ntk::compute_color_encoded_depth2(image.depth(), depth_as_color,
			 &zmin, &zmax, &zmed, &blackWhite_im);
	pFrameAux = depth_as_color;
	pFrame = &pFrameAux;

    // Create an image the same size and colour-depth as our input stream
   	IplImage* pLowRezFrame = cvCreateImage(cvSize(pFrame->width, pFrame->height), IPL_DEPTH_8U, 3);
 
    uchar *ptr = NULL; // Pointer to our pixel
 
    int red, green, blue, blackWhite; // Integers to hold our pixel values
 
    // Get the width and height of our webcam input stream
    width  = pFrame->width ;
    height = pFrame->height;
 
    // Loop controling vars
    char keypress;
    bool quit = false;

	FILE *fp;
	unsigned char iimg0[198],iimg1[198];
	int ii, ij;

	//Escribímos "STA" al principi del vector "END" al final
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
    int dev[]={0,1};

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

	ssize_t returnValue = write(fd0, str, 198);

	keypress = cvWaitKey(1000);

	float windowXSize = totalWindowXSize;
	float windowYSize = totalWindowYSize;
	float windowXStart = totalWindowXStart;
	float windowYStart = totalWindowYStart;

	unsigned char redcolor;
	unsigned char greencolor;
	unsigned char bluecolor;

	int div;

    while (quit == false)
    {
        // Grab a frame from the webcam 
        grabber.waitForNextFrame();
		grabber.copyImageTo(image);
		post_processor.processImage(image);

		ntk::compute_color_encoded_depth2(image.depth(), depth_as_color,
			 &zmin, &zmax, &zmed, &blackWhite_im);
		pFrameAux = depth_as_color;
		pFrame = &pFrameAux;

        // Draw the original frame and low resolution version
        cvShowImage("WebCam", pFrame);
        cvShowImage("Low Rez Stream", pLowRezFrame);

		//Borramos todos los píxeles, por si el nuevo cuadro no coincide
		if(eye!=0 || move){
            cvRectangle(
                pLowRezFrame,
                cvPoint(0,0),
                cvPoint(pFrame->width, pFrame->height),
                CV_RGB(0, 0, 0),
                CV_FILLED,
                8,
                0
            );			
		}
		//Si hemos detectado movimiento actualizamos la posición del centro del cuadro
		if(move){
			windowXStart=totalWindowXStart;
			windowYStart=totalWindowYStart;
			move=false;
		}

		//Hemos abierto los ojos hace menos de eye frames
		if(eye!=0){
			windowXSize=totalWindowXSize-eye/100.0;
			windowYSize=totalWindowYSize-eye/100.0;
			if(eye>0){			
				eye=eye-1;
			}else{
				eye=eye+1;
			}
		}else{
			windowXSize=totalWindowXSize;
			windowYSize=totalWindowYSize;
		}

        // Calculate our blocksize per frame to cater for slider
        blockXSize = ceil(width  * windowXSize / divisionsX);
        blockYSize = ceil(height * windowYSize / divisionsY);
		if(blockXSize<1){ blockXSize=1; }
		if(blockYSize<1){ blockYSize=1;	}

		if((width * windowXStart) - (width * windowXSize  / 2.0)<0){
			windowXStart = 0.0 + windowXSize  / 2.0; 
		}
		if((width * windowXStart) + (width * windowXSize  / 2.0)>width){
			windowXStart = 1.0 - windowXSize  / 2.0; 
		}
		if((height * windowYStart) - (height * windowYSize  / 2.0)<0){
			windowYStart = 0.0 + windowYSize  / 2.0; 
		}
		if((height * windowYStart) + (height * windowYSize  / 2.0)>height){
			windowYStart = 1.0 - windowYSize  / 2.0; 
		}

        // Reset our colour counters for each block
        uchar redSum[blockXSize*blockYSize];
        uchar greenSum[blockXSize*blockYSize];
        uchar blueSum[blockXSize*blockYSize];
        int imageSum[blockXSize*blockYSize];		

		ij = 0;
        // Loop through each block vertically
        for (int yLoop = (height * windowYStart) - (height * windowYSize  / 2.0); yLoop < (height * windowYStart) + (height * windowYSize  / 2.0); yLoop += blockYSize)
        {
			ii = 0;
	        // Loop through each block horizontally
	        for (int xLoop = (width * windowXStart) - (width * windowXSize  / 2.0); xLoop < (width * windowXStart) + (width * windowXSize  / 2.0); xLoop += blockXSize)
	        {
                // Read every pixel in the block and calculate the average colour
                for (int pixXLoop = 0; pixXLoop < blockXSize; pixXLoop++)
                {
 
                    for (int pixYLoop = 0; pixYLoop < blockYSize; pixYLoop++)
                    {
						if((yLoop + pixYLoop < height) && (xLoop + pixXLoop < width)){
		                    imageSum[pixXLoop*blockYSize+pixYLoop]=blackWhite_im[yLoop + pixYLoop][xLoop + pixXLoop];
		                    // Get the pixel colour from the webcam stream
		                    ptr = cvPtr2D(pFrame, yLoop + pixYLoop, xLoop + pixXLoop, NULL);	 
						}else{
							imageSum[pixXLoop*blockYSize+pixYLoop]=255;
							ptr[0]=255;
							ptr[1]=255;
							ptr[2]=255;
						}
	                    // Add each component to its sum
	                    redSum[pixXLoop*blockYSize+pixYLoop] = ptr[2];
	                    greenSum[pixXLoop*blockYSize+pixYLoop] = ptr[1];
	                    blueSum[pixXLoop*blockYSize+pixYLoop] = ptr[0];
 
                    } // End of inner y pixel counting loop
 
                } // End of outer x pixel countier loop
                std::sort(imageSum, imageSum + blockXSize * blockYSize);
				std::sort(redSum, redSum + blockXSize * blockYSize);
				std::sort(greenSum, greenSum + blockXSize * blockYSize);
				std::sort(blueSum, blueSum + blockXSize * blockYSize);

				div=9;
                // Calculate the average colour of the block

				if(!close_eye){
		            red   = redSum[blockXSize*blockYSize/div];
		            green = greenSum[blockXSize*blockYSize/div];
		            blue  = blueSum[blockXSize*blockYSize/div];
		            blackWhite = imageSum[blockXSize*blockYSize/div];
				}else{
		            red   = 0;
		            green = 0;
		            blue  = 0;
		            blackWhite = 0;
				}
				if(blackWhite>255 && div>0){ // && blackWhite<=0
					div=div-1;
	                blackWhite = imageSum[blockXSize*blockYSize/div];
				}

                //Si queremos probar como se vería todo en un solo color
                if(useBlackWhite){
                    red   = blackWhite;
                    green = 0;
                    blue  = 0;
                }
				redcolor = c;
				greencolor = c;
				bluecolor = c;
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
	 	returnValue = write (fd0, iimg1, 198);
 		returnValue = write (fd1, iimg0, 198);
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

