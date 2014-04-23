#include <ntk/ntk.h>
#include <ntk/utils/debug.h>
#include <ntk/camera/openni_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.cpp>
#include <QApplication>
#include <QDir>
#include <QMutex>
using namespace cv;
using namespace ntk;
namespace opt
{
ntk::arg<bool> high_resolution("--highres", "High resolution color image.", 0);
ntk::arg<int> kinect_id("--kinect-id", "Kinect id", 0);
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

    char last_c = 0;
    while ( true && (last_c != 27) )
    {
        // Wait for a new frame, get a local copy and postprocess it.
        grabber.waitForNextFrame();
	grabber.copyImageTo(image);
	post_processor.processImage(image);
        // Prepare the depth view, mapped onto rgb frame.
        //cv::Mat1b debug_depth_img = 255-normalize_toMat1b(image.mappedDepth());
        // Prepare the color view with skeleton and handpoint.
	compute_color_encoded_depth(image.depth(), depth_as_color, &zmin, &zmax);
        cv::Mat3b debug_color_img;
        image.rgb().copyTo(debug_color_img);
	imshow("depth", depth_as_color);
	//imshow("depth", debug_depth_img);
        imshow("color", debug_color_img);
        last_c = (cv::waitKey(10) & 0xff);
	if( last_c == 32 )
	{
		record.saveCurrentFrame(image);
		last_c = 0; 
		printf("Image %08d saved\n", image.timestamp());
		fflush(stdout);
	}
    }

    grabber.stop();

}

