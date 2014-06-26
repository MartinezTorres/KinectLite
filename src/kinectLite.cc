////////////////////////////////////////////////////////////////////////
// Simple Kinect driver
//
// Manuel Martinez (manuel.martinez@kit.edu)
//
//
// If module gspca_kinect gets in the way do: 
//    echo blacklist gspca_kinect | sudo tee  /etc/modprobe.d/blacklist-kinect.conf
//
// If you need root permissions do:
//    echo -e 'SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"'\\n'SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"'\\n'SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"' | sudo tee /etc/udev/rules.d/51-kinect.rules
// 
// license: LGPLv3
//
// GCC-FLAGS: -O3 -g -lusb-1.0 -std=c++0x `pkg-config opencv --cflags` `pkg-config opencv --libs`


#include <KinectLite.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

int main() {
	
	KinectLite kinect;
	
	if (not kinect.isOpen()) return -1;
	
	kinect.tilt(0);
	
//	kinect.startDepth();
//	kinect.startImg();

//	kinect.startDepth(false, false, false); //Start depth with filters disabled
	kinect.startImg(true, 2); //Start IR at high resolution

	while (cv::waitKey(10)%256 != 'q') {
		
//		vector<uint16_t> D = kinect.getDepth();
//		cv::Mat1s disp = cv::Mat1s(480, 640, (int16_t *)&D[0]).clone();
//		cv::imshow("depth", cv::Mat1b((1024-disp)*.5));

//		vector<uint8_t> I = kinect.getBGR();
//		cv::Mat3b imgbgr = cv::Mat3b(480,640,(cv::Vec3b *)&I[0]).clone();
//		cv::imshow("img", imgbgr);

		vector<uint16_t> IR = kinect.getIR();
//		cv::Mat1d imgIR = cv::Mat1d(cv::Mat1s(488,640,(int16_t *)&IR[0]))*(1./1024);
		cv::Mat1d imgIR = cv::Mat1d(cv::Mat1s(1024,1280,(int16_t *)&IR[0]))*(1./1024);
		cv::imshow("imgIR", imgIR);
		
		cv::imwrite("img.png", cv::Mat1b(imgIR*255));

		
	}
	 return 0;
}
