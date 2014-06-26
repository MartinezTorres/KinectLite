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
// GCC-FLAGS: -g -Ofast -lusb-1.0 -std=c++0x `pkg-config opencv --cflags` `pkg-config opencv --libs`


#include <KinectLite.h>

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

static inline void MSLEEP( int n = 1) { std::this_thread::sleep_for( std::chrono::milliseconds(n) ); }

int main() {

	
	cv::Mat1d P(488*8,(640+200)*8, 0.), W(488*8,(640+200)*8,1E-200);

	cout << "Start Kinect" << endl;
	KinectLite kinect;
	if (not kinect.isOpen()) return 0;
	kinect.startImg();
	kinect.startDepth(false, false, false); //Start depth with filters disabled
	kinect.startImg(true, 1); //Start IR at high resolution
	
	int nImages = 16;
	for (int nImage = 0; nImage<nImages; nImage++) {
		
		cout << "Processing Image Nº" << nImage+1 << endl;
		cout << "Start in 3.." << endl;	sleep(1);
		cout << "Start in 2.." << endl;	sleep(1);
		cout << "Start in 1.." << endl;	sleep(1);
		
		vector<uint16_t> disp, ir;
		
		for (int i=0; i<15; i++) //Discard first 15 IR and depth images
			kinect.getDepth();
		
		cout << "Get Depth Image" << endl;
		for (int i=0; i<15; i++) { //On the following 15 depth images, discard pixels which values are not the same
			vector<uint16_t> disp2 = kinect.getDepth();
			if (disp.empty())
				disp = disp2;
			else
				for (int k=0; k<480*640; k++)
					if (disp2[k]!=disp[k])
						disp[k]=2047;
		}

		cout << "Get IR Image" << endl;
		for (int i=0; i<2; i++) //Discard first IR image
			ir = kinect.getIR();
			

		cout << "Calculate a Pattern" << endl;
		cv::Mat1d D = cv::Mat1d(cv::Mat1w(480, 640, &disp[0]))(cv::Rect(0,0,632,480));
		cv::Mat1d I = cv::Mat1d(cv::Mat1w(488, 640, &ir[0]))/1024.;
		cv::resize(I, I, cv::Size(640*8,488*8), 0, 0, cv::INTER_CUBIC);
		for (int i=0; i<D.rows; i++) {
			for (int j=0; j<D.cols; j++) {

				int d = D(i,j);
				if (d==2047) continue;
				
				P(cv::Rect(j*8+8+d,i*8+8,48,48)) += I(cv::Rect(j*8+8,i*8+8,48,48));
				W(cv::Rect(j*8+8+d,i*8+8,48,48)) += 1;
			}
		}
	}
	cv::imwrite("pattern.png", cv::Mat1w(P*65535./W));	
	return 0;
}
