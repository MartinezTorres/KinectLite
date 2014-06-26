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
// GCC-FLAGS: -Ofast -lusb-1.0 -std=c++0x  `pkg-config opencv --cflags` `pkg-config opencv --libs` -lpython2.7 -shared -Wl,--export-dynamic -lboost_python -o KinectLite.so -I/usr/include/python2.7 -fPIC 


#include <KinectLite.h>

#include <opencv/cv.h>

#include <boost/python.hpp>
#include <numpy/arrayobject.h>


using namespace boost::python;

struct KinectLitePython {
	
	std::shared_ptr<KinectLite> kl;

	bool isOpen() {
		
		if (not kl) kl = std::shared_ptr<KinectLite>(new KinectLite);
		return kl->isOpen();
	}
	
	PyObject* getBGR() {
		
		if (not kl) kl = std::shared_ptr<KinectLite>(new KinectLite);
		kl->startImg();
		std::vector<uint8_t> I = kl->getBGR();
		cv::Mat3b M(480,640,(cv::Vec3b *)&I[0]);

        npy_intp dims[3]    = { M.rows, M.cols, 1};
        npy_intp strides[3] = { M.step, M.elemSize(), M.elemSize1()};
        return PyArray_New(&PyArray_Type, 2, dims, NPY_UBYTE, strides, M.data, 0, NPY_WRITEABLE | NPY_C_CONTIGUOUS, NULL);
	}
	
 	PyObject* getDepth() {
		
		if (not kl) kl = std::shared_ptr<KinectLite>(new KinectLite);
		kl->startDepth();
		std::vector<uint16_t> D = kl->getDepth();
		cv::Mat1s M(480, 640, (int16_t *)&D[0]);

        npy_intp dims[3]    = { M.rows, M.cols, 1};
        npy_intp strides[3] = { M.step, M.elemSize(), M.elemSize1()};
        return PyArray_New(&PyArray_Type, 2, dims, NPY_USHORT, strides, M.data, 0, NPY_WRITEABLE | NPY_C_CONTIGUOUS, NULL);
	}
}; 

BOOST_PYTHON_MODULE(KinectLite)
{
    class_<KinectLitePython>("KinectLite")
        .def("isOpen", &KinectLitePython::isOpen)
        .def("getBGR", &KinectLitePython::getBGR)
        .def("getDepth", &KinectLitePython::getDepth)
   ;
}
