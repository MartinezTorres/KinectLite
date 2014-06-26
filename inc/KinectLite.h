////////////////////////////////////////////////////////////////////////
// Stripped down Kinect driver.
//
// Manuel Martinez (manuel.martinez@kit.edu)
//
// Thanks to OpenKinect.org for the protocol documentation.
//
// license: LGPLv3

#pragma once

#include <libusb-1.0/libusb.h>

#include <thread>

#include <vector>
#include <array>
#include <queue>
#include <cstring>
#include <cmath>

class KinectLite {
private:

	static const int NTRANSFERS=16;
	static const int NPACKS=16;

	libusb_context *context;
	libusb_device_handle *camera, *motor;

	bool isoPkRecv;
	std::vector<libusb_transfer *> libusbTransfers;
	
	struct KinectTransfer {
		
		std::string data;
		uint8_t seq;
		bool sync;
		uint32_t size;
		uint32_t remaining;
		std::queue<std::string> q;
		bool enabled;
	};
	
	KinectTransfer iInfo, dInfo;

	bool isGood;
	std::mutex mtx;
	std::thread t;
	typedef std::lock_guard<std::mutex> Lock;

	static void MSLEEP( int n = 1) { std::this_thread::sleep_for( std::chrono::milliseconds(n) ); }


	void writeReg(int reg, int val) {
		uint8_t data[] = { 'G', 'M', 2, 0, 3, 0, uint8_t(0&255), uint8_t((0>>8)&255), uint8_t(reg&255), uint8_t((reg>>8)&255), uint8_t(val&255), uint8_t((val>>8)&255) };
		libusb_control_transfer( camera, 0x40, 0, 0, 0, data, sizeof(data), 0 );
		do MSLEEP(1); while (not libusb_control_transfer( camera, 0xc0, 0, 0, 0,  data, sizeof(data), 0 ));
	}

	////////////////////////////////////////////////////////////////////
	// Communication
	
	void run() {
		
		while (isGood and context==NULL) MSLEEP(1);
		timeval tv; tv.tv_sec = 0; tv.tv_usec = 1; 
		while (isGood) {
			
			isoPkRecv = false;
			libusb_handle_events_timeout(context, &tv);
			if (not isoPkRecv) MSLEEP(1);
		}
	};
	
	void isoCB(struct libusb_transfer *transfer) {
		
		KinectTransfer &kt = (transfer->endpoint==0x81)?iInfo:dInfo;
		
		isoPkRecv = true;

		if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
			
			kt.sync = false;			
			if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE)
				isGood = false;

			transfer->status = LIBUSB_TRANSFER_COMPLETED;
			if (isGood and kt.enabled) libusb_submit_transfer(transfer);
			
			return;
		}
			
		
		for (int i=0; i<transfer->num_iso_packets; i++) {
			
			uint8_t *buf = &((uint8_t*)transfer->buffer)[1920*i];
			uint32_t length = transfer->iso_packet_desc[i].actual_length;

			if (length < 12) continue;
			
			if (length == kt.remaining and (buf[0] != 'R' or buf[1] != 'B')) {
				
				printf("%d %d\n", kt.remaining, length);
				
				memcpy( &kt.data[kt.size], &buf[0], length );
				kt.size += length;
				kt.remaining -= length;
				continue;
			}
			
			
			if (kt.remaining or buf[0] != 'R' or buf[1] != 'B' or buf[5] != kt.seq or length+kt.size-12 > kt.data.size() ) kt.sync = false;
			++kt.seq;

			uint type = buf[3]&15;
			uint hlength = (buf[6]<<8)+buf[7];
			if ( type==1 ) {
				kt.sync=true;
				kt.seq =buf[5]+1;
				kt.size=0;
				kt.remaining=0;
			}

			if ( type!=5 and length==960 and hlength>960)
				kt.remaining = hlength - 960;
			else if (length != hlength)
				kt.sync=false;

			if ( not kt.sync ) { continue; };
			
			memcpy( &kt.data[kt.size], &buf[12], length-12 );
			kt.size += length-12;

			if ( type==5 ) {

				Lock l(mtx);
				kt.q.push(kt.data.substr(0, kt.size));					

				if( kt.q.size() > 2 ) kt.q.pop();
				kt.size=0;
				kt.sync=false;
			}			
		}
		if (isGood and kt.enabled) libusb_submit_transfer(transfer);
	}

	std::string getRAW( KinectTransfer &kt, double timeOut=2. ) {
		
		while (isGood and timeOut>0. and kt.q.empty()) {
			MSLEEP(1);
			timeOut -= 0.001;
		}
		
		if (kt.q.empty()) return "";
		
		Lock l(mtx);
		std::string data = kt.q.front(); 
		kt.q.pop(); 
		return data; 
	}
	
public:
	KinectLite() :
		context(NULL), 
		camera(NULL), 
		motor(NULL), 
		isGood(true), 
		t([&, this](){this->run();}) {
		
		libusb_init( &context );

		libusb_device **devs;
		int cnt = libusb_get_device_list(context, &devs);
		for (int i =0; i<cnt; i++) {
			struct libusb_device_descriptor desc;
			libusb_get_device_descriptor(devs[i], &desc);
			
			if ( !camera && desc.idVendor == 0x045e && desc.idProduct == 0x02ae && !libusb_open(devs[i], &camera) )
				if( libusb_claim_interface (camera, 0) ) 
					{ libusb_close( camera ); camera = NULL; }

			if ( !motor  && desc.idVendor == 0x045e && desc.idProduct == 0x02b0 && !libusb_open(devs[i], &motor ) ) 
				if( libusb_claim_interface (motor, 0) ) 
					{ libusb_close( motor ); motor = NULL; }
		}
		libusb_free_device_list (devs, 1);

		isGood = camera;
		if (not isGood) return;
		
		iInfo.data.resize(2000000);
		dInfo.data.resize(2000000);
		iInfo.size = dInfo.size = 0;
		iInfo.remaining = dInfo.remaining = 0;
		iInfo.sync = dInfo.sync = false;
		iInfo.enabled = dInfo.enabled = false;
		
		auto CB = [](struct libusb_transfer *transfer){((KinectLite*)transfer->user_data)->isoCB(transfer);};
		
		for (int i=0; i<NTRANSFERS; i++) {
			libusbTransfers.push_back( libusb_alloc_transfer(NPACKS) );
			libusb_fill_iso_transfer( libusbTransfers.back(), camera, 0x81, new uint8_t[1920*NPACKS], 1920*NPACKS, NPACKS, CB, (void *)this, 0);
			libusb_set_iso_packet_lengths( libusbTransfers.back(), 1920);

			libusbTransfers.push_back( libusb_alloc_transfer(NPACKS) );
			libusb_fill_iso_transfer( libusbTransfers.back(), camera, 0x82, new uint8_t[1920*NPACKS], 1920*NPACKS, NPACKS, CB, (void *)this, 0);
			libusb_set_iso_packet_lengths( libusbTransfers.back(), 1920);
		}
	}

	~KinectLite() { 
	
		if (iInfo.enabled) stopImg();
		if (dInfo.enabled) stopDepth();
		isGood = false;
		t.join();

		if (camera) { writeReg( 5,    0); writeReg( 6,    0); libusb_release_interface( camera, 0); libusb_close( camera ); camera=NULL; }
		if (motor) { libusb_release_interface( motor, 0); libusb_close( motor ); motor=NULL; }
		
		for (uint8_t i=0; i<libusbTransfers.size(); i++) {
			delete[] libusbTransfers[i]->buffer;
			libusb_free_transfer(libusbTransfers[i]);
		}
		
		if (context) { libusb_exit( context ); context = NULL; }
		
	}
	
	bool isOpen() { return isGood; }

	////////////////////////////////////////////////////////////////////
	// Tilting and Leds

	double tilt() {

		if (not motor) return -180.;
		
		auto limit = std::chrono::system_clock::now()+std::chrono::seconds(2);
		while (std::chrono::system_clock::now() < limit) {
			
			uint8_t data[10];
			libusb_control_transfer(motor, 0xc0, 0x32, 0, 0, data, 10, 0);
			if (data[9]==0 and data[8]!=128) return 0.5*int8_t(data[8]);
			MSLEEP(10);
		}

		return -180;
	}
	
	double tilt( double angle ) {

		if (not motor) return -180.;
		
		libusb_control_transfer(motor, 0x40, 0x31,int(round(2*angle)), 0, NULL, 0, 0);	
		
		auto limit = std::chrono::system_clock::now()+std::chrono::seconds(2);
		while (std::chrono::system_clock::now() < limit) {

			uint8_t data[10];
			libusb_control_transfer(motor, 0xc0, 0x32, 0, 0, data, 10, 0);
			if( data[9]==0 ) break; // Goal reached!
			if( data[9]==1 ) libusb_control_transfer(motor, 0x40, 0x31, 0, 0, NULL, 0, 0); //Stall!
			MSLEEP(10);
		}
		
		return tilt();
	}
		
	std::vector<double> acc() {
		
		if ( not motor ) return std::vector<double>(3,0);

		uint8_t data[10];
		libusb_control_transfer(motor, 0xc0, 0x32, 0, 0, data, 10, 0);
		
		std::vector<double> r(3);

		r[0] = 9.81*(data[2]*256+data[3])/819.;
		r[1] = 9.81*(data[4]*256+data[5])/819.;
		r[2] = 9.81*(data[6]*256+data[7])/819.;
		
		return r;
	}
	
	void ledOff()    {	if (motor) libusb_control_transfer(motor, 0x40, 6, 0, 0, NULL, 0, 0); }
	void ledGreen()  {	if (motor) libusb_control_transfer(motor, 0x40, 6, 1, 0, NULL, 0, 0); }
	void ledRed()    {	if (motor) libusb_control_transfer(motor, 0x40, 6, 2, 0, NULL, 0, 0); }
	void ledYellow() {	if (motor) libusb_control_transfer(motor, 0x40, 6, 3, 0, NULL, 0, 0); }
	void ledBGreen() {	if (motor) libusb_control_transfer(motor, 0x40, 6, 4, 0, NULL, 0, 0); }
	void ledRedYel() {	if (motor) libusb_control_transfer(motor, 0x40, 6, 6, 0, NULL, 0, 0); }
	
	////////////////////////////////////////////////////////////////////
	// Depth Stream

	void startDepth( bool hole=true, bool gmc=true, bool wBal=true ) {
		
		if (not isGood) return;
		
		if (dInfo.enabled) stopDepth();

		dInfo.enabled = true;
		
		for (uint32_t i=0; i<libusbTransfers.size(); i++)
			if (libusbTransfers[i]->endpoint==0x82)
				libusb_submit_transfer( libusbTransfers[i] );

		writeReg( 6,    0); // STREAM1_MODE

		writeReg(0x105, 0); // Disable auto-cycle of projector
		writeReg(26,    2); // PARAM_IR_RESOLUTION
		writeReg(27,   30); // PARAM_IR_FPS
		writeReg(18,    1); // PARAM_DEPTH_FORMAT
		writeReg(19,    1); // PARAM_DEPTH_RESOLUTION
		writeReg(20,   30); // PARAM_DEPTH_FPS
		writeReg(22, hole); // PARAM_DEPTH_HOLE_FILTER
		writeReg(36,  gmc); // PARAM_DEPTH_GMC_MODE
		writeReg(45, wBal); // PARAM_DEPTH_WHITE_BALANCE_ENABLE
		writeReg(62,    1); // PARAM_APC_ENABLE

		writeReg( 6,    2); // STREAM1_MODE
		getDepthRAW();
	}
	
	void stopDepth() {
		
		if (not isGood ) return;
		dInfo.enabled = false;
		writeReg( 6,    0); // STREAM1_MODE
	}
	
	
	static std::vector<uint16_t> dToDISP( const std::string in ) {

		std::vector<uint16_t> out;
		out.reserve(640*480);
		
		struct { int c; const uint8_t *i; int pop() { return c++&1?*i++&15:*i>>4; } } t = { 0, (const uint8_t*)&in[0] };
		
		int value = 0;
		while (out.size() < 640*480-1) {
			
			int c = t.pop();
			if( c == 0xf ) {				
				int aux  = (t.pop()<<4) + t.pop();
				if (aux & 0x80)
					value += aux-192;
				else
					value = (aux<<8) + (t.pop()<<4) + t.pop();
			} else if (c==0xe)
				for (int n=t.pop(); n; n--) out.push_back(value);
			else 
				value += c-6;
			//if (value>2047 or t.i >= in.size()) return std::vector<uint16_t>(640*480,2047);
			out.push_back(value);
		}
		return out;
	}

	std::string getDepthRAW( double timeOut=2. ) { return getRAW( dInfo, timeOut ); }
	std::vector<uint16_t> getDepth( double timeOut=2.) { return dToDISP(getDepthRAW(timeOut)); }
	
	static std::vector<double> getDefaultDisp2Depth(double fx = 580) {
		
		std::function<double(double)> depth = [&](int i){ return  0.075*fx/(100.-i/8.); };

		std::vector<double> disp2depth(2048);
		for (int i=0; i<2048; i++)
			disp2depth[i]=depth(i);
		return disp2depth;
	}

	std::vector<std::array<double, 3>> getPointCloud(double fx = 580) {
		
		static const double ifx = 1.0 / fx;
		static const double ify = 1.0 / fx;
		static const double cx = 320-4.5;
		static const double cy = 240-.5;
		
		std::vector<uint16_t> depth = getDepth();
		std::vector<double> d2d = getDefaultDisp2Depth(fx);

		std::vector<std::array<double, 3>> ret;		
		for (int i=0; i<480; i++) {
			for (int j=0; j<640; j++) {
				if (depth[i*480+j]==2047) continue;
				double d = d2d[depth[i*480+j]];
				ret.push_back( {{ d*(j-cx)*ifx, d*(i-cy)*ify, d }} );
			}
		}
				
		return ret;
	}


	////////////////////////////////////////////////////////////////////
	// RGB/Ir Stream

	static std::vector<uint8_t> cToUYVY( const std::string &in ) {
		
		std::vector<uint8_t> out(640*480*2);
		uint8_t* o = &out[0];

		struct { int c; const uint8_t *i; int pop() { return c++&1?*i++&15:*i>>4; } } t = { 0, (const uint8_t*)&in[0] };

		for (int k=0; k<480; k++) {
			int y=0, u=0, v=0, c;
			for (int j=0; j<640; j+=2) {
				
				c = t.pop();
				*o++ = u = (c == 0xf) ? (t.pop()<<4) + t.pop() : u + c - 6;

				c = t.pop();
				*o++ = y = (c == 0xf) ? (t.pop()<<4) + t.pop() : y + c - 6;

				c = t.pop();
				*o++ = v = (c == 0xf) ? (t.pop()<<4) + t.pop() : v + c - 6;

				c = t.pop();
				*o++ = y = (c == 0xf) ? (t.pop()<<4) + t.pop() : y + c - 6;
			}
		}
		return out;
	}
	
	static std::vector<uint8_t> UYVYToBGR( const std::vector<uint8_t> &in) {
		
		std::vector<uint8_t> out(6*(in.size()/4));

		uint8_t* o = &out[0];
		uint8_t* i = (uint8_t*)&in[0];

		for (uint count=0; count<out.size(); count+=6) {
			
			int y1, y2 ,u,v;
			v = *i++ - 128;
			y1 = *i++;
			u = *i++ - 128;
			y2 = *i++;
			
			*o++ = std::max(0, std::min(255, y1 + ((v*1436) >> 10)));
			*o++ = std::max(0, std::min(255, y1 - ((u*352 + v*731) >> 10)));
			*o++ = std::max(0, std::min(255, y1 + ((u*1814) >> 10)));
			
			*o++ = std::max(0, std::min(255, y2 + ((v*1436) >> 10)));
			*o++ = std::max(0, std::min(255, y2 - ((u*352 + v*731) >> 10)));
			*o++ = std::max(0, std::min(255, y2 + ((u*1814) >> 10)));
		}
		return out;
	}

	static std::vector<uint16_t> irTo16( const std::string &in ) {
		
		std::vector<uint16_t> out(in.size()*8/10);
		
		uint8_t *ip = (uint8_t *)&in[0];
		
		uint16_t *op = (uint16_t *)&out[0];
		
		for (uint32_t i=0; i<in.size(); i+=5) {
			*op++ = ((ip[i+0]<<2)|(ip[i+1]>>6))&1023;			
			*op++ = ((ip[i+1]<<4)|(ip[i+2]>>4))&1023;			
			*op++ = ((ip[i+2]<<6)|(ip[i+3]>>2))&1023;			
			*op++ = ((ip[i+3]<<8)|(ip[i+4]>>0))&1023;			
		}
		return out;
	}
	
	void startImg( bool ir = false, int sz=1) {

		if (not isGood) return;
		
		if (iInfo.enabled) stopImg(); // Already enabled

		iInfo.enabled = true;
		
		for (uint32_t i=0; i<libusbTransfers.size(); i++)
			if (libusbTransfers[i]->endpoint==0x81)
				libusb_submit_transfer( libusbTransfers[i] );
		
		int f[]={60, 30, 15};
		if (ir) {
			writeReg(0x105, 0); // Disable auto-cycle of projector
			writeReg(26,   sz); // PARAM_IR_RESOLUTION
			writeReg(27,f[sz]); // PARAM_IR_FPS
			writeReg( 5,    3);  // STREAM2_MODE
		} else {
			writeReg(26,    1); // PARAM_IR_RESOLUTION
			writeReg(27,   30); // PARAM_IR_FPS

			writeReg(12,    1); 
			writeReg(13,    1); 
			writeReg(14,   15);
			writeReg(71,    0); // disable Hflip
			writeReg( 5,    1);  // STREAM2_MODE
		}
		
		while (not iInfo.q.empty()) { Lock l(mtx); iInfo.q.pop(); }
		
		getImgRAW();
	}
	
	void stopImg() {
		
		if (not isGood) return;
		iInfo.enabled = false;
		writeReg( 5,    0); // STREAM2_MODE
	}
	
	std::string getImgRAW(double timeOut=2.) { return getRAW( iInfo, timeOut ); }
	std::vector<uint16_t> getIR(double timeOut=2.) { return irTo16(getImgRAW(timeOut)); }
	std::vector<uint8_t > getUYVY( double timeOut=2.) { return cToUYVY(getImgRAW(timeOut)); }
	std::vector<uint8_t > getBGR( double timeOut=2.) { return UYVYToBGR(cToUYVY(getImgRAW(timeOut))); }

};

