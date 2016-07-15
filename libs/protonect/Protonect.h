//  ofProtonect.cpp
//
//  Created by Theodore Watson on 11/16/15


//#include "ofMain.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <libfreenect2/usb/transfer_pool.h>
#include <libfreenect2/usb/event_loop.h>

class Protonect{

    public:

    struct Frame {
        ci::Surface8uRef rgb;
        ci::Channel32fRef ir;
        ci::Channel32fRef depth;
    };
    
        Protonect();
    
        int openKinect(std::string serialNo, unsigned int frames = libfreenect2::Frame::Color|libfreenect2::Frame::Ir|libfreenect2::Frame::Depth);
        Frame updateKinect(int minDist=50, int maxDist=8000);
        int closeKinect();
        bool isOpened();
    
        libfreenect2::Freenect2 & getFreenect2Instance(){
            return freenect2;
        }
    
    protected:
  
        bool bOpened;
        
        libfreenect2::Freenect2 freenect2;

        libfreenect2::Freenect2Device *dev = 0;
        libfreenect2::PacketPipeline *pipeline = 0;

        libfreenect2::FrameMap frames;

        libfreenect2::Registration* registration;
        libfreenect2::SyncMultiFrameListener * listener;
        libfreenect2::Frame  * undistorted = NULL;
        libfreenect2::Frame  * registered = NULL;
        libfreenect2::Frame  * bigFrame = NULL;

};