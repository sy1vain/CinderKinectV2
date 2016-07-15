//
//  ofxKinectV2.h
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#pragma once

#include <vector>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <libfreenect2/usb/transfer_pool.h>
#include <libfreenect2/usb/event_loop.h>

typedef std::shared_ptr<class KinectV2> KinectV2Ref;

class KinectV2 : public std::enable_shared_from_this<KinectV2> {
    
public:
    
    struct KinectDeviceInfo{
        std::string serial;
        int deviceId;
        int freenectId;
    };
    
    struct Options{
        
    };
    
    static KinectV2Ref create();
    ~KinectV2();
    
    static std::vector <KinectDeviceInfo> getDeviceList();
    static unsigned int getNumDevices(){
        return getDeviceList().size();
    }
    
    void open(KinectDeviceInfo kinect);
    void open(unsigned int deviceId=0);
    void open(const std::string& serial);
    void close(bool wait=false);
    
    bool isBusy();
    bool isOpen();
    bool isOpening();
    bool isClosing();
    
    bool checkFrameNew(){
        return checkRGBFrameNew() || checkIRFrameNew() || checkDepthFrameNew();
    }
    bool checkRGBFrameNew();
    bool checkIRFrameNew();
    bool checkDepthFrameNew();
    
    ci::Surface8uRef getSurfaceRGB();
    ci::Channel32fRef getChannelIR();
    ci::Channel32fRef getChannelDepth();
    
    int minDistance();
    KinectV2Ref minDistance(int dist);
    int maxDistance();
    KinectV2Ref maxDistance(int dist);
    
    bool colorEnabled();
    KinectV2Ref colorEnabled(bool enabled);
    bool irEnabled();
    KinectV2Ref irEnabled(bool enabled);
    bool depthEnabled();
    KinectV2Ref depthEnabled(bool enabled);
    
protected:
    KinectV2();
    
    static std::shared_ptr<libfreenect2::Freenect2> getFreenect(){
        static std::shared_ptr<libfreenect2::Freenect2> freenect;
        if(!freenect) freenect = std::shared_ptr<libfreenect2::Freenect2>(new libfreenect2::Freenect2());
        return freenect;
    }
    
    
    void joinThread();
    void startThread(const std::string serial);
    void runDevice(const std::string serial);
    
    void handleRGBFrame(libfreenect2::Frame *frame);
    void handleIRFrame(libfreenect2::Frame *frame);
    void handleDepthFrame(libfreenect2::Frame *frame);
    
    bool _opened, _opening, _closing;
    
    bool _newRGBFrame, _newIRFrame, _newDepthFrame;
    ci::Surface8uRef _surfaceRGB;
    ci::Channel32fRef _channelIR;
    ci::Channel32fRef _channelDepth;
    
    std::shared_ptr<std::thread> _thread;
    std::recursive_mutex _recursive_mutex;
    
    //settings
    int _distMin, _distMax;
    bool _color, _ir, _depth;
    float _timeout;
};
