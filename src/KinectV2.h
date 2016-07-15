//
//  ofxKinectV2.h
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#pragma once

#include "Protonect.h"

typedef std::shared_ptr<class KinectV2> KinectV2Ref;

class KinectV2 {
    
public:
    
    struct KinectDeviceInfo{
        std::string serial;
        int deviceId;   //if you have the same devices plugged in device 0 will always be the same Kinect
        int freenectId; //don't use this one - this is the index given by freenect2 - but this can change based on order device is plugged in
    };
    
    class Options {
    public:
        Options(): _distMin(500), _distMax(6000), _color(true), _ir(true), _depth(true), _timeout(5){}
        int minDistance(){ return _distMin; }
        Options& minDistance(int dist){ _distMin = dist; return *this; }
        int maxDistance(){ return _distMax; }
        Options& maxDistance(int dist){ _distMax = dist; return *this; }
        bool color(){ return _color; }
        Options& color(bool color){ _color = color; return *this; }
        bool ir(){ return _ir; }
        Options& ir(bool ir){ _ir = ir; return *this; }
        bool depth(){ return _depth; }
        Options& depth(bool depth){ _depth = depth; return *this; }
        float timeout(){ return _timeout; }
        Options& timeout(float timeout){ _timeout = timeout; return *this; }
    protected:
        int _distMin, _distMax;
        bool _color, _ir, _depth;
        float _timeout;
    };
    
    static KinectV2Ref create(Options opts=Options());
    static KinectV2Ref create(std::string serial, Options opts=Options());
    static KinectV2Ref create(unsigned int deviceId, Options opts=Options());
    ~KinectV2();
    
    std::vector <KinectDeviceInfo> getDeviceList();
    unsigned int getNumDevices();
    
    bool open(std::string serial);
    bool open(unsigned int deviceId = 0);
    void update();
    void close();
    
    bool isOpen();
    bool checkFrameNew(){
        return checkRGBFrameNew() || checkIRFrameNew() || checkDepthFrameNew();
    }
    bool checkRGBFrameNew();
    bool checkIRFrameNew();
    bool checkDepthFrameNew();
    
    ci::Surface8uRef getSurfaceRGB();
    ci::Channel32fRef getChannelIR();
    ci::Channel32fRef getChannelDepth();
    
    
    int getMinDist();
    void setMinDist(int dist);
    int getMaxDist();
    void setMaxDist(int dist);
    float getTimeout();
    void setTimeout(float timeout);
    
protected:
    KinectV2(Options opt);
    bool isThreadRunning();
    void startThread();
    void stopThread();
    void threadedFunction();
    
    ci::Surface8uRef _surfaceRGB;
    ci::Channel32fRef _channelIR;
    ci::Channel32fRef _channelDepth;
    
    std::shared_ptr<std::thread> _thread;
    bool _threadRunning;
    std::recursive_mutex _lock;
    
    bool bNewRGBFrame, bNewIRFrame, bNewDepthFrame;
    bool bOpened;
    
    Options _opts;
    
    Protonect protonect;
    
    ci::Timer timeout;
};
