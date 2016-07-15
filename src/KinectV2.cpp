//
//  ofxKinectV2.cpp
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#include "KinectV2.h"
#include "cinder/Log.h"
#include "cinder/Utilities.h"

KinectV2Ref KinectV2::create(Options opts){
    return KinectV2Ref(new KinectV2(opts));
}

KinectV2Ref KinectV2::create(std::string serial, Options opts){
    KinectV2Ref kinect = KinectV2::create(opts);
    kinect->open(serial);
    return kinect;
}

KinectV2Ref KinectV2::create(unsigned int deviceId, Options opts){
    KinectV2Ref kinect = KinectV2::create(opts);
    kinect->open(deviceId);
    return kinect;
}

//--------------------------------------------------------------------------------
KinectV2::KinectV2(Options opts){
    bNewRGBFrame = bNewIRFrame = bNewDepthFrame  = false;
    bOpened    = false;
    
    _opts = opts;
}

//--------------------------------------------------------------------------------
KinectV2::~KinectV2(){
    close();
}

//--------------------------------------------------------------------------------
static bool sortBySerialName( const KinectV2::KinectDeviceInfo & A, const KinectV2::KinectDeviceInfo & B ){
    return A.serial < B.serial;
}

//--------------------------------------------------------------------------------
std::vector <KinectV2::KinectDeviceInfo> KinectV2::getDeviceList(){
    std::vector <KinectDeviceInfo> devices;
    
    int num = protonect.getFreenect2Instance().enumerateDevices();
    for (int i = 0; i < num; i++){
        KinectDeviceInfo kdi;
        kdi.serial = protonect.getFreenect2Instance().getDeviceSerialNumber(i);
        kdi.freenectId = i; 
        devices.push_back(kdi);
    }
    
    for (int i = 0; i < num; i++){
        devices[i].deviceId = i;
    }
    
    return devices;
}

//--------------------------------------------------------------------------------
unsigned int KinectV2::getNumDevices(){
   return getDeviceList().size(); 
}

//--------------------------------------------------------------------------------
bool KinectV2::open(unsigned int deviceId){
    
    std::vector <KinectDeviceInfo> devices = getDeviceList();
    
    if( devices.size() == 0 ){
        CI_LOG_E("no devices connected");
        return false;
    }
    
    if( deviceId >= devices.size() ){
        CI_LOG_E(" deviceId " << deviceId << " is bigger or equal to the number of connected devices " << devices.size());
        return false;
    }

    std::string serial = devices[deviceId].serial;
    return open(serial);
}

//--------------------------------------------------------------------------------
bool KinectV2::open(std::string serial){
    close(); 
    
    bNewRGBFrame = bNewIRFrame = bNewDepthFrame  = false;
    bOpened    = false;
    
    unsigned int frames = 0;
    if(_opts.color()) frames |= libfreenect2::Frame::Color;
    if(_opts.ir()) frames |= libfreenect2::Frame::Ir;
    if(_opts.depth()) frames |= libfreenect2::Frame::Depth;
    
    int retVal = protonect.openKinect(serial, frames);
    
    if(retVal!=0){
        return false;
    }
    
    startThread();
    
    bOpened = true;
    return true;
}

//--------------------------------------------------------------------------------
bool KinectV2::isThreadRunning(){
    std::lock_guard<std::recursive_mutex> locked(_lock);
    return _threadRunning && protonect.isOpened();
}

void KinectV2::startThread(){
    if(isThreadRunning()) stopThread();
    
    std::lock_guard<std::recursive_mutex> locked(_lock);
    _threadRunning = true;
    
    _thread = std::shared_ptr<std::thread>(new std::thread(std::bind(&KinectV2::threadedFunction, this)));
}

void KinectV2::stopThread(){
    {
        std::lock_guard<std::recursive_mutex> locked(_lock);
        _threadRunning = false;
    }
    if(_thread && _thread->joinable()){
        _thread->join();
    }
    _thread.reset();
}




//--------------------------------------------------------------------------------
void KinectV2::threadedFunction(){
    timeout.start();
    while(isThreadRunning()){
        Protonect::Frame frame = protonect.updateKinect(getMinDist(), getMaxDist());
        {
            std::lock_guard<std::recursive_mutex> locked(_lock);
            if(frame.rgb){
                _surfaceRGB = frame.rgb;
                bNewRGBFrame = true;
                timeout.start();
            }
            if(frame.ir){
                _channelIR = frame.ir;
                bNewIRFrame = true;
                timeout.start();
            }
            if(frame.depth){
                _channelDepth = frame.depth;
                bNewDepthFrame = true;
                timeout.start();
            }
        }
        
        ci::sleep(15);
    }
    
    CI_LOG_D("Thread stopped!");
}

//--------------------------------------------------------------------------------
void KinectV2::update(){
}

//--------------------------------------------------------------------------------
bool KinectV2::isOpen(){
    return protonect.isOpened();
}

//--------------------------------------------------------------------------------
bool KinectV2::checkRGBFrameNew(){
    if(timeout.getSeconds()>getTimeout()) close();
    return bNewRGBFrame;
}
bool KinectV2::checkIRFrameNew(){
    if(timeout.getSeconds()>getTimeout()) close();
    return bNewIRFrame;
}
bool KinectV2::checkDepthFrameNew(){
    if(timeout.getSeconds()>getTimeout()) close();
    return bNewDepthFrame;
}

//--------------------------------------------------------------------------------
ci::Surface8uRef KinectV2::getSurfaceRGB(){
    std::lock_guard<std::recursive_mutex> locked(_lock);
    bNewRGBFrame = false;
    return _surfaceRGB;
}

//--------------------------------------------------------------------------------
ci::Channel32fRef KinectV2::getChannelIR(){
    std::lock_guard<std::recursive_mutex> locked(_lock);
    bNewIRFrame = false;
    return _channelIR;
}

//--------------------------------------------------------------------------------
ci::Channel32fRef KinectV2::getChannelDepth(){
    std::lock_guard<std::recursive_mutex> locked(_lock);
    bNewDepthFrame = false;
    return _channelDepth;
}

//--------------------------------------------------------------------------------
int KinectV2::getMinDist(){
    std::lock_guard<std::recursive_mutex> locked(_lock);
    return _opts.minDistance();
}

//--------------------------------------------------------------------------------
void KinectV2::setMinDist(int dist){
    std::lock_guard<std::recursive_mutex> locked(_lock);
    _opts.minDistance(dist);
}

//--------------------------------------------------------------------------------
int KinectV2::getMaxDist(){
    std::lock_guard<std::recursive_mutex> locked(_lock);
    return _opts.maxDistance();
}

//--------------------------------------------------------------------------------
void KinectV2::setMaxDist(int dist){
    std::lock_guard<std::recursive_mutex> locked(_lock);
    _opts.maxDistance(dist);
}

//--------------------------------------------------------------------------------
float KinectV2::getTimeout(){
    std::lock_guard<std::recursive_mutex> locked(_lock);
    return _opts.timeout();
}

//--------------------------------------------------------------------------------
void KinectV2::setTimeout(float timeout){
    std::lock_guard<std::recursive_mutex> locked(_lock);
    _opts.timeout(timeout);
}

//--------------------------------------------------------------------------------
void KinectV2::close(){
    if( bOpened ){
        stopThread();
        protonect.closeKinect();
        bOpened = false;
    }
}


