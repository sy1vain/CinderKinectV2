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

KinectV2Ref KinectV2::create(){
    return KinectV2Ref(new KinectV2());
}

KinectV2::KinectV2::KinectV2() : _opened(false), _opening(false), _closing(false), _distMin(500), _distMax(6000), _color(true), _ir(true), _depth(true), _timeout(3) {
    CI_LOG_I("Creating KinectV2");
}

KinectV2::~KinectV2(){
    close(true);
    CI_LOG_I("Destroyed KinectV2");
}

std::vector<KinectV2::KinectDeviceInfo> KinectV2::getDeviceList(){
    std::vector <KinectDeviceInfo> devices;
    
    int num = getFreenect()->enumerateDevices();
    
    for (int i=0; i<num; i++){
        KinectDeviceInfo kdi;
        kdi.serial = getFreenect()->getDeviceSerialNumber(i);
        kdi.freenectId = i;
        devices.push_back(kdi);
    }
    
    std::sort(devices.begin(), devices.end(), [](const KinectDeviceInfo& A, const KinectDeviceInfo& B) -> bool {
        return A.serial < B.serial;
    });
    
    for(int i=0; i<num; i++){
        devices[i].deviceId = i;
    }
    
    return devices;
}

void KinectV2::open(KinectV2::KinectDeviceInfo kinect){
    close(true);
    open(kinect.serial);
}

void KinectV2::open(unsigned int deviceId){
    close(true);
    std::vector <KinectDeviceInfo> devices = getDeviceList();
    
    if(devices.empty()){
        CI_LOG_E("no devices connected");
        return;
    }
    
    if(deviceId >= devices.size()){
        CI_LOG_E(" deviceId " << deviceId << " is bigger or equal to the number of connected devices " << devices.size());
        return;
    }
    
    std::string serial = devices[deviceId].serial;
    open(serial);
}

void KinectV2::open(const std::string& serial){
    close(true);
    CI_LOG_I("Open device " << serial);
    
    {
        std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
        _surfaceRGB.reset();
        _channelIR.reset();
        _channelDepth.reset();
        _opening = true;
    }
    
    CI_LOG_I("Open device " << serial);
    
    startThread(serial);
}

void KinectV2::close(bool wait){
    if(isClosing()) return;
    if(!isOpen() && !isOpening()) return;
    
    CI_LOG_D("Closing device");
    
    {
        std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
        _closing = true;
    }
    
    if(wait) joinThread();
}

bool KinectV2::isBusy(){
    return isOpen() || isOpening() || isClosing();
}

bool KinectV2::isOpen(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    return _opened;
}

bool KinectV2::isOpening(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    return _opening;
}

bool KinectV2::isClosing(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    return _closing;
}

bool KinectV2::checkRGBFrameNew(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    return _newRGBFrame;
}
bool KinectV2::checkIRFrameNew(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    return _newIRFrame;
}
bool KinectV2::checkDepthFrameNew(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    return _newDepthFrame;
}

ci::Surface8uRef KinectV2::getSurfaceRGB(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    _newRGBFrame = false;
    return _surfaceRGB;
}

ci::Channel32fRef KinectV2::getChannelIR(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    _newIRFrame = false;
    return _channelIR;
}

ci::Channel32fRef KinectV2::getChannelDepth(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    _newDepthFrame = false;
    return _channelDepth;
}

int KinectV2::minDistance(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    return _distMin;
}

KinectV2Ref KinectV2::minDistance(int dist){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    _distMin = dist;
    return shared_from_this();
}

int KinectV2::maxDistance(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    return _distMax;
}

KinectV2Ref KinectV2::maxDistance(int dist){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    _distMax = dist;
    return shared_from_this();
}

bool KinectV2::colorEnabled(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    return _color;
}

KinectV2Ref KinectV2::colorEnabled(bool enabled){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    _color = enabled;
    return shared_from_this();
}

bool KinectV2::irEnabled(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    return _ir;
}

KinectV2Ref KinectV2::irEnabled(bool enabled){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    _ir = enabled;
    return shared_from_this();
}

bool KinectV2::depthEnabled(){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    return _depth;
}

KinectV2Ref KinectV2::depthEnabled(bool enabled){
    std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
    _depth = enabled;
    return shared_from_this();
}


void KinectV2::joinThread(){
    if(_thread && _thread->joinable()){
        _thread->join();
    }
    _thread.reset();
}

void KinectV2::startThread(const std::string serial){
    joinThread();
    
    std::lock_guard<std::recursive_mutex> locked(_recursive_mutex);
    _thread = std::shared_ptr<std::thread>(new std::thread(std::bind(&KinectV2::runDevice, this, serial)));
}

void KinectV2::runDevice(const std::string serial){
    ci::ThreadSetup _threadSetup;
    
    while(isOpening()){
        
        libfreenect2::PacketPipeline *pipeline = new libfreenect2::OpenCLPacketPipeline();
        libfreenect2::Freenect2Device *dev = getFreenect()->openDevice(serial, pipeline);
        
        if(dev==0){
            std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
            _opening = false;
            _opened = false;
            _closing = false;
            return;
        }
        
        
        {
            std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
            _opened = true;
        }
        
        unsigned int frame_types = 0;
        if(colorEnabled()) frame_types |= libfreenect2::Frame::Color;
        if(depthEnabled()) frame_types |= libfreenect2::Frame::Depth;
        if(irEnabled()) frame_types |= libfreenect2::Frame::Ir;
        
        libfreenect2::SyncMultiFrameListener* listener = new libfreenect2::SyncMultiFrameListener(frame_types);
        libfreenect2::FrameMap frames;
        
        bool enable_rgb = colorEnabled();
        bool enable_depth = depthEnabled() || irEnabled();
        
        if(enable_rgb) dev->setColorFrameListener(listener);
        if(enable_depth) dev->setIrAndDepthFrameListener(listener);
        
        dev->startStreams(enable_rgb, enable_depth);
        {
            std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
            _opening = false;
        }
        
        _timeoutTimer.start();
    
        while(!isClosing()){
            if(listener->hasNewFrame()){
                _timeoutTimer.start();
                listener->waitForNewFrame(frames);
                
                handleRGBFrame(frames[libfreenect2::Frame::Color]);
                handleIRFrame(frames[libfreenect2::Frame::Ir]);
                handleDepthFrame(frames[libfreenect2::Frame::Depth]);
                
                listener->release(frames);
            }else{
                if(_timeoutTimer.getSeconds()>=_timeout) close(false);
                
                ci::sleep(10);
            }
        }
        
        {
            std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
            _opened = false;
            _newRGBFrame = _newIRFrame = _newDepthFrame = false;
            _surfaceRGB.reset();
            _channelIR.reset();
            _channelDepth.reset();
        }
        
        //cleanup
        listener->release(frames);
        
        dev->stop();
        dev->close();
        delete listener;
        delete dev;
        dev = 0;
        
        {
            std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
            _closing = false;
        }
    }
}

void KinectV2::handleRGBFrame(libfreenect2::Frame *frame){
    if(!frame) return;
    ci::SurfaceChannelOrder channelOrder = (frame->format==libfreenect2::Frame::Format::RGBX)?ci::SurfaceChannelOrder::RGBX:ci::SurfaceChannelOrder::BGRX;
    ci::Surface8uRef colorSurface = ci::Surface8u::create(frame->width, frame->height, true);
    colorSurface->setChannelOrder(channelOrder);
    memcpy(colorSurface->getData(), frame->data, frame->width*frame->height*4 );
    {
        std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
        _surfaceRGB = colorSurface;
        _newRGBFrame = true;
    }
}

void KinectV2::handleIRFrame(libfreenect2::Frame *frame){
    if(!frame) return;
    ci::Channel32fRef irChannel = ci::Channel32f::create(frame->width, frame->height);
    memcpy(irChannel->getData(), reinterpret_cast<float*>(frame->data), frame->width*frame->height*4);
    
    float* data = irChannel->getData();
    for(int i=0; i<irChannel->getWidth() * irChannel->getHeight(); i++){
        data[i] /= 65535;
    }
    
    {
        std::lock_guard<std::recursive_mutex> lock(_recursive_mutex);
        _channelIR = irChannel;
        _newIRFrame = true;
    }
}

void KinectV2::handleDepthFrame(libfreenect2::Frame *frame){
    if(!frame) return;
    int minDist = minDistance();
    int maxDist = maxDistance();
    ci::Channel32fRef depthChannel = ci::Channel32f::create(frame->width, frame->height);
    memcpy(depthChannel->getData(), reinterpret_cast<float*>(frame->data), frame->width*frame->height*4);
    
    float* data = depthChannel->getData();
    for(int i=0; i<depthChannel->getWidth() * depthChannel->getHeight(); i++){
        data[i] = glm::clamp(ci::lmap<float>(data[i], minDist, maxDist, 0.f, 1.f), 0.f, 1.f);
    }
    
    {
        std::lock_guard<std::recursive_mutex> locked(_recursive_mutex);
        _channelDepth = depthChannel;
        _newDepthFrame = true;
    }
}
