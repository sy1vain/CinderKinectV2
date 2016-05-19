//  ofProtonect.cpp
//
//  Modified by Theodore Watson on 11/16/15
//  from the Protonect example in https://github.com/OpenKinect/libfreenect2


/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <iostream>
#include <signal.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include "cinder/Log.h"
#include "Protonect.h"

Protonect::Protonect(){
    bOpened = false;
    
//
//    if( ofGetLogLevel() == OF_LOG_VERBOSE ){
//        libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
//    }else{
//        libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
//    }
}

int Protonect::openKinect(std::string serial){
          
//      pipeline = new libfreenect2::CpuPacketPipeline();
//        pipeline = new libfreenect2::OpenGLPacketPipeline();
        pipeline = new libfreenect2::OpenCLPacketPipeline();

      if(pipeline)
      {
        dev = freenect2.openDevice(serial, pipeline);
      }

      if(dev == 0)
      {
          CI_LOG_E("failure opening device with serial " << serial);
          return -1;
      }

    
      listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
      undistorted = new libfreenect2::Frame(512, 424, 4);
      registered  = new libfreenect2::Frame(512, 424, 4);

      dev->setColorFrameListener(listener);
      dev->setIrAndDepthFrameListener(listener);
      dev->start();

    CI_LOG_V("device serial: " << dev->getSerialNumber());
    CI_LOG_V("device firmware: " << dev->getFirmwareVersion());

      registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

    bOpened = true;
    
    return 0;
}

Protonect::Frame Protonect::updateKinect(int minDist, int maxDist){
  
    Frame frame;
    
    if(bOpened){
        //collect frames
        listener->waitForNewFrame(frames);
        
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        
        //get RGB
        ci::SurfaceChannelOrder channelOrder = (rgb->format==libfreenect2::Frame::Format::RGBX)?ci::SurfaceChannelOrder::RGBX:ci::SurfaceChannelOrder::BGRX;
        ci::Surface8uRef colorSurface = ci::Surface8u::create(rgb->width, rgb->height, true);
        colorSurface->setChannelOrder(channelOrder);
        memcpy(colorSurface->getData(), rgb->data, rgb->width*rgb->height*4 );
        frame.rgb = colorSurface;
        
        //get IR
        ci::Channel32fRef irChannel = ci::Channel32f::create(ir->width, ir->height);
        memcpy(irChannel->getData(), reinterpret_cast<float*>(ir->data), ir->width*ir->height*4);
        
        float* data = irChannel->getData();
        for(int i=0; i<irChannel->getWidth() * irChannel->getHeight(); i++){
            data[i] /= 65535;
        }
        
        frame.ir = irChannel;
        
        
        //get DEPTH
        ci::Channel32fRef depthChannel = ci::Channel32f::create(depth->width, depth->height);
        memcpy(depthChannel->getData(), reinterpret_cast<float*>(depth->data), depth->width*depth->height*4);
        
        data = depthChannel->getData();
        for(int i=0; i<depthChannel->getWidth() * depthChannel->getHeight(); i++){
            data[i] = glm::clamp(ci::lmap<float>(data[i], minDist, maxDist, 0.f, 1.f), 0.f, 1.f);
        }
        
        frame.depth = depthChannel;
                
    
        listener->release(frames);
    }
    
    return frame;
}

int Protonect::closeKinect(){

  if(bOpened){
      listener->release(frames);

      // TODO: restarting ir stream doesn't work!
      // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
      dev->stop();
      dev->close();
      
      delete listener;
      listener = NULL;
      
      delete undistorted;
      undistorted = NULL;

      delete registered;
      registered = NULL;
      
      delete registration;
      bOpened = false; 
  }

  return 0;
}

