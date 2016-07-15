#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"

#include "cinder/gl/Texture.h"
#include "cinder/Utilities.h"

#include "KinectV2.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class BasicExampleApp : public App {
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
    void keyUp( KeyEvent event ) override;
	void update() override;
	void draw() override;
    
    KinectV2Ref kinect;
    
    gl::TextureRef textureRGB;
    gl::TextureRef textureIR;
    gl::TextureRef textureDepth;
    
};

void BasicExampleApp::setup()
{
}

void BasicExampleApp::mouseDown( MouseEvent event )
{
}

void BasicExampleApp::keyUp(KeyEvent event){
    
    if(event.getChar()==' '){
        kinect = KinectV2::create();
        kinect->open();
    }
    
    if(!kinect) return;
    
    if(event.getChar()=='c'){
        textureRGB = textureIR = textureDepth = gl::TextureRef();
        kinect->colorEnabled(!kinect->colorEnabled())->open();
    }
    
    if(event.getChar()=='d'){
        textureRGB = textureIR = textureDepth = gl::TextureRef();
        kinect->depthEnabled(!kinect->depthEnabled())->open();
    }
    
    if(event.getChar()=='i'){
        textureRGB = textureIR = textureDepth = gl::TextureRef();
        kinect->irEnabled(!kinect->irEnabled())->open();
    }
    
    if(event.getChar()=='-'){
        kinect->minDistance(kinect->minDistance()-100);
    }
    if(event.getChar()=='='){
        kinect->minDistance(kinect->minDistance()+100);
    }
    
    if(event.getChar()=='_'){
        kinect->maxDistance(kinect->maxDistance()-100);
    }
    if(event.getChar()=='+'){
        kinect->maxDistance(kinect->maxDistance()+100);
    }
}

void BasicExampleApp::update()
{
    if(!kinect) return;
    if(kinect->checkRGBFrameNew()){
        textureRGB = gl::Texture::create(*kinect->getSurfaceRGB());
    }
    
    if(kinect->checkIRFrameNew()){
        textureIR = gl::Texture::create(*kinect->getChannelIR());
    }
    
    if(kinect->checkDepthFrameNew()){
        textureDepth = gl::Texture::create(*kinect->getChannelDepth());
    }
}

void BasicExampleApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
    
    ci::gl::ScopedMatrices scpMat;
    if(textureRGB){
        gl::draw(textureRGB, ci::Rectf(0,0,1024,576));
        gl::translate(0,576);
    }
    if(textureIR){
        gl::draw(textureIR);
        gl::translate(512,0);
    }
    if(textureDepth) gl::draw(textureDepth);
    
    ci::gl::drawString(ci::toString(getAverageFps()), vec2(10,10));
}

CINDER_APP( BasicExampleApp, RendererGl, [](BasicExampleApp::Settings *settings){
    settings->setWindowSize(1024, 1000);
});
