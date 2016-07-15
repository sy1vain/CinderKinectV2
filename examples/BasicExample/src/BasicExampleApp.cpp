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
    
    textureRGB.reset();
    textureIR.reset();
    textureDepth.reset();
    
    if(event.getChar()==' '){
        kinect = KinectV2::create();
        kinect->open();
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
    
    if(textureRGB) gl::draw(textureRGB);
    if(textureIR) gl::draw(textureIR);
    if(textureDepth) gl::draw(textureDepth);
    
    ci::gl::drawString(ci::toString(getAverageFps()), vec2(10,10));
}

CINDER_APP( BasicExampleApp, RendererGl )
