#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"

#include "cinder/gl/Texture.h"

#include "KinectV2.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class BasicExampleApp : public App {
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;
    
    KinectV2Ref kinect;
    
    gl::TextureRef textureRGB;
    gl::TextureRef textureIR;
    gl::TextureRef textureDepth;
    
};

void BasicExampleApp::setup()
{
    kinect = KinectV2::create(0, KinectV2::Options().minDistance(0).maxDistance(2000));
    ci::gl::enableAlphaBlending();
    
//    auto devices = kinect->getDeviceList();
    
}

void BasicExampleApp::mouseDown( MouseEvent event )
{
}

void BasicExampleApp::update()
{
    
    auto rgb = kinect->getSurfaceRGB();
    if(rgb) textureRGB = gl::Texture::create(*rgb);
    
    auto ir = kinect->getChannelIR();
    if(ir) textureIR = gl::Texture::create(*ir);
    
    auto depth = kinect->getChannelDepth();
    if(depth) textureIR = gl::Texture::create(*depth);
    
}

void BasicExampleApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
    
    if(textureRGB) gl::draw(textureRGB);
    if(textureIR) gl::draw(textureIR);
    if(textureDepth) gl::draw(textureDepth);
}

CINDER_APP( BasicExampleApp, RendererGl )
