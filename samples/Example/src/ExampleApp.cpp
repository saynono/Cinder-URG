#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/Text.h"
#include "cinder/Serial.h"
#include "cinder/Log.h"

#include "UrgController.h"


using namespace ci;
using namespace ci::app;
using namespace std;

class ExampleApp : public App {
  public:
	void setup();
    void onData();
	void update();
	void draw();
	
    nono::UrgController               mUrgController;
    vector<gl::TextureRef>            mTextures;

};

void ExampleApp::setup()
{
    
    // print the devices
    for( const auto &dev : Serial::getDevices() )
        console() << "Device: " << dev.getName() << "       " << dev.getPath() << " (Error with Path on OSX?) " << endl;
    
    try {
        Serial::Device dev = Serial::findDeviceByNameContains( "tty.usbmodem" );
        mUrgController.setup( "/dev/"+dev.getPath() );
    }
    catch( SerialExc &exc ) {
        CI_LOG_EXCEPTION( "coult not initialize the serial device", exc );
        exit( -1 );
    }
    
    mUrgController.onData.connect( boost::bind(&ExampleApp::onData, this) );
    
    
    Font font = Font( "OpenSans", 28 );
    int radius = 100;
    while( radius < 10000 ){
        TextLayout simple;
        simple.setFont( font );
        simple.setColor( Color( 1, 0, 0 ) );
        simple.addLine( toString(radius/10) + "cm" );
        gl::TextureRef t = gl::Texture::create( simple.render( true, true ) );
        mTextures.push_back(t);
        radius *= 2;
    }
    
}

void ExampleApp::onData(){
    nono::UrgController::URG_FrameRef frame = mUrgController.getCurrentFrame();
    console() << "onData! => " << frame->timestamp << std::endl;
}

void ExampleApp::update()
{	
}

void ExampleApp::draw()
{
	
    gl::enableAlphaBlending();
	gl::clear( Color( 0, 0, 0 ) );

    auto data = mUrgController.getCurrentFrame();
    
    int w = getWindowWidth();
    int h = getWindowHeight();
    
    gl::pushMatrices();
    gl::translate( vec2(w/2,h/2) );
    
    float s = .25;
    gl::scale( vec3(s,s,1) );
    gl::color( Color( 1,1,1 ) );
    vec2 pCenter(0,0);
    for(auto& d: data->points){
        float x = cosf(d.radians) * d.distance;
        float y = sinf(d.radians) * d.distance;
        gl::drawLine( pCenter, vec2(x,y) );
    }
    
    int radius = 100;
    int cnt = 0;
    while( radius < 10000 ){
        radius *= 2;
        gl::color( Color( 1,0, 0 ) );
        gl::drawStrokedCircle(pCenter, radius);
        gl::color( Color( 1,1,1 ) );
        gl::draw( mTextures[cnt++] , vec2( -radius+10, -10 ) );
        
    }
    
    
    gl::popMatrices();
}

CINDER_APP( ExampleApp, RendererGl (RendererGl::Options().stencil().msaa (16)),
           [&] (App::Settings * settings)
{
    settings->setWindowSize (800, 800);
    settings->setFrameRate (60.0f);
    settings->setTitle ("Cinder Urg Wrapper Basic");
    settings->setHighDensityDisplayEnabled();
})