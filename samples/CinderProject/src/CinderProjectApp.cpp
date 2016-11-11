#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/Serial.h"

#include "UrgController.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class CinderProjectApp : public App {
  public:

	void setup();
	void onData();
	void update();
	void draw();
	
    nono::UrgController               mUrgController;

};

void CinderProjectApp::setup()
{

    try {
        Serial::Device dev = Serial::findDeviceByNameContains( "tty.usbmodem" );
        mUrgController.setup( "/dev/"+dev.getPath() );
    }
    catch( SerialExc &exc ) {
        CI_LOG_EXCEPTION( "coult not initialize the serial device", exc );
        exit( -1 );
    }
    
    mUrgController.onData.connect( boost::bind(&CinderProjectApp::onData, this) );
}

void CinderProjectApp::onData(){
    nono::UrgController::URG_FrameRef frame = mUrgController.getCurrentFrame();
    console() << "onData! => " << frame->timestamp << std::endl;
}

void CinderProjectApp::update()
{	
}

void CinderProjectApp::draw()
{	
    gl::clear( Color( 0, 0, 0 ) );
    gl::color( Color( 1,1,1 ) );
    int cnt = 10;
    auto data = mUrgController.getCurrentFrame();
    for(auto& d: data->points){
        gl::drawLine( vec2(10,cnt), vec2(10+d.distance,cnt++) );
    }
}

CINDER_APP( CinderProjectApp, RendererGl (RendererGl::Options().stencil().msaa (16)),
           [&] (App::Settings * settings)
{
    settings->setWindowSize (800, 800);
    settings->setFrameRate (60.0f);
    settings->setTitle ("Cinder URG Wrapper Basic");
    settings->setHighDensityDisplayEnabled();
})
