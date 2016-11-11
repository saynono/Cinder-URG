//
//  UrgController.h
//
//  Created by say nono on 10.11.16
//  Copyright (c) 2016 nono. All rights reserved.
//

#pragma once


#include "cinder/app/App.h"
#include "cinder/app/Event.h"
#include "cinder/Thread.h"
#include "cinder/utilities.h"
#include "cinder/ConcurrentCircularBuffer.h"
#include "cinder/Log.h"

#include <boost/signals2.hpp>

extern "C"{
#include "urg_ctrl.h"
}

using namespace ci;
using namespace ci::app;
using namespace std;


namespace nono {
    
    class UrgController: public Event {

public:
      
    struct URG_Properties{
        long distanceMin;
        long distanceMax;
    };

    struct URG_Point{
//        public:
//        
//        vec2 getPosition(){
//            float rad = toRadians(degrees);
//            return vec2(cos(rad)*distance, sin(rad)*distance);
//        }
//            
//        void setPosition(vec2 p){
//            degrees = toDegrees(atan2f(p.y, p.x));
//            distance = long( length(p) );
//        }
        
        double degrees;
        double radians;
        long distance;
    };
        
    struct URG_Frame{
        URG_Properties properties;
        long timestamp;
        vector<URG_Point> points;
    };
        
        
    using URG_FrameRef = shared_ptr<URG_Frame>;
    
	
    UrgController();
    ~UrgController();
    
    void setup( string port );

    void kill();
    void start();
    void stop();
        
    bool isFound();
        
    URG_FrameRef getCurrentFrame();
    
    boost::signals2::signal<void(void)> onData;

    
private:
        
    bool init();
    void getSensorProperties();
    void threadedFunction();
        
    bool checkError( int ret );
        
    // check if the device has shutdown (weird bug in etherdream driver) and reconnect if nessecary
    bool checkConnection(bool bForceReconnect = true);
        
    enum {
        URG_NOTFOUND = 0,
        URG_FOUND
    } state;
//
//    int pps;
//    bool bWaitBeforeSend;
    bool bThreadRunning;
    string mUrgPort;
        
        
    URG_Properties mProperties;
        
    ConcurrentCircularBuffer< URG_FrameRef >	*mFrames;
    URG_FrameRef mFrameCurrent;
        
	mutable std::mutex mMutex;
	std::shared_ptr<std::thread> mThread;
    std::vector<long> dataRaw;
    std::vector<URG_Point> dataIncoming;
        
    urg_t urg;
        
    
};

} // namespace nono



