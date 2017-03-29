//
//  UrgController.h
//
//  Created by say nono on 10.11.16
//  Copyright (c) 2016 nono. All rights reserved.
//

#include "UrgController.h"

namespace nono {
    
    
    UrgController::UrgController():state(URG_NOTFOUND){
      bThreadRunning = false;
    }
    
    //--------------------------------------------------------------
    
    UrgController::~UrgController(){
        kill();
    }
    
    //--------------------------------------------------------------
    
    UrgControllerRef UrgController::create( string port ){
        UrgControllerRef ref = std::make_shared<UrgController>();
        ref->setup(port);
        return ref;
    }

    //--------------------------------------------------------------
    
    void UrgController::setup( string port ){
        
        mUrgPort = port;
        urg_initialize(&urg);
        
        init();
        start();
    
    }
    
    //--------------------------------------------------------------
    
    void UrgController::kill(){
        stop();
        urg_disconnect(&urg);
        bThreadRunning = false;
      
        if( mFrames != nullptr ) mFrames->cancel();
        if( mThread && mThread->joinable()){
            mThread->join();
            
        }
        console() << "Ended the urg." << std::endl;
    }
    
    //--------------------------------------------------------------
    
    bool UrgController::isFound() {
        return state == URG_FOUND;
    }
    
    //--------------------------------------------------------------
    
    bool UrgController::init() {
        
        mFrames = new ConcurrentCircularBuffer<URG_FrameRef>( 2 );
        if( checkError(urg_connect(&urg, mUrgPort.c_str(), 115200))){
            CI_LOG_E( "Error Urg: Could not open " << mUrgPort );
            return false;
        }
        getSensorProperties();
        
        lock_guard<mutex> lock(mMutex);
        state = URG_FOUND;
        mFrameCurrent = std::make_shared<URG_Frame>();
        mFrameCurrent->timestamp = 0;
        return true;
    }
    
    //--------------------------------------------------------------
    
    void UrgController::getSensorProperties(){
        int dataSize = urg_dataMax(&urg);
        if(dataSize > 0)
            dataRaw.resize(dataSize);
        else{
            CI_LOG_E( "Error URG: Max datasize wrong value: " << dataSize );
        }
        mProperties.distanceMax = urg_maxDistance(&urg);
        mProperties.distanceMin = urg_minDistance(&urg);
        
        urg_parameter_t parameters;
        urg_parameters(&urg, &parameters);
        mProperties.rpm = parameters.scan_rpm_;
        
        mProperties.areaMin = parameters.area_min_;
        mProperties.areaMax = parameters.area_max_;
        mProperties.areaTotal = parameters.area_total_;
        mProperties.areaFront = parameters.area_front_;        
    }
    
    //--------------------------------------------------------------
    void UrgController::threadedFunction() {
//        while (isThreadRunning() != 0) {
        checkError(urg_setCaptureTimes(&urg, 0));
        while(bThreadRunning){
            switch (state) {
                case URG_NOTFOUND:
                    cinder::sleep(1000);
                    init();
                    break;
                    
                case URG_FOUND:

                    checkError(urg_requestData(&urg, URG_GD, URG_FIRST, URG_LAST));
                    if(checkError(urg_receiveData(&urg, dataRaw.data(), dataRaw.size()))){
                        CI_LOG_E( "Error URG: Stopping Thread???" );
//                        stop();
                        continue;
                    }
                    
                    lock_guard<mutex> lock(mMutex);
                    size_t numSteps = size_t(floor(dataRaw.size() / float(urg.skip_lines_)));
                    dataIncoming.resize(numSteps);
                    
                    URG_FrameRef frameCurrent = std::make_shared<URG_Frame>();
                    frameCurrent->properties = mProperties;
                    frameCurrent->timestamp = ci::app::getElapsedSeconds()*1000;
                    frameCurrent->points.resize(numSteps);
                    mFrames->pushFront(frameCurrent);
                    
                    if(numSteps == 0){
                        continue;
                    }                    
                    for(size_t i=0; i<numSteps; i++){
//                        frameCurrent->points[i].degrees = urg_index2deg(&urg, i*urg.skip_lines_);
                        frameCurrent->points[i].index = i;
                        frameCurrent->points[i].radians = urg_index2rad(&urg, i*urg.skip_lines_);
                        frameCurrent->points[i].distance = dataRaw[i];
                    }
//                console() << " URG Frames :" << mFrames->getSize() << std::endl;
                    mFrames->popBack(&mFrameCurrent);
                    if( bThreadRunning ) onData.emit( mFrameCurrent );
            }
            cinder::sleep(10);
        }
    }
    
    //--------------------------------------------------------------
	
    void UrgController::start(){
        
        if(!bThreadRunning){
//            console() << "URG::start thread " << std::endl;
            bool err = checkError(urg_laserOn(&urg));
            if( err ) return;
            bThreadRunning = true;
            mThread = std::shared_ptr<std::thread>( new std::thread( &UrgController::threadedFunction, this ) );
        }
    }
    
    //--------------------------------------------------------------

    void UrgController::stop(){
        if(bThreadRunning){
            console() << "URG::stop thread " << std::endl;
            bThreadRunning = false;
            mThread->join();
            checkError(urg_laserOff(&urg));
        }
    }
    
    //--------------------------------------------------------------
    
    bool UrgController::checkError( int ret ){
        bool error = ret<0;
        if(error){
            CI_LOG_E("Error URG: " << urg_error(&urg) );
        }
        return error;
    }
    
    //--------------------------------------------------------------
    
    UrgController::URG_FrameRef UrgController::getCurrentFrame(){
        return mFrameCurrent;
    }
    
    //--------------------------------------------------------------
    
    UrgController::URG_Properties UrgController::getProperties(){
        return mProperties;
    }
    
    urg_t* UrgController::getUrgRef(){
        return &urg;
    }
    
    //--------------------------------------------------------------
    
} // namespace nono
