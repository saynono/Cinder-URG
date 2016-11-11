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
        mFrames->cancel();
        console() << "Ended the urg." << std::endl;
    }
    
    //--------------------------------------------------------------
    
    bool UrgController::isFound() {
        return state == URG_FOUND;
    }
    
    //--------------------------------------------------------------
    
    bool UrgController::init() {
        
        
        if( checkError(urg_connect(&urg, mUrgPort.c_str(), 115200))){
            CI_LOG_E( "Error Urg: Could not open " << mUrgPort );
            return false;
        }
        mFrames = new ConcurrentCircularBuffer<URG_FrameRef>( 2 );
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
        
        //urg_(&urg, &minDistance, &maxDistance);
        console() << "Notice Urg: Data Size: " << dataSize;
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
                        frameCurrent->points[i].degrees = urg_index2deg(&urg, i*urg.skip_lines_);
                        frameCurrent->points[i].radians = urg_index2rad(&urg, i*urg.skip_lines_);
                        frameCurrent->points[i].distance = dataRaw[i];
//                        if( dataRaw[i] < 10 ) console() << i << "/" << numSteps << " deg: " << frameCurrent->points[i].degrees << "    d : " << dataRaw[i] << std::endl;
                    }
                    mFrames->popBack(&mFrameCurrent);
                    onData();
            }
            cinder::sleep(10);
        }
    }
    
    //--------------------------------------------------------------
	
    void UrgController::start(){
        
        if(!bThreadRunning){
            console() << "URG::start thread " << std::endl;
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
    
} // namespace nono
