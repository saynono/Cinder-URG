//
//  UrgEvents.cpp
//  GMP_Lidar_Setup
//
//  Created by say nono on 03.12.16.
//  Copyright (c) 2016 nono. All rights reserved.
//

#include "UrgEvents.h"


extern "C"{
#include "urg_ctrl.h"
}


namespace nono {

bool sortLinePoints(float p1, float p2) {
    return (p1 < p2);
}

    
UrgEvents::UrgEvents(){
}
    
UrgEvents::~UrgEvents(){
}
    
void UrgEvents::setup( UrgControllerRef controller ){
    
    controller->onData.connect( bind(&UrgEvents::onData, this, std::placeholders::_1) );
    
    mUrg = controller->getUrgRef();

    mProperties = controller->getProperties();
//    getWindow()->getSignalUp().connect( std::bind( &Gui::onWindowResize, this) );
    bHasNewData = false;
}
	
void UrgEvents::onData( UrgController::URG_FrameRef frame ){
    int minDist = 30; // 10 cm
    for( auto zone: mZones ){
        UrgZoneState z = BEHIND;
        for( auto line: zone.second->lines ){
            zone.second->intersection[line.first] = 0;
            if( line.second.size() > 0 ){
                float d = frame->points[line.first].distance;
                if( d > minDist ){
                    if( d < line.second.front() ){
                        z = INFRONT;
//                        console() << " INFRONT: " << line.first << "   " << d << "  front: " << line.second.front() << std::endl;
                    }else if(d > line.second.back() ){
                        z = min(BEHIND,z);
                    }else{
                        zone.second->intersection[line.first] = frame->points[line.first].distance;
                        z = min(INSIDE,z);
                    }
                }
            }
        }
        
        if( z != zone.second->currentState ){
//            console() << zone.first << "Changed " << z << std::endl;
            UrgZoneState zOld = zone.second->currentState;
            zone.second->currentState = z;
            onTrigger.emit( zone.second->id, z, zOld );
        }
    }    
}
    
void UrgEvents::update(){
}
    
int UrgEvents::setZone( const vector<vec2>& points, int id ){
    lock_guard<mutex> lock(mMutex);
    ZoneRef zone = std::make_shared<Zone>();
    zone->id = id;//mZones.size();
    zone->currentState = BEHIND;
    for( int i=0;i<points.size();i++ ){
        zone->points.push_back( points[i] );
    }
    mZones[zone->id] = zone;
    recalculateZones();
    return zone->id;
}
    
void UrgEvents::recalculateZones(){
    
    for( auto zone: mZones ){
        
        zone.second->lines.clear();
        
        float minAngle = atan2(zone.second->points[0].x,zone.second->points[0].y);
        float maxAngle = atan2(zone.second->points[0].x,zone.second->points[0].y);
        for( int i=1;i<zone.second->points.size();i++ ){
            float a = atan2(zone.second->points[i].x,zone.second->points[i].y);
            minAngle = min(a,minAngle);
            maxAngle = max(a,maxAngle);
        }
        minAngle -= M_PI_2;
        maxAngle -= M_PI_2;
        int minIndex = urg_rad2index(mUrg, minAngle);
        int maxIndex = urg_rad2index(mUrg, maxAngle);
        
        auto points = zone.second->points;
        for( int index=minIndex;index<=maxIndex;index++){
            float rad = urg_index2rad( mUrg, index);
            double x = cosf(rad) * 100000.0f;
            double y = sinf(rad) * 100000.0f;
            vec2 lineVector(x,-y);
            int cntIntersects = 0;
            for( int i=0;i<points.size();i++ ){
                vec2 posIntersect;
                bool intesects = nono::utils::intersectionLineLine( points[i], points[(i+1)%points.size()], vec2(), lineVector, &posIntersect );
                if( intesects ){
                    zone.second->lines[index].push_back( length(posIntersect) );
                }
            }
            if(zone.second->lines.count(index)){
                std::sort(zone.second->lines[index].begin(), zone.second->lines[index].end(), sortLinePoints);
            }
        }
    }
}
    
    


} //namespace