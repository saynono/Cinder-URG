//
//  UrgEvents.h
//  GMP_Lidar_Setup
//
//  Created by say nono on 03.12.16.
//  Copyright (c) 2016 nono. All rights reserved.
//

#pragma once


#include "cinder/app/App.h"
#include "UrgController.h"
#include "UrgUtils.h"

using namespace ci;
using namespace ci::app;
using namespace std;

namespace nono {

enum UrgZoneState{
    INFRONT = 0,
    INSIDE = 1,
    BEHIND = 2
};

class UrgEvents{

public:
    
    

    UrgEvents();
    ~UrgEvents();
    void setup( UrgControllerRef controller );
	void onData( UrgController::URG_FrameRef );
    void update();
    int setZone( const vector<vec2>& points, int id );
    
//private:
    
    struct Zone{
        int id;
        vector<vec2> points;
        UrgZoneState currentState;
        map< int, vector<long> > lines;
        map< int, long > intersection;
    };

    
    using ZoneRef = std::shared_ptr<Zone>;
    
    urg_t*                                      mUrg;
    map< int, ZoneRef>                          mZones;
    UrgController::URG_Properties               mProperties;
    bool                                        bHasNewData;
    mutable std::mutex                          mMutex;

    
    signals::Signal<void( int &, UrgZoneState &, UrgZoneState & ) > onTrigger;
    
    
//    boost::signals2::signal<void(int&)> onZoneTrigger;

    
    void recalculateZones();
    
};

} // namespace nono

