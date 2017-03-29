//
//  UrgHelper.h
//  GMP_Lidar_Setup
//
//  Created by say nono on 02.12.16.
//
//

#pragma once

#include "cinder/app/App.h"

namespace nono {

    namespace utils {

    static bool intersectionLineLine(vec2 p1a, vec2 p1b, vec2 p2a, vec2 p2b, vec2 *pRes){
        double xD1,yD1,xD2,yD2,xD3,yD3;
        double dot,deg,len1,len2;
        double segmentLen1,segmentLen2;
        double ua,ub,div;
        
        // calculate differences
        xD1=p1b.x-p1a.x;
        xD2=p2b.x-p2a.x;
        yD1=p1b.y-p1a.y;
        yD2=p2b.y-p2a.y;
        xD3=p1a.x-p2a.x;
        yD3=p1a.y-p2a.y;
        
        // calculate the lengths of the two lines
        len1=sqrt(xD1*xD1+yD1*yD1);
        len2=sqrt(xD2*xD2+yD2*yD2);
        
        // calculate angle between the two lines.
        dot=(xD1*xD2+yD1*yD2); // dot product
        deg=dot/(len1*len2);
        
        // if abs(angle)==1 then the lines are parallell,
        // so no intersection is possible
        if(fabs(deg)==1) return false;
        
        // find intersection Pt between two lines
        div=yD2*xD1-xD2*yD1;
        if(div == 0) return false;
        
        ua=(xD2*yD3-yD2*xD3)/div;
        ub=(xD1*yD3-yD1*xD3)/div;
        
        vec2 pt;
        pt.x=p1a.x+ua*xD1;
        pt.y=p1a.y+ua*yD1;
        
        // calculate the combined length of the two segments
        // between Pt-p1 and Pt-p2
        xD1=pt.x-p1a.x;
        xD2=pt.x-p1b.x;
        yD1=pt.y-p1a.y;
        yD2=pt.y-p1b.y;
        segmentLen1=sqrt(xD1*xD1+yD1*yD1)+sqrt(xD2*xD2+yD2*yD2);
        
        // calculate the combined length of the two segments
        // between Pt-p3 and Pt-p4
        xD1=pt.x-p2a.x;
        xD2=pt.x-p2b.x;
        yD1=pt.y-p2a.y;
        yD2=pt.y-p2b.y;
        segmentLen2=sqrt(xD1*xD1+yD1*yD1)+sqrt(xD2*xD2+yD2*yD2);
        
        // if the lengths of both sets of segments are the same as
        // the lenghts of the two lines the point is actually
        // on the line segment.
        
        // if the point isn’t on the line, return null
        double abs1 = fabs(len1-segmentLen1);
        double abs2 = fabs(len2-segmentLen2);
        if(abs1>0.01 || abs2>0.01)
            return false;  
        
        // return the valid intersection  
        pRes->x = pt.x;
        pRes->y = pt.y;
        
        return true;  
    }
    
}} // namespace nono::utils