/***********************************************************************
vehicle.h - vehicle class (fish & rabbits moving in the sandbox)
Copyright (c) 2016 Thomas Wolf

This file is part of the Magic Sand.

The Magic Sand is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Magic Sand is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Augmented Reality Sandbox; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#pragma once
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

#include "ZedProjector/ZedProjector.h"

class Vehicle{

public:
    Vehicle(std::shared_ptr<ZedProjector> const& k, ofGlmPoint slocation, ofRectangle sborders, bool sliveInWater, glm::vec2 motherLocation);
    
    // Virtual functions
    virtual void setup() = 0;
    virtual void applyBehaviours(bool seekMother) = 0;
    virtual void draw() = 0;
    
    void update();
    
    std::vector<glm::vec2> getForces(void);
    
    const ofGlmPoint& getLocation() const {
        return location;
    }
    const ofGlmPoint& getVelocity() const {
        return velocity;
    }
    
    const float getAngle() const {
        return angle;
    }
    
    const bool foundMother() const {
        return mother;
    }
    
    void setMotherLocation(glm::vec2 loc){
        motherLocation = loc;
    }
    
protected:
    void updateBeachDetection();
    ofGlmPoint seekEffect();
    ofGlmPoint bordersEffect();
    ofGlmPoint slopesEffect();
    virtual ofGlmPoint wanderEffect();
    void applyVelocityChange(const ofGlmPoint & force);
    
    std::shared_ptr<ZedProjector> zedProjector;

    ofGlmPoint location;
    ofGlmPoint velocity;
    ofGlmPoint globalVelocityChange;
    glm::vec2 currentForce;
    float angle; // direction of the drawing
    
    glm::vec2 separateF ;
    glm::vec2 seekF ;
    glm::vec2 bordersF ;
    glm::vec2 slopesF ;
    glm::vec2 wanderF ;

    bool beach;
    bool border;
    
    bool mother;
    glm::vec2 motherLocation;
    
    // For slope effect
    float beachDist;
    glm::vec2 beachSlope;
    
    bool liveInWater; // true for fish who want to stay in the water, false for rabbits who want to stay on the ground
    
    glm::vec2 projectorCoord;
    ofRectangle borders, internalBorders;
    float maxVelocityChange;
    float maxRotation;
    int r, minborderDist, desiredseparation, cor;
    
    float wanderR ;         // Radius for our "wander circle"
    float wanderD ;         // Distance for our "wander circle"
    float change ;
    float wandertheta;
    float topSpeed;
};

class Fish : public Vehicle {
public:
    Fish(std::shared_ptr<ZedProjector> const& k, ofGlmPoint slocation, ofRectangle sborders, glm::vec2 motherLocation) : Vehicle(k, slocation, sborders, true, motherLocation){}

    void setup();
    void applyBehaviours(bool seekMother);
    void draw();
    
private:
    ofGlmPoint wanderEffect();
};

class Rabbit : public Vehicle {
public:
    Rabbit(std::shared_ptr<ZedProjector> const& k, ofGlmPoint slocation, ofRectangle sborders, glm::vec2 motherLocation) : Vehicle(k, slocation, sborders, false, motherLocation){}
    
    void setup();
    void applyBehaviours(bool seekMother);
    void draw();

private:
    ofGlmPoint wanderEffect();

    int maxStraightPath; // max rabbit straight path length
    int currentStraightPathLength;// current rabbit straight path length
    
    float velocityIncreaseStep; // Rabbit increase step
    float minVelocity;
    bool setWait;
    int waitCounter;
    int waitTime;
    int maxWaitingTime;
    int minWaitingTime;
};

