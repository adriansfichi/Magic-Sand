/***********************************************************************
vehicle.cpp - vehicle class (fish & rabbits moving in the sandbox)
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

#include "vehicle.h"

Vehicle::Vehicle(std::shared_ptr<ZedProjector> const& k, ofGlmPoint slocation, ofRectangle sborders, bool sliveInWater, glm::vec2 smotherLocation) {
    zedProjector = k;
    liveInWater = sliveInWater;
    location = slocation;
    borders = sborders;
    globalVelocityChange=glm::vec3(0, 0,0);
    velocity= glm::vec3(0.0, 0.0,0.0);
    angle = 0;
    wandertheta = 0;
    mother = false;
    motherLocation = smotherLocation;
}

void Vehicle::updateBeachDetection(){
    // Find sandbox gradients and elevations in the next 10 steps of vehicle v, update vehicle variables
    ofGlmPoint futureLocation;
    futureLocation = location;
    beachSlope = glm::vec2(0);
    beach = false;
    int i = 1;
    while (i < 10 && !beach)
    {
        bool overwater = zedProjector->elevationAtZedCoord(futureLocation.x, futureLocation.y) > 0;
        if ((overwater && liveInWater) || (!overwater && !liveInWater))
        {
            beach = true;
            beachDist = i;
            beachSlope = zedProjector->gradientAtZedCoord(futureLocation.x,futureLocation.y);
            if (liveInWater)
                beachSlope *= -1;
        }
        futureLocation += velocity; // Go to next future location step
        i++;
    }
}

ofGlmPoint Vehicle::bordersEffect(){
    ofGlmPoint desired, futureLocation;
    
    // Predict location 10 (arbitrary choice) frames ahead
    futureLocation = location + velocity*10;
    
    ofGlmPoint target = location;
    if (!internalBorders.inside(futureLocation)){ // Go to the opposite direction
        border = true;
        if (futureLocation.x < internalBorders.getLeft())
            target.x = borders.getRight();
        if (futureLocation.y < internalBorders.getTop())
            target.y = borders.getBottom();
        if (futureLocation.x > internalBorders.getRight())
            target.x = borders.getLeft();
        if (futureLocation.y > internalBorders.getBottom())
            target.y = borders.getTop();
    } else {
        border = false;
    }
    
    desired = target - location;
	desired= glm::normalize(desired);
    desired *= topSpeed;
    
    ofGlmPoint velocityChange(0);
    velocityChange = desired - velocity;

	glm::clamp(velocityChange,glm::vec3(0), glm::vec3(maxVelocityChange));
    return velocityChange;
}

ofGlmPoint Vehicle::wanderEffect(){
    
    ofGlmPoint velocityChange, desired;
    
    wandertheta += ofRandom(-change,change);     // Randomly change wander theta
    
    ofGlmPoint front = velocity;
	glm::normalize(front);
    front *= wanderD;
    ofGlmPoint circleloc = location + front;
    
	float h =glm::angle(front,glm::vec3(1,0,0)); // We need to know the heading to offset wandertheta
    
    ofGlmPoint circleOffSet = ofGlmPoint(wanderR*cos(wandertheta+h),wanderR*sin(wandertheta+h),0);
    ofGlmPoint target = circleloc + circleOffSet;
    
    desired = target - location;
    glm::normalize(desired);
    desired *= topSpeed;
    
    velocityChange = desired - velocity;
	glm::clamp(velocityChange, glm::vec3(0), glm::vec3(maxVelocityChange));
    
    return velocityChange;
}

ofGlmPoint Vehicle::slopesEffect(){
    ofGlmPoint desired, velocityChange;
    
    desired =ofGlmPoint(beachSlope,0);
    glm::normalize(desired);
    desired *= topSpeed;
    if(beach){
        desired /= beachDist; // The closest the beach is, the more we want to avoid it
    }
    velocityChange = desired - velocity;
	glm::clamp(velocityChange, glm::vec3(0), glm::vec3(maxVelocityChange));
    
    return velocityChange;
}

ofGlmPoint Vehicle::seekEffect(){
    ofGlmPoint desired;
    desired = ofGlmPoint(motherLocation - location,0);
    
    float d = desired.length();
    glm::normalize(desired);
    
    //If we are closer than XX pixels slow down
    if (d < 10) {
        desired *= ofMap(d,0,100,0,topSpeed);
        mother = true;
    } else {
        //Otherwise, proceed at maximum speed.
        desired *= topSpeed;
    }
    
    ofGlmPoint velocityChange;
    velocityChange = desired - velocity;
	glm::clamp(velocityChange, glm::vec3(0), glm::vec3(maxVelocityChange));
    
    //If we are further than XX pixels we don't see the mother
    if (d > 100) {
        velocityChange = ofGlmPoint(0);
    }
    
    return velocityChange;
}

//--------------------------------------------------------------
//ofGlmPoint Vehicle::separateEffect(vector<vehicle> vehicles){
////    float desiredseparation = r*2;
//    ofGlmPoint velocityChange;
//    int count = 0;
//    ofGlmPoint diff;
//    vector<vehicle>::iterator other;
//    for (other = vehicles.begin(); other < vehicles.end(); other++){
//        float d = (location - other->getLocation()).length();
//        if((d>0) && (d < desiredseparation)){
//            diff = location - other->getLocation();
//            diff.normalize();
//            diff /= d;
//            velocityChange+= diff;
//            count ++;
//        }
//    }
//    if(count > 0){
//        velocityChange /= count;
//        velocityChange.normalize();
//        velocityChange*=topSpeed;
//
//        velocityChange -= velocity;
//        velocityChange.limit(maxVelocityChange);
//    }
//    return velocityChange;
//}

std::vector<glm::vec2> Vehicle::getForces(void)
{
    std::vector<glm::vec2> Forces;
    Forces.push_back( separateF);
    Forces.push_back( seekF);
    Forces.push_back( bordersF);
    Forces.push_back( slopesF);
    Forces.push_back( wanderF);
    return Forces;
}

void Vehicle::applyVelocityChange(const ofGlmPoint & velocityChange){
    globalVelocityChange += velocityChange;
}

void Vehicle::update(){
    projectorCoord = zedProjector->zedCoordToProjCoord(location.x, location.y);
    if (!mother || glm::length2(velocity) != 0)
    {
        velocity += globalVelocityChange;
		glm::clamp(velocity, glm::vec3(0), glm::vec3(topSpeed));
        location += velocity;
        globalVelocityChange *= 0;
        
        float desiredAngle = ofRadToDeg(atan2(velocity.y,velocity.x));
        float angleChange = desiredAngle - angle;
        angleChange += (angleChange > 180) ? -360 : (angleChange < -180) ? 360 : 0; // To take into account that the difference between -180 and 180 is 0 and not 360
        angleChange *= velocity.length();
        angleChange /= topSpeed;
        angleChange = max(min(angleChange, maxRotation), -maxRotation);
        angle += angleChange;
    }
}


//==============================================================
// Derived class Fish
//==============================================================

void Fish::setup(){
    minborderDist = 50;
    internalBorders = borders;
    internalBorders.scaleFromCenter((borders.width-minborderDist)/borders.width, (borders.height-minborderDist)/borders.height);
    
    wanderR = 10;         // Radius for our "wander circle"
    wanderD = 80;         // Distance for our "wander circle"
    change = 0.3;
    
    r = 12;
    desiredseparation = 24;
    maxVelocityChange = 1;
    maxRotation = 30;
    topSpeed =2;
}

ofGlmPoint Fish::wanderEffect(){
    
    ofGlmPoint velocityChange, desired;
    
    wandertheta += ofRandom(-change,change);     // Randomly change wander theta
    
    ofGlmPoint front = velocity;
    glm::normalize(front);
    front *= wanderD;
    ofGlmPoint circleloc = location + front;
    
    float h = ofRadToDeg(atan2(front.y,front.x)); // Signed angle
    
    ofGlmPoint circleOffSet = ofGlmPoint(wanderR*cos(wandertheta+h),wanderR*sin(wandertheta+h),0);
    ofGlmPoint target = circleloc + circleOffSet;
    
    desired = target - location;
    glm::normalize(desired);
    desired *= topSpeed;
    
    velocityChange = desired - velocity;
	glm::clamp(velocityChange, glm::vec3(0), glm::vec3(maxVelocityChange));
    
    return velocityChange;
}

void Fish::applyBehaviours(bool seekMother){
    updateBeachDetection();
    
    //    separateF = separateEffect(vehicles);
    seekF = glm::vec2(0);
    if (seekMother)
        seekF = seekEffect();
    bordersF = bordersEffect();
    slopesF = slopesEffect();
    wanderF = wanderEffect();
    
    //    separateF*=1;//2;
    seekF *= 1;
    bordersF *=2;
    slopesF *= 2;//2;
    wanderF *= 0.8;
    
    if (beach){
        applyVelocityChange(ofGlmPoint (slopesF,0));
    }
    if (border){
        applyVelocityChange(ofGlmPoint(bordersF,0));
    }
    //    applyVelocityChange(separateF);
    if (glm::length2(seekF) == 0){
        applyVelocityChange(ofGlmPoint(wanderF,0));
    } else {
        applyVelocityChange(ofGlmPoint(seekF,0));
    }
    //    currentForce = separateF+seekF+bordersF+slopesF;
}

void Fish::draw()
{
    ofPushMatrix();
    ofTranslate(projectorCoord);
    ofRotate(angle);
    
    // Compute tail angle
    float nv = 0.5;//velocity.lengthSquared()/10; // Tail movement amplitude
    float fact = 50+250*velocity.length()/topSpeed;
    float tailangle = nv/25 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50)-25);
    
    // Color of the fish
    nv = 255;
    fact = 50;
    float hsb = nv/50 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50));
    
    // Fish scale
    float sc = 7;
    float tailSize = 1*sc;
    float fishLength = 2*sc;
    float fishHead = tailSize;
    
    ofPolyline fish;
	fish.curveTo(glm::vec3(1,1,1));
    fish.curveTo(-fishLength-tailSize*cos(tailangle+0.8), tailSize*sin(tailangle+0.8),0,0);
    fish.curveTo( -fishLength-tailSize*cos(tailangle+0.8), tailSize*sin(tailangle+0.8),0,0);
    fish.curveTo( -fishLength, 0,0);
    fish.curveTo( 0, -fishHead,0);
    fish.curveTo(fishHead, 0,0);
    fish.curveTo(0, fishHead,0);
    fish.curveTo(-fishLength,0, 0);
    fish.curveTo(-fishLength-tailSize*cos(tailangle-0.8), tailSize*sin(tailangle-0.8),0);
    fish.curveTo( -fishLength-tailSize*cos(tailangle-0.8), tailSize*sin(tailangle-0.8),0);
    fish.close();
    ofSetLineWidth(2.0);
    ofColor c = ofColor(255);
    ofSetColor(c);
    if (mother)
    {
        c.setHsb((int)hsb, 255, 255); // rainbow
        ofFill();
    } else {
        ofNoFill();
    }
    fish.draw();
    if (mother)
    {
        c.setHsb(255-(int)hsb, 255, 255); // rainbow
        ofSetColor(c);
    }
    ofDrawCircle(0, 0, sc*0.5);
    ofNoFill();
    ofPopMatrix();
}

//==============================================================
// Derived class Rabbit
//==============================================================

void Rabbit::setup(){
    minborderDist = 50;
    internalBorders = borders;
    internalBorders.scaleFromCenter((borders.width-minborderDist)/borders.width, (borders.height-minborderDist)/borders.height);
    
    wanderR = 50;         // Radius for our "wander circle"
    wanderD = 0;         // Distance for our "wander circle"
    change = 1;
    
    r = 12;
    desiredseparation = 24;
    maxVelocityChange = 1;
    maxRotation = 360;
    topSpeed =3;
    velocityIncreaseStep = 2;
    maxStraightPath = 20;
    minVelocity = velocityIncreaseStep;
    
    minWaitingTime = 2;
    maxWaitingTime = 10;
    setWait = false;
}

ofGlmPoint Rabbit::wanderEffect(){
    
    ofGlmPoint velocityChange, desired;
    
    wandertheta = ofRandom(-change,change);     // Randomly change wander theta
    
    float currDir = ofDegToRad(angle);
    ofGlmPoint front = glm::vec3(cos(currDir), sin(currDir),0);
    
    glm::normalize(front);
    front *= wanderD;
    ofGlmPoint circleloc = location + front;
    
    //    float h = ofRadToDeg(atan2(front.x,front.y));
    //	float h = front.angle(glm::vec2(1,0)); // We need to know the heading to offset wandertheta
    
    ofGlmPoint circleOffSet = ofGlmPoint(wanderR*cos(wandertheta+currDir),wanderR*sin(wandertheta+currDir),0);
    ofGlmPoint target = circleloc + circleOffSet;
    
    desired = target - location;
    glm::normalize(desired);
    desired *= topSpeed;
    
    velocityChange = desired;// - velocity;
	glm::clamp(velocityChange, glm::vec3(0), glm::vec3(maxVelocityChange));
    
    return velocityChange;
}

void Rabbit::applyBehaviours(bool seekMother){
    updateBeachDetection();
    
    //    separateF = separateEffect(vehicles);
    seekF = glm::vec2(0);
    if (seekMother)
        seekF = seekEffect();
    bordersF = bordersEffect();
    slopesF = slopesEffect();
    wanderF = wanderEffect();
    
    ofGlmPoint littleSlopeF = ofGlmPoint(slopesF,0);
    
    //    separateF*=1;//2;
    seekF *= 1;
    bordersF *=0.5;
    slopesF *= 2;//2;
    wanderF *= 1;// Used to introduce some randomness in the direction changes
    littleSlopeF *= 1;
    
    float currDir = ofDegToRad(angle);
    ofGlmPoint oldDir = glm::vec3(cos(currDir), sin(currDir),0)*velocityIncreaseStep;
    if (beach)
        oldDir= oldDir*(velocityIncreaseStep/beachDist);
    
    if (setWait){
        waitCounter++;
        if (waitCounter > waitTime){
            setWait = false;
            // Compute a new direction
            //            oldDir.scale(topSpeed);
            wanderF = wanderEffect();
            ofGlmPoint newDir;
            if (glm::length2(seekF) == 0){
                newDir = ofGlmPoint(wanderF,0);
            } else {
                newDir = ofGlmPoint(seekF,0);
            }

            //            newDir +=littleSlopeF;
            if (border)
                newDir += ofGlmPoint(bordersF,0);
            if (beach)
                newDir += ofGlmPoint(slopesF,0);
            
            newDir= newDir*velocityIncreaseStep;
            applyVelocityChange(newDir);
            
            currentStraightPathLength = 0;
            angle = ofRadToDeg(atan2(newDir.y,newDir.x));
            
        }
    } else {
        if (!beach && !border && !mother && currentStraightPathLength < maxStraightPath)
        {
            
            applyVelocityChange(oldDir); // Just accelerate
            currentStraightPathLength++;
        } else { // Wee need to decelerate and then change direction
            if (glm::length2(velocity) > minVelocity*minVelocity) // We are not stopped yet
            {
                applyVelocityChange(-oldDir); // Just deccelerate
            } else {
                velocity = ofGlmPoint(0);
                setWait = true;
                waitCounter = 0;
                waitTime = ofRandom(minWaitingTime, maxWaitingTime);
                if (beach)
                    waitTime = 0;
            }
        }
    }
}

void Rabbit::draw()//, std::vector<glm::vec2> forces)
{
    ofPushMatrix();
    ofTranslate(projectorCoord);
    ofRotate(angle);
    
    // Rabbit scale
    float sc = 1;
    
    ofFill();
    ofSetLineWidth(1.0);  // Line widths apply to polylines
    
    ofColor c1 = ofColor(255);
    ofColor c2 = ofColor(0);
    if (mother)
    {
        float nv = 255;
        int fact = 50;
        float et = ofGetElapsedTimef();
        float hsb = nv/50 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50));
        c1.setHsb((int)hsb, 255, 255); // rainbow
        c2.setHsb(255-(int)hsb, 255, 255);
    }
    
    ofPath body;
    body.curveTo( ofGlmPoint(-2*sc, 5.5*sc,0));
    body.curveTo( ofGlmPoint(-2*sc, 5.5*sc,0));
    body.curveTo( ofGlmPoint(-9*sc, 7.5*sc,0));
    body.curveTo( ofGlmPoint(-17*sc, 0*sc,0));
    body.curveTo( ofGlmPoint(-9*sc, -7.5*sc,0));
    body.curveTo( ofGlmPoint(-2*sc, -5.5*sc,0));
    body.curveTo( ofGlmPoint(4*sc, 0*sc,0));
    body.curveTo( ofGlmPoint(4*sc, 0*sc,0));
    body.close();
    ofSetColor(c1);
    body.setFillColor(c1);
    body.draw();
    
    ofSetColor(c2);
    ofDrawCircle(-19*sc, 0, 2*sc);

    ofPath head;
    head.curveTo( ofGlmPoint(0, 1.5*sc,0));
    head.curveTo( ofGlmPoint(0, 1.5*sc,0));
    head.curveTo( ofGlmPoint(-3*sc, 1.5*sc,0));
    head.curveTo( ofGlmPoint(-9*sc, 3.5*sc,0));
    head.curveTo( ofGlmPoint(0, 5.5*sc,0));
    head.curveTo( ofGlmPoint(8*sc, 0,0));
    head.curveTo( ofGlmPoint(0, -5.5*sc,0));
    head.curveTo( ofGlmPoint(-9*sc, -3.5*sc,0));
    head.curveTo( ofGlmPoint(-3*sc, -1.5*sc,0));
    head.curveTo( ofGlmPoint(0, -1.5*sc,0));
    head.curveTo( ofGlmPoint(0, -1.5*sc,0));
    head.close();
    ofSetColor(c2);
    head.setFillColor(c2);
    head.draw();
    
    ofSetColor(c1);
    ofDrawCircle(8.5*sc, 0, 1*sc);

    ofSetColor(255);
    ofNoFill();

    ofPopMatrix();
}


