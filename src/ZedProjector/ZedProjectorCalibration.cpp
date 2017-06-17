/***********************************************************************
ZedProjectorCalibration.cpp - ZedProjectorCalibration compute
the calibration of the Zed and projector.
Copyright (c) 2016 Thomas Wolf

--- Adapted from ofxZedProjectorToolkit by Gene Kogan:
https://github.com/genekogan/ofxZedProjectorToolkit
Copyright (c) 2014 Gene Kogan

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
with the Magic Sand; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#include "ZedProjectorCalibration.h"
#include "ofxXmlPoco.h"

ofxZedProjectorToolkit::ofxZedProjectorToolkit(glm::vec2 sprojRes, glm::vec2 szedRes) {
	projRes = sprojRes;
	zedRes = szedRes;
    calibrated = false;
}

void ofxZedProjectorToolkit::calibrate(vector<glm::vec3> pairsZed,
                                          vector<glm::vec2> pairsProjector) {
    int nPairs = pairsZed.size();
    A.set_size(nPairs*2, 11);
    y.set_size(nPairs*2, 1);
    
    for (int i=0; i<nPairs; i++) {
        A(2*i, 0) = pairsZed[i].x;
        A(2*i, 1) = pairsZed[i].y;
        A(2*i, 2) = pairsZed[i].z;
        A(2*i, 3) = 1;
        A(2*i, 4) = 0;
        A(2*i, 5) = 0;
        A(2*i, 6) = 0;
        A(2*i, 7) = 0;
        A(2*i, 8) = -pairsZed[i].x * pairsProjector[i].x;
        A(2*i, 9) = -pairsZed[i].y * pairsProjector[i].x;
        A(2*i, 10) = -pairsZed[i].z * pairsProjector[i].x;
        
        A(2*i+1, 0) = 0;
        A(2*i+1, 1) = 0;
        A(2*i+1, 2) = 0;
        A(2*i+1, 3) = 0;
        A(2*i+1, 4) = pairsZed[i].x;
        A(2*i+1, 5) = pairsZed[i].y;
        A(2*i+1, 6) = pairsZed[i].z;
        A(2*i+1, 7) = 1;
        A(2*i+1, 8) = -pairsZed[i].x * pairsProjector[i].y;
        A(2*i+1, 9) = -pairsZed[i].y * pairsProjector[i].y;
        A(2*i+1, 10) = -pairsZed[i].z * pairsProjector[i].y;
        
        y(2*i, 0) = pairsProjector[i].x;
        y(2*i+1, 0) = pairsProjector[i].y;
    }
    
    dlib::qr_decomposition<dlib::matrix<double, 0, 11> > qrd(A);
    x = qrd.solve(y);
    cout << "x: "<< x << endl;
    projMatrice = glm::mat4x4(x(0,0), x(1,0), x(2,0), x(3,0),
                              x(4,0), x(5,0), x(6,0), x(7,0),
                              x(8,0), x(9,0), x(10,0), 1,
                              0, 0, 0, 1);
    calibrated = true;
}

glm::mat4x4 ofxZedProjectorToolkit::getProjectionMatrix() {
    return projMatrice;
}

glm::vec2 ofxZedProjectorToolkit::getProjectedPoint(glm::vec3 worldPoint) {
    glm::vec4 pts = glm::vec4(worldPoint,0);
    pts.w = 1;
    glm::vec4 rst = projMatrice*(pts);
    glm::vec2 projectedPoint(rst.x/rst.z, rst.y/rst.z);
    return projectedPoint;
}

vector<double> ofxZedProjectorToolkit::getCalibration()
{
    vector<double> coefficients;
    for (int i=0; i<11; i++) {
        coefficients.push_back(x(i, 0));
    }
    return coefficients;
}

bool ofxZedProjectorToolkit::loadCalibration(string path){
	ofxXmlPoco xml;
    if (!xml.load(path))
        return false;
	xml.setTo("RESOLUTIONS");
	glm::vec2 sprojRes = xml.getValue<glm::vec2>("PROJECTOR");
	glm::vec2 szedRes = xml.getValue<glm::vec2>("Zed");
	if (sprojRes!=projRes || szedRes!=zedRes)
		return false;
    xml.setTo("//CALIBRATION/COEFFICIENTS");
    for (int i=0; i<11; i++) {
        x(i, 0) = xml.getValue<float>("COEFF"+ofToString(i));
    }
    projMatrice = glm::mat4x4(x(0,0), x(1,0), x(2,0), x(3,0),
                              x(4,0), x(5,0), x(6,0), x(7,0),
                              x(8,0), x(9,0), x(10,0), 1,
                              0, 0, 0, 0);
    calibrated = true;
    return true;
}

bool ofxZedProjectorToolkit::saveCalibration(string path){
	ofxXmlPoco xml;
	xml.addChild("CALIBRATION");
	xml.setTo("//CALIBRATION");
	xml.addChild("RESOLUTIONS");
	xml.setTo("RESOLUTIONS");
	xml.addValue("PROJECTOR", projRes);
	xml.addValue("Zed", zedRes);
	xml.setTo("//CALIBRATION");
	xml.addChild("COEFFICIENTS");
	xml.setTo("COEFFICIENTS");
	for (int i=0; i<11; i++) {
		ofxXmlPoco coeff;
        coeff.addValue("COEFF"+ofToString(i), x(i, 0));
        xml.addXml(coeff);
    }
    xml.setToParent();
    return xml.save(path);
}


