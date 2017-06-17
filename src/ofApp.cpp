/***********************************************************************
ofApp.cpp - main openframeworks app
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

#include "ofApp.h"

void ofApp::setup() {
	// OF basics
	ofSetFrameRate(60);
	ofBackground(0);
	ofSetVerticalSync(true);
	//	ofSetLogLevel(OF_LOG_VERBOSE);
	ofSetLogLevel("ofThread", OF_LOG_WARNING);

	// Setup zedProjector
	zedProjector = std::make_shared<ZedProjector>(projWindow);
	zedProjector->setup(true);

	// Setup sandSurfaceRenderer
	sandSurfaceRenderer = new SandSurfaceRenderer(zedProjector, projWindow);
	sandSurfaceRenderer->setup(true);

	// Retrieve variables
	kinectRes = zedProjector->getZedRes();
	projRes = glm::vec2(projWindow->getWidth(), projWindow->getHeight());
	kinectROI = zedProjector->getZedROI();

	fboVehicles.allocate(projRes.x, projRes.y, GL_RGBA);
	fboVehicles.begin();
	ofClear(0, 0, 0, 255);
	fboVehicles.end();

	setupGui();

	// Vehicles
	showMotherFish = false;
	showMotherRabbit = false;
	motherPlatformSize = 30;
}

void ofApp::addNewFish() {
	glm::vec2 location;
	setRandomVehicleLocation(kinectROI, true, location);
	auto f = Fish(zedProjector, ofGlmPoint(location, 0), kinectROI, motherFish);
	f.setup();
	fish.push_back(f);
}

void ofApp::addNewRabbit() {
	glm::vec2 location;
	setRandomVehicleLocation(kinectROI, false, location);
	auto r = Rabbit(zedProjector, ofGlmPoint(location, 0), kinectROI, motherRabbit);
	r.setup();
	rabbits.push_back(r);
}

bool ofApp::addMotherFish() {
	int minborderDist = 40;
	ofRectangle internalBorders = kinectROI;
	internalBorders.scaleFromCenter((kinectROI.width - minborderDist) / kinectROI.width, (kinectROI.height - minborderDist) / kinectROI.height);

	// Try to set a location for the Fish mother outside of the water to be sure the fish cannot reach her without help
	glm::vec2 location;
	if (!setRandomVehicleLocation(internalBorders, false, location)) {
		return false;
	}
	motherFish = ofGlmPoint(location, 0);

	// Set the mother Fish plateform location under the sea level
	motherFish.z = zedProjector->elevationToZedDepth(-10, motherFish.x, motherFish.y);
	for (auto & f : fish) {
		f.setMotherLocation(motherFish);
	}
	showMotherFish = true;
	return true;
}

bool ofApp::addMotherRabbit() {
	int minborderDist = 40;
	ofRectangle internalBorders = kinectROI;
	internalBorders.scaleFromCenter((kinectROI.width - minborderDist) / kinectROI.width, (kinectROI.height - minborderDist) / kinectROI.height);

	// Set a location for the Rabbits mother inside of the water to be sure the rabbits cannot reach her without help
	glm::vec2 location;
	if (!setRandomVehicleLocation(internalBorders, true, location)) {
		return false;
	}
	motherRabbit = ofGlmPoint(location, 0);

	// Set the mother Rabbit plateform location over the sea level
	motherRabbit.z = zedProjector->elevationToZedDepth(10, motherRabbit.x, motherRabbit.y);

	for (auto & r : rabbits) {
		r.setMotherLocation(motherRabbit);
	}
	showMotherRabbit = true;
	return true;
}

bool ofApp::setRandomVehicleLocation(ofRectangle area, bool liveInWater, glm::vec2 & location) {
	bool okwater = false;
	int count = 0;
	int maxCount = 100;
	while (!okwater && count < maxCount) {
		count++;
		float x = ofRandom(area.getLeft(), area.getRight());
		float y = ofRandom(area.getTop(), area.getBottom());
		bool insideWater = zedProjector->elevationAtZedCoord(x, y) < 0;
		if ((insideWater && liveInWater) || (!insideWater && !liveInWater)) {
			location = glm::vec2(x, y);
			okwater = true;
		}
	}
	return okwater;
}

void ofApp::update() {
	// Call zedProjector->update() first during the update function()
	zedProjector->update();

	sandSurfaceRenderer->update();

	if (zedProjector->isROIUpdated())
		kinectROI = zedProjector->getZedROI();

	if (zedProjector->isImageStabilized()) {
		for (auto & f : fish) {
			f.applyBehaviours(showMotherFish);
			f.update();
		}
		for (auto & r : rabbits) {
			r.applyBehaviours(showMotherRabbit);
			r.update();
		}
		drawVehicles();
	}
	gui->update();
}


void ofApp::draw() {
	sandSurfaceRenderer->drawMainWindow(300, 30, 600, 450);//400, 20, 400, 300);
	fboVehicles.draw(300, 30, 600, 450);
	zedProjector->drawMainWindow(300, 30, 600, 450);
	gui->draw();
}

void ofApp::drawProjWindow(ofEventArgs &args) {
	zedProjector->drawProjectorWindow();

	if (!zedProjector->isCalibrating()) {
		sandSurfaceRenderer->drawProjectorWindow();
		fboVehicles.draw(0, 0);
	}
}

void ofApp::drawVehicles()
{
	fboVehicles.begin();
	ofClear(255, 255, 255, 0);
	if (showMotherFish)
		drawMotherFish();
	if (showMotherRabbit)
		drawMotherRabbit();
	for (auto & f : fish) {
		f.draw();
	}
	for (auto & r : rabbits) {
		r.draw();
	}
	fboVehicles.end();
}

void ofApp::drawMotherFish()
{
	// Mother fish scale
	float sc = 10;
	float tailSize = 1 * sc;
	float fishLength = 2 * sc;
	float fishHead = tailSize;
	float tailangle = 0;

	ofPushMatrix();
	ofTranslate(zedProjector->zedCoordToProjCoord(motherFish.x + tailSize, motherFish.y));

	ofFill();
	ofSetColor(ofColor::blueSteel);
	ofDrawCircle(-0.5*sc, 0, motherPlatformSize);

	ofFill();
	ofSetColor(255);
	ofPolyline fish;
	fish.curveTo(-fishLength - tailSize*cos(tailangle + 0.8), tailSize*sin(tailangle + 0.8), 0);
	fish.curveTo(-fishLength - tailSize*cos(tailangle + 0.8), tailSize*sin(tailangle + 0.8), 0);
	fish.curveTo(-fishLength, 0, 0);
	fish.curveTo(0, -fishHead, 0);
	fish.curveTo(fishHead, 0, 0);
	fish.curveTo(0, fishHead, 0);
	fish.curveTo(-fishLength, 0, 0);
	fish.curveTo(-fishLength - tailSize*cos(tailangle - 0.8), tailSize*sin(tailangle - 0.8), 0);
	fish.curveTo(-fishLength - tailSize*cos(tailangle - 0.8), tailSize*sin(tailangle - 0.8), 0);
	fish.close();
	ofSetLineWidth(2.0);
	fish.draw();
	ofSetColor(255);
	ofDrawCircle(0, 0, 5);
	ofPopMatrix();
}

void ofApp::drawMotherRabbit()
{
	float sc = 2; // MotherRabbit scale
	ofPushMatrix();
	ofTranslate(zedProjector->zedCoordToProjCoord(motherRabbit.x + 5 * sc, motherRabbit.y));

	ofFill();
	ofSetColor(ofColor::green);
	ofDrawCircle(-5 * sc, 0, motherPlatformSize);

	ofFill();
	ofSetLineWidth(1.0);
	ofPath body;
	body.curveTo(ofGlmPoint(-2 * sc, 5.5*sc, 0));
	body.curveTo(ofGlmPoint(-2 * sc, 5.5*sc, 0));
	body.curveTo(ofGlmPoint(-9 * sc, 7.5*sc, 0));
	body.curveTo(ofGlmPoint(-17 * sc, 0 * sc, 0));
	body.curveTo(ofGlmPoint(-9 * sc, -7.5*sc, 0));
	body.curveTo(ofGlmPoint(-2 * sc, -5.5*sc, 0));
	body.curveTo(ofGlmPoint(4 * sc, 0 * sc, 0));
	body.curveTo(ofGlmPoint(4 * sc, 0 * sc, 0));
	body.close();
	body.setFillColor(0);
	body.draw();

	ofSetColor(255);
	ofDrawCircle(-19 * sc, 0, 2 * sc);

	ofPath head;
	head.curveTo(ofGlmPoint(0, 1.5*sc, 0));
	head.curveTo(ofGlmPoint(0, 1.5*sc, 0));
	head.curveTo(ofGlmPoint(-3 * sc, 1.5*sc, 0));
	head.curveTo(ofGlmPoint(-9 * sc, 3.5*sc, 0));
	head.curveTo(ofGlmPoint(0, 5.5*sc, 0));
	head.curveTo(ofGlmPoint(8 * sc, 0, 0));
	head.curveTo(ofGlmPoint(0, -5.5*sc, 0));
	head.curveTo(ofGlmPoint(-9 * sc, -3.5*sc, 0));
	head.curveTo(ofGlmPoint(-3 * sc, -1.5*sc, 0));
	head.curveTo(ofGlmPoint(0, -1.5*sc, 0));
	head.curveTo(ofGlmPoint(0, -1.5*sc, 0));
	head.close();
	head.setFillColor(255);
	head.draw();

	ofSetColor(0);
	ofDrawCircle(8.5*sc, 0, 1 * sc);

	ofPopMatrix();
	ofSetColor(255);
}

void ofApp::keyPressed(int key) {

}

void ofApp::keyReleased(int key) {

}

void ofApp::mouseMoved(int x, int y) {

}

void ofApp::mouseDragged(int x, int y, int button) {

}

void ofApp::mousePressed(int x, int y, int button) {

}

void ofApp::mouseReleased(int x, int y, int button) {

}

void ofApp::mouseEntered(int x, int y) {

}

void ofApp::mouseExited(int x, int y) {

}

void ofApp::windowResized(int w, int h) {

}

void ofApp::gotMessage(ofMessage msg) {

}

void ofApp::dragEvent(ofDragInfo dragInfo) {

}

void ofApp::setupGui() {
	// instantiate and position the gui //
	gui = new ofxDatGui();
	gui->addSlider("# of fish", 0, 10, fish.size())->setPrecision(0);
	gui->addSlider("# of rabbits", 0, 10, rabbits.size())->setPrecision(0);
	gui->addToggle("Mother fish", showMotherFish);
	gui->addToggle("Mother rabbit", showMotherRabbit);
	gui->addButton("Remove all animals");
	gui->addBreak();
	gui->addHeader(":: Game ::", false);

	gui->onButtonEvent(this, &ofApp::onButtonEvent);
	gui->onToggleEvent(this, &ofApp::onToggleEvent);
	gui->onSliderEvent(this, &ofApp::onSliderEvent);
	gui->setLabelAlignment(ofxDatGuiAlignment::CENTER);

	gui->setPosition(ofxDatGuiAnchor::BOTTOM_RIGHT); // You have to do it at the end
	gui->setAutoDraw(false); // troubles with multiple windows drawings on Windows
}

void ofApp::onButtonEvent(ofxDatGuiButtonEvent e) {
	if (e.target->is("Remove all animals")) {
		fish.clear();
		rabbits.clear();
		showMotherFish = false;
		showMotherRabbit = false;
		gui->getSlider("# of fish")->setValue(0);
		gui->getSlider("# of rabbits")->setValue(0);
		gui->getToggle("Mother fish")->setChecked(false);
		gui->getToggle("Mother rabbit")->setChecked(false);
	}
}

void ofApp::onToggleEvent(ofxDatGuiToggleEvent e) {
	if (e.target->is("Mother fish")) {
		if (!showMotherFish) {
			if (!addMotherFish())
				e.target->setChecked(false);
		}
		else {
			showMotherFish = e.checked;
		}
	}
	else if (e.target->is("Mother rabbit")) {
		if (!showMotherRabbit) {
			if (!addMotherRabbit())
				e.target->setChecked(false);
		}
		else {
			showMotherRabbit = e.checked;
		}
	}
}

void ofApp::onSliderEvent(ofxDatGuiSliderEvent e) {
	if (e.target->is("# of fish")) {
		if (e.value > fish.size())
			while (e.value > fish.size()) {
				addNewFish();
			}
		if (e.value < fish.size())
			while (e.value < fish.size()) {
				fish.pop_back();
			}

	}
	else if (e.target->is("# of rabbits")) {
		if (e.value > rabbits.size())
			while (e.value > rabbits.size()) {
				addNewRabbit();
			}
		if (e.value < rabbits.size())
			while (e.value < rabbits.size()) {
				rabbits.pop_back();
			}
	}
}
