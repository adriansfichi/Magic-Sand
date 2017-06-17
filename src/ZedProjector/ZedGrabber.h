/***********************************************************************
ZedGrabber - ZedGrabber takes care of the communication with
the zed and the filtering of depth frame.
Copyright (c) 2016 Thomas Wolf

--- Adapted from FrameFilter of the Augmented Reality Sandbox
Copyright (c) 2012-2015 Oliver Kreylos

This file is part of the Magic Sand.

The Magic Sand is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Magic Sand is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.
***********************************************************************/

#pragma once
// Standard includes
#include <stdio.h>
#include <string.h>

// OpenGL includes


// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

#include "Utils.h"

class ZedGrabber: public ofThread {
public:
	typedef unsigned short RawDepth; // Data type for raw depth values
	typedef float FilteredDepth; // Data type for filtered depth values

	ZedGrabber();
	~ZedGrabber();
    void start();
    void stop();
    void performInThread(std::function<void(ZedGrabber&)> action);
    bool setup();
	bool openZed();
	void setupFramefilter(int gradFieldresolution, float newMaxOffset, ofRectangle ROI, bool spatialFilter, bool followBigChange, int numAveragingSlots);
    void initiateBuffers(void); // Reinitialise buffers
    void resetBuffers(void);
    
    glm::vec3 getStatBuffer(int x, int y);
    float getAveragingBuffer(int x, int y, int slotNum);
    float getValidBuffer(int x, int y);
    
    void setFollowBigChange(bool newfollowBigChange);
    void setzedROI(ofRectangle szedROI);
    void setAveragingSlotsNumber(int snumAveragingSlots);
    void setGradFieldResolution(int sgradFieldresolution);
    
    void decStoredframes(){
        storedframes -= 1;
    }
    
    bool isImageStabilized(){
        return firstImageReady;
    }
    
    bool isFrameNew(){
        return newFrame;
    }
    
    glm::vec2 getzedSize(){
        return glm::vec2(width, height);
    }
    
    float getRawDepthAt(int x, int y){
        return zedDepthImage.getData()[(int)(y*width+x)];
    }

	glm::mat4x4  getWorldMatrix();
    
    int getNumAveragingSlots(){
        return numAveragingSlots;
    }
    
    void setMaxOffset(float newMaxOffset){
        maxOffset = newMaxOffset;
    }
    
    void setSpatialFiltering(bool newspatialFilter){
        spatialFilter = newspatialFilter;
    }
    
	ofThreadChannel<ofFloatPixels> filtered;
	ofThreadChannel<ofPixels> colored;
	ofThreadChannel<glm::vec2*> gradient;

	//------------------------------------------ofxKuZed implementation

	bool started();		//is ZED working now



	//All textures and pixels arrays are "lazy" updated,
	//that is thay are updated only by request
	ofFloatPixels &getDepthPixels_mm();
	ofPixels &getDepthPixels_grayscale(float min_depth_mm = 0.0, float max_depth_mm = 5000.0);
	ofTexture &getDepthTexture(float min_depth_mm = 0.0, float max_depth_mm = 5000.0);

	ofTexture &getLeftTexture();
	ofPixels &getLeftPixels();
	ofTexture &getRightTexture();
	ofPixels &getRightPixels();

	vector<ofGlmPoint> &getPointCloud();
	vector<ofColor> &getPointCloudColors();
	vector<ofFloatColor> &getPointCloudFloatColors();	//required for ofMesh

	void drawLeft(float x, float y, float w = 0, float h = 0);
	void drawRight(float x, float y, float w = 0, float h = 0);
	void drawDepth(float x, float y, float w = 0, float h = 0, float min_mm = 0, float max_mm = 5000);
	void drawPointCloud();	//draws a mesh of points

    
private:
	void threadedFunction() override;
    void filter();
    bool isInsideROI(int x, int y); // test is x, y is inside ROI
    void applySpaceFilter();
    void updateGradientField();
    
	bool newFrame;
    bool bufferInitiated;
    bool firstImageReady;
    int storedframes;
    
    // Thread lambda functions (actions)
	vector<std::function<void(ZedGrabber&)> > actions;
	ofMutex actionsLock;
    
    // zed parameters
	bool zedOpened;
	sl::Camera  zed;
    unsigned int width, height; // Width and height of zed frames
    int minX, maxX, ROIwidth; // ROI definition
    int minY, maxY, ROIheight;
    
    // General buffers
    ofxCvColorImage         zedColorImage;
    ofShortPixels     zedDepthImage;
    ofFloatPixels filteredframe;
    glm::vec2* gradField;
    
    // Filtering buffers
	float* averagingBuffer; // Buffer to calculate running averages of each pixel's depth value
	float* statBuffer; // Buffer retaining the running means and variances of each pixel's depth value
	float* validBuffer; // Buffer holding the most recent stable depth value for each pixel
    
    // Gradient computation variables
    int gradFieldcols, gradFieldrows;
    int gradFieldresolution;           //Resolution of grid relative to window width and height in pixels
    float maxgradfield, depthrange;
    
    // Frame filter parameters
	int numAveragingSlots; // Number of slots in each pixel's averaging buffer
	int averagingSlotIndex; // Index of averaging slot in which to store the next frame's depth values
	unsigned int minNumSamples; // Minimum number of valid samples needed to consider a pixel stable
	float maxVariance; // Maximum variance to consider a pixel stable
    float initialValue;
    float outsideROIValue;
	float hysteresis; // Amount by which a new filtered value has to differ from the current value to update the display
    bool followBigChange;
    float bigChange; // Amount of change over which the averaging slot is reset to new value
	float instableValue; // Value to assign to instable pixels if retainValids is false
	bool spatialFilter; // Flag whether to apply a spatial filter to time-averaged depth values
    float maxOffset;
    
    int minInitFrame; // Minimal number of frame to consider the zed initialized
    int currentInitFrame;
    

	//ofxKuZed implementation
	//Buffers

	bool useImages_ = true;
	bool useDepth_ = true;
	bool usePointCloud_ = true;
	bool usePointCloudColors_ = true;

	bool pointCloudFlipY_ = true;
	bool pointCloudFlipZ_ = true;

	ofPixels leftPixels_, rightPixels_, depthPixels_grayscale_;
	ofTexture leftTexture_, rightTexture_, depthTexture_;
	ofFloatPixels depthPixels_mm_;
	vector<ofGlmPoint> pointCloud_;
	vector<ofColor> pointCloudColors_;
	vector<ofFloatColor> pointCloudFloatColors_;

	//Flags for lazy updating
	bool leftPixelsDirty_, rightPixelsDirty_, leftTextureDirty_, rightTextureDirty_;
	bool depthPixels_mm_Dirty_, depthPixels_grayscale_Dirty_, depthTextureDirty_;
	bool pointCloudDirty_, pointCloudFloatColorsDirty_;


	void markBuffersDirty(bool dirty);	//Mark all buffers dirty (need to update by request)
	void fillPointCloud();
    // Debug
//    int blockX, blockY;
};
