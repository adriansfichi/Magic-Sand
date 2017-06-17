/***********************************************************************
ZedProjector - ZedProjector takes care of the spatial conversion
between the various coordinate systems, control the zedGrabber and
perform the calibration of the Zed and projector.
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
with the Magic Sand; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#ifndef __GreatSand__ZedProjector__
#define __GreatSand__ZedProjector__

#include <iostream>
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ZedGrabber.h"
#include "ofxModal.h"

#include "ZedProjectorCalibration.h"
#include "Utils.h"
class ofxModalThemeProjZed : public ofxModalTheme {
public:
	ofxModalThemeProjZed()
	{
		animation.speed = 0.1f;
		fonts.title = ofxSmartFont::add("ofxbraitsch/fonts/HelveticaNeueLTStd-Md.otf", 20, "modal-title");
		fonts.message = ofxSmartFont::add("ofxbraitsch/fonts/Roboto-Regular.ttf", 16, "modal-message");
	}
};

class ZedProjector {
public:
	ZedProjector(std::shared_ptr<ofAppBaseWindow> const& p);

	// Running loop functions
	void setup(bool sdisplayGui);
	void update();
	void updateNativeScale(float scaleMin, float scaleMax);
	void drawProjectorWindow();
	void drawMainWindow(float x, float y, float width, float height);
	void drawGradField();

	// Coordinate conversion functions
	glm::vec2 worldCoordToProjCoord(glm::vec3 vin);
	glm::vec3 projCoordAndWorldZToWorldCoord(float projX, float projY, float worldZ);
	glm::vec2 zedCoordToProjCoord(float x, float y);
	glm::vec3 ZedCoordToWorldCoord(float x, float y);
	glm::vec2 worldCoordToZedCoord(glm::vec3 wc);
	glm::vec3 RawZedCoordToWorldCoord(float x, float y);
	float elevationAtZedCoord(float x, float y);
	float elevationToZedDepth(float elevation, float x, float y);
	glm::vec2 gradientAtZedCoord(float x, float y);

	// Setup & calibration functions
	void startFullCalibration();
	void startAutomaticROIDetection();
	void startAutomaticZedProjectorCalibration();
	void setGradFieldResolution(int gradFieldResolution);
	void setSpatialFiltering(bool sspatialFiltering);
	void setFollowBigChanges(bool sfollowBigChanges);

	// Gui and event functions
	void setupGui();
	void onButtonEvent(ofxDatGuiButtonEvent e);
	void onToggleEvent(ofxDatGuiToggleEvent e);
	void onSliderEvent(ofxDatGuiSliderEvent e);
	void onConfirmModalEvent(ofxModalEvent e);
	void onCalibModalEvent(ofxModalEvent e);

	// Functions for shaders
	void bind() {
		FilteredDepthImage.getTexture().bind();
	}
	void unbind() {
		FilteredDepthImage.getTexture().unbind();
	}
	glm::mat4x4  getTransposedZedWorldMatrix() {
		return glm::transpose(ZedWorldMatrix);
	} // For shaders: OpenGL is row-major order and OF is column-major order
	glm::mat4x4  getTransposedZedProjMatrix() {
		return glm::transpose(ZedProjMatrix);
	}

	// Getter and setter
	ofTexture & getTexture() {
		return FilteredDepthImage.getTexture();
	}
	ofRectangle getZedROI() {
		return zedROI;
	}
	glm::vec2 getZedRes() {
		return zedRes;
	}
	glm::vec4 getBasePlaneEq() {
		return basePlaneEq;
	}
	glm::vec3 getBasePlaneNormal() {
		return basePlaneNormal;
	}
	glm::vec3 getBasePlaneOffset() {
		return basePlaneOffset;
	}
	bool isCalibrating() {
		return calibrating;
	}
	bool isCalibrated() {
		return projZedCalibrated;
	}
	bool isImageStabilized() {
		return imageStabilized;
	}
	bool isBasePlaneUpdated() { // To be called after update()
		return basePlaneUpdated;
	}
	bool isROIUpdated() { // To be called after update()
		return ROIUpdated;
	}
	bool isCalibrationUpdated() { // To be called after update()
		return projZedCalibrationUpdated;
	}

private:
	enum Calibration_state
	{
		CALIBRATION_STATE_FULL_AUTO_CALIBRATION,
		CALIBRATION_STATE_ROI_AUTO_DETERMINATION,
		CALIBRATION_STATE_ROI_MANUAL_DETERMINATION,
		CALIBRATION_STATE_PROJ_Zed_AUTO_CALIBRATION,
		CALIBRATION_STATE_PROJ_Zed_MANUAL_CALIBRATION
	};
	enum Full_Calibration_state
	{
		FULL_CALIBRATION_STATE_ROI_DETERMINATION,
		FULL_CALIBRATION_STATE_AUTOCALIB,
		FULL_CALIBRATION_STATE_DONE
	};
	enum ROI_calibration_state
	{
		ROI_CALIBRATION_STATE_INIT,
		ROI_CALIBRATION_STATE_READY_TO_MOVE_UP,
		ROI_CALIBRATION_STATE_MOVE_UP,
		ROI_CALIBRATION_STATE_DONE
	};
	enum Auto_calibration_state
	{
		AUTOCALIB_STATE_INIT_FIRST_PLANE,
		AUTOCALIB_STATE_INIT_POINT,
		AUTOCALIB_STATE_NEXT_POINT,
		AUTOCALIB_STATE_COMPUTE,
		AUTOCALIB_STATE_DONE
	};

	// Private methods
	void exit(ofEventArgs& e);
	void setupGradientField();

	void updateCalibration();
	void updateFullAutoCalibration();
	void updateROIAutoCalibration();
	void updateROIFromColorImage();
	void updateROIFromDepthImage();
	void updateROIManualCalibration();
	void updateROIFromCalibration();
	void setMaxZedGrabberROI();
	void setNewZedROI();
	void updateZedGrabberROI(ofRectangle ROI);

	void updateProjZedAutoCalibration();
	void updateProjZedManualCalibration();
	bool addPointPair();
	void updateMaxOffset();
	void updateBasePlane();
	void askToFlattenSand();

	void drawChessboard(int x, int y, int chessboardSize);
	void drawArrow(glm::vec2 projectedPoint, glm::vec2 v1);

	void saveCalibrationAndSettings();
	bool loadSettings();
	bool saveSettings();

	// States variables
	bool secondScreenFound;
	bool ZedOpened;
	bool ROIcalibrated;
	bool projZedCalibrated;
	bool calibrating;
	bool ROIUpdated;
	bool projZedCalibrationUpdated;
	bool basePlaneUpdated;
	bool imageStabilized;
	bool waitingForFlattenSand;
	bool drawZedView;
	Calibration_state calibrationState;
	ROI_calibration_state ROICalibState;
	Auto_calibration_state autoCalibState;
	Full_Calibration_state fullCalibState;

	// Projector window
	std::shared_ptr<ofAppBaseWindow> projWindow;

	//Zed grabber
	ZedGrabber               zedGrabber;
	bool                        spatialFiltering;
	bool                        followBigChanges;
	int                         numAveragingSlots;

	//Zed buffer
	ofxCvFloatImage             FilteredDepthImage;
	ofxCvColorImage             ZedColorImage;
	glm::vec2*                    gradField;

	// Projector and Zed variables
	glm::vec2 projRes;
	glm::vec2 zedRes;

	// FBos
	ofFbo fboProjWindow;
	ofFbo fboMainWindow;

	//Images and cv matrixes
	cv::Mat                     cvRgbImage;
	ofxCvFloatImage             Dptimg;

	//Gradient field variables
	int gradFieldcols, gradFieldrows;
	int gradFieldResolution;
	float arrowLength;
	int fishInd;

	// Calibration variables
	ofxZedProjectorToolkit*  kpt;
	vector<ofVec2f>             currentProjectorPoints;
	vector<cv::Point2f>         cvPoints;
	vector<glm::vec3>             pairsZed;
	vector<glm::vec2>             pairsProjector;

	// ROI calibration variables
	ofxCvGrayscaleImage         thresholdedImage;
	ofxCvContourFinder          contourFinder;
	float                       threshold;
	ofPolyline                  large;
	ofRectangle                 zedROI, zedROIManualCalib;

	// Base plane
	glm::vec3 basePlaneNormal, basePlaneNormalBack;
	glm::vec3 basePlaneOffset, basePlaneOffsetBack;
	glm::vec4 basePlaneEq; // Base plane equation in GLSL-compatible format

						   // Conversion matrices
	glm::mat4x4                 ZedProjMatrix;
	glm::mat4x4                  ZedWorldMatrix;

	// Max offset for keeping Zed points
	float maxOffset;
	float maxOffsetSafeRange;
	float maxOffsetBack;

	// Autocalib points
	ofGlmPoint* autoCalibPts; // Center of autocalib chess boards
	int currentCalibPts;
	bool cleared;
	int trials;
	bool upframe;

	// Chessboard variables
	int   chessboardSize;
	int   chessboardX;
	int   chessboardY;

	// GUI Modal window & interface
	bool displayGui;
	shared_ptr<ofxModalConfirm>   confirmModal;
	shared_ptr<ofxModalAlert>   calibModal;
	shared_ptr<ofxModalThemeProjZed>   modalTheme;
	ofxDatGui* gui;
};


#endif 


