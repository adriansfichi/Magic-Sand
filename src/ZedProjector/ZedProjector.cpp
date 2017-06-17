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

#include "ZedProjector.h"
#include "ZedProjectorCalibration.h"
#include "ofxXmlPoco.h"

using namespace ofxCSG;

ZedProjector::ZedProjector(std::shared_ptr<ofAppBaseWindow> const& p)
	:ROIcalibrated(false),
	projZedCalibrated(false),
	calibrating(true),
	basePlaneUpdated(false),
	projZedCalibrationUpdated(false),
	ROIUpdated(false),
	imageStabilized(false),
	waitingForFlattenSand(false),
	drawZedView(false)
{
	projWindow = p;
}

void ZedProjector::setup(bool sdisplayGui) {
	ofAddListener(ofEvents().exit, this, &ZedProjector::exit);

	// instantiate the modal windows //
	modalTheme = make_shared<ofxModalThemeProjZed>();
	confirmModal = make_shared<ofxModalConfirm>();
	confirmModal->setTheme(modalTheme);
	confirmModal->addListener(this, &ZedProjector::onConfirmModalEvent);
	confirmModal->setButtonLabel("Ok");

	calibModal = make_shared<ofxModalAlert>();
	calibModal->setTheme(modalTheme);
	calibModal->addListener(this, &ZedProjector::onCalibModalEvent);
	calibModal->setButtonLabel("Cancel");
	confirmModal->setMessage("No calibration file could be found for the Zed and the projector. Starting calibration process.");
	confirmModal->show();
	displayGui = sdisplayGui;

	//Check the size and location of the second window to fit the second screen
	//secondScreenFound = checkProjectorWindow();
	//if (!secondScreenFound){
	//    confirmModal->setMessage("Projector not found. Please check that the projector is (1) connected, (2) powerer and (3) not in mirror mode.");
	//    confirmModal->show();
	//}

	// calibration chessboard config
	chessboardSize = 300;
	chessboardX = 5;
	chessboardY = 4;

	// 	Gradient Field
	gradFieldResolution = 10;
	arrowLength = 25;

	// Setup default base plane
	basePlaneNormalBack = glm::vec3(0, 0, 1); // This is our default baseplane normal
	basePlaneOffsetBack = glm::vec3(0, 0, 870); // This is our default baseplane offset
	basePlaneNormal = basePlaneNormalBack;
	basePlaneOffset = basePlaneOffsetBack;
	basePlaneEq = getPlaneEquation(basePlaneOffset, basePlaneNormal);
	maxOffsetBack = basePlaneOffset.z - 300;
	maxOffset = maxOffsetBack;
	maxOffsetSafeRange = 50; // Range above the autocalib measured max offset

							 // zedGrabber: start & default setup
	ZedOpened = zedGrabber.setup();
	if (!ZedOpened) {
		confirmModal->setMessage("Cannot connect to Zed. Please check that the Zed is (1) connected, (2) powerer and (3) not used by another application.");
		confirmModal->show();
	}
	spatialFiltering = true;
	followBigChanges = false;
	numAveragingSlots = 15;

	// Get projector and Zed width & height
	projRes = glm::vec2(projWindow->getWidth(), projWindow->getHeight());
	zedRes = zedGrabber.getzedSize();
	zedROI = ofRectangle(0, 0, zedRes.x, zedRes.y);

	// Initialize the fbos and images
	FilteredDepthImage.allocate(zedRes.x, zedRes.y);
	ZedColorImage.allocate(zedRes.x, zedRes.y);
	thresholdedImage.allocate(zedRes.x, zedRes.y);
	Dptimg.allocate(20, 20); // Small detailed ROI

	kpt = new ofxZedProjectorToolkit(projRes, zedRes);

	//Try to load calibration file if possible
	if (kpt->loadCalibration("settings/calibration.xml"))
	{
		ofLogVerbose("ZedProjector") << "ZedProjector.setup(): Calibration loaded ";
		ZedProjMatrix = kpt->getProjectionMatrix();
		ofLogVerbose("ZedProjector") << "ZedProjector.setup(): ZedProjMatrix: " << ZedProjMatrix;
		projZedCalibrated = true;
	}
	else {
		if (displayGui) {
			// Show auto calibration modal window
			confirmModal->setMessage("No calibration file could be found for the Zed and the projector. Starting calibration process.");
			confirmModal->show();
		}
		ofLogVerbose("ZedProjector") << "ZedProjector.setup(): Calibration could not be loaded";
	}

	//Try to load settings file if possible
	if (loadSettings())
	{
		ofLogVerbose("ZedProjector") << "ZedProjector.setup(): Settings loaded ";
		ROIcalibrated = true;
	}
	else {
		ofLogVerbose("ZedProjector") << "ZedProjector.setup(): Settings could not be loaded ";
	}

	// finish zedGrabber setup and start the grabber
	zedGrabber.setupFramefilter(gradFieldResolution, maxOffset, zedROI, spatialFiltering, followBigChanges, numAveragingSlots);
	ZedWorldMatrix = zedGrabber.getWorldMatrix();
	ofLogVerbose("ZedProjector") << "ZedProjector.setup(): ZedWorldMatrix: " << ZedWorldMatrix;

	// Setup gradient field
	setupGradientField();

	fboProjWindow.allocate(projRes.x, projRes.y, GL_RGBA);
	fboProjWindow.begin();
	ofClear(255, 255, 255, 0);
	fboProjWindow.end();

	fboMainWindow.allocate(zedRes.x, zedRes.y, GL_RGBA);
	fboMainWindow.begin();
	ofClear(255, 255, 255, 0);
	fboMainWindow.end();

	if (displayGui)
		setupGui();

	zedGrabber.start(); // Start the acquisition
}

void ZedProjector::exit(ofEventArgs& e) {
	if (saveSettings())
	{
		ofLogVerbose("ZedProjector") << "exit(): Settings saved ";
	}
	else {
		ofLogVerbose("ZedProjector") << "exit(): Settings could not be saved ";
	}
}

void ZedProjector::setupGradientField() {
	gradFieldcols = zedRes.x / gradFieldResolution;
	gradFieldrows = zedRes.y / gradFieldResolution;

	gradField = new glm::vec2[gradFieldcols*gradFieldrows];
	glm::vec2* gfPtr = gradField;
	for (unsigned int y = 0; y<gradFieldrows; ++y)
		for (unsigned int x = 0; x<gradFieldcols; ++x, ++gfPtr)
			*gfPtr = glm::vec2(0);
}

void ZedProjector::setGradFieldResolution(int sgradFieldResolution) {
	gradFieldResolution = sgradFieldResolution;
	setupGradientField();
	zedGrabber.performInThread([sgradFieldResolution](ZedGrabber & kg) {
		kg.setGradFieldResolution(sgradFieldResolution);
	});
}

void ZedProjector::update() {
	// Clear updated state variables
	basePlaneUpdated = false;
	ROIUpdated = false;
	projZedCalibrationUpdated = false;

	if (displayGui)
		gui->update();

	// Get depth image from Zed grabber
	ofFloatPixels filteredframe;
	if (zedGrabber.filtered.tryReceive(filteredframe)) {
		FilteredDepthImage.setFromPixels(filteredframe.getData(), zedRes.x, zedRes.y);
		FilteredDepthImage.updateTexture();

		// Get color image from Zed grabber
		ofPixels coloredframe;
		if (zedGrabber.colored.tryReceive(coloredframe)) {
			ZedColorImage.setFromPixels(coloredframe);
		}

		// Get gradient field from Zed grabber
		zedGrabber.gradient.tryReceive(gradField);

		// Update grabber stored frame number
		zedGrabber.lock();
		zedGrabber.decStoredframes();
		zedGrabber.unlock();

		// Is the depth image stabilized
		imageStabilized = zedGrabber.isImageStabilized();

		// Are we calibrating ?
		if (calibrating && !waitingForFlattenSand) {
			updateCalibration();
		}
		else {
			//ofEnableAlphaBlending();
			fboMainWindow.begin();
			if (drawZedView) {
				FilteredDepthImage.draw(0, 0);
				ofNoFill();
				ofDrawRectangle(zedROI);
				ofDrawRectangle(0, 0, zedRes.x, zedRes.y);
			}
			else {
				ofClear(0, 0, 0, 0);
			}
			fboMainWindow.end();
		}
	}
}

void ZedProjector::updateCalibration() {
	if (calibrationState == CALIBRATION_STATE_FULL_AUTO_CALIBRATION) {
		updateFullAutoCalibration();
	}
	else if (calibrationState == CALIBRATION_STATE_ROI_AUTO_DETERMINATION) {
		updateROIAutoCalibration();
	}
	else if (calibrationState == CALIBRATION_STATE_ROI_MANUAL_DETERMINATION) {
		updateROIManualCalibration();
	}
	else if (calibrationState == CALIBRATION_STATE_PROJ_Zed_AUTO_CALIBRATION) {
		updateProjZedAutoCalibration();
	}
	else if (calibrationState == CALIBRATION_STATE_PROJ_Zed_MANUAL_CALIBRATION) {
		updateProjZedManualCalibration();
	}
}

void ZedProjector::updateFullAutoCalibration() {
	if (fullCalibState == FULL_CALIBRATION_STATE_ROI_DETERMINATION) {
		updateROIAutoCalibration();
		if (ROICalibState == ROI_CALIBRATION_STATE_DONE) {
			fullCalibState = FULL_CALIBRATION_STATE_AUTOCALIB;
			autoCalibState = AUTOCALIB_STATE_INIT_FIRST_PLANE;
		}
	}
	else if (fullCalibState == FULL_CALIBRATION_STATE_AUTOCALIB) {
		updateProjZedAutoCalibration();
		if (autoCalibState == AUTOCALIB_STATE_DONE) {
			fullCalibState = FULL_CALIBRATION_STATE_DONE;
		}
	}
}

void ZedProjector::updateROIAutoCalibration() {
	//updateROIFromColorImage();
	updateROIFromDepthImage();
}

void ZedProjector::updateROIFromCalibration() {
	ofVec2f a = worldCoordToZedCoord(projCoordAndWorldZToWorldCoord(0, 0, basePlaneOffset.z));
	ofVec2f b = worldCoordToZedCoord(projCoordAndWorldZToWorldCoord(projRes.x, 0, basePlaneOffset.z));
	ofVec2f c = worldCoordToZedCoord(projCoordAndWorldZToWorldCoord(projRes.x, projRes.y, basePlaneOffset.z));
	ofVec2f d = worldCoordToZedCoord(projCoordAndWorldZToWorldCoord(0, projRes.y, basePlaneOffset.z));
	float x1 = max(a.x, d.x);
	float x2 = min(b.x, c.x);
	float y1 = max(a.y, b.y);
	float y2 = min(c.y, d.y);
	ofRectangle smallZedROI = ofRectangle(ofGlmPoint(max(x1, zedROI.getLeft()), max(y1, zedROI.getTop()), 0), ofGlmPoint(min(x2, zedROI.getRight()), min(y2, zedROI.getBottom()), 0));
	zedROI = smallZedROI;

	zedROI.standardize();
	ofLogVerbose("ZedProjector") << "updateROIFromCalibration(): final ZedROI : " << zedROI;
	setNewZedROI();
}

//TODO: update color image ROI acquisition to use calibration modal
void ZedProjector::updateROIFromColorImage() {
	fboProjWindow.begin();
	ofBackground(255);
	fboProjWindow.end();
	if (ROICalibState == ROI_CALIBRATION_STATE_INIT) { // set Zed to max depth range
		ROICalibState = ROI_CALIBRATION_STATE_MOVE_UP;
		large = ofPolyline();
		threshold = 90;

	}
	else if (ROICalibState == ROI_CALIBRATION_STATE_MOVE_UP) {
		while (threshold < 255) {
			ZedColorImage.setROI(0, 0, zedRes.x, zedRes.y);
			thresholdedImage = ZedColorImage;
			cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), threshold, 255, CV_THRESH_BINARY_INV);
			contourFinder.findContours(thresholdedImage, 12, zedRes.x*zedRes.y, 5, true);
			ofPolyline small = ofPolyline();
			for (int i = 0; i < contourFinder.nBlobs; i++) {
				ofxCvBlob blobContour = contourFinder.blobs[i];
				if (blobContour.hole) {
					ofPolyline poly = ofPolyline(blobContour.ptsGlm);
					if (poly.inside(zedRes.x / 2, zedRes.y / 2))
					{
						if (small.size() == 0 || poly.getArea() < small.getArea()) {
							small = poly;
						}
					}
				}
			}
			ofLogVerbose("ZedProjector") << "ZedProjector.updateROIFromColorImage(): small.getArea(): " << small.getArea();
			ofLogVerbose("ZedProjector") << "ZedProjector.updateROIFromColorImage(): large.getArea(): " << large.getArea();
			if (large.getArea() < small.getArea())
			{
				ofLogVerbose("ZedProjector") << "updateROIFromColorImage(): We take the largest contour line surroundings the center of the screen at all threshold level";
				large = small;
			}
			threshold += 1;
		}
		zedROI = large.getBoundingBox();
		zedROI.standardize();
		ofLogVerbose("ZedProjector") << "updateROIFromColorImage(): zedROI : " << zedROI;
		ROICalibState = ROI_CALIBRATION_STATE_DONE;
		setNewZedROI();
	}
	else if (ROICalibState == ROI_CALIBRATION_STATE_DONE) {
	}
}

void ZedProjector::updateROIFromDepthImage() {
	if (ROICalibState == ROI_CALIBRATION_STATE_INIT) {
		calibModal->setMessage("Enlarging acquisition area & resetting buffers.");
		setMaxZedGrabberROI();
		calibModal->setMessage("Stabilizing acquisition.");
		ROICalibState = ROI_CALIBRATION_STATE_READY_TO_MOVE_UP;
	}
	else if (ROICalibState == ROI_CALIBRATION_STATE_READY_TO_MOVE_UP && imageStabilized) {
		calibModal->setMessage("Scanning depth field to find sandbox walls.");
		ofLogVerbose("ZedProjector") << "updateROIFromDepthImage(): ROI_CALIBRATION_STATE_READY_TO_MOVE_UP: got a stable depth image";
		ROICalibState = ROI_CALIBRATION_STATE_MOVE_UP;
		large = ofPolyline();
		ofxCvFloatImage temp;
		temp.setFromPixels(FilteredDepthImage.getFloatPixelsRef().getData(), zedRes.x, zedRes.y);
		temp.setNativeScale(FilteredDepthImage.getNativeScaleMin(), FilteredDepthImage.getNativeScaleMax());
		temp.convertToRange(0, 1);
		thresholdedImage.setFromPixels(temp.getFloatPixelsRef());
		threshold = 0; // We go from the higher distance to the Zed (lower position) to the lower distance
	}
	else if (ROICalibState == ROI_CALIBRATION_STATE_MOVE_UP) {
		while (threshold < 255) {
			cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), 255 - threshold, 255, CV_THRESH_TOZERO_INV);
			thresholdedImage.updateTexture();
			contourFinder.findContours(thresholdedImage, 12, zedRes.x*zedRes.y, 5, true, false);
			ofPolyline small = ofPolyline();
			for (int i = 0; i < contourFinder.nBlobs; i++) {
				ofxCvBlob blobContour = contourFinder.blobs[i];
				if (blobContour.hole) {
					ofPolyline poly = ofPolyline(blobContour.ptsGlm);
					if (poly.inside(zedRes.x / 2, zedRes.y / 2))
					{
						if (small.size() == 0 || poly.getArea() < small.getArea()) {
							small = poly;
						}
					}
				}
			}
			if (large.getArea() < small.getArea())
			{
				ofLogVerbose("ZedProjector") << "updateROIFromDepthImage(): updating ROI";
				large = small;
			}
			threshold += 1;
		}
		if (large.getArea() == 0)
		{
			calibModal->hide();
			confirmModal->setTitle("Calibration failed");
			confirmModal->setMessage("The sandbox walls could not be found.");
			confirmModal->show();
			calibrating = false;
		}
		else {
			zedROI = large.getBoundingBox();
			//            insideROIPoly = large.getResampledBySpacing(10);
			zedROI.standardize();
			calibModal->setMessage("Sand area successfully detected");
			ofLogVerbose("ZedProjector") << "updateROIFromDepthImage(): final zedROI : " << zedROI;
			setNewZedROI();
			if (calibrationState == CALIBRATION_STATE_ROI_AUTO_DETERMINATION) {
				calibrating = false;
				calibModal->hide();
			}
		}
		ROICalibState = ROI_CALIBRATION_STATE_DONE;
	}
	else if (ROICalibState == ROI_CALIBRATION_STATE_DONE) {
	}
}
//TODO: Add manual ROI calibration
void ZedProjector::updateROIManualCalibration() {
	//    fboProjWindow.begin();
	//    ofBackground(255);
	//    fboProjWindow.end();
	//    
	//    if (ROICalibState == ROI_CALIBRATION_STATE_MOVE_UP) {
	//        ZedROIManualCalib.setSize(ofGetMouseX()-ZedROIManualCalib.x,ofGetMouseY()-ZedROIManualCalib.y);
	//    }
	//    
	//    if (ROICalibState == ROI_CALIBRATION_STATE_INIT) {
	//        resultMessage = "Please click on first ROI corner";
	//    } else if (ROICalibState == ROI_CALIBRATION_STATE_MOVE_UP) {
	//        resultMessage = "Please click on second ROI corner";
	//    } else if (ROICalibState == ROI_CALIBRATION_STATE_DONE){
	//        resultMessage = "Manual ROI update done";
	//    }
}

void ZedProjector::setMaxZedGrabberROI() {
	updateZedGrabberROI(ofRectangle(0, 0, zedRes.x, zedRes.y));
}

void ZedProjector::setNewZedROI() {
	// Cast to integer values
	zedROI.x = static_cast<int>(zedROI.x);
	zedROI.y = static_cast<int>(zedROI.y);
	zedROI.width = static_cast<int>(zedROI.width);
	zedROI.height = static_cast<int>(zedROI.height);

	// Update states variables
	ROIcalibrated = true;
	ROIUpdated = true;
	saveCalibrationAndSettings();
	updateZedGrabberROI(zedROI);
}

void ZedProjector::updateZedGrabberROI(ofRectangle ROI) {
	zedGrabber.performInThread([ROI](ZedGrabber & kg) {
		kg.setzedROI(ROI);
	});
	//    while (zedGrabber.isImageStabilized()){
	//    } // Wait for zedGrabber to reset buffers
	imageStabilized = false; // Now we can wait for a clean new depth frame
}

void ZedProjector::updateProjZedAutoCalibration() {
	if (autoCalibState == AUTOCALIB_STATE_INIT_FIRST_PLANE) {
		if (!ROIcalibrated) {
			updateROIAutoCalibration();
		}
		else {
			calibModal->setMessage("Enlarging acquisition area & resetting buffers.");
			setMaxZedGrabberROI();
			zedGrabber.performInThread([](ZedGrabber & kg) {
				kg.setMaxOffset(0);
			});
			calibModal->setMessage("Stabilizing acquisition.");
			autoCalibState = AUTOCALIB_STATE_INIT_POINT;
		}
	}
	else if (autoCalibState == AUTOCALIB_STATE_INIT_POINT && imageStabilized) {
		calibModal->setMessage("Acquiring sea level plane.");
		updateBasePlane(); // Find base plane
		autoCalibPts = new ofGlmPoint[10];
		float cs = 2 * chessboardSize / 3;
		float css = 3 * chessboardSize / 4;
		ofPoint sc = ofPoint(projRes.x / 2, projRes.y / 2);

		// Prepare 10 locations for the calibration chessboard
		autoCalibPts[0] = ofPoint(cs, cs) - sc;
		autoCalibPts[1] = ofPoint(projRes.x - cs, cs) - sc;
		autoCalibPts[2] = ofPoint(projRes.x - cs, projRes.y - cs) - sc;
		autoCalibPts[3] = ofPoint(cs, projRes.y - cs) - sc;
		autoCalibPts[4] = ofPoint(projRes.x / 2 + cs, projRes.y / 2) - sc;
		autoCalibPts[5] = ofPoint(css, css) - sc;
		autoCalibPts[6] = ofPoint(projRes.x - css, css) - sc;
		autoCalibPts[7] = ofPoint(projRes.x - css, projRes.y - css) - sc;
		autoCalibPts[8] = ofPoint(css, projRes.y - css) - sc;
		autoCalibPts[9] = ofPoint(projRes.x / 2 - cs, projRes.y / 2) - sc;
		currentCalibPts = 0;
		cleared = false;
		upframe = false;
		trials = 0;
		autoCalibState = AUTOCALIB_STATE_NEXT_POINT;
	}
	else if (autoCalibState == AUTOCALIB_STATE_NEXT_POINT && imageStabilized) {
		if (currentCalibPts < 5 || (upframe && currentCalibPts < 10)) {
			if (!upframe) {
				string mess = "Acquiring low level calibration point " + std::to_string(currentCalibPts + 1) + "/5.";
				calibModal->setMessage(mess);
			}
			else {
				string mess = "Acquiring high level calibration point " + std::to_string(currentCalibPts - 4) + "/5.";
				calibModal->setMessage(mess);
			}

			cvRgbImage = ofxCv::toCv(ZedColorImage.getPixels());
			cv::Size patternSize = cv::Size(chessboardX - 1, chessboardY - 1);
			int chessFlags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK;
			bool foundChessboard = findChessboardCorners(cvRgbImage, patternSize, cvPoints, chessFlags);
			if (foundChessboard) {
				if (cleared) { // We have previously detected a cleared screen <- Be sure that we don't acquire several times the same chessboard
					cv::Mat gray;
					cvtColor(cvRgbImage, gray, CV_RGB2GRAY);
					cornerSubPix(gray, cvPoints, cv::Size(11, 11), cv::Size(-1, -1),
						cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

					drawChessboardCorners(cvRgbImage, patternSize, cv::Mat(cvPoints), foundChessboard);
					ZedColorImage.updateTexture();
					fboMainWindow.begin();
					ZedColorImage.draw(0, 0);
					fboMainWindow.end();

					ofLogVerbose("ZedProjector") << "autoCalib(): Chessboard found for point :" << currentCalibPts;
					bool okchess = addPointPair();

					if (okchess) {
						fboProjWindow.begin(); // Clear projector
						ofBackground(255);
						fboProjWindow.end();
						cleared = false;
						trials = 0;
						currentCalibPts++;
					}
					else {
						// We cannot get all depth points for the chessboard
						trials++;
						ofLogVerbose("ZedProjector") << "autoCalib(): Depth points of chessboard not allfound on trial : " << trials;
						if (trials >10) {
							// Move the chessboard closer to the center of the screen
							ofLogVerbose("ZedProjector") << "autoCalib(): Chessboard could not be found moving chessboard closer to center ";
							autoCalibPts[currentCalibPts] = 4 * autoCalibPts[currentCalibPts] / 5;
							fboProjWindow.begin(); // Clear projector
							ofBackground(255);
							fboProjWindow.end();
							cleared = false;
							trials = 0;
						}
					}
				}
			}
			else {
				if (cleared == false) {
					ofLogVerbose("ZedProjector") << "autoCalib(): Clear screen found, drawing next chessboard";
					cleared = true; // The cleared fbo screen was seen by the Zed
					ofPoint dispPt = ofPoint(projRes.x / 2, projRes.y / 2) + autoCalibPts[currentCalibPts]; // Compute next chessboard position
					drawChessboard(dispPt.x, dispPt.y, chessboardSize); // We can now draw the next chess board
				}
				else {
					// We cannot find the chessboard
					trials++;
					ofLogVerbose("ZedProjector") << "autoCalib(): Chessboard not found on trial : " << trials;
					if (trials >10) {
						// Move the chessboard closer to the center of the screen
						ofLogVerbose("ZedProjector") << "autoCalib(): Chessboard could not be found moving chessboard closer to center ";
						autoCalibPts[currentCalibPts] = 3 * autoCalibPts[currentCalibPts] / 4;
						fboProjWindow.begin(); // Clear projector
						ofBackground(255);
						fboProjWindow.end();
						cleared = false;
						trials = 0;
					}
				}
			}
		}
		else {
			if (upframe) { // We are done
				calibModal->setMessage("Updating acquision ceiling.");
				updateMaxOffset(); // Find max offset
				autoCalibState = AUTOCALIB_STATE_COMPUTE;
			}
			else { // We ask for higher points
				calibModal->hide();
				confirmModal->show();
				confirmModal->setMessage("Please cover the sandbox with a board and press ok.");
			}
		}
	}
	else if (autoCalibState == AUTOCALIB_STATE_COMPUTE) {
		updateZedGrabberROI(zedROI); // Goes back to ZedROI and maxoffset
		zedGrabber.performInThread([this](ZedGrabber & kg) {
			kg.setMaxOffset(this->maxOffset);
		});
		if (pairsZed.size() == 0) {
			ofLogVerbose("ZedProjector") << "autoCalib(): Error: No points acquired !!";
			calibModal->hide();
			confirmModal->setTitle("Calibration failed");
			confirmModal->setMessage("No point could be acquired. ");
			confirmModal->show();
			calibrating = false;
		}
		else {
			ofLogVerbose("ZedProjector") << "autoCalib(): Calibrating";
			kpt->calibrate(pairsZed, pairsProjector);
			ZedProjMatrix = kpt->getProjectionMatrix();

			updateROIFromCalibration(); // Compute the limite of the ROI according to the projected area 

			projZedCalibrated = true; // Update states variables
			projZedCalibrationUpdated = true;
			calibrating = false;
			calibModal->setMessage("Calibration successfull.");
			calibModal->hide();
			//saveCalibrationAndSettings(); // Already done in updateROIFromCalibration
		}
		autoCalibState = AUTOCALIB_STATE_DONE;
	}
	else if (autoCalibState == AUTOCALIB_STATE_DONE) {
	}
}
//TODO: Add manual Prj Zed calibration
void ZedProjector::updateProjZedManualCalibration() {
	// Draw a Chessboard
	drawChessboard(ofGetMouseX(), ofGetMouseY(), chessboardSize);
	// Try to find the chess board on the Zed color image
	cvRgbImage = ofxCv::toCv(ZedColorImage.getPixels());
	cv::Size patternSize = cv::Size(chessboardX - 1, chessboardY - 1);
	int chessFlags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK;
	bool foundChessboard = findChessboardCorners(cvRgbImage, patternSize, cvPoints, chessFlags);
	if (foundChessboard) {
		cv::Mat gray;
		cvtColor(cvRgbImage, gray, CV_RGB2GRAY);
		cornerSubPix(gray, cvPoints, cv::Size(11, 11), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		drawChessboardCorners(cvRgbImage, patternSize, cv::Mat(cvPoints), foundChessboard);
	}
}

void ZedProjector::updateBasePlane() {
	ofRectangle smallROI = zedROI;
	smallROI.scaleFromCenter(0.75); // Reduce ROI to avoid problems with borders
	ofLogVerbose("ZedProjector") << "updateBasePlane(): smallROI: " << smallROI;
	int sw = static_cast<int>(smallROI.width);
	int sh = static_cast<int>(smallROI.height);
	int sl = static_cast<int>(smallROI.getLeft());
	int st = static_cast<int>(smallROI.getTop());
	ofLogVerbose("ZedProjector") << "updateBasePlane(): sw: " << sw << " sh : " << sh << " sl : " << sl << " st : " << st << " sw*sh : " << sw*sh;
	if (sw*sh == 0) {
		ofLogVerbose("ZedProjector") << "updateBasePlane(): smallROI is null, cannot compute base plane normal";
		return;
	}
	glm::vec4 pt;
	glm::vec3* points;
	points = new glm::vec3[sw*sh];
	ofLogVerbose("ZedProjector") << "updateBasePlane(): Computing points in smallROI : " << sw*sh;
	for (int x = 0; x<sw; x++) {
		for (int y = 0; y<sh; y++) {
			points[x + y*sw] = ZedCoordToWorldCoord(x + sl, y + st);
		}
	}
	ofLogVerbose("ZedProjector") << "updateBasePlane(): Computing plane from points";
	basePlaneEq = plane_from_points(points, sw*sh);
	basePlaneNormal = glm::vec3(basePlaneEq);
	basePlaneOffset = glm::vec3(0, 0, -basePlaneEq.w);
	basePlaneNormalBack = basePlaneNormal;
	basePlaneOffsetBack = basePlaneOffset;
	basePlaneUpdated = true;
}

void ZedProjector::updateMaxOffset() {
	ofRectangle smallROI = zedROI;
	smallROI.scaleFromCenter(0.75); // Reduce ROI to avoid problems with borders
	ofLogVerbose("ZedProjector") << "updateMaxOffset(): smallROI: " << smallROI;
	int sw = static_cast<int>(smallROI.width);
	int sh = static_cast<int>(smallROI.height);
	int sl = static_cast<int>(smallROI.getLeft());
	int st = static_cast<int>(smallROI.getTop());
	ofLogVerbose("ZedProjector") << "updateMaxOffset(): sw: " << sw << " sh : " << sh << " sl : " << sl << " st : " << st << " sw*sh : " << sw*sh;
	if (sw*sh == 0) {
		ofLogVerbose("ZedProjector") << "updateMaxOffset(): smallROI is null, cannot compute base plane normal";
		return;
	}
	glm::vec4 pt;
	glm::vec3* points;
	points = new glm::vec3[sw*sh];
	ofLogVerbose("ZedProjector") << "updateMaxOffset(): Computing points in smallROI : " << sw*sh;
	for (int x = 0; x<sw; x++) {
		for (int y = 0; y<sh; y++) {
			points[x + y*sw] = ZedCoordToWorldCoord(x + sl, y + st);//vertexCcvertexCc;
		}
	}
	ofLogVerbose("ZedProjector") << "updateMaxOffset(): Computing plane from points";
	glm::vec4 eqoff = plane_from_points(points, sw*sh);
	maxOffset = -eqoff.w - maxOffsetSafeRange;
	maxOffsetBack = maxOffset;
	// Update max Offset
	ofLogVerbose("ZedProjector") << "updateMaxOffset(): maxOffset" << maxOffset;
	zedGrabber.performInThread([this](ZedGrabber & kg) {
		kg.setMaxOffset(this->maxOffset);
	});
}

bool ZedProjector::addPointPair() {
	bool okchess = true;
	string resultMessage;
	ofLogVerbose("ZedProjector") << "addPointPair(): Adding point pair in Zed world coordinates";
	int nDepthPoints = 0;
	for (int i = 0; i<cvPoints.size(); i++) {
		glm::vec3 worldPoint = ZedCoordToWorldCoord(cvPoints[i].x, cvPoints[i].y);
		if (worldPoint.z > 0)   nDepthPoints++;
	}
	if (nDepthPoints == (chessboardX - 1)*(chessboardY - 1)) {
		for (int i = 0; i<cvPoints.size(); i++) {
			glm::vec3 worldPoint = ZedCoordToWorldCoord(cvPoints[i].x, cvPoints[i].y);
			//            cout << "Zed: " << worldPoint << "Proj: " << currentProjectorPoints[i] << endl;
			pairsZed.push_back(worldPoint);
			pairsProjector.push_back(currentProjectorPoints[i]);
		}
		resultMessage = "addPointPair(): Added " + ofToString((chessboardX - 1)*(chessboardY - 1)) + " points pairs.";
	}
	else {
		resultMessage = "addPointPair(): Points not added because not all chessboard\npoints' depth known. Try re-positionining.";
		okchess = false;
	}
	ofLogVerbose("ZedProjector") << resultMessage;
	return okchess;
}

void ZedProjector::askToFlattenSand() {
	fboProjWindow.begin();
	ofBackground(255);
	fboProjWindow.end();
	confirmModal->setMessage("Please flatten the sand surface.");
	confirmModal->show();
	waitingForFlattenSand = true;
}

void ZedProjector::drawProjectorWindow() {
	fboProjWindow.draw(0, 0);
}

void ZedProjector::drawMainWindow(float x, float y, float width, float height) {
	fboMainWindow.draw(x, y, width, height);
	if (displayGui)
		gui->draw();
}

void ZedProjector::drawChessboard(int x, int y, int chessboardSize) {
	fboProjWindow.begin();
	ofFill();
	// Draw the calibration chess board on the projector window
	float w = chessboardSize / chessboardX;
	float h = chessboardSize / chessboardY;

	float xf = x - chessboardSize / 2; // x and y are chess board center size
	float yf = y - chessboardSize / 2;

	currentProjectorPoints.clear();

	ofClear(255, 0);
	ofSetColor(0);
	ofTranslate(xf, yf);
	for (int j = 0; j<chessboardY; j++) {
		for (int i = 0; i<chessboardX; i++) {
			int x0 = ofMap(i, 0, chessboardX, 0, chessboardSize);
			int y0 = ofMap(j, 0, chessboardY, 0, chessboardSize);
			if (j>0 && i>0) {
				currentProjectorPoints.push_back(glm::vec2(xf + x0, yf + y0));
			}
			if ((i + j) % 2 == 0) ofDrawRectangle(x0, y0, w, h);
		}
	}
	ofSetColor(255);
	fboProjWindow.end();
}

void ZedProjector::drawGradField()
{
	ofClear(255, 0);
	for (int rowPos = 0; rowPos< gradFieldrows; rowPos++)
	{
		for (int colPos = 0; colPos< gradFieldcols; colPos++)
		{
			float x = colPos*gradFieldResolution + gradFieldResolution / 2;
			float y = rowPos*gradFieldResolution + gradFieldResolution / 2;
			glm::vec2 projectedPoint = zedCoordToProjCoord(x, y);
			int ind = colPos + rowPos * gradFieldcols;
			glm::vec2 v2 = gradField[ind];
			v2 *= arrowLength;

			ofSetColor(255, 0, 0, 255);
			if (ind == fishInd)
				ofSetColor(0, 255, 0, 255);

			drawArrow(projectedPoint, v2);
		}
	}
}

void ZedProjector::drawArrow(glm::vec2 projectedPoint, glm::vec2 v1)
{
	float angle = ofRadToDeg(atan2(v1.y, v1.x));
	float length = v1.length();
	ofFill();
	ofPushMatrix();
	ofTranslate(projectedPoint);
	ofRotate(angle);
	ofSetColor(255, 0, 0, 255);
	ofDrawLine(0, 0, length, 0);
	ofDrawLine(length, 0, length - 7, 5);
	ofDrawLine(length, 0, length - 7, -5);
	ofPopMatrix();
}

void ZedProjector::updateNativeScale(float scaleMin, float scaleMax) {
	FilteredDepthImage.setNativeScale(scaleMin, scaleMax);
}

glm::vec2 ZedProjector::zedCoordToProjCoord(float x, float y) // x, y in Zed pixel coord
{
	return worldCoordToProjCoord(ZedCoordToWorldCoord(x, y));
}

glm::vec2 ZedProjector::worldCoordToProjCoord(glm::vec3 vin)
{
	glm::vec4 wc = glm::vec4(vin, 0);
	wc.w = 1;
	glm::vec4 screenPos = ZedProjMatrix*wc;
	glm::vec2 projectedPoint(screenPos.x / screenPos.z, screenPos.y / screenPos.z);
	return projectedPoint;
}

glm::vec3 ZedProjector::projCoordAndWorldZToWorldCoord(float projX, float projY, float worldZ)
{
	float a = ZedProjMatrix[0][0] - ZedProjMatrix[2][0] * projX;
	float b = ZedProjMatrix[0][1] - ZedProjMatrix[2][1] * projX;
	float c = (ZedProjMatrix[2][2] * worldZ + 1)*projX - (ZedProjMatrix[0][2] * worldZ + ZedProjMatrix[0][3]);
	float d = ZedProjMatrix[1][0] - ZedProjMatrix[2][0] * projY;
	float e = ZedProjMatrix[1][1] - ZedProjMatrix[2][1] * projY;
	float f = (ZedProjMatrix[2][2] * worldZ + 1)*projY - (ZedProjMatrix[1][2] * worldZ + ZedProjMatrix[1][3]);

	float det = a*e - b*d;
	if (det == 0)
		return glm::vec3(0);
	float y = (a*f - d*c) / det;
	float x = (c*e - b*f) / det;
	return glm::vec3(x, y, worldZ);
}

glm::vec3 ZedProjector::ZedCoordToWorldCoord(float x, float y) // x, y in Zed pixel coord
{
	glm::vec4 kc = glm::vec4(x, y, 0, 0);
	int ind = static_cast<int>(y) * zedRes.x + static_cast<int>(x);
	kc.z = FilteredDepthImage.getFloatPixelsRef().getData()[ind];
	kc.w = 1;
	glm::vec4 wc = ZedWorldMatrix*kc*kc.z;
	return glm::vec3(wc);
}

glm::vec2 ZedProjector::worldCoordToZedCoord(glm::vec3 wc)
{
	float x = (wc.x / wc.z - ZedWorldMatrix[0][3]) / ZedWorldMatrix[0][0];
	float y = (wc.y / wc.z - ZedWorldMatrix[1][3]) / ZedWorldMatrix[1][1];
	return glm::vec2(x, y);
}

glm::vec3 ZedProjector::RawZedCoordToWorldCoord(float x, float y) // x, y in Zed pixel coord
{
	glm::vec4 kc = glm::vec4(x, y, zedGrabber.getRawDepthAt(static_cast<int>(x), static_cast<int>(y)), 0);
	kc.w = 1;
	glm::vec4 wc = ZedWorldMatrix*kc*kc.z;
	return glm::vec3(wc);
}

float ZedProjector::elevationAtZedCoord(float x, float y) // x, y in Zed pixel coordinate
{
	glm::vec4 wc = glm::vec4(ZedCoordToWorldCoord(x, y), 0);
	wc.w = 1;
	float elevation = -glm::dot(basePlaneEq, wc);
	return elevation;
}

float ZedProjector::elevationToZedDepth(float elevation, float x, float y) // x, y in Zed pixel coordinate
{
	glm::vec4 wc = glm::vec4(ZedCoordToWorldCoord(x, y), 0);
	wc.z = 0;
	wc.w = 1;
	float ZedDepth = -(glm::dot(basePlaneEq, wc) + elevation) / basePlaneEq.z;
	return ZedDepth;
}

glm::vec2 ZedProjector::gradientAtZedCoord(float x, float y) {
	int ind = static_cast<int>(floor(x / gradFieldResolution)) + gradFieldcols*static_cast<int>(floor(y / gradFieldResolution));
	fishInd = ind;
	return gradField[ind];
}

void ZedProjector::setupGui() {
	// instantiate and position the gui //
	gui = new ofxDatGui(ofxDatGuiAnchor::TOP_RIGHT);
	gui->addFRM();
	gui->addBreak();
	gui->addSlider("Tilt X", -30, 30, 0);
	gui->addSlider("Tilt Y", -30, 30, 0);
	gui->addSlider("Vertical offset", -100, 100, 0);
	gui->addButton("Reset sea level");
	gui->addBreak();

	auto advancedFolder = gui->addFolder("Advanced", ofColor::purple);
	advancedFolder->addToggle("Display Zed depth view", drawZedView)->setName("Draw Zed depth view");
	advancedFolder->addSlider("Ceiling", -300, 300, 0);
	advancedFolder->addToggle("Spatial filtering", spatialFiltering);
	advancedFolder->addToggle("Quick reaction", followBigChanges);
	advancedFolder->addSlider("Averaging", 1, 40, numAveragingSlots)->setPrecision(0);
	advancedFolder->addBreak();
	advancedFolder->addButton("Calibrate")->setName("Full Calibration");
	//	advancedFolder->addButton("Update ROI from calibration");
	//    gui->addButton("Automatically detect sand region");
	//    calibrationFolder->addButton("Manually define sand region");
	//    gui->addButton("Automatically calibrate Zed & projector");
	//    calibrationFolder->addButton("Manually calibrate Zed & projector");

	//    gui->addBreak();
	gui->addHeader(":: Settings ::", false);

	// once the gui has been assembled, register callbacks to listen for component specific events //
	gui->onButtonEvent(this, &ZedProjector::onButtonEvent);
	gui->onToggleEvent(this, &ZedProjector::onToggleEvent);
	gui->onSliderEvent(this, &ZedProjector::onSliderEvent);

	// disactivate autodraw
	gui->setAutoDraw(false);
}

void ZedProjector::startFullCalibration() {
	calibrating = true;
	calibrationState = CALIBRATION_STATE_FULL_AUTO_CALIBRATION;
	fullCalibState = FULL_CALIBRATION_STATE_ROI_DETERMINATION;
	ROICalibState = ROI_CALIBRATION_STATE_INIT;
	confirmModal->setTitle("Full calibration");
	calibModal->setTitle("Full calibration");
	askToFlattenSand();
	ofLogVerbose("ZedProjector") << "startFullCalibration(): Starting full calibration";
}

void ZedProjector::startAutomaticROIDetection() {
	calibrating = true;
	calibrationState = CALIBRATION_STATE_ROI_AUTO_DETERMINATION;
	ROICalibState = ROI_CALIBRATION_STATE_INIT;
	ofLogVerbose("ZedProjector") << "onButtonEvent(): Finding ROI";
	confirmModal->setTitle("Detect sand region");
	calibModal->setTitle("Detect sand region");
	askToFlattenSand();
	ofLogVerbose("ZedProjector") << "startAutomaticROIDetection(): starting ROI detection";
}

void ZedProjector::startAutomaticZedProjectorCalibration() {
	calibrating = true;
	calibrationState = CALIBRATION_STATE_PROJ_Zed_AUTO_CALIBRATION;
	autoCalibState = AUTOCALIB_STATE_INIT_POINT;
	confirmModal->setTitle("Calibrate projector");
	calibModal->setTitle("Calibrate projector");
	askToFlattenSand();
	ofLogVerbose("ZedProjector") << "startAutomaticZedProjectorCalibration(): Starting autocalib";
}

void ZedProjector::setSpatialFiltering(bool sspatialFiltering) {
	spatialFiltering = sspatialFiltering;
	zedGrabber.performInThread([sspatialFiltering](ZedGrabber & kg) {
		kg.setSpatialFiltering(sspatialFiltering);
	});
}

void ZedProjector::setFollowBigChanges(bool sfollowBigChanges) {
	followBigChanges = sfollowBigChanges;
	zedGrabber.performInThread([sfollowBigChanges](ZedGrabber & kg) {
		kg.setFollowBigChange(sfollowBigChanges);
	});
}

void ZedProjector::onButtonEvent(ofxDatGuiButtonEvent e) {
	if (e.target->is("Full Calibration")) {
		startFullCalibration();
	}
	else if (e.target->is("Update ROI from calibration")) {
		updateROIFromCalibration();
	}
	else if (e.target->is("Automatically detect sand region")) {
		startAutomaticROIDetection();
	}
	else if (e.target->is("Manually define sand region")) {
		// Not implemented yet
	}
	else if (e.target->is("Automatically calibrate Zed & projector")) {
		startAutomaticZedProjectorCalibration();
	}
	else if (e.target->is("Manually calibrate Zed & projector")) {
		// Not implemented yet
	}
	else if (e.target->is("Reset sea level")) {
		gui->getSlider("Tilt X")->setValue(0);
		gui->getSlider("Tilt Y")->setValue(0);
		gui->getSlider("Vertical offset")->setValue(0);
		basePlaneNormal = basePlaneNormalBack;
		basePlaneOffset = basePlaneOffsetBack;
		basePlaneEq = getPlaneEquation(basePlaneOffset, basePlaneNormal);
		basePlaneUpdated = true;
	}
}

void ZedProjector::onToggleEvent(ofxDatGuiToggleEvent e) {
	if (e.target->is("Spatial filtering")) {
		setSpatialFiltering(e.checked);
	}
	else if (e.target->is("Quick reaction")) {
		setFollowBigChanges(e.checked);
	}
	else if (e.target->is("Draw Zed depth view")) {
		drawZedView = e.checked;
	}
}

void ZedProjector::onSliderEvent(ofxDatGuiSliderEvent e) {
	if (e.target->is("Tilt X") || e.target->is("Tilt Y")) {
		basePlaneNormal = glm::rotate(basePlaneNormalBack, (glm::mediump_float)gui->getSlider("Tilt X")->getValue(), glm::vec3(1, 0, 0));
		glm::rotate(basePlaneNormal, (glm::mediump_float)gui->getSlider("Tilt Y")->getValue(), glm::vec3(0, 1, 0));
		basePlaneEq = getPlaneEquation(basePlaneOffset, basePlaneNormal);
		basePlaneUpdated = true;
	}
	else if (e.target->is("Vertical offset")) {
		basePlaneOffset.z = basePlaneOffsetBack.z + e.value;
		basePlaneEq = getPlaneEquation(basePlaneOffset, basePlaneNormal);
		basePlaneUpdated = true;
	}
	else if (e.target->is("Ceiling")) {
		maxOffset = maxOffsetBack - e.value;
		ofLogVerbose("ZedProjector") << "onSliderEvent(): maxOffset" << maxOffset;
		zedGrabber.performInThread([this](ZedGrabber & kg) {
			kg.setMaxOffset(this->maxOffset);
		});
	}
	else if (e.target->is("Averaging")) {
		numAveragingSlots = e.value;
		zedGrabber.performInThread([e](ZedGrabber & kg) {
			kg.setAveragingSlotsNumber(e.value);
		});
	}
}

void ZedProjector::onConfirmModalEvent(ofxModalEvent e) {
	if (e.type == ofxModalEvent::SHOWN) {
		ofLogVerbose("ZedProjector") << "Confirm modal window is open";
	}
	else if (e.type == ofxModalEvent::HIDDEN) {
		if (!projZedCalibrated && !calibrating)
			startFullCalibration();
		if (calibrating)
			calibModal->show();
		if (!ZedOpened) {
			confirmModal->setMessage("Still no connection to Zed. Please check that the Zed is (1) connected, (2) powerer and (3) not used by another application.");
			confirmModal->show();
		}
		ofLogVerbose("ZedProjector") << "Confirm modal window is closed";
	}
	else if (e.type == ofxModalEvent::CANCEL) {
		calibrating = false;
		ZedOpened = true; // The user don't care...
		ofLogVerbose("ZedProjector") << "Modal cancel button pressed: Aborting";
	}
	else if (e.type == ofxModalEvent::CONFIRM) {
		if (calibrating) {
			if (waitingForFlattenSand) {
				waitingForFlattenSand = false;
			}
			else if ((calibrationState == CALIBRATION_STATE_PROJ_Zed_AUTO_CALIBRATION || (calibrationState == CALIBRATION_STATE_FULL_AUTO_CALIBRATION && fullCalibState == FULL_CALIBRATION_STATE_AUTOCALIB))
				&& autoCalibState == AUTOCALIB_STATE_NEXT_POINT) {
				if (!upframe) {
					upframe = true;
				}
			}
		}
		if (!ZedOpened) {
			ZedOpened = zedGrabber.openZed();
		}
		ofLogVerbose("ZedProjector") << "Modal confirm button pressed";
	}
}

void ZedProjector::onCalibModalEvent(ofxModalEvent e) {
	if (e.type == ofxModalEvent::SHOWN) {
		cout << "calib modal window is open" << endl;
	}
	else if (e.type == ofxModalEvent::HIDDEN) {
		cout << "calib modal window is closed" << endl;
	}
	else if (e.type == ofxModalEvent::CONFIRM) {
		calibrating = false;
		ofLogVerbose("ZedProjector") << "Modal cancel button pressed: Aborting";
	}
}

void ZedProjector::saveCalibrationAndSettings() {
	if (kpt->saveCalibration("settings/calibration.xml"))
	{
		ofLogVerbose("ZedProjector") << "update(): initialisation: Calibration saved ";
	}
	else {
		ofLogVerbose("ZedProjector") << "update(): initialisation: Calibration could not be saved ";
	}
	if (saveSettings())
	{
		ofLogVerbose("ZedProjector") << "update(): initialisation: Settings saved ";
	}
	else {
		ofLogVerbose("ZedProjector") << "update(): initialisation: Settings could not be saved ";
	}
}

bool ZedProjector::loadSettings() {
	string settingsFile = "settings/ZedProjectorSettings.xml";
	ofxXmlPoco xml;
	if (!xml.load(settingsFile))
		return false;
	xml.setTo("ZedSETTINGS");
	zedROI = xml.getValue<ofRectangle>("ZedROI");
	basePlaneNormalBack = xml.getValue<glm::vec3>("basePlaneNormalBack");
	basePlaneNormal = basePlaneNormalBack;
	basePlaneOffsetBack = xml.getValue<glm::vec3>("basePlaneOffsetBack");
	basePlaneOffset = basePlaneOffsetBack;
	basePlaneEq = xml.getValue<glm::vec4>("basePlaneEq");
	maxOffsetBack = xml.getValue<float>("maxOffsetBack");
	maxOffset = maxOffsetBack;
	spatialFiltering = xml.getValue<bool>("spatialFiltering");
	followBigChanges = xml.getValue<bool>("followBigChanges");
	numAveragingSlots = xml.getValue<int>("numAveragingSlots");
	return true;
}

bool ZedProjector::saveSettings() {
	string settingsFile = "settings/ZedProjectorSettings.xml";

	ofxXmlPoco xml;
	xml.addChild("zedSETTINGS");
	xml.setTo("zedSETTINGS");
	xml.addValue("zedROI", zedROI);
	xml.addValue("basePlaneNormalBack", basePlaneNormalBack);
	xml.addValue("basePlaneOffsetBack", basePlaneOffsetBack);
	xml.addValue("basePlaneEq", basePlaneEq);
	xml.addValue("maxOffsetBack", maxOffsetBack);
	xml.addValue("spatialFiltering", spatialFiltering);
	xml.addValue("followBigChanges", followBigChanges);
	xml.addValue("numAveragingSlots", numAveragingSlots);
	xml.setToParent();
	return xml.save(settingsFile);
}
