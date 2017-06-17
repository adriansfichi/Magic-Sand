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

#include "ZedGrabber.h"
#include "ofConstants.h"

// OpenGL includes



//// Using std and sl namespaces
using namespace std;
using namespace sl;

ZedGrabber::ZedGrabber()
	:newFrame(true),
	bufferInitiated(false),
	zedOpened(false)
{
}

ZedGrabber::~ZedGrabber() {
	//    stop();
	waitForThread(true);
	//	waitForThread(true);
}

/// Start the thread.
void ZedGrabber::start() {
	startThread(true);
}

/// Signal the thread to stop.  After calling this method,
/// isThreadRunning() will return false and the while loop will stop
/// next time it has the chance to.
void ZedGrabber::stop() {
	stopThread();
}

bool ZedGrabber::setup() {
	// settings and defaults
	storedframes = 0;

	//zed.init();
	//zed.setRegistration(true); // To have correspondance between RGB and depth images
	//zed.setUseTexture(false);
	//width = zed.getWidth();
	//height = zed.getHeight();

	// Setup configuration parameters for the ZED
	InitParameters initParameters;
	initParameters.camera_resolution = sl::RESOLUTION_HD720;
	initParameters.depth_mode = sl::DEPTH_MODE_PERFORMANCE; //need quite a powerful graphic card in QUALITY
	initParameters.coordinate_units = sl::UNIT_METER; // set meter as the OpenGL world will be in meters
	initParameters.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed

																				// Open the ZED
	ERROR_CODE err = zed.open(initParameters);
	if (err != SUCCESS) {
		cout << errorCode2str(err) << endl;
		zed.close();
		//viewer.exit();
		return 1; // Quit if an error occurred
	}
	// Create an RGBA sl::Mat object
	sl::Mat image_depth_zed(zed.getResolution(), MAT_TYPE_8U_C4);

	width = image_depth_zed.getWidth();
	height = image_depth_zed.getHeight();

	zedDepthImage.allocate(width, height, 1);
	filteredframe.allocate(width, height, 1);
	zedColorImage.allocate(width, height);
	zedColorImage.setUseTexture(false);


	depthPixels_grayscale_.allocate(width, height, 1);
	depthPixels_mm_.allocate(width, height, 1);

	leftPixels_.allocate(width, height, 3);
	rightPixels_.allocate(width, height, 3);

	leftTexture_.allocate(width, height, GL_RGB, false);
	rightTexture_.allocate(width, height, GL_RGB, false);
	depthTexture_.allocate(width, height, GL_LUMINANCE, false);
	return true;
}

bool ZedGrabber::openZed() {
	zedOpened = zed.open();
	return zedOpened;
}
void ZedGrabber::setupFramefilter(int sgradFieldresolution, float newMaxOffset, ofRectangle ROI, bool sspatialFilter, bool sfollowBigChange, int snumAveragingSlots) {
	gradFieldresolution = sgradFieldresolution;
	ofLogVerbose("zedGrabber") << "setupFramefilter(): Gradient Field resolution: " << gradFieldresolution;
	gradFieldcols = width / gradFieldresolution;
	ofLogVerbose("zedGrabber") << "setupFramefilter(): Width: " << width << " Gradient Field Cols: " << gradFieldcols;
	gradFieldrows = height / gradFieldresolution;
	ofLogVerbose("zedGrabber") << "setupFramefilter(): Height: " << height << " Gradient Field Rows: " << gradFieldrows;

	spatialFilter = sspatialFilter;
	followBigChange = sfollowBigChange;
	numAveragingSlots = snumAveragingSlots;
	minNumSamples = (numAveragingSlots + 1) / 2;
	maxOffset = newMaxOffset;

	//Framefilter default parameters
	maxVariance = 4;
	hysteresis = 0.5f;
	bigChange = 10.0f;
	instableValue = 0.0;
	maxgradfield = 1000;
	initialValue = 4000;
	outsideROIValue = 3999;
	minInitFrame = 60;

	//Setup ROI
	setzedROI(ROI);

	//setting buffers
	initiateBuffers();
}

void ZedGrabber::initiateBuffers(void) {
	filteredframe.set(0);

	averagingBuffer = new float[numAveragingSlots*height*width];
	float* averagingBufferPtr = averagingBuffer;
	for (int i = 0; i<numAveragingSlots; ++i)
		for (unsigned int y = 0; y<height; ++y)
			for (unsigned int x = 0; x<width; ++x, ++averagingBufferPtr)
				*averagingBufferPtr = initialValue;

	averagingSlotIndex = 0;

	/* Initialize the statistics buffer: */
	statBuffer = new float[height*width * 3];
	float* sbPtr = statBuffer;
	for (unsigned int y = 0; y<height; ++y)
		for (unsigned int x = 0; x<width; ++x)
			for (int i = 0; i<3; ++i, ++sbPtr)
				*sbPtr = 0.0;

	/* Initialize the valid buffer: */
	validBuffer = new float[height*width];
	float* vbPtr = validBuffer;
	for (unsigned int y = 0; y<height; ++y)
		for (unsigned int x = 0; x<width; ++x, ++vbPtr)
			*vbPtr = initialValue;

	/* Initialize the gradient field buffer: */
	gradField = new glm::vec2[gradFieldcols*gradFieldrows];
	glm::vec2* gfPtr = gradField;
	for (unsigned int y = 0; y<gradFieldrows; ++y)
		for (unsigned int x = 0; x<gradFieldcols; ++x, ++gfPtr)
			*gfPtr = glm::vec2(0);

	bufferInitiated = true;
	currentInitFrame = 0;
	firstImageReady = false;
}

void ZedGrabber::resetBuffers(void) {
	if (bufferInitiated) {
		bufferInitiated = false;
		delete[] averagingBuffer;
		delete[] statBuffer;
		delete[] validBuffer;
		delete[] gradField;
	}
	initiateBuffers();
}
cv::Mat slMat2cvMat(sl::Mat& input) {
	//convert MAT_TYPE to CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	//cv::Mat and sl::Mat will share the same memory pointer
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

void ZedGrabber::threadedFunction() {
	while (isThreadRunning()) {
		this->actionsLock.lock(); // Update the grabber state if needed
		for (auto & action : this->actions) {
			action(*this);
		}
		this->actions.clear();
		this->actionsLock.unlock();

		if (zed.grab() == SUCCESS) {
			
			zedDepthImage = getDepthPixels_mm();

			filter();
			filteredframe.setImageType(OF_IMAGE_GRAYSCALE);
			updateGradientField();
			zedColorImage.setFromPixels(getDepthPixels_grayscale().getPixels(), width, height);
		}
		if (storedframes == 0)
		{
			filtered.send(std::move(filteredframe));
			gradient.send(std::move(gradField));
			colored.send(std::move(zedColorImage.getPixels()));
			lock();
			storedframes += 1;
			unlock();
		}

	}
	zed.close();
	delete[] averagingBuffer;
	delete[] statBuffer;
	delete[] validBuffer;
	delete[] gradField;
}

void ZedGrabber::performInThread(std::function<void(ZedGrabber&)> action) {
	this->actionsLock.lock();
	this->actions.push_back(action);
	this->actionsLock.unlock();
}

void ZedGrabber::filter()
{
	if (bufferInitiated)
	{
		const RawDepth* inputFramePtr = static_cast<const RawDepth*>(zedDepthImage.getData());
		float* averagingBufferPtr = averagingBuffer + averagingSlotIndex*height*width;
		float* statBufferPtr = statBuffer;
		float* validBufferPtr = validBuffer;
		float* filteredFramePtr = filteredframe.getData();

		inputFramePtr += minY*width;  // We only scan zed ROI
		averagingBufferPtr += minY*width;
		statBufferPtr += minY*width * 3;
		validBufferPtr += minY*width;
		filteredFramePtr += minY*width;

		for (unsigned int y = minY; y<maxY; ++y)
		{
			inputFramePtr += minX;
			averagingBufferPtr += minX;
			statBufferPtr += minX * 3;
			validBufferPtr += minX;
			filteredFramePtr += minX;
			for (unsigned int x = minX; x<maxX; ++x, ++inputFramePtr, ++averagingBufferPtr, statBufferPtr += 3, ++validBufferPtr, ++filteredFramePtr)
			{
				float newVal = static_cast<float>(*inputFramePtr);
				float oldVal = *averagingBufferPtr;

				if (newVal > maxOffset)//we are under the ceiling plane
				{
					*averagingBufferPtr = newVal; // Store the value
					if (followBigChange && statBufferPtr[0] > 0) { // Follow big changes
						float oldFiltered = statBufferPtr[1] / statBufferPtr[0]; // Compare newVal with average
						if (oldFiltered - newVal >= bigChange || newVal - oldFiltered >= bigChange)
						{
							float* aaveragingBufferPtr;
							for (int i = 0; i < numAveragingSlots; i++) { // update all averaging slots
								aaveragingBufferPtr = averagingBuffer + i*height*width + y*width + x;
								*aaveragingBufferPtr = newVal;
							}
							statBufferPtr[0] = numAveragingSlots; //Update statistics
							statBufferPtr[1] = newVal*numAveragingSlots;
							statBufferPtr[2] = newVal*newVal*numAveragingSlots;
						}
					}
					/* Update the pixel's statistics: */
					++statBufferPtr[0]; // Number of valid samples
					statBufferPtr[1] += newVal; // Sum of valid samples
					statBufferPtr[2] += newVal*newVal; // Sum of squares of valid samples

													   /* Check if the previous value in the averaging buffer was not initiated */
					if (oldVal != initialValue)
					{
						--statBufferPtr[0]; // Number of valid samples
						statBufferPtr[1] -= oldVal; // Sum of valid samples
						statBufferPtr[2] -= oldVal * oldVal; // Sum of squares of valid samples
					}
				}
				// Check if the pixel is "stable": */
				if (statBufferPtr[0] >= minNumSamples &&
					statBufferPtr[2] * statBufferPtr[0] <= maxVariance*statBufferPtr[0] * statBufferPtr[0] + statBufferPtr[1] * statBufferPtr[1])
				{
					/* Check if the new running mean is outside the previous value's envelope: */
					float newFiltered = statBufferPtr[1] / statBufferPtr[0];
					if (abs(newFiltered - *validBufferPtr) >= hysteresis)
					{
						/* Set the output pixel value to the depth-corrected running mean: */
						*filteredFramePtr = *validBufferPtr = newFiltered;
					}
					else {
						/* Leave the pixel at its previous value: */
						*filteredFramePtr = *validBufferPtr;
					}
				}
				*filteredFramePtr = *validBufferPtr;
			}
			inputFramePtr += width - maxX;
			averagingBufferPtr += width - maxX;
			statBufferPtr += (width - maxX) * 3;
			validBufferPtr += width - maxX;
			filteredFramePtr += width - maxX;
		}

		/* Go to the next averaging slot: */
		if (++averagingSlotIndex == numAveragingSlots)
			averagingSlotIndex = 0;

		if (!firstImageReady) {
			currentInitFrame++;
			if (currentInitFrame > minInitFrame)
				firstImageReady = true;
		}

		/* Apply a spatial filter if requested: */
		if (spatialFilter)
		{
			applySpaceFilter();
		}
	}
}

void ZedGrabber::applySpaceFilter()
{
	for (int filterPass = 0; filterPass<2; ++filterPass)
	{
		/* Low-pass filter the entire output frame in-place: */
		for (unsigned int x = minX; x<maxX; ++x)
		{
			/* Get a pointer to the current column: */
			float* colPtr = filteredframe.getData() + x;

			/* Filter the first pixel in the column: */
			float lastVal = *colPtr;
			*colPtr = (colPtr[0] * 2.0f + colPtr[width]) / 3.0f;
			colPtr += width;

			/* Filter the interior pixels in the column: */
			for (unsigned int y = minY + 1; y<maxY - 1; ++y, colPtr += width)
			{
				/* Filter the pixel: */
				float nextLastVal = *colPtr;
				*colPtr = (lastVal + colPtr[0] * 2.0f + colPtr[width])*0.25f;
				lastVal = nextLastVal;
			}

			/* Filter the last pixel in the column: */
			*colPtr = (lastVal + colPtr[0] * 2.0f) / 3.0f;
		}
		float* rowPtr = filteredframe.getData();
		for (unsigned int y = minY; y<maxY; ++y)
		{
			/* Filter the first pixel in the row: */
			float lastVal = *rowPtr;
			*rowPtr = (rowPtr[0] * 2.0f + rowPtr[1]) / 3.0f;
			++rowPtr;

			/* Filter the interior pixels in the row: */
			for (unsigned int x = minX + 1; x<maxX - 1; ++x, ++rowPtr)
			{
				/* Filter the pixel: */
				float nextLastVal = *rowPtr;
				*rowPtr = (lastVal + rowPtr[0] * 2.0f + rowPtr[1])*0.25f;
				lastVal = nextLastVal;
			}

			/* Filter the last pixel in the row: */
			*rowPtr = (lastVal + rowPtr[0] * 2.0f) / 3.0f;
			++rowPtr;
		}
	}
}

void ZedGrabber::updateGradientField()
{
	int ind = 0;
	float gx;
	float gy;
	int gvx, gvy;
	float lgth = 0;
	float* filteredFramePtr = filteredframe.getData();
	for (unsigned int y = 0; y<gradFieldrows; ++y) {
		for (unsigned int x = 0; x<gradFieldcols; ++x) {
			if (isInsideROI(x*gradFieldresolution, y*gradFieldresolution) && isInsideROI((x + 1)*gradFieldresolution, (y + 1)*gradFieldresolution)) {
				gx = 0;
				gvx = 0;
				gy = 0;
				gvy = 0;
				for (unsigned int i = 0; i<gradFieldresolution; i++) {
					ind = y*gradFieldresolution*width + i*width + x*gradFieldresolution;
					if (filteredFramePtr[ind] != 0 && filteredFramePtr[ind + gradFieldresolution - 1] != 0) {
						gvx += 1;
						gx += filteredFramePtr[ind] - filteredFramePtr[ind + gradFieldresolution - 1];
					}
					ind = y*gradFieldresolution*width + i + x*gradFieldresolution;
					if (filteredFramePtr[ind] != 0 && filteredFramePtr[ind + (gradFieldresolution - 1)*width] != 0) {
						gvy += 1;
						gy += filteredFramePtr[ind] - filteredFramePtr[ind + (gradFieldresolution - 1)*width];
					}
				}
				if (gvx != 0 && gvy != 0)
					gradField[y*gradFieldcols + x] = glm::vec2(gx / gradFieldresolution / gvx, gy / gradFieldresolution / gvy);
				if (gradField[y*gradFieldcols + x].length() > maxgradfield) {
					gradField[y*gradFieldcols + x]= gradField[y*gradFieldcols + x]*maxgradfield;// /= gradField[y*gradFieldcols+x].length()*maxgradfield;
					lgth += 1;
				}
			}
			else {
				gradField[y*gradFieldcols + x] = glm::vec2(0);
			}
		}
	}
}

bool ZedGrabber::isInsideROI(int x, int y) {
	if (x<minX || x>maxX || y<minY || y>maxY)
		return false;
	return true;
}

void ZedGrabber::setzedROI(ofRectangle ROI) {
	minX = static_cast<int>(ROI.getMinX());
	maxX = static_cast<int>(ROI.getMaxX());
	minY = static_cast<int>(ROI.getMinY());
	maxY = static_cast<int>(ROI.getMaxY());
	ROIwidth = maxX - minX;
	ROIheight = maxY - minY;
	resetBuffers();
}

void ZedGrabber::setAveragingSlotsNumber(int snumAveragingSlots) {
	if (bufferInitiated) {
		bufferInitiated = false;
		delete[] averagingBuffer;
		delete[] statBuffer;
		delete[] validBuffer;
		delete[] gradField;
	}
	numAveragingSlots = snumAveragingSlots;
	minNumSamples = (numAveragingSlots + 1) / 2;
	initiateBuffers();
}

void ZedGrabber::setGradFieldResolution(int sgradFieldresolution) {
	if (bufferInitiated) {
		bufferInitiated = false;
		delete[] averagingBuffer;
		delete[] statBuffer;
		delete[] validBuffer;
		delete[] gradField;
	}
	gradFieldresolution = sgradFieldresolution;
	initiateBuffers();
}

void ZedGrabber::setFollowBigChange(bool newfollowBigChange) {
	if (bufferInitiated) {
		bufferInitiated = false;
		delete[] averagingBuffer;
		delete[] statBuffer;
		delete[] validBuffer;
		delete[] gradField;
	}
	followBigChange = newfollowBigChange;
	initiateBuffers();
}

glm::vec3 ZedGrabber::getStatBuffer(int x, int y) {
	float* statBufferPtr = statBuffer + 3 * (x + y*width);
	return glm::vec3(statBufferPtr[0], statBufferPtr[1], statBufferPtr[2]);
}

float ZedGrabber::getAveragingBuffer(int x, int y, int slotNum) {
	float* averagingBufferPtr = averagingBuffer + slotNum*height*width + (x + y*width);
	return *averagingBufferPtr;
}

float ZedGrabber::getValidBuffer(int x, int y) {
	float* validBufferPtr = validBuffer + (x + y*width);
	return *validBufferPtr;
}

glm::mat4x4 ZedGrabber::getWorldMatrix() {
	auto mat = glm::mat4x4();
	sl::Pose camera_pose;
	zed.getPosition(camera_pose);
	auto position = camera_pose.pose_data;
	if (zedOpened) {
	
		mat = glm::mat4x4(position.r00, position.r01, position.r02, position.getTranslation().x,
			position.r10, position.r11, position.r12, position.getTranslation().y,
			position.r20, position.r21, position.r22, position.getTranslation().z,
			0, 0, 0, position.getTranslation().z);
	}
	return mat;
}

bool ZedGrabber::started()
{
	return true;
}

ofFloatPixels & ZedGrabber::getDepthPixels_mm()
{
	if (started()) {
		if (depthPixels_mm_Dirty_) {
			depthPixels_mm_Dirty_ = false;
			sl::Mat zedView;
			zed.retrieveMeasure(zedView, sl::MEASURE_DEPTH);
			cv::Mat image_ocv = slMat2cvMat(zedView);

			float *pix = depthPixels_mm_.getData();
			int step = zedView.getStep() / 4;
			for (int y = 0; y < height; y++) {
				for (int x = 0; x < width; x++) {
					float pixel = ((float*)(zedView.getPtr<sl::uchar1>(sl::MEM_CPU)))[x + step*y];
					pix[x + y * width] = pixel;
				}
			}
		}
	}

	return depthPixels_mm_;
}

//------------------------------------------------------------------------------------------------------
ofPixels & ZedGrabber::getDepthPixels_grayscale(float min_depth_mm, float max_depth_mm)
{
	if (started()) {
		if (depthPixels_grayscale_Dirty_) {
			depthPixels_grayscale_Dirty_ = false;

			sl::Mat zedView;
			zed.retrieveMeasure(zedView, sl::MEASURE_DEPTH);

			uchar *pix = depthPixels_grayscale_.getData();

			//ofLog() << zedView.width << " // " << zedView.height << endl;
			for (int y = 0; y < height; y++) {
				for (int x = 0; x < width; x++) {
					sl::uchar3 pixel;
					auto error = zedView.getValue(x, y, &pixel);
					int index = (x + y * width);
					pix[index] = pixel.z;
				}
			}
		}
	}

	return depthPixels_grayscale_;
}

//------------------------------------------------------------------------------------------------------
ofTexture &ZedGrabber::getDepthTexture(float min_depth_mm, float max_depth_mm)
{
	if (started()) {
		if (!useDepth_) {
			ofLogWarning() << "ZED: trying to access depth buffer. You need to call setUseDepth(true) before it!" << endl;
		}
		else {
			if (depthTextureDirty_) {
				depthTextureDirty_ = false;
				depthTexture_.loadData(getDepthPixels_grayscale(min_depth_mm, max_depth_mm));
			}
		}
	}
	return depthTexture_;
}

//------------------------------------------------------------------------------------------------------
ofPixels & ZedGrabber::getLeftPixels()
{
	if (started()) {
		if (!useImages_) {
			ofLogWarning() << "ZED: trying to access left image pixels. You need to call setUseImages(true) before it!" << endl;
		}
		else {
			if (leftPixelsDirty_) {
				leftPixelsDirty_ = false;
				sl::Mat zedView;
				zed.retrieveImage(zedView, sl::VIEW_LEFT);
				uchar *pix = leftPixels_.getData();

				for (int y = 0; y < height; y++) {
					for (int x = 0; x < width; x++) {
						sl::uchar4 pixel;
						auto error = zedView.getValue(x, y, &pixel, sl::MEM_CPU);
						int index = 3 * (x + y * width);
						pix[index + 0] = pixel.z;
						pix[index + 1] = pixel.y;
						pix[index + 2] = pixel.x;
					}
				}
			}
		}
	}
	return leftPixels_;
}


//------------------------------------------------------------------------------------------------------
ofTexture & ZedGrabber::getLeftTexture()
{
	if (started()) {
		if (!useImages_) {
			ofLogWarning() << "ZED: trying to access left image. You need to call setUseImages(true) before it!" << endl;
		}
		else {
			if (leftTextureDirty_) {
				leftTextureDirty_ = false;
				leftTexture_.loadData(getLeftPixels());
			}
		}
	}
	return leftTexture_;
}

//------------------------------------------------------------------------------------------------------
ofPixels & ZedGrabber::getRightPixels()
{
	if (started()) {
		if (!useImages_) {
			ofLogWarning() << "ZED: trying to access right image pixels. You need to call setUseImages(true) before it!" << endl;
		}
		else {
			if (rightPixelsDirty_) {
				rightPixelsDirty_ = false;

				sl::Mat zedView;
				zed.retrieveImage(zedView, sl::VIEW_RIGHT);

				uchar *pix = rightPixels_.getData();
				for (int y = 0; y < height; y++) {
					for (int x = 0; x < width; x++) {
						sl::float3 pixel;
						auto error = zedView.getValue(x, y, &pixel, sl::MEM_CPU);
						int index = 3 * (x + y * width);
						pix[index + 0] = pixel.z;
						pix[index + 1] = pixel.y;
						pix[index + 2] = pixel.x;
					}
				}
			}
		}
	}
	return rightPixels_;
}

//------------------------------------------------------------------------------------------------------
ofTexture & ZedGrabber::getRightTexture()
{
	if (started()) {
		if (!useImages_) {
			ofLogWarning() << "ZED: trying to access right image. You need to call setUseImages(true) before it!" << endl;
		}
		else {
			if (rightTextureDirty_) {
				rightTextureDirty_ = false;
				rightTexture_.loadData(getRightPixels());
			}
		}
	}
	return rightTexture_;
}

//------------------------------------------------------------------------------------------------------
void ZedGrabber::fillPointCloud() {
	if (started()) {
		if (!usePointCloud_) {
			ofLogWarning() << "ZED: trying to access point cloud. You need to call setUsePointCloud(true,true|false) before it!" << endl;
		}
		else {
			if (pointCloudDirty_) {
				pointCloudDirty_ = false;

				if (!usePointCloudColors_) {
					sl::Mat zedView;
					zed.retrieveMeasure(zedView, sl::MEASURE_XYZ);
					//XYZ, 3D coordinates of the image points, 4 channels, FLOAT  (the 4th channel may contains the colors)

					int w = zedView.getWidth();
					int h = zedView.getHeight();
					int step = zedView.getStep() / 4;
					pointCloud_.resize(w*h);
					pointCloudColors_.clear();

					float *data = (float*)(zedView.getPtr<sl::uchar1>(sl::MEM_CPU));
					for (int y = 0; y < h; y++) {
						for (int x = 0; x < w; x++) {
							int index = x * 4 + step*y;
							pointCloud_[x + w*y] = ofPoint(data[index], data[index + 1], data[index + 2]);
						}
					}
				}
				else {
					sl::Mat zedView;
					zed.retrieveImage(zedView, sl::VIEW_LEFT);
					zed.retrieveMeasure(zedView, sl::MEASURE_XYZRGBA);
					//XYZRGBA, 3D coordinates and Color of the image , 4 channels, FLOAT (the 4th channel encode 4 UCHAR for color) \ingroup Enumerations*/
					int w = zedView.getWidth();
					int h = zedView.getHeight();
					int step = zedView.getStep() / 4;
					int step_char = zedView.getStep();
					pointCloud_.resize(w*h);
					pointCloudColors_.resize(w*h);

					float *data = (float*)(zedView.getPtr<sl::uchar1>(sl::MEM_CPU));
					uchar *data_char = zedView.getPtr<sl::uchar1>(sl::MEM_CPU);
					for (int y = 0; y < h; y++) {
						for (int x = 0; x < w; x++) {
							int index = x * 4 + step*y;
							int index_color = (index + 3) * 4;
							pointCloud_[x + w*y] = ofPoint(data[index], data[index + 1], data[index + 2]);
							pointCloudColors_[x + w*y] = ofColor(data_char[index_color], data_char[index_color + 1], data_char[index_color + 2], data_char[index_color + 3]);
						}
					}
				}
				//flip points if required
				if (pointCloudFlipY_) {
					for (size_t i = 0; i < pointCloud_.size(); i++) {
						pointCloud_[i].y = -pointCloud_[i].y;
					}
				}
				if (pointCloudFlipZ_) {
					for (size_t i = 0; i < pointCloud_.size(); i++) {
						pointCloud_[i].z = -pointCloud_[i].z;
					}
				}
			}
		}
	}

}

//------------------------------------------------------------------------------------------------------
vector<ofGlmPoint>& ZedGrabber::getPointCloud()
{
	fillPointCloud();
	return pointCloud_;
}

//------------------------------------------------------------------------------------------------------
vector<ofColor>& ZedGrabber::getPointCloudColors()
{
	fillPointCloud();
	return pointCloudColors_;
}

//------------------------------------------------------------------------------------------------------
vector<ofFloatColor>& ZedGrabber::getPointCloudFloatColors()
{
	if (pointCloudFloatColorsDirty_) {
		pointCloudFloatColorsDirty_ = false;
		fillPointCloud();
		//convert pointCloudColors_ to pointCloudFloatColors_
		size_t n = pointCloudColors_.size();
		pointCloudFloatColors_.resize(n);
		for (size_t i = 0; i < n; i++) {
			pointCloudFloatColors_[i] = pointCloudColors_[i];
		}

	}
	return pointCloudFloatColors_;
}

//------------------------------------------------------------------------------------------------------
void ZedGrabber::drawLeft(float x, float y, float w, float h)
{
	if (w == 0) w = width;
	if (h == 0) h = height;
	getLeftTexture().draw(x, y, w, h);
}

//------------------------------------------------------------------------------------------------------
void ZedGrabber::drawRight(float x, float y, float w, float h)
{
	if (w == 0) w = width;
	if (h == 0) h = height;
	getRightTexture().draw(x, y, w, h);
}

//------------------------------------------------------------------------------------------------------
void ZedGrabber::drawDepth(float x, float y, float w, float h, float min_mm, float max_mm)
{
	if (w == 0) w = width;
	if (h == 0) h = height;
	getDepthTexture(min_mm, max_mm).draw(x, y, w, h);

}

//------------------------------------------------------------------------------------------------------
void ZedGrabber::drawPointCloud()
{
	vector<ofGlmPoint> &points = getPointCloud();
	vector<ofFloatColor> &colors = getPointCloudFloatColors();
	ofMesh mesh;
	mesh.addVertices(points);
	if (colors.size() == points.size()) mesh.addColors(colors);
	mesh.drawVertices();
}