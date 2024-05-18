///////////////////////////////////////////////////////////////////////////////
// FILE:          XZellZeissCamera.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   The X-Zell implementation of the Zeiss Axio camera.
//                
// AUTHOR:        Jeremy Uff, j.uff@influxury.net, 23/02/2024
//
// COPYRIGHT:     X-Zell Biotech Pte Ltd, Singapore
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
#include "stdafx.h"
#include "mcam_zei.h"
#include "mcam_zei_ex.h"

#include "XZellZeissCam.h"
#include <cstdio>
#include <string>
#include <math.h>
#include "ModuleInterface.h"
#include <sstream>
#include <algorithm>
#include <iostream>
#include <future>

const double XZellZeissCamera::nominalPixelSizeUm_ = 1.0;
double g_IntensityFactor_ = 1.0;

// External names used used by the rest of the system
// to load particular device from the "XZellZeissCam.dll" library
const char* g_CameraDeviceName = "XZellZeissCam";
const char* g_HubDeviceName = "XZellHub";

// constants for naming pixel types (allowed values of the "PixelType" property)
const char* g_PixelType_8bit = "8bit";
const char* g_PixelType_16bit = "16bit";
const char* g_PixelType_32bitRGB = "32bitRGB";
const char* g_PixelType_64bitRGB = "64bitRGB";
const char* g_PixelType_32bit = "32bit";  // floating point greyscale

// constants for naming camera modes
const char* g_Sine_Wave = "Artificial Waves";
const char* g_Norm_Noise = "Noise";
const char* g_Color_Test = "Color Test Pattern";

enum { MODE_ARTIFICIAL_WAVES, MODE_NOISE, MODE_COLOR_TEST };


// START OF ZEISS SPECIFIC DEV CODE
void* pContext;
unsigned long contextSize = 0;
ContinuousCallbackProc callBackProc;
unsigned long pimageByteSize = 0;
long error = 0;

// Variables for Double Buffering
unsigned short* ImageBufferWithHeader[2]; // 2 images buffer in which the ImageCllback copies the current image
unsigned short* processedImage;
int imageNumber[2];
int lastImageRead = 0;
std::mutex bufferMutex[2];

unsigned int imageCount = 0;

bool processImageInSDK = true;

// CONDITION VARIABLE IMPLEMENTATION START
std::condition_variable cv;
std::mutex mtx;
bool newImageAvailable = false;
// CONDITION VARIABLE IMPLEMENTATION END
// END OF ZEISS SPECIFIC DEV CODE

///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////

MODULE_API void InitializeModuleData()
{
   RegisterDevice(g_CameraDeviceName, MM::CameraDevice, "XZell Zeiss camera");
   RegisterDevice(g_HubDeviceName, MM::HubDevice, "XZellHub");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
   if (deviceName == 0)
      return 0;

   // decide which device class to create based on the deviceName parameter
   if (strcmp(deviceName, g_CameraDeviceName) == 0)
   {
      // create camera
      return new XZellZeissCamera();
   }
   else if (strcmp(deviceName, g_HubDeviceName) == 0)
   {
	  return new DemoHub();
   }

   // ...supplied name not recognized
   return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
   delete pDevice;
}

///////////////////////////////////////////////////////////////////////////////
// XZellZeissCamera implementation
// ~~~~~~~~~~~~~~~~~~~~~~~~~~

/**
* XZellZeissCamera constructor.
* Setup default all variables and create device properties required to exist
* before intialization. In this case, no such properties were required. All
* properties will be created in the Initialize() method.
*
* As a general guideline Micro-Manager devices do not access hardware in the
* the constructor. We should do as little as possible in the constructor and
* perform most of the initialization in the Initialize() method.
*/
XZellZeissCamera::XZellZeissCamera() :
   CCameraBase<XZellZeissCamera> (),
   exposureMaximum_(10000.0),
   dPhase_(0),
   initialized_(false),
   readoutUs_(0.0),
   scanMode_(1),
   bitDepth_(8),
   roiX_(0),
   roiY_(0),
   sequenceStartTime_(0),
   isSequenceable_(false),
   sequenceMaxLength_(100),
   sequenceRunning_(false),
   sequenceIndex_(0),
	binSize_(1),
	cameraCCDXSize_(2464),
	cameraCCDYSize_(2056),
   ccdT_ (0.0),
   triggerDevice_(""),
   stopOnOverflow_(false),
	dropPixels_(false),
   fastImage_(false),
   saturatePixels_(false),
	fractionOfPixelsToDropOrSaturate_(0.002),
   shouldRotateImages_(false),
   shouldDisplayImageNumber_(false),
   stripeWidth_(1.0),
   multiROIFillValue_(0),
   nComponents_(1),
   mode_(MODE_ARTIFICIAL_WAVES),
   imgManpl_(0),
   pcf_(1.0),
   photonFlux_(50.0),
   readNoise_(2.5)
{
   memset(testProperty_,0,sizeof(testProperty_));

   // call the base class method to set-up default error codes/messages
   InitializeDefaultErrorMessages();
   readoutStartTime_ = GetCurrentMMTime();
   thd_ = new ZeissAcquisitionThread(this);

   // parent ID display
   CreateHubIDProperty();

   CreateFloatProperty("MaximumExposureMs", exposureMaximum_, false,
         new CPropertyAction(this, &XZellZeissCamera::OnMaxExposure),
         true);
}

/**
* XZellZeissCamera destructor.
* If this device used as intended within the Micro-Manager system,
* Shutdown() will be always called before the destructor. But in any case
* we need to make sure that all resources are properly released even if
* Shutdown() was not called.
*/
XZellZeissCamera::~XZellZeissCamera()
{
   StopSequenceAcquisition();
   delete thd_;
}

/**
* Obtains device name.
* Required by the MM::Device API.
*/
void XZellZeissCamera::GetName(char* name) const
{
    LogMessage("ZEISS API METHOD ENTRY: GetName");
   // Return the name used to referr to this device adapte
   CDeviceUtils::CopyLimitedString(name, g_CameraDeviceName);
}

/**
* Intializes the hardware.
* Required by the MM::Device API.
* Typically we access and initialize hardware at this point.
* Device properties are typically created here as well, except
* the ones we need to use for defining initialization parameters.
* Such pre-initialization properties are created in the constructor.
* (This device does not have any pre-initialization properties)
*/
int XZellZeissCamera::Initialize()
{
    LogMessage("ZEISS API METHOD ENTRY: Initialize");
   if (initialized_)
      return DEVICE_OK;

   // START OF ZEISS SPECIFIC DEV CODE
   long binningValue = 0;
   long pixelClockIndex = 0;
   BOOL hasSubSampling = false;
   long resolutionWidth = 0;
   long resolutionHeight = 0;
   eMcamScanMode scanMode;
   MCammLineFlickerSuppressionMode lineFlickerSuppressionMode;
   long bitsPerPixel = 0;
   MCammColorMatrixOptimizationMode colorMatrixMode;
   //unsigned short* ImageBufferWithHeader = NULL;
   //long imageSize = 0;

   long error = McammLibInit(false);
   int numCam = McamGetNumberofCameras();

   std::ostringstream ossA;
   ossA << "DEV: Number of Cameras found: " << numCam << "\n";
   LogMessage(ossA.str().c_str());

   if (numCam > 0)
   {
       McammInit(0);
       McammInfo(0, &cameraInfos);

       {
	       std::ostringstream ossB;
	       ossB << "AxioCam " << cameraInfos.Features << " #" << cameraInfos.SerienNummer;
	       LogMessage(ossB.str().c_str());
	   }

        long result = McammGetCurrentBinning(0, &binningValue);
        if (result == 0)
        {
            std::ostringstream oss;
            oss << "DEV: Retrieved Binning Value: " << binningValue << "\n";
            oss << "DEV: binSize_ Value: " << binSize_ << "\n";
            LogMessage(oss.str().c_str());
            binSize_ = binningValue;
        }
        result = McammGetCurrentPixelClock(0, &pixelClockIndex);
        if (result == 0)
        {
            std::ostringstream oss;
            oss << "DEV: Retrieved Pixel Clock Index: " << pixelClockIndex << "\n";
            LogMessage(oss.str().c_str());
        }
        long hasSubSamplingResult = MCammHasSubsampling(0, 1, &hasSubSampling);
        if (hasSubSamplingResult > 0)
        {
            std::ostringstream oss;
            oss << "DEV: hasSubSampling can be 1: " << hasSubSampling << "\n";
            LogMessage(oss.str().c_str());
        }
        hasSubSamplingResult = MCammHasSubsampling(0, 2, &hasSubSampling);
        if (hasSubSamplingResult > 0)
        {
            std::ostringstream oss;
            oss << "DEV: hasSubSampling can be 2: " << hasSubSampling << "\n";
            LogMessage(oss.str().c_str());
        }
        {
            long currentResolution = McammGetCurrentResolution(0);
            std::ostringstream oss;
            oss << "DEV: Current Resolution: " << currentResolution << "\n";
            LogMessage(oss.str().c_str());
        }
        {
            long numberOfResolutions = McammGetNumberOfResolutions(0);
            std::ostringstream oss;
            oss << "DEV: Number of Resolutions: " << numberOfResolutions << "\n";
            LogMessage(oss.str().c_str());
        }
        result = McammGetResolutionValues(0, 1, &resolutionWidth, &resolutionHeight, &scanMode);
        if (result == 0)
        {
            std::ostringstream oss;
            oss << "DEV: Resolution 1 Width: " << resolutionWidth << "\n";
            oss << "DEV: Resolution 1 Height: " << resolutionHeight << "\n";
            LogMessage(oss.str().c_str());
        }
        result = McammGetResolutionValues(0, 2, &resolutionWidth, &resolutionHeight, &scanMode);
        if (result == 0)
        {
            std::ostringstream oss;
            oss << "DEV: Resolution 2 Width: " << resolutionWidth << "\n";
            oss << "DEV: Resolution 2 Height: " << resolutionHeight << "\n";
            LogMessage(oss.str().c_str());
        }
        result = McammGetResolutionValues(0, 0, &resolutionWidth, &resolutionHeight, &scanMode);
        if (result == 0)
        {
            std::ostringstream oss;
            oss << "DEV: Resolution 0 Width: " << resolutionWidth << "\n";
            oss << "DEV: Resolution 0 Height: " << resolutionHeight << "\n";
            LogMessage(oss.str().c_str());
        }
        result = McammGetLineFlickerSuppressionMode(0, &lineFlickerSuppressionMode);
        if (result == 0)
        {
			std::string lineFlickerSuppressionModeValue;
			switch (lineFlickerSuppressionMode) {
            case mcammLineFlickerSuppressionOff:
                lineFlickerSuppressionModeValue = "Off";
                break;
            case mcammLineFlickerSuppressionLinear:
                lineFlickerSuppressionModeValue = "Linear";
                break;
            case mcammLineFlickerSuppressionBiLinear:
                lineFlickerSuppressionModeValue = "BiLinear";
                break;
            }
            std::ostringstream oss;
            oss << "DEV: Line Flicker Suppression Mode: " << lineFlickerSuppressionModeValue << "\n";
            LogMessage(oss.str().c_str());
        }
        result = McammGetCurrentBitsPerPixelEx(0, &bitsPerPixel);
        if (result == 0)
        {
            std::ostringstream oss;
            oss << "DEV: BitsPerPixel: " << bitsPerPixel << "\n";
            LogMessage(oss.str().c_str());
        }
        //result = McammSetBitsPerPixel(0, 8);
        //result = McammGetCurrentBitsPerPixelEx(0, &bitsPerPixel);
        //if (result == 0)
        //{
        //    std::ostringstream oss;
        //    oss << "DEV: BitsPerPixel: " << bitsPerPixel << "\n";
        //    LogMessage(oss.str().c_str());
        //}

        result = McammGetColorMatrixOptimizationMode(0, &colorMatrixMode);
        if (result == 0)
        {
            std::ostringstream oss;
            oss << "DEV: Color Matrix Mode: " << colorMatrixMode << "\n";
            LogMessage(oss.str().c_str());
        }
	    
    }
   // END OF ZEISS SPECIFIC DEV CODE


   DemoHub* pHub = static_cast<DemoHub*>(GetParentHub());
   if (pHub)
   {
      char hubLabel[MM::MaxStrLength];
      pHub->GetLabel(hubLabel);
      SetParentID(hubLabel); // for backward comp.
   }
   else
      LogMessage(NoHubError);

   // set property list
   // -----------------

   // Name
   int nRet = CreateStringProperty(MM::g_Keyword_Name, g_CameraDeviceName, true);
   if (DEVICE_OK != nRet)
      return nRet;

   // Description
   nRet = CreateStringProperty(MM::g_Keyword_Description, "XZell Zeiss Camera Device Adapter", true);
   if (DEVICE_OK != nRet)
      return nRet;

   // CameraName
   nRet = CreateStringProperty(MM::g_Keyword_CameraName, "XZellZeissCamera-MultiMode", true);
   assert(nRet == DEVICE_OK);

   // CameraID
   nRet = CreateStringProperty(MM::g_Keyword_CameraID, "V1.0", true);
   assert(nRet == DEVICE_OK);

   // binning
   CPropertyAction *pAct = new CPropertyAction (this, &XZellZeissCamera::OnBinning);
   nRet = CreateIntegerProperty(MM::g_Keyword_Binning, 1, false, pAct);
   assert(nRet == DEVICE_OK);

   nRet = SetAllowedBinning();
   if (nRet != DEVICE_OK)
      return nRet;

   // pixel type
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnPixelType);
   nRet = CreateStringProperty(MM::g_Keyword_PixelType, g_PixelType_8bit, false, pAct);
   assert(nRet == DEVICE_OK);

   std::vector<std::string> pixelTypeValues;
   pixelTypeValues.push_back(g_PixelType_8bit);
   pixelTypeValues.push_back(g_PixelType_16bit); 

   nRet = SetAllowedValues(MM::g_Keyword_PixelType, pixelTypeValues);
   if (nRet != DEVICE_OK)
      return nRet;

   // Bit depth
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnBitDepth);
   nRet = CreateIntegerProperty("BitDepth", 8, false, pAct);
   assert(nRet == DEVICE_OK);

   std::vector<std::string> bitDepths;
   bitDepths.push_back("8");
   bitDepths.push_back("10");
   bitDepths.push_back("11");
   bitDepths.push_back("12");
   bitDepths.push_back("14");
   bitDepths.push_back("16");
   bitDepths.push_back("32");
   nRet = SetAllowedValues("BitDepth", bitDepths);
   if (nRet != DEVICE_OK)
      return nRet;

   // exposure
   nRet = CreateFloatProperty(MM::g_Keyword_Exposure, 10.0, false);
   assert(nRet == DEVICE_OK);
   SetPropertyLimits(MM::g_Keyword_Exposure, 0.0, exposureMaximum_);

	CPropertyActionEx *pActX = 0;
	// create an extended (i.e. array) properties 1 through 4
	
	for(int ij = 1; ij < 7;++ij)
	{
      std::ostringstream os;
      os<<ij;
      std::string propName = "TestProperty" + os.str();
		pActX = new CPropertyActionEx(this, &XZellZeissCamera::OnTestProperty, ij);
      nRet = CreateFloatProperty(propName.c_str(), 0., false, pActX);
      if(0!=(ij%5))
      {
         // try several different limit ranges
         double upperLimit = (double)ij*pow(10.,(double)(((ij%2)?-1:1)*ij));
         double lowerLimit = (ij%3)?-upperLimit:0.;
         SetPropertyLimits(propName.c_str(), lowerLimit, upperLimit);
      }
	}

   // Test Property with an async callback
   // When the leader is set the follower will be set to the same value
   // with some delay (default 2 seconds).
   // This is to allow downstream testing of callbacks originating from
   // device threads.
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnAsyncLeader);
   CreateStringProperty("AsyncPropertyLeader", "init", false, pAct);
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnAsyncFollower);
   CreateStringProperty("AsyncPropertyFollower", "init", true, pAct);
   CreateIntegerProperty("AsyncPropertyDelayMS", 2000, false);

   //pAct = new CPropertyAction(this, &XZellZeissCamera::OnSwitch);
   //nRet = CreateIntegerProperty("Switch", 0, false, pAct);
   //SetPropertyLimits("Switch", 8, 1004);
	
	
	// scan mode
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnScanMode);
   nRet = CreateIntegerProperty("ScanMode", 1, false, pAct);
   assert(nRet == DEVICE_OK);
   AddAllowedValue("ScanMode","1");
   AddAllowedValue("ScanMode","2");
   AddAllowedValue("ScanMode","3");

   // camera gain
   nRet = CreateIntegerProperty(MM::g_Keyword_Gain, 0, false);
   assert(nRet == DEVICE_OK);
   SetPropertyLimits(MM::g_Keyword_Gain, -5, 8);

   // camera offset
   nRet = CreateIntegerProperty(MM::g_Keyword_Offset, 0, false);
   assert(nRet == DEVICE_OK);

   // camera temperature
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnCCDTemp);
   nRet = CreateFloatProperty(MM::g_Keyword_CCDTemperature, 0, false, pAct);
   assert(nRet == DEVICE_OK);
   SetPropertyLimits(MM::g_Keyword_CCDTemperature, -100, 10);

   // camera temperature RO
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnCCDTemp);
   nRet = CreateFloatProperty("CCDTemperature RO", 0, true, pAct);
   assert(nRet == DEVICE_OK);

   // readout time
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnReadoutTime);
   nRet = CreateFloatProperty(MM::g_Keyword_ReadoutTime, 0, false, pAct);
   assert(nRet == DEVICE_OK);

   // CCD size of the camera we are modeling
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnCameraCCDXSize);
   CreateIntegerProperty("OnCameraCCDXSize", 512, false, pAct);
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnCameraCCDYSize);
   CreateIntegerProperty("OnCameraCCDYSize", 512, false, pAct);

   // Trigger device
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnTriggerDevice);
   CreateStringProperty("TriggerDevice", "", false, pAct);

   pAct = new CPropertyAction (this, &XZellZeissCamera::OnDropPixels);
   CreateIntegerProperty("DropPixels", 0, false, pAct);
   AddAllowedValue("DropPixels", "0");
   AddAllowedValue("DropPixels", "1");

	pAct = new CPropertyAction (this, &XZellZeissCamera::OnSaturatePixels);
   CreateIntegerProperty("SaturatePixels", 0, false, pAct);
   AddAllowedValue("SaturatePixels", "0");
   AddAllowedValue("SaturatePixels", "1");

   pAct = new CPropertyAction (this, &XZellZeissCamera::OnFastImage);
   CreateIntegerProperty("FastImage", 0, false, pAct);
   AddAllowedValue("FastImage", "0");
   AddAllowedValue("FastImage", "1");

   pAct = new CPropertyAction (this, &XZellZeissCamera::OnFractionOfPixelsToDropOrSaturate);
   CreateFloatProperty("FractionOfPixelsToDropOrSaturate", 0.002, false, pAct);
	SetPropertyLimits("FractionOfPixelsToDropOrSaturate", 0., 0.1);

   pAct = new CPropertyAction(this, &XZellZeissCamera::OnShouldRotateImages);
   CreateIntegerProperty("RotateImages", 0, false, pAct);
   AddAllowedValue("RotateImages", "0");
   AddAllowedValue("RotateImages", "1");

   pAct = new CPropertyAction(this, &XZellZeissCamera::OnShouldDisplayImageNumber);
   CreateIntegerProperty("DisplayImageNumber", 0, false, pAct);
   AddAllowedValue("DisplayImageNumber", "0");
   AddAllowedValue("DisplayImageNumber", "1");

   pAct = new CPropertyAction(this, &XZellZeissCamera::OnStripeWidth);
   CreateFloatProperty("StripeWidth", 0, false, pAct);
   SetPropertyLimits("StripeWidth", 0, 10);

   pAct = new CPropertyAction(this, &XZellZeissCamera::OnMultiROIFillValue);
   CreateIntegerProperty("MultiROIFillValue", 0, false, pAct);
   SetPropertyLimits("MultiROIFillValue", 0, 65536);

   // Whether or not to use exposure time sequencing
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnIsSequenceable);
   std::string propName = "UseExposureSequences";
   CreateStringProperty(propName.c_str(), "No", false, pAct);
   AddAllowedValue(propName.c_str(), "Yes");
   AddAllowedValue(propName.c_str(), "No");

   // Camera mode: 
   pAct = new CPropertyAction (this, &XZellZeissCamera::OnMode);
   propName = "Mode";
   CreateStringProperty(propName.c_str(), g_Sine_Wave, false, pAct);
   AddAllowedValue(propName.c_str(), g_Sine_Wave);
   AddAllowedValue(propName.c_str(), g_Norm_Noise);
   AddAllowedValue(propName.c_str(), g_Color_Test);

   // Photon Conversion Factor for Noise type camera
   pAct = new CPropertyAction(this, &XZellZeissCamera::OnPCF);
   propName = "Photon Conversion Factor";
   CreateFloatProperty(propName.c_str(), pcf_, false, pAct);
   SetPropertyLimits(propName.c_str(), 0.01, 10.0);

   // Read Noise (expressed in electrons) for the Noise type camera
   pAct = new CPropertyAction(this, &XZellZeissCamera::OnReadNoise);
   propName = "ReadNoise (electrons)";
   CreateFloatProperty(propName.c_str(), readNoise_, false, pAct);
   SetPropertyLimits(propName.c_str(), 0.25, 50.0);

   // Photon Flux for the Noise type camera
   pAct = new CPropertyAction(this, &XZellZeissCamera::OnPhotonFlux);
   propName = "Photon Flux";
   CreateFloatProperty(propName.c_str(), photonFlux_, false, pAct);
   SetPropertyLimits(propName.c_str(), 2.0, 5000.0);

   // Simulate application crash
   pAct = new CPropertyAction(this, &XZellZeissCamera::OnCrash);
   CreateStringProperty("SimulateCrash", "", false, pAct);
   AddAllowedValue("SimulateCrash", "");
   AddAllowedValue("SimulateCrash", "Dereference Null Pointer");
   AddAllowedValue("SimulateCrash", "Divide by Zero");

   // synchronize all properties
   // --------------------------
   nRet = UpdateStatus();
   if (nRet != DEVICE_OK)
      return nRet;


   // setup the buffer
   // ----------------
   nRet = ResizeImageBuffer();
   if (nRet != DEVICE_OK)
      return nRet;

#ifdef TESTRESOURCELOCKING
   TestResourceLocking(true);
   LogMessage("TestResourceLocking OK",true);
#endif


   initialized_ = true;




   // initialize image buffer
   GenerateEmptyImage(img_);
   return DEVICE_OK;


}

/**
* Shuts down (unloads) the device.
* Required by the MM::Device API.
* Ideally this method will completely unload the device and release all resources.
* Shutdown() may be called multiple times in a row.
* After Shutdown() we should be allowed to call Initialize() again to load the device
* without causing problems.
*/
int XZellZeissCamera::Shutdown()
{
    LogMessage("ZEISS API METHOD ENTRY: Shutdown");
   initialized_ = false;

   // START OF ZEISS SPECIFIC DEV CODE
   McammClose(0);
   LogMessage("DEV: McammClose complete");

   McammLibTerm();
   LogMessage("DEV: McammLibTerm complete");
   // END OF ZEISS SPECIFIC DEV CODE

   return DEVICE_OK;
}


int XZellZeissCamera::SnapImage()
{
    LogMessage("ZEISS API METHOD ENTRY: SnapImage");
    unsigned short* ImageBufferWithHeader = NULL;
    long imageWidth = 0;
    long imageHeight = 0;
    long imageSize = 0;

	static int callCounter = 0;
	++callCounter;

   MM::MMTime startTime = GetCurrentMMTime();
   double exp = GetExposure();
   if (sequenceRunning_ && IsCapturing()) 
   {
      exp = GetSequenceExposure();
   }

	// START OF ZEISS SPECIFIC DEV CODE
   {
       std::ostringstream oss;
       oss << "DEV: Retrieved Exposure: " << exp << "\n";
       LogMessage(oss.str().c_str());
   }

   McammSetExposure(0, static_cast<long>(exp * 1000)); // 50000 equals 50 ms

   McammGetCurrentDataSize(0, &imageWidth, &imageHeight);
   {
       std::ostringstream oss;
       oss << "DEV: Retrieved Image Width: " << imageWidth << "\n";
       oss << "DEV: Retrieved Image Height: " << imageHeight << "\n";
       LogMessage(oss.str().c_str());
   }
   McammGetCurrentImageDataSize(0, &imageSize);
   {
       std::ostringstream oss;
       oss << "DEV: Retrieved Image Size: " << imageSize << "\n";
       LogMessage(oss.str().c_str());
   }

   ImageBufferWithHeader = (unsigned short*)malloc(imageSize);
   imageSize /= 2;



   long error = McammAcquisitionEx(0, ImageBufferWithHeader, imageSize, NULL, NULL);
   //long error = McammAcquisitionEx(0, ImageBufferWithHeader, imageSize, SnapImageProgress, this);
    {
	    std::ostringstream oss;
		oss << "DEV: McammAcquisitionEx complete with error code: " << error << "\n";
		LogMessage(oss.str().c_str());
    }

   if (error == 0)
   {
       IMAGE_HEADER* imageHeader = (IMAGE_HEADER*)ImageBufferWithHeader;
       unsigned short* pixelData = ImageBufferWithHeader + imageHeader->headerSize;
       unsigned short pixelValue1 = pixelData[0];
       unsigned short pixelValue2 = pixelData[1];
       unsigned short pixelValue3 = pixelData[2];

       int imageCount = imageHeader->imageNumber;

       {
           //printf("Image Count = %d, ", imageCount);
           std::ostringstream oss;
           oss << "DEV: Image Count = " << imageCount << "\n";
           LogMessage(oss.str().c_str());
       }

       if (cameraInfos.Type == mcamRGB)
       {
           //printf("R %d - G %d - B %d \n", imageCount, pixelValue1, pixelValue2, pixelValue3);
           std::ostringstream oss;
           oss << "DEV: R " << pixelValue1 << "G " << pixelValue2 << "B " << pixelValue3 << "\n";
           LogMessage(oss.str().c_str());
       }
       else
       {
           //printf("pixels data = %d - %d - %d \n", imageCount, pixelValue1, pixelValue2, pixelValue3);
           std::ostringstream oss;
           oss << "DEV: pixels data = " << pixelValue1 << " - " << pixelValue2 << " - " << pixelValue3 << "\n";
           LogMessage(oss.str().c_str());
       }

       int headerSize = imageHeader->headerSize;
       {
           std::ostringstream oss;
           oss << "DEV: Header Size = " << headerSize << "\n";
           LogMessage(oss.str().c_str());
       }

       int pixelFormat = imageHeader->pixelFormat;
       {
           std::ostringstream oss;
           oss << "DEV: Pixel Format = " << pixelFormat << "\n";
           LogMessage(oss.str().c_str());
       }

       int bitsPerPixel = imageHeader->bitsPerPixel;
       {
           std::ostringstream oss;
           oss << "DEV: Bits per Pixel = " << bitsPerPixel << "\n";
           LogMessage(oss.str().c_str());
       }

       int bytesPerPixel = imageHeader->bytesPerPixel;
       unsigned int numerator = bytesPerPixel >> 16;  // Shift right by 16 bits to get the upper 16 bits (numerator)
       unsigned int denominator = bytesPerPixel & 0xFFFF;  // Use bitwise AND to mask the lower 16 bits

       {
           std::ostringstream oss;
           oss << "DEV: Numerator of Bytes per Pixel = " << numerator << "\n";
           oss << "DEV: Denominator of Bytes per Pixel = " << denominator << "\n";
           LogMessage(oss.str().c_str());
       }

       {
           std::ostringstream oss;
           oss << "DEV: img_.Width() = " << img_.Width() << "\n";
           oss << "DEV: img_.Height() = " << img_.Height() << "\n";
           oss << "DEV: img_.Depth() = " << img_.Depth() << "\n";
           LogMessage(oss.str().c_str());
       }
       
       memcpy(img_.GetPixelsRW(), ImageBufferWithHeader, static_cast<size_t>(img_.Width()) * img_.Height() * img_.Depth());
		
   }
   else
   {
       //printf("Error %d\n", error);
       std::ostringstream oss;
       oss << "Error: " << error << "\n";
       LogMessage(oss.str().c_str());
   }

   if (ImageBufferWithHeader != NULL)
   {
       free(ImageBufferWithHeader);
   }
	// END OF ZEISS SPECIFIC DEV CODE

   //if (!fastImage_)
   //{
   //   GenerateSyntheticImage(img_, exp);
   //}

   MM::MMTime s0(0,0);
   if( s0 < startTime )
   {
      while (exp > (GetCurrentMMTime() - startTime).getMsec())
      {
         CDeviceUtils::SleepMs(1);
      }		
   }
   else
   {
      std::cerr << "You are operating this device adapter without setting the core callback, timing functions aren't yet available" << std::endl;
      // called without the core callback probably in off line test program
      // need way to build the core in the test program

   }
   readoutStartTime_ = GetCurrentMMTime();

   return DEVICE_OK;
}


/**
* Returns pixel data.
* Required by the MM::Camera API.
* The calling program will assume the size of the buffer based on the values
* obtained from GetImageBufferSize(), which in turn should be consistent with
* values returned by GetImageWidth(), GetImageHight() and GetImageBytesPerPixel().
* The calling program allso assumes that camera never changes the size of
* the pixel buffer on its own. In other words, the buffer can change only if
* appropriate properties are set (such as binning, pixel type, etc.)
*/
const unsigned char* XZellZeissCamera::GetImageBuffer()
{
    LogMessage("ZEISS API METHOD ENTRY: GetImageBuffer");
   MMThreadGuard g(imgPixelsLock_);
   MM::MMTime readoutTime(readoutUs_);
   while (readoutTime > (GetCurrentMMTime() - readoutStartTime_)) {}		
   unsigned char *pB = (unsigned char*)(img_.GetPixels());
   return pB;
}

/**
* Returns image buffer X-size in pixels.
* Required by the MM::Camera API.
*/
unsigned XZellZeissCamera::GetImageWidth() const
{
    LogMessage("ZEISS API METHOD ENTRY: GetImageWidth");
   return img_.Width();
}

/**
* Returns image buffer Y-size in pixels.
* Required by the MM::Camera API.
*/
unsigned XZellZeissCamera::GetImageHeight() const
{
    LogMessage("ZEISS API METHOD ENTRY: GetImageHeight");
   return img_.Height();
}

/**
* Returns image buffer pixel depth in bytes.
* Required by the MM::Camera API.
*/
unsigned XZellZeissCamera::GetImageBytesPerPixel() const
{
    LogMessage("ZEISS API METHOD ENTRY: GetImageBytesPerPixel");
   return img_.Depth();
} 

/**
* Returns the bit depth (dynamic range) of the pixel.
* This does not affect the buffer size, it just gives the client application
* a guideline on how to interpret pixel values.
* Required by the MM::Camera API.
*/
unsigned XZellZeissCamera::GetBitDepth() const
{
    LogMessage("ZEISS API METHOD ENTRY: GetBitDepth");
   return bitDepth_;
}

/**
* Returns the size in bytes of the image buffer.
* Required by the MM::Camera API.
*/
long XZellZeissCamera::GetImageBufferSize() const
{
    LogMessage("ZEISS API METHOD ENTRY: GetImageBufferSize");
   return img_.Width() * img_.Height() * GetImageBytesPerPixel();
}

/**
* Sets the camera Region Of Interest.
* Required by the MM::Camera API.
* This command will change the dimensions of the image.
* Depending on the hardware capabilities the camera may not be able to configure the
* exact dimensions requested - but should try do as close as possible.
* If the hardware does not have this capability the software should simulate the ROI by
* appropriately cropping each frame.
* This demo implementation ignores the position coordinates and just crops the buffer.
* If multiple ROIs are currently set, then this method clears them in favor of
* the new ROI.
* @param x - top-left corner coordinate
* @param y - top-left corner coordinate
* @param xSize - width
* @param ySize - height
*/
int XZellZeissCamera::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize)
{
    LogMessage("ZEISS API METHOD ENTRY: SetROI");
   multiROIXs_.clear();
   multiROIYs_.clear();
   multiROIWidths_.clear();
   multiROIHeights_.clear();
   if (xSize == 0 && ySize == 0)
   {
      // effectively clear ROI
      ResizeImageBuffer();
      roiX_ = 0;
      roiY_ = 0;
   }
   else
   {
      // apply ROI
      img_.Resize(xSize, ySize);
      roiX_ = x;
      roiY_ = y;
   }
   return DEVICE_OK;
}

/**
* Returns the actual dimensions of the current ROI.
* If multiple ROIs are set, then the returned ROI should encompass all of them.
* Required by the MM::Camera API.
*/
int XZellZeissCamera::GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize)
{
    LogMessage("ZEISS API METHOD ENTRY: GetROI");
   x = roiX_;
   y = roiY_;

   xSize = img_.Width();
   ySize = img_.Height();

   return DEVICE_OK;
}

/**
* Resets the Region of Interest to full frame.
* Required by the MM::Camera API.
*/
int XZellZeissCamera::ClearROI()
{
   ResizeImageBuffer();
   roiX_ = 0;
   roiY_ = 0;
   multiROIXs_.clear();
   multiROIYs_.clear();
   multiROIWidths_.clear();
   multiROIHeights_.clear();
   return DEVICE_OK;
}

/**
 * Queries if multiple ROIs have been set (via the SetMultiROI method). Must
 * return true even if only one ROI was set via that method, but must return
 * false if an ROI was set via SetROI() or if ROIs have been cleared.
 * Optional method in the MM::Camera API; by default cameras do not support
 * multiple ROIs, so this method returns false.
 */
bool XZellZeissCamera::IsMultiROISet()
{
    LogMessage("ZEISS API METHOD ENTRY: IsMultiROISet");
   return multiROIXs_.size() > 0;
}

/**
* Returns the current exposure setting in milliseconds.
* Required by the MM::Camera API.
*/
double XZellZeissCamera::GetExposure() const
{
    LogMessage("ZEISS API METHOD ENTRY: GetExposure");
   char buf[MM::MaxStrLength];
   int ret = GetProperty(MM::g_Keyword_Exposure, buf);
   if (ret != DEVICE_OK)
      return 0.0;
   return atof(buf);
}

/**
 * Returns the current exposure from a sequence and increases the sequence counter
 * Used for exposure sequences
 */
double XZellZeissCamera::GetSequenceExposure() 
{
   if (exposureSequence_.size() == 0) 
      return this->GetExposure();

   double exposure = exposureSequence_[sequenceIndex_];

   sequenceIndex_++;
   if (sequenceIndex_ >= exposureSequence_.size())
      sequenceIndex_ = 0;

   return exposure;
}

/**
* Sets exposure in milliseconds.
* Required by the MM::Camera API.
*/
void XZellZeissCamera::SetExposure(double exp)
{
    LogMessage("ZEISS API METHOD ENTRY: SetExposure");
   SetProperty(MM::g_Keyword_Exposure, CDeviceUtils::ConvertToString(exp));
   GetCoreCallback()->OnExposureChanged(this, exp);;
}

/**
* Returns the current binning factor.
* Required by the MM::Camera API.
*/
int XZellZeissCamera::GetBinning() const
{
    LogMessage("ZEISS API METHOD ENTRY: GetBinning");
   char buf[MM::MaxStrLength];
   int ret = GetProperty(MM::g_Keyword_Binning, buf);
   if (ret != DEVICE_OK)
      return 1;
   return atoi(buf);
}

/**
* Sets binning factor.
* Required by the MM::Camera API.
*/
int XZellZeissCamera::SetBinning(int binF)
{
    LogMessage("ZEISS API METHOD ENTRY: SetBinning");
   return SetProperty(MM::g_Keyword_Binning, CDeviceUtils::ConvertToString(binF));
}

int XZellZeissCamera::SetAllowedBinning() 
{
   std::vector<std::string> binValues;
   binValues.push_back("1");
   binValues.push_back("2");
   //if (scanMode_ < 3)
   //   binValues.push_back("4");
   //if (scanMode_ < 2)
   //   binValues.push_back("8");
   //if (binSize_ == 8 && scanMode_ == 3) {
   //   SetProperty(MM::g_Keyword_Binning, "2");
   //} else if (binSize_ == 8 && scanMode_ == 2) {
   //   SetProperty(MM::g_Keyword_Binning, "4");
   //} else if (binSize_ == 4 && scanMode_ == 3) {
   //   SetProperty(MM::g_Keyword_Binning, "2");
   //}
      
   LogMessage("Setting Allowed Binning settings", true);
   return SetAllowedValues(MM::g_Keyword_Binning, binValues);
}

BOOL LiveCallback(unsigned short* img, long bytesize, long currbufnr, LONGLONG FrameTime, void* UserParam)
{
    int lockNumber = 0;
    bool locked = false;

    for (int i = 0; i < 2; i++)
    {
        locked = bufferMutex[i].try_lock();

        if (locked)
        {
            lockNumber = i;
            break;
        }
    }

    if (locked)
    {
        memcpy(ImageBufferWithHeader[lockNumber], img, bytesize);
        imageCount++;
        imageNumber[lockNumber] = imageCount;

        bufferMutex[lockNumber].unlock();

        // CONDITION VARIABLE IMPLEMENTATION START
        // Notify the main loop that a new image is available
        {
            std::lock_guard<std::mutex> lock(mtx);
            newImageAvailable = true;
        }
        cv.notify_one();  // Notify the main loop
        // CONDITION VARIABLE IMPLEMENTATION END
    }

    return true;
}


/**
 * Required by the MM::Camera API
 * Please implement this yourself and do not rely on the base class implementation
 * The Base class implementation is deprecated and will be removed shortly
 */
int XZellZeissCamera::StartSequenceAcquisition(double interval)
{
    LogMessage("ZEISS API METHOD ENTRY: StartSequenceAcquisition(interval)");
   return StartSequenceAcquisition(LONG_MAX, interval, false);
}

/**
* Simple implementation of Sequence Acquisition
* A sequence acquisition should run on its own thread and transport new images
* coming of the camera into the MMCore circular buffer.
*/
int XZellZeissCamera::StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow)
{
    LogMessage("ZEISS API METHOD ENTRY: StartSequenceAcquisition(numImages, interval_ms, stopOnOverflow)");
   if (IsCapturing())
      return DEVICE_CAMERA_BUSY_ACQUIRING;

   int ret = GetCoreCallback()->PrepareForAcq(this);
   if (ret != DEVICE_OK)
      return ret;
   sequenceStartTime_ = GetCurrentMMTime();
   imageCounter_ = 0;

   // START OF ZEISS SPECIFIC DEV CODE
   double exp = GetExposure();
   if (processImageInSDK)
   {
       McammSetCameraBuffering(0, false);
       McammSetColorMatrixOptimizationMode(0, mcammAllPipelineStage);
   }
   else
   {
       McammSetColorMatrixOptimizationMode(0, mcammNoOptimization);
   }

   McammSetExposure(0, static_cast<long>(exp * 1000)); // 50000 equals 50 ms

   long imageSize = 0;
   McammGetMaxImageDataSize(0, &imageSize);
   {
       std::ostringstream oss;
       oss << "DEV: Retrieved Image Size: " << imageSize << "\n";
       LogMessage(oss.str().c_str());
   }

   ImageBufferWithHeader[0] = (unsigned short*)malloc(imageSize);
   ImageBufferWithHeader[1] = (unsigned short*)malloc(imageSize);
   processedImage = (unsigned short*)malloc(imageSize);

   imageNumber[0] = -1;
   imageNumber[1] = -1;

   long error = McammGetIPInfo(0, &pContext, &contextSize, &LiveCallback, &pimageByteSize);
   {
       std::ostringstream oss;
       oss << "DEV: McammGetIPInfo called with Error Code " << error << "\n";
       LogMessage(oss.str().c_str());
   }
   {
       std::ostringstream oss;
       oss << "DEV: Context Data: " << pContext << "\n";
       LogMessage(oss.str().c_str());
   }
   {
       std::ostringstream oss;
       oss << "DEV: Context Size: " << contextSize << "\n";
       LogMessage(oss.str().c_str());
   }
   {
       std::ostringstream oss;
       oss << "DEV: Image Byte Size: " << pimageByteSize << "\n";
       LogMessage(oss.str().c_str());
   }
   error = McammStartContinuousAcquisition(0, 15, NULL);
   // END OF ZEISS SPECIFIC DEV CODE  

   LogMessage("ZEISS API: StartSequenceAcquisition calling thd_->Start");
   thd_->Start(numImages,interval_ms);
   stopOnOverflow_ = stopOnOverflow;
   return DEVICE_OK;
}

/**                                                                       
* Stop and wait for the Sequence thread finished                                   
*/                                                                        
int XZellZeissCamera::StopSequenceAcquisition()                                     
{
    LogMessage("ZEISS API METHOD ENTRY: StopSequenceAcquisition");


    // START OF ZEISS SPECIFIC DEV CODE
    error = McammStopContinuousAcquisition(0);
    // END OF ZEISS SPECIFIC DEV CODE   

   if (!thd_->IsStopped()) {
      thd_->Stop();                                                       
      //thd_->wait();                                                       
   }

    // START OF ZEISS SPECIFIC DEV CODE
    free(ImageBufferWithHeader[0]);
    free(ImageBufferWithHeader[1]);
    free(processedImage);
    // END OF ZEISS SPECIFIC DEV CODE                                                                      
                                                                          
   return DEVICE_OK;                                                      
} 

/*
 * Inserts Image and MetaData into MMCore circular Buffer
 */
int XZellZeissCamera::InsertImage()
{
   MM::MMTime timeStamp = this->GetCurrentMMTime();
   char label[MM::MaxStrLength];
   this->GetLabel(label);
 
   // Important:  metadata about the image are generated here:
   Metadata md;
   md.put("Camera", label);
   md.put(MM::g_Keyword_Elapsed_Time_ms, CDeviceUtils::ConvertToString((timeStamp - sequenceStartTime_).getMsec()));
   md.put(MM::g_Keyword_Metadata_ROI_X, CDeviceUtils::ConvertToString( (long) roiX_)); 
   md.put(MM::g_Keyword_Metadata_ROI_Y, CDeviceUtils::ConvertToString( (long) roiY_)); 

   imageCounter_++;

   char buf[MM::MaxStrLength];
   GetProperty(MM::g_Keyword_Binning, buf);
   md.put(MM::g_Keyword_Binning, buf);

   MMThreadGuard g(imgPixelsLock_);

   const unsigned char* pI = GetImageBuffer();
   unsigned int w = GetImageWidth();
   unsigned int h = GetImageHeight();
   unsigned int b = GetImageBytesPerPixel();

   int ret = GetCoreCallback()->InsertImage(this, pI, w, h, b, nComponents_, md.Serialize().c_str());
   if (!stopOnOverflow_ && ret == DEVICE_BUFFER_OVERFLOW)
   {
      // do not stop on overflow - just reset the buffer
      GetCoreCallback()->ClearImageBuffer(this);
      // don't process this same image again...
      return GetCoreCallback()->InsertImage(this, pI, w, h, b, nComponents_, md.Serialize().c_str(), false);
   }
   else
   {
      return ret;
   }
}

/*
 * Do actual capturing
 * Called from inside the thread  
 */
int XZellZeissCamera::RunSequenceOnThread()
{
   int ret=DEVICE_ERR;
   MM::MMTime startTime = GetCurrentMMTime();
   
   // Trigger
   if (triggerDevice_.length() > 0) {
      MM::Device* triggerDev = GetDevice(triggerDevice_.c_str());
      if (triggerDev != 0) {
      	LogMessage("trigger requested");
      	triggerDev->SetProperty("Trigger","+");
      }
   }

   double exposure = GetSequenceExposure();

   if (!fastImage_)
   {
      GenerateSyntheticImage(img_, exposure);
   }

   // Simulate exposure duration
   while ((GetCurrentMMTime() - startTime).getMsec() < exposure)
   {
      CDeviceUtils::SleepMs(1);
   }

   ret = InsertImage();

   if (ret != DEVICE_OK)
   {
      return ret;
   }
   return ret;
};

// START OF ZEISS SPECIFIC DEV CODE
int XZellZeissCamera::CaptureImage(void)
{
    LogMessage("ZEISS API METHOD ENTRY: CaptureImage");
    int ret = DEVICE_ERR;

    ret = SnapImage();
    if (ret != DEVICE_OK)
    {
        return ret;
    }

    ret = InsertImage();
    return ret;
};
// END OF ZEISS SPECIFIC DEV CODE

// START OF ZEISS SPECIFIC DEV CODE
int XZellZeissCamera::MoveImageToCircularBuffer()
{
    LogMessage("ZEISS API METHOD ENTRY: MoveImageToCircularBuffer");

    // START OF SPINNAKER STYLE CODE
    if (!IsCapturing())
    {
        LogMessage("DEV: Camera is not capturing! Cannot retrieve image!");
        //SetErrorText(SPKR_ERROR, "Camera is not capturing! Cannot retrieve image!");
        return DEVICE_ERR;
    }

    //SPKR::ImagePtr ip =
    //    this->GetNextImage((int)this->GetExposure() + 1000);
    // END OF SPINNAKER STYLE CODE

    // START OF ZEISS SPECIFIC DEV CODE
    double exp = GetExposure();
    if (sequenceRunning_ && IsCapturing())
    {
        exp = GetSequenceExposure();
    }

    {
        std::ostringstream oss;
        oss << "DEV: Retrieved Exposure: " << exp << "\n";
        LogMessage(oss.str().c_str());
    }
    MMThreadGuard g(imgPixelsLock_);

    
    // CONDITION VARIABLE IMPLEMENTATION START
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [] { return newImageAvailable; });  // Wait for a new image
    // CONDITION VARIABLE IMPLEMENTATION END

    int lockNum = -1;
    unsigned short* bufferWithHeader = NULL;
    bool newImage = false;

    for (int k = 0; k < 2; k++) {

        {
            std::ostringstream oss;
            oss << "DEV: XZellZeissCamera::MoveImageToCircularBuffer k loop, k: " << k << "\n";
            LogMessage(oss.str().c_str());
        }
        lockNum++;
        {
            std::ostringstream oss;
            oss << "DEV: XZellZeissCamera::MoveImageToCircularBuffer - imageNumber[lockNum]: " << imageNumber[lockNum] << "\n";
            oss << "DEV: XZellZeissCamera::MoveImageToCircularBuffer - lastImageRead: " << lastImageRead << "\n";
            LogMessage(oss.str().c_str());
        }
        if (imageNumber[lockNum] > lastImageRead) {
            LogMessage("DEV: XZellZeissCamera::MoveImageToCircularBuffer -  imageNumber[lockNum] > lastImageRead");
            bool locked = bufferMutex[lockNum].try_lock();

            if (locked)
            {
                LogMessage("DEV: XZellZeissCamera::MoveImageToCircularBuffer locked -  bufferWithHeader = ImageBufferWithHeader[lockNum]");
                {
                    std::ostringstream oss;
                    oss << "DEV: XZellZeissCamera::MoveImageToCircularBuffer lockNum: " << lockNum << "\n";
                    LogMessage(oss.str().c_str());
                }
                bufferWithHeader = ImageBufferWithHeader[lockNum];
                lastImageRead = imageNumber[lockNum];
                newImage = true;
                // CONDITION VARIABLE IMPLEMENTATION START
                bufferMutex[lockNum].unlock();  // Unlock after accessing the buffer
                // CONDITION VARIABLE IMPLEMENTATION END
                break;
            }
        }
    }

    {
        std::ostringstream oss;
        oss << "DEV: XZellZeissCamera::MoveImageToCircularBuffer - newImage: " << newImage << "\n";
        LogMessage(oss.str().c_str());
    }

    // CONDITION VARIABLE IMPLEMENTATION START
    newImageAvailable = false;  // Reset the flag
    // CONDITION VARIABLE IMPLEMENTATION END

    if (newImage)
    {
        if (!processImageInSDK)
        {
            error = McammExecuteIPFunction(pContext, processedImage, bufferWithHeader);
            bufferWithHeader = processedImage;
        }

        IMAGE_HEADER* imageHeader = (IMAGE_HEADER*)bufferWithHeader;
        unsigned short* pixelData = bufferWithHeader + imageHeader->headerSize;
        unsigned short pixelValue1 = pixelData[0];
        unsigned short pixelValue2 = pixelData[1];
        unsigned short pixelValue3 = pixelData[2];

        {
            std::ostringstream oss;
            oss << "DEV:pixels data = " << imageCount << ": " << pixelValue1 << " - " << pixelValue2 << " - " << pixelValue3;
            LogMessage(oss.str().c_str());
	        
        }

        //bufferMutex[lockNum].unlock();

		// TODO HOW DOES THE ABOVE ZEISS EXAMPLE CODE COMBINE WITH THE INSERTIMAGE CODE BELOW?

		const unsigned char* imgBuf = reinterpret_cast<const unsigned char*>(pixelData);
        memcpy(img_.GetPixelsRW(),
            imgBuf,
            img_.Width() * img_.Height() * img_.Depth());
        
	    //const unsigned char* imgBuf = GetImageBuffer();
	    unsigned int w = GetImageWidth();
	    unsigned int h = GetImageHeight();
	    unsigned int b = GetImageBytesPerPixel();

	    int ret = GetCoreCallback()->InsertImage(this, imgBuf, w, h, b);
	    //int ret = GetCoreCallback()->InsertImage(this, imgBuf, w, h, b, md.Serialize().c_str());

	    if (!stopOnOverflow_ && ret == DEVICE_BUFFER_OVERFLOW)
	    {
	        // do not stop on overflow - just reset the buffer
	        GetCoreCallback()->ClearImageBuffer(this);
	        // don't process this same image again...
	        ret = GetCoreCallback()->InsertImage(this, imgBuf, w, h, b);
	        //ret = GetCoreCallback()->InsertImage(this, imgBuf, w, h, b, md.Serialize().c_str(), false);
	    }
	    if (ret == DEVICE_OK)
	        imageCounter_++;
        return ret;
    }


    return DEVICE_CAMERA_BUSY_ACQUIRING;// TODO: how to better deal with this

    //// END OF ZEISS SPECIFIC DEV CODE

    return DEVICE_OK;
}
//// END OF ZEISS SPECIFIC DEV CODE

bool XZellZeissCamera::IsCapturing() {
    LogMessage("ZEISS API METHOD ENTRY: IsCapturing");
   return !thd_->IsStopped();
}

/*
 * called from the thread function before exit 
 */
void XZellZeissCamera::OnThreadExiting() throw()
{
    LogMessage("ZEISS INNER METHOD ENTRY: OnThreadExiting");
   try
   {
      LogMessage(g_Msg_SEQUENCE_ACQUISITION_THREAD_EXITING);
      GetCoreCallback()?GetCoreCallback()->AcqFinished(this,0):DEVICE_OK;
   }
   catch(...)
   {
      LogMessage(g_Msg_EXCEPTION_IN_ON_THREAD_EXITING, false);
   }
}


ZeissAcquisitionThread::ZeissAcquisitionThread(XZellZeissCamera* pCam)
   :intervalMs_(default_intervalMS)
   ,numImages_(default_numImages)
   ,imageCounter_(0)
   ,stop_(true)
   ,suspend_(false)
   ,camera_(pCam)
   ,startTime_(0)
   ,actualDuration_(0)
   ,lastFrameTime_(0)
{};

ZeissAcquisitionThread::~ZeissAcquisitionThread() {};

void ZeissAcquisitionThread::Stop() {
   MMThreadGuard g(this->stopLock_);
   stop_=true;
}

void ZeissAcquisitionThread::Start(long numImages, double intervalMs)
{
   MMThreadGuard g1(this->stopLock_);
   MMThreadGuard g2(this->suspendLock_);
   numImages_=numImages;
   intervalMs_=intervalMs;
   imageCounter_=0;
   stop_ = false;
   suspend_=false;
   camera_->LogMessage("ZEISS API: ZeissAcquisitionThread::Start calling activate");
   activate();
   actualDuration_ = MM::MMTime{};
   startTime_= camera_->GetCurrentMMTime();
   lastFrameTime_ = MM::MMTime{};
}

bool ZeissAcquisitionThread::IsStopped(){
   MMThreadGuard g(this->stopLock_);
   return stop_;
}

void ZeissAcquisitionThread::Suspend() {
   MMThreadGuard g(this->suspendLock_);
   suspend_ = true;
}

bool ZeissAcquisitionThread::IsSuspended() {
   MMThreadGuard g(this->suspendLock_);
   return suspend_;
}

void ZeissAcquisitionThread::Resume() {
   MMThreadGuard g(this->suspendLock_);
   suspend_ = false;
}

int ZeissAcquisitionThread::svc(void) throw()
{
    camera_->LogMessage("ZEISS INNER METHOD ENTRY: svc");
   int ret=DEVICE_ERR;
   try 
   {
      do
      {
          camera_->LogMessage("DEV: ZeissAcquisitionThread::svc do / while loop\n");
         //ret = camera_->CaptureImage();
         ret = camera_->MoveImageToCircularBuffer();
      } while (DEVICE_OK == ret && !IsStopped() && imageCounter_++ < numImages_-1);
      if (IsStopped())
         camera_->LogMessage("SeqAcquisition interrupted by the user\n");
   }catch(...){
      camera_->LogMessage(g_Msg_EXCEPTION_IN_THREAD, false);
   }
   stop_=true;
   actualDuration_ = camera_->GetCurrentMMTime() - startTime_;
   camera_->OnThreadExiting();
   return ret;
}


///////////////////////////////////////////////////////////////////////////////
// XZellZeissCamera Action handlers
///////////////////////////////////////////////////////////////////////////////

int XZellZeissCamera::OnMaxExposure(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(exposureMaximum_);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(exposureMaximum_);
   }
   return DEVICE_OK;
}


/*
* this Read Only property will update whenever any property is modified
*/

int XZellZeissCamera::OnTestProperty(MM::PropertyBase* pProp, MM::ActionType eAct, long indexx)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(testProperty_[indexx]);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(testProperty_[indexx]);
   }
	return DEVICE_OK;

}

void XZellZeissCamera::SlowPropUpdate(std::string leaderValue)
{
      // wait in order to simulate a device doing something slowly
      // in a thread
      long delay; GetProperty("AsyncPropertyDelayMS", delay);
      CDeviceUtils::SleepMs(delay);
      {
         MMThreadGuard g(asyncFollowerLock_);
         asyncFollower_ = leaderValue;
      }
      OnPropertyChanged("AsyncPropertyFollower", leaderValue.c_str());
   }

int XZellZeissCamera::OnAsyncFollower(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet){
      MMThreadGuard g(asyncFollowerLock_);
      pProp->Set(asyncFollower_.c_str());
   }
   // no AfterSet as this is a readonly property
   return DEVICE_OK;
}

int XZellZeissCamera::OnAsyncLeader(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet){
      pProp->Set(asyncLeader_.c_str());
   }
   if (eAct == MM::AfterSet)
   {
      pProp->Get(asyncLeader_);
      fut_ = std::async(std::launch::async, &XZellZeissCamera::SlowPropUpdate, this, asyncLeader_);
   }
	return DEVICE_OK;
}

/**
* Handles "Binning" property.
*/
int XZellZeissCamera::OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    LogMessage("ZEISS METHOD ENTRY: OnBinning");
   int ret = DEVICE_ERR;
   switch(eAct)
   {
   case MM::AfterSet:
      {
       LogMessage("ZEISS METHOD ENTRY: OnBinning - MM::AfterSet");
         if(IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

         // START OF ZEISS SPECIFIC DEV CODE
         long binSize;
         pProp->Get(binSize);
         binSize_ = (int)binSize;

         McammSetBinning(0, binSize);

         {
             std::ostringstream oss;  // New oss, different scope
             oss << "DEV: McammSetBinning to " << binSize << "\n";
             LogMessage(oss.str().c_str());
         }

         ret = ResizeImageBuffer();

         //// the user just set the new value for the property, so we have to
         //// apply this value to the 'hardware'.
         //long binFactor;
         //pProp->Get(binFactor);
         //if(binFactor > 0 && binFactor < 3)
         //{
         //   // calculate ROI using the previous bin settings
         //   double factor = (double) binFactor / (double) binSize_;
         //   roiX_ = (unsigned int) (roiX_ / factor);
         //   roiY_ = (unsigned int) (roiY_ / factor);
         //   for (unsigned int i = 0; i < multiROIXs_.size(); ++i)
         //   {
         //      multiROIXs_[i]  = (unsigned int) (multiROIXs_[i] / factor);
         //      multiROIYs_[i] = (unsigned int) (multiROIYs_[i] / factor);
         //      multiROIWidths_[i] = (unsigned int) (multiROIWidths_[i] / factor);
         //      multiROIHeights_[i] = (unsigned int) (multiROIHeights_[i] / factor);
         //   }
         //   img_.Resize( (unsigned int) (img_.Width()/factor), 
         //                  (unsigned int) (img_.Height()/factor) );
         //   binSize_ = binFactor;
         //   std::ostringstream os;
         //   os << binSize_;
         //   OnPropertyChanged("Binning", os.str().c_str());
         //   ret=DEVICE_OK;
         //}
      }break;
   case MM::BeforeGet:
      {
       LogMessage("ZEISS METHOD ENTRY: OnBinning - MM::BeforeGet");
         ret=DEVICE_OK;
			pProp->Set(binSize_);
      }break;
   default:
      break;
   }
   return ret; 
}

/**
* Handles "PixelType" property.
*/
int XZellZeissCamera::OnPixelType(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int ret = DEVICE_ERR;
   switch(eAct)
   {
   case MM::AfterSet:
      {
         if(IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

         std::string pixelType;
         pProp->Get(pixelType);

         if (pixelType.compare(g_PixelType_8bit) == 0)
         {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 1);
            bitDepth_ = 8;
            ret=DEVICE_OK;
         }
         else if (pixelType.compare(g_PixelType_16bit) == 0)
         {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 2);
            bitDepth_ = 16;
            ret=DEVICE_OK;
         }
         else
         {
            // on error switch to default pixel type
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 1);
            pProp->Set(g_PixelType_8bit);
            bitDepth_ = 8;
            ret = ERR_UNKNOWN_MODE;
         }
      }
      break;
   case MM::BeforeGet:
      {
         long bytesPerPixel = GetImageBytesPerPixel();
         if (bytesPerPixel == 1)
         {
         	pProp->Set(g_PixelType_8bit);
         }
         else if (bytesPerPixel == 2)
         {
         	pProp->Set(g_PixelType_16bit);
         }
		 else
         {
            pProp->Set(g_PixelType_8bit);
         }
         ret = DEVICE_OK;
      } break;
   default:
      break;
   }
   return ret; 
}

/**
* Handles "BitDepth" property.
*/
int XZellZeissCamera::OnBitDepth(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int ret = DEVICE_ERR;
   switch(eAct)
   {
   case MM::AfterSet:
      {
         if(IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

         long bitDepth;
         pProp->Get(bitDepth);

			unsigned int bytesPerComponent;

         switch (bitDepth) {
            case 8:
					bytesPerComponent = 1;
               bitDepth_ = 8;
               ret=DEVICE_OK;
            break;
            case 10:
					bytesPerComponent = 2;
               bitDepth_ = 10;
               ret=DEVICE_OK;
            break;
            case 11:
                    bytesPerComponent = 2;
                bitDepth_ = 11;
                ret = DEVICE_OK;
            break;
            case 12:
					bytesPerComponent = 2;
               bitDepth_ = 12;
               ret=DEVICE_OK;
            break;
            case 14:
					bytesPerComponent = 2;
               bitDepth_ = 14;
               ret=DEVICE_OK;
            break;
            case 16:
					bytesPerComponent = 2;
               bitDepth_ = 16;
               ret=DEVICE_OK;
            break;
            case 32:
               bytesPerComponent = 4;
               bitDepth_ = 32; 
               ret=DEVICE_OK;
            break;
            default: 
               // on error switch to default pixel type
					bytesPerComponent = 1;

               pProp->Set((long)8);
               bitDepth_ = 8;
               ret = ERR_UNKNOWN_MODE;
            break;
         }
			char buf[MM::MaxStrLength];
			GetProperty(MM::g_Keyword_PixelType, buf);
			std::string pixelType(buf);
			unsigned int bytesPerPixel = 1;
			

         // automagickally change pixel type when bit depth exceeds possible value
         if (pixelType.compare(g_PixelType_8bit) == 0)
         {
				if( 2 == bytesPerComponent)
				{
					SetProperty(MM::g_Keyword_PixelType, g_PixelType_16bit);
					bytesPerPixel = 2;
				}
				else if ( 4 == bytesPerComponent)
            {
					SetProperty(MM::g_Keyword_PixelType, g_PixelType_32bit);
					bytesPerPixel = 4;

            }else
				{
				   bytesPerPixel = 1;
				}
         }
         else if (pixelType.compare(g_PixelType_16bit) == 0)
         {
				bytesPerPixel = 2;
         }
			img_.Resize(img_.Width(), img_.Height(), bytesPerPixel);

      } break;
   case MM::BeforeGet:
      {
         pProp->Set((long)bitDepth_);
         ret=DEVICE_OK;
      } break;
   default:
      break;
   }
   return ret; 
}
/**
* Handles "ReadoutTime" property.
*/
int XZellZeissCamera::OnReadoutTime(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      double readoutMs;
      pProp->Get(readoutMs);

      readoutUs_ = readoutMs * 1000.0;
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set(readoutUs_ / 1000.0);
   }

   return DEVICE_OK;
}

int XZellZeissCamera::OnDropPixels(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      long tvalue = 0;
      pProp->Get(tvalue);
		dropPixels_ = (0==tvalue)?false:true;
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set(dropPixels_?1L:0L);
   }

   return DEVICE_OK;
}

int XZellZeissCamera::OnFastImage(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      long tvalue = 0;
      pProp->Get(tvalue);
		fastImage_ = (0==tvalue)?false:true;
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set(fastImage_?1L:0L);
   }

   return DEVICE_OK;
}

int XZellZeissCamera::OnSaturatePixels(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      long tvalue = 0;
      pProp->Get(tvalue);
		saturatePixels_ = (0==tvalue)?false:true;
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set(saturatePixels_?1L:0L);
   }

   return DEVICE_OK;
}

int XZellZeissCamera::OnFractionOfPixelsToDropOrSaturate(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      double tvalue = 0;
      pProp->Get(tvalue);
		fractionOfPixelsToDropOrSaturate_ = tvalue;
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set(fractionOfPixelsToDropOrSaturate_);
   }

   return DEVICE_OK;
}

int XZellZeissCamera::OnShouldRotateImages(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      long tvalue = 0;
      pProp->Get(tvalue);
      shouldRotateImages_ = (tvalue != 0);
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set((long) shouldRotateImages_);
   }

   return DEVICE_OK;
}

int XZellZeissCamera::OnShouldDisplayImageNumber(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      long tvalue = 0;
      pProp->Get(tvalue);
      shouldDisplayImageNumber_ = (tvalue != 0);
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set((long) shouldDisplayImageNumber_);
   }

   return DEVICE_OK;
}

int XZellZeissCamera::OnStripeWidth(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      pProp->Get(stripeWidth_);
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set(stripeWidth_);
   }

   return DEVICE_OK;
}

int XZellZeissCamera::OnMultiROIFillValue(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      long tvalue = 0;
      pProp->Get(tvalue);
      multiROIFillValue_ = (int) tvalue;
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set((long) multiROIFillValue_);
   }

   return DEVICE_OK;
}

/*
* Handles "ScanMode" property.
* Changes allowed Binning values to test whether the UI updates properly
*/
int XZellZeissCamera::OnScanMode(MM::PropertyBase* pProp, MM::ActionType eAct)
{ 
   if (eAct == MM::AfterSet) {
      pProp->Get(scanMode_);
      SetAllowedBinning();
      if (initialized_) {
         int ret = OnPropertiesChanged();
         if (ret != DEVICE_OK)
            return ret;
      }
   } else if (eAct == MM::BeforeGet) {
      LogMessage("Reading property ScanMode", true);
      pProp->Set(scanMode_);
   }
   return DEVICE_OK;
}




int XZellZeissCamera::OnCameraCCDXSize(MM::PropertyBase* pProp , MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
		pProp->Set(cameraCCDXSize_);
   }
   else if (eAct == MM::AfterSet)
   {
      long value;
      pProp->Get(value);
		if ( (value < 16) || (33000 < value))
			return DEVICE_ERR;  // invalid image size
		if( value != cameraCCDXSize_)
		{
			cameraCCDXSize_ = value;
			img_.Resize(cameraCCDXSize_/binSize_, cameraCCDYSize_/binSize_);
		}
   }
	return DEVICE_OK;

}

int XZellZeissCamera::OnCameraCCDYSize(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
		pProp->Set(cameraCCDYSize_);
   }
   else if (eAct == MM::AfterSet)
   {
      long value;
      pProp->Get(value);
		if ( (value < 16) || (33000 < value))
			return DEVICE_ERR;  // invalid image size
		if( value != cameraCCDYSize_)
		{
			cameraCCDYSize_ = value;
			img_.Resize(cameraCCDXSize_/binSize_, cameraCCDYSize_/binSize_);
		}
   }
	return DEVICE_OK;

}

int XZellZeissCamera::OnTriggerDevice(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(triggerDevice_.c_str());
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(triggerDevice_);
   }
   return DEVICE_OK;
}


int XZellZeissCamera::OnCCDTemp(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(ccdT_);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(ccdT_);
   }
   return DEVICE_OK;
}

int XZellZeissCamera::OnIsSequenceable(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   std::string val = "Yes";
   if (eAct == MM::BeforeGet)
   {
      if (!isSequenceable_) 
      {
         val = "No";
      }
      pProp->Set(val.c_str());
   }
   else if (eAct == MM::AfterSet)
   {
      isSequenceable_ = false;
      pProp->Get(val);
      if (val == "Yes") 
      {
         isSequenceable_ = true;
      }
   }

   return DEVICE_OK;
}


int XZellZeissCamera::OnMode(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   std::string val;
   if (eAct == MM::BeforeGet)
   {
      switch (mode_)
      {
         case MODE_ARTIFICIAL_WAVES:
            val = g_Sine_Wave;
            break;
         case MODE_NOISE:
            val = g_Norm_Noise;
            break;
         case MODE_COLOR_TEST:
            val = g_Color_Test;
            break;
         default:
            val = g_Sine_Wave;
            break;
      }
      pProp->Set(val.c_str());
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(val);
      if (val == g_Norm_Noise)
      {
         mode_ = MODE_NOISE;
      }
      else if (val == g_Color_Test)
      {
         mode_ = MODE_COLOR_TEST;
      }
      else
      {
         mode_ = MODE_ARTIFICIAL_WAVES;
      }
   }
   return DEVICE_OK;
}

int XZellZeissCamera::OnPCF(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(pcf_);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(pcf_);
   }
   return DEVICE_OK;
}

int XZellZeissCamera::OnPhotonFlux(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(photonFlux_);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(photonFlux_);
   }
   return DEVICE_OK;
}

int XZellZeissCamera::OnReadNoise(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(readNoise_);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(readNoise_);
   }
   return DEVICE_OK;
}


int XZellZeissCamera::OnCrash(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   AddAllowedValue("SimulateCrash", "");
   AddAllowedValue("SimulateCrash", "Dereference Null Pointer");
   AddAllowedValue("SimulateCrash", "Divide by Zero");
   if (eAct == MM::BeforeGet)
   {
      pProp->Set("");
   }
   else if (eAct == MM::AfterSet)
   {
      std::string choice;
      pProp->Get(choice);
      if (choice == "Dereference Null Pointer")
      {
         int* p = 0;
         volatile int i = *p;
         i++;
      }
      else if (choice == "Divide by Zero")
      {
         volatile int i = 1, j = 0, k;
         k = i / j;
      }
   }
   return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Private XZellZeissCamera methods
///////////////////////////////////////////////////////////////////////////////

/**
* Sync internal image buffer size to the chosen property values.
*/
int XZellZeissCamera::ResizeImageBuffer()
{
   char buf[MM::MaxStrLength];
   //int ret = GetProperty(MM::g_Keyword_Binning, buf);
   //if (ret != DEVICE_OK)
   //   return ret;
   //binSize_ = atol(buf);

   int ret = GetProperty(MM::g_Keyword_PixelType, buf);
   if (ret != DEVICE_OK)
      return ret;

	std::string pixelType(buf);
	int byteDepth = 0;

   if (pixelType.compare(g_PixelType_8bit) == 0)
   {
      byteDepth = 1;
   }
   else if (pixelType.compare(g_PixelType_16bit) == 0)
   {
      byteDepth = 2;
   }

   img_.Resize(cameraCCDXSize_/binSize_, cameraCCDYSize_/binSize_, byteDepth);
   return DEVICE_OK;
}

void XZellZeissCamera::GenerateEmptyImage(ImgBuffer& img)
{
   MMThreadGuard g(imgPixelsLock_);
   if (img.Height() == 0 || img.Width() == 0 || img.Depth() == 0)
      return;
   unsigned char* pBuf = const_cast<unsigned char*>(img.GetPixels());
   {
       std::ostringstream oss;
       oss << "DEV: GenerateEmptyImage img_.Width() = " << img_.Width() << "\n";
       oss << "DEV: GenerateEmptyImage img_.Height() = " << img_.Height() << "\n";
       oss << "DEV: GenerateEmptyImage img_.Depth() = " << img_.Depth() << "\n";
       LogMessage(oss.str().c_str());
   }
   memset(pBuf, 0, img.Height()*img.Width()*img.Depth());
}



/**
* Generates an image.
*
* Options:
* 1. a spatial sine wave.
* 2. Gaussian noise
*/
void XZellZeissCamera::GenerateSyntheticImage(ImgBuffer& img, double exp)
{
  
   MMThreadGuard g(imgPixelsLock_);

   if (mode_ == MODE_NOISE)
   {
      double max = 1 << GetBitDepth();
      int offset = 10;
      if (max > 256)
      {
         offset = 100;
      }
	   double readNoiseDN = readNoise_ / pcf_;
      AddBackgroundAndNoise(img, offset, readNoiseDN);
      AddSignal (img, photonFlux_, exp, pcf_);
      if (imgManpl_ != 0)
      {
         imgManpl_->ChangePixels(img);
      }
      return;
   }
   else if (mode_ == MODE_COLOR_TEST)
   {
      if (GenerateColorTestPattern(img))
         return;
   }

	//std::string pixelType;
	char buf[MM::MaxStrLength];
   GetProperty(MM::g_Keyword_PixelType, buf);
   std::string pixelType(buf);

	if (img.Height() == 0 || img.Width() == 0 || img.Depth() == 0)
      return;

   double lSinePeriod = 3.14159265358979 * stripeWidth_;
   unsigned imgWidth = img.Width();
   unsigned int* rawBuf = (unsigned int*) img.GetPixelsRW();
   double maxDrawnVal = 0;
   long lPeriod = (long) imgWidth / 2;
   double dLinePhase = 0.0;
   const double dAmp = exp;
   double cLinePhaseInc = 2.0 * lSinePeriod / 4.0 / img.Height();
   if (shouldRotateImages_) {
      // Adjust the angle of the sin wave pattern based on how many images
      // we've taken, to increase the period (i.e. time between repeat images).
      cLinePhaseInc *= (((int) dPhase_ / 6) % 24) - 12;
   }

   static bool debugRGB = false;
#ifdef TIFFDEMO
	debugRGB = true;
#endif
   static  unsigned char* pDebug  = NULL;
   static unsigned long dbgBufferSize = 0;
   static long iseq = 1;

 

	// for integer images: bitDepth_ is 8, 10, 12, 16 i.e. it is depth per component
   long maxValue = (1L << bitDepth_)-1;

	long pixelsToDrop = 0;
	if( dropPixels_)
		pixelsToDrop = (long)(0.5 + fractionOfPixelsToDropOrSaturate_*img.Height()*imgWidth);
	long pixelsToSaturate = 0;
	if( saturatePixels_)
		pixelsToSaturate = (long)(0.5 + fractionOfPixelsToDropOrSaturate_*img.Height()*imgWidth);

   unsigned j, k;
   if (pixelType.compare(g_PixelType_8bit) == 0)
   {
      double pedestal = 127 * exp / 100.0 * GetBinning() * GetBinning();
      unsigned char* pBuf = const_cast<unsigned char*>(img.GetPixels());
      for (j=0; j<img.Height(); j++)
      {
         for (k=0; k<imgWidth; k++)
         {
            long lIndex = imgWidth*j + k;
            unsigned char val = (unsigned char) (g_IntensityFactor_ * std::min(255.0, (pedestal + dAmp * sin(dPhase_ + dLinePhase + (2.0 * lSinePeriod * k) / lPeriod))));
            if (val > maxDrawnVal) {
                maxDrawnVal = val;
            }
            *(pBuf + lIndex) = val;
         }
         dLinePhase += cLinePhaseInc;
      }
	   for(int snoise = 0; snoise < pixelsToSaturate; ++snoise)
		{
			j = (unsigned)( (double)(img.Height()-1)*(double)rand()/(double)RAND_MAX);
			k = (unsigned)( (double)(imgWidth-1)*(double)rand()/(double)RAND_MAX);
			*(pBuf + imgWidth*j + k) = (unsigned char)maxValue;
		}
		int pnoise;
		for(pnoise = 0; pnoise < pixelsToDrop; ++pnoise)
		{
			j = (unsigned)( (double)(img.Height()-1)*(double)rand()/(double)RAND_MAX);
			k = (unsigned)( (double)(imgWidth-1)*(double)rand()/(double)RAND_MAX);
			*(pBuf + imgWidth*j + k) = 0;
		}

   }
   else if (pixelType.compare(g_PixelType_16bit) == 0)
   {
      double pedestal = maxValue/2 * exp / 100.0 * GetBinning() * GetBinning();
      double dAmp16 = dAmp * maxValue/255.0; // scale to behave like 8-bit
      unsigned short* pBuf = (unsigned short*) const_cast<unsigned char*>(img.GetPixels());
      for (j=0; j<img.Height(); j++)
      {
         for (k=0; k<imgWidth; k++)
         {
            long lIndex = imgWidth*j + k;
            unsigned short val = (unsigned short) (g_IntensityFactor_ * std::min((double)maxValue, pedestal + dAmp16 * sin(dPhase_ + dLinePhase + (2.0 * lSinePeriod * k) / lPeriod)));
            if (val > maxDrawnVal) {
                maxDrawnVal = val;
            }
            *(pBuf + lIndex) = val;
         }
         dLinePhase += cLinePhaseInc;
      }         
	   for(int snoise = 0; snoise < pixelsToSaturate; ++snoise)
		{
			j = (unsigned)(0.5 + (double)img.Height()*(double)rand()/(double)RAND_MAX);
			k = (unsigned)(0.5 + (double)imgWidth*(double)rand()/(double)RAND_MAX);
			*(pBuf + imgWidth*j + k) = (unsigned short)maxValue;
		}
		int pnoise;
		for(pnoise = 0; pnoise < pixelsToDrop; ++pnoise)
		{
			j = (unsigned)(0.5 + (double)img.Height()*(double)rand()/(double)RAND_MAX);
			k = (unsigned)(0.5 + (double)imgWidth*(double)rand()/(double)RAND_MAX);
			*(pBuf + imgWidth*j + k) = 0;
		}
	
	}
    

    if (shouldDisplayImageNumber_) {
        // Draw a seven-segment display in the upper-left corner of the image,
        // indicating the image number.
        int divisor = 1;
        int numDigits = 0;
        while (imageCounter_ / divisor > 0) {
            divisor *= 10;
            numDigits += 1;
        }
        int remainder = imageCounter_;
        for (int i = 0; i < numDigits; ++i) {
            // Black out the background for this digit.
            // TODO: for now, hardcoded sizes, which will cause buffer
            // overflows if the image size is too small -- but that seems
            // unlikely.
            int xBase = (numDigits - i - 1) * 20 + 2;
            int yBase = 2;
            for (int x = xBase; x < xBase + 20; ++x) {
                for (int y = yBase; y < yBase + 20; ++y) {
                    long lIndex = imgWidth*y + x;

                    if (pixelType.compare(g_PixelType_8bit) == 0) {
                        *((unsigned char*) rawBuf + lIndex) = 0;
                    }
                    else if (pixelType.compare(g_PixelType_16bit) == 0) {
                        *((unsigned short*) rawBuf + lIndex) = 0;
                    }
                    else if (pixelType.compare(g_PixelType_32bit) == 0 ||
                             pixelType.compare(g_PixelType_32bitRGB) == 0) {
                        *((unsigned int*) rawBuf + lIndex) = 0;
                    }
                }
            }
            // Draw each segment, if appropriate.
            int digit = remainder % 10;
            for (int segment = 0; segment < 7; ++segment) {
                if (!((1 << segment) & SEVEN_SEGMENT_RULES[digit])) {
                    // This segment is not drawn.
                    continue;
                }
                // Determine if the segment is horizontal or vertical.
                int xStep = SEVEN_SEGMENT_HORIZONTALITY[segment];
                int yStep = (xStep + 1) % 2;
                // Calculate starting point for drawing the segment.
                int xStart = xBase + SEVEN_SEGMENT_X_OFFSET[segment] * 16;
                int yStart = yBase + SEVEN_SEGMENT_Y_OFFSET[segment] * 8 + 1;
                // Draw one pixel at a time of the segment.
                for (int pixNum = 0; pixNum < 8 * (xStep + 1); ++pixNum) {
                    long lIndex = imgWidth * (yStart + pixNum * yStep) + (xStart + pixNum * xStep);
                    if (pixelType.compare(g_PixelType_8bit) == 0) {
                        *((unsigned char*) rawBuf + lIndex) = static_cast<unsigned char>(maxDrawnVal);
                    }
                    else if (pixelType.compare(g_PixelType_16bit) == 0) {
                        *((unsigned short*) rawBuf + lIndex) = static_cast<unsigned short>(maxDrawnVal);
                    }
                    else if (pixelType.compare(g_PixelType_32bit) == 0 ||
                             pixelType.compare(g_PixelType_32bitRGB) == 0) {
                        *((unsigned int*) rawBuf + lIndex) = static_cast<unsigned int>(maxDrawnVal);
                    }
                }
            }
            remainder /= 10;
        }
    }
   if (multiROIXs_.size() > 0)
   {
      // Blank out all pixels that are not in an ROI.
      // TODO: it would be more efficient to only populate pixel values that
      // *are* in an ROI, but that would require substantial refactoring of
      // this function.
      for (unsigned int i = 0; i < imgWidth; ++i)
      {
         for (unsigned j = 0; j < img.Height(); ++j)
         {
            bool shouldKeep = false;
            for (unsigned int k = 0; k < multiROIXs_.size(); ++k)
            {
               unsigned xOffset = multiROIXs_[k] - roiX_;
               unsigned yOffset = multiROIYs_[k] - roiY_;
               unsigned width = multiROIWidths_[k];
               unsigned height = multiROIHeights_[k];
               if (i >= xOffset && i < xOffset + width &&
                        j >= yOffset && j < yOffset + height)
               {
                  // Pixel is inside an ROI.
                  shouldKeep = true;
                  break;
               }
            }
            if (!shouldKeep)
            {
               // Blank the pixel.
               long lIndex = imgWidth * j + i;
               if (pixelType.compare(g_PixelType_8bit) == 0)
               {
                  *((unsigned char*) rawBuf + lIndex) = static_cast<unsigned char>(multiROIFillValue_);
               }
               else if (pixelType.compare(g_PixelType_16bit) == 0)
               {
                  *((unsigned short*) rawBuf + lIndex) = static_cast<unsigned short>(multiROIFillValue_);
               }
               else if (pixelType.compare(g_PixelType_32bit) == 0 ||
                        pixelType.compare(g_PixelType_32bitRGB) == 0)
               {
                  *((unsigned int*) rawBuf + lIndex) = static_cast<unsigned int>(multiROIFillValue_);
               }
            }
         }
      }
   }
   dPhase_ += lSinePeriod / 4.;
}


bool XZellZeissCamera::GenerateColorTestPattern(ImgBuffer& img)
{
   unsigned width = img.Width(), height = img.Height();
   switch (img.Depth())
   {
      case 1:
      {
         const unsigned char maxVal = 255;
         unsigned char* rawBytes = img.GetPixelsRW();
         for (unsigned y = 0; y < height; ++y)
         {
            for (unsigned x = 0; x < width; ++x)
            {
               if (y == 0)
               {
                  rawBytes[x] = (unsigned char) (maxVal * (x + 1) / (width - 1));
               }
               else {
                  rawBytes[x + y * width] = rawBytes[x];
               }
            }
         }
         return true;
      }
      case 2:
      {
         const unsigned short maxVal = 65535;
         unsigned short* rawShorts =
            reinterpret_cast<unsigned short*>(img.GetPixelsRW());
         for (unsigned y = 0; y < height; ++y)
         {
            for (unsigned x = 0; x < width; ++x)
            {
               if (y == 0)
               {
                  rawShorts[x] = (unsigned short) (maxVal * (x + 1) / (width - 1));
               }
               else {
                  rawShorts[x + y * width] = rawShorts[x];
               }
            }
         }
         return true;
      }
      case 4:
      {
         const unsigned long maxVal = 255;
         unsigned* rawPixels = reinterpret_cast<unsigned*>(img.GetPixelsRW());
         for (unsigned section = 0; section < 8; ++section)
         {
            unsigned ystart = section * (height / 8);
            unsigned ystop = section == 7 ? height : ystart + (height / 8);
            for (unsigned y = ystart; y < ystop; ++y)
            {
               for (unsigned x = 0; x < width; ++x)
               {
                  rawPixels[x + y * width] = 0;
                  for (unsigned component = 0; component < 4; ++component)
                  {
                     unsigned sample = 0;
                     if (component == section ||
                           (section >= 4 && section - 4 != component))
                     {
                        sample = maxVal * (x + 1) / (width - 1);
                     }
                     sample &= 0xff; // Just in case
                     rawPixels[x + y * width] |= sample << (8 * component);
                  }
               }
            }
         }
         return true;
      }
   }
   return false;
}


void XZellZeissCamera::TestResourceLocking(const bool recurse)
{
   if(recurse)
      TestResourceLocking(false);
}

/**
* Generate an image with offset plus noise
*/
void XZellZeissCamera::AddBackgroundAndNoise(ImgBuffer& img, double mean, double stdDev)
{ 
	char buf[MM::MaxStrLength];
   GetProperty(MM::g_Keyword_PixelType, buf);
	std::string pixelType(buf);

   int maxValue = 1 << GetBitDepth();
   long nrPixels = img.Width() * img.Height();
   if (pixelType.compare(g_PixelType_8bit) == 0)
   {
      unsigned char* pBuf = (unsigned char*) const_cast<unsigned char*>(img.GetPixels());
      for (long i = 0; i < nrPixels; i++) 
      {
         double value = GaussDistributedValue(mean, stdDev);
         if (value < 0) 
         {
            value = 0;
         }
         else if (value > maxValue)
         {
            value = maxValue;
         }
         *(pBuf + i) = (unsigned char) value;
      }
   }
   else if (pixelType.compare(g_PixelType_16bit) == 0)
   {
      unsigned short* pBuf = (unsigned short*) const_cast<unsigned char*>(img.GetPixels());
      for (long i = 0; i < nrPixels; i++) 
      {
         double value = GaussDistributedValue(mean, stdDev);
         if (value < 0) 
         {
            value = 0;
         }
         else if (value > maxValue)
         {
            value = maxValue;
         }
         *(pBuf + i) = (unsigned short) value;
      }
   }
}


/**
* Adds signal to an image
* Assume a homogenuous illumination
* Calculates the signal for each pixel individually as:
* photon flux * exposure time / conversion factor
* Assumes QE of 100%
*/
void XZellZeissCamera::AddSignal(ImgBuffer& img, double photonFlux, double exp, double cf)
{ 
	char buf[MM::MaxStrLength];
   GetProperty(MM::g_Keyword_PixelType, buf);
	std::string pixelType(buf);

   int maxValue = (1 << GetBitDepth()) -1;
   long nrPixels = img.Width() * img.Height();
   double photons = photonFlux * exp;
   double shotNoise = sqrt(photons);
   double digitalValue = photons / cf;
   double shotNoiseDigital = shotNoise / cf;
   if (pixelType.compare(g_PixelType_8bit) == 0)
   {
      unsigned char* pBuf = (unsigned char*) const_cast<unsigned char*>(img.GetPixels());
      for (long i = 0; i < nrPixels; i++) 
      {
         double value = *(pBuf + i) + GaussDistributedValue(digitalValue, shotNoiseDigital);
         if (value < 0) 
         {
            value = 0;
         }
         else if (value > maxValue)
         {
            value = maxValue;
         }
         *(pBuf + i) =  (unsigned char) value;
      }
   }
   else if (pixelType.compare(g_PixelType_16bit) == 0)
   {
      unsigned short* pBuf = (unsigned short*) const_cast<unsigned char*>(img.GetPixels());
      for (long i = 0; i < nrPixels; i++) 
      {
         double value = *(pBuf + i) + GaussDistributedValue(digitalValue, shotNoiseDigital);
         if (value < 0) 
         {
            value = 0;
         }
         else if (value > maxValue)
         {
            value = maxValue;
         }
         *(pBuf + i) = (unsigned short) value;
      }
   }
}


/**
 * Uses Marsaglia polar method to generate Gaussian distributed value.  
 * Then distributes this around mean with the desired std
 */
double XZellZeissCamera::GaussDistributedValue(double mean, double std)
{
   double s = 2;
   double u = 1; // incosequential, but avoid potantial use of uninitialized value
   double v;
   double halfRandMax = (double) RAND_MAX / 2.0;
   while (s >= 1 || s <= 0) 
   {
      // get random values between -1 and 1
      u = (double) rand() / halfRandMax - 1.0;
      v = (double) rand() / halfRandMax - 1.0;
      s = u * u + v * v;
   }
   double tmp = sqrt( -2 * log(s) / s);
   double x = u * tmp;

   return mean + std * x;
}

int XZellZeissCamera::RegisterImgManipulatorCallBack(ImgManipulator* imgManpl)
{
   imgManpl_ = imgManpl;
   return DEVICE_OK;
}


////////// BEGINNING OF POORLY ORGANIZED CODE //////////////
//////////  CLEANUP NEEDED ////////////////////////////

int DemoHub::Initialize()
{
  	initialized_ = true;
 
	return DEVICE_OK;
}

int DemoHub::DetectInstalledDevices()
{  
   ClearInstalledDevices();

   // make sure this method is called before we look for available devices
   InitializeModuleData();

   char hubName[MM::MaxStrLength];
   GetName(hubName); // this device name
   for (unsigned i=0; i<GetNumberOfDevices(); i++)
   { 
      char deviceName[MM::MaxStrLength];
      bool success = GetDeviceName(i, deviceName, MM::MaxStrLength);
      if (success && (strcmp(hubName, deviceName) != 0))
      {
         MM::Device* pDev = CreateDevice(deviceName);
         AddInstalledDevice(pDev);
      }
   }
   return DEVICE_OK; 
}

void DemoHub::GetName(char* pName) const
{
   CDeviceUtils::CopyLimitedString(pName, g_HubDeviceName);
}
