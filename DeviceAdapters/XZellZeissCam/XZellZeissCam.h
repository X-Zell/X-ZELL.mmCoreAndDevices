///////////////////////////////////////////////////////////////////////////////
// FILE:          XZellZeissCamera.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   The X-Zell implementation of the Zeiss Axio camera.
//                
// AUTHOR:        Jeremy Uff, j.uff@influxury.net, 23/02/2024
//
// COPYRIGHT:     X-Zell Biotech Pte Ltd, Singapore
//
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

#pragma once

#include "DeviceBase.h"
#include "ImgBuffer.h"
#include "DeviceThreads.h"
#include <string>
#include <algorithm>
#include <stdint.h>
#include <future>

//////////////////////////////////////////////////////////////////////////////
// Error codes
//
#define ERR_UNKNOWN_MODE         102
#define ERR_UNKNOWN_POSITION     103
#define ERR_IN_SEQUENCE          104
#define ERR_SEQUENCE_INACTIVE    105
#define ERR_STAGE_MOVING         106
#define HUB_NOT_AVAILABLE        107

const char* NoHubError = "Parent Hub not defined.";

// Defines which segments in a seven-segment display are lit up for each of
// the numbers 0-9. Segments are:
//
//  0       1
// 1 2     2 4
//  3       8
// 4 5    16 32
//  6      64
const int SEVEN_SEGMENT_RULES[] = {1+2+4+16+32+64, 4+32, 1+4+8+16+64,
      1+4+8+32+64, 2+4+8+32, 1+2+8+32+64, 2+8+16+32+64, 1+4+32,
      1+2+4+8+16+32+64, 1+2+4+8+32+64};
// Indicates if the segment is horizontal or vertical.
const int SEVEN_SEGMENT_HORIZONTALITY[] = {1, 0, 0, 1, 0, 0, 1};
// X offset for this segment.
const int SEVEN_SEGMENT_X_OFFSET[] = {0, 0, 1, 0, 0, 1, 0};
// Y offset for this segment.
const int SEVEN_SEGMENT_Y_OFFSET[] = {0, 0, 0, 1, 1, 1, 2};

class ImgManipulator 
{
   public:
      virtual int ChangePixels(ImgBuffer& img) = 0;
};

////////////////////////
// DemoHub
//////////////////////

class DemoHub : public HubBase<DemoHub>
{
public:
   DemoHub() :
      initialized_(false),
      busy_(false)
   {}
   ~DemoHub() {}

   // Device API
   // ---------
   int Initialize();
   int Shutdown() {return DEVICE_OK;};
   void GetName(char* pName) const; 
   bool Busy() { return busy_;} ;

   // HUB api
   int DetectInstalledDevices();

private:
   void GetPeripheralInventory();

   std::vector<std::string> peripherals_;
   bool initialized_;
   bool busy_;
};


//////////////////////////////////////////////////////////////////////////////
// XZellZeissCamera class
// Simulation of the Camera device
//////////////////////////////////////////////////////////////////////////////

class ZeissAcquisitionThread;

class XZellZeissCamera : public CCameraBase<XZellZeissCamera>  
{
public:
   XZellZeissCamera();
   ~XZellZeissCamera();
  
   // MMDevice API
   // ------------
   int Initialize();
   int Shutdown();
  
   void GetName(char* name) const;      
   
   // MMCamera API
   // ------------
   int SnapImage();
   const unsigned char* GetImageBuffer();
   unsigned GetImageWidth() const;
   unsigned GetImageHeight() const;
   unsigned GetImageBytesPerPixel() const;
   unsigned GetBitDepth() const;
   long GetImageBufferSize() const;
   double GetExposure() const;
   void SetExposure(double exp);
   int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize); 
   int GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize); 
   int ClearROI();
   bool IsMultiROISet();
   int StartSequenceAcquisition(double interval);
   int StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow);
   int StopSequenceAcquisition();
   int InsertImage();
   int RunSequenceOnThread();
   bool IsCapturing();
   void OnThreadExiting() throw();
   int GetBinning() const;
   int SetBinning(int bS);

   int   IsExposureSequenceable(bool& isSequenceable) const
   {
       isSequenceable = false; return DEVICE_OK;
   }

   unsigned  GetNumberOfComponents() const { return nComponents_;};

   // action interface
   // ----------------
   int OnMaxExposure(MM::PropertyBase* pProp, MM::ActionType eAct);
   void SlowPropUpdate(std::string leaderValue);
   int OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnPixelType(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnBitDepth(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnReadoutTime(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnScanMode(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnErrorSimulation(MM::PropertyBase* , MM::ActionType );
   int OnCameraCCDXSize(MM::PropertyBase* , MM::ActionType );
   int OnCameraCCDYSize(MM::PropertyBase* , MM::ActionType );
   int OnDropPixels(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnFastImage(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSaturatePixels(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnFractionOfPixelsToDropOrSaturate(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnShouldRotateImages(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnShouldDisplayImageNumber(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnStripeWidth(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnMultiROIFillValue(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnMode(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnPCF(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnPhotonFlux(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnReadNoise(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnCrash(MM::PropertyBase* pProp, MM::ActionType eAct);

   // Special public DemoCamera methods
   void AddBackgroundAndNoise(ImgBuffer& img, double mean, double stdDev);
   void AddSignal(ImgBuffer& img, double photonFlux, double exp, double cf);
   // this function replace normal_distribution in C++11
   double GaussDistributedValue(double mean, double std);

   int RegisterImgManipulatorCallBack(ImgManipulator* imgManpl);
   int MoveImageToCircularBuffer();
   int CaptureImage();
   long GetCCDXSize() { return cameraCCDXSize_; }
   long GetCCDYSize() { return cameraCCDYSize_; }


private:
   int SetAllowedBinning();
   void TestResourceLocking(const bool);
   void GenerateEmptyImage(ImgBuffer& img);
   void GenerateSyntheticImage(ImgBuffer& img, double exp);
   bool GenerateColorTestPattern(ImgBuffer& img);
   int ResizeImageBuffer();

   static const double nominalPixelSizeUm_;

   double exposureMaximum_;
   double dPhase_;
   ImgBuffer img_;
   bool busy_;
   bool stopOnOverFlow_;
   bool initialized_;
   double readoutUs_;
   MM::MMTime readoutStartTime_;
   long scanMode_;
   int bitDepth_;
   unsigned roiX_;
   unsigned roiY_;
   MM::MMTime sequenceStartTime_;
   long sequenceMaxLength_;
   bool sequenceRunning_;
   unsigned long sequenceIndex_;
   double GetSequenceExposure();
   std::vector<double> exposureSequence_;
   long imageCounter_;
	long binSize_;
	long cameraCCDXSize_;
	long cameraCCDYSize_;
   double ccdT_;

   bool stopOnOverflow_;

	bool dropPixels_;
   bool fastImage_;
	bool saturatePixels_;
	double fractionOfPixelsToDropOrSaturate_;
   bool shouldRotateImages_;
   bool shouldDisplayImageNumber_;
   double stripeWidth_;
   int multiROIFillValue_;
   std::vector<unsigned> multiROIXs_;
   std::vector<unsigned> multiROIYs_;
   std::vector<unsigned> multiROIWidths_;
   std::vector<unsigned> multiROIHeights_;
   
   MMThreadLock imgPixelsLock_;
   friend class ZeissAcquisitionThread;
   int nComponents_;
   ZeissAcquisitionThread * thd_;
   std::future<void> fut_;
   int mode_;
   ImgManipulator* imgManpl_;
   double pcf_;
   double photonFlux_;
   double readNoise_;

   // START OF ZEISS SPECIFIC DEV CODE
   SMCAMINFO cameraInfos;
   // END OF ZEISS SPECIFIC DEV CODE
};

//class MySequenceThread : public MMDeviceThreadBase
class ZeissAcquisitionThread : public MMDeviceThreadBase
{
   friend class XZellZeissCamera;
   enum { default_numImages=1, default_intervalMS = 100 };
   public:
      ZeissAcquisitionThread(XZellZeissCamera* pCam);
      ~ZeissAcquisitionThread();
      void Stop();
      void Start(long numImages, double intervalMs);
      bool IsStopped();
      void Suspend();
      bool IsSuspended();
      void Resume();
      double GetIntervalMs(){return intervalMs_;}                               
      void SetLength(long images) {numImages_ = images;}                        
      long GetLength() const {return numImages_;}
      long GetImageCounter(){return imageCounter_;}                             
      MM::MMTime GetStartTime(){return startTime_;}                             
      MM::MMTime GetActualDuration(){return actualDuration_;}
   private:                                                                     
      int svc(void) throw();
      double intervalMs_;                                                       
      long numImages_;                                                          
      long imageCounter_;                                                       
      bool stop_;                                                               
      bool suspend_;                                                            
      XZellZeissCamera* camera_;                                                     
      MM::MMTime startTime_;                                                    
      MM::MMTime actualDuration_;                                               
      MM::MMTime lastFrameTime_;                                                
      MMThreadLock stopLock_;                                                   
      MMThreadLock suspendLock_;                                                
};



// from somewhere
unsigned int htoi(const char* ptr)
{
    unsigned int value = 0;
    char ch = *ptr;

    /*--------------------------------------------------------------------------*/

    while (ch == ' ' || ch == '\t')
        ch = *(++ptr);

    for (;;) {

        if (ch >= '0' && ch <= '9')
            value = (value << 4) + (ch - '0');
        else if (ch >= 'A' && ch <= 'F')
            value = (value << 4) + (ch - 'A' + 10);
        else if (ch >= 'a' && ch <= 'f')
            value = (value << 4) + (ch - 'a' + 10);
        else
            return value;
        ch = *(++ptr);
    }
}

// always send in an even number of ASCII charaters!!
void WriteHexString(FILE* fpw, const char* const pdata)
{
    const char* p = pdata;

    while (*p != 0)
    {

        char hexrep[3];
        hexrep[0] = *p;
        hexrep[1] = *(p + 1);
        hexrep[2] = 0;
        p += 2;
        unsigned int theChar = htoi(hexrep);
        putc(theChar, fpw);
    }

}
