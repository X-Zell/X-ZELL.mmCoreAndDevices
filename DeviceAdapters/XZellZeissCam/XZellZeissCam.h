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
#include <map>
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

class MySequenceThread;

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
   bool SupportsMultiROI();
   bool IsMultiROISet();
   int GetMultiROICount(unsigned& count);
   int SetMultiROI(const unsigned* xs, const unsigned* ys,
           const unsigned* widths, const unsigned* heights,
           unsigned numROIs);
   int GetMultiROI(unsigned* xs, unsigned* ys, unsigned* widths,
           unsigned* heights, unsigned* length);
   int PrepareSequenceAcqusition()
   {
       LogMessage("API METHOD ENTRY: PrepareSequenceAcqusition");
       return DEVICE_OK;
   }
   int StartSequenceAcquisition(double interval);
   int StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow);
   int StopSequenceAcquisition();
   int InsertImage();
   int RunSequenceOnThread();
   bool IsCapturing();
   void OnThreadExiting() throw(); 
   double GetNominalPixelSizeUm() const {
       LogMessage("API METHOD ENTRY: GetPixelSizeUm");
       return nominalPixelSizeUm_;
   }
   double GetPixelSizeUm() const {return nominalPixelSizeUm_ * GetBinning();}
   int GetBinning() const;
   int SetBinning(int bS);

   int IsExposureSequenceable(bool& isSequenceable) const;
   int GetExposureSequenceMaxLength(long& nrEvents) const;
   int StartExposureSequence();
   int StopExposureSequence();
   int ClearExposureSequence();
   int AddToExposureSequence(double exposureTime_ms);
   int SendExposureSequence() const;

   unsigned  GetNumberOfComponents() const { return nComponents_;};

   // action interface
   // ----------------
   int OnMaxExposure(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnTestProperty(MM::PropertyBase* pProp, MM::ActionType eAct, long);
   int OnAsyncFollower(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnAsyncLeader(MM::PropertyBase* pProp, MM::ActionType eAct);
   void SlowPropUpdate(std::string leaderValue);
   int OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnPixelType(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnBitDepth(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnReadoutTime(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnScanMode(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnErrorSimulation(MM::PropertyBase* , MM::ActionType );
   int OnCameraCCDXSize(MM::PropertyBase* , MM::ActionType );
   int OnCameraCCDYSize(MM::PropertyBase* , MM::ActionType );
   int OnTriggerDevice(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnDropPixels(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnFastImage(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSaturatePixels(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnFractionOfPixelsToDropOrSaturate(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnShouldRotateImages(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnShouldDisplayImageNumber(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnStripeWidth(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnSupportsMultiROI(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnMultiROIFillValue(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnCCDTemp(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnIsSequenceable(MM::PropertyBase* pProp, MM::ActionType eAct);
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
   bool isSequenceable_;
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
	std::string triggerDevice_;

   bool stopOnOverflow_;

	bool dropPixels_;
   bool fastImage_;
	bool saturatePixels_;
	double fractionOfPixelsToDropOrSaturate_;
   bool shouldRotateImages_;
   bool shouldDisplayImageNumber_;
   double stripeWidth_;
   bool supportsMultiROI_;
   int multiROIFillValue_;
   std::vector<unsigned> multiROIXs_;
   std::vector<unsigned> multiROIYs_;
   std::vector<unsigned> multiROIWidths_;
   std::vector<unsigned> multiROIHeights_;

	double testProperty_[10];
   std::string asyncLeader_;
   std::string asyncFollower_;
   MMThreadLock imgPixelsLock_;
   MMThreadLock asyncFollowerLock_;
   friend class MySequenceThread;
   int nComponents_;
   MySequenceThread * thd_;
   std::future<void> fut_;
   int mode_;
   ImgManipulator* imgManpl_;
   double pcf_;
   double photonFlux_;
   double readNoise_;
};

class MySequenceThread : public MMDeviceThreadBase
{
   friend class CDemoCamera;
   enum { default_numImages=1, default_intervalMS = 100 };
   public:
      MySequenceThread(XZellZeissCamera* pCam);
      ~MySequenceThread();
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

//////////////////////////////////////////////////////////////////////////////
// CDemoStateDevice class
// Simulation of a state device in which the number of states can be specified (state device)
//////////////////////////////////////////////////////////////////////////////

class CDemoStateDevice : public CStateDeviceBase<CDemoStateDevice>
{
public:
   CDemoStateDevice();
   ~CDemoStateDevice();
  
   // MMDevice API
   // ------------
   int Initialize();
   int Shutdown();
  
   void GetName(char* pszName) const;
   bool Busy();
  

   // action interface
   // ----------------
   unsigned long GetNumberOfPositions()const { return numPos_; };
   int OnState(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnNumberOfStates(MM::PropertyBase* pProp, MM::ActionType eAct);

   int OnSequence(MM::PropertyBase* pProp, MM::ActionType eAct);
   int SetGateOpen(bool open);
   int GetGateOpen(bool& open);

private:
   uint16_t numPatterns_;
   long numPos_;
   bool initialized_;
   MM::MMTime changedTime_;
   bool busy_;
   bool sequenceOn_;
   bool gateOpen_;
   bool isClosed_;
   long position_;
};

//////////////////////////////////////////////////////////////////////////////
// TransposeProcessor class
// transpose an image
// K.H.
//////////////////////////////////////////////////////////////////////////////
class TransposeProcessor : public CImageProcessorBase<TransposeProcessor>
{
public:
   TransposeProcessor () : inPlace_ (false), pTemp_(NULL), tempSize_(0), busy_(false)
   {
      // parent ID display
      CreateHubIDProperty();
   }
   ~TransposeProcessor () {if( NULL!= pTemp_) free(pTemp_); tempSize_=0;  }

   int Shutdown() {return DEVICE_OK;}
   void GetName(char* name) const {strcpy(name,"TransposeProcessor");}

   int Initialize();

   bool Busy(void) { return busy_;};

    // really primative image transpose algorithm which will work fine for non-square images... 
   template <typename PixelType>
   int TransposeRectangleOutOfPlace(PixelType* pI, unsigned int width, unsigned int height)
   {
      int ret = DEVICE_OK;
      unsigned long tsize = width*height*sizeof(PixelType);
      if( this->tempSize_ != tsize)
      {
         if( NULL != this->pTemp_)
         {
            free(pTemp_);
            pTemp_ = NULL;
         }
         pTemp_ = (PixelType *)malloc(tsize);
      }
      if( NULL != pTemp_)
      {
         PixelType* pTmpImage = (PixelType *) pTemp_;
         tempSize_ = tsize;
         for( unsigned long ix = 0; ix < width; ++ix)
         {
            for( unsigned long iy = 0; iy < height; ++iy)
            {
               pTmpImage[iy + ix*width] = pI[ ix + iy*height];
            }
         }
         memcpy( pI, pTmpImage, tsize);
      }
      else
      {
         ret = DEVICE_ERR;
      }

      return ret;
   }

   
   template <typename PixelType>
   void TransposeSquareInPlace(PixelType* pI, unsigned int dim)
   { 
      PixelType tmp;
      for( unsigned long ix = 0; ix < dim; ++ix)
      {
         for( unsigned long iy = ix; iy < dim; ++iy)
         {
            tmp = pI[iy*dim + ix];
            pI[iy*dim +ix] = pI[ix*dim + iy];
            pI[ix*dim +iy] = tmp; 
         }
      }

      return;
   }

   int Process(unsigned char* buffer, unsigned width, unsigned height, unsigned byteDepth);

   // action interface
   // ----------------
   int OnInPlaceAlgorithm(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   bool inPlace_;
   void* pTemp_;
   unsigned long tempSize_;
   bool busy_;
};



//////////////////////////////////////////////////////////////////////////////
// ImageFlipX class
// flip an image
// K.H.
//////////////////////////////////////////////////////////////////////////////
class ImageFlipX : public CImageProcessorBase<ImageFlipX>
{
public:
   ImageFlipX () :  busy_(false) {}
   ~ImageFlipX () {  }

   int Shutdown() {return DEVICE_OK;}
   void GetName(char* name) const {strcpy(name,"ImageFlipX");}

   int Initialize();
   bool Busy(void) { return busy_;};

   template <typename PixelType>
   int Flip(PixelType* pI, unsigned int width, unsigned int height)
   {
      PixelType tmp;
      int ret = DEVICE_OK;
      for( unsigned long iy = 0; iy < height; ++iy)
      {
         for( unsigned long ix = 0; ix <  (width>>1) ; ++ix)
         {
            tmp = pI[ ix + iy*width];
            pI[ ix + iy*width] = pI[ width - 1 - ix + iy*width];
            pI[ width -1 - ix + iy*width] = tmp;
         }
      }
      return ret;
   }

   int Process(unsigned char* buffer, unsigned width, unsigned height, unsigned byteDepth);

   int OnPerformanceTiming(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   bool busy_;
   MM::MMTime performanceTiming_;
};


//////////////////////////////////////////////////////////////////////////////
// ImageFlipY class
// flip an image
// K.H.
//////////////////////////////////////////////////////////////////////////////
class ImageFlipY : public CImageProcessorBase<ImageFlipY>
{
public:
   ImageFlipY () : busy_(false), performanceTiming_(0.) {}
   ~ImageFlipY () {  }

   int Shutdown() {return DEVICE_OK;}
   void GetName(char* name) const {strcpy(name,"ImageFlipY");}

   int Initialize();
   bool Busy(void) { return busy_;};

   template <typename PixelType>
   int Flip(PixelType* pI, unsigned int width, unsigned int height)
   {
      PixelType tmp;
      int ret = DEVICE_OK;
      for( unsigned long ix = 0; ix < width ; ++ix)
      {
         for( unsigned long iy = 0; iy < (height>>1); ++iy)
         {
            tmp = pI[ ix + iy*width];
            pI[ ix + iy*width] = pI[ ix + (height - 1 - iy)*width];
            pI[ ix + (height - 1 - iy)*width] = tmp;
         }
      }
      return ret;
   }


   int Process(unsigned char* buffer, unsigned width, unsigned height, unsigned byteDepth);

   // action interface
   // ----------------
   int OnPerformanceTiming(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   bool busy_;
   MM::MMTime performanceTiming_;

};



//////////////////////////////////////////////////////////////////////////////
// MedianFilter class
// apply Median filter an image
// K.H.
//////////////////////////////////////////////////////////////////////////////
class MedianFilter : public CImageProcessorBase<MedianFilter>
{
public:
   MedianFilter () : busy_(false), performanceTiming_(0.),pSmoothedIm_(0), sizeOfSmoothedIm_(0)
   {
      // parent ID display
      CreateHubIDProperty();
   };
   ~MedianFilter () { if(0!=pSmoothedIm_) free(pSmoothedIm_); };

   int Shutdown() {return DEVICE_OK;}
   void GetName(char* name) const {strcpy(name,"MedianFilter");}

   int Initialize();
   bool Busy(void) { return busy_;};

   // NOTE: this utility MODIFIES the argument, make a copy yourself if you want the original data preserved
   template <class U> U FindMedian(std::vector<U>& values ) {
      std::sort(values.begin(), values.end());
      return values[(values.size())>>1];
   };


   template <typename PixelType>
   int Filter(PixelType* pI, unsigned int width, unsigned int height)
   {
      int ret = DEVICE_OK;
      int x[9];
      int y[9];

      const unsigned long thisSize = sizeof(*pI)*width*height;
      if( thisSize != sizeOfSmoothedIm_)
      {
         if(NULL!=pSmoothedIm_)
         {
            sizeOfSmoothedIm_ = 0;
            free(pSmoothedIm_);
         }
         // malloc is faster than new...
         pSmoothedIm_ = (PixelType*)malloc(thisSize);
         if(NULL!=pSmoothedIm_)
         {
            sizeOfSmoothedIm_ = thisSize;
         }
      }

      PixelType* pSmooth = (PixelType*) pSmoothedIm_;

      if(NULL != pSmooth)
      {
      /*Apply 3x3 median filter to reduce shot noise*/
      for (unsigned int i=0; i<width; i++) {
         for (unsigned int j=0; j<height; j++) {
            x[0]=i-1;
            y[0]=(j-1);
            x[1]=i;
            y[1]=(j-1);
            x[2]=i+1;
            y[2]=(j-1);
            x[3]=i-1;
            y[3]=(j);
            x[4]=i;
            y[4]=(j);
            x[5]=i+1;
            y[5]=(j);
            x[6]=i-1;
            y[6]=(j+1);
            x[7]=i;
            y[7]=(j+1);
            x[8]=i+1;
            y[8]=(j+1);
            // truncate the median filter window  -- duplicate edge points
            // this could be more efficient, we could fill in the interior image [1,w0-1]x[1,h0-1] then explicitly fill in the edge pixels.
            // also the temporary image could be as small as 2 rasters of the image
            for(int ij =0; ij < 9; ++ij)
            {
               if( x[ij] < 0)
                  x[ij] = 0;
               else if( int(width-1) < x[ij])
                  x[ij] = int(width-1);
               if( y[ij] < 0)
                  y[ij] = 0;
               else if( int(height-1) < y[ij])
                  y[ij] = (int)(height-1);
            }
            std::vector<PixelType> windo;
            for(int ij = 0; ij < 9; ++ij)
            {
               windo.push_back(pI[ x[ij] + width*y[ij]]);
            }
            pSmooth[i + j*width] = FindMedian(windo);
         }
      }

      memcpy( pI, pSmoothedIm_, thisSize);
      }
      else
         ret = DEVICE_ERR;

      return ret;
   }
   int Process(unsigned char* buffer, unsigned width, unsigned height, unsigned byteDepth);

   // action interface
   // ----------------
   int OnPerformanceTiming(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   bool busy_;
   MM::MMTime performanceTiming_;
   void*  pSmoothedIm_;
   unsigned long sizeOfSmoothedIm_;
   


};



struct Point
{
   public:
      Point(int lx, int ly) {x = lx; y = ly;};
   int x;
   int y;
};

struct PointD
{
   public:
      PointD(double lx, double ly) {x = lx; y = ly;};
   double x;
   double y;
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
