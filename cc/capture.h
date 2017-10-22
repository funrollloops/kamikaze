#ifndef __KAMIKAZE_CAPTURE_H__
#define __KAMIKAZE_CAPTURE_H__

#include <opencv2/opencv.hpp>

#include "logging.h"
#include "raspicam/src/raspicam_cv.h"

class CaptureSource {
  public:
    virtual bool grab() = 0;
    virtual bool retrieve(cv::Mat *image) = 0;
};

class WebcamCaptureSource : public CaptureSource {
  public:
    WebcamCaptureSource(int webcam, int width, int height) : capture_(webcam) {
      QCHECK(capture_.isOpened()) << "Failed to open --webcam=" << webcam;
      // A return value of false doesn't mean the prop set failed!
      QCHECK(capture_.set(cv::CAP_PROP_FRAME_WIDTH, width) || true)
          << " tried to set width to " << width;
      QCHECK(capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height) || true)
          << " tried to set height to " << height;
    }
    bool grab() override { return capture_.grab(); }
    bool retrieve(cv::Mat* image) override { return capture_.retrieve(*image); }

  private:
    cv::VideoCapture capture_;
};

class RaspiCamCaptureSource : public CaptureSource {
  public:
    RaspiCamCaptureSource(int width, int height) {
      QCHECK(capture_.set(cv::CAP_PROP_FRAME_WIDTH, width) || true)
          << " tried to set width to " << width;
      QCHECK(capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height) || true)
          << " tried to set height to " << height;
      // capture_.set(cv::CAP_PROP_FORMAT, CV_8UC1);
      QCHECK(capture_.open()) << " failed to open raspicam";
    }

    ~RaspiCamCaptureSource() { capture_.release(); }

    bool grab() override { return capture_.grab(); }

    bool retrieve(cv::Mat* image) override {
      capture_.retrieve(*image);
      return true;
    }

  private:
    raspicam::RaspiCam_Cv capture_;
};

#endif
