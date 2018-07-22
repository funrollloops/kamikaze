#ifndef __BLASTER_CAPTURE_H__
#define __BLASTER_CAPTURE_H__

#include <chrono>
#include <condition_variable>
#include <experimental/optional>
#include <mutex>
#include <string>
#include <thread>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>

#include "robot.h"

class CaptureSource {
  public:
    virtual bool isOpened() = 0;
    virtual bool grab() = 0;
    virtual bool retrieve(cv::Mat *image) = 0;
    virtual std::string describe() = 0;
    virtual cv::Size size() const = 0;
};

class WebcamCaptureSource : public CaptureSource {
  public:
    WebcamCaptureSource(int webcam, int width, int height)
        : webcam_(webcam), capture_(webcam), size_(width, height) {
      // A return value of false doesn't mean the prop set failed!
      capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
      capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
      capture_.set(cv::CAP_PROP_FPS, 60);
    }
    bool isOpened() override { return capture_.isOpened(); }
    bool grab() override { return capture_.grab(); }
    bool retrieve(cv::Mat* image) override { return capture_.retrieve(*image); }
    std::string describe() override {
      std::ostringstream buf;
      buf << "--webcam=" << webcam_;
      return buf.str();
    }
    cv::Size size() const override { return size_; }

  private:
    const int webcam_;
    cv::VideoCapture capture_;
    const cv::Size size_;
};

class RaspiCamCaptureSource : public CaptureSource {
  public:
    RaspiCamCaptureSource(int width, int height): size_(width, height) {
      capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
      capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
      CHECK(capture_.open()) << " failed to open raspicam";
    }

    ~RaspiCamCaptureSource() { capture_.release(); }

    bool isOpened() override { return capture_.isOpened(); }
    bool grab() override { return capture_.grab(); }
    bool retrieve(cv::Mat* image) override {
      capture_.retrieve(*image);
      return true;
    }
    std::string describe() override { return "--raspicam"; }
    cv::Size size() const override { return size_; }

  private:
    raspicam::RaspiCam_Cv capture_;
    const cv::Size size_;
};

class AsyncCaptureSource {
public:
  AsyncCaptureSource(Robot *robot, std::unique_ptr<CaptureSource> source);
  ~AsyncCaptureSource();

  using clock = std::chrono::high_resolution_clock;

  struct LatestImage {
    std::chrono::time_point<clock> timestamp;
    Robot::Pos pos;
    cv::Mat image;
  };

  class CaptureScope {
  public:
    CaptureScope() = delete;
    CaptureScope(const CaptureScope&) = delete;
    CaptureScope& operator=(const CaptureScope&) = delete;
    CaptureScope(CaptureScope &&other) : source(other.source) {
      other.source = nullptr;
    }
    CaptureScope& operator=(CaptureScope&&) = delete;

    ~CaptureScope() { if (source) source->FinishCapture(); }

  private:
    friend AsyncCaptureSource;
    CaptureScope(AsyncCaptureSource *source) : source(source) {}
    AsyncCaptureSource *source = nullptr;
  };

  // Returns the next image. Sequential calls are guaranteed not to return the
  // same image.
  LatestImage next_image();

  // Begins (asynchronously) capturing a video to the given path.
  CaptureScope StartCapture(const std::string& filename);

private:
  void BackgroundTask();
  void FinishCapture();

  Robot& robot_;
  std::unique_ptr<CaptureSource> source_;
  std::experimental::optional<cv::VideoWriter> writer_;
  LatestImage latest_image_;

  // Thread management.
  std::thread retrieve_thread_;
  std::mutex writer_mu_;
  std::mutex latest_image_mu_;
  std::condition_variable new_image_ready_cv_;
  std::atomic<bool> done_{false};
  bool new_image_ready_ = false;
};

#endif
