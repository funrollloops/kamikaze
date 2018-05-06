#include "capture.h"

const static auto kFourCC = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

AsyncCaptureSource::AsyncCaptureSource(Robot *robot,
                                       std::unique_ptr<CaptureSource> source)
    : robot_(*CHECK_NOTNULL(robot)), source_(std::move(CHECK_NOTNULL(source))),
      retrieve_thread_([this] { BackgroundTask(); }) {
  CHECK(source_->isOpened()) << "failed to open " << source_->describe();
}

AsyncCaptureSource::~AsyncCaptureSource() {
  done_ = true;
  retrieve_thread_.join();
}

void AsyncCaptureSource::BackgroundTask() {
  while (!done_.load(std::memory_order_relaxed)) {
    cv::Mat image;
    if (!source_->grab()) { // Defer decoding until after we calculate the
                            // timestamp.
      LOG(ERROR) << "error grabbing from " << source_->describe() << std::endl;
    } else {
      std::chrono::time_point<clock> capture_time = clock::now();
      auto pos = robot_.tell();
      if (!source_->retrieve(&image)) {
        LOG(ERROR) << "error retrieving from " << source_->describe()
                   << std::endl;
      }
      std::unique_lock<std::mutex> lock(mu_);
      if (writer_) {
        writer_->write(image);
      }
      latest_image_.timestamp = capture_time;
      latest_image_.pos = pos;
      latest_image_.image = std::move(image);
      new_image_ready_ = true;
      lock.unlock();
      new_image_ready_cv_.notify_one();
    }
  }
}

AsyncCaptureSource::LatestImage AsyncCaptureSource::next_image() {
  std::unique_lock<std::mutex> lock(mu_);
  new_image_ready_cv_.wait(lock, [&] { return new_image_ready_; });
  new_image_ready_ = false;
  return std::move(latest_image_);
}

AsyncCaptureSource::CaptureScope
AsyncCaptureSource::StartCapture(const std::string &filename) {
  std::unique_lock<std::mutex> lock(mu_);
  CHECK(!writer_) << "Nested capture scopes are not supported";
  writer_.emplace(filename, kFourCC, /*fps=*/15, source_->size());
  CHECK(writer_->isOpened());
  return CaptureScope(this);
}

void AsyncCaptureSource::FinishCapture() {
  std::unique_lock<std::mutex> lock(mu_);
  CHECK(writer_) << "Tried to FinishCapture when none was in progress.";
  writer_->release();
}