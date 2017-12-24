#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <experimental/optional>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "capture.h"
#include "operators.h"
#include "robot.h"

#include <opencv2/opencv.hpp>

DEFINE_int32(webcam, 0,
             "Webcam# to use. Usually 0 for built-in, 1+ for external.");
DEFINE_bool(raspicam, false, "Fetch images from raspicam.");
DEFINE_bool(
    wait_between_images, true,
    "When doing detection on images, set to true to wait after each image.");
DEFINE_bool(preview, true, "Enable preview window.");

constexpr char kFaceCascadeFile[] =
    "../haarcascades/haarcascade_frontalface_default.xml";
constexpr char kEyeCascadeFile[] = "../haarcascades/haarcascade_eye.xml";
constexpr char kMouthCascadeFile[] = "../haarcascades/haarcascade_smile.xml";
static const cv::Size kMinFaceSize(100, 100);
static const auto kMinTimeBetweenFire = std::chrono::seconds(5);
static const auto kFireTime = std::chrono::milliseconds(500);
static const int kMinConsecutiveOnTargetToFire = 10;

#define FIND_EYES 0

static const cv::Scalar kBlue(255, 0, 0);
static const cv::Scalar kGreen(0, 255, 0);
static const cv::Scalar kRed(0, 0, 255);
static const cv::Scalar kTeal(255, 255, 0);
static const cv::Scalar kYellow(0, 255, 255);
static const cv::Scalar kWhite(255, 255, 255);

static const cv::Point kImageSize(1280, 720);
static const cv::Point kTarget = kImageSize / 2;
static const cv::Point kFovInSteps(300, -250);
static constexpr int kTargetSize = 20;
static constexpr int kMinStep = 4;

using std::experimental::optional;
using std::experimental::nullopt;

const char *kActionNames[] = {"MOVE", "FIRE"};
struct Action {
  enum ActionEnum { MOVE, FIRE };

  Action(ActionEnum cmd, Robot::Pos delta) : cmd(cmd), delta(delta) {}

  ActionEnum cmd;
  Robot::Pos delta;
};

using time_point = std::chrono::time_point<std::chrono::high_resolution_clock>;
time_point now() { return std::chrono::high_resolution_clock::now(); }

void DoAction(Action action, Robot::Pos pos, Robot *robot) {
  switch (action.cmd) {
  case Action::MOVE:
    robot->moveTo({int16_t(action.delta.first + pos.first),
                   int16_t(action.delta.second + pos.second)});
    return;
  case Action::FIRE:
    robot->fire(kFireTime);
    return;
  }
}

std::ostream &operator<<(std::ostream &os, Action action) {
  return os << kActionNames[action.cmd] << "(" << action.delta << ")";
}

class Recognizer {
public:
  using Mat = cv::Mat;

  Recognizer(Robot *robot) : robot_(robot) {
    CHECK(face_detector_->load(kFaceCascadeFile))
        << " error loading " << kFaceCascadeFile;
  }

  static void PlotFeature(cv::Mat &mat, const cv::Rect &feature,
                          const cv::Scalar &color) {
    if (FLAGS_preview) {
      cv::rectangle(mat, feature.tl(), feature.br(), color);
    }
  }

  static cv::Rect GuessMouthLocation(const cv::Rect &face) {
    return cv::Rect(
        cv::Point(face.x + face.width / 4, face.y + 9 * face.height / 12),
        cv::Size(face.width / 2, face.height / 6));
  }

  static cv::Rect BestMouth(const std::vector<cv::Rect> &mouths) {
    cv::Rect best = mouths[0];
    for (const cv::Rect &mouth : mouths)
      if (mouth.width * mouth.height > best.width * best.height)
        best = mouth;
    return best;
  }

  static cv::Rect BestFace(std::vector<cv::Rect> &faces) {
    cv::Rect best_face = faces[0];
    for (const cv::Rect& face : faces) {
      if (sqdist(Center(face), kTarget) < sqdist(Center(best_face), kTarget)) {
        best_face = face;
      }
    }
    return best_face;
  }

  optional<Action> DetermineAction(cv::Mat &input_img, const cv::Point &mouth) {
    CHECK(input_img.rows == kImageSize.y)
        << "rows=" << input_img.rows << " expected_height=" << kImageSize.y;
    CHECK(input_img.cols == kImageSize.x) << "cols=" << input_img.cols
                                           << " expected_with=" << kImageSize.x;
    constexpr int kTargetSize = 20;
    const cv::Rect target(kTarget - kTargetSize / 2,
                          cv::Size(kTargetSize, kTargetSize));
    PlotFeature(input_img, target, kTeal);
    auto vec = (kTarget - mouth) * kFovInSteps / kImageSize;
    if (abs(vec.x) > 4 || abs(vec.y) > 4) {
      maybe_fire_ = 0;
      return Action(Action::MOVE, Robot::Pos{int16_t(vec.x), int16_t(vec.y)});
    }

    auto fire_time = now();
    if (++maybe_fire_ > kMinConsecutiveOnTargetToFire &&
        fire_time - last_fire_ > kMinTimeBetweenFire) {
      maybe_fire_ = 0;
      last_fire_ = fire_time;
      return Action(Action::FIRE, Robot::Pos{0, 0});
    }
    return nullopt;
  }

  std::vector<cv::Rect> DetectMultiScale(cv::CascadeClassifier *cc,
                                         const cv::Mat &mat,
                                         double scale_factor = 1.3,
                                         int min_neighbors = 3,
                                         cv::Size min_size = {0, 0}) {
    std::vector<cv::Rect> rects;
    cc->detectMultiScale(mat, rects, scale_factor, min_neighbors, /*flags=*/0,
                         /*minSize=*/min_size);
    return rects;
  }

  void Detect(time_point timestamp, Robot::Pos pos, cv::Mat &input_img) {
    if (timestamp - last_action_ <=  std::chrono::milliseconds(10)) {
      return;
    }
    cv::cvtColor(input_img, gray_, cv::COLOR_BGR2GRAY);
    std::ostringstream line1;
    std::ostringstream line2;
    auto faces =
        DetectMultiScale(face_detector_.get(), gray_, 1.3, 5, kMinFaceSize);
    for (const cv::Rect& face : faces) {
      PlotFeature(input_img, face, kBlue);
    }
    line2 << "pos=" << pos << " ";
    if (!faces.empty()) {
      auto face = BestFace(faces);
      cv::Rect mouth = GuessMouthLocation(face);
      PlotFeature(input_img, mouth, kYellow);
      if (optional<Action> action = DetermineAction(input_img, Center(mouth))) {
        DoAction(*action, pos, robot_);
        line2 << *action << " ";
      }
      if (maybe_fire_ > 0) {
        line2 << "maybe_fire(" << maybe_fire_ << ") ";
      }
    }
    last_action_ = now();
    line1 << "latency "
          << ((last_action_ - timestamp) / std::chrono::milliseconds(1))
          << " ms";
    if (FLAGS_preview) {
      const cv::Size line1_size =
          cv::getTextSize(line1.str(), cv::FONT_HERSHEY_PLAIN,
                          /*fontScale=*/1, /*thickness=*/1, nullptr);
      cv::putText(input_img, line1.str(), kImageSize - line1_size,
                  cv::FONT_HERSHEY_PLAIN, 1, kWhite);
      cv::putText(input_img, line2.str(), cv::Point(0, 40),
                  cv::FONT_HERSHEY_PLAIN, 2, kWhite);
      cv::imshow("img", input_img);
    } else {
      std::cout << line1.str() << "\t" << line2.str() << std::endl;
    }
  }

  Robot *robot() { return robot_; }

private:
  int maybe_fire_ = 0;
  Mat img_;
  Mat gray_;
  Robot *robot_;
  time_point last_action_;
  time_point last_fire_;
  std::unique_ptr<cv::CascadeClassifier> face_detector_{
      new cv::CascadeClassifier};
};

struct LatestImage {
  time_point timestamp;
  Robot::Pos pos;
  cv::Mat image;
};

void DetectImages(Recognizer *recognizer, int argc, char **argv) {
  cv::Mat image;
  for (int i = 0; i < argc; ++i) {
    std::cout << "=== " << argv[i] << std::endl;
    image = cv::imread(argv[i], 1);
    if (!image.data) {
      std::cerr << "error reading image " << argv[i] << std::endl;
      continue;
    }
    auto start = now();
    recognizer->Detect(start, recognizer->robot()->tell(), image);
    auto latency_ms = (now() - start) / std::chrono::milliseconds(1);
    std::cout << "latency: " << latency_ms << " ms" << std::endl;
    if (cv::waitKey(FLAGS_wait_between_images ? 0 : 1) == 'q')
      break;
  }
}

void DetectWebcam(CaptureSource *capture, Recognizer *recognizer,
                  Robot *robot) {
  std::mutex mu;
  std::condition_variable latest_image_cv;
  LatestImage latest_image;
  std::atomic<bool> done(false);
  bool latest_image_ready = false;

  std::thread capture_thread([&] {
    while (!done.load(std::memory_order_relaxed)) {
      cv::Mat image;
      if (!capture->grab()) {  // Defer decoding until after we calculate the timestamp.
        std::cerr << "error grabbing from --webcam=" << FLAGS_webcam
                  << std::endl;
      } else {
        time_point capture_time = now();
        auto pos = robot->tell();
        if (!capture->retrieve(&image)) {
          std::cerr << "error retrieving from --webcam=" << FLAGS_webcam
                    << std::endl;
        }
        std::unique_lock<std::mutex> lock(mu);
        latest_image.timestamp = capture_time;
        latest_image.pos = pos;
        latest_image.image = image;
        latest_image_ready = true;
        lock.unlock();
        latest_image_cv.notify_one();
      }
    }
  });

  std::thread detect_thread([&] {
    LatestImage latest;
    for (int key = 0; key != 'q';) {
      {
        std::unique_lock<std::mutex> lock(mu);
        latest_image_cv.wait(lock, [&] { return latest_image_ready; });
        latest = std::move(latest_image);
        latest_image_ready = false;
      }
      recognizer->Detect(latest.timestamp, latest.pos, latest.image);
      key = cv::waitKey(1000 / 30);
      while (key == 'p')
        key = cv::waitKey(0); // Wait for another key to be pressed.
      const int16_t kManualMove = kFovInSteps.x / 4;
      const auto act = [&](Action::ActionEnum action, int16_t x, int16_t y) {
        DoAction(Action{action, Robot::Pos{x, y}}, robot->tell(), robot);
      };
      switch (key) {
      case 'w': act(Action::MOVE, 0, kManualMove); break;
      case 'a': act(Action::MOVE, -kManualMove, 0); break;
      case 's': act(Action::MOVE, 0, -kManualMove); break;
      case 'd': act(Action::MOVE, kManualMove, 0); break;
      case 'f': act(Action::FIRE, 0, 0); break;
      }
    }
    done = true;
  });

  detect_thread.join();
  capture_thread.join();
}

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  std::unique_ptr<Robot> robot = Robot::FromFlags();
  Recognizer recognizer(robot.get());
  if (FLAGS_raspicam) {
    RaspiCamCaptureSource raspicam(kImageSize.x, kImageSize.y);
    DetectWebcam(&raspicam, &recognizer, robot.get());
  } else if (argc == 1) {
    WebcamCaptureSource webcam(FLAGS_webcam, kImageSize.x, kImageSize.y);
    DetectWebcam(&webcam, &recognizer, robot.get());
  } else {
    DetectImages(&recognizer, argc - 1, argv + 1);
  }
  return 0;
}
