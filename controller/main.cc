#include <atomic>
#include <chrono>
#include <ctime>
#include <experimental/optional>
#include <iostream>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "capture.h"
#include "operators.h"
#include "robot.h"
#include "robot_from_flags.h"

DEFINE_int32(webcam, 0,
             "Webcam# to use. Usually 0 for built-in, 1+ for external.");
#ifdef ENABLE_RASPICAM
DEFINE_bool(raspicam, false, "Fetch images from raspicam.");
#endif
DEFINE_bool(
    wait_between_images, true,
    "When doing detection on images, set to true to wait after each image.");
DEFINE_bool(enable_fire, false, "Enable firing.");
DEFINE_bool(preview, true, "Enable preview window.");
DEFINE_bool(track, true,
            "Track faces. When disabled, turret moves manually only.");
DEFINE_bool(track_while_maybe_fire, true,
            "Keep tracking when within target zone.");
DEFINE_bool(tunnel_vision, true,
            "Check for faces close to target area first.");
DEFINE_string(save_directory, "",
              "Enable saving pictures/video and plath them in this directory.");
DEFINE_bool(save_video, true, "Enable saving video.");
DEFINE_uint64(
    preview_size, 0,
    "Set fixed size, last four digits for vertical resolution. e.g. 8000600 for"
    " 800x600");
DEFINE_double(webcam_skew_angle, 2, "Degrees to rotate camera output CW");
DEFINE_string(face_cascade_file,
              "haarcascades/haarcascade_frontalface_default.xml",
              "Path to haarcascade face model.");

namespace {
using std::experimental::optional;
using std::experimental::nullopt;

// static const cv::Point kTargetCenter(561, 287);  // v2 prototype, 8ft.
static const cv::Point kTargetCenter(588, 296);  // v1 bot, Chi-pressure, 8ft.
static const cv::Size kMinFaceSize(80, 80);
static const cv::Size kMaxFaceSize(110, 110);
static const auto kMinTimeBetweenFire = std::chrono::seconds(5);
static const auto kFireTime = std::chrono::milliseconds(500);
static const int kMinConsecutiveOnTargetToFire = 5;

#define FIND_EYES 0

// Preview.
static const cv::Scalar kBlue(255, 0, 0);
static const cv::Scalar kDarkBlue(128, 0, 0);
static const cv::Scalar kGreen(0, 255, 0);
static const cv::Scalar kRed(0, 0, 255);
static const cv::Scalar kTeal(255, 255, 0);
static const cv::Scalar kYellow(0, 255, 255);
static const cv::Scalar kWhite(255, 255, 255);
static const char kWindowName[] = "preview";

static const cv::Point kImageSize(1280, 720);
// Targeting.
static constexpr int kTargetSize = 4;
static const cv::Rect kTargetArea(kTargetCenter - kTargetSize / 2,
                                  cv::Size(kTargetSize, kTargetSize));
// Movement.
static const cv::Point kFovInSteps(700, 500);
static constexpr int kMinStep = 2;
static constexpr std::chrono::seconds kExtraVideoTime(2);

using time_point = std::chrono::time_point<std::chrono::high_resolution_clock>;
time_point now() { return std::chrono::high_resolution_clock::now(); }

class RotatedCaptureSource : public CaptureSource {
public:
  RotatedCaptureSource(std::unique_ptr<CaptureSource> source, float skew_angle)
      : source_(std::move(source)), skew_angle_(skew_angle) {}
  bool isOpened() final { return source_->isOpened(); }
  bool grab() final { return source_->grab(); }
  bool retrieve(cv::Mat *image) {
    if (!source_->retrieve(&temp_))
      return false;
    cv::warpAffine(temp_, *image, rotation_matrix_, size_);
    return true;
  }
  std::string describe() {
    std::ostringstream buf;
    buf << source_->describe() << " rotated " << skew_angle_ << " degrees";
    return buf.str();
  }
  cv::Size size() const { return source_->size(); }

private:
  const std::unique_ptr<CaptureSource> source_;
  const cv::Size size_{source_->size()};
  float skew_angle_;
  const cv::Mat rotation_matrix_{cv::getRotationMatrix2D(
      cv::Point(size_.width / 2, size_.height / 2), skew_angle_, 1.0)};
  cv::Mat temp_;
};

struct Action {
  enum ActionEnum { MOVE, FIRE };
  ActionEnum action;
  optional<Robot::Pos> move_to;
};

std::ostream& operator<<(std::ostream &os, Action action) {
  switch (action.action) {
    case Action::MOVE:
      return os << "MOVE(" << *action.move_to << ")";
    case Action::FIRE:
      return os << "FIRE";
  }
  LOG(FATAL) << "Tried to log invalid action=" << action.action
             << " move_to=" << action.move_to.value_or(Robot::Pos{-1, -1});
  __builtin_unreachable();
}

std::string GetFileBase() {
  auto now =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  char buf[64];
  std::size_t time_len = std::strftime(buf, sizeof(buf), "%Y-%m-%d_%a_%H_%M_%S",
                                       std::localtime(&now));
  const std::string filebase =
      FLAGS_save_directory + "/" + std::string(buf, time_len);
  LOG(ERROR) << "Writing images to filebase=" << filebase;
  return filebase;
}

void SaveImage(const std::string &filename,
    const cv::Mat& img) {
  cv::imwrite(filename, img, {cv::IMWRITE_JPEG_OPTIMIZE, 1});
}

class Recognizer {
public:
  using Mat = cv::Mat;

  Recognizer() {
    CHECK(face_detector_->load(FLAGS_face_cascade_file))
        << " error loading " << FLAGS_face_cascade_file;
  }

  std::vector<cv::Rect> FilterFaces(std::vector<cv::Rect> faces,
                                    cv::Mat *input_img) {
    for (auto face = faces.begin(); face != faces.end();) {
      bool too_big = face->size().width > kMaxFaceSize.width ||
                     face->size().height > kMaxFaceSize.height;
      LOG_EVERY_N(INFO, 100) << " face width=" << face->width
                             << " height=" << face->height;
      if (input_img)
        PlotFeature(*input_img, *face, too_big ? kDarkBlue : kBlue);
      if (too_big) face = faces.erase(face);
      else ++face;
    }
    return faces;
  }

  std::vector<cv::Rect> GetCandidateFaces(cv::Mat& input_img) {
    cv::cvtColor(input_img, gray_, cv::COLOR_BGR2GRAY);
    // First do tunnel-vision detection.
    if (FLAGS_tunnel_vision) {
      std::vector<cv::Rect> faces =
          DetectMultiScale(face_detector_.get(), gray_(tunnel_vision_area_),
                           1.3, 5, kMinFaceSize);
      faces = FilterFaces(faces, /*input_img=*/nullptr);
      if (!faces.empty()) {
        for (cv::Rect& face : faces) {
          face.x += tunnel_vision_area_.x;
          face.y += tunnel_vision_area_.y;
        }
        return FilterFaces(faces, &input_img);  // Just to re-plot.
      }
    }
    // Then full-scale.
    std::vector<cv::Rect> faces =
        DetectMultiScale(face_detector_.get(), gray_, 1.3, 5, kMinFaceSize);
    return FilterFaces(faces, &input_img);
  }

  optional<Action> Detect(time_point timestamp, Robot::Pos pos,
                          cv::Mat &input_img) {
    auto start_decide = now();
    if (timestamp - last_action_ <=  std::chrono::milliseconds(10)) {
      return nullopt;
    }
    std::ostringstream line1;
    std::ostringstream line2;
    auto faces = GetCandidateFaces(input_img);
    PlotFeature(input_img, kTargetArea, kTeal);
    if (!FLAGS_track) {
      line2 << "NOTRACK ";
    }
    if (!FLAGS_enable_fire) {
      line2 << "NOFIRE ";
    }
    line2 << "pos=" << pos << " ";
    optional<Action> action;
    if (!faces.empty()) {
      auto face = BestFace(faces);
      cv::Rect mouth = GuessMouthLocation(face);
      PlotFeature(input_img, mouth, kYellow);
      action = ChooseAction(input_img, Center(mouth), pos);
      if (action) {
        line2 << *action << " ";
      }
      if (maybe_fire_ > 0) {
        line2 << "maybe_fire(" << maybe_fire_ << ") ";
      }
    }
    auto decide_time = now();
    line1 << "i2d latency "
          << ((decide_time - timestamp) / std::chrono::milliseconds(1))
          << " ms / d2d latency "
          << ((decide_time - last_decide_) / std::chrono::milliseconds(1))
          << " ms / detect time "
          << ((decide_time - start_decide) / std::chrono::milliseconds(1))
          << "ms";
    last_decide_ = decide_time;
    if (FLAGS_preview) {
      const cv::Size line1_size =
          cv::getTextSize(line1.str(), cv::FONT_HERSHEY_PLAIN,
                          /*fontScale=*/1, /*thickness=*/1, nullptr);
      cv::putText(input_img, line1.str(), kImageSize - line1_size,
                  cv::FONT_HERSHEY_PLAIN, 1, kWhite);
      cv::putText(input_img, line2.str(), cv::Point(0, 40),
                  cv::FONT_HERSHEY_PLAIN, 2, kWhite);
      cv::imshow(kWindowName, input_img);
    } else {
      std::cout << line1.str() << "\t" << line2.str() << std::endl;
    }
    return action;
  }

private:

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
      if (sqdist(Center(face), kTargetCenter) <
              sqdist(Center(best_face), kTargetCenter)) {
        best_face = face;
      }
    }
    return best_face;
  }

  optional<Action> ChooseAction(const cv::Mat &input_img,
                                const cv::Point &mouth, const Robot::Pos &pos) {
    CHECK(input_img.rows == kImageSize.y)
        << "rows=" << input_img.rows << " expected_height=" << kImageSize.y;
    CHECK(input_img.cols == kImageSize.x) << "cols=" << input_img.cols
                                          << " expected_width=" << kImageSize.x;
    auto vec = (mouth - kTargetCenter) * kFovInSteps / kImageSize;
    if (!FLAGS_enable_fire) maybe_fire_ = 0;
    if (abs(vec.x) > kMinStep || abs(vec.y) > kMinStep) {
      maybe_fire_ = 0;
    } else {
      auto fire_time = now();
      if (++maybe_fire_ > kMinConsecutiveOnTargetToFire &&
          fire_time - last_fire_ > kMinTimeBetweenFire) {
        maybe_fire_ = 0;
        last_fire_ = fire_time;
        return Action{Action::FIRE};
      }
    }

    if (!FLAGS_track_while_maybe_fire && maybe_fire_ > 0)
      return nullopt;
    return Action{Action::MOVE, pos.add(vec.x, vec.y)};
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
  int maybe_fire_ = 0;
  Mat img_;
  Mat gray_;
  time_point last_action_;
  time_point last_fire_;
  time_point last_decide_;
  std::unique_ptr<cv::CascadeClassifier> face_detector_{
      new cv::CascadeClassifier};
  cv::Rect tunnel_vision_area_{kTargetCenter.x - kMinFaceSize.width,
                               kTargetCenter.y - kMinFaceSize.height,
                               kMinFaceSize.width * 2, kMinFaceSize.height * 2};
};

void DetectImages(Recognizer *recognizer, Robot *robot, int argc, char **argv) {
  cv::Mat image;
  for (int i = 0; i < argc; ++i) {
    std::cout << "=== " << argv[i] << std::endl;
    image = cv::imread(argv[i], 1);
    if (!image.data) {
      std::cerr << "error reading image " << argv[i] << std::endl;
      continue;
    }
    auto start = now();
    recognizer->Detect(start, robot->tell(), image); // Ignore returned action.
    auto latency_ms = (now() - start) / std::chrono::milliseconds(1);
    std::cout << "latency: " << latency_ms << " ms" << std::endl;
    if (cv::waitKey(FLAGS_wait_between_images ? 0 : 1) == 'q')
      break;
  }
}

void DetectWebcam(AsyncCaptureSource *capture, Recognizer *recognizer,
                  Robot *robot) {
  AsyncCaptureSource::LatestImage latest;
  auto fire = [&](bool is_manual) {
    if (!is_manual && !FLAGS_enable_fire) return;
    if (!is_manual) FLAGS_enable_fire = false;
    if (FLAGS_save_directory.empty()) {
      robot->fire(kFireTime);
      return;
    }
    const std::string filebase = GetFileBase();
    SaveImage(filebase + "+annotated.jpg", latest.image);
    optional<AsyncCaptureSource::CaptureScope> capture_control;
    if (FLAGS_save_video) {
      capture_control.emplace(capture->StartCapture(filebase + ".avi"));
    }
    SaveImage(filebase + "+0000ms.jpg", capture->next_image().image);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot->fire(kFireTime);
    SaveImage(filebase + "+0500ms.jpg", capture->next_image().image);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    SaveImage(filebase + "+1000ms.jpg", capture->next_image().image);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    SaveImage(filebase + "+1500ms.jpg", capture->next_image().image);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    SaveImage(filebase + "+2000ms.jpg", capture->next_image().image);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    SaveImage(filebase + "+2500ms.jpg", capture->next_image().image);
    if (capture_control) std::this_thread::sleep_for(kExtraVideoTime);
  };
  int M = 4;
  while (true) {
    latest = capture->next_image();
    if (optional<Action> action =
            recognizer->Detect(latest.timestamp, latest.pos, latest.image)) {
      switch (action->action) {
      case Action::MOVE:
        if (FLAGS_track) robot->moveTo(*action->move_to);
        break;
      case Action::FIRE: fire(/*is_manual=*/false); break;
      }
    }
    int key = cv::waitKey(1);
    while (key == 'p')
      key = cv::waitKey(0); // Wait for another key to be pressed.
    switch (key) {
    case -1:
    case 255: break;
    case 'h':
    case '?':
      std::cout << "h or ?: this help message" << std::endl
                << "z/x: reduce/increase manual step size" << std::endl
                << "t: stop/start tracking" << std::endl
                << "y: enable/disable firing" << std::endl
                << "v: toggle updating preview window" << std::endl
                << "wasd or arrow keys: manually move turret" << std::endl
                << "f: manually fire" << std::endl;
      break;
    case 'z':
      if (M<64) M *= 4; else M = 64;
      std::cout << "step divisor=" << M << std::endl;
      break;
    case 'x':
      if (M>4) M /= 4; else M = 1;
      std::cout << "step divisor=" << M << std::endl;
      break;
    case 't': FLAGS_track = !FLAGS_track; break;
    case 'y': FLAGS_enable_fire = !FLAGS_enable_fire; break;
    case 'v': FLAGS_preview = !FLAGS_preview; break;
    case /*up_arrow=*/82:
    case 'w': robot->moveTo(robot->tell().add(0, -kFovInSteps.y/M)); break;
    case /*left_arrow=*/81:
    case 'a': robot->moveTo(robot->tell().add(-kFovInSteps.x/M, 0)); break;
    case /*down_arrow=*/84:
    case 's': robot->moveTo(robot->tell().add(0, kFovInSteps.y/M)); break;
    case /*right_arrow=*/83:
    case 'd': robot->moveTo(robot->tell().add(kFovInSteps.x/M, 0)); break;
    case 'f': fire(/*is_manual=*/true); break;
    case '`': robot->fire(std::chrono::milliseconds(100)); break;
    case 'q':
      return; // Quit.
    case /*ESC*/27:
    default:
      std::cout << "got key=" << key << std::endl;
    }
  }
}

std::unique_ptr<CaptureSource> GetCaptureSource(int argc) {
  std::unique_ptr<CaptureSource> source;

#ifdef ENABLE_RASPICAM
  if (FLAGS_raspicam) {
    source =
        std::make_unique<RaspiCamCaptureSource>(kImageSize.x, kImageSize.y);
  }
#endif

  if (!source && argc == 1) {
    source = std::make_unique<WebcamCaptureSource>(FLAGS_webcam, kImageSize.x,
                                                   kImageSize.y);
  }
  if (source && FLAGS_webcam_skew_angle != 0) {
    source = std::make_unique<RotatedCaptureSource>(std::move(source),
                                                    FLAGS_webcam_skew_angle);
  }
  return source;
}

}  // namespace

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_preview) {
    cv::namedWindow(kWindowName,
                    cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL |
                        cv::WINDOW_GUI_EXPANDED);
    if (FLAGS_preview_size) {
      cv::resizeWindow(kWindowName, FLAGS_preview_size / 10000,
                       FLAGS_preview % 10000);
    }
  }

  std::unique_ptr<Robot> robot = RobotFromFlags();
  Recognizer recognizer;
  if (std::unique_ptr<CaptureSource> source = GetCaptureSource(argc)) {
    AsyncCaptureSource async_source(robot.get(), std::move(source));
    DetectWebcam(&async_source, &recognizer, robot.get());
  } else {
    DetectImages(&recognizer, robot.get(), argc - 1, argv + 1);
  }
  return 0;
}
