/*
   Copyright (C) 2026 ArduPilot.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ArgosCameraController.hh"
#include "SocketUDP.hh"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>

#include <gz/msgs/double.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/common/Console.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

class ArgosCameraController::Impl
{
  public: struct CaptureRequest
  {
    uint32_t requestId {};
    int32_t imageIndex {};
    uint32_t timeBootMs {};
    int32_t lat {};
    int32_t lon {};
    int32_t altMm {};
    int32_t relativeAltMm {};
    std::array<double, 4> vehicleQuaternion {};
  };

  public: struct CameraSnapshot
  {
    double hfov {};
    double zoomPercent {};
    double zoomFactor {1.0};
    std::array<double, 4> cameraQuaternion {};
    double yaw {};
    double roll {};
    double pitch {};
    bool valid {};
  };

  public: void OnZoomPercent(const msgs::Double &_msg);
  public: void OnImage(const msgs::Image &_msg);
  public: void OnThermalImage(const msgs::Image &_msg);
  public: void OnRecordingResult(const msgs::StringMsg &_msg);
  public: void SendResult(const std::string &_message);
  public: void SaveCapture(const msgs::Image &_msg,
                           const CaptureRequest &_request);
  public: bool InitializeCamera(EntityComponentManager &_ecm);

  public: transport::Node node;
  public: transport::Node::Publisher zoomPublisher;
  public: transport::Node::Publisher commandPublisher;
  public: transport::Node::Publisher recordPublisher;
  public: std::unique_ptr<SocketUDP> sitlSocket;
  public: std::string commandTopic{"/argos/camera/zoom/percent"};
  public: std::string zoomTopic;
  public: std::string imageTopic;
  public: std::string thermalImageTopic;
  public: std::string recordCommandTopic{"/argos/camera/record/command"};
  public: std::string recordResultTopic{"/argos/camera/record/result"};
  public: std::string captureDirectory{"/tmp/argos-camera"};
  public: Model model;
  public: Link pitchLink;
  public: Sensor cameraSensor;
  public: Sensor thermalSensor;
  public: Entity yawJoint{kNullEntity};
  public: Entity rollJoint{kNullEntity};
  public: Entity pitchJoint{kNullEntity};
  public: rendering::ScenePtr scene;
  public: rendering::CameraPtr camera;
  public: std::string cameraName;
  public: std::mutex stateMutex;
  public: std::mutex captureMutex;
  public: std::deque<CaptureRequest> captureRequests;
  public: CameraSnapshot snapshot;
  public: double minPercent{0.0};
  public: double maxPercent{100.0};
  public: double minZoom{1.0};
  public: double maxZoom{125.0};
  public: double referenceHfov{2.0};
  public: uint16_t sitlUdpPort{0};
  public: uint16_t sitlReplyPort{0};
  public: bool imageSubscribed {};
  public: bool thermalImageSubscribed {};
  public: uint64_t thermalFrameCount {};
  public: std::chrono::steady_clock::time_point thermalFirstFrameTime;
  public: double thermalFirstSimTime {};
  public: bool recordResultSubscribed {};
};

//////////////////////////////////////////////////
void ArgosCameraController::Impl::OnZoomPercent(const msgs::Double &_msg)
{
  const double percent = _msg.data();
  if (!std::isfinite(percent))
  {
    gzerr << "ArgosCameraController: ignoring non-finite zoom percentage.\n";
    return;
  }

  if (percent < this->minPercent || percent > this->maxPercent)
  {
    gzwarn << "ArgosCameraController: clamping zoom percentage "
           << percent << " to [" << this->minPercent << ", "
           << this->maxPercent << "].\n";
  }
  const double clampedPercent = std::max(
      this->minPercent, std::min(this->maxPercent, percent));
  const double fraction = (clampedPercent - this->minPercent) /
      (this->maxPercent - this->minPercent);
  const double zoomFactor = this->minZoom +
      fraction * (this->maxZoom - this->minZoom);

  gzmsg << "ArgosCameraController: received zoom percentage = "
        << percent << ", mapped zoom factor = " << zoomFactor
        << ", publishing to CameraZoomPlugin = "
        << this->zoomTopic << ".\n";

  msgs::Double zoomCommand;
  zoomCommand.set_data(zoomFactor);
  if (!this->zoomPublisher.Publish(zoomCommand))
  {
    gzerr << "ArgosCameraController: failed to publish zoom factor "
          << zoomFactor << " to [" << this->zoomTopic << "].\n";
    return;
  }

  std::lock_guard<std::mutex> lock(this->stateMutex);
  this->snapshot.zoomPercent = clampedPercent;
  this->snapshot.zoomFactor = zoomFactor;
}

//////////////////////////////////////////////////
bool ArgosCameraController::Impl::InitializeCamera(
    EntityComponentManager &_ecm)
{
  if (!this->cameraSensor.Valid(_ecm))
  {
    const Entity linkEntity = this->model.LinkByName(_ecm, "pitch_link");
    if (linkEntity == kNullEntity)
      return false;
    this->pitchLink = Link(linkEntity);
    const Entity cameraEntity =
        this->pitchLink.SensorByName(_ecm, "rgb_camera");
    if (cameraEntity == kNullEntity)
      return false;
    this->cameraSensor = Sensor(cameraEntity);
    this->yawJoint = this->model.JointByName(_ecm, "yaw_joint");
    this->rollJoint = this->model.JointByName(_ecm, "roll_joint");
    this->pitchJoint = this->model.JointByName(_ecm, "pitch_joint");
  }
  if (!this->cameraSensor.Valid(_ecm))
    return false;
  if (this->imageTopic.empty() || this->cameraName.empty())
  {
    auto maybeTopic = this->cameraSensor.Topic(_ecm);
    if (!maybeTopic.has_value())
      return false;
    if (this->imageTopic.empty())
      this->imageTopic = maybeTopic.value();
    if (this->cameraName.empty())
    {
      this->cameraName = removeParentScope(
          scopedName(this->cameraSensor.Entity(), _ecm, "::", false), "::");
    }
  }

  if (!this->camera && !this->cameraName.empty())
  {
    if (rendering::loadedEngines().empty())
      return false;
    if (!this->scene)
      this->scene = rendering::sceneFromFirstRenderEngine();
    if (!this->scene || !this->scene->IsInitialized())
      return false;
    this->camera = std::dynamic_pointer_cast<rendering::Camera>(
        this->scene->SensorByName(this->cameraName));
  }
  return static_cast<bool>(this->camera);
}

//////////////////////////////////////////////////
void ArgosCameraController::Impl::SendResult(
    const std::string &_message)
{
  if (!this->sitlSocket || this->sitlReplyPort == 0)
  {
    gzerr << "ArgosCameraController: cannot return bridge result; "
          << "SITL reply socket is not configured.\n";
    return;
  }
  if (this->sitlSocket->sendto(
          _message.data(), _message.size(), "127.0.0.1",
          this->sitlReplyPort) !=
      static_cast<ssize_t>(_message.size()))
  {
    gzerr << "ArgosCameraController: failed to return bridge result.\n";
  }
}

//////////////////////////////////////////////////
void ArgosCameraController::Impl::OnRecordingResult(
    const msgs::StringMsg &_msg)
{
  const std::string result = _msg.data();
  const auto separator = result.find('|');
  if (separator == std::string::npos)
  {
    gzerr << "ArgosCameraController: invalid recording result message.\n";
    return;
  }
  this->SendResult("V|" + result);
}

//////////////////////////////////////////////////
void ArgosCameraController::Impl::SaveCapture(
    const msgs::Image &_msg, const CaptureRequest &_request)
{
  CameraSnapshot cameraState;
  {
    std::lock_guard<std::mutex> lock(this->stateMutex);
    cameraState = this->snapshot;
  }

  const uint64_t minimumRowSize =
      static_cast<uint64_t>(_msg.width()) * 3U;
  const uint64_t expectedSize =
      static_cast<uint64_t>(_msg.step()) * _msg.height();
  if (!cameraState.valid || _msg.width() == 0 || _msg.height() == 0 ||
      _msg.pixel_format_type() != msgs::PixelFormatType::RGB_INT8 ||
      _msg.step() < minimumRowSize ||
      _msg.data().size() < expectedSize)
  {
    this->SendResult("C|" + std::to_string(_request.requestId) +
                     "|ERR|invalid_sensor_frame_or_camera_state");
    return;
  }
  for (double value : _request.vehicleQuaternion)
  {
    if (!std::isfinite(value))
    {
      this->SendResult("C|" + std::to_string(_request.requestId) +
                       "|ERR|invalid_vehicle_attitude");
      return;
    }
  }

  if (mkdir(this->captureDirectory.c_str(), 0755) != 0 && errno != EEXIST)
  {
    this->SendResult("C|" + std::to_string(_request.requestId) +
                     "|ERR|capture_directory_unavailable");
    return;
  }

  const auto now = std::chrono::system_clock::now();
  const auto epochUs = std::chrono::duration_cast<std::chrono::microseconds>(
      now.time_since_epoch()).count();
  const std::time_t epochSeconds =
      std::chrono::system_clock::to_time_t(now);
  std::tm utc {};
  gmtime_r(&epochSeconds, &utc);
  char timestamp[40];
  strftime(timestamp, sizeof(timestamp), "%Y%m%dT%H%M%S", &utc);

  const int32_t captureId = _request.imageIndex + 1;
  char filename[128];
  snprintf(filename, sizeof(filename), "capture_%06d_%s_%06lldZ.jpg",
           captureId, timestamp,
           static_cast<long long>(epochUs % 1000000));
  const std::string imagePath = this->captureDirectory + "/" + filename;
  const std::string sidecarPath = imagePath + ".json";

  cv::Mat rgb(static_cast<int>(_msg.height()), static_cast<int>(_msg.width()),
              CV_8UC3, const_cast<char *>(_msg.data().data()),
              _msg.step());
  cv::Mat bgr;
  cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
  bool imageSaved = false;
  try
  {
    imageSaved = cv::imwrite(imagePath, bgr);
  }
  catch (const cv::Exception &error)
  {
    gzerr << "ArgosCameraController: image encode failed: "
          << error.what() << ".\n";
  }
  if (!imageSaved)
  {
    this->SendResult("C|" + std::to_string(_request.requestId) +
                     "|ERR|image_write_failed");
    return;
  }

  gz::math::Quaterniond cameraNedFromEnu(
      0.0, std::sqrt(0.5), std::sqrt(0.5), 0.0);
  const gz::math::Quaterniond cameraNed =
      cameraNedFromEnu * gz::math::Quaterniond(
          cameraState.cameraQuaternion[0], cameraState.cameraQuaternion[1],
          cameraState.cameraQuaternion[2], cameraState.cameraQuaternion[3]);
  const gz::math::Quaterniond vehicleQuaternion(
      _request.vehicleQuaternion[0], _request.vehicleQuaternion[1],
      _request.vehicleQuaternion[2], _request.vehicleQuaternion[3]);
  const auto vehicleEuler = vehicleQuaternion.Euler();
  const double actualZoomFactor = this->referenceHfov / cameraState.hfov;
  const double actualZoomPercent =
      ((actualZoomFactor - this->minZoom) /
       (this->maxZoom - this->minZoom)) *
      (this->maxPercent - this->minPercent) + this->minPercent;
  const std::string fileUrl = "file://" + imagePath;
  std::ofstream sidecar(sidecarPath);
  if (!sidecar)
  {
    std::remove(imagePath.c_str());
    this->SendResult("C|" + std::to_string(_request.requestId) +
                     "|ERR|metadata_write_failed");
    return;
  }
  sidecar << "{\n"
          << "  \"capture_id\": " << captureId << ",\n"
          << "  \"time_utc_unix_us\": " << epochUs << ",\n"
          << "  \"vehicle_state_time_boot_ms\": "
          << _request.timeBootMs << ",\n"
          << "  \"image_file\": \"" << imagePath << "\",\n"
          << "  \"width\": " << _msg.width() << ",\n"
          << "  \"height\": " << _msg.height() << ",\n"
          << "  \"pixel_format\": \"RGB_INT8\",\n"
          << "  \"vehicle\": {\"latitude_deg_e7\": " << _request.lat
          << ", \"longitude_deg_e7\": " << _request.lon
          << ", \"altitude_msl_mm\": " << _request.altMm
          << ", \"altitude_agl_mm\": "
          << _request.relativeAltMm << ", \"quaternion_wxyz\": ["
          << _request.vehicleQuaternion[0] << ", "
          << _request.vehicleQuaternion[1] << ", "
          << _request.vehicleQuaternion[2] << ", "
          << _request.vehicleQuaternion[3] << "], \"roll_rad\": "
          << vehicleEuler.X() << ", \"pitch_rad\": " << vehicleEuler.Y()
          << ", \"yaw_rad\": " << vehicleEuler.Z() << "},\n"
          << "  \"gimbal\": {\"yaw_rad\": " << cameraState.yaw
          << ", \"roll_rad\": " << cameraState.roll
          << ", \"pitch_rad\": " << cameraState.pitch << "},\n"
          << "  \"camera\": {\"world_quaternion_enu_wxyz\": ["
          << cameraState.cameraQuaternion[0] << ", "
          << cameraState.cameraQuaternion[1] << ", "
          << cameraState.cameraQuaternion[2] << ", "
          << cameraState.cameraQuaternion[3]
          << "], \"quaternion_ned_wxyz\": ["
          << cameraNed.W() << ", " << cameraNed.X() << ", "
          << cameraNed.Y() << ", " << cameraNed.Z()
          << "], \"actual_hfov_rad\": " << cameraState.hfov
          << ", \"actual_zoom_factor\": " << actualZoomFactor
          << ", \"actual_zoom_percent\": " << actualZoomPercent
          << ", \"commanded_zoom_percent\": "
          << cameraState.zoomPercent << ", \"commanded_zoom_factor\": "
          << cameraState.zoomFactor << "}\n}\n";
  sidecar.close();
  if (!sidecar)
  {
    std::remove(imagePath.c_str());
    std::remove(sidecarPath.c_str());
    this->SendResult("C|" + std::to_string(_request.requestId) +
                     "|ERR|metadata_write_failed");
    return;
  }

  std::ostringstream result;
  result << "C|" << _request.requestId << "|OK|" << epochUs << "|"
         << fileUrl << "|" << cameraNed.W() << "|" << cameraNed.X()
         << "|" << cameraNed.Y() << "|" << cameraNed.Z();
  this->SendResult(result.str());
}

//////////////////////////////////////////////////
void ArgosCameraController::Impl::OnImage(const msgs::Image &_msg)
{
  CaptureRequest request;
  {
    std::lock_guard<std::mutex> lock(this->captureMutex);
    if (this->captureRequests.empty())
      return;
    request = this->captureRequests.front();
    this->captureRequests.pop_front();
  }
  this->SaveCapture(_msg, request);
}

//////////////////////////////////////////////////
void ArgosCameraController::Impl::OnThermalImage(const msgs::Image &_msg)
{
  ++this->thermalFrameCount;
  if (this->thermalFrameCount == 1)
  {
    this->thermalFirstFrameTime = std::chrono::steady_clock::now();
    const auto &stamp = _msg.header().stamp();
    this->thermalFirstSimTime =
        static_cast<double>(stamp.sec()) +
        static_cast<double>(stamp.nsec()) * 1e-9;
  }
  if (this->thermalFrameCount % 30 != 0)
    return;

  const auto now = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(
      now - this->thermalFirstFrameTime).count();
  const double fps = elapsed > 0.0 ?
      static_cast<double>(this->thermalFrameCount - 1) / elapsed : 0.0;
  const auto &stamp = _msg.header().stamp();
  const double simTime = static_cast<double>(stamp.sec()) +
      static_cast<double>(stamp.nsec()) * 1e-9;
  const double simElapsed = simTime - this->thermalFirstSimTime;
  const double simFps = simElapsed > 0.0 ?
      static_cast<double>(this->thermalFrameCount - 1) / simElapsed : 0.0;
  gzmsg << "[Thermal] frame=" << this->thermalFrameCount
        << " resolution=" << _msg.width() << "x" << _msg.height()
        << " format="
        << msgs::PixelFormatType_Name(_msg.pixel_format_type())
        << " sim_fps=" << simFps << " wall_fps=" << fps << std::endl;
}

//////////////////////////////////////////////////
ArgosCameraController::~ArgosCameraController() = default;

//////////////////////////////////////////////////
ArgosCameraController::ArgosCameraController() :
    impl(std::make_unique<ArgosCameraController::Impl>())
{
}

//////////////////////////////////////////////////
void ArgosCameraController::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  this->impl->model = Model(_entity);
  if (_sdf->HasElement("command_topic"))
  {
    this->impl->commandTopic = _sdf->Get<std::string>("command_topic");
  }
  if (_sdf->HasElement("zoom_topic"))
  {
    this->impl->zoomTopic = _sdf->Get<std::string>("zoom_topic");
  }
  if (_sdf->HasElement("min_percent"))
  {
    this->impl->minPercent = _sdf->Get<double>("min_percent");
  }
  if (_sdf->HasElement("max_percent"))
  {
    this->impl->maxPercent = _sdf->Get<double>("max_percent");
  }
  if (_sdf->HasElement("min_zoom"))
  {
    this->impl->minZoom = _sdf->Get<double>("min_zoom");
  }
  if (_sdf->HasElement("max_zoom"))
  {
    this->impl->maxZoom = _sdf->Get<double>("max_zoom");
  }
  if (_sdf->HasElement("sitl_udp_port"))
  {
    const int port = _sdf->Get<int>("sitl_udp_port");
    if (port < 1 || port > 65535)
    {
      gzerr << "ArgosCameraController: invalid SITL UDP port ["
            << port << "].\n";
      return;
    }
    this->impl->sitlUdpPort = static_cast<uint16_t>(port);
  }
  if (_sdf->HasElement("sitl_reply_udp_port"))
  {
    const int port = _sdf->Get<int>("sitl_reply_udp_port");
    if (port < 1 || port > 65535)
    {
      gzerr << "ArgosCameraController: invalid SITL reply UDP port ["
            << port << "].\n";
      return;
    }
    this->impl->sitlReplyPort = static_cast<uint16_t>(port);
  }
  if (_sdf->HasElement("record_command_topic"))
  {
    this->impl->recordCommandTopic =
        _sdf->Get<std::string>("record_command_topic");
  }
  if (_sdf->HasElement("record_result_topic"))
  {
    this->impl->recordResultTopic =
        _sdf->Get<std::string>("record_result_topic");
  }
  if (_sdf->HasElement("image_topic"))
  {
    this->impl->imageTopic = _sdf->Get<std::string>("image_topic");
  }
  if (_sdf->HasElement("capture_directory"))
  {
    this->impl->captureDirectory =
        _sdf->Get<std::string>("capture_directory");
  }
  if (_sdf->HasElement("reference_hfov"))
  {
    this->impl->referenceHfov = _sdf->Get<double>("reference_hfov");
  }

  if (this->impl->commandTopic.empty() || this->impl->zoomTopic.empty() ||
      this->impl->recordCommandTopic.empty() ||
      this->impl->recordResultTopic.empty() ||
      this->impl->captureDirectory.empty() ||
      !std::isfinite(this->impl->minPercent) ||
      !std::isfinite(this->impl->maxPercent) ||
      !std::isfinite(this->impl->minZoom) ||
      !std::isfinite(this->impl->maxZoom) ||
      !std::isfinite(this->impl->referenceHfov) ||
      this->impl->minPercent >= this->impl->maxPercent ||
      this->impl->minZoom < 1.0 ||
      this->impl->minZoom >= this->impl->maxZoom ||
      this->impl->referenceHfov <= 0.0)
  {
    gzerr << "ArgosCameraController: invalid zoom bridge configuration.\n";
    return;
  }

  this->impl->zoomPublisher =
      this->impl->node.Advertise<msgs::Double>(this->impl->zoomTopic);
  if (!this->impl->zoomPublisher)
  {
    gzerr << "ArgosCameraController: unable to advertise zoom topic ["
          << this->impl->zoomTopic << "].\n";
    return;
  }
  this->impl->recordPublisher =
      this->impl->node.Advertise<msgs::StringMsg>(
          this->impl->recordCommandTopic);
  if (!this->impl->recordPublisher)
  {
    gzerr << "ArgosCameraController: unable to advertise recording command "
          << "topic [" << this->impl->recordCommandTopic << "].\n";
    return;
  }

  if (!this->impl->node.Subscribe(
          this->impl->commandTopic,
          &ArgosCameraController::Impl::OnZoomPercent,
          this->impl.get()))
  {
    gzerr << "ArgosCameraController: unable to subscribe to command topic ["
          << this->impl->commandTopic << "].\n";
    return;
  }

  if (this->impl->sitlUdpPort > 0)
  {
    if (this->impl->sitlReplyPort == 0)
    {
      gzerr << "ArgosCameraController: SITL reply port is required when "
            << "the SITL command port is enabled.\n";
      return;
    }
    this->impl->commandPublisher =
        this->impl->node.Advertise<msgs::Double>(this->impl->commandTopic);
    if (!this->impl->commandPublisher)
    {
      gzerr << "ArgosCameraController: unable to advertise SITL zoom input ["
            << this->impl->commandTopic << "].\n";
      return;
    }
    this->impl->sitlSocket =
        std::make_unique<SocketUDP>(true, false);
    if (!this->impl->sitlSocket->bind("127.0.0.1",
                                      this->impl->sitlUdpPort))
    {
      gzerr << "ArgosCameraController: failed to bind SITL zoom UDP listener "
            << "on 127.0.0.1:" << this->impl->sitlUdpPort << ".\n";
      this->impl->sitlSocket.reset();
      return;
    }
    gzmsg << "ArgosCameraController: accepting SITL zoom percentages over "
          << "UDP 127.0.0.1:" << this->impl->sitlUdpPort
          << " and forwarding them to [" << this->impl->commandTopic
          << "].\n";
  }

  gzmsg << "ArgosCameraController: translating zoom percent on ["
        << this->impl->commandTopic << "] to zoom factor on ["
        << this->impl->zoomTopic << "].\n";
}

//////////////////////////////////////////////////
void ArgosCameraController::PreUpdate(
    const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  if (this->impl->InitializeCamera(_ecm))
  {
    if (!this->impl->imageSubscribed &&
        !this->impl->node.Subscribe(
            this->impl->imageTopic,
            &ArgosCameraController::Impl::OnImage,
            this->impl.get()))
    {
      gzerr << "ArgosCameraController: failed to subscribe to sensor image "
            << "topic [" << this->impl->imageTopic << "].\n";
    }
    else
    {
      this->impl->imageSubscribed = true;
    }

    if (!this->impl->thermalSensor.Valid(_ecm))
    {
      const Entity thermalEntity =
          this->impl->pitchLink.SensorByName(_ecm, "thermal_camera");
      if (thermalEntity != kNullEntity)
        this->impl->thermalSensor = Sensor(thermalEntity);
    }
    if (this->impl->thermalSensor.Valid(_ecm) &&
        !this->impl->thermalImageSubscribed)
    {
      const auto maybeTopic = this->impl->thermalSensor.Topic(_ecm);
      if (maybeTopic.has_value())
      {
        this->impl->thermalImageTopic = maybeTopic.value();
        if (!this->impl->node.Subscribe(
                this->impl->thermalImageTopic,
                &ArgosCameraController::Impl::OnThermalImage,
                this->impl.get()))
        {
          gzerr << "ArgosCameraController: failed to subscribe to thermal "
                << "sensor image topic [" << this->impl->thermalImageTopic
                << "].\n";
        }
        else
        {
          this->impl->thermalImageSubscribed = true;
          gzmsg << "ArgosCameraController: monitoring thermal sensor frames "
                << "on [" << this->impl->thermalImageTopic << "].\n";
        }
      }
    }
  }

  if (this->impl->camera)
  {
    const auto linkPose = this->impl->pitchLink.WorldPose(_ecm);
    const auto sensorPose = this->impl->cameraSensor.Pose(_ecm);
    const auto *yawPosition =
        _ecm.Component<components::JointPosition>(this->impl->yawJoint);
    const auto *rollPosition =
        _ecm.Component<components::JointPosition>(this->impl->rollJoint);
    const auto *pitchPosition =
        _ecm.Component<components::JointPosition>(this->impl->pitchJoint);
    if (linkPose && sensorPose && yawPosition && rollPosition &&
        pitchPosition && !yawPosition->Data().empty() &&
        !rollPosition->Data().empty() && !pitchPosition->Data().empty())
    {
      const auto cameraPose = linkPose.value() * sensorPose.value();
      const auto cameraRotation = cameraPose.Rot();
      std::lock_guard<std::mutex> lock(this->impl->stateMutex);
      this->impl->snapshot.hfov = this->impl->camera->HFOV().Radian();
      this->impl->snapshot.cameraQuaternion = {
          cameraRotation.W(), cameraRotation.X(),
          cameraRotation.Y(), cameraRotation.Z()};
      this->impl->snapshot.yaw = yawPosition->Data()[0];
      this->impl->snapshot.roll = rollPosition->Data()[0];
      this->impl->snapshot.pitch = pitchPosition->Data()[0];
      this->impl->snapshot.valid =
          std::isfinite(this->impl->snapshot.hfov) &&
          this->impl->snapshot.hfov > 0.0;
    }
  }

  if (!this->impl->recordResultSubscribed &&
      !this->impl->node.Subscribe(
          this->impl->recordResultTopic,
          &ArgosCameraController::Impl::OnRecordingResult,
          this->impl.get()))
  {
    gzerr << "ArgosCameraController: failed to subscribe to recording result "
          << "topic [" << this->impl->recordResultTopic << "].\n";
  }
  else
  {
    this->impl->recordResultSubscribed = true;
  }

  if (!this->impl->sitlSocket)
    return;

  char buffer[1024];
  while (true)
  {
    const auto received = this->impl->sitlSocket->recv(
        buffer, sizeof(buffer) - 1, 0);
    if (received <= 0)
      return;
    buffer[received] = '\0';

    if (buffer[0] == 'C' && buffer[1] == '|')
    {
      Impl::CaptureRequest request;
      const int parsed = sscanf(
          buffer, "C|%u|%d|%u|%d|%d|%d|%d|%lf|%lf|%lf|%lf",
          &request.requestId, &request.imageIndex, &request.timeBootMs,
          &request.lat, &request.lon, &request.altMm,
          &request.relativeAltMm, &request.vehicleQuaternion[0],
          &request.vehicleQuaternion[1], &request.vehicleQuaternion[2],
          &request.vehicleQuaternion[3]);
      if (parsed != 11)
      {
        gzwarn << "ArgosCameraController: ignoring malformed capture request.\n";
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(this->impl->captureMutex);
        if (this->impl->captureRequests.size() >= 8)
        {
          this->impl->SendResult(
              "C|" + std::to_string(request.requestId) +
              "|ERR|capture_queue_full");
          continue;
        }
        this->impl->captureRequests.push_back(request);
      }
      continue;
    }

    if (buffer[0] == 'V' && buffer[1] == '|')
    {
      unsigned requestId = 0;
      unsigned start = 0;
      if (sscanf(buffer, "V|%u|%u", &requestId, &start) != 2 || start > 1)
      {
        gzwarn << "ArgosCameraController: ignoring malformed video request.\n";
        continue;
      }
      msgs::StringMsg command;
      command.set_data(std::to_string(requestId) + "|" +
                       (start ? "start" : "stop"));
      if (!this->impl->recordPublisher.Publish(command))
      {
        this->impl->SendResult("V|" + std::to_string(requestId) +
                               "|ERR|record_command_publish_failed");
      }
      continue;
    }

    errno = 0;
    char *end = nullptr;
    const double percent = std::strtod(buffer, &end);
    while (end && (*end == ' ' || *end == '\t' ||
                   *end == '\r' || *end == '\n'))
    {
      ++end;
    }
    if (errno != 0 || end == buffer || !end || *end != '\0' ||
        !std::isfinite(percent))
    {
      gzwarn << "ArgosCameraController: ignoring invalid SITL zoom datagram.\n";
      continue;
    }

    msgs::Double command;
    command.set_data(percent);
    if (!this->impl->commandPublisher.Publish(command))
    {
      gzerr << "ArgosCameraController: failed to forward SITL zoom percentage "
            << percent << " to [" << this->impl->commandTopic << "].\n";
    }
  }
}

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::ArgosCameraController,
    gz::sim::System,
    gz::sim::systems::ArgosCameraController::ISystemConfigure,
    gz::sim::systems::ArgosCameraController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::ArgosCameraController,
    "ArgosCameraController")
