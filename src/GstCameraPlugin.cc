/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "GstCameraPlugin.hh"

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <cerrno>
#include <iostream>
#include <memory>
#include <mutex>
#include <cstdio>
#include <ctime>
#include <chrono>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/msgs/image.pb.h>
#include <gz/sim/Model.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

#include <opencv2/opencv.hpp>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

//////////////////////////////////////////////////

class GstCameraPlugin::Impl {
   public:
    void InitializeCamera();
    void StartStreaming();
    static void *StartThread(void *);
    void StartGstThread();

    void OnImage(const msgs::Image &msg);
    void OnVideoStreamEnable(const msgs::Boolean &_msg);
    void OnRecordingCommand(const msgs::StringMsg &_msg);
    void OnRenderTeardown();

    void StopStreaming();
    void StopGstThread();
    bool StartRecording(uint32_t requestId);
    bool StopRecording();
    void PublishRecordingResult(uint32_t requestId, const char *status,
                               const std::string &filePath);
    void ProcessPendingRecordingCommand();

    std::string udpHost{"127.0.0.1"};
    int udpPort{5600};
    bool useRtmpPipeline{false};
    std::string rtmpLocation;
    bool useBasicPipeline{false};
    bool useCuda{false};
    std::string imageTopic;
    std::string enableTopic;
    std::string recordCommandTopic;
    std::string recordResultTopic;
    std::string recordingDirectory{"/tmp/argos-camera"};
    unsigned int recordingRate{30};

    unsigned int width{0};
    unsigned int height{0};

    // Unused by actual pipeline since it's based on the gazebo topic rate?
    unsigned int rate{5};

    pthread_t threadId;
    bool isGstMainLoopActive{false};
    bool requestedStartStreaming{false};

    GMainLoop *gst_loop{nullptr};
    GstElement *source{nullptr};
    void CreateMpeg2tsPipeline(GstElement *pipeline);
    void CreateRtmpPipeline(GstElement *pipeline);
    void CreateGenericPipeline(GstElement *pipeline);
    GstElement *CreateEncoder();
    void HandleRecordingCommand(const msgs::StringMsg &_msg);

    bool is_initialised{false};
    bool recordingInitialized{false};
    bool pendingRecordingStart{false};
    uint32_t pendingRecordingRequestId{};
    bool isRecording{false};
    uint32_t nextRecordingId{1};
    std::mutex recordingMutex;
    std::string recordingFilePath;
    GstElement *recordPipeline{nullptr};
    GstElement *recordSource{nullptr};
    guint64 recordingFrameIndex{0};
    Sensor parentSensor;
    rendering::ScenePtr scene;
    rendering::CameraPtr camera;
    std::string cameraName;
    std::vector<common::ConnectionPtr> connections;
    transport::Node node;
    transport::Node::Publisher recordingResultPublisher;
};

//////////////////////////////////////////////////
GstCameraPlugin::GstCameraPlugin()
    : impl(std::make_unique<GstCameraPlugin::Impl>())
{
}

GstCameraPlugin::~GstCameraPlugin()
{
    impl->OnRenderTeardown();
}

void GstCameraPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
    impl->parentSensor = Sensor(_entity);

    if (!impl->parentSensor.Valid(_ecm))
    {
        gzerr << "GstCameraPlugin: must be attached to a camera sensor. "
                 "Failed to initialize" << std::endl;
        return;
    }

    if (auto maybeName = impl->parentSensor.Name(_ecm))
    {
        gzmsg << "GstCameraPlugin: attached to sensor ["
              << maybeName.value() << "]" << std::endl;
    }
    else
    {
        gzerr << "GstCameraPlugin: camera sensor has invalid name. "
                 "Failed to initialize" << std::endl;
        return;
    }

    if (_sdf->HasElement("udp_host"))
    {
        impl->udpHost = _sdf->Get<std::string>("udp_host");
    }

    if (_sdf->HasElement("udp_port"))
    {
        impl->udpPort = _sdf->Get<int>("udp_port");
    }
    gzmsg << "GstCameraPlugin: streaming video to "
          << impl->udpHost << ":"
          << impl->udpPort << std::endl;

    // uses MPEG2TS pipeline by default. RTMP and Generic are
    // mutually exclusive with priority to RTMP
    if (_sdf->HasElement("rtmp_location"))
    {
        impl->rtmpLocation = _sdf->Get<std::string>("rtmp_location");
        impl->useRtmpPipeline = true;

    }
    else if (_sdf->HasElement("use_basic_pipeline"))
    {
        impl->useBasicPipeline = _sdf->Get<bool>("use_basic_pipeline");
    }

    // Use CUDA for video encoding
    if (_sdf->HasElement("use_cuda"))
    {
        impl->useCuda = _sdf->Get<bool>("use_cuda");
    }

    if (_sdf->HasElement("start_streaming"))
    {
        impl->requestedStartStreaming =
            _sdf->Get<bool>("start_streaming");
    }

    if (_sdf->HasElement("image_topic"))
    {
        impl->imageTopic = _sdf->Get<std::string>("image_topic");
    }

    if (_sdf->HasElement("enable_topic"))
    {
        impl->enableTopic = _sdf->Get<std::string>("enable_topic");
    }
    if (_sdf->HasElement("record_command_topic"))
    {
        impl->recordCommandTopic =
            _sdf->Get<std::string>("record_command_topic");
    }
    if (_sdf->HasElement("record_result_topic"))
    {
        impl->recordResultTopic =
            _sdf->Get<std::string>("record_result_topic");
    }
    if (_sdf->HasElement("recording_directory"))
    {
        impl->recordingDirectory =
            _sdf->Get<std::string>("recording_directory");
    }
    if (_sdf->HasElement("recording_rate"))
    {
        const int rate = _sdf->Get<int>("recording_rate");
        if (rate < 1 || rate > 120)
        {
            gzerr << "GstCameraPlugin: invalid recording rate ["
                  << rate << "].\n";
            return;
        }
        impl->recordingRate = static_cast<unsigned int>(rate);
    }

    //! @note subscriptions are deferred to Pre-Update as the enclosing
    //  sensor must be fully initialised before entity - component queries
    //  for topics names etc. to succeed.

    // subscribe to events
    impl->connections.push_back(
        _eventMgr.Connect<gz::sim::events::RenderTeardown>(
            std::bind(&GstCameraPlugin::Impl::OnRenderTeardown, impl.get())));
}

void GstCameraPlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    if (impl->cameraName.empty())
    {
        Entity cameraEntity = impl->parentSensor.Entity();
        impl->cameraName = removeParentScope(
            scopedName(cameraEntity, _ecm, "::", false), "::");
        gzmsg << "GstCameraPlugin: camera name ["
              << impl->cameraName << "]" << std::endl;
    }

    // complete initialisation deferred from Configure()
    if (!impl->is_initialised)
    {
        if (impl->imageTopic.empty())
        {
            auto maybeTopic = impl->parentSensor.Topic(_ecm);
            if (!maybeTopic.has_value())
            {
                return;
            }
            impl->imageTopic = maybeTopic.value();
        }

        if (impl->enableTopic.empty())
        {
            auto maybeTopic = impl->parentSensor.Topic(_ecm);
            if (!maybeTopic.has_value())
            {
                return;
            }
            impl->enableTopic = maybeTopic.value() + "/enable_streaming";
        }
        gzmsg << "GstCameraPlugin: image topic ["
              << impl->imageTopic << "]" << std::endl;
        gzmsg << "GstCameraPlugin: enable topic ["
              << impl->enableTopic << "]" << std::endl;

        // subscribe to gazebo topics
        impl->node.Subscribe(impl->imageTopic,
            &GstCameraPlugin::Impl::OnImage, impl.get());
        impl->node.Subscribe(impl->enableTopic,
            &GstCameraPlugin::Impl::OnVideoStreamEnable, impl.get());
        if (!impl->recordCommandTopic.empty() &&
            !impl->recordResultTopic.empty())
        {
            impl->recordingResultPublisher =
                impl->node.Advertise<msgs::StringMsg>(
                    impl->recordResultTopic);
            if (!impl->recordingResultPublisher)
            {
                gzerr << "GstCameraPlugin: unable to advertise recording "
                      << "result topic [" << impl->recordResultTopic
                      << "].\n";
            }
            impl->recordingInitialized = impl->node.Subscribe(
                impl->recordCommandTopic,
                &GstCameraPlugin::Impl::HandleRecordingCommand, impl.get());
            if (!impl->recordingInitialized)
            {
                gzerr << "GstCameraPlugin: unable to subscribe to recording "
                      << "command topic [" << impl->recordCommandTopic
                      << "].\n";
            }
        }

        impl->is_initialised = true;
    }

    if (!impl->camera && !impl->cameraName.empty())
    {
        impl->InitializeCamera();
        return;
    }
}

void GstCameraPlugin::Impl::InitializeCamera()
{
    // Wait for render engine to be available.
    if (rendering::loadedEngines().empty())
    {
        return;
    }

    // Get scene.
    if (!scene)
    {
        scene = rendering::sceneFromFirstRenderEngine();
    }

    // Return if scene not ready or no sensors available.
    if (scene == nullptr || !scene->IsInitialized()
        || scene->SensorCount() == 0)
    {
        gzwarn << "GstCameraPlugin: no scene or camera sensors available"
               << std::endl;
        return;
    }

    // Get camera.
    if (!camera)
    {
        auto sensor = scene->SensorByName(cameraName);
        if (!sensor)
        {
            gzerr << "GstCameraPlugin: unable to find sensor ["
                  << cameraName << "]" << std::endl;
            return;
        }

        camera = std::dynamic_pointer_cast<rendering::Camera>(sensor);
        if (!camera)
        {
            gzerr << "GstCameraPlugin: sensor ["
            << cameraName << "] is not a camera" << std::endl;
            return;
        }
    }
}

void GstCameraPlugin::Impl::StartStreaming()
{
    if (!isGstMainLoopActive)
    {
        pthread_create(&threadId, NULL, StartThread, this);
    }
}

void *GstCameraPlugin::Impl::StartThread(void *param)
{
    GstCameraPlugin::Impl *impl = (GstCameraPlugin::Impl *)param;
    impl->StartGstThread();
    return nullptr;
}

void GstCameraPlugin::Impl::StartGstThread()
{
    gst_init(nullptr, nullptr);

    gst_loop = g_main_loop_new(nullptr, FALSE);
    if (!gst_loop)
    {
        gzerr << "GstCameraPlugin: failed to create GStreamer main loop"
              << std::endl;
        return;
    }

    GstElement *pipeline = gst_pipeline_new(nullptr);
    if (!pipeline)
    {
        gzerr << "GstCameraPlugin: GStreamer pipeline failed" << std::endl;
        return;
    }

    source = gst_element_factory_make("appsrc", nullptr);
    if (useRtmpPipeline)
    {
        CreateRtmpPipeline(pipeline);
    }
    else if (useBasicPipeline)
    {
        CreateGenericPipeline(pipeline);
    }
    else
    {
        CreateMpeg2tsPipeline(pipeline);
    }

    // Configure source element
    g_object_set(G_OBJECT(source), "caps",
        gst_caps_new_simple("video/x-raw",
                            "format", G_TYPE_STRING, "I420",
                            "width", G_TYPE_INT, width,
                            "height", G_TYPE_INT, height,
                            "framerate", GST_TYPE_FRACTION,
                            this->rate, 1, nullptr),
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "stream-type", GST_APP_STREAM_TYPE_STREAM,
                 "format", GST_FORMAT_TIME, nullptr);

    gst_object_ref(source);

    // Start
    auto ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret != GST_STATE_CHANGE_SUCCESS)
    {
        gzmsg << "GstCameraPlugin: GStreamer element set state returned: "
              << ret << std::endl;
    }

    // this call blocks until the main_loop is killed
    gzmsg << "GstCameraPlugin: starting GStreamer main loop" << std::endl;
    isGstMainLoopActive = true;
    g_main_loop_run(gst_loop);
    isGstMainLoopActive = false;
    gzmsg << "GstCameraPlugin: stopping GStreamer main loop" << std::endl;

    // Clean up
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    gst_object_unref(source);
    g_main_loop_unref(gst_loop);
    gst_loop = nullptr;
    source = nullptr;
}

void GstCameraPlugin::Impl::CreateRtmpPipeline(GstElement *pipeline)
{
    gzdbg << "GstCameraPlugin: creating RTMP pipeline" << std::endl;
    GstElement *queue = gst_element_factory_make("queue", nullptr);
    GstElement *converter = gst_element_factory_make("videoconvert", nullptr);
    GstElement *encoder = CreateEncoder();
    GstElement *payloader = gst_element_factory_make("flvmux", nullptr);
    GstElement *sink = gst_element_factory_make("rtmpsink", nullptr);

    g_object_set(G_OBJECT(sink), "location", rtmpLocation.c_str(), nullptr);

    if (!source || !queue || !converter || !encoder || !payloader || !sink)
    {
        gzerr << "GstCameraPlugin: failed to create GStreamer elements"
              << std::endl;
        return;
    }

    // Connect all elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder,
        payloader, sink, nullptr);

    // Link all elements
    if (gst_element_link_many(source, queue, converter, encoder,
        payloader, sink, nullptr) != TRUE)
    {
        gzerr << "GstCameraPlugin: failed to link GStreamer elements"
              << std::endl;
        return;
    }
}

void GstCameraPlugin::Impl::CreateGenericPipeline(GstElement *pipeline)
{
    gzdbg << "GstCameraPlugin: creating generic pipeline" << std::endl;
    GstElement *queue = gst_element_factory_make("queue", nullptr);
    GstElement *converter = gst_element_factory_make("videoconvert", nullptr);
    GstElement *encoder = CreateEncoder();
    GstElement *payloader = gst_element_factory_make("rtph264pay", nullptr);
    GstElement *sink = gst_element_factory_make("udpsink", nullptr);

    g_object_set(G_OBJECT(sink), "host", udpHost.c_str(),
        "port", udpPort, nullptr);

    if (!source || !queue || !converter || !encoder || !payloader || !sink)
    {
        gzerr << "GstCameraPlugin: failed to create GStreamer elements"
              << std::endl;
        return;
    }

    // Connect all elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder,
        payloader, sink, nullptr);

    // Link all elements
    if (gst_element_link_many(source, queue, converter, encoder,
        payloader, sink, nullptr) != TRUE)
    {
        gzerr << "GstCameraPlugin: failed to link GStreamer elements"
              << std::endl;
        return;
    }
}

void GstCameraPlugin::Impl::CreateMpeg2tsPipeline(GstElement *pipeline)
{
    gzdbg << "GstCameraPlugin: creating MPEG2TS pipeline" << std::endl;
    GstElement *queue = gst_element_factory_make("queue", nullptr);
    GstElement *converter = gst_element_factory_make("videoconvert", nullptr);
    GstElement *encoder = CreateEncoder();
    GstElement *h264_parser = gst_element_factory_make("h264parse", nullptr);
    GstElement *payloader = gst_element_factory_make("mpegtsmux", nullptr);
    GstElement *queue_mpeg = gst_element_factory_make("queue", nullptr);
    GstElement *sink = gst_element_factory_make("udpsink", nullptr);

    g_object_set(G_OBJECT(payloader), "alignment", 7, nullptr);
    g_object_set(G_OBJECT(sink), "host", udpHost.c_str(), "port", udpPort,
        "sync", false, nullptr);

    if (!source || !queue || !converter || !encoder || !h264_parser
        || !payloader || !queue_mpeg || !sink)
    {
        gzerr << "GstCameraPlugin: failed to create GStreamer elements"
              << std::endl;
        return;
    }

    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder,
        h264_parser, payloader, queue_mpeg, sink, nullptr);
    if (gst_element_link_many(source, queue, converter, encoder,
        h264_parser, payloader, queue_mpeg, sink, nullptr) != TRUE)
    {
        gzerr << "GstCameraPlugin: failed to link GStreamer elements"
              << std::endl;
        return;
    }
}

GstElement* GstCameraPlugin::Impl::CreateEncoder()
{
    GstElement* encoder{nullptr};
    if (useCuda)
    {
        gzdbg << "Using Cuda" << std::endl;
        encoder = gst_element_factory_make("nvh264enc", nullptr);
        g_object_set(G_OBJECT(encoder), "bitrate", 800, "preset", 1, nullptr);
    }
    else
    {
        encoder = gst_element_factory_make("x264enc", nullptr);
        g_object_set(G_OBJECT(encoder), "bitrate", 800, "speed-preset", 6,
            "tune", 4, "key-int-max", 10, nullptr);
    }
    return encoder;
}

void GstCameraPlugin::Impl::OnImage(const msgs::Image &msg)
{
    width = msg.width();
    height = msg.height();

    bool startedStreamingThisFrame = false;
    if (requestedStartStreaming)
    {
        StartStreaming();
        requestedStartStreaming = false;
        startedStreamingThisFrame = true;
    }

    bool haveRecordPipeline = false;
    {
        std::lock_guard<std::mutex> lock(recordingMutex);
        if (pendingRecordingStart && width > 0 && height > 0)
        {
            const uint32_t requestId = pendingRecordingRequestId;
            pendingRecordingStart = false;
            const bool started = StartRecording(requestId);
            PublishRecordingResult(requestId, started ? "OK" : "ERR",
                                   started ? recordingFilePath : "recording_start_failed");
        }
        haveRecordPipeline = isRecording && recordSource != nullptr;
    }
    const bool haveStreamPipeline =
        isGstMainLoopActive && !startedStreamingThisFrame;
    if (!haveStreamPipeline && !haveRecordPipeline)
        return;

    if (msg.pixel_format_type() != msgs::PixelFormatType::RGB_INT8 ||
        msg.step() < width * 3U ||
        msg.data().size() < static_cast<size_t>(msg.step()) * height)
    {
        gzerr << "GstCameraPlugin: unsupported or incomplete RGB sensor frame.\n";
        return;
    }

    // Alloc buffer
    const gsize size = static_cast<gsize>(width) * height * 3U / 2U;

    // Color Conversion from RGB to YUV
    cv::Mat frame(
        height, width, CV_8UC3,
        const_cast<char *>(msg.data().c_str()), msg.step());
    cv::Mat frameYUV = cv::Mat(height * 3 / 2, width, CV_8UC1);
    cvtColor(frame, frameYUV, cv::COLOR_RGB2YUV_I420);

    if (haveStreamPipeline)
    {
        GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);
        if (!buffer)
        {
            gzerr << "GstCameraPlugin: gst_buffer_new_allocate failed"
                  << std::endl;
        }
        else
        {
            GstMapInfo map;
            if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE))
            {
                gzerr << "GstCameraPlugin: gst_buffer_map failed\n";
                gst_buffer_unref(buffer);
            }
            else
            {
                memcpy(map.data, frameYUV.data, size);
                gst_buffer_unmap(buffer, &map);
                const GstFlowReturn ret =
                    gst_app_src_push_buffer(GST_APP_SRC(this->source), buffer);
                if (ret != GST_FLOW_OK)
                {
                    gzerr << "GstCameraPlugin: gst_app_src_push_buffer failed"
                          << std::endl;
                    if (gst_loop)
                        g_main_loop_quit(gst_loop);
                }
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(recordingMutex);
        if (isRecording && recordSource)
        {
            GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);
            if (!buffer)
            {
                gzerr << "GstCameraPlugin: recording buffer allocation failed.\n";
            }
            else
            {
                GstMapInfo map;
                if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE))
                {
                    gzerr << "GstCameraPlugin: recording buffer map failed.\n";
                    gst_buffer_unref(buffer);
                }
                else
                {
                    memcpy(map.data, frameYUV.data, size);
                    gst_buffer_unmap(buffer, &map);
                    GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(
                        recordingFrameIndex, GST_SECOND, recordingRate);
                    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
                    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(
                        1, GST_SECOND, recordingRate);
                    recordingFrameIndex++;
                    if (gst_app_src_push_buffer(
                            GST_APP_SRC(recordSource), buffer) != GST_FLOW_OK)
                    {
                        gzerr << "GstCameraPlugin: recording appsrc rejected "
                              << "a sensor frame.\n";
                    }
                }
            }
        }
    }
}

bool GstCameraPlugin::Impl::StartRecording(uint32_t requestId)
{
    gst_init(nullptr, nullptr);
    if (isRecording)
        return true;
    if (width == 0 || height == 0 || recordingRate == 0)
        return false;
    if (mkdir(recordingDirectory.c_str(), 0755) != 0 && errno != EEXIST)
    {
        gzerr << "GstCameraPlugin: unable to create recording directory ["
              << recordingDirectory << "].\n";
        return false;
    }

    const auto now = std::chrono::system_clock::now();
    const auto epochMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    const std::time_t epochSeconds =
        std::chrono::system_clock::to_time_t(now);
    std::tm utc {};
    gmtime_r(&epochSeconds, &utc);
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y%m%dT%H%M%S", &utc);
    char filename[96];
    snprintf(filename, sizeof(filename), "video_%06u_%s_%03lldZ.mp4",
             requestId, timestamp,
             static_cast<long long>(epochMs % 1000));
    recordingFilePath = recordingDirectory + "/" + filename;

    GstElement *pipeline = gst_pipeline_new(nullptr);
    GstElement *appsrc = gst_element_factory_make("appsrc", nullptr);
    GstElement *queue = gst_element_factory_make("queue", nullptr);
    GstElement *converter = gst_element_factory_make("videoconvert", nullptr);
    GstElement *encoder = gst_element_factory_make("x264enc", nullptr);
    GstElement *muxer = gst_element_factory_make("mp4mux", nullptr);
    GstElement *sink = gst_element_factory_make("filesink", nullptr);
    if (!pipeline || !appsrc || !queue || !converter || !encoder ||
        !muxer || !sink)
    {
        gzerr << "GstCameraPlugin: required MP4 recording GStreamer element "
              << "is unavailable.\n";
        if (pipeline)
            gst_object_unref(pipeline);
        GstElement *elements[] = {
            appsrc, queue, converter, encoder, muxer, sink};
        for (GstElement *element : elements)
        {
            if (element)
                gst_object_unref(element);
        }
        return false;
    }

    g_object_set(G_OBJECT(appsrc), "caps",
        gst_caps_new_simple("video/x-raw",
                            "format", G_TYPE_STRING, "I420",
                            "width", G_TYPE_INT, width,
                            "height", G_TYPE_INT, height,
                            "framerate", GST_TYPE_FRACTION,
                            recordingRate, 1, nullptr),
                            "is-live", TRUE, "do-timestamp", FALSE,
                 "format", GST_FORMAT_TIME, "block", FALSE, nullptr);
    g_object_set(G_OBJECT(encoder), "bitrate", 4000,
                 "speed-preset", 6, "tune", 4,
                 "key-int-max", recordingRate, nullptr);
    g_object_set(G_OBJECT(sink), "location", recordingFilePath.c_str(),
                 "sync", FALSE, nullptr);
    gst_bin_add_many(GST_BIN(pipeline), appsrc, queue, converter, encoder,
                     muxer, sink, nullptr);
    if (!gst_element_link_many(appsrc, queue, converter, encoder,
                               muxer, sink, nullptr))
    {
        gzerr << "GstCameraPlugin: failed to link MP4 recording pipeline.\n";
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        return false;
    }
    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) ==
        GST_STATE_CHANGE_FAILURE)
    {
        gzerr << "GstCameraPlugin: failed to start MP4 recording pipeline.\n";
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        return false;
    }
    recordPipeline = pipeline;
    recordSource = appsrc;
    recordingFrameIndex = 0;
    isRecording = true;
    gzmsg << "GstCameraPlugin: recording sensor frames to ["
          << recordingFilePath << "].\n";
    return true;
}

bool GstCameraPlugin::Impl::StopRecording()
{
    if (!isRecording)
        return true;

    const GstFlowReturn eosResult =
        gst_app_src_end_of_stream(GST_APP_SRC(recordSource));
    GstBus *bus = gst_element_get_bus(recordPipeline);
    GstMessage *message = bus ? gst_bus_timed_pop_filtered(
        bus, 5 * GST_SECOND,
        static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR))
        : nullptr;
    bool succeeded = eosResult == GST_FLOW_OK && message != nullptr &&
                     GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS;
    if (message && GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR)
    {
        GError *error = nullptr;
        gchar *debug = nullptr;
        gst_message_parse_error(message, &error, &debug);
        gzerr << "GstCameraPlugin: MP4 recording failed: "
              << (error ? error->message : "unknown GStreamer error")
              << std::endl;
        if (error) g_error_free(error);
        g_free(debug);
    }
    if (message)
        gst_message_unref(message);
    if (bus)
        gst_object_unref(bus);
    gst_element_set_state(recordPipeline, GST_STATE_NULL);
    gst_object_unref(recordPipeline);
    recordPipeline = nullptr;
    recordSource = nullptr;
    isRecording = false;
    if (!succeeded)
    {
        gzerr << "GstCameraPlugin: timed out or failed while finalizing "
              << "MP4 recording.\n";
    }
    return succeeded;
}

void GstCameraPlugin::Impl::PublishRecordingResult(
    uint32_t requestId, const char *status, const std::string &filePath)
{
    if (!recordingResultPublisher)
    {
        gzerr << "GstCameraPlugin: recording result publisher is unavailable.\n";
        return;
    }
    msgs::StringMsg result;
    result.set_data(std::to_string(requestId) + "|" + status + "|" + filePath);
    if (!recordingResultPublisher.Publish(result))
    {
        gzerr << "GstCameraPlugin: failed to publish recording result.\n";
    }
}

void GstCameraPlugin::Impl::HandleRecordingCommand(
    const msgs::StringMsg &_msg)
{
    const std::string command = _msg.data();
    const auto first = command.find('|');
    const auto second = command.find('|', first == std::string::npos
        ? first : first + 1);
    if (first == std::string::npos || second != std::string::npos)
    {
        gzerr << "GstCameraPlugin: invalid recording command.\n";
        return;
    }
    uint32_t requestId = 0;
    try
    {
        requestId = static_cast<uint32_t>(
            std::stoul(command.substr(0, first)));
    }
    catch (const std::exception &)
    {
        gzerr << "GstCameraPlugin: invalid recording request ID.\n";
        return;
    }
    const std::string action = command.substr(first + 1);
    std::string resultPath;
    const char *resultStatus = "OK";
    {
        std::lock_guard<std::mutex> lock(recordingMutex);
        if (action == "start")
        {
            if (isRecording)
            {
                resultPath = recordingFilePath;
            }
            else if (width == 0 || height == 0)
            {
                pendingRecordingStart = true;
                pendingRecordingRequestId = requestId;
                return;
            }
            else if (StartRecording(requestId))
            {
                resultPath = recordingFilePath;
            }
            else
            {
                resultStatus = "ERR";
                resultPath = "recording_start_failed";
            }
        }
        else if (action == "stop")
        {
            pendingRecordingStart = false;
            if (!StopRecording())
            {
                resultStatus = "ERR";
                resultPath = recordingFilePath;
            }
            else
            {
                resultPath = recordingFilePath;
            }
        }
        else
        {
            resultStatus = "ERR";
            resultPath = "unknown_recording_action";
        }
    }
    PublishRecordingResult(requestId, resultStatus, resultPath);
}

void GstCameraPlugin::Impl::OnVideoStreamEnable(const msgs::Boolean &msg)
{
    gzmsg << "GstCameraPlugin:: streaming: "
          << (msg.data() ? "started" : "stopped")  << std::endl;
    if (msg.data())
    {
      requestedStartStreaming = true;
    }
    else
    {
      requestedStartStreaming = false;
      StopStreaming();
    }
}

void GstCameraPlugin::Impl::OnRenderTeardown()
{
    {
        std::lock_guard<std::mutex> lock(recordingMutex);
        StopRecording();
    }
    StopStreaming();
    camera.reset();
    scene.reset();
}

void GstCameraPlugin::Impl::StopStreaming()
{
    if (isGstMainLoopActive)
    {
        StopGstThread();

        pthread_join(threadId, NULL);
        isGstMainLoopActive = false;
    }
}

void GstCameraPlugin::Impl::StopGstThread()
{
    if (gst_loop)
    {
        g_main_loop_quit(gst_loop);
    }
}

//////////////////////////////////////////////////

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::GstCameraPlugin,
    gz::sim::System,
    gz::sim::systems::GstCameraPlugin::ISystemConfigure,
    gz::sim::systems::GstCameraPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::GstCameraPlugin,
    "GstCameraPlugin")
