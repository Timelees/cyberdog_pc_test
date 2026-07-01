// ros
#include "pose_estimation.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/camera_subscriber.hpp>
#ifdef APRILTAG_DIRECT_CAMERA_SUB
#include <image_transport/camera_common.hpp>
#endif
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <algorithm>
#include <condition_variable>
#include <optional>
#include <thread>
#include <unordered_map>
#ifdef tf2_ros_NODE_INTERFACE
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

// apriltag
#include "tag_functions.hpp"
#include <apriltag.h>


#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var)
{
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

const static std::unordered_map<std::string, rmw_qos_profile_t> qos_profiles{
    {"default", rmw_qos_profile_default},
    {"sensor_data", rmw_qos_profile_sensor_data},
    {"system_default", rmw_qos_profile_system_default},
};

#ifdef APRILTAG_DIRECT_CAMERA_SUB
// Galactic image_transport appends "/raw" to the base topic, which does not match
// RealSense topics such as ".../image_rect_raw". Subscribe to the image topic directly.
class DirectCameraSubscriber {
public:
    using Callback = std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr&,
                                        const sensor_msgs::msg::CameraInfo::ConstSharedPtr&)>;

    DirectCameraSubscriber(rclcpp::Node* node,
                           const std::string& image_topic,
                           Callback callback,
                           rmw_qos_profile_t qos)
        : callback_(std::move(callback))
    {
        const std::string expanded =
            rclcpp::expand_topic_or_service_name(image_topic, node->get_name(), node->get_namespace());
        const std::string info_topic = image_transport::getCameraInfoTopic(expanded);
        const auto image_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos);

        info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic, image_qos,
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) { latest_info_ = info; });

        image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
            expanded, image_qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr image) {
                const sensor_msgs::msg::CameraInfo::ConstSharedPtr info = latest_info_;
                if(info) {
                    callback_(image, info);
                }
            });
    }

private:
    Callback callback_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;
    rclcpp::SubscriptionBase::SharedPtr image_sub_;
    rclcpp::SubscriptionBase::SharedPtr info_sub_;
};
#endif

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions& options);

    ~AprilTagNode() override;

private:
    struct LatchedTagPose {
        bool latched{false};
        std::string parent_frame_id;
        std::string child_frame_id;
        geometry_msgs::msg::Transform transform;
    };

    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    apriltag_family_t* tf{nullptr};
    apriltag_detector_t* const td;

    // parameter
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;

    std::function<void(apriltag_family_t*)> tf_destructor;

#ifdef APRILTAG_DIRECT_CAMERA_SUB
    std::unique_ptr<DirectCameraSubscriber> sub_cam;
#else
    const image_transport::CameraSubscriber sub_cam;
#endif
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

    pose_estimation_f estimate_pose = nullptr;

    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;
    std::atomic<bool> shutdown_{false};
    std::thread processing_thread_;
    bool latch_first_tag_pose_{true};
    bool publish_latched_when_lost_{true};
    bool use_first_pose_as_global_{true};
    mutable std::mutex latched_pose_mutex_;
    std::unordered_map<int, LatchedTagPose> latched_tag_poses_;

    void processingLoop();
    void processImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);
    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);
    void latchTagPose(int tag_id,
                      const std::string& parent_frame_id,
                      const std::string& child_frame_id,
                      const geometry_msgs::msg::Transform& transform);
    void publishLatchedStaticTransform(const LatchedTagPose& latched);
    std::optional<geometry_msgs::msg::TransformStamped> makeTagTransform(
        const std_msgs::msg::Header& header,
        int tag_id,
        const geometry_msgs::msg::Transform& transform,
        const std::string& child_frame_id) const;
    void appendLatchedTagTransforms(const std_msgs::msg::Header& header,
                                  std::vector<geometry_msgs::msg::TransformStamped>& tfs);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);
};

#ifndef APRILTAG_STANDALONE_MAIN
RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)
#endif


AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
#ifdef APRILTAG_DIRECT_CAMERA_SUB
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
#else
    // topics
    sub_cam{
#ifdef image_transport_NODE_INTERFACE
        image_transport::RequiredInterfaces{*this},
#else
        this,
#endif
        this->get_node_topics_interface()->resolve_topic_name("image_rect"),
        std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
        declare_parameter("image_transport", "raw", descr({}, true)),
#ifdef image_transport_QoS
        rclcpp::QoS{rclcpp::QoSInitialization::from_rmw(
            qos_profiles.at(declare_parameter("qos_profile", "sensor_data", descr("qos profile to use. 'default', 'sensor_data' or 'system_default'", true)))
        )}
#else
        qos_profiles.at(declare_parameter("qos_profile", "sensor_data", descr("qos profile to use. 'default', 'sensor_data' or 'system_default'", true)))
#endif
    },
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
#endif
    tf_broadcaster(
#ifdef tf2_ros_NODE_INTERFACE
        tf2_ros::TransformBroadcaster::RequiredInterfaces { *this }
#else
        this
#endif
    ),
    static_tf_broadcaster(
#ifdef tf2_ros_NODE_INTERFACE
        tf2_ros::StaticTransformBroadcaster::RequiredInterfaces { *this }
#else
        this
#endif
    )
{
    // read-only parameters
    const std::string tag_family = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));

    // get tag names, IDs and sizes
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));

    // get method for estimating tag pose
    const std::string pose_estimation_method =
        declare_parameter("pose_estimation_method", "pnp",
                          descr("pose estimation method: \"pnp\" (more accurate) or \"homography\" (faster), "
                                "set to \"\" (empty) to disable pose estimation",
                                true));

    if(!pose_estimation_method.empty()) {
        if(pose_estimation_methods.count(pose_estimation_method)) {
            estimate_pose = pose_estimation_methods.at(pose_estimation_method);
        }
        else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown pose estimation method '" << pose_estimation_method << "'.");
        }
    }

    // detector parameters in "detector" namespace
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));

    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));
    latch_first_tag_pose_ = declare_parameter(
        "latch_first_tag_pose", true,
        descr("latch the first successful tag pose as the global reference", true));
    publish_latched_when_lost_ = declare_parameter(
        "publish_latched_when_lost", true,
        descr("keep publishing latched tag TF when the tag is no longer visible", true));
    use_first_pose_as_global_ = declare_parameter(
        "use_first_pose_as_global", true,
        descr("after latch, always publish the first tag pose instead of live updates", true));

    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" + std::to_string(frames.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    if(!sizes.empty()) {
        // use tag specific size
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    if(tag_fun.count(tag_family)) {
        tf = tag_fun.at(tag_family).first();
        tf_destructor = tag_fun.at(tag_family).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + tag_family);
    }

#ifdef APRILTAG_DIRECT_CAMERA_SUB
    const std::string qos_profile =
        declare_parameter<std::string>("qos_profile", "default", descr("qos profile to use. 'default', 'sensor_data' or 'system_default'", true));
    DirectCameraSubscriber::Callback camera_cb =
        std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2);
    sub_cam = std::make_unique<DirectCameraSubscriber>(
        this,
        get_node_topics_interface()->resolve_topic_name("image_rect"),
        camera_cb,
        qos_profiles.at(qos_profile));
#endif

    processing_thread_ = std::thread(&AprilTagNode::processingLoop, this);
}

AprilTagNode::~AprilTagNode()
{
    shutdown_ = true;
    frame_cv_.notify_all();
    if(processing_thread_.joinable()) {
        processing_thread_.join();
    }

    apriltag_detector_destroy(td);
    if(tf != nullptr) {
        tf_destructor(tf);
    }
}

void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_image_ = msg_img;
        latest_info_ = msg_ci;
    }
    frame_cv_.notify_one();
}

void AprilTagNode::processingLoop()
{
    while(!shutdown_) {
        sensor_msgs::msg::Image::ConstSharedPtr msg_img;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr msg_ci;

        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_cv_.wait(lock, [this] { return shutdown_ || latest_image_ != nullptr; });
            if(shutdown_) {
                break;
            }
            msg_img = latest_image_;
            msg_ci = latest_info_;
            latest_image_.reset();
            latest_info_.reset();
        }

        processImage(msg_img, msg_ci);
    }
}

void AprilTagNode::latchTagPose(int tag_id,
                              const std::string& parent_frame_id,
                              const std::string& child_frame_id,
                              const geometry_msgs::msg::Transform& transform)
{
    std::lock_guard<std::mutex> lock(latched_pose_mutex_);
    auto& latched = latched_tag_poses_[tag_id];
    if(latched.latched) {
        return;
    }

    latched.latched = true;
    latched.parent_frame_id = parent_frame_id;
    latched.child_frame_id = child_frame_id;
    latched.transform = transform;
    RCLCPP_INFO(
        get_logger(),
        "latched global tag pose for id=%d: %s -> %s, translation=[%.3f, %.3f, %.3f]",
        tag_id,
        latched.parent_frame_id.c_str(),
        latched.child_frame_id.c_str(),
        latched.transform.translation.x,
        latched.transform.translation.y,
        latched.transform.translation.z);

  // Latched tag pose is fixed; publish once on /tf_static instead of flooding /tf
  // at camera rate, which can congest DDS and block other TF (e.g. base_link tree).
  if(use_first_pose_as_global_) {
      publishLatchedStaticTransform(latched);
  }
}

void AprilTagNode::publishLatchedStaticTransform(const LatchedTagPose& latched)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = latched.parent_frame_id;
    tf.child_frame_id = latched.child_frame_id;
    tf.transform = latched.transform;
    static_tf_broadcaster.sendTransform(tf);
    RCLCPP_INFO(
        get_logger(),
        "published latched tag TF on /tf_static: %s -> %s",
        latched.parent_frame_id.c_str(),
        latched.child_frame_id.c_str());
}

std::optional<geometry_msgs::msg::TransformStamped> AprilTagNode::makeTagTransform(
    const std_msgs::msg::Header& header,
    int tag_id,
    const geometry_msgs::msg::Transform& transform,
    const std::string& child_frame_id) const
{
    std::lock_guard<std::mutex> lock(latched_pose_mutex_);
    const auto latched_it = latched_tag_poses_.find(tag_id);
    const bool has_latched = latch_first_tag_pose_ &&
                             latched_it != latched_tag_poses_.end() &&
                             latched_it->second.latched;

    geometry_msgs::msg::TransformStamped tf;
    tf.header = header;
    tf.child_frame_id = child_frame_id;
    tf.transform = transform;

    if(has_latched && use_first_pose_as_global_) {
        tf.header.frame_id = latched_it->second.parent_frame_id;
        tf.child_frame_id = latched_it->second.child_frame_id;
        tf.transform = latched_it->second.transform;
    } else {
        tf.header.frame_id = header.frame_id;
    }

    return tf;
}

void AprilTagNode::appendLatchedTagTransforms(
    const std_msgs::msg::Header& header,
    std::vector<geometry_msgs::msg::TransformStamped>& tfs)
{
    if(!latch_first_tag_pose_ || !publish_latched_when_lost_) {
        return;
    }

    std::lock_guard<std::mutex> lock(latched_pose_mutex_);
    for(const auto& [tag_id, latched] : latched_tag_poses_) {
        if(!latched.latched) {
            continue;
        }

        const auto already_published = std::any_of(
            tfs.begin(), tfs.end(),
            [&latched](const geometry_msgs::msg::TransformStamped& tf) {
                return tf.header.frame_id == latched.parent_frame_id &&
                       tf.child_frame_id == latched.child_frame_id;
            });
        if(already_published) {
            continue;
        }

        geometry_msgs::msg::TransformStamped tf;
        tf.header = header;
        tf.header.frame_id = latched.parent_frame_id;
        tf.child_frame_id = latched.child_frame_id;
        tf.transform = latched.transform;
        tfs.push_back(tf);

        RCLCPP_DEBUG(
            get_logger(),
            "republish latched global tag pose for id=%d while tag is not visible",
            tag_id);
    }
}

void AprilTagNode::processImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                                const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    // camera intrinsics for rectified images
    const std::array<double, 4> intrinsics = {msg_ci->p[0], msg_ci->p[5], msg_ci->p[2], msg_ci->p[6]};

    // check for valid intrinsics
    const bool calibrated = msg_ci->width && msg_ci->height &&
                            intrinsics[0] && intrinsics[1] && intrinsics[2] && intrinsics[3];

    if(estimate_pose != nullptr && !calibrated) {
        RCLCPP_WARN_STREAM(get_logger(), "The camera is not calibrated! Set 'pose_estimation_method' to \"\" (empty) to disable pose estimation and this warning.");
    }

    // convert to 8bit monochrome image
    const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg_img, "mono8");
    const cv::Mat& img_uint8 = cv_ptr->image;

    image_u8_t im{img_uint8.cols, img_uint8.rows, static_cast<int>(img_uint8.step[0]), const_cast<uint8_t*>(img_uint8.data)};

    // detect tags
    mutex.lock();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    mutex.unlock();

    if(profile)
        timeprofile_display(td->tp);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;

    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    for(int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        RCLCPP_DEBUG(get_logger(),
                     "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                     i, det->family->nbits, det->family->h, det->id,
                     det->hamming, det->decision_margin);

        // ignore untracked tags
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming > max_hamming) { continue; }

        // detection
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre.x = det->c[0];
        msg_detection.centre.y = det->c[1];
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
        if(det->H != nullptr) {
            std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
        }
        msg_detections.detections.push_back(msg_detection);

        // 3D orientation and position
        if(estimate_pose != nullptr && calibrated) {
            // set child frame name by generic tag name or configured tag name
            const std::string child_frame_id = tag_frames.count(det->id)
                ? tag_frames.at(det->id)
                : std::string(det->family->name) + ":" + std::to_string(det->id);
            const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            const geometry_msgs::msg::Transform transform = estimate_pose(det, intrinsics, size);

            if(latch_first_tag_pose_) {
                latchTagPose(det->id, msg_img->header.frame_id, child_frame_id, transform);
            }

            const auto tf = makeTagTransform(msg_img->header, det->id, transform, child_frame_id);
            if(tf.has_value()) {
                tfs.push_back(tf.value());
            }
        }
    }

    const bool use_static_latched_tf = latch_first_tag_pose_ && use_first_pose_as_global_;
    if(!use_static_latched_tf) {
        appendLatchedTagTransforms(msg_img->header, tfs);
    }

    pub_detections->publish(msg_detections);

    // When latched pose is published on /tf_static, skip high-rate /tf output.
    if(estimate_pose != nullptr && !tfs.empty() && !use_static_latched_tf) {
        tf_broadcaster.sendTransform(tfs);
    }

    apriltag_detections_destroy(detections);
}

rcl_interfaces::msg::SetParametersResult
AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    mutex.lock();

    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);

        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        IF("detector.refine", td->refine_edges)
        IF("detector.sharpening", td->decode_sharpening)
        IF("detector.debug", td->debug)
        IF("max_hamming", max_hamming)
        IF("profile", profile)
    }

    mutex.unlock();

    result.successful = true;

    return result;
}

#ifdef APRILTAG_STANDALONE_MAIN
int main(int argc, char** argv)
{
    int exit_code = 0;
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<AprilTagNode>(rclcpp::NodeOptions{}));
    }
    catch(const std::exception& exception) {
        RCLCPP_ERROR(rclcpp::get_logger("apriltag_node"), "fatal error: %s", exception.what());
        exit_code = 1;
    }

    if(rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return exit_code;
}
#endif
