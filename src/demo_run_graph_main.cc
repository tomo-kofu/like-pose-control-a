// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// An example of sending OpenCV webcam frames into a MediaPipe graph.
#include <cstdlib>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
////////////////////////////////////////////////////////////////////////////////
// modifyer add start
////////////////////////////////////////////////////////////////////////////////
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/rect.pb.h"
#include "mediapipe/framework/formats/detection.pb.h"
////////////////////////////////////////////////////////////////////////////////
// modifyer add end
////////////////////////////////////////////////////////////////////////////////
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"

////////////////////////////////////////////////////////////////////////////////
// modifyer add start
////////////////////////////////////////////////////////////////////////////////
#include <Windows.h>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <vector>
#pragma comment(lib, "winmm.lib") // 
#include <chrono>
#include <ctime>
#include "mediapipe/framework/output_stream_poller.h"
#include <math.h>
#include <conio.h>
#include "nlohmann/json.hpp"
#include <fstream>
#include <Shellapi.h>
#pragma comment(lib,"shell32.lib") // need Shellapi.h
#include <signal.h>
#include <wincrypt.h>
#include <filesystem>
////////////////////////////////////////////////////////////////////////////////
// modifyer add end
////////////////////////////////////////////////////////////////////////////////

constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream[] = "output_video";
////////////////////////////////////////////////////////////////////////////////
// modifyer add start
////////////////////////////////////////////////////////////////////////////////
// pose
constexpr char kOutputPoseLandmarks[] = "pose_landmarks";
constexpr char kOutputPoseWorldLandmarks[] = "pose_world_landmarks";
constexpr char kOutputPoseDetection[] = "pose_detection";
constexpr char kOutputROILandmarks[] = "roi_from_landmarks";
// hand
constexpr char kOutputHandLandmarks[] = "hand_landmarks";
constexpr char kOutputHandWorldLandmarks[] = "hand_world_landmarks";
// face_mesh
constexpr char kOutputFaceMeshLandmarks[] = "face_mesh_landmarks";

// TODO #ifdefでビルドフラグで分ける
#define DETECT_MODE_POSE        1
//#define DETECT_MODE_HAND        1
//#define DETECT_MODE_FACE_MESH   1
////////////////////////////////////////////////////////////////////////////////
// modifyer add end
////////////////////////////////////////////////////////////////////////////////
//constexpr char kWindowName[] = "MediaPipe";
#ifdef DETECT_MODE_POSE
constexpr char kWindowName[] = "Like Pose Command";
#elif DETECT_MODE_HAND
constexpr char kWindowName[] = "Like Hand Command";
#elif DETECT_MODE_FACE_MESH
constexpr char kWindowName[] = "Like Face Command";
#endif // DETECT_MODE_FACE_MESH

ABSL_FLAG(std::string, calculator_graph_config_file, "",
          "Name of file containing text format CalculatorGraphConfig proto.");
ABSL_FLAG(std::string, input_camera_id, "",
          "camera id. "
  "If not provided, attempt to use one(1).");
ABSL_FLAG(std::string, input_device_count, "",
          "count of device. "
          "If not provided, attempt to use one(1).");
ABSL_FLAG(std::string, input_user_count, "",
          "count of user. "
          "If not provided, attempt to use one(1).");
ABSL_FLAG(std::string, input_proc_fps, "",
          "fps of proc. "
          "If not provided, attempt to default fps(5.0).");
ABSL_FLAG(std::string, input_config_path, "",
          "config file path. "
          "If not provided, attempt to no use config file.");
ABSL_FLAG(std::string, input_display_mosaic_scale, "",
          "display mosaic scale. "
          "If not provided,attempt to use one(1).");
ABSL_FLAG(std::string, is_auto_start, "",
          "is auto start. "
          "If not provided,attempt to use false.");

////////////////////////////////////////////////////////////////////////////////
// modifyer add start
////////////////////////////////////////////////////////////////////////////////


////////////
// common //
////////////
int getNumberFromKeyCode(int key_code);
std::time_t getEpocTimByTime();
uint64_t getEpocTimeByChrono();

const int WINDOW_TOP_BAR_SIZE = 30;


// config
#define CONFIG_KEY_PC_INFO           "pc_info"
#define CONFIG_KEY_ACTION_INFO       "action_info"
#define CONFIG_KEY_TEMPLATE_CONFIG   "template_config"

//   pc info
#define CONFIG_KEY_MAIN_DISPLAY_WIDTH                "main_display_width"
#define CONFIG_KEY_MAIN_DISPLAY_HEIGHT               "main_display_height"
#define CONFIG_KEY_NO_MOUSE_ABSOLUTE_RATE            "no_mouse_absolute_rate"
#define CONFIG_KEY_CAMERA_ID                         "camera_id"
#define CONFIG_KEY_CAMERA_DISPLAY_WIDTH              "camera_display_width"
#define CONFIG_KEY_CAMERA_DISPLAY_HEIGHT             "camera_display_height"
#define CONFIG_KEY_CAMERA_FPS                        "camera_fps"
#define CONFIG_KEY_CAMERA_PROC_FPS                   "camera_proc_fps"
#define CONFIG_KEY_CAMERA_LOW_PROC_FPS               "camera_low_proc_fps"

//   action info
#define CONFIG_KEY_IS_AUTO_START                            "is_auto_start"
#define CONFIG_KEY_FOCUS_DISPLAY_LEFT                       "focus_display_left"
#define CONFIG_KEY_FOCUS_DISPLAY_TOP                        "focus_display_top"
#define CONFIG_KEY_FOCUS_DISPLAY_RIGHT                      "focus_display_right"
#define CONFIG_KEY_FOCUS_DISPLAY_BOTTOM                     "focus_display_bottom"
#define CONFIG_KEY_IS_DISPLAY_MIRROR                        "is_display_mirror"
#define CONFIG_KEY_DISPLAY_MOSAIC_SCALE                     "display_mosaic_scale"
#define CONFIG_KEY_DISPLAY_FACE_MOSAIC_SCALE                "display_face_mosaic_scale"
#define CONFIG_KEY_DISPLAY_FACE_MOSAIC_WIDTH                "display_face_mosaic_width"
#define CONFIG_KEY_DISPLAY_FACE_MOSAIC_HEIGHT               "display_face_mosaic_height"
#define CONFIG_KEY_FRAME_DIFF_FRAME_MEMORY_MSEC             "frame_diff_frame_memory_msec"
#define CONFIG_KEY_GAME_START_WAIT_MSEC                     "game_start_wait_msec"
#define CONFIG_KEY_KEY_GROUP                                "key_group"
#define CONFIG_KEY_PERSON_SIZE_SCALE_COORDINATE             "person_size_scale_coordinate"
#define CONFIG_KEY_PERSON_SIZE_SCALE_SUM                    "person_size_scale_sum"
#define CONFIG_KEY_PERSON_SIZE_SCALE_FRAME_DIFF             "person_size_scale_frame_diff"
#define CONFIG_KEY_PERSON_SIZE_SCALE_DIST_COMPARE           "person_size_scale_dist_compare"
#define CONFIG_KEY_PERSON_SIZE_SCALE_MARK_COMPARE           "person_size_scale_mark_compare"
#define CONFIG_KEY_PERSON_SIZE_SCALE_VECTOR_DIST_COMPARE    "person_size_scale_vector_dist_compare"
#define CONFIG_KEY_IS_ENABLE_CHANGE_X_FRONT_REVERSE         "is_enable_change_x_front_reverse"
#define CONFIG_KEY_X_FRONT_REVERSE_CHECK_MSEC               "x_front_reverse_check_msec"
#define CONFIG_KEY_EYES_DIST                                "eyes_dist"
#define CONFIG_KEY_LOGOUT_COORDINATE_TYPES                  "logout_coordinate_types"
#define CONFIG_KEY_LOGOUT_COORDINATE_NUMBERS                "logout_coordinate_numbers"

#define CONFIG_KEY_COORDINATE_V1             "coordinate_v1"
#define CONFIG_KEY_COORDINATE_V2             "coordinate_v2"
#define CONFIG_KEY_COORDINATE_V3             "coordinate_v3"
#define CONFIG_KEY_SUM_V1                    "sum_v1"
#define CONFIG_KEY_FRAME_DIFF_V1             "frame_diff_v1"
#define CONFIG_KEY_DIST_COMPARE_V1           "dist_compare_v1"
#define CONFIG_KEY_MIX_V1                    "mix_v1"
#define CONFIG_KEY_MARK_COMPARE_V1           "mark_compare_v1"
#define CONFIG_KEY_CONTINUE_V1               "continue_v1"
#define CONFIG_KEY_VECTOR_DIST_COMPARE_V1    "vector_dist_compare_v1"

#define CONFIG_KEY_ACTION_CODES            "action_codes"
#define CONFIG_KEY_NOT_ACTION_CODES        "not_action_codes"
#define CONFIG_KEY_KEY_POINTS              "key_points"
#define CONFIG_KEY_THRESHOLD_CONDITIONS    "threshold_conditions"
#define CONFIG_KEY_DISPLAY_COLOR           "display_color"
#define CONFIG_KEY_ACTION_FLAG             "action_flags"
#define CONFIG_KEY_CHECK_VECTOR            "check_vector"
#define CONFIG_KEY_CHECK_VECTOR_RATE       "check_vector_rate"
#define CONFIG_KEY_ACTION_CONDITIONS       "action_conditions"
#define CONFIG_KEY_VISIBILITY_THRESHOLD    "visibility_threshold"
#define CONFIG_KEY_NEUTRAL_KEY_GROUP_ID    "neutral_key_group_id"
#define CONFIG_KEY_CONDITIONS_TIMEOUT_MSEC "conditions_timeout_msec"

#define CONFIG_KEY_THRESHOLD  "threshold"
#define CONFIG_KEY_CONDITION  "condition"

#define CONFIG_KEY_COORDINATE_V2_KEY_POINTS_MAX_SIZE                  3
#define CONFIG_KEY_COORDINATE_V3_KEY_POINTS_MULTIPLE_SIZE             5
#define CONFIG_KEY_SUM_V1_KEY_POINTS_MULTIPLE_SIZE                    2
#define CONFIG_KEY_DIST_COMPARE_V1_KEY_POINTS_MULTIPLE_SIZE           5
#define CONFIG_KEY_MARK_COMPARE_V1_KEY_POINTS_MULTIPLE_SIZE           7
#define CONFIG_KEY_VECTOR_DIST_COMPARE_V1_KEY_POINTS_MULTIPLE_SIZE    7

int enable_TemplateConfig_index = -1;
int target_TemplateConfig_index = 0;
std::string config_path = "config.json";

void clearConfigV1_TemplateConfig();
void changeConfigV1_TemplateConfig(std::string& file_path, int index);

void loadConfigV1(std::string& file_path);

void loadConfigV1_All(nlohmann::json& json_data);
void loadConfigV1_PCInfo(nlohmann::json& json_data);
void loadConfigV1_ActionInfo(nlohmann::json& json_data);
void loadConfigV1_TemplateConfigs(nlohmann::json& json_data, int target_index);
void loadConfigV1_TemplateConfig(nlohmann::json& json_data, int index);

void loadConfigV1_CoordinateV2(nlohmann::json& json_data, int index, int child_index);
std::vector<std::vector<int> > loadConfigV1_ActionCodesList(nlohmann::json& json_data);
int toIntFromStringHex(std::string str_element);
std::vector<int> loadConfigV1_ActionCodes(nlohmann::json& json_data);
std::vector<std::vector<int> > loadConfigV1_KeyPointsList(nlohmann::json& json_data);
std::vector<int> loadConfigV1_KeyPoints(nlohmann::json& json_data);
std::vector<std::pair<float, int> > loadConfigV1_ThresholdConditionList(nlohmann::json& json_data);
std::pair<float, int> loadConfigV1_ThresholdCondition(nlohmann::json& json_data);
std::vector<int> loadConfigV1_ActionFlagList(nlohmann::json& json_data);
int loadConfigV1_ActionFlag(nlohmann::json& json_data);
int loadConfigV1_DisplayColor(nlohmann::json& json_data);

void loadConfigV1_CoordinateV3(nlohmann::json& json_data, int index, int child_index);
std::vector<std::vector<int> > loadConfigV1_KeyPointsListV3(nlohmann::json& json_data);
std::vector<int> loadConfigV1_KeyPointsV3(nlohmann::json& json_data);

std::vector<std::vector<int> > loadConfigV1_KeyPointsListVN(nlohmann::json& json_data, int array_size);
std::vector<int> loadConfigV1_KeyPointsVN(nlohmann::json& json_data, int array_size);

void loadConfigV1_SumV1(nlohmann::json& json_data, int index, int child_index);
std::vector<int> loadConfigV1_KeyPointsSumV1(nlohmann::json& json_data);
std::vector<std::vector<int> > loadConfigV1_KeyPointsListSumV1(nlohmann::json& json_data);

void loadConfigV1_FrameDiffV1(nlohmann::json& json_data, int index, int child_index);
std::vector<std::vector<int> > loadConfigV1_KeyPointsListFrameDiffV1(nlohmann::json& json_data);
std::vector<int> loadConfigV1_KeyPointsFrameDiffV1(nlohmann::json& json_data);
std::vector<std::vector<std::pair<float, int> > > loadConfigV1_ThresholdConditionListList(nlohmann::json& json_data);

void loadConfigV1_DistCompareV1(nlohmann::json& json_data, int index, int child_index);

void loadConfigV1_MixV1(nlohmann::json& json_data, int index, int child_index);

std::vector<int> loadConfigV1_KeyPointsV4(nlohmann::json& json_data);
void loadConfigV1_MarkCompareV1(nlohmann::json& json_data, int index, int child_index);

std::vector<int> loadConfigV1_VecIntV1(nlohmann::json& json_data);
std::vector<std::vector<int> > loadConfigV1_VecVecIntV1(nlohmann::json& json_data);
void loadConfigV1_ContinueV1(nlohmann::json& json_data, int index, int child_index);

std::vector<int> loadConfigV1_KeyPointsV5(nlohmann::json& json_data);
void loadConfigV1_VectorDistCompareV1(nlohmann::json& json_data, int index, int child_index);

// dump
void dumpConfigV1();
void dumpConfigV1_PCInfo(std::string indent_str);
void dumpConfigV1_ActionInfo(std::string indent_str);
void dumpConfigV1_CoordinateV2(int index, std::string indent_str);
void dumpConfigV1_CoordinateV3(int index, std::string indent_str);
void dumpConfigV1_SumV1(int index, std::string indent_str);
void dumpConfigV1_FrameDiffV1(int index, std::string indent_str);
void dumpConfigV1_DistCompareV1(int index, std::string indent_str);
void dumpConfigV1_MixV1(int index, std::string indent_str);
void dumpConfigV1_MarkCompareV1(int index, std::string indent_str);
void dumpConfigV1_ContinueV1(int index, std::string indent_str);
void dumpConfigV1_VectorDistCompareV1(int index, std::string indent_str);

void dumpConfigV1_ActionCodesList(std::vector<std::vector<int> > action_codes_list, std::string indent_str);
void dumpConfigV1_NotActionCodesList(std::vector<std::vector<int> > not_action_codes_list, std::string indent_str);
std::string getDumpHexFromUInt64(uint64_t num);
std::string getDumpHexFromInt(int num);
void dumpConfigV1_ActionCodes(std::vector<int> action_codes, std::string indent_str);
void dumpConfigV1_NotActionCodes(std::vector<int> not_action_codes, std::string indent_str);
void dumpConfigV1_KeyPointsList(std::vector<std::vector<int> > key_points_list, std::string indent_str);
void dumpConfigV1_KeyPoints(std::vector<int> key_points, std::string indent_str);
void dumpConfigV1_ThresholdConditions(std::vector<std::pair<float, int> > threshold_conditions, std::string indent_str);
void dumpConfigV1_ThresholdCondition(std::pair<float, int> threshold_condition, std::string indent_str);
std::string getDumpHexFromColorScalar(cv::Scalar scalar);
void dumpConfigV1_DisplayColors(std::vector<cv::Scalar> display_colors, std::string indent_str);
void dumpConfigV1_ActionFlags(std::vector<int> action_flags, std::string indent_str);
void dumpConfigV1_ThresholdConditionsList(std::vector<std::vector<std::pair<float, int> > > threshold_conditions_list, std::string indent_str);
void dumpConfigV1_ActionConditionsList(std::vector<std::vector<int> > action_conditions_list, std::string indent_str);
void dumpConfigV1_ActionConditions(std::vector<int> action_conditions, std::string indent_str);
void dumpConfigV1_VecInt(std::vector<int> key_points, std::string indent_str);
void dumpConfigV1_VecVecInt(std::vector<std::vector<int> > key_points_list, std::string indent_str);

///////////
// start //
///////////
bool is_auto_start = true;

std::string usage_context = "please input below charactor,\r\n" \
                            "  s, p, d, q\r\n" \
                            "    each charactor mean below,\r\n" \
                            "      s: start.\r\n" \
                            "      p: pause.\r\n" \
                            "      d: change debug display on or off(default).\r\n" \
                            "      q: quit program.\r\n";

////////////
// camera //
////////////
int camera_id = 0;
int camera_width = 640;
int camera_height = 480;
int camera_fps = 30;
float camera_proc_fps = 5.0;
int camera_proc_fps_wait_msec = (int)(1000.0 / camera_proc_fps);
float camera_low_proc_fps = 1.0;
cv::Mat last_frame_mat_0;
cv::Mat last_frame_mat_1;
std::queue<mediapipe::NormalizedLandmarkList> landmarks_2d_queue;

/////////////
// display //
/////////////
int display_mosaic_scale = 1;
int display_face_mosaic_scale = 1;
float display_face_mosaic_width = 3.5;
float display_face_mosaic_height = 3.5;
float last_proc_fps = 0.0;
int proc_fps_average_size = 10;
std::vector<uint64_t> proc_msec_list;
uint64_t last_input_timestamp = 0;
float last_done_fps = 0.0;
std::vector<uint64_t> done_msec_list;

float calcProcFPS();


////////////////////
// mediapipe pose //
////////////////////
auto getImageMP(cv::Mat& input_frame_cv);
absl::Status RunMPPGraphsAtCameraDevice(std::vector<mediapipe::CalculatorGraph*>& graphs);
void threadDoMPPGraph(mediapipe::OutputStreamPoller* poller_pose_landmarks, mediapipe::OutputStreamPoller* poller_pose_world_landmarks);
void threadDoKeyCode();
void threadDoConsoleKeyCode();


#ifdef DETECT_MODE_POSE
// from mediapipe/graphs/pose_tracking/pose_tracking_cpu.pbtxt
std::string default_config_context = "input_stream: \"input_video\"\r\n" \
"output_stream: \"pose_landmarks\"\r\n" \
"output_stream: \"pose_world_landmarks\"\r\n" \
"node {\r\n" \
"  calculator: \"ConstantSidePacketCalculator\"\r\n" \
"  output_side_packet: \"PACKET:enable_segmentation\"\r\n" \
"  node_options: {\r\n" \
"    [type.googleapis.com/mediapipe.ConstantSidePacketCalculatorOptions]: {\r\n" \
"      packet { bool_value: false }\r\n" \
"    }\r\n" \
"  }\r\n" \
"}\r\n" \
"node {\r\n" \
"  calculator: \"FlowLimiterCalculator\"\r\n" \
"  input_stream: \"input_video\"\r\n" \
"  input_stream: \"FINISHED:output_video\"\r\n" \
"  input_stream_info: {\r\n" \
"    tag_index: \"FINISHED\"\r\n" \
"    back_edge: true\r\n" \
"  }\r\n" \
"  output_stream: \"throttled_input_video\"\r\n" \
"}\r\n" \
"node {\r\n" \
"  calculator: \"PoseLandmarkCpu\"\r\n" \
"  input_side_packet: \"ENABLE_SEGMENTATION:enable_segmentation\"\r\n" \
"  input_stream: \"IMAGE:throttled_input_video\"\r\n" \
"  output_stream: \"LANDMARKS:pose_landmarks\"\r\n" \
"  output_stream: \"WORLD_LANDMARKS:pose_world_landmarks\"\r\n" \
"  output_stream: \"SEGMENTATION_MASK:segmentation_mask\"\r\n" \
"  output_stream: \"DETECTION:pose_detection\"\r\n" \
"  output_stream: \"ROI_FROM_LANDMARKS:roi_from_landmarks\"\r\n" \
"}\r\n" \
"node {\r\n" \
"  calculator: \"PoseRendererCpu\"\r\n" \
"  input_stream: \"IMAGE:throttled_input_video\"\r\n" \
"  input_stream: \"LANDMARKS:pose_landmarks\"\r\n" \
"  input_stream: \"SEGMENTATION_MASK:segmentation_mask\"\r\n" \
"  input_stream: \"DETECTION:pose_detection\"\r\n" \
"  input_stream: \"ROI:roi_from_landmarks\"\r\n" \
"  output_stream: \"IMAGE:output_video\"\r\n" \
"}\r\n";
#elif DETECT_MODE_HAND
// from mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt
std::string default_config_context = "input_stream: \"input_video\"\r\n" \
"output_stream: \"landmarks\"\r\n" \
"output_stream: \"world_landmarks\"\r\n" \
"node {\r\n" \
"  calculator: \"ConstantSidePacketCalculator\"\r\n" \
"  output_side_packet: \"PACKET:num_hands\"\r\n" \
"  node_options: {\r\n" \
"    [type.googleapis.com/mediapipe.ConstantSidePacketCalculatorOptions]: {\r\n" \
"      #packet { int_value: 2 }\r\n" \
"      packet { int_value: 1 }\r\n" \
"    }\r\n" \
"  }\r\n" \
"}\r\n" \
"node {\r\n" \
"  calculator: \"HandLandmarkTrackingCpu\"\r\n" \
"  input_stream: \"IMAGE:input_video\"\r\n" \
"  input_side_packet: \"NUM_HANDS:num_hands\"\r\n" \
"  output_stream: \"LANDMARKS:hand_landmarks\"\r\n" \
"  output_stream: \"WORLD_LANDMARKS:hand_world_landmarks\"\r\n" \
"  output_stream: \"HANDEDNESS:handedness\"\r\n" \
"  output_stream: \"PALM_DETECTIONS:multi_palm_detections\"\r\n" \
"  output_stream: \"HAND_ROIS_FROM_LANDMARKS:multi_hand_rects\"\r\n" \
"  output_stream: \"HAND_ROIS_FROM_PALM_DETECTIONS:multi_palm_rects\"\r\n" \
"}\r\n" \
"node {\r\n" \
"  calculator: \"HandRendererSubgraph\"\r\n" \
"  input_stream: \"IMAGE:input_video\"\r\n" \
"  input_stream: \"DETECTIONS:multi_palm_detections\"\r\n" \
"  input_stream: \"LANDMARKS:hand_landmarks\"\r\n" \
"  input_stream: \"HANDEDNESS:handedness\"\r\n" \
"  input_stream: \"NORM_RECTS:0:multi_palm_rects\"\r\n" \
"  input_stream: \"NORM_RECTS:1:multi_hand_rects\"\r\n" \
"  output_stream: \"IMAGE:output_video\"\r\n" \
"}\r\n";
#elif DETECT_MODE_FACE_MESH
// from mediapipe/graphs/face_mesh/face_mesh_desktop_live.pbtxt
std::string default_config_context = "input_stream: \"input_video\"\r\n" \
"output_stream: \"multi_face_landmarks\"\r\n" \
"node {\r\n" \
"  calculator: \"FlowLimiterCalculator\"\r\n" \
"  input_stream: \"input_video\"\r\n" \
"  input_stream: \"FINISHED:output_video\"\r\n" \
"  input_stream_info: {\r\n" \
"    tag_index: \"FINISHED\"\r\n" \
"    back_edge: true\r\n" \
"  }\r\n" \
"  output_stream: \"throttled_input_video\"\r\n" \
"}\r\n" \
"node {\r\n" \
"  calculator: \"ConstantSidePacketCalculator\"\r\n" \
"  output_side_packet: \"PACKET:0:num_faces\"\r\n" \
"  output_side_packet: \"PACKET:1:with_attention\"\r\n" \
"  node_options: {\r\n" \
"    [type.googleapis.com/mediapipe.ConstantSidePacketCalculatorOptions]: {\r\n" \
"      packet { int_value: 1 }\r\n" \
"      packet { bool_value: true }\r\n" \
"    }\r\n" \
"  }\r\n" \
"}\r\n" \
"node {\r\n" \
"  calculator: \"FaceLandmarkFrontCpu\"\r\n" \
"  input_stream: \"IMAGE:throttled_input_video\"\r\n" \
"  input_side_packet: \"NUM_FACES:num_faces\"\r\n" \
"  input_side_packet: \"WITH_ATTENTION:with_attention\"\r\n" \
"  output_stream: \"LANDMARKS:face_mesh_landmarks\"\r\n" \
"  output_stream: \"ROIS_FROM_LANDMARKS:face_rects_from_landmarks\"\r\n" \
"  output_stream: \"DETECTIONS:face_detections\"\r\n" \
"  output_stream: \"ROIS_FROM_DETECTIONS:face_rects_from_detections\"\r\n" \
"}\r\n" \
"node {\r\n" \
"  calculator: \"FaceRendererCpu\"\r\n" \
"  input_stream: \"IMAGE:throttled_input_video\"\r\n" \
"  input_stream: \"LANDMARKS:face_mesh_landmarks\"\r\n" \
"  input_stream: \"NORM_RECTS:face_rects_from_landmarks\"\r\n" \
"  input_stream: \"DETECTIONS:face_detections\"\r\n" \
"  output_stream: \"IMAGE:output_video\"\r\n" \
"}\r\n";
#endif // DETECT_MODE_FACE_MESH


const int MP_LANDMARK_VALUE_NO_X          = 1;
const int MP_LANDMARK_VALUE_NO_Y          = 2;
const int MP_LANDMARK_VALUE_NO_Z          = 3;
const int MP_LANDMARK_VALUE_NO_VISIBILITY = 4;
const int MP_LANDMARK_VALUE_NO_PRESENCE   = 5;

const int MP_LANDMARK_VALUE_NO_MAX        = MP_LANDMARK_VALUE_NO_PRESENCE;


int POSE_NO_COUNT = 0;
const int POSE_NO_NOSE = POSE_NO_COUNT++; // 0
const int POSE_NO_LEFT_EYE_INNER = POSE_NO_COUNT++; // 1
const int POSE_NO_LEFT_EYE = POSE_NO_COUNT++; // 2
const int POSE_NO_LEFT_EYE_OUTER = POSE_NO_COUNT++; // 3
const int POSE_NO_RIGHT_EYE_INNER = POSE_NO_COUNT++; // 4
const int POSE_NO_RIGHT_EYE = POSE_NO_COUNT++; // 5
const int POSE_NO_RIGHT_EYE_OUTER = POSE_NO_COUNT++; // 6
const int POSE_NO_LEFT_EAR = POSE_NO_COUNT++; // 7
const int POSE_NO_RIGHT_EAR = POSE_NO_COUNT++; // 8
const int POSE_NO_LEFT_MOUTH = POSE_NO_COUNT++; // 9
const int POSE_NO_RIGHT_MOUTH = POSE_NO_COUNT++; // 10
const int POSE_NO_LEFT_SHOULDER = POSE_NO_COUNT++; // 11
const int POSE_NO_RIGHT_SHOULDER = POSE_NO_COUNT++; // 12
const int POSE_NO_LEFT_ELBOW = POSE_NO_COUNT++; // 13
const int POSE_NO_RIGHT_ELBOW = POSE_NO_COUNT++; // 14
const int POSE_NO_LEFT_WRIST = POSE_NO_COUNT++; // 15
const int POSE_NO_RIGHT_WRIST = POSE_NO_COUNT++; // 16
const int POSE_NO_LEFT_PINKY = POSE_NO_COUNT++; // 17
const int POSE_NO_RIGHT_PINKY = POSE_NO_COUNT++; // 18
const int POSE_NO_LEFT_INDEX = POSE_NO_COUNT++; // 19
const int POSE_NO_RIGHT_INDEX = POSE_NO_COUNT++; // 20
const int POSE_NO_LEFT_THUMB = POSE_NO_COUNT++; // 21
const int POSE_NO_RIGHT_THUMB = POSE_NO_COUNT++; // 22
const int POSE_NO_LEFT_HIP = POSE_NO_COUNT++; // 23
const int POSE_NO_RIGHT_HIP = POSE_NO_COUNT++; // 24
const int POSE_NO_LEFT_KNEE = POSE_NO_COUNT++; // 25
const int POSE_NO_RIGHT_KNEE = POSE_NO_COUNT++; // 26
const int POSE_NO_LEFT_ANKLE = POSE_NO_COUNT++; // 27
const int POSE_NO_RIGHT_ANKLE = POSE_NO_COUNT++; // 28
const int POSE_NO_LEFT_HEEL = POSE_NO_COUNT++; // 29
const int POSE_NO_RIGHT_HEEL = POSE_NO_COUNT++; // 30
const int POSE_NO_LEFT_FOOT_INDEX = POSE_NO_COUNT++; // 31
const int POSE_NO_RIGHT_FOOT_INDEX = POSE_NO_COUNT++; // 32
// add
const int POSE_NO_CENTER_BODY = POSE_NO_COUNT++; // 33;
const int POSE_NO_CENTER_SHOULDER = POSE_NO_COUNT++; // 34;
const int POSE_NO_CENTER_HIP = POSE_NO_COUNT++; // 35;
const int POSE_NO_LEFT_BODY = POSE_NO_COUNT++; // 36;
const int POSE_NO_RIGHT_BODY = POSE_NO_COUNT++; // 37;
const int POSE_NO_FRONT_BODY = POSE_NO_COUNT++; // 38;
const int POSE_NO_FRONT_SHOULDER = POSE_NO_COUNT++; // 39;
const int POSE_NO_FRONT_HIP = POSE_NO_COUNT++; // 40;
const int POSE_NO_WORLD_CENTER_BODY = POSE_NO_COUNT++; // 41;
const int POSE_NO_CENTER_EYE = POSE_NO_COUNT++; // 42;
const int POSE_NO_CENTER_EAR = POSE_NO_COUNT++; // 43;
const int POSE_NO_CENTER_MOUTH = POSE_NO_COUNT++; // 44;
const int POSE_NO_CENTER_NOSE = POSE_NO_COUNT++; // 45;
const int POSE_NO_FACE_UP = POSE_NO_COUNT++; // 46;
const int POSE_NO_CENTER_LEFT_FINGER = POSE_NO_COUNT++; // 47;
const int POSE_NO_CENTER_RIGHT_FINGER = POSE_NO_COUNT++; // 48;
const int POSE_NO_LEFT_HAND_UP = POSE_NO_COUNT++; // 49;
const int POSE_NO_RIGHT_HAND_UP = POSE_NO_COUNT++; // 50;
const int POSE_NO_LEFT_HAND_FRONT = POSE_NO_COUNT++; // 51;
const int POSE_NO_RIGHT_HAND_FRONT = POSE_NO_COUNT++; // 52;

//const int POSE_NO_MAX = POSE_NO_RIGHT_FOOT_INDEX;
const int POSE_NO_MAX = POSE_NO_RIGHT_HAND_FRONT + 1; // add


int HAND_NO_COUNT = 0;
const int HAND_NO_WRIST = HAND_NO_COUNT++; // 0
const int HAND_NO_THUMB_CMC = HAND_NO_COUNT++; // 1
const int HAND_NO_THUMB_MCP = HAND_NO_COUNT++; // 2
const int HAND_NO_THUMB_IP = HAND_NO_COUNT++; // 3
const int HAND_NO_THUMB_TIP = HAND_NO_COUNT++; // 4
const int HAND_NO_INDEX_FINGER_MCP = HAND_NO_COUNT++; // 5
const int HAND_NO_INDEX_FINGER_PIP = HAND_NO_COUNT++; // 6
const int HAND_NO_INDEX_FINGER_DIP = HAND_NO_COUNT++; // 7
const int HAND_NO_INDEX_FINGER_TIP = HAND_NO_COUNT++; // 8
const int HAND_NO_MIDDLE_FINGER_MCP = HAND_NO_COUNT++; // 9
const int HAND_NO_MIDDLE_FINGER_PIP = HAND_NO_COUNT++; // 10
const int HAND_NO_MIDDLE_FINGER_DIP = HAND_NO_COUNT++; // 11
const int HAND_NO_MIDDLE_FINGER_TIP = HAND_NO_COUNT++; // 12
const int HAND_NO_RING_FINGER_MCP = HAND_NO_COUNT++; // 13
const int HAND_NO_RING_FINGER_PIP = HAND_NO_COUNT++; // 14
const int HAND_NO_RING_FINGER_DIP = HAND_NO_COUNT++; // 15
const int HAND_NO_RING_FINGER_TIP = HAND_NO_COUNT++; // 16
const int HAND_NO_PINKY_MCP = HAND_NO_COUNT++; // 17
const int HAND_NO_PINKY_PIP = HAND_NO_COUNT++; // 18
const int HAND_NO_PINKY_DIP = HAND_NO_COUNT++; // 19
const int HAND_NO_PINKY_TIP = HAND_NO_COUNT++; // 20
const int HAND_NO_CENTER = HAND_NO_COUNT++; // 21
const int HAND_NO_SIDE = HAND_NO_COUNT++; // 22
const int HAND_NO_UP = HAND_NO_COUNT++; // 23
const int HAND_NO_FRONT = HAND_NO_COUNT++; // 24
const int HAND_NO_WORLD_CENTER = HAND_NO_COUNT++; // 25

//const int HAND_NO_MAX = HAND_NO_PINKY_TIP;
const int HAND_NO_MAX = HAND_NO_WORLD_CENTER + 1; // add

// refer
//   image: https://github.com/google/mediapipe/blob/a908d668c730da128dfa8d9f6bd25d519d006692/mediapipe/modules/face_geometry/data/canonical_face_model_uv_visualization.png
//   text: https://github.com/tensorflow/tfjs-models/blob/838611c02f51159afdd77469ce67f0e26b7bbb23/face-landmarks-detection/src/mediapipe-facemesh/keypoints.ts
//int FACE_MESH_NO_COUNT = 0;
const int FACE_MESH_NO_NOSE = 1;
const int FACE_MESH_NO_TOP = 10;
const int FACE_MESH_NO_BOTTOM = 152;
const int FACE_MESH_NO_LEFT = 323;
const int FACE_MESH_NO_RIGHT = 93;
const int FACE_MESH_NO_NOSE_OUTSIDE_LEFT = 358;
const int FACE_MESH_NO_NOSE_OUTSIDE_RIGHT = 129;
const int FACE_MESH_NO_LEFT_CHEEK = 425;
const int FACE_MESH_NO_RIGHT_CHEEK = 205;
const int FACE_MESH_NO_LIP_INNER_UPPER = 13;
const int FACE_MESH_NO_LIP_INNER_LOWER = 14;
const int FACE_MESH_NO_LIP_INNER_LEFT_SIDE = 308;
const int FACE_MESH_NO_LIP_INNER_RIGHT_SIDE = 78;
const int FACE_MESH_NO_LEFT_EYE_UPPER = 386;
const int FACE_MESH_NO_LEFT_EYE_LOWER = 374;
const int FACE_MESH_NO_LEFT_EYE_IN_SIDE = 362;
const int FACE_MESH_NO_LEFT_EYE_OUT_SIDE = 263;
const int FACE_MESH_NO_LEFT_EYE_BROW_UPPER_IN_SIDE = 417;
const int FACE_MESH_NO_LEFT_EYE_BROW_UPPER_CENTER = 334;
const int FACE_MESH_NO_LEFT_EYE_BROW_UPPER_OUT_SIDE = 383;
const int FACE_MESH_NO_RIGHT_EYE_UPPER = 159;
const int FACE_MESH_NO_RIGHT_EYE_LOWER = 145;
const int FACE_MESH_NO_RIGHT_EYE_IN_SIDE = 133;
const int FACE_MESH_NO_RIGHT_EYE_OUT_SIDE = 33;
const int FACE_MESH_NO_RIGHT_EYE_BROW_UPPER_IN_SIDE = 193;
const int FACE_MESH_NO_RIGHT_EYE_BROW_UPPER_CENTER = 105;
const int FACE_MESH_NO_RIGHT_EYE_BROW_UPPER_OUT_SIDE = 156;
const int FACE_MESH_NO_LEFT_EYE_IRIS_CENTER = 468;
const int FACE_MESH_NO_RIGHT_EYE_IRIS_CENTER = 473;
const int FACE_MESH_NO_LEFT_UPPER_FACE = 251;
const int FACE_MESH_NO_LEFT_LOWER_FACE = 197;
const int FACE_MESH_NO_RIGHT_UPPER_FACE = 21;
const int FACE_MESH_NO_RIGHT_LOWER_FACE = 172;
int FACE_MESH_NO_COUNT = 478;
const int FACE_MESH_NO_LIP_CENTER = FACE_MESH_NO_COUNT++; // 478
const int FACE_MESH_NO_EYE_CENTER = FACE_MESH_NO_COUNT++; // 479
const int FACE_MESH_NO_LEFT_EYE_CENTER = FACE_MESH_NO_COUNT++; // 480
const int FACE_MESH_NO_RIGHT_EYE_CENTER = FACE_MESH_NO_COUNT++; // 481
const int FACE_MESH_NO_WORLD_CENTER = FACE_MESH_NO_COUNT++; // 482
const int FACE_MESH_NO_CENTER = FACE_MESH_NO_COUNT++; // 483
const int FACE_MESH_NO_SIDE = FACE_MESH_NO_COUNT++; // 484
const int FACE_MESH_NO_UP = FACE_MESH_NO_COUNT++; // 485
const int FACE_MESH_NO_FRONT = FACE_MESH_NO_COUNT++; // 486
const int FACE_MESH_NO_NOSE_INNER = FACE_MESH_NO_COUNT++; // 487

//FACE_MESH_NO_COUNT = 468;
//FACE_MESH_NO_COUNT = 478;

//const int FACE_MESH_NO_MAX = 478;
const int FACE_MESH_NO_MAX = FACE_MESH_NO_NOSE_INNER + 1; // add

float eyes_dist = 0.075; // 目間距離(m)

std::vector<int> face_around_silhouette = {10,  338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288, 397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136, 172, 58,  132, 93,  234, 127, 162, 21,  54,  103, 67,  109};
std::vector<int> face_lipsUpperOuter_silhouette = {61, 185, 40, 39, 37, 0, 267, 269, 270, 409, 291};
std::vector<int> face_lipsLowerOuter_silhouette = {146, 91, 181, 84, 17, 314, 405, 321, 375, 291};
std::vector<int> face_lipsUpperInner_silhouette = {78, 191, 80, 81, 82, 13, 312, 311, 310, 415, 308};
std::vector<int> face_lipsLowerInner_silhouette = {78, 95, 88, 178, 87, 14, 317, 402, 318, 324, 308};
std::vector<int> face_rightEyeUpper0_silhouette = {246, 161, 160, 159, 158, 157, 173};
std::vector<int> face_rightEyeLower0_silhouette = {33, 7, 163, 144, 145, 153, 154, 155, 133};
std::vector<int> face_rightEyeUpper1_silhouette = {247, 30, 29, 27, 28, 56, 190};
std::vector<int> face_rightEyeLower1_silhouette = {130, 25, 110, 24, 23, 22, 26, 112, 243};
std::vector<int> face_rightEyeUpper2_silhouette = {113, 225, 224, 223, 222, 221, 189};
std::vector<int> face_rightEyeLower2_silhouette = {226, 31, 228, 229, 230, 231, 232, 233, 244};
std::vector<int> face_rightEyeLower3_silhouette = {143, 111, 117, 118, 119, 120, 121, 128, 245};
std::vector<int> face_rightEyebrowUpper_silhouette = {156, 70, 63, 105, 66, 107, 55, 193};
std::vector<int> face_rightEyebrowLower_silhouette = {35, 124, 46, 53, 52, 65};
//std::vector<int> face_rightEyeIris_silhouette = {473, 474, 475, 476, 477};
std::vector<int> face_rightEyeIris_silhouette = {/*473, */474, 475, 476, 477}; // 先頭は目の中心なので外す
std::vector<int> face_leftEyeUpper0_silhouette = {466, 388, 387, 386, 385, 384, 398};
std::vector<int> face_leftEyeLower0_silhouette = {263, 249, 390, 373, 374, 380, 381, 382, 362};
std::vector<int> face_leftEyeUpper1_silhouette = {467, 260, 259, 257, 258, 286, 414};
std::vector<int> face_leftEyeLower1_silhouette = {359, 255, 339, 254, 253, 252, 256, 341, 463};
std::vector<int> face_leftEyeUpper2_silhouette = {342, 445, 444, 443, 442, 441, 413};
std::vector<int> face_leftEyeLower2_silhouette = {446, 261, 448, 449, 450, 451, 452, 453, 464};
std::vector<int> face_leftEyeLower3_silhouette = {372, 340, 346, 347, 348, 349, 350, 357, 465};
std::vector<int> face_leftEyebrowUpper_silhouette = {383, 300, 293, 334, 296, 336, 285, 417};
std::vector<int> face_leftEyebrowLower_silhouette = {265, 353, 276, 283, 282, 295};
//std::vector<int> face_leftEyeIris_silhouette = {468, 469, 470, 471, 472};
std::vector<int> face_leftEyeIris_silhouette = {/*468, */469, 470, 471, 472}; // 先頭は目の中心なので外す
std::vector<int> face_midwayBetweenEyes_silhouette = {168};
std::vector<int> face_noseTip_silhouette = {1};
std::vector<int> face_noseBottom_silhouette = {2};
std::vector<int> face_noseRightCorner_silhouette = {98};
std::vector<int> face_noseLeftCorner_silhouette = {327};
std::vector<int> face_rightCheek_silhouette = {205};
std::vector<int> face_leftCheek_silhouette = {425};


const int NO_X =          0;
const int NO_Y =          1;
const int NO_Z =          2;
const int NO_VISIBILITY = 3;
const int NO_PRESENCE =   4;

const int NO_MAX = NO_PRESENCE;


//std::vector<int> pose_vector_vec;
std::vector<int> pose_vector_vec {
  /*POSE_NO_NOSE*/ POSE_NO_CENTER_NOSE,
  /*POSE_NO_LEFT_EYE_INNER*/ POSE_NO_CENTER_EYE,
  /*POSE_NO_LEFT_EYE*/ POSE_NO_CENTER_EYE,
  /*POSE_NO_LEFT_EYE_OUTER*/ POSE_NO_CENTER_EYE,
  /*POSE_NO_RIGHT_EYE_INNER*/ POSE_NO_CENTER_EYE,
  /*POSE_NO_RIGHT_EYE*/ POSE_NO_CENTER_EYE,
  /*POSE_NO_RIGHT_EYE_OUTER*/ POSE_NO_CENTER_EYE,
  /*POSE_NO_LEFT_EAR*/ POSE_NO_CENTER_EAR,
  /*POSE_NO_RIGHT_EAR*/ POSE_NO_CENTER_EAR,
  /*POSE_NO_LEFT_MOUTH*/ POSE_NO_CENTER_MOUTH,
  /*POSE_NO_RIGHT_MOUTH*/ POSE_NO_CENTER_MOUTH,
  /*POSE_NO_LEFT_SHOULDER*/ POSE_NO_CENTER_SHOULDER,
  /*POSE_NO_RIGHT_SHOULDER*/ POSE_NO_CENTER_SHOULDER,
  /*POSE_NO_LEFT_ELBOW*/ POSE_NO_LEFT_SHOULDER,
  /*POSE_NO_RIGHT_ELBOW*/ POSE_NO_RIGHT_SHOULDER,
  /*POSE_NO_LEFT_WRIST*/ POSE_NO_LEFT_ELBOW,
  /*POSE_NO_RIGHT_WRIST*/ POSE_NO_RIGHT_ELBOW,
  /*POSE_NO_LEFT_PINKY*/ POSE_NO_LEFT_WRIST,
  /*POSE_NO_RIGHT_PINKY*/ POSE_NO_RIGHT_WRIST,
  /*POSE_NO_LEFT_INDEX*/ POSE_NO_LEFT_WRIST,
  /*POSE_NO_RIGHT_INDEX*/ POSE_NO_RIGHT_WRIST,
  /*POSE_NO_LEFT_THUMB*/ POSE_NO_LEFT_WRIST,
  /*POSE_NO_RIGHT_THUMB*/ POSE_NO_RIGHT_WRIST,
  /*POSE_NO_LEFT_HIP*/ POSE_NO_CENTER_HIP,
  /*POSE_NO_RIGHT_HIP*/ POSE_NO_CENTER_HIP,
  /*POSE_NO_LEFT_KNEE*/ POSE_NO_LEFT_HIP,
  /*POSE_NO_RIGHT_KNEE*/ POSE_NO_RIGHT_HIP,
  /*POSE_NO_LEFT_ANKLE*/ POSE_NO_LEFT_KNEE,
  /*POSE_NO_RIGHT_ANKLE*/ POSE_NO_RIGHT_KNEE,
  /*POSE_NO_LEFT_HEEL*/ POSE_NO_LEFT_ANKLE,
  /*POSE_NO_RIGHT_HEEL*/ POSE_NO_RIGHT_ANKLE,
  /*POSE_NO_LEFT_FOOT_INDEX*/ POSE_NO_LEFT_ANKLE,
  /*POSE_NO_RIGHT_FOOT_INDEX*/ POSE_NO_RIGHT_ANKLE,
  // 0 - 32(1 - 33)
  // add
  /*POSE_NO_CENTER_BODY*/ POSE_NO_CENTER_BODY,
  /*POSE_NO_CENTER_SHOULDER*/ POSE_NO_CENTER_BODY,
  /*POSE_NO_CENTER_HIP*/ POSE_NO_CENTER_BODY,
  /*POSE_NO_LEFT_BODY*/ POSE_NO_CENTER_BODY,
  /*POSE_NO_RIGHT_BODY*/ POSE_NO_CENTER_BODY,
  /*POSE_NO_FRONT_BODY*/ POSE_NO_CENTER_BODY,
  /*POSE_NO_FRONT_SHOULDER*/ POSE_NO_CENTER_SHOULDER,
  /*POSE_NO_FRONT_HIP*/ POSE_NO_CENTER_HIP,
  /*POSE_NO_WORLD_CENTER_BODY*/ POSE_NO_CENTER_BODY,
  /*POSE_NO_CENTER_EYE*/ POSE_NO_CENTER_EAR,
  /*POSE_NO_CENTER_EAR*/ POSE_NO_CENTER_SHOULDER,
  /*POSE_NO_CENTER_MOUTH*/ POSE_NO_CENTER_EAR,
  /*POSE_NO_CENTER_NOSE*/ POSE_NO_CENTER_EAR,
  /*POSE_NO_FACE_UP*/ POSE_NO_CENTER_MOUTH,
  /*POSE_NO_CENTER_LEFT_FINGER*/ POSE_NO_LEFT_WRIST,
  /*POSE_NO_CENTER_RIGHT_FINGER*/ POSE_NO_RIGHT_WRIST,
  /*POSE_NO_LEFT_HAND_UP*/ POSE_NO_CENTER_LEFT_FINGER,
  /*POSE_NO_RIGHT_HAND_UP*/ POSE_NO_CENTER_RIGHT_FINGER,
  /*POSE_NO_LEFT_HAND_FRONT*/ POSE_NO_CENTER_LEFT_FINGER,
  /*POSE_NO_RIGHT_HAND_FRONT*/ POSE_NO_CENTER_RIGHT_FINGER,
  // 33 - 52(34 - 53)
};


std::vector<int> hand_vector_vec {
  /*HAND_NO_WRIST*/ HAND_NO_WRIST,
  /*HAND_NO_THUMB_CMC*/ HAND_NO_WRIST,
  /*HAND_NO_THUMB_MCP*/ HAND_NO_THUMB_CMC,
  /*HAND_NO_THUMB_IP*/ HAND_NO_THUMB_MCP,
  /*HAND_NO_THUMB_TIP*/ HAND_NO_THUMB_IP,
  /*HAND_NO_INDEX_FINGER_MCP*/ HAND_NO_WRIST,
  /*HAND_NO_INDEX_FINGER_PIP*/ HAND_NO_INDEX_FINGER_MCP,
  /*HAND_NO_INDEX_FINGER_DIP*/ HAND_NO_INDEX_FINGER_PIP,
  /*HAND_NO_INDEX_FINGER_TIP*/ HAND_NO_INDEX_FINGER_DIP,
  /*HAND_NO_MIDDLE_FINGER_MCP*/ HAND_NO_WRIST,
  /*HAND_NO_MIDDLE_FINGER_PIP*/ HAND_NO_MIDDLE_FINGER_MCP,
  /*HAND_NO_MIDDLE_FINGER_DIP*/ HAND_NO_MIDDLE_FINGER_PIP,
  /*HAND_NO_MIDDLE_FINGER_TIP*/ HAND_NO_MIDDLE_FINGER_DIP,
  /*HAND_NO_RING_FINGER_MCP*/ HAND_NO_WRIST,
  /*HAND_NO_RING_FINGER_PIP*/ HAND_NO_RING_FINGER_MCP,
  /*HAND_NO_RING_FINGER_DIP*/ HAND_NO_RING_FINGER_PIP,
  /*HAND_NO_RING_FINGER_TIP*/ HAND_NO_RING_FINGER_DIP,
  /*HAND_NO_PINKY_MCP*/ HAND_NO_WRIST,
  /*HAND_NO_PINKY_PIP*/ HAND_NO_PINKY_MCP,
  /*HAND_NO_PINKY_DIP*/ HAND_NO_PINKY_PIP,
  /*HAND_NO_PINKY_TIP*/ HAND_NO_PINKY_DIP,
  /*HAND_NO_CENTER*/ HAND_NO_WRIST,
  /*HAND_NO_SIDE*/ HAND_NO_WRIST,
  /*HAND_NO_UP*/ HAND_NO_RING_FINGER_MCP,
  /*HAND_NO_FRONT*/ HAND_NO_MIDDLE_FINGER_MCP,
  /*HAND_NO_WORLD_CENTER*/ HAND_NO_WORLD_CENTER,
};

// 指の角度は第三関節から先端までの角度とする
// TODO
std::vector<int> hand_mcp_vector_map {
  /*HAND_NO_WRIST*/ HAND_NO_WRIST,
  /*HAND_NO_THUMB_CMC*/ HAND_NO_THUMB_CMC,
  /*HAND_NO_THUMB_MCP*/ HAND_NO_THUMB_CMC,
  /*HAND_NO_THUMB_IP*/ HAND_NO_THUMB_CMC,
  /*HAND_NO_THUMB_TIP*/ HAND_NO_THUMB_CMC,
  /*HAND_NO_INDEX_FINGER_MCP*/ HAND_NO_INDEX_FINGER_MCP,
  /*HAND_NO_INDEX_FINGER_PIP*/ HAND_NO_INDEX_FINGER_MCP,
  /*HAND_NO_INDEX_FINGER_DIP*/ HAND_NO_INDEX_FINGER_MCP,
  /*HAND_NO_INDEX_FINGER_TIP*/ HAND_NO_INDEX_FINGER_MCP,
  /*HAND_NO_MIDDLE_FINGER_MCP*/ HAND_NO_MIDDLE_FINGER_MCP,
  /*HAND_NO_MIDDLE_FINGER_PIP*/ HAND_NO_MIDDLE_FINGER_MCP,
  /*HAND_NO_MIDDLE_FINGER_DIP*/ HAND_NO_MIDDLE_FINGER_MCP,
  /*HAND_NO_MIDDLE_FINGER_TIP*/ HAND_NO_MIDDLE_FINGER_MCP,
  /*HAND_NO_RING_FINGER_MCP*/ HAND_NO_RING_FINGER_MCP,
  /*HAND_NO_RING_FINGER_PIP*/ HAND_NO_RING_FINGER_MCP,
  /*HAND_NO_RING_FINGER_DIP*/ HAND_NO_RING_FINGER_MCP,
  /*HAND_NO_RING_FINGER_TIP*/ HAND_NO_RING_FINGER_MCP,
  /*HAND_NO_PINKY_MCP*/ HAND_NO_PINKY_MCP,
  /*HAND_NO_PINKY_PIP*/ HAND_NO_PINKY_MCP,
  /*HAND_NO_PINKY_DIP*/ HAND_NO_PINKY_MCP,
  /*HAND_NO_PINKY_TIP*/ HAND_NO_PINKY_MCP,
};


std::vector<int> face_mesh_vector_vec;
void init_face_mesh_vector_vec();
void init_face_mesh_vector_vec() {
  for (int i = 0; i < FACE_MESH_NO_MAX; i++) {
    face_mesh_vector_vec.push_back(-1);
  }
  // nose
  face_mesh_vector_vec[FACE_MESH_NO_NOSE] = FACE_MESH_NO_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_NOSE_OUTSIDE_LEFT] = FACE_MESH_NO_NOSE_INNER;
  face_mesh_vector_vec[FACE_MESH_NO_NOSE_OUTSIDE_RIGHT] = FACE_MESH_NO_NOSE_INNER;
  // face
  face_mesh_vector_vec[FACE_MESH_NO_TOP] = FACE_MESH_NO_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_BOTTOM] = FACE_MESH_NO_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_LEFT] = FACE_MESH_NO_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_RIGHT] = FACE_MESH_NO_CENTER;
  // lip
  face_mesh_vector_vec[FACE_MESH_NO_LIP_INNER_UPPER] = FACE_MESH_NO_LIP_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_LIP_INNER_LOWER] = FACE_MESH_NO_LIP_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_LIP_INNER_LEFT_SIDE] = FACE_MESH_NO_LIP_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_LIP_INNER_RIGHT_SIDE] = FACE_MESH_NO_LIP_CENTER;
  // eye left
  face_mesh_vector_vec[FACE_MESH_NO_LEFT_EYE_UPPER] = FACE_MESH_NO_LEFT_EYE_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_LEFT_EYE_LOWER] = FACE_MESH_NO_LEFT_EYE_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_LEFT_EYE_IN_SIDE] = FACE_MESH_NO_LEFT_EYE_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_LEFT_EYE_OUT_SIDE] = FACE_MESH_NO_LEFT_EYE_CENTER;
  // eye right
  face_mesh_vector_vec[FACE_MESH_NO_RIGHT_EYE_UPPER] = FACE_MESH_NO_RIGHT_EYE_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_RIGHT_EYE_LOWER] = FACE_MESH_NO_RIGHT_EYE_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_RIGHT_EYE_IN_SIDE] = FACE_MESH_NO_RIGHT_EYE_CENTER;
  face_mesh_vector_vec[FACE_MESH_NO_RIGHT_EYE_OUT_SIDE] = FACE_MESH_NO_RIGHT_EYE_CENTER;
}


template <class TList, class T> void set_and_calc_average(const TList& landmarks, std::vector<int> target_list, T& average_landmark);
//void set_and_calc_average(const mediapipe::LandmarkList& landmarks, std::vector<int> target_list, mediapipe::Landmark& average_landmark);
//void set_and_calc_average(const mediapipe::NormalizedLandmarkList& landmarks, std::vector<int> target_list, mediapipe::NormalizedLandmark& average_landmark);
// template: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void convert_x_slide_landmark(TList& landmarks);
// template: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void convert_x_reverse_landmark(TList& landmarks);
// template: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void add_and_calc_landmark_pose(const TList& landmarks, TList& added_landmarks);
// template: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void add_and_calc_landmark_hand(const TList& landmarks, TList& added_landmarks);
// template: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void add_and_calc_landmark_face_mesh(const TList& landmarks, TList& added_landmarks);
void calc_from_landmark_to_world_landmark_face_mesh(const mediapipe::NormalizedLandmarkList& landmarks, mediapipe::LandmarkList& world_landmarks);
void calc_and_set_world_center_body_landmark(mediapipe::NormalizedLandmarkList& landmarks_2d, mediapipe::LandmarkList& landmarks_3d);
void calc_and_set_world_center_hand_landmark(mediapipe::NormalizedLandmarkList& landmarks_2d, mediapipe::LandmarkList& landmarks_3d);
void calc_and_set_world_center_face_mesh_landmark(mediapipe::NormalizedLandmarkList& landmarks_2d, mediapipe::LandmarkList& landmarks_3d);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> std::vector<float> calc_vector_to_vec(const T& landmark, const T& parent_landmark);
// template TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
template <class TList> std::vector<float> calc_vector_from_parent_index_to_vec(const TList& landmarks, int target_index, int parent_index);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T calc_vector(const T& landmark, const T& parent_landmark);
// template TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
template <class TList, class T> T calc_vector_from_parent_index(const TList& landmarks, int target_index, int parent_index);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T calc_vector_reverse(const T& landmark, const T& parent_landmark);
// template TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
template <class TList, class T> T calc_vector_from_parent_index_reverse(const TList& landmarks, int target_index, int parent_index);
// template TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList, T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void calc_world_vector_from_parent(const TList& landmarks, std::vector<int>& vector_vec, TList& landmarks_vec);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T get_reverse_vec(const T& landmark);
extern float calc_rad2dig(float radian);
extern float calc_dig2rad(float digree);
// template: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> float calc_vector_length(const T& vec);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> float calc_dot(const T& vec_1, const T& vec_2);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> float calc_subtended_angle(const T& parent_landmark, const T& landmark);
extern void calc_angles(mediapipe::LandmarkList& landmarks, std::vector<int> pose_vector_vec);
extern void calc_camera_diff_angle(mediapipe::LandmarkList& landmarks, mediapipe::LandmarkList& camera_diff_angle);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> void set_x_line(T& vec);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> void set_y_line(T& vec);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> void set_z_line(T& vec);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> void set_xyz_line(T& vec);
//extern std::pair<float, int> get_line_consider(std::vector& consider_vec, int target_line, int turn_line);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> std::pair<float, int> get_line_consider(T& consider_vec, int target_line, int turn_line);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> std::pair<float, int> get_line_consider_and_to(T& consider_vec, int target_line, int turn_line);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T get_line_turn_ajust(T& input_vector, T& consider_vec, int target_line, int turn_line, bool is_reverse);
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T get_line_turn_ajust_x(T& input_vector);

void logout_coordinate();
void logout_landmarks(const mediapipe::NormalizedLandmarkList& landmarks, std::string indent = "");
void logout_landmark(const mediapipe::NormalizedLandmarkList& landmarks, std::vector<int> number_specify, std::string indent = "");
void logout_world_landmarks(const mediapipe::LandmarkList& landmarks, std::string indent = "");
void logout_world_landmark(const mediapipe::LandmarkList& landmarks, std::vector<int> number_specify, std::string indent = "");

int scale_size = 1000;
int scale_size_x = 640;
int scale_size_y = 320;
int scale_size_z = 100;

std::vector<std::vector<mediapipe::CalculatorGraph*> > graphs_vec;
bool grab_frames = false;

std::mutex mutex_key_code;
//std::unique_lock<std::mutex> lock_key_code(mutex_key_code);
std::condition_variable condition_key_code;
std::vector<std::pair<int, int> > queue_key_code;
const int key_code_wait_interval_msec = 50;

int game_start_wait_msec = 3000;

std::vector<int> logout_type_specify;
std::vector<int> logout_number_specify;


////////////////
// test/debug //
//#define DEFAULT_IMAGE_SHOW_WAIT_MSEC 300
#define DEFAULT_IMAGE_SHOW_WAIT_MSEC 100
//#define DEFAULT_IMAGE_SHOW_WAIT_MSEC 1000
//#define DEFAULT_IMAGE_SHOW_WAIT_MSEC 0


bool is_input_frame = false;
int count_of_input_frame = 0; 

int wait_sleep_time_mili_second = 1000;
//int wait_sleep_time_mili_second = 100; // test

mediapipe::NormalizedLandmarkList last_target_vec_2d; // 2dの位置(x, y, z)
mediapipe::NormalizedLandmarkList last_2nd_target_vec_2d; // 2dの位置(x, y, z)
mediapipe::LandmarkList last_target_vec; // center bodyからの相対位置(x, y, z)
mediapipe::LandmarkList last_2nd_target_vec; // center bodyからの相対位置(x, y, z)
mediapipe::LandmarkList last_target_vec_angle; // 関節相対位置(x, y, z)、関節角度(presence)
mediapipe::LandmarkList last_2nd_target_vec_angle; // 関節相対位置(x, y, z)、関節角度(presence)
mediapipe::LandmarkList last_target_camera_diff_angle; // カメラ相対角度(x, y, z)
mediapipe::LandmarkList last_2nd_target_camera_diff_angle; // カメラ相対角度(x, y, z)
bool is_record_model_mode = false;
// array
int current_number = 0;
int max_model_vecs_size = 10;
std::vector<double> set_model_threshold_vecs;

////////////////
// frame diff //
////////////////
template <class T> void add_vector_specify_size(std::vector<T>& vec, T& context, int& max_size);
void add_frame_diff_landmarks(size_t& timestamp, mediapipe::LandmarkList& landmarks, mediapipe::NormalizedLandmarkList& landmarks_2d, mediapipe::LandmarkList& vec_landmarks, std::vector<mediapipe::LandmarkList>& landmarks_vec, std::vector<mediapipe::NormalizedLandmarkList>& landmarks_2d_vec, std::vector<mediapipe::LandmarkList>& landmarks_vec_vec, std::vector<uint64_t>& timestamps_vec);

std::vector<mediapipe::LandmarkList> frame_diff_landmarks;
std::vector<mediapipe::NormalizedLandmarkList> frame_diff_2d_landmarks;
std::vector<mediapipe::LandmarkList> frame_diff_vec_landmarks;
std::vector<uint64_t> frame_diff_timestamps; // for frame_diff_landmarks timestamp
int frame_diff_landmarks_max_buffer_msec = 1000;


///////////
// image //
///////////
auto loadImageForMediapipe(std::string& file_path);
void drawLineHorizon(cv::Mat& image, int size, int height, cv::Scalar color);
void drawLineVertical(cv::Mat& image, int size, int width, cv::Scalar color);
cv::Scalar getScalarColorFromInt(int color_int);
int getColorFromPaletto();
void drawLineUpDownAndLeftRightThreshold(cv::Mat& image, int size);
void drawProcInfo(cv::Mat& image);
void drawMakerInfo(cv::Mat& image);
bool checkIsOnMaskImage();
void setMaskImage(cv::Mat& image);
cv::Mat getROIImage(cv::Mat& image);
void convertMosicImage(cv::Mat& image, int multiple);
void drawFaceMosic(cv::Mat& image, int multiple, float multiple_width, float multiple_height, mediapipe::NormalizedLandmarkList& landmarks);
void drawLandmarksPose(cv::Mat& image, mediapipe::NormalizedLandmarkList& landmarks);
void drawLandmarksHand(cv::Mat& image, mediapipe::NormalizedLandmarkList& landmarks);
void drawLandmarksFace(cv::Mat& image, mediapipe::NormalizedLandmarkList& landmarks);
void drawCircleLines(cv::Mat& image, mediapipe::NormalizedLandmarkList& landmarks, std::vector<int>& silhouette, bool is_circle, cv::Scalar color);

const int HELP_DEBUG_LINE_SIZE = 2;

bool is_display_mirror = true;
//bool is_display_mirror = false;


///////////
// sound //
///////////
long getFileSize(std::string& file_path);
char* loadBinaryFile(std::string& file_path);
char* loadTextFile(std::string& file_path);
bool check_exist_file(std::string file_path);


////////////
// action //
////////////
void doKeyEvent(int key_code, int key_flag, int wait_mili_second);
void doMouseEvent(int event, int dx, int dy, bool is_absolute);
void doMultiKeyEvent(int key_code, int key_flag, int wait_mili_second = 0);
int doMultiEvent(std::vector<int> key_codes, int index, int key_flag, int wait_mili_second = 0);
int getMousePoint(int& x, int& y);
void recordMousePoint(int& current_number);
void clearMousePoint(int& current_number);
void recordKeyCode(int& current_number, int& key_code);
void clearKeyCode(int& current_number);
void clearRecordAction(int& current_number);
void mouseEvent(int event, int dx, int dy, bool is_absolute = false);

// use (keybd_event/mouse_event) or SendInput
bool is_use_keyboard_mouse_event_now = false; // SendInput
//bool is_use_keyboard_mouse_event_now = true; // keybd_event/mouse_event

//int key_default_flag = 0;
int key_default_flag = KEYEVENTF_EXTENDEDKEY; // 0x0001
int mouse_default_flag = 0;
int sleep_default_flag = 0;
int other_default_flag = 0;

const int SLEEP_FLAG                = 0x00010000; // sleep(ms)最大値は0xffff（下16bit）

const int MOUSE_FLAG                = 0x00020000;
const int MOUSE_INDEX_D_X           = 1;
const int MOUSE_INDEX_D_Y           = 2;
const int MOUSE_INDEX_INCREMENT     = MOUSE_INDEX_D_Y;
// const int MOUSE_F_NO_ABSOLUTE       = 0x00010001; // not use this flag, use key_code in absolute flag.

const int KEYBD_FLAG                = 0x00040000;
const int SLEEP_NANO_FLAG           = 0x00080000; // sleep(ns)最大値は0xffff（下16bit）

// do not use, use MOUSEEVENTF_XXX
const int CLICK_MOUSE_DOWN_LEFT         = 0x0101;
const int CLICK_MOUSE_DOWN_RIGHT        = 0x0102;
const int CLICK_MOUSE_DOWN_MIDDLE       = 0x0103;
const int CLICK_MOUSE_DOWN_SCROLL_LEFT  = 0x0104;
const int CLICK_MOUSE_DOWN_SCROLL_RIGHT = 0x0105;
const int CLICK_MOUSE_SCROLL_UP         = 0x0106;
const int CLICK_MOUSE_DOWN_UP_LEFT      = 0x0107;
const int CLICK_MOUSE_DOWN_UP_RIGHT     = 0x0108;
const int CLICK_MOUSE_DOWN_UP_MIDDLE    = 0x0109;
const int CLICK_MOUSE_MOVE              = 0x0110;
const int CLICK_MOUSE_UP_LEFT           = 0x0201;
const int CLICK_MOUSE_UP_RIGHT          = 0x0202;
const int CLICK_MOUSE_UP_MIDDLE         = 0x0203;
const int CLICK_MOUSE_UP_SCROLL_LEFT    = 0x0204; // not exist
const int CLICK_MOUSE_UP_SCROLL_RIGHT   = 0x0205; // not exist
const int CLICK_MOUSE_SCROLL_DOWN       = 0x0206;
/*
// refer https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-mouse_event
MOUSEEVENTF_ABSOLUTE   0x8000
MOUSEEVENTF_LEFTDOWN   0x0002
MOUSEEVENTF_LEFTUP     0x0004
MOUSEEVENTF_MIDDLEDOWN 0x0020
MOUSEEVENTF_MIDDLEUP   0x0040
MOUSEEVENTF_MOVE       0x0001
MOUSEEVENTF_RIGHTDOWN  0x0008
MOUSEEVENTF_RIGHTUP    0x0010
MOUSEEVENTF_WHEEL      0x0800
MOUSEEVENTF_XDOWN      0x0080
MOUSEEVENTF_XUP        0x0100
MOUSEEVENTF_WHEEL      0x0800
MOUSEEVENTF_HWHEEL     0x1000
*/
const float DISPLAY_RATE = 65535.0;
int main_display_width = 1920;
int main_display_height = 1080;
int no_mouse_absolute_rate = 0;
float display_rate_x = DISPLAY_RATE / main_display_width;
float display_rate_y = DISPLAY_RATE / main_display_height;
int focus_display_left = -1;
int focus_display_top = -1;
int focus_display_right = -1;
int focus_display_bottom = -1;
int cut_left = 0;
int cut_top = 0;
int cut_right = 0;
int cut_bottom = 0;
int focus_display_width = 0;
int focus_display_height = 0;

void fix_person_size_scale();
void fix_person_size_scale_coordinate();
void fix_person_size_scale_sum();
void fix_person_size_scale_frame_diff();
void fix_person_size_scale_dist_compare();
void fix_person_size_scale_mark_compare();
void fix_person_size_scale_vector_dist_compare();
float person_size_scale_coordinate = 1.0;
float person_size_scale_sum = 1.0;
float person_size_scale_frame_diff = 1.0;
float person_size_scale_dist_compare = 1.0;
float person_size_scale_mark_compare = 1.0;
float person_size_scale_vector_dist_compare = 1.0;


// coordinate v1
// T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> bool runCalcPoseCoordinateAndAction_v1(T& target_landmark);

const int KEY_SHIFT              = 0x10;   // 16
const int KEY_CTRL               = 0x11;   // 17
const int KEY_ALT                = 0x12;   // 18
const int KEY_LEFT_ARROW         = 0x25;   // 37
const int KEY_UP_ARROW           = 0x26;   // 38
const int KEY_RIGHT_ARROW        = 0x27;   // 39
const int KEY_DOWN_ARROW         = 0x28;   // 40
const int KEY_LEFT_ARROW_SHIFT   = 0x1025;
const int KEY_UP_ARROW_SHIFT     = 0x1026;
const int KEY_RIGHT_ARROW_SHIFT  = 0x1027;
const int KEY_DOWN_ARROW_SHIFT   = 0x1028;
const int KEY_LEFT_ARROW_CTRL    = 0x1125;
const int KEY_UP_ARROW_CTRL      = 0x1126;
const int KEY_RIGHT_ARROW_CTRL   = 0x1127;
const int KEY_DOWN_ARROW_CTRL    = 0x1128;

const int NO_KEY_MAX_OVER_THRESHOLD    = 0;
const int NO_KEY_MIDDLE_OVER_THRESHOLD = 1;
const int NO_KEY_MIN_OVER_THRESHOLD    = 2;
const int NO_KEY_MAX                   = 3;
std::vector<float> key_arrow_down_up_threshold;
std::vector<float> key_arrow_left_right_threshold;


// coordinate v2
// T: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
template <class T> float getLandmarkValueFromCode(T& landmark, /*int& landmarks_array_no, */int& value_no);
float getCoordinateValueFromCode(std::vector<int>& landmark_code);
bool isConditionValue(float& value, float& threshold, int& condition_no);
bool runCalcPoseCoordinateAndAction_v2(std::vector<std::vector<int> >& target_key_code_list, std::vector<std::vector<int> >& target_key_points, std::vector<std::pair<float, int> >& threshold_condition_list, std::vector<int>& target_flag_list, int only_no = -1);

const int KEY_POINT_COORDINATE_MODE_NO     = 0;
const int KEY_POINT_LANDMARKS_ARRAY_NO     = 1;
const int KEY_POINT_LANDMARK_COORDINATE_NO = 2;

const int KEY_POINT_NO_MAX = KEY_POINT_LANDMARK_COORDINATE_NO + 1;

const int COORDINATE_MODE_LAST_2D                = 1;
const int COORDINATE_MODE_LAST_3D                = 2;
const int COORDINATE_MODE_LAST_3D_VEC_ANGLE      = 3;
const int COORDINATE_MODE_LAST_CAMERA_DIFF_ANGLE = 4;
const int COORDINATE_MODE_BASE_2D                = 5; // TODO
const int COORDINATE_MODE_BASE_3D                = 6; // TODO
const int COORDINATE_MODE_BASE_3D_VEC_ANGLE      = 7; // TODO
const int COORDINATE_MODE_BASE_CAMERA_DIFF_ANGLE = 8; // TODO

const int COORDINATE_MODE_BASE_PLUS = COORDINATE_MODE_BASE_2D - COORDINATE_MODE_LAST_2D;

#ifdef DETECT_MODE_POSE
const int COORDINATE_MODE_NO_MAX = COORDINATE_MODE_LAST_CAMERA_DIFF_ANGLE;
#elif DETECT_MODE_HAND
const int COORDINATE_MODE_NO_MAX = COORDINATE_MODE_LAST_CAMERA_DIFF_ANGLE;
#elif DETECT_MODE_FACE_MESH
const int COORDINATE_MODE_NO_MAX = COORDINATE_MODE_LAST_3D_VEC_ANGLE; // TODO not imple: 3D_VEC_ANGLE(一部), CAMERA_DIFF_ANGLE
#endif // DETECT_MODE_FACE_MESH

const int CONDITION_LESS_THAN       = 1;
const int CONDITION_EQUAL_LESS_THAN = 2;
const int CONDITION_EQUAL           = 3;
const int CONDITION_EQUAL_MORE_THAN = 4;
const int CONDITION_MORE_THAN       = 5;

const int CONDITION_NO_MAX = CONDITION_MORE_THAN;

std::vector<std::vector<int> > action_key_code_list;
std::vector<std::vector<int> > target_key_points;
std::vector<std::pair<float, int> > threshold_condition_list;
std::vector<int> action_flag_list;
std::vector<cv::Scalar> display_color_list;


// coordinate v3
// T: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
//template <class T> float getLandmarkValueFromCode(T& landmark, /*int& landmarks_array_no, */int& value_no);
//float getCoordinateValueFromCode(std::vector<int>& landmark_code);
//bool isConditionValue(float& value, float& threshold, int& condition_no);
bool runCalcPoseCoordinateAndAction_v3(std::vector<std::vector<int> >& target_key_code_list, std::vector<std::vector<int> >& target_key_points, std::vector<int>& target_flag_lists, int only_no = -1);

//const int KEY_POINT_COORDINATE_MODE_NO     = 0;
//const int KEY_POINT_LANDMARKS_ARRAY_NO     = 1;
//const int KEY_POINT_LANDMARK_COORDINATE_NO = 2;
const int KEY_POINT_THRESHOLD_VALUE_NO       = 3;
const int KEY_POINT_CONDITION_NO             = 4;

const int KEY_POINT_V3_MAX                   = KEY_POINT_CONDITION_NO + 1;

const int KEY_POINT_THRESHOLD_VALUE_DIVID = 1000;

std::vector<std::vector<int> > coord_v3_action_key_code_list;
std::vector<std::vector<int> > coord_v3_target_key_points;
  // multi point: 複数の条件をANDとする（orにはしない、orは、action_key_code_list側を増やす）
std::vector<int> coord_v3_last_run_key;
std::vector<int> coord_v3_action_flag_list;
std::vector<cv::Scalar> coord_v3_display_color_list;


// coordinate v4
// TODO 文法を記述できるAPIを活用したい。（JavaScriptのexecutionみたいな）


// coordinate sum v1
//const int KEY_POINT_COORDINATE_MODE_NO     = 0;
//const int KEY_POINT_LANDMARKS_ARRAY_NO     = 1;

const int KEY_POINT_SUM_V1_MAX = KEY_POINT_LANDMARKS_ARRAY_NO + 1;

// T: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
template <class T> float getVectorValueFrom2Landmark(T& landmark, T& landmark_2nd);
float getVectorValueFromCode(std::vector<int> landmark_code, float visibility_threshold);
//bool isConditionValue(float& value, float& threshold, int& condition_no);
bool runSumAndCalcPoseVectorAndAction_v1(std::vector<std::vector<int> >& sum_target_key_code_list, std::vector<std::vector<int> >& sum_target_key_points, std::vector<std::pair<float, int> >& sum_threshold_condition_list, std::vector<float>& sum_vector_list, std::vector<int>& sum_target_flag_list, int only_no = -1);

// 条件は、座標移動のv2と類似で、あとは3D座標の移動距離を蓄積にする
std::vector<std::vector<int> > sum_action_key_code_list;
std::vector<float> sum_vector_list;
std::vector<std::vector<int> > sum_target_key_points; // multi point: 複数を合計していく
std::vector<std::pair<float, int> > sum_threshold_condition_list;
std::vector<int> sum_action_flag_list;
std::vector<float> sum_visibility_threshold_list;


// base diff v1
//   TODO imple
void setBaseData();


mediapipe::NormalizedLandmarkList base_diff_target_vec_2d; // 2dの位置(x, y, z)
mediapipe::LandmarkList base_diff_target_vec; // center bodyからの相対位置(x, y, z)
mediapipe::LandmarkList base_diff_target_vec_angle; // 関節相対位置(x, y, z)、関節角度(presence)
mediapipe::LandmarkList base_diff_target_camera_diff_angle; // 関節相対位置(x, y, z)、関節角度(presence)
std::vector<int> base_diff_v1_action_key_code_list;
std::vector<std::vector<int> > base_diff_v1_target_key_points;


// frame diff v1
uint64_t runActionCalcDiff_v1(int only_no = -1, uint64_t arg_input_last_run_timestamp = (uint64_t)0);
bool check_x_front_reverse();
bool check_x_front_no_reverse();
void check_change_x_front_reverse();

std::vector<std::vector<int> > frame_diff_v1_action_key_code_list;
std::vector<std::vector<int> > frame_diff_v1_key_points;
std::vector<std::pair<float, int> > frame_diff_v1_threshold_condition;
std::vector<std::vector<std::pair<float, int> > > frame_diff_v1_check_vector;
std::vector<std::vector<std::pair<float, int> > > frame_diff_v1_check_vector_rate;
// flag(group, skip, etc)
std::vector<int> frame_diff_v1_flags;
std::vector<size_t> frame_diff_v1_last_action_timestamp_list;

const int FD_V1_CHECK_VECTOR_X = 0;
const int FD_V1_CHECK_VECTOR_Y = 1;
const int FD_V1_CHECK_VECTOR_Z = 2;

const int FD_V1_CHECK_VECTOR_NO_MAX = FD_V1_CHECK_VECTOR_Z + 1;

const int FD_V1_FLAG_NO_UP_AFTER_DOWN           = 0x0001; // このキー実行すると即座にボタンをUP(リリース)しない
const int FD_V1_FLAG_NO_UP_NOT_MATCH            = 0x0002; // ポーズ等が不一致の場合、キーUPしない
// TODO not imple ing
const int FD_V1_FLAG_NO_UP_LAST_KEY             = 0x0004; // このキーは最後のボタンのみ押し続ける。他のキーはUP(リリース)する
                                                          // 他のキーUPより優先する
const int FD_V1_FLAG_UP_ONLY                    = 0x0008; // このキーはキーUP(リリース)のみ。DOWNはしない。 // TODO nothing
// TODO not imple
const int FD_V1_FLAG_LOOP                       = 0x0010; // このキーは別スレッドで連続実行させる。（連続的に入力させるフラグ）
const int FD_V1_FLAG_SKIP_ALL_KEY               = 0x0020; // このキー実行すると他のキーは実行しない。このフレーム以前も同様 // TODO nothing
const int FD_V1_FLAG_SKIP_SPECIFY_KEY           = 0x0040; // このキー実行すると指定キーは実行しない。このフレーム以前も同様 // TODO nothing
                                                          //   指定は、次のベクトルに以降に順番の番号を入れておく。
                                                          //   順番の番号は、frame_diff_v1_action_key_code_list。
const int FD_V1_FLAG_WAIT_KET                   = 0x0080; // このキー実行を少し待つ。未来に実行したとセットする。 // TODO nothing
                                                          //   ウェイトミリ秒は次のベクトルに入れておく。
                                                          //   ゲーム側が複数のボタンの入力待ちをしているかもなので不要かも？

bool is_x_front_reverse = false;
int x_front_reverse_check_msec = 500;
bool is_enable_change_x_front_reverse = false;


// landmark 2 point dist compare v1
std::vector<std::vector<int> > landmark_2point_dist_compare_v1_action_key_code_list;
std::vector<std::vector<int> > landmark_2point_dist_compare_v1_key_points;
std::vector<int> landmark_2point_dist_compare_v1_flag;

const int DIST_COMPARE_COORDINATE_MODE_NO = 0;
const int DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST = 1;
const int DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND = 2;
const int DIST_COMPARE_THRESHOLD_VALUE_NO = 3;
const int DIST_COMPARE_CONDITION_NO = 4;

const int DIST_COMPARE_V1_MAX = DIST_COMPARE_CONDITION_NO + 1;

const int DIST_COMPARE_DIVID_RATE = 1000;


template <class TList, class T> float getLandmarkValueFromCode(TList& landamrks, int& landmarks_array_no_1st, int& landmarks_array_no_2nd);
float get2CoordinateLandmarkFromCode(std::vector<int>& key_points);
bool runLandmark2PointDistCompareAction_v1(std::vector<std::vector<int> >& target_key_code_list, std::vector<std::vector<int> >& target_key_points, std::vector<int>& target_flag_lists, int only_no = -1);


// mix action v1
std::vector<std::vector<int> > mix_v1_action_key_code_list;
std::vector<int> mix_v1_action_flag;
std::vector<std::vector<int> > mix_v1_action_condition_list;
std::map<uint64_t, uint64_t> mix_v1_last_run_timestamp; // first: (condition_list_no1 << 48) | (condition_list_no2 << 32) | condition_list_value;

const int CONDITION_FLAG_MODE = 0xffff0000;
const int CONDITION_FLAG_NO   = 0x0000ffff;

const int CONDITION_MODE_COORDINATE_V1          = 0x00010000;
const int CONDITION_MODE_COORDINATE_V2          = 0x00020000;
const int CONDITION_MODE_COORDINATE_V3          = 0x00030000;
const int CONDITION_MODE_SUM_VECTOR_V1          = 0x00040000;
const int CONDITION_MODE_POSE_DIFF_V1           = 0x00050000;
const int CONDITION_MODE_CAMERA_DIFF_V1         = 0x00060000;
const int CONDITION_MODE_FRAME_DIFF_V1          = 0x00070000;
const int CONDITION_MODE_DIST_COMPARE_V1        = 0x00080000;
const int CONDITION_MODE_MIX_V1                 = 0x00100000;
const int CONDITION_MODE_MARK_COMPARE_V1        = 0x00200000;
const int CONDITION_MODE_CONTINUE_V1            = 0x00400000;
const int CONDITION_MODE_VECTOR_DIST_COMPARE_V1 = 0x00800000;

const int ACTION_CODE_DUMMY = 0x00;

bool runActionCalcMix_v1(std::map<uint64_t, uint64_t>& last_run_timestamp, int only_no = -1);


// mark compare v1
std::vector<std::vector<int> > mark_compare_v1_action_key_code_list;
std::vector<std::vector<int> > mark_compare_v1_not_action_key_code_list;
std::vector<int> mark_compare_v1_neutral_key_group_ids;
std::vector<int> mark_compare_v1_action_flag;
// key points: 0:座標系, 1:座標軸系, 2:関節A, 3:関節B, 4:関節Aoffset(*1000), 5:関節Boffset(*1000), 6:A-B比較条件
std::vector<std::vector<int> > mark_compare_v1_key_points;

const int MARK_COMPARE_COORDINATE_MODE_NO =   0;
const int MARK_COMPARE_COORDINATE_LINE_NO =   1;
const int MARK_COMPARE_LANDMARKS_ARRAY_NO_A = 2;
const int MARK_COMPARE_LANDMARKS_ARRAY_NO_B = 3;
const int MARK_COMPARE_OFFSET_A =             4;
const int MARK_COMPARE_OFFSET_B =             5;
const int MARK_COMPARE_CONDITION_NO =         6;

const int MARK_COMPARE_V1_MAX =               MARK_COMPARE_CONDITION_NO + 1;

const int MARK_COMPARE_DIVID_RATE = 1000;


bool runActionCalcMarkCompare_v1(std::vector<std::vector<int> >& target_key_code_list, std::vector<std::vector<int> >& target_key_points, std::vector<int>& target_flag_lists, int only_no = -1);


// continue v1
std::vector<std::vector<int> > continue_v1_action_key_code_list;
std::vector<std::vector<int> > continue_v1_not_action_key_code_list;
std::vector<int> continue_v1_neutral_key_group_ids;
std::vector<int> continue_v1_action_flag;
std::vector<std::vector<int> > continue_v1_action_condition_list;
std::vector<std::vector<int> > continue_v1_condition_timeout_msec_list; // timeout per condition
std::map<uint64_t, uint64_t> continue_v1_last_run_timestamp; // first: (condition_list_no1 << 48) | (condition_list_no2 << 32) | condition_list_value;
std::vector<int> continue_v1_done_no;
std::map<uint64_t, size_t> continue_v1_id_timestamp; // key:id, value:timestamp

void dump_timestamp_map(std::map<uint64_t, uint64_t>& timestamp_map, std::string indent = "");
void dump_timestamp_map_map(std::map<uint64_t, std::map<uint64_t, uint64_t> >& timestamp_map, std::string indent = "");
bool runActionCalcContinue_v1(int only_no = -1);
std::map<uint64_t, uint64_t> set_continue_to_mix_timestamp(int index, uint64_t timestamp);


// vector dist compare v1
std::vector<std::vector<int> > vector_dists_compare_v1_action_key_code_list;
std::vector<std::vector<int> > vector_dists_compare_v1_not_action_key_code_list;
std::vector<std::vector<int> > vector_dists_compare_v1_key_points;
std::vector<int> vector_dists_compare_v1_neutral_key_group_ids;
std::vector<int> vector_dists_compare_v1_flag;

const int VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST_START = 0;
const int VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST_END = 1;
const int VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND_START = 2;
const int VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND_END = 3;
const int VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST_OFFSET = 4;
const int VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND_OFFSET = 5;
const int VECTOR_DIST_COMPARE_CONDITION_NO = 6;

const int VECTOR_DIST_COMPARE_V1_MAX = VECTOR_DIST_COMPARE_CONDITION_NO + 1;

const int VECTOR_DIST_COMPARE_DIVID_RATE = 1000;


//template <class TList, class T> float getLandmarkValueFromCode(TList& landamrks, int& landmarks_array_no_1st, int& landmarks_array_no_2nd);
//float get2CoordinateLandmarkFromCode(std::vector<int>& key_points);
bool runVectorDistCompareAction_v1(std::vector<std::vector<int> >& target_key_code_list, std::vector<std::vector<int> >& not_target_key_code_list, std::vector<std::vector<int> >& target_key_points, std::vector<int>& target_flag_lists, int only_no = -1);


// key code group
std::map<int, int> key_code_group_id_map; // key:key_code, value:group_id
std::map<int, std::vector<int> > group_id_key_code_map; // key:group_id, value:key_code
std::map<int, int> latest_running_key_code_group_map; // key:group_id, value:key_code
int latest_running_action_no; // action_no

const int GROUP_ID_HEIGHT_ARROW_KEY = 0;
const int GROUP_ID_WIDTH_ARROW_KEY = 1;
const int GROUP_ID_NUMBER_AND_ALFABBET = 2;


int get_group_id(int key_code);
int get_latest_key_code(int key_code);
void release_latest_running_key_code(int group_id);
int push_key_code(std::vector<int> key_codes, int index);
int push_or_pushing_latest_running_key_code(std::vector<int> key_codes, int index);
void update_latest_running_key_code(int key_code);
void clear_latest_running_key_code(int key_code);
void clear_latest_running_key_code_by_group_id(int group_id);
void dump_latest_running_key_code();

void verify_check_config_no();


/////////
// mix //
/////////
bool isGameStart = false;
bool isGameStarted = false;
//bool isLowFPSPoseStart = false;
//bool isLastLowFPSPoseStart = false;
bool isLowFPSPoseStart = true;
bool isLastLowFPSPoseStart = isLowFPSPoseStart;
bool is_debug_display = false;


// limit
//#define LIMIT_PLAY_TIME_SECOND  -1 // no limit
#define LIMIT_PLAY_TIME_SECOND  300
//#define LIMIT_PLAY_TIME_SECOND  30
//#define LIMIT_PLAY_TIME_SECOND  10

// owner site
#define OWNER_SITE_PATH         "owner-site.html"
#define OWNER_SITE_EXPECT_HASH  "0d1d72fd25439c76765bd14131f36994"
float owner_site_open_rate = 0.333;
//float owner_site_open_rate = 0.9; // test
#define HASH_SIZE 16

void show_owner_site_page(std::string path);
bool is_open_owner_site(float rate);
void show_owner_site_page();
std::string calc_md5_hash(std::string path);
bool check_key_file_and_hash(std::string expect_hash, std::string path);
void signal_func(int signal_no);
void regist_signal_func();
void verify_check();


////////////////////////////////////////////////////////////////////////////////
// modifyer add end
////////////////////////////////////////////////////////////////////////////////

float calcProcFPS(std::vector<uint64_t>& msec_list) {
  uint64_t sum_msec = 0;
  for (int i = 0; i < msec_list.size(); i++) {
    sum_msec += msec_list[i];
  }
  uint64_t average_msec = (double)sum_msec / msec_list.size();
  float fps = 1000.0 / average_msec;
  return fps;
}

auto getImageMP(cv::Mat& input_frame_cv) {
  auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGB, input_frame_cv.cols, input_frame_cv.rows,
        mediapipe::ImageFrame::kDefaultAlignmentBoundary);
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    input_frame_cv.copyTo(input_frame_mat);
  return input_frame;
}

absl::Status RunMPPGraphsAtCameraDevice(std::vector<mediapipe::CalculatorGraph*>& graphs) {
  LOG(INFO) << "Initialize the camera or load the video.";
  cv::VideoCapture capture;
  capture.open(camera_id);
  RET_CHECK(capture.isOpened());

  cv::VideoWriter writer;
  LOG(INFO) << "do camera setting.";
  cv::namedWindow(kWindowName, /*flags=WINDOW_AUTOSIZE*/ 1);
#if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
  capture.set(cv::CAP_PROP_FRAME_WIDTH, camera_width);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height);
  capture.set(cv::CAP_PROP_FPS, camera_fps);
  LOG(INFO) << "CAMERA FRAME RESOLUTION SETTTING: (" << camera_width << ", " << camera_height << ").";
  LOG(INFO) << "CAMERA FRAME SETTTING FPS: " << camera_fps << ".";
#endif
  
  int i = 0;
  
  LOG(INFO) << "Start running the calculator graph.";
  
#ifdef DETECT_MODE_POSE
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller_world_landmarks, graphs[i]->AddOutputStreamPoller(kOutputPoseWorldLandmarks));
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller_landmarks, graphs[i]->AddOutputStreamPoller(kOutputPoseLandmarks));
  mediapipe::OutputStreamPoller* poller_world_landmarks_ptr = &poller_world_landmarks;
#elif DETECT_MODE_HAND
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller_world_landmarks, graphs[i]->AddOutputStreamPoller(kOutputHandWorldLandmarks));
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller_landmarks, graphs[i]->AddOutputStreamPoller(kOutputHandLandmarks));
  mediapipe::OutputStreamPoller* poller_world_landmarks_ptr = &poller_world_landmarks;
#elif DETECT_MODE_FACE_MESH
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller_landmarks, graphs[i]->AddOutputStreamPoller(kOutputFaceMeshLandmarks));
  mediapipe::OutputStreamPoller* poller_world_landmarks_ptr = NULL;
#endif // DETECT_MODE_FACE_MESH
  
  MP_RETURN_IF_ERROR(graphs[i]->StartRun({}));
  
  std::thread threadDoMPP(threadDoMPPGraph, &poller_landmarks, poller_world_landmarks_ptr);
  threadDoMPP.detach();
  
  
  LOG(INFO) << "Start grabbing and processing frames.";
  grab_frames = true;
  
  
  std::thread threadDoKC(threadDoKeyCode);
  threadDoKC.detach();
  
  std::thread threadDoCKC(threadDoConsoleKeyCode);
  threadDoCKC.detach();
  
  
  LOG(INFO) << "graphs.size(): " << graphs.size();
  LOG(INFO) << "RunMPPGraph() while() start.";
  
  uint64_t frame_start_msec = -1;
  uint64_t frame_end_msec = -1;
  if (isLowFPSPoseStart == false) {
    camera_proc_fps_wait_msec = 1000.0 / camera_proc_fps;
  } else {
    camera_proc_fps_wait_msec = 1000.0 / camera_low_proc_fps;
  }
  LOG(INFO) << "RunMPPGraph() loop fps(" << camera_proc_fps << ") and wait msec(" << camera_proc_fps_wait_msec << ").";
  
  bool is_first_run = true;
  bool is_on_mask = false;
  
  uint64_t game_start_msec = -1;
  bool is_count_down_start_print = false;
  
  while (grab_frames) {
    if (frame_start_msec == -1) {
      frame_start_msec = getEpocTimeByChrono();
    }
    // Capture opencv camera or video frame.
    cv::Mat camera_frame_raw;
    capture >> camera_frame_raw;
    if (camera_frame_raw.empty()) {
      LOG(INFO) << "Ignore empty frames from camera.";
      continue;
    }
    if (is_first_run == true) {
      is_on_mask = checkIsOnMaskImage();
      is_first_run = false;
    }
    if (is_on_mask == true) {
      //setMaskImage(camera_frame_raw);
      camera_frame_raw = getROIImage(camera_frame_raw);
    }
    
    // 前の画像を描画する。ランドマークあれば描画する。
    mediapipe::NormalizedLandmarkList landmarks_2d;
    while (landmarks_2d_queue.empty() == false) {
      landmarks_2d = landmarks_2d_queue.front();
      landmarks_2d_queue.pop();
    }
    cv::Mat last_frame_mat;
    if (count_of_input_frame % 2 == 0) {
      last_frame_mat_0 = camera_frame_raw.clone();
      if (last_frame_mat_1.empty() == false) {
        last_frame_mat = last_frame_mat_1;
      }
    } else {
      last_frame_mat_1 = camera_frame_raw.clone();
      if (last_frame_mat_0.empty() == false) {
        last_frame_mat = last_frame_mat_0;
      }
    }
    if (last_frame_mat.empty() == false) {
      if (landmarks_2d.landmark_size() != 0) {
        if (display_face_mosaic_scale > 1) {
          drawFaceMosic(last_frame_mat, display_face_mosaic_scale, display_face_mosaic_width, display_face_mosaic_height, landmarks_2d);
        }
      }
      if (display_mosaic_scale > 1) {
        convertMosicImage(last_frame_mat, display_mosaic_scale);
      }
      if (is_display_mirror == true) {
        cv::flip(last_frame_mat, last_frame_mat, /*flipcode=HORIZONTAL*/ 1);
      }
      if (landmarks_2d.landmark_size() != 0) {
        mediapipe::NormalizedLandmarkList tmp_landmarks_2d = landmarks_2d;
#ifdef DETECT_MODE_POSE
        drawLandmarksPose(last_frame_mat, tmp_landmarks_2d);
#elif DETECT_MODE_HAND
        drawLandmarksHand(last_frame_mat, tmp_landmarks_2d);
#elif DETECT_MODE_FACE_MESH
        drawLandmarksFace(last_frame_mat, tmp_landmarks_2d);
#endif // DETECT_MODE_FACE_MESH
      }
      drawLineUpDownAndLeftRightThreshold(last_frame_mat, HELP_DEBUG_LINE_SIZE);
      //LOG(INFO) << "draw landamrk image(" << last_frame_mat.cols << ", " << last_frame_mat.rows << ").";
      // FPSやmsecを描画
      if (is_debug_display == true) {
        drawProcInfo(last_frame_mat);
      }
      drawMakerInfo(last_frame_mat);
      cv::imshow(kWindowName, last_frame_mat);
      int wait_msec = 1;
      const int pressed_key = cv::waitKey(wait_msec);
    }
    
    cv::Mat camera_frame;
    cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGB);
    
    last_input_timestamp = getEpocTimeByChrono();
    
    // Wrap Mat into an ImageFrame.
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
        mediapipe::ImageFrame::kDefaultAlignmentBoundary);
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);
    
    // Send image packet into the graph.
    size_t frame_timestamp_us =
        (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
    if (is_input_frame == false) {
      LOG(INFO) << "input frame image size: (" << input_frame_mat.size() << ")";
    }
    
    
    cv::Mat output_frame_mat;
    for (int i = 0; i < graphs.size(); i++) {
      // divide image
      // TODO
      
      MP_RETURN_IF_ERROR(graphs[i]->AddPacketToInputStream(
          kInputStream, mediapipe::Adopt(input_frame.release())
                            .At(mediapipe::Timestamp(frame_timestamp_us))));
    }
    
    //LOG(INFO) << "frame input end. count: " << count_of_input_frame;
    
    if (is_input_frame == false) {
      //LOG(INFO) << "output frame image size: (" << output_frame_mat.size() << ")";
      is_input_frame = true;
    }
    count_of_input_frame++;
    
    if (isGameStart == true) {
      if (game_start_msec == -1) {
        game_start_msec = getEpocTimeByChrono() + game_start_wait_msec;
      } else if (game_start_msec < getEpocTimeByChrono()) {
        isGameStarted = true;
      }
    } else {
      game_start_msec = -1;
      isGameStarted = false;
    }
    // count down
    if (game_start_msec != -1) {
      if (is_count_down_start_print == false) {
        int before_wait_msec = game_start_msec - getEpocTimeByChrono();
        if (before_wait_msec > 0) {
          int before_wait_sec = before_wait_msec / 1000;
          printf("start waiting... %d sec, wait %d msec.\r\n", before_wait_sec, before_wait_msec);
        } else {
          printf("start waiting... 0 sec, wait 0 msec.\r\n");
          printf("starting.\r\n");
          is_count_down_start_print = true;
        }
      }
    } else {
      if (is_count_down_start_print == true) {
        is_count_down_start_print = false;
      }
    }
    
    frame_end_msec = getEpocTimeByChrono();
    int frame_proc_diff_msec = frame_end_msec - frame_start_msec;
    int wait_msec = camera_proc_fps_wait_msec - frame_proc_diff_msec;
    //LOG(INFO) << "wait(" << wait_msec << ") diff(" << frame_proc_diff_msec << "), frame_end_msec("<< frame_end_msec << "), frame_start_msec("<< frame_start_msec << ")";
    if (wait_msec > 0) {
      //LOG(INFO) << "RunMPPGraph() wait(" << wait_msec << ") thread loop.";
      Sleep(wait_msec);
    }
    
    frame_start_msec = -1;
  }
  
  LOG(INFO) << "RunMPPGraph() while() end.";
  
  // join()前に、poseが検出できるフレームを入力か、タイムアウト等しないとプログラムが停止しない。poller.Next()で待ってしまう。要修正
  
  //threadDoMPP.join();

  LOG(INFO) << "Shutting down.";
  if (writer.isOpened()) writer.release();
  absl::Status status_last;
  for (int i = 0; i < graphs.size(); i++) {
    MP_RETURN_IF_ERROR(graphs[i]->CloseInputStream(kInputStream));
    //MP_RETURN_IF_ERROR(graphs[i]->WaitUntilDone());
    status_last = graphs[i]->WaitUntilDone();
  }
  return status_last;
}

//------------------------------------------------------------------------------
// thread do MPP Graph start
//------------------------------------------------------------------------------
void threadDoMPPGraph(mediapipe::OutputStreamPoller* poller_2d_landmarks, mediapipe::OutputStreamPoller *poller_3d_landmarks) {
  LOG(INFO) << "threadDoMPPGraph() start.";
  
  LOG(INFO) << "threadDoMPPGraph() wait input frame start.";
  
  while (is_input_frame == false) {
    Sleep(wait_sleep_time_mili_second);
  }
  
  LOG(INFO) << "threadDoMPPGraph() wait input frame end.";
  
  int max_game_loop_count = -1;
  //max_game_loop_count = 20;
  int game_loop_count = 0;
  
  uint64_t limit_end_timestamp_us = getEpocTimeByChrono();
  if (LIMIT_PLAY_TIME_SECOND == -1) {
    limit_end_timestamp_us = -1;
  } else {
    limit_end_timestamp_us += LIMIT_PLAY_TIME_SECOND * 1000;
  }
  
  uint64_t frame_timestamp_us = -1;
  while (grab_frames) {
    // limit show ad disable
    //if (limit_end_timestamp_us != -1) {
    //  uint64_t now_timestamp_us = getEpocTimeByChrono();
    //  if (now_timestamp_us > limit_end_timestamp_us) {
    //    LOG(INFO) << "threadDoMPPGraph() reach limit, show owner site.";
    //    //LOG(ERROR) << "threadDoMPPGraph() show_owner_site_page() start.";
    //    //show_owner_site_page();
    //    // force show owner site
    //    std::string path = OWNER_SITE_PATH;
    //    show_owner_site_page(path);
    //    //LOG(ERROR) << "threadDoMPPGraph() show_owner_site_page() end.";
    //    limit_end_timestamp_us += LIMIT_PLAY_TIME_SECOND * 1000;
    //  }
    //}
    
    uint64_t output_timestamp = 0;
    uint64_t last_in_timestamp = 0; // last_input_timestampが上書きされないようにこのスレッドで保持
    //LOG(INFO) << "run packet 2d landmarks.";
    mediapipe::Packet packet_2d_landmarks;
    // TODO：ポーズが検出できないと永続的に停止する：要修正（終了時に、このループを抜けるよう、ダミーポーズ画像をinputするようにしておく？）
    if (poller_2d_landmarks->Next(&packet_2d_landmarks)) {
      //LOG(INFO) << "get packet 2d landmarks.";
      
      last_in_timestamp = last_input_timestamp;
      output_timestamp = getEpocTimeByChrono();
      uint64_t last_proc_msec = output_timestamp - last_in_timestamp;
      proc_msec_list.push_back(last_proc_msec);
      while(proc_fps_average_size < proc_msec_list.size()) {
        proc_msec_list.erase(proc_msec_list.begin());
      }
      LOG(INFO) << "proc(msec): " << last_proc_msec;
      last_proc_fps = calcProcFPS(proc_msec_list);
      LOG(INFO) << "last proc fps: " << last_proc_fps;
      
#ifdef DETECT_MODE_POSE
      auto& output_2d_landmarks = packet_2d_landmarks.Get<mediapipe::NormalizedLandmarkList>();
#elif DETECT_MODE_HAND
      auto& output_2d_landmarks_list = packet_2d_landmarks.Get<std::vector<mediapipe::NormalizedLandmarkList> >();
      LOG(INFO) << "get landmarks size: " << output_2d_landmarks_list.size();
      if (output_2d_landmarks_list.size() > 1) {
        LOG(INFO) << "get landmarks size is over 1(" << output_2d_landmarks_list.size() << "), therefor use first data.";
      }
      mediapipe::NormalizedLandmarkList output_2d_landmarks = output_2d_landmarks_list[0];
#elif DETECT_MODE_FACE_MESH
      auto& output_2d_landmarks_list = packet_2d_landmarks.Get<std::vector<mediapipe::NormalizedLandmarkList> >();
      LOG(INFO) << "get landmarks size: " << output_2d_landmarks_list.size();
      if (output_2d_landmarks_list.size() > 1) {
        LOG(INFO) << "get landmarks size is over 1(" << output_2d_landmarks_list.size() << "), therefor use first data.";
      }
      mediapipe::NormalizedLandmarkList output_2d_landmarks = output_2d_landmarks_list[0];
#endif // DETECT_MODE_FACE_MESH
      
      //LOG(INFO) << "landmark size: " << output_2d_landmarks.landmark_size() << ", (" << output_2d_landmarks.landmark(0).x() << ", " << output_2d_landmarks.landmark(0).y() << ", " << output_2d_landmarks.landmark(0).z() << ")";
      
      mediapipe::NormalizedLandmarkList add_landmarks(output_2d_landmarks);
      if (is_display_mirror == true) {
        // side reverse
        convert_x_slide_landmark<mediapipe::NormalizedLandmarkList, mediapipe::NormalizedLandmark>(add_landmarks);
      }
#ifdef DETECT_MODE_POSE
      add_and_calc_landmark_pose<mediapipe::NormalizedLandmarkList, mediapipe::NormalizedLandmark>(output_2d_landmarks, add_landmarks);
#elif DETECT_MODE_HAND
      add_and_calc_landmark_hand<mediapipe::NormalizedLandmarkList, mediapipe::NormalizedLandmark>(output_2d_landmarks, add_landmarks);
#elif DETECT_MODE_FACE_MESH
      add_and_calc_landmark_face_mesh<mediapipe::NormalizedLandmarkList, mediapipe::NormalizedLandmark>(output_2d_landmarks, add_landmarks);
#endif // DETECT_MODE_FACE_MESH
      last_2nd_target_vec_2d = last_target_vec_2d;
      last_target_vec_2d = add_landmarks;
      
      if (frame_timestamp_us == -1) {
        frame_timestamp_us = getEpocTimeByChrono();
      }
      
      //logout_landmarks(output_2d_landmarks);
      //logout_landmarks(add_landmarks);
      //logout_landmark(add_landmarks, logout_number_specify);
    }
    if (poller_3d_landmarks != NULL) {
      //LOG(INFO) << "run packet 3d landmarks.";
      mediapipe::Packet packet_3d_landmarks;
      // TODO：ポーズが検出できないと永続的に停止する：要修正（終了時に、このループを抜けるよう、ダミーポーズ画像をinputするようにしておく？）
      if (poller_3d_landmarks->Next(&packet_3d_landmarks)) {
        //LOG(INFO) << "get packet 3d landmarks.";
#ifdef DETECT_MODE_POSE
        auto& output_3d_landmarks = packet_3d_landmarks.Get<mediapipe::LandmarkList>();
#elif DETECT_MODE_HAND
        auto& output_3d_landmarks_list = packet_3d_landmarks.Get<std::vector<mediapipe::LandmarkList> >();
        LOG(INFO) << "get landmarks size: " << output_3d_landmarks_list.size();
        if (output_3d_landmarks_list.size() > 1) {
          LOG(INFO) << "get landmarks size is over 1(" << output_3d_landmarks_list.size() << "), therefor use first data.";
        }
        mediapipe::LandmarkList output_3d_landmarks = output_3d_landmarks_list[0];
#else // DETECT_MODE_FACE_MESH
        // dummy code(for compile)
        mediapipe::LandmarkList output_3d_landmarks;
#endif // DETECT_MODE_FACE_MESH
        
        //LOG(INFO) << "world landmark size: " << output_3d_landmarks.landmark_size() << ", (" << output_3d_landmarks.landmark(0).x() << ", " << output_3d_landmarks.landmark(0).y() << ", " << output_3d_landmarks.landmark(0).z() << ")";
        
        mediapipe::LandmarkList output_3d_landmarks_x_reverse = output_3d_landmarks;
        if (is_display_mirror == true) {
          // side reverse
          convert_x_reverse_landmark<mediapipe::LandmarkList, mediapipe::Landmark>(output_3d_landmarks_x_reverse);
        }
        
        mediapipe::LandmarkList add_world_landmarks(output_3d_landmarks_x_reverse);
#ifdef DETECT_MODE_POSE
        add_and_calc_landmark_pose<mediapipe::LandmarkList, mediapipe::Landmark>(output_3d_landmarks_x_reverse, add_world_landmarks);
#elif DETECT_MODE_HAND
        add_and_calc_landmark_hand<mediapipe::LandmarkList, mediapipe::Landmark>(output_3d_landmarks_x_reverse, add_world_landmarks);
#elif DETECT_MODE_FACE_MESH
        //add_and_calc_landmark_face_mesh<mediapipe::LandmarkList, mediapipe::Landmark>(output_3d_landmarks_x_reverse, add_world_landmarks);
#endif // DETECT_MODE_FACE_MESH
        last_2nd_target_vec = last_target_vec;
        last_target_vec = add_world_landmarks;
        
        mediapipe::LandmarkList vector_world_landmarks;
#ifdef DETECT_MODE_POSE
        calc_world_vector_from_parent<mediapipe::LandmarkList, mediapipe::Landmark>(add_world_landmarks, pose_vector_vec, vector_world_landmarks);
        calc_angles(vector_world_landmarks, pose_vector_vec);
#elif DETECT_MODE_HAND
        calc_world_vector_from_parent<mediapipe::LandmarkList, mediapipe::Landmark>(add_world_landmarks, hand_vector_vec, vector_world_landmarks);
        calc_angles(vector_world_landmarks, hand_vector_vec);
#elif DETECT_MODE_FACE_MESH
        //calc_world_vector_from_parent<mediapipe::LandmarkList, mediapipe::Landmark>(add_world_landmarks, face_mesh_vector_vec, vector_world_landmarks);
        //calc_angles(vector_world_landmarks, face_mesh_vector_vec);
#endif // DETECT_MODE_FACE_MESH
        last_2nd_target_vec_angle = last_target_vec_angle;
        last_target_vec_angle = vector_world_landmarks;
        
        mediapipe::LandmarkList camera_diff_angle;
        calc_camera_diff_angle(vector_world_landmarks, camera_diff_angle);
        last_2nd_target_camera_diff_angle = last_target_camera_diff_angle;
        last_target_camera_diff_angle = camera_diff_angle;
        
        if (frame_timestamp_us == -1) {
          frame_timestamp_us = getEpocTimeByChrono();
        }
        
        // last_target_vec
        //logout_world_landmarks(output_3d_landmarks);
        //logout_world_landmark(output_3d_landmarks, logout_number_specify);
        //logout_world_landmarks(output_3d_landmarks_x_reverse);
        //logout_world_landmark(output_3d_landmarks_x_reverse, logout_number_specify);
        //logout_world_landmarks(add_world_landmarks);
        //logout_world_landmark(add_world_landmarks, logout_number_specify);
        // last_target_vec_angle
        //logout_world_landmarks(vector_world_landmarks);
        //logout_world_landmark(vector_world_landmarks, logout_number_specify);
        // last_target_camera_diff_angle
        //logout_world_landmarks(camera_diff_angle);
        //logout_world_landmark(camera_diff_angle, logout_number_specify);
      }
    } else {
      // 2dから3dデータを生成
      mediapipe::LandmarkList add_world_landmarks;
#ifdef DETECT_MODE_FACE_MESH
      calc_from_landmark_to_world_landmark_face_mesh(last_target_vec_2d, add_world_landmarks);
      last_2nd_target_vec = last_target_vec;
      last_target_vec = add_world_landmarks;
      
      mediapipe::LandmarkList vector_world_landmarks;
      calc_world_vector_from_parent<mediapipe::LandmarkList, mediapipe::Landmark>(add_world_landmarks, face_mesh_vector_vec, vector_world_landmarks);
      calc_angles(vector_world_landmarks, face_mesh_vector_vec);
      
      last_2nd_target_vec_angle = last_target_vec_angle;
      last_target_vec_angle = vector_world_landmarks;
      
      //mediapipe::LandmarkList camera_diff_angle;
      //calc_camera_diff_angle(vector_world_landmarks, camera_diff_angle);
      //last_2nd_target_camera_diff_angle = last_target_camera_diff_angle;
      //last_target_camera_diff_angle = camera_diff_angle;
      
      if (frame_timestamp_us == -1) {
        frame_timestamp_us = getEpocTimeByChrono();
      }
#endif // DETECT_MODE_FACE_MESH
      // last_target_vec
      //logout_world_landmarks(add_world_landmarks);
      //logout_world_landmark(add_world_landmarks, logout_number_specify);
      // last_target_vec_angle
      //logout_world_landmarks(vector_world_landmarks);
      //logout_world_landmark(vector_world_landmarks, logout_number_specify);
      // last_target_camera_diff_angle
      //logout_world_landmarks(camera_diff_angle);
      //logout_world_landmark(camera_diff_angle, logout_number_specify);
    }
    if (last_target_vec_2d.landmark_size() != 0 && last_target_vec.landmark_size() != 0) {
#ifdef DETECT_MODE_POSE
      calc_and_set_world_center_body_landmark(last_target_vec_2d, last_target_vec);
#elif DETECT_MODE_HAND
      calc_and_set_world_center_hand_landmark(last_target_vec_2d, last_target_vec);
#elif DETECT_MODE_FACE_MESH
      calc_and_set_world_center_face_mesh_landmark(last_target_vec_2d, last_target_vec);
#endif // DETECT_MODE_FACE_MESH
      //logout_world_landmarks(last_target_vec);
      //logout_world_landmark(last_target_vec, logout_number_specify);
      //logout_world_landmarks(last_target_vec_angle);
      //logout_world_landmark(last_target_vec_angle, logout_number_specify);
      
      add_frame_diff_landmarks(frame_timestamp_us, last_target_vec, last_target_vec_2d, last_target_vec_angle, frame_diff_landmarks, frame_diff_2d_landmarks, frame_diff_vec_landmarks, frame_diff_timestamps);
    }
    if (last_target_vec_2d.landmark_size() != 0) {
      landmarks_2d_queue.push(last_target_vec_2d);
    }
    logout_coordinate();
    
    if (isGameStarted == true) {
      std::unique_lock<std::mutex> lock_key_code(mutex_key_code);
      queue_key_code.clear(); // all reset
      
      if (is_enable_change_x_front_reverse == true) {
        check_change_x_front_reverse();
      }
      
      // no use
      //if (key_arrow_down_up_threshold.empty() == false) {
      //  runCalcPoseCoordinateAndAction_v1(last_target_vec_2d.landmark(POSE_NO_CENTER_BODY));
      //}
      if (action_key_code_list.empty() == false) {
        runCalcPoseCoordinateAndAction_v2(action_key_code_list, target_key_points, threshold_condition_list, action_flag_list);
      }
      if (coord_v3_action_key_code_list.empty() == false) {
        runCalcPoseCoordinateAndAction_v3(coord_v3_action_key_code_list, coord_v3_target_key_points, coord_v3_action_flag_list);
      }
      if (sum_action_key_code_list.empty() == false) {
        runSumAndCalcPoseVectorAndAction_v1(sum_action_key_code_list, sum_target_key_points, sum_threshold_condition_list, sum_vector_list, sum_action_flag_list);
      }
      if (frame_diff_v1_action_key_code_list.empty() == false) {
        runActionCalcDiff_v1();
      }
      if (landmark_2point_dist_compare_v1_action_key_code_list.empty() == false) {
        runLandmark2PointDistCompareAction_v1(landmark_2point_dist_compare_v1_action_key_code_list, landmark_2point_dist_compare_v1_key_points, landmark_2point_dist_compare_v1_flag);
      }
      if (mark_compare_v1_action_key_code_list.empty() == false) {
        runActionCalcMarkCompare_v1(mark_compare_v1_action_key_code_list, mark_compare_v1_key_points, mark_compare_v1_action_flag);
      }
      if (vector_dists_compare_v1_action_key_code_list.empty() == false) {
        runVectorDistCompareAction_v1(vector_dists_compare_v1_action_key_code_list, vector_dists_compare_v1_not_action_key_code_list, vector_dists_compare_v1_key_points, vector_dists_compare_v1_flag);
      }
      if (mix_v1_action_key_code_list.empty() == false) {
        runActionCalcMix_v1(mix_v1_last_run_timestamp);
      }
      if (continue_v1_action_key_code_list.empty() == false) {
        runActionCalcContinue_v1();
      }
      
      if (max_game_loop_count != -1 && game_loop_count > max_game_loop_count) {
        LOG(INFO) << "game_loop_count(" << game_loop_count << ") over max_game_loop_count(" << max_game_loop_count << ").";
        break;
      }
      game_loop_count++;
    }
    
    uint64_t done_timestamp = getEpocTimeByChrono();
    uint64_t last_done_msec = done_timestamp - last_in_timestamp;
    LOG(INFO) << "frame and action done msec: " << last_done_msec;
    done_msec_list.push_back(last_done_msec);
    while(proc_fps_average_size < done_msec_list.size()) {
      done_msec_list.erase(done_msec_list.begin());
    }
    last_done_fps = calcProcFPS(done_msec_list);
    LOG(INFO) << "frame and action done fps: " << last_done_fps;
    
    frame_timestamp_us = -1;
  }
  
  LOG(INFO) << "threadDoMPPGraph() end.";
}


//------------------------------------------------------------------------------
// thread do MPP Graph end
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// thread do key code start
//------------------------------------------------------------------------------
void threadDoKeyCode() {
  LOG(INFO) << "threadDoKeyCode() start.";
  
  uint64_t frame_timestamp_us = -1;
  uint64_t end_frame_timestamp_us = -1;
  while (grab_frames) {
    if (true) {
      // mutex lock
      std::unique_lock<std::mutex> lock_key_code(mutex_key_code);
      //if (queue_key_code.size() == 0) {
      //  LOG(INFO) << "threadDoKeyCode() wait start.";
      //  condition_key_code.wait(lock_key_code); // TODO wait need signal
      //  LOG(INFO) << "threadDoKeyCode() wait end.";
      //}
      
      frame_timestamp_us = getEpocTimeByChrono();
      
      for (int i = 0; i < queue_key_code.size(); i++) {
        int key_code = queue_key_code[i].first;
        int flag = queue_key_code[i].second;
        
        //LOG(INFO) << "threadDoKeyCode() event run.";
        doMultiKeyEvent(key_code, key_default_flag);
        if (!(flag & FD_V1_FLAG_NO_UP_AFTER_DOWN)) {
          //LOG(INFO) << "threadDoKeyCode() event up run.";
          // TODO mouse and keyboard and sleep: -> doMultiEvent()
          doMultiKeyEvent(key_code, key_default_flag | KEYEVENTF_KEYUP);
        }
      }
      // mutex unlock
    }
    
    end_frame_timestamp_us = getEpocTimeByChrono();
    int diff_timestamp = end_frame_timestamp_us - frame_timestamp_us;
    int sleep_timestamp = key_code_wait_interval_msec - diff_timestamp; // TODO modify from key_code_wait_interval_msec to fps
    if (sleep_timestamp > 0) {
      //LOG(INFO) << "threadDoKeyCode() sleep(" << sleep_timestamp << ").";
      Sleep(sleep_timestamp);
    }
    
    frame_timestamp_us = -1;
    end_frame_timestamp_us = -1;
  }
  
  LOG(INFO) << "threadDoKeyCode() end.";
}


//------------------------------------------------------------------------------
// thread do key code end
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// thread do console key code start
//------------------------------------------------------------------------------
void threadDoConsoleKeyCode() {
  LOG(INFO) << "threadDoConsoleKeyCode() start.";
  
  LOG(INFO) << "threadDoConsoleKeyCode() wait input frame start.";
  
  while (is_input_frame == false) {
    Sleep(wait_sleep_time_mili_second);
  }
  
  LOG(INFO) << "threadDoConsoleKeyCode() wait input frame end.";
  
  if (is_auto_start == true) {
    isGameStart = true;
    isLowFPSPoseStart = false;
    camera_proc_fps_wait_msec = (int)(1000.0 / camera_proc_fps);
  }
  
  printf("%s", usage_context.c_str());
  
  while (grab_frames) {
    printf("Please input command key code: ");
    //int key_code = std::cin.get();
    int key_code = getchar();
    //int key_code = getc(std::cin);
    std::string key_code_str = getDumpHexFromInt(key_code);
    switch (key_code) {
    default:
      printf("input key code(%c:%d, 0x%s) is unknown, skip\r\n", key_code, key_code, key_code_str.c_str());
      break;
    case 'q': // 0x51
      printf("input q, quit.\r\n");
      grab_frames = false;
      break;
    case 's': // 0x53
      printf("input s, start.\r\n");
      isGameStart = true;
      isLowFPSPoseStart = false;
      camera_proc_fps_wait_msec = (int)(1000.0 / camera_proc_fps);
      break;
    case 'p': // 0x50
      printf("input p, pause.\r\n");
      isGameStart = false;
      isGameStarted = false;
      isLowFPSPoseStart = true;
      camera_proc_fps_wait_msec = (int)(1000.0 / camera_low_proc_fps);
      break;
    case 'd': // 0x44
      printf("input d, change debug display on/off.\r\n");
      if (is_debug_display == true) {
        is_debug_display = false;
      } else {
        is_debug_display = true;
      }
    case 'c': // 0x43
      printf("input c, change template config.\r\n");
      {
        // なぜか0x10が入力されるので、0x10だとスキップして再読み込みするようにする
        // TODO ★
        printf("please input template config number(0 - %d).\r\n", enable_TemplateConfig_index);
        int input_template_config_number = getchar() - 0x30;
        if (input_template_config_number < 0) {
          printf("input template config number(%d) is under enable number(0), therefor skip and do nothing.\r\n", input_template_config_number);
          break;
        }
        if (input_template_config_number > enable_TemplateConfig_index) {
          printf("input template config number(%d) is over enable number(%d), therefor skip and do nothing.\r\n", input_template_config_number, enable_TemplateConfig_index);
          break;
        }
        target_TemplateConfig_index = input_template_config_number;
        changeConfigV1_TemplateConfig(config_path, target_TemplateConfig_index);
        // verify check
        verify_check();
        printf("success change template config number(%d).\r\n", enable_TemplateConfig_index);
      }
      break;
    case 0x0A: // enter
      //printf("input key code(%d, 0x%s: enter), skip.\r\n", key_code, key_code_str.c_str());
      printf("\r\n");
      break;
    }
  }
  
  LOG(INFO) << "threadDoConsoleKeyCode() end.";
}


//------------------------------------------------------------------------------
// thread do console key code end
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// coordinate start
//------------------------------------------------------------------------------
template <class TList, class T> void set_and_calc_average(const TList& landmarks, std::vector<int> target_list, T& average_landmark) {
  float x = (float)0.0;
  float y = (float)0.0;
  float z = (float)0.0;
  float visibility = (float)0.0;
  float presence = (float)0.0;
  for (int i = 0; i < target_list.size(); i++) {
    T landmark = landmarks.landmark(target_list[i]);
    x += landmark.x();
    y += landmark.y();
    z += landmark.z();
    visibility += landmark.visibility();
    presence += landmark.presence();
  }
  average_landmark.set_x((float)(x / target_list.size()));
  average_landmark.set_y((float)(y / target_list.size()));
  average_landmark.set_z((float)(z / target_list.size()));
  average_landmark.set_visibility((float)(visibility / target_list.size()));
  average_landmark.set_presence((float)(presence / target_list.size()));
}

// template:
//   TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
//   T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void convert_x_slide_landmark(TList& landmarks) {
  for (int i = 0; i < landmarks.landmark_size(); i++) {
    T* landmark = landmarks.mutable_landmark(i);
    landmark->set_x(1.0 - landmark->x());
  }
}

// template:
//   TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
//   T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void convert_x_reverse_landmark(TList& landmarks) {
  for (int i = 0; i < landmarks.landmark_size(); i++) {
    T* landmark = landmarks.mutable_landmark(i);
    landmark->set_x(landmark->x() * -1.0);
  }
}

// template:
//   TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
//   T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void add_and_calc_landmark_pose(const TList& landmarks, TList& added_landmarks) {
  // center
  //   body
  std::vector<int> center_body_points;
  center_body_points.push_back(POSE_NO_LEFT_SHOULDER);
  center_body_points.push_back(POSE_NO_RIGHT_SHOULDER);
  center_body_points.push_back(POSE_NO_LEFT_HIP);
  center_body_points.push_back(POSE_NO_RIGHT_HIP);
  T* center_body_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, center_body_points, *center_body_landmark);
  //   shoulder
  std::vector<int> center_shoulder_points;
  center_shoulder_points.push_back(POSE_NO_LEFT_SHOULDER);
  center_shoulder_points.push_back(POSE_NO_RIGHT_SHOULDER);
  T* center_shoulder_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, center_shoulder_points, *center_shoulder_landmark);
  //   hip
  std::vector<int> center_hip_points;
  center_hip_points.push_back(POSE_NO_LEFT_HIP);
  center_hip_points.push_back(POSE_NO_RIGHT_HIP);
  T* center_hip_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, center_hip_points, *center_hip_landmark);
  // left
  //   body
  std::vector<int> left_body_points;
  left_body_points.push_back(POSE_NO_LEFT_SHOULDER);
  left_body_points.push_back(POSE_NO_LEFT_HIP);
  T* left_body_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, left_body_points, *left_body_landmark);
  // right
  //   body
  std::vector<int> right_body_points;
  right_body_points.push_back(POSE_NO_RIGHT_SHOULDER);
  right_body_points.push_back(POSE_NO_RIGHT_HIP);
  T* right_body_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, right_body_points, *right_body_landmark);
  //T left_body_landmark = calc_vector_from_parent_index_reverse<TList, T>(added_landmarks, POSE_NO_LEFT_BODY, pose_vector_vec[POSE_NO_LEFT_BODY]);
//LOG(INFO) << "front body: start";
  // TODO change from left_body_landmark to vector★★
  //T left_body_vector = calc_vector_from_parent_index<TList, T>(added_landmarks, POSE_NO_LEFT_BODY, pose_vector_vec[POSE_NO_LEFT_BODY]);
  //T ajust_body_vec_landmark = get_line_turn_ajust_x(left_body_vector);
  T right_body_vector = calc_vector_from_parent_index<TList, T>(added_landmarks, POSE_NO_RIGHT_BODY, pose_vector_vec[POSE_NO_RIGHT_BODY]);
  T ajust_body_vec_landmark = get_line_turn_ajust_x(right_body_vector);
//LOG(INFO) << "ajust_body_vec_landmark";
//logout_t_landmark(ajust_body_vec_landmark);
  std::vector<float> front_body_ajust_array;
  y_turn_coord((float)90.0, front_body_ajust_array); // TODO ★ from y:90.0 to center_shoulder(body)_line
//LOG(INFO) << "front_body_ajust_array";
//logout_t_array(front_body_ajust_array);
  std::vector<float> ajust_body_vector = convert_landmark_to_vector(ajust_body_vec_landmark);
  std::vector<float> front_body_ajust_vector = calc_dot_vector3x1_vector3x3(ajust_body_vector, front_body_ajust_array);
//LOG(INFO) << "front_body_ajust_vector";
//logout_t_array(front_body_ajust_vector);
  
  T left_body_vec_landmark = calc_vector_from_parent_index_reverse<TList, T>(added_landmarks, POSE_NO_LEFT_BODY, pose_vector_vec[POSE_NO_LEFT_BODY]);
  std::vector<float> left_body_vec_vector = convert_landmark_to_vector(left_body_vec_landmark);
//LOG(INFO) << "left_body_vec_landmark";
//logout_t_landmark(left_body_vec_landmark);
//logout_t_array(left_body_vec_vector);
  
  std::vector<float> front_body_vector = calc_turn_vec_x(front_body_ajust_vector, left_body_vec_vector);
//LOG(INFO) << "front_body_vector";
//logout_t_array(front_body_vector);
  //T center_body_landmark = calc_vector_from_parent_index_reverse<TList, T>(added_landmarks, POSE_NO_CENTER_BODY, pose_vector_vec[POSE_NO_CENTER_BODY]);
  std::vector<float> center_body_landmark_vector = convert_landmark_to_vector(*center_body_landmark);
//LOG(INFO) << "center_body_landmark_vector";
//logout_t_array(center_body_landmark_vector);
  std::vector<float> front_body_coord = add_vector(front_body_vector, center_body_landmark_vector);
  T* front_body_landmark = added_landmarks.add_landmark();
  *front_body_landmark = convert_vector_to_landmark<T>(front_body_coord);
//LOG(INFO) << "front_body_landmark";
//logout_t_landmark(*front_body_landmark);
  
  //     shoulder
  //T center_shoulder_landmark = calc_vector_from_parent_index_reverse<TList, T>(added_landmarks, POSE_NO_CENTER_SHOULDER, pose_vector_vec[POSE_NO_CENTER_SHOULDER]);
//LOG(INFO) << "front shoulder: start";
  //T left_shoulder_vector = calc_vector_from_parent_index<TList, T>(added_landmarks, POSE_NO_LEFT_SHOULDER, pose_vector_vec[POSE_NO_LEFT_SHOULDER]);
  //T ajust_shoulder_vec_landmark = get_line_turn_ajust_x(left_body_vector);
  T right_shoulder_vector = calc_vector_from_parent_index<TList, T>(added_landmarks, POSE_NO_RIGHT_SHOULDER, pose_vector_vec[POSE_NO_RIGHT_SHOULDER]);
  T ajust_shoulder_vec_landmark = get_line_turn_ajust_x(right_shoulder_vector);
//LOG(INFO) << "ajust_shoulder_vec_landmark";
//logout_t_landmark(ajust_shoulder_vec_landmark);
  std::vector<float> front_shoulder_ajust_array;
  y_turn_coord((float)90.0, front_shoulder_ajust_array); // TODO from y:90.0 to center_shoulder(shoulder)_line
//LOG(INFO) << "front_shoulder_ajust_array";
//logout_t_array(front_shoulder_ajust_array);
  std::vector<float> ajust_shoulder_vector = convert_landmark_to_vector(ajust_shoulder_vec_landmark);
  std::vector<float> front_shoulder_ajust_vector = calc_dot_vector3x1_vector3x3(ajust_shoulder_vector, front_shoulder_ajust_array);
//LOG(INFO) << "front_shoulder_ajust_vector";
//logout_t_array(front_shoulder_ajust_vector);
  
  T left_shoulder_vec_landmark = calc_vector_from_parent_index_reverse<TList, T>(added_landmarks, POSE_NO_LEFT_BODY, pose_vector_vec[POSE_NO_LEFT_BODY]);
  std::vector<float> left_shoulder_vec_vector = convert_landmark_to_vector(left_shoulder_vec_landmark);
//LOG(INFO) << "left_shoulder_vec_landmark";
//logout_t_landmark(left_shoulder_vec_landmark);
//logout_t_array(left_shoulder_vec_vector);
  
  std::vector<float> front_shoulder_vector = calc_turn_vec_x(front_shoulder_ajust_vector, left_shoulder_vec_vector);
//LOG(INFO) << "front_shoulder_vector";
//logout_t_array(front_shoulder_vector);
  //T center_shoulder_landmark = calc_vector_from_parent_index_reverse<TList, T>(added_landmarks, POSE_NO_CENTER_BODY, pose_vector_vec[POSE_NO_CENTER_BODY]);
  std::vector<float> center_shoulder_landmark_vector = convert_landmark_to_vector(*center_shoulder_landmark);
//LOG(INFO) << "center_shoulder_landmark_vector";
//logout_t_array(center_shoulder_landmark_vector);
  std::vector<float> front_shoulder_coord = add_vector(front_shoulder_vector, center_shoulder_landmark_vector);
  T* front_shoulder_landmark = added_landmarks.add_landmark();
  *front_shoulder_landmark = convert_vector_to_landmark<T>(front_shoulder_coord);
//LOG(INFO) << "front_shoulder_landmark";
//logout_t_landmark(*front_shoulder_landmark);
  
  //     hip
  //T center_hip_landmark = calc_vector_from_parent_index_reverse<TList, T>(added_landmarks, POSE_NO_CENTER_HIP, pose_vector_vec[POSE_NO_CENTER_HIP]);
//LOG(INFO) << "front hip: start";
  //T left_hip_vector = calc_vector_from_parent_index<TList, T>(added_landmarks, POSE_NO_LEFT_HIP, pose_vector_vec[POSE_NO_LEFT_HIP]);
  //T ajust_hip_vec_landmark = get_line_turn_ajust_x(left_body_vector);
  T right_hip_vector = calc_vector_from_parent_index<TList, T>(added_landmarks, POSE_NO_RIGHT_HIP, pose_vector_vec[POSE_NO_RIGHT_HIP]);
  T ajust_hip_vec_landmark = get_line_turn_ajust_x(right_hip_vector);
//LOG(INFO) << "ajust_hip_vec_landmark";
//logout_t_landmark(ajust_hip_vec_landmark);
  std::vector<float> front_hip_ajust_array;
  y_turn_coord((float)90.0, front_hip_ajust_array); // TODO from y:90.0 to center_hip(hip)_line
//LOG(INFO) << "front_hip_ajust_array";
//logout_t_array(front_hip_ajust_array);
  std::vector<float> ajust_hip_vector = convert_landmark_to_vector(ajust_hip_vec_landmark);
  std::vector<float> front_hip_ajust_vector = calc_dot_vector3x1_vector3x3(ajust_hip_vector, front_hip_ajust_array);
//LOG(INFO) << "front_hip_ajust_vector";
//logout_t_array(front_hip_ajust_vector);
  
  T left_hip_vec_landmark = calc_vector_from_parent_index_reverse<TList, T>(added_landmarks, POSE_NO_LEFT_BODY, pose_vector_vec[POSE_NO_LEFT_BODY]);
  std::vector<float> left_hip_vec_vector = convert_landmark_to_vector(left_hip_vec_landmark);
//LOG(INFO) << "left_hip_vec_landmark";
//logout_t_landmark(left_hip_vec_landmark);
//logout_t_array(left_hip_vec_vector);
  
  std::vector<float> front_hip_vector = calc_turn_vec_x(front_hip_ajust_vector, left_hip_vec_vector);
//LOG(INFO) << "front_hip_vector";
//logout_t_array(front_hip_vector);
  //T center_hip_landmark = calc_vector_from_parent_index_reverse<TList, T>(added_landmarks, POSE_NO_CENTER_BODY, pose_vector_vec[POSE_NO_CENTER_BODY]);
  std::vector<float> center_hip_landmark_vector = convert_landmark_to_vector(*center_hip_landmark);
//LOG(INFO) << "center_hip_landmark_vector";
//logout_t_array(center_hip_landmark_vector);
  std::vector<float> front_hip_coord = add_vector(front_hip_vector, center_hip_landmark_vector);
  T* front_hip_landmark = added_landmarks.add_landmark();
  *front_hip_landmark = convert_vector_to_landmark<T>(front_hip_coord);
//LOG(INFO) << "front_hip_landmark";
//logout_t_landmark(*front_hip_landmark);
  
  T* world_center_body_landmark = added_landmarks.add_landmark();
  world_center_body_landmark->set_x(-1.0);
  world_center_body_landmark->set_y(-1.0);
  world_center_body_landmark->set_z(-1.0);
  //world_center_body_landmark->set_visibility(-1.0);
  //world_center_body_landmark->set_presence(-1.0);
  
  // head center
  //   eye
  std::vector<int> center_eye_points;
  center_eye_points.push_back(POSE_NO_LEFT_EYE);
  center_eye_points.push_back(POSE_NO_RIGHT_EYE);
  T* center_eye_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, center_eye_points, *center_eye_landmark);
  //   ear
  std::vector<int> center_ear_points;
  center_ear_points.push_back(POSE_NO_LEFT_EAR);
  center_ear_points.push_back(POSE_NO_RIGHT_EAR);
  T* center_ear_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, center_ear_points, *center_ear_landmark);
  //   mouth
  std::vector<int> center_mouth_points;
  center_mouth_points.push_back(POSE_NO_LEFT_MOUTH);
  center_mouth_points.push_back(POSE_NO_RIGHT_MOUTH);
  T* center_mouth_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, center_mouth_points, *center_mouth_landmark);
  // TODO
  // nose
  std::vector<int> center_nose_points;
  center_nose_points.push_back(POSE_NO_CENTER_EYE);
  center_nose_points.push_back(POSE_NO_CENTER_MOUTH);
  T* center_nose_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, center_nose_points, *center_nose_landmark);
  // face up
  // TODO not imple
  T* up_face_landmark = added_landmarks.add_landmark();
  *up_face_landmark = *center_nose_landmark;
  
  // left
  //   finger center
  std::vector<int> left_finger_center_points;
  left_finger_center_points.push_back(POSE_NO_LEFT_PINKY);
  //left_finger_center_points.push_back(POSE_NO_LEFT_THUMB);
  left_finger_center_points.push_back(POSE_NO_LEFT_INDEX);
  T* left_finger_center_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, left_finger_center_points, *left_finger_center_landmark);
  // right
  //   finger center
  std::vector<int> right_finger_center_points;
  right_finger_center_points.push_back(POSE_NO_RIGHT_PINKY);
  //right_finger_center_points.push_back(POSE_NO_RIGHT_THUMB);
  right_finger_center_points.push_back(POSE_NO_RIGHT_INDEX);
  T* right_finger_center_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, right_finger_center_points, *right_finger_center_landmark);

  // hand
  T* left_left_hand_landmark = added_landmarks.add_landmark();
  *left_left_hand_landmark = added_landmarks.landmark(POSE_NO_LEFT_INDEX);
  T* left_right_hand_landmark = added_landmarks.add_landmark();
  *left_right_hand_landmark = added_landmarks.landmark(POSE_NO_RIGHT_INDEX);
  // hand front
  //   left
  // TODO not imple
  T* front_left_hand_landmark = added_landmarks.add_landmark();
  
  //   right
  // TODO not imple
  T* front_right_hand_landmark = added_landmarks.add_landmark();
  
}

// template:
//   TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
//   T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void add_and_calc_landmark_hand(const TList& landmarks, TList& added_landmarks) {
  // center
  std::vector<int> center_points;
  center_points.push_back(HAND_NO_WRIST);
  center_points.push_back(HAND_NO_MIDDLE_FINGER_MCP);
  T* center_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, center_points, *center_landmark);
  
  // side
  T* side_point = added_landmarks.add_landmark();
  *side_point = landmarks.landmark(HAND_NO_MIDDLE_FINGER_MCP);
  
  // up
  T* up_point = added_landmarks.add_landmark();
  *up_point = landmarks.landmark(HAND_NO_INDEX_FINGER_MCP);
  
  // front
  T side_hand_vector = calc_vector_from_parent_index<TList, T>(added_landmarks, HAND_NO_MIDDLE_FINGER_MCP, HAND_NO_WRIST);
  T ajust_center_vec_landmark = get_line_turn_ajust_x(side_hand_vector);
  std::vector<float> front_hand_ajust_array;
  y_turn_coord((float)90.0, front_hand_ajust_array); // TODO ★ from y:90.0 to center_shoulder(body)_line
  std::vector<float> ajust_center_vector = convert_landmark_to_vector(ajust_center_vec_landmark);
  std::vector<float> front_center_ajust_vector = calc_dot_vector3x1_vector3x3(ajust_center_vector, front_hand_ajust_array);
  
  T reverse_center_vec_landmark = calc_vector_from_parent_index_reverse<TList, T>(added_landmarks, HAND_NO_WRIST, HAND_NO_MIDDLE_FINGER_MCP);
  std::vector<float> reverse_center_vec_vector = convert_landmark_to_vector(reverse_center_vec_landmark);
  
  std::vector<float> front_center_vector = calc_turn_vec_x(front_center_ajust_vector, reverse_center_vec_vector);
  std::vector<float> hand_center_landmark_vector = convert_landmark_to_vector(*center_landmark);
  std::vector<float> front_hand_coord = add_vector(front_center_vector, hand_center_landmark_vector);
  T* hand_front_landmark = added_landmarks.add_landmark();
  *hand_front_landmark = convert_vector_to_landmark<T>(front_hand_coord);
  // TODO
  // left/rightの区別がついていない
  // 手は直立しないのであまり使えないデータとなっている
  
  // world center
  T* world_center = added_landmarks.add_landmark();
  // 後で値は設定
}

// template:
//   TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
//   T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void add_and_calc_landmark_face_mesh(const TList& landmarks, TList& added_landmarks) {
  // lip center
  std::vector<int> lip_center_points;
  lip_center_points.push_back(FACE_MESH_NO_LIP_INNER_UPPER);
  lip_center_points.push_back(FACE_MESH_NO_LIP_INNER_LOWER);
  lip_center_points.push_back(FACE_MESH_NO_LIP_INNER_LEFT_SIDE);
  lip_center_points.push_back(FACE_MESH_NO_LIP_INNER_RIGHT_SIDE);
  T* lip_center_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, lip_center_points, *lip_center_landmark);
  
  // eye center
  std::vector<int> eye_center_points;
  eye_center_points.push_back(FACE_MESH_NO_LEFT_EYE_IN_SIDE);
  eye_center_points.push_back(FACE_MESH_NO_RIGHT_EYE_IN_SIDE);
  //eye_center_points.push_back(FACE_MESH_NO_LEFT_EYE_OUT_SIDE);
  //eye_center_points.push_back(FACE_MESH_NO_RIGHT_EYE_OUT_SIDE);
  T* eye_center_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, eye_center_points, *eye_center_landmark);
  
  // left eye center
  std::vector<int> left_eye_center_points;
  left_eye_center_points.push_back(FACE_MESH_NO_LEFT_EYE_UPPER);
  left_eye_center_points.push_back(FACE_MESH_NO_LEFT_EYE_LOWER);
  left_eye_center_points.push_back(FACE_MESH_NO_LEFT_EYE_IN_SIDE);
  left_eye_center_points.push_back(FACE_MESH_NO_LEFT_EYE_OUT_SIDE);
  T* left_eye_center_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, left_eye_center_points, *left_eye_center_landmark);
  
  // right eye center
  std::vector<int> right_eye_center_points;
  right_eye_center_points.push_back(FACE_MESH_NO_RIGHT_EYE_UPPER);
  right_eye_center_points.push_back(FACE_MESH_NO_RIGHT_EYE_LOWER);
  right_eye_center_points.push_back(FACE_MESH_NO_RIGHT_EYE_IN_SIDE);
  right_eye_center_points.push_back(FACE_MESH_NO_RIGHT_EYE_OUT_SIDE);
  T* right_eye_center_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, right_eye_center_points, *right_eye_center_landmark);
  
  // world center
  T* world_center = added_landmarks.add_landmark();
  // 後で値は設定
  
  // face center
  std::vector<int> face_center_points;
  face_center_points.push_back(FACE_MESH_NO_TOP);
  face_center_points.push_back(FACE_MESH_NO_BOTTOM);
  face_center_points.push_back(FACE_MESH_NO_LEFT);
  face_center_points.push_back(FACE_MESH_NO_RIGHT);
  T* face_center_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, face_center_points, *face_center_landmark);
  
  // face side
  T* face_side_landmark = added_landmarks.add_landmark();
  *face_side_landmark = added_landmarks.landmark(FACE_MESH_NO_LEFT);
  
  // face up
  T* face_up_landmark = added_landmarks.add_landmark();
  *face_up_landmark = added_landmarks.landmark(FACE_MESH_NO_TOP);
  
  // face front
  T right_face_vector = calc_vector_from_parent_index<TList, T>(added_landmarks, FACE_MESH_NO_RIGHT, FACE_MESH_NO_CENTER);
  T ajust_center_vec_landmark = get_line_turn_ajust_x(right_face_vector);
  std::vector<float> front_face_ajust_array;
  y_turn_coord((float)90.0, front_face_ajust_array); // TODO ★ from y:90.0 to center_shoulder(body)_line
  std::vector<float> ajust_center_vector = convert_landmark_to_vector(ajust_center_vec_landmark);
  std::vector<float> front_center_ajust_vector = calc_dot_vector3x1_vector3x3(ajust_center_vector, front_face_ajust_array);
  
  T left_center_vec_landmark = calc_vector_from_parent_index_reverse<TList, T>(added_landmarks, FACE_MESH_NO_LEFT, FACE_MESH_NO_CENTER);
  std::vector<float> left_center_vec_vector = convert_landmark_to_vector(left_center_vec_landmark);
  
  std::vector<float> front_center_vector = calc_turn_vec_x(front_center_ajust_vector, left_center_vec_vector);
  std::vector<float> face_center_landmark_vector = convert_landmark_to_vector(*face_center_landmark);
  std::vector<float> front_face_coord = add_vector(front_center_vector, face_center_landmark_vector);
  T* face_front_landmark = added_landmarks.add_landmark();
  *face_front_landmark = convert_vector_to_landmark<T>(front_face_coord);
  
  // nose inner
  std::vector<int> nose_inner_points;
  nose_inner_points.push_back(FACE_MESH_NO_NOSE_OUTSIDE_LEFT);
  nose_inner_points.push_back(FACE_MESH_NO_NOSE_OUTSIDE_RIGHT);
  T* nose_inner_landmark = added_landmarks.add_landmark();
  set_and_calc_average(added_landmarks, nose_inner_points, *nose_inner_landmark);
}

void calc_from_landmark_to_world_landmark_face_mesh(const mediapipe::NormalizedLandmarkList& landmarks, mediapipe::LandmarkList& world_landmarks) {
  // 中心座標を取得し、上下左右の差分の数値に変換
  // その他(前後、visibility, presence)の数値はコピー
  
  // 目間距離を元に計算する
  //   sqrt(x2 + y2 + z2) = eye_dist
  //   x2 + y2 + z2 = eye_dist2
  //   x2 + y2 = eye_dist2 - z2
  //   (pix_x * pix_rate)2 + (pix_y * pix_rate)2 = eye_dist2 - z2
  //   pix_rate2(pix_x2 + pix_y2) = eye_dist2 - z2
  //   pix_rate2 = (eye_dist2 - z2) / (pix_x2 + pix_y2)
  //   pix_rate = sqrt((eye_dist2 - z2) / (pix_x2 + pix_y2))
  
  mediapipe::NormalizedLandmark left_eye_center = landmarks.landmark(FACE_MESH_NO_LEFT_EYE_CENTER);
  mediapipe::NormalizedLandmark right_eye_center = landmarks.landmark(FACE_MESH_NO_RIGHT_EYE_CENTER);
  
  int image_width = focus_display_width;
  int image_height = focus_display_height;
  
  float dist_z = left_eye_center.z() - right_eye_center.z();
  float dist_pix_x = (left_eye_center.x() - right_eye_center.x()) * image_width;
  float dist_pix_y = (left_eye_center.y() - right_eye_center.y()) * image_height;
  //float eyes_dist2 = eyes_dist * eyes_dist;
  //float dist_z2 = dist_z * dist_z;
  //float dist_pix_x2 = dist_pix_x * dist_pix_x;
  //float dist_pix_y2 = dist_pix_y * dist_pix_y;
  //float pix_rate2_upper = abs(eyes_dist2 - dist_z2);
  //float pix_rate2_lower = dist_pix_x2 + dist_pix_y2;
  //float pix_rate2 = pix_rate2_upper / pix_rate2_lower;
  //LOG(INFO) << "eyes_dist2(" << eyes_dist2 << "), dist_z2(" << dist_z2 << "), dist_pix_x2(" << dist_pix_x2 << "), dist_pix_y2(" << dist_pix_y2 << "), pix_rate2_upper(" << pix_rate2_upper << "), pix_rate2_lower(" << pix_rate2_lower << "), pix_rate2(" << pix_rate2 << ")";
  float pix_rate = sqrt(abs(eyes_dist * eyes_dist - dist_z * dist_z) / (dist_pix_x * dist_pix_x + dist_pix_y * dist_pix_y));
  LOG(INFO) << "calc pix_rate(" << pix_rate << ") from data: dist_z(" << dist_z << ", dist_pix_x(" << dist_pix_x << "), dist_pix_y(" << dist_pix_y << "))";
  
  mediapipe::NormalizedLandmark face_center = landmarks.landmark(FACE_MESH_NO_NOSE);
  
  for (int i = 0; i < landmarks.landmark_size(); i++) {
    mediapipe::NormalizedLandmark target_2d_landmark = landmarks.landmark(i);
    mediapipe::Landmark* target_3d_landmark = world_landmarks.add_landmark();
    target_3d_landmark->set_x((target_2d_landmark.x() - face_center.x()) * image_width * pix_rate);
    target_3d_landmark->set_y((target_2d_landmark.y() - face_center.y()) * image_height * pix_rate);
    target_3d_landmark->set_z(target_2d_landmark.z() - face_center.z());
    target_3d_landmark->set_visibility(target_2d_landmark.visibility());
    target_3d_landmark->set_presence(target_2d_landmark.presence());
  }
}

void calc_and_set_world_center_body_landmark(mediapipe::NormalizedLandmarkList& landmarks_2d, mediapipe::LandmarkList& landmarks_3d) {
  // pickup: shoulder, hip 
  std::vector<int> target_landmark_list;
  target_landmark_list.push_back(POSE_NO_LEFT_SHOULDER);
  target_landmark_list.push_back(POSE_NO_RIGHT_SHOULDER);
  target_landmark_list.push_back(POSE_NO_LEFT_HIP);
  target_landmark_list.push_back(POSE_NO_RIGHT_HIP);
  int target_center_landmark = POSE_NO_CENTER_HIP;
  std::vector<float> x_list;
  std::vector<float> y_list;
  
  // max
  std::pair<int, float> max_landmark_value;
  bool is_max_coord = 0; // 0:x, 1:y, 2:z
  max_landmark_value.first = -1;
  max_landmark_value.second = 0.0;
  for (int i = 0; i < target_landmark_list.size(); i++) {
    int landmark_no = target_landmark_list[i];
    float x_2d = landmarks_2d.landmark(target_center_landmark).x() - landmarks_2d.landmark(landmark_no).x();
    float y_2d = landmarks_2d.landmark(target_center_landmark).y() - landmarks_2d.landmark(landmark_no).y();
    
    if (max_landmark_value.second < x_2d) {
      max_landmark_value.second = x_2d;
      max_landmark_value.first = landmark_no;
      is_max_coord = 0;
    }
    if (max_landmark_value.second < y_2d) {
      max_landmark_value.second = y_2d;
      max_landmark_value.first = landmark_no;
      is_max_coord = 1;
    }
  }
  
  // 2d : 3d -> m/pixel
  int landmark_no = max_landmark_value.first;
  float pixel_m_rate;
  if (is_max_coord == 0) {
    float x_3d = landmarks_3d.landmark(target_center_landmark).x() - landmarks_3d.landmark(landmark_no).x();
    pixel_m_rate = std::abs(x_3d / max_landmark_value.second);
  } else {
    float y_3d = landmarks_3d.landmark(target_center_landmark).y() - landmarks_3d.landmark(landmark_no).y();
    pixel_m_rate = std::abs(y_3d / max_landmark_value.second);
  }
  
  mediapipe::NormalizedLandmark center_body = landmarks_2d.landmark(POSE_NO_CENTER_BODY);
  float center_x_m = (center_body.x() - 0.5) * pixel_m_rate;
  float center_y_m = (center_body.y() - 0.5) * pixel_m_rate;
  float center_z_m = 0.0; // TODO z
  
  mediapipe::NormalizedLandmark* world_center_body_2d = landmarks_2d.mutable_landmark(POSE_NO_WORLD_CENTER_BODY);
  world_center_body_2d->set_x(center_x_m);
  world_center_body_2d->set_y(center_y_m);
  world_center_body_2d->set_z(center_z_m);
  mediapipe::Landmark* world_center_body_3d = landmarks_3d.mutable_landmark(POSE_NO_WORLD_CENTER_BODY);
  world_center_body_3d->set_x(center_x_m);
  world_center_body_3d->set_y(center_y_m);
  world_center_body_3d->set_z(center_z_m);
}

void calc_and_set_world_center_hand_landmark(mediapipe::NormalizedLandmarkList& landmarks_2d, mediapipe::LandmarkList& landmarks_3d) {
  // same algorizum: calc_and_set_world_center_body_landmark()
  // pickup
  std::vector<int> target_landmark_list;
  target_landmark_list.push_back(HAND_NO_THUMB_MCP);
  target_landmark_list.push_back(HAND_NO_INDEX_FINGER_MCP);
  target_landmark_list.push_back(HAND_NO_WRIST);
  target_landmark_list.push_back(HAND_NO_PINKY_MCP);
  int target_center_landmark = HAND_NO_CENTER;
  std::vector<float> x_list;
  std::vector<float> y_list;
  
  // max
  std::pair<int, float> max_landmark_value;
  bool is_max_coord = 0; // 0:x, 1:y, 2:z
  max_landmark_value.first = -1;
  max_landmark_value.second = 0.0;
  for (int i = 0; i < target_landmark_list.size(); i++) {
    int landmark_no = target_landmark_list[i];
    float x_2d = landmarks_2d.landmark(target_center_landmark).x() - landmarks_2d.landmark(landmark_no).x();
    float y_2d = landmarks_2d.landmark(target_center_landmark).y() - landmarks_2d.landmark(landmark_no).y();
    
    if (max_landmark_value.second < x_2d) {
      max_landmark_value.second = x_2d;
      max_landmark_value.first = landmark_no;
      is_max_coord = 0;
    }
    if (max_landmark_value.second < y_2d) {
      max_landmark_value.second = y_2d;
      max_landmark_value.first = landmark_no;
      is_max_coord = 1;
    }
  }
  
  // 2d : 3d -> m/pixel
  int landmark_no = max_landmark_value.first;
  float pixel_m_rate;
  if (is_max_coord == 0) {
    float x_3d = landmarks_3d.landmark(target_center_landmark).x() - landmarks_3d.landmark(landmark_no).x();
    pixel_m_rate = std::abs(x_3d / max_landmark_value.second);
  } else {
    float y_3d = landmarks_3d.landmark(target_center_landmark).y() - landmarks_3d.landmark(landmark_no).y();
    pixel_m_rate = std::abs(y_3d / max_landmark_value.second);
  }
  
  mediapipe::NormalizedLandmark center_body = landmarks_2d.landmark(HAND_NO_CENTER);
  float center_x_m = (center_body.x() - 0.5) * pixel_m_rate;
  float center_y_m = (center_body.y() - 0.5) * pixel_m_rate;
  float center_z_m = 0.0; // TODO z
  
  mediapipe::NormalizedLandmark* world_center_body_2d = landmarks_2d.mutable_landmark(HAND_NO_WORLD_CENTER);
  world_center_body_2d->set_x(center_x_m);
  world_center_body_2d->set_y(center_y_m);
  world_center_body_2d->set_z(center_z_m);
  mediapipe::Landmark* world_center_body_3d = landmarks_3d.mutable_landmark(HAND_NO_WORLD_CENTER);
  world_center_body_3d->set_x(center_x_m);
  world_center_body_3d->set_y(center_y_m);
  world_center_body_3d->set_z(center_z_m);
}

void calc_and_set_world_center_face_mesh_landmark(mediapipe::NormalizedLandmarkList& landmarks_2d, mediapipe::LandmarkList& landmarks_3d) {
  // same algorizum: calc_and_set_world_center_body_landmark()
  // pickup
  std::vector<int> target_landmark_list;
  target_landmark_list.push_back(FACE_MESH_NO_LEFT_UPPER_FACE);
  target_landmark_list.push_back(FACE_MESH_NO_RIGHT_UPPER_FACE);
  target_landmark_list.push_back(FACE_MESH_NO_LEFT_LOWER_FACE);
  target_landmark_list.push_back(FACE_MESH_NO_RIGHT_LOWER_FACE);
  int target_center_landmark = FACE_MESH_NO_NOSE;
  std::vector<float> x_list;
  std::vector<float> y_list;
  
  // max
  std::pair<int, float> max_landmark_value;
  bool is_max_coord = 0; // 0:x, 1:y, 2:z
  max_landmark_value.first = -1;
  max_landmark_value.second = 0.0;
  for (int i = 0; i < target_landmark_list.size(); i++) {
    int landmark_no = target_landmark_list[i];
    float x_2d = landmarks_2d.landmark(target_center_landmark).x() - landmarks_2d.landmark(landmark_no).x();
    float y_2d = landmarks_2d.landmark(target_center_landmark).y() - landmarks_2d.landmark(landmark_no).y();
    
    if (max_landmark_value.second < x_2d) {
      max_landmark_value.second = x_2d;
      max_landmark_value.first = landmark_no;
      is_max_coord = 0;
    }
    if (max_landmark_value.second < y_2d) {
      max_landmark_value.second = y_2d;
      max_landmark_value.first = landmark_no;
      is_max_coord = 1;
    }
  }
  
  // 2d : 3d -> m/pixel
  int landmark_no = max_landmark_value.first;
  float pixel_m_rate;
  if (is_max_coord == 0) {
    float x_3d = landmarks_3d.landmark(target_center_landmark).x() - landmarks_3d.landmark(landmark_no).x();
    pixel_m_rate = std::abs(x_3d / max_landmark_value.second);
  } else {
    float y_3d = landmarks_3d.landmark(target_center_landmark).y() - landmarks_3d.landmark(landmark_no).y();
    pixel_m_rate = std::abs(y_3d / max_landmark_value.second);
  }
  
  mediapipe::NormalizedLandmark center_body = landmarks_2d.landmark(FACE_MESH_NO_NOSE);
  float center_x_m = (center_body.x() - 0.5) * pixel_m_rate;
  float center_y_m = (center_body.y() - 0.5) * pixel_m_rate;
  float center_z_m = 0.0; // TODO z
  
  mediapipe::NormalizedLandmark* world_center_body_2d = landmarks_2d.mutable_landmark(FACE_MESH_NO_WORLD_CENTER);
  world_center_body_2d->set_x(center_x_m);
  world_center_body_2d->set_y(center_y_m);
  world_center_body_2d->set_z(center_z_m);
  mediapipe::Landmark* world_center_body_3d = landmarks_3d.mutable_landmark(FACE_MESH_NO_WORLD_CENTER);
  world_center_body_3d->set_x(center_x_m);
  world_center_body_3d->set_y(center_y_m);
  world_center_body_3d->set_z(center_z_m);
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> std::vector<float> calc_vector_to_vec(const T& landmark, const T& parent_landmark) {
  std::vector<float> vec;
  //vec.push_back(parent_landmark.x() - landmark.x());
  //vec.push_back(parent_landmark.y() - landmark.y());
  //vec.push_back(parent_landmark.z() - landmark.z());
  vec.push_back(landmark.x() - parent_landmark.x());
  vec.push_back(landmark.y() - parent_landmark.y());
  vec.push_back(landmark.z() - parent_landmark.z());
  return vec;
}

// template TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
template <class TList> std::vector<float> calc_vector_from_parent_index_to_vec(const TList& landmarks, int target_index, int parent_index) {
  return calc_vector_to_vec(landmarks.landmark(target_index), landmarks.landmark(parent_index));
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> void add_vector(const T& landmark, T& sum_landmark) {
  sum_landmark.set_x(sum_landmark.x() + landmark.x());
  sum_landmark.set_y(sum_landmark.y() + landmark.y());
  sum_landmark.set_z(sum_landmark.z() + landmark.z());
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T calc_vector(const T& landmark, const T& parent_landmark) {
  T vec;
  vec.set_x(landmark.x() - parent_landmark.x());
  vec.set_y(landmark.y() - parent_landmark.y());
  vec.set_z(landmark.z() - parent_landmark.z());
  //vec.set_x(parent_landmark.x() - landmark.x());
  //vec.set_y(parent_landmark.y() - landmark.y());
  //vec.set_z(parent_landmark.z() - landmark.z());
  return vec;
}

// template TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
template <class TList, class T> T calc_vector_from_parent_index(const TList& landmarks, int target_index, int parent_index) {
  return calc_vector(landmarks.landmark(target_index), landmarks.landmark(parent_index));
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T calc_vector_reverse(const T& landmark, const T& parent_landmark) {
  T vec;
  //vec.set_x(landmark.x() - parent_landmark.x());
  //vec.set_y(landmark.y() - parent_landmark.y());
  //vec.set_z(landmark.z() - parent_landmark.z());
  vec.set_x(parent_landmark.x() - landmark.x());
  vec.set_y(parent_landmark.y() - landmark.y());
  vec.set_z(parent_landmark.z() - landmark.z());
  return vec;
}

// template TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
template <class TList, class T> T calc_vector_from_parent_index_reverse(const TList& landmarks, int target_index, int parent_index) {
  return calc_vector_reverse(landmarks.landmark(target_index), landmarks.landmark(parent_index));
}

// template TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList, T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> void calc_world_vector_from_parent(const TList& landmarks, std::vector<int>& vector_vec, TList& landmarks_vec) {
  // file://C:\Users\reorio\Desktop\hide\game\pose_control\windows\mediapipe\bazel-bin\mediapipe\framework\formats\landmark.pb.h
  //   mediapipe::LandmarkList
  for (int i = 0; i < landmarks.landmark_size(); i++) {
    T* landmark = landmarks_vec.add_landmark();
    if (vector_vec[i] == -1) {
      continue;
    }
    if (vector_vec[i] >= landmarks.landmark_size()) {
      LOG(WARNING) << "calc_world_vector_from_parent() index's(" << i << ") parent_index(" << vector_vec[i] << ") is over landmark_size(" << landmarks.landmark_size() << ")";
      continue;
    }
    *landmark = calc_vector_from_parent_index<TList, T>(landmarks, i, vector_vec[i]);
  }
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T get_reverse_vec(const T& landmark) {
  mediapipe::Landmark reverse_vec;
  reverse_vec.set_x(- landmark.x());
  reverse_vec.set_y(- landmark.y());
  reverse_vec.set_z(- landmark.z());
  return reverse_vec;
}

std::vector<float> add_vector(std::vector<float>& input_vec_1, std::vector<float>& input_vec_2) {
  std::vector<float> result;
  if (input_vec_1.size() > input_vec_2.size()) {
    result.resize(input_vec_1.size());
  } else {
    result.resize(input_vec_2.size());
  }
  for (int i = 0; i < input_vec_1.size(); i++) {
    result[i] += input_vec_1[i];
  }
  for (int i = 0; i < input_vec_2.size(); i++) {
    result[i] += input_vec_2[i];
  }
  return result;
}

float calc_rad2dig(float radian) {
  return radian * 180.0 / M_PI;
}

float calc_dig2rad(float digree) {
  return digree / 180.0 * M_PI;
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> float calc_vector_length(const T& vec) {
  return pow((vec.x() * vec.x()) + (vec.y() * vec.y()) + (vec.z() * vec.z()), 0.5);
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> float calc_dot(const T& vec_1, const T& vec_2) {
  return vec_1.x() * vec_2.x() + vec_1.y() * vec_2.y() + vec_1.z() * vec_2.z();
}

const int TURN_ARRAY_1_SIZE = 3;
const int TURN_ARRAY_2_SIZE = 3;
const int TURN_ARRAY_SIZE = TURN_ARRAY_1_SIZE * TURN_ARRAY_2_SIZE;

std::vector<float> calc_dot_vector3x1_vector3x3(std::vector<float>& vec_1, std::vector<float>& vec_2) {
  cv::Mat mat_1(vec_1);
  cv::Mat mat_2_0(vec_2);
  cv::Mat mat_2 = mat_2_0.reshape(0, TURN_ARRAY_2_SIZE);
  cv::Mat mat_r(1, 3, CV_32FC1);
  
  mat_2 * mat_1;
  mat_1 * mat_2;
  mat_r = mat_2 * mat_1;
  
  std::vector<float> result;
  for (cv::MatConstIterator_<float> itr = mat_r.begin<float>(); itr != mat_r.end<float>(); itr++) {
    result.push_back(*itr);
  }
  
  return result;
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> float calc_subtended_angle(const T& parent_landmark, const T& landmark) {
  float vec_length_parent = calc_vector_length(parent_landmark);
  float vec_length_target = calc_vector_length(landmark);
  float vec_length_multiple = vec_length_parent * vec_length_target;
  if (vec_length_multiple == 0.0) {
    return -360.0;
  }
  float inner = calc_dot(parent_landmark, landmark);
  float radian = acos(inner / (vec_length_parent * vec_length_target));
  float angle = calc_rad2dig(radian);
  return angle;
}


void calc_angles(mediapipe::LandmarkList& landmarks, std::vector<int> vector_vec) {
  for (int i = 0; i < vector_vec.size(); i++) {
    mediapipe::Landmark* landmark_target = landmarks.mutable_landmark(i);
    if (vector_vec[i] != -1) {
      mediapipe::Landmark landmark_parent_reverse = get_reverse_vec(landmarks.landmark(vector_vec[i]));
      float angle = calc_subtended_angle(landmark_parent_reverse, *landmark_target);
      landmark_target->set_presence(angle);
    } else {
      landmark_target->set_visibility(-1.0);
      //landmark_target->set_presence(-1.0);
    }
  }
}

void calc_camera_diff_angle(mediapipe::LandmarkList& landmarks, mediapipe::LandmarkList& camera_diff_angle) {
  for (int i = 0; i < landmarks.landmark_size(); i++) {
    mediapipe::Landmark* landmark = landmarks.mutable_landmark(i);
    mediapipe::Landmark* camera_diff = camera_diff_angle.add_landmark();
    float x = landmark->x();
    float y = landmark->y();
    float z = landmark->z();
    float angle_x;
    float angle_y;
    float angle_z;
    
    if (y == 0.0) {
      if (z <= 0.0) {
        angle_x = 90.0;
      } else {
        angle_x = -90.0;
      }
    } else {
      angle_x = calc_rad2dig(atan(z / y));
    }
    //float c_x = sqrt(z * z + y * y);
    //float c_x = y / cos(angle_x); // TODO c(斜め)の正負で、角度の調整
    if (x == 0.0) {
      if (z <= 0.0) {
        angle_y = 90.0;
      } else {
        angle_y = -90.0;
      }
      angle_z = 90.0;
    } else {
      angle_y = calc_rad2dig(atan(z / x));
      angle_z = calc_rad2dig(atan(y / x));
    }
    
    // +-で補正
    bool is_use_ajust = true;
    is_use_ajust = false; // test
    if (is_use_ajust == true) {
      if (y > 0.0) {
        angle_x += 90.0;
      } else if (y < 0.0) {
        angle_x -= 90.0;
      } else {
        if (z < 0.0) {
          angle_x = 0.0;
        } else if (z > 0.0) {
          angle_x = 180.0;
        } else {
          angle_x = 0.0;
          //angle_x = 180.0;
          //angle_x = -180.0;
        }
      }
      if (x > 0.0) {
        angle_y += 90.0;
        angle_z += 90.0;
      } else if (x < 0.0) {
        angle_y -= 90.0;
        angle_z -= 90.0;
      } else {
        if (z < 0.0) {
          angle_y = 0.0;
        } else if (z > 0.0) {
          angle_y = 180.0;
        } else {
          angle_y = 0.0;
          //angle_y = 180.0;
          //angle_y = -180.0;
        }
        if (y < 0.0) {
          angle_z = 0.0;
        } else if (y > 0.0) {
          angle_z = 180.0;
        } else {
          angle_z = 0.0;
          //angle_z = 180.0;
          //angle_z = -180.0;
        }
      }
    }
    
    camera_diff->set_x(angle_x);
    camera_diff->set_y(angle_y);
    camera_diff->set_z(angle_z);
  }
}

const int LINE_ID_NO_X = 0;
const int LINE_ID_NO_Y = 1;
const int LINE_ID_NO_Z = 2;

const int TURN_API_ID_NO_X = 0;
const int TURN_API_ID_NO_Y = 1;
const int TURN_API_ID_NO_Z = 2;
const int TURN_API_ID_NO_NG = -1;

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> void set_x_line(T& vec) {
  vec.set_x((float)1.0);
  vec.set_y((float)0.0);
  vec.set_z((float)0.0);
}
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> void set_y_line(T& vec) {
  vec.set_x((float)0.0);
  vec.set_y((float)1.0);
  vec.set_z((float)0.0);
}
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> void set_z_line(T& vec) {
  vec.set_x((float)0.0);
  vec.set_y((float)0.0);
  vec.set_z((float)1.0);
}
// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> void set_xyz_line(T& vec) {
  vec.set_x((float)1.0);
  vec.set_y((float)1.0);
  vec.set_z((float)1.0);
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> std::vector<float> convert_landmark_to_vector(T& landmark) {
  std::vector<float> result;
  result.resize(TURN_ARRAY_1_SIZE);
  result[0] = landmark.x();
  result[1] = landmark.y();
  result[2] = landmark.z();
  return result;
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T convert_vector_to_landmark(std::vector<float>& input_vec) {
  T landmark;
  landmark.set_x(input_vec[0]);
  landmark.set_y(input_vec[1]);
  landmark.set_z(input_vec[2]);
  return landmark;
}

void init_turn_array(std::vector<float>& turn_coord) {
  if (turn_coord.size() < TURN_ARRAY_SIZE) {
    turn_coord.resize(TURN_ARRAY_SIZE);
  }
}
void x_turn_coord(float angle, std::vector<float>& turn_coord) {
  init_turn_array(turn_coord);
  
  float angle_radians = calc_dig2rad(angle);
  
  turn_coord[0] = (float)1.0;
  turn_coord[1] = (float)0.0;
  turn_coord[2] = (float)0.0;
  turn_coord[3] = (float)0.0;
  turn_coord[4] = (float)cos(angle_radians);
  turn_coord[5] = (float)-sin(angle_radians);
  turn_coord[6] = (float)0.0;
  turn_coord[7] = (float)sin(angle_radians);
  turn_coord[8] = (float)cos(angle_radians);
}

void y_turn_coord(float angle, std::vector<float>& turn_coord) {
  init_turn_array(turn_coord);
  
  float angle_radians = calc_dig2rad(angle);
  
  turn_coord[0] = (float)cos(angle_radians);
  turn_coord[1] = (float)0.0;
  turn_coord[2] = (float)sin(angle_radians);
  turn_coord[3] = (float)0.0;
  turn_coord[4] = (float)1.0;
  turn_coord[5] = (float)0.0;
  turn_coord[6] = (float)-sin(angle_radians);
  turn_coord[7] = (float)0.0;
  turn_coord[8] = (float)cos(angle_radians);
}

void z_turn_coord(float angle, std::vector<float>& turn_coord) {
  init_turn_array(turn_coord);
  
  float angle_radians = calc_dig2rad(angle);
  
  turn_coord[0] = (float)cos(angle_radians);
  turn_coord[1] = (float)-sin(angle_radians);
  turn_coord[2] = (float)0.0;
  turn_coord[3] = (float)sin(angle_radians);
  turn_coord[4] = (float)cos(angle_radians);
  turn_coord[5] = (float)0.0;
  turn_coord[6] = (float)0.0;
  turn_coord[7] = (float)0.0;
  turn_coord[8] = (float)1.0;
}


// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> std::pair<float, int> get_line_consider(T& consider_vec, int target_line, int turn_line) {
  T target_angle_vec;

  T ajust_line;
  int turn_api_id = TURN_API_ID_NO_NG;
  if (target_line == LINE_ID_NO_X) {
    set_x_line(ajust_line);
    target_angle_vec.set_x(consider_vec.x());
    if (turn_line == LINE_ID_NO_X) {
      LOG(INFO) << __FILE__ << ":" << __LINE__ << "invalid input paramter";
      // TODO error
    } else if (turn_line == LINE_ID_NO_Y) {
      target_angle_vec.set_z(consider_vec.z());
      turn_api_id = TURN_API_ID_NO_Y;
    } else if (turn_line == LINE_ID_NO_Z) {
      target_angle_vec.set_y(consider_vec.y());
      turn_api_id = TURN_API_ID_NO_Z;
    }
  } else if (target_line == LINE_ID_NO_Y) {
    set_y_line(ajust_line);
    target_angle_vec.set_y(consider_vec.y());
    if (turn_line == LINE_ID_NO_X) {
      target_angle_vec.set_z(consider_vec.z());
      turn_api_id = TURN_API_ID_NO_X;
    } else if (turn_line == LINE_ID_NO_Y) {
      LOG(INFO) << __FILE__ << ":" << __LINE__ << "invalid input paramter";
      // TODO error
    } else if (turn_line == LINE_ID_NO_Z) {
      target_angle_vec.set_x(consider_vec.x());
      turn_api_id = TURN_API_ID_NO_Z;
    }
  } else if (target_line == LINE_ID_NO_Z) {
    set_z_line(ajust_line);
    target_angle_vec.set_z(consider_vec.z());
    if (turn_line == LINE_ID_NO_X) {
      target_angle_vec.set_y(consider_vec.y());
      turn_api_id = TURN_API_ID_NO_X;
    } else if (turn_line == LINE_ID_NO_Y) {
      target_angle_vec.set_x(consider_vec.x());
      turn_api_id = TURN_API_ID_NO_Y;
    } else if (turn_line == LINE_ID_NO_Z) {
      LOG(INFO) << __FILE__ << ":" << __LINE__ << "invalid input paramter";
      // TODO error
    }
  }
  float ajust_angle = calc_subtended_angle(target_angle_vec, ajust_line);

  return std::pair<float, int>(ajust_angle, turn_api_id);
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> std::pair<float, int> get_line_consider_and_to(T& consider_vec, int target_line, int turn_line) {
  T target_angle_vec;
  float and_to = 0.0;
  
  T ajust_line;
  if (target_line == LINE_ID_NO_X) {
    set_x_line(ajust_line);
    target_angle_vec.set_x(consider_vec.x());
    int turn_api_id = TURN_API_ID_NO_NG;
    if (turn_line == LINE_ID_NO_X) {
      LOG(INFO) << __FILE__ << ":" << __LINE__ << "invalid input paramter";
      // TODO error
    } else if (turn_line == LINE_ID_NO_Y) {
      target_angle_vec.set_z(consider_vec.z());
      turn_api_id = TURN_API_ID_NO_Y;
    } else if (turn_line == LINE_ID_NO_Z) {
      target_angle_vec.set_y(consider_vec.y());
      turn_api_id = TURN_API_ID_NO_Z;
    }
  } else if (target_line == LINE_ID_NO_Y) {
    set_y_line(ajust_line);
    target_angle_vec.set_y(consider_vec.y());
    if (turn_line == LINE_ID_NO_X) {
      target_angle_vec.set_z(consider_vec.z());
      turn_api_id = TURN_API_ID_NO_X;
      // TODO test
      and_to = consider_vec.z();
    } else if (turn_line == LINE_ID_NO_Y) {
      LOG(INFO) << __FILE__ << ":" << __LINE__ << "invalid input paramter";
      // TODO error
    } else if (turn_line == LINE_ID_NO_Z) {
      target_angle_vec.set_x(consider_vec.x());
      turn_api_id = TURN_API_ID_NO_Z;
      // TODO test
      and_to = consider_vec.x();
    }
  } else if (target_line == LINE_ID_NO_Z) {
    set_z_line(ajust_line);
    target_angle_vec.set_z(consider_vec.z());
    if (turn_line == LINE_ID_NO_X) {
      target_angle_vec.set_y(consider_vec.y());
      turn_api_id = TURN_API_ID_NO_X;
    } else if (turn_line == LINE_ID_NO_Y) {
      target_angle_vec.set_x(consider_vec.x());
      turn_api_id = TURN_API_ID_NO_Y;
    } else if (turn_line == LINE_ID_NO_Z) {
      LOG(INFO) << __FILE__ << ":" << __LINE__ << "invalid input paramter";
      // TODO error
    }
  }

  float ajust_angle = calc_subtended_angle(target_angle_vec, ajust_line);
  if (and_to < (float)0.0) {
    ajust_angle *= (float)-1.0;
  }

  return std::pair<float, int>(ajust_angle, turn_api);
}

void get_turn_coord(float angle, int turn_api_id, std::vector<float>& ajust_array) {
  switch (turn_api_id) {
  case TURN_API_ID_NO_X:
    x_turn_coord(angle, ajust_array);
    break;
  case TURN_API_ID_NO_Y:
    y_turn_coord(angle, ajust_array);
    break;
  case TURN_API_ID_NO_Z:
    z_turn_coord(angle, ajust_array);
    break;
  case TURN_API_ID_NO_NG:
  default:
    LOG(INFO) << "input parameter invalid";
    // TODO error
    break;
  }
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T get_line_turn_ajust(T& input_vector, T& consider_vec, int target_line, int turn_line, bool is_reverse=false) {
  std::pair<float, int> line_consider = get_line_consider(consider_vec, target_line, turn_line);
  float ajust_angle = line_consider.first;
  int turn_api_id = line_consider.second;
  if (is_reverse == true) {
    ajust_angle *= (float)-1.0;
  }
  
  //std::vector<std::vector<float> > ajust_array;
  std::vector<float> ajust_array;
  get_turn_coord(ajust_angle, turn_api_id, ajust_array);
  /*
  switch (turn_api_id) {
  case TURN_API_ID_NO_X:
    x_turn_coord(ajust_angle, ajust_array);
    break;
  case TURN_API_ID_NO_Y:
    y_turn_coord(ajust_angle, ajust_array);
    break;
  case TURN_API_ID_NO_Z:
    z_turn_coord(ajust_angle, ajust_array);
    break;
  case TURN_API_ID_NO_NG:
  default:
    LOG(INFO) << "input parameter invalid";
    // TODO error
    break;
  }
  */
  std::vector<float> line_vector = calc_dot_vector3x1_vector3x3(convert_landmark_to_vector(input_vector), ajust_array);
  T line_vec;
  line_vec.set_x(line_vector[0]);
  line_vec.set_y(line_vector[1]);
  line_vec.set_z(line_vector[2]);
  return line_vec;
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T get_line_turn_ajust_x(T& input_vector) {
  T y_line_ajusted = get_line_turn_ajust(input_vector, input_vector, LINE_ID_NO_X, LINE_ID_NO_Z, false);
//LOG(INFO) << "y_line_ajusted";
//logout_t_landmark(y_line_ajusted);
  T zy_line_ajusted = get_line_turn_ajust(y_line_ajusted, y_line_ajusted, LINE_ID_NO_X, LINE_ID_NO_Y, true);
//LOG(INFO) << "zy_line_ajusted";
//logout_t_landmark(zy_line_ajusted);
  return zy_line_ajusted;
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T get_line_turn_ajust_y(T& input_vector) {
  var x_line_ajusted = get_line_turn_ajust(input_vector, input_vector, LINE_ID_NO_Y, LINE_ID_NO_Z, true);
  var zx_line_ajusted = get_line_turn_ajust(x_line_ajusted, x_line_ajusted, LINE_ID_NO_Y, LINE_ID_NO_X, true);
  return zx_line_ajusted;
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> T get_line_turn_ajust_z(T& input_vector) {
  var y_line_ajusted = get_line_turn_ajust(input_vector, input_vector, LINE_ID_NO_Z, LINE_ID_NO_X, true);
  var xy_line_ajusted = get_line_turn_ajust(y_line_ajusted, y_line_ajusted, LINE_ID_NO_Z, LINE_ID_NO_Y, true);
  return xy_line_ajusted;
}

std::vector<float> calc_turn_vec_x(std::vector<float>& input_vec, std::vector<float>& consider_vec) {
  mediapipe::Landmark consider_landmark = convert_vector_to_landmark<mediapipe::Landmark>(consider_vec); // とりあえずの変換
  std::pair<float, int> y_line_consider = get_line_consider(consider_landmark, LINE_ID_NO_X, LINE_ID_NO_Z);
  float y_ajust_angle = y_line_consider.first;
  int y_turn_api_id = y_line_consider.second;
  std::vector<float> y_ajust_array;
  get_turn_coord(-y_ajust_angle, y_turn_api_id, y_ajust_array);
  std::vector<float> y_ajust_vec = calc_dot_vector3x1_vector3x3(input_vec, y_ajust_array);
  
  std::pair<float, int> zy_line_consider = get_line_consider(consider_landmark, LINE_ID_NO_X, LINE_ID_NO_Y);
  float zy_ajust_angle = zy_line_consider.first;
  int zy_turn_api_id = zy_line_consider.second;
  std::vector<float> zy_ajust_array;
  get_turn_coord(zy_ajust_angle, zy_turn_api_id, zy_ajust_array);
  std::vector<float> zy_ajust_vec = calc_dot_vector3x1_vector3x3(y_ajust_vec, zy_ajust_array);
  
  return zy_ajust_vec;
}

std::vector<float> calc_turn_vec_y() {
  std::vector<float> result;
  return result;
}

std::vector<float> calc_turn_vec_z() {
  std::vector<float> result;
  return result;
}

// template T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> void logout_t_landmark(const T& landmark) {
  LOG(INFO) << " (" << landmark.x() << ", " << landmark.y() << ", " << landmark.z() << ", " << landmark.visibility() << ", " << landmark.presence() << ")";
}

// template TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList
template <class TList, class T> void logout_t_landmarks(const TList& landmarks) {
  LOG(INFO) << "landmarks(" << landmarks.landmark_size() << "): ";
  
  for (int i = 0; i < landmarks.landmark_size(); i++) {
    T landmark = landmarks.landmark(i);
    LOG(INFO) << "index: " << i << " (" << landmark.x() << ", " << landmark.y() << ", " << landmark.z() << ", " << landmark.visibility() << ", " << landmark.presence() << ")\r\n";
  }
}

template <class TList> void logout_t_array(const TList& array) {
  LOG(INFO) << "array(" << array.size() << "): ";
  
  for (int i = 0; i < array.size(); i++) {
    LOG(INFO) << "array[" << i << " ]: " << array[i] << ".";
  }
}

void logout_coordinate() {
  for (int logout_type : logout_type_specify) {
    switch (logout_type) {
    case COORDINATE_MODE_LAST_2D:
      LOG(WARNING) << "logout type(" << logout_type << ":LAST_2D).";
      logout_landmark(last_target_vec_2d, logout_number_specify, "  ");
      break;
    case COORDINATE_MODE_LAST_3D:
      LOG(WARNING) << "logout type(" << logout_type << ":LAST_3D).";
      logout_world_landmark(last_target_vec, logout_number_specify, "  ");
      break;
    case COORDINATE_MODE_LAST_3D_VEC_ANGLE:
      LOG(WARNING) << "logout type(" << logout_type << ":LAST_3D_VEC_ANGLE).";
      logout_world_landmark(last_target_vec_angle, logout_number_specify, "  ");
      break;
    case COORDINATE_MODE_LAST_CAMERA_DIFF_ANGLE:
      LOG(WARNING) << "logout type(" << logout_type << ":LAST_CAMERA_DIFF_ANGLE).";
      logout_world_landmark(last_target_camera_diff_angle, logout_number_specify, "  ");
      break;
    default:
      LOG(WARNING) << "logout type(" << logout_type << ") is uknown, skip.";
      break;
    }
  }
}

void logout_landmarks(const mediapipe::NormalizedLandmarkList& landmarks, std::string indent) {
  LOG(INFO) << indent << "screen 2d landmarks(" << landmarks.landmark_size() << "): ";
  
  for (int i = 0; i < landmarks.landmark_size(); i++) {
    mediapipe::NormalizedLandmark landmark = landmarks.landmark(i);
    LOG(INFO) << indent << "index: " << i << " (" << landmark.x() << ", " << landmark.y() << ", " << landmark.z() << ", " << landmark.visibility() << ", " << landmark.presence() << ")\r\n";
  }
}

void logout_landmark(const mediapipe::NormalizedLandmarkList& landmarks, std::vector<int> number_specify, std::string indent) {
  LOG(INFO) << indent << "specify 2d landmark(" << landmarks.landmark_size() << "): ";
  
  for (int i = 0; i < number_specify.size(); i++) {
    int j = number_specify[i];
    if (j >= landmarks.landmark_size()) {
      continue;
    }
    mediapipe::NormalizedLandmark landmark = landmarks.landmark(j);
    LOG(INFO) << indent << "index: " << j << " (" << landmark.x() << ", " << landmark.y() << ", " << landmark.z() << ", " << landmark.visibility() << ", " << landmark.presence() << ")\r\n";
  }
}

void logout_world_landmarks(const mediapipe::LandmarkList& landmarks, std::string indent) {
  LOG(INFO) << indent << "world landmarks(" << landmarks.landmark_size() << "): ";
  
  for (int i = 0; i < landmarks.landmark_size(); i++) {
    mediapipe::Landmark landmark = landmarks.landmark(i);
    LOG(INFO) << indent << "index: " << i << " (" << landmark.x() << ", " << landmark.y() << ", " << landmark.z() << ", " << landmark.visibility() << ", " << landmark.presence() << ")\r\n";
  }
}

void logout_world_landmark(const mediapipe::LandmarkList& landmarks, std::vector<int> number_specify, std::string indent) {
  LOG(INFO) << indent << "specify world landmark(" << landmarks.landmark_size() << "): ";
  
  for (int i = 0; i < number_specify.size(); i++) {
    int j = number_specify[i];
    if (j >= landmarks.landmark_size()) {
      continue;
    }
    mediapipe::Landmark landmark = landmarks.landmark(j);
    LOG(INFO) << indent << "index: " << j << " (" << landmark.x() << ", " << landmark.y() << ", " << landmark.z() << ", " << landmark.visibility() << ", " << landmark.presence() << ")\r\n";
  }
}


//------------------------------------------------------------------------------
// coordinate end
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// common start
//------------------------------------------------------------------------------
int getNumberFromKeyCode(int key_code) {
  if (key_code >= 0x30 && key_code <= 0x39) {
    // input 0 - 9
    return key_code - 0x30;
  } else {
    LOG(INFO) << "WARN input key code is unknown, therefore set 0.";
    return 0;
  }
}

std::time_t getEpocTimByTime() {
  return std::time(nullptr);
}

uint64_t getEpocTimeByChrono() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void clearConfigV1_TemplateConfig() {
  // coordinate v1
  key_arrow_down_up_threshold.clear();;
  key_arrow_left_right_threshold.clear();
  // coordinate v2
  action_key_code_list.clear();
  target_key_points.clear();
  threshold_condition_list.clear();
  action_flag_list.clear();
  display_color_list.clear();
  // coordinate v3
  coord_v3_action_key_code_list.clear();
  coord_v3_target_key_points.clear();
  coord_v3_action_flag_list.clear();
  coord_v3_display_color_list.clear();
  // sum v1
  sum_action_key_code_list.clear();
  sum_vector_list.clear();
  sum_target_key_points.clear();
  sum_threshold_condition_list.clear();
  sum_action_flag_list.clear();
  sum_visibility_threshold_list.clear();
  // frame diff v1
  frame_diff_v1_action_key_code_list.clear();
  frame_diff_v1_key_points.clear();
  frame_diff_v1_threshold_condition.clear();
  frame_diff_v1_check_vector.clear();
  frame_diff_v1_check_vector_rate.clear();
  frame_diff_v1_flags.clear();
  frame_diff_v1_last_action_timestamp_list.clear();
  // landmark 2 point dist compare v1
  landmark_2point_dist_compare_v1_action_key_code_list.clear();
  landmark_2point_dist_compare_v1_key_points.clear();
  landmark_2point_dist_compare_v1_flag.clear();
  // mix action v1
  mix_v1_action_key_code_list.clear();
  mix_v1_action_flag.clear();
  mix_v1_action_condition_list.clear();
  mix_v1_last_run_timestamp.clear();
  // mark compare v1
  mark_compare_v1_action_key_code_list.clear();
  mark_compare_v1_action_flag.clear();
  mark_compare_v1_key_points.clear();
  // continue v1
  continue_v1_action_key_code_list.clear();
  continue_v1_action_flag.clear();
  continue_v1_action_condition_list.clear();
  continue_v1_condition_timeout_msec_list.clear();
  continue_v1_last_run_timestamp.clear();
  continue_v1_done_no.clear();
  continue_v1_id_timestamp.clear();
  // vector dist compare v1
  vector_dists_compare_v1_action_key_code_list.clear();
  vector_dists_compare_v1_key_points.clear();
  vector_dists_compare_v1_flag.clear();
}

void changeConfigV1_TemplateConfig(std::string& file_path, int index) {
  clearConfigV1_TemplateConfig();
  
  std::ifstream in_file(file_path);
  if (in_file.is_open() == false) {
    LOG(ERROR) << "json file can not open, please check that file(" << file_path << ").";
    exit(-1);
  }
  nlohmann::json json_data;
  try {
    json_data = nlohmann::json::parse(in_file);
  } catch (nlohmann::json::parse_error& ex) {
    LOG(ERROR) << "json file(" << file_path << ") load error(id: " << ex.id << ", at byte: " << ex.byte << ") context: " << ex.what();
    exit(-1);
  }
  
  //LOG(INFO) << "json_data: " << json_data.dump(2);
  
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_TEMPLATE_CONFIG) {
      loadConfigV1_TemplateConfigs(element.value(), index);
    }
  }
}

void loadConfigV1(std::string& file_path) {
  std::ifstream in_file(file_path);
  if (in_file.is_open() == false) {
    LOG(ERROR) << "json file can not open, please check that file(" << file_path << ").";
    exit(-1);
  }
  nlohmann::json json_data;
  try {
    json_data = nlohmann::json::parse(in_file);
  } catch (nlohmann::json::parse_error& ex) {
    LOG(ERROR) << "json file(" << file_path << ") load error(id: " << ex.id << ", at byte: " << ex.byte << ") context: " << ex.what();
    exit(-1);
  }
  
  LOG(INFO) << "json_data: " << json_data.dump(2);
  
  loadConfigV1_All(json_data);
  
  LOG(INFO) << "json data loaded.";
}

void loadConfigV1_All(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_All() start";
  
  if (json_data.is_object() == false) {
    LOG(INFO) << "load json data is not object, cannot load data.";
    return;
  }
  
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_PC_INFO) {
      loadConfigV1_PCInfo(element.value());
    } else if (element.key() == CONFIG_KEY_ACTION_INFO) {
      loadConfigV1_ActionInfo(element.value());
    } else if (element.key() == CONFIG_KEY_TEMPLATE_CONFIG) {
      loadConfigV1_TemplateConfigs(element.value(), target_TemplateConfig_index);
    }
  }
  
  LOG(INFO) << "loadConfigV1_All() end";
}

void loadConfigV1_PCInfo(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_PCInfo() start";
  
  int count = 1;
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_MAIN_DISPLAY_WIDTH) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      main_display_width = element.value();
    } else if (element.key() == CONFIG_KEY_MAIN_DISPLAY_HEIGHT) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      main_display_height = element.value();
    } else if (element.key() == CONFIG_KEY_NO_MOUSE_ABSOLUTE_RATE) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      no_mouse_absolute_rate = element.value();
    } else if (element.key() == CONFIG_KEY_CAMERA_ID) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      camera_id = element.value();
    } else if (element.key() == CONFIG_KEY_CAMERA_DISPLAY_WIDTH) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      camera_width = element.value();
    } else if (element.key() == CONFIG_KEY_CAMERA_DISPLAY_HEIGHT) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      camera_height = element.value();
    } else if (element.key() == CONFIG_KEY_CAMERA_FPS) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      camera_fps = element.value();
    } else if (element.key() == CONFIG_KEY_CAMERA_PROC_FPS) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      camera_proc_fps = element.value();
    } else if (element.key() == CONFIG_KEY_CAMERA_LOW_PROC_FPS) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      camera_low_proc_fps = element.value();
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_PCInfo() skip config key: " << element.key();
    }
    
    count++;
  }
  
  LOG(INFO) << "loadConfigV1_PCInfo() end";
}

void loadConfigV1_ActionInfo(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_ActionInfo() start";
  
  int count = 1;
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_IS_AUTO_START) {
      if (element.value().is_boolean() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      is_auto_start = element.value();
    } else if (element.key() == CONFIG_KEY_FOCUS_DISPLAY_LEFT) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      focus_display_left = element.value();
    } else if (element.key() == CONFIG_KEY_FOCUS_DISPLAY_TOP) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      focus_display_top = element.value();
    } else if (element.key() == CONFIG_KEY_FOCUS_DISPLAY_RIGHT) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      focus_display_right = element.value();
    } else if (element.key() == CONFIG_KEY_FOCUS_DISPLAY_BOTTOM) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      focus_display_bottom = element.value();
    } else if (element.key() == CONFIG_KEY_IS_DISPLAY_MIRROR) {
      if (element.value().is_boolean() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      is_display_mirror = element.value();
    } else if (element.key() == CONFIG_KEY_DISPLAY_MOSAIC_SCALE) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      display_mosaic_scale = element.value();
    } else if (element.key() == CONFIG_KEY_DISPLAY_FACE_MOSAIC_SCALE) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      display_face_mosaic_scale = element.value();
    } else if (element.key() == CONFIG_KEY_DISPLAY_FACE_MOSAIC_WIDTH) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      display_face_mosaic_width = element.value();
    } else if (element.key() == CONFIG_KEY_DISPLAY_FACE_MOSAIC_HEIGHT) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      display_face_mosaic_height = element.value();
    } else if (element.key() == CONFIG_KEY_FRAME_DIFF_FRAME_MEMORY_MSEC) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      frame_diff_landmarks_max_buffer_msec = element.value();
    } else if (element.key() == CONFIG_KEY_GAME_START_WAIT_MSEC) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      game_start_wait_msec = element.value();
    } else if (element.key() == CONFIG_KEY_PERSON_SIZE_SCALE_COORDINATE) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      person_size_scale_coordinate = element.value();
    } else if (element.key() == CONFIG_KEY_PERSON_SIZE_SCALE_SUM) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      person_size_scale_sum = element.value();
    } else if (element.key() == CONFIG_KEY_PERSON_SIZE_SCALE_FRAME_DIFF) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      person_size_scale_frame_diff = element.value();
    } else if (element.key() == CONFIG_KEY_PERSON_SIZE_SCALE_DIST_COMPARE) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      person_size_scale_dist_compare = element.value();
    } else if (element.key() == CONFIG_KEY_PERSON_SIZE_SCALE_MARK_COMPARE) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      person_size_scale_mark_compare = element.value();
    } else if (element.key() == CONFIG_KEY_PERSON_SIZE_SCALE_VECTOR_DIST_COMPARE) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      person_size_scale_vector_dist_compare = element.value();
    } else if (element.key() == CONFIG_KEY_KEY_GROUP) {
      if (element.value().is_array() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() key(" << element.key() << ") element is not array, skip.";
        continue;
      }
      std::vector<std::vector<int> > key_group = loadConfigV1_ActionCodesList(element.value());
      int count = 0;
      for (int group_id = 0; group_id < key_group.size(); group_id++) {
        for (int j = 0; j < key_group[group_id].size(); j++) {
          key_code_group_id_map[key_group[group_id][j]] = group_id;
        }
        group_id_key_code_map[group_id] = key_group[group_id];
        latest_running_key_code_group_map[group_id] = 0;
      }
    } else if (element.key() == CONFIG_KEY_IS_ENABLE_CHANGE_X_FRONT_REVERSE) {
      if (element.value().is_boolean() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not boolean, skip.";
        continue;
      }
      is_enable_change_x_front_reverse = element.value();
    } else if (element.key() == CONFIG_KEY_X_FRONT_REVERSE_CHECK_MSEC) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      x_front_reverse_check_msec = element.value();
    } else if (element.key() == CONFIG_KEY_EYES_DIST) {
      if (element.value().is_number() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
        continue;
      }
      eyes_dist = element.value();
    } else if (element.key() == CONFIG_KEY_LOGOUT_COORDINATE_TYPES) {
      if (element.value().is_array() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not array, skip.";
        continue;
      }
      for (auto& child_element : element.value()) {
        if (child_element.is_number() == false) {
          // ignore
          LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
          continue;
        }
        logout_type_specify.push_back(child_element);
      }
    } else if (element.key() == CONFIG_KEY_LOGOUT_COORDINATE_NUMBERS) {
      if (element.value().is_array() == false) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not array, skip.";
        continue;
      }
      for (auto& child_element : element.value()) {
        if (child_element.is_number() == false) {
          // ignore
          LOG(ERROR) << "WARN: loadConfigV1_PCInfo() key(" << element.key() << ") element is not number, skip.";
          continue;
        }
        logout_number_specify.push_back(child_element);
      }
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_ActionInfo() skip config key: " << element.key();
    }
    
    count++;
  }
  
  LOG(INFO) << "loadConfigV1_ActionInfo() end";
}

void loadConfigV1_TemplateConfigs(nlohmann::json& json_data, int target_index) {
  LOG(INFO) << "loadConfigV1_TemplateConfigs() start";
  
  bool is_detect_target_index = false;
  
  int count = 0;
  for (auto& template_element : json_data) {
    
    if (count != target_index) {
      count++;
      continue;
    }
    
    loadConfigV1_TemplateConfig(template_element, count);
    
    is_detect_target_index = true;
    
    count++;
  }
  
  enable_TemplateConfig_index = count - 1;
  
  if (is_detect_target_index == false) {
    LOG(ERROR) << "ERROR: can not specify index(" << target_index << ") of TemplateConfig, enable index is 0 - " << count << ", please check.";
    exit(-1);
  }
  
  LOG(INFO) << "loadConfigV1_TemplateConfigs() end";
}

void loadConfigV1_TemplateConfig(nlohmann::json& json_data, int index) {
  LOG(INFO) << "loadConfigV1_TemplateConfig() start";
  
  // TODO ログ出力用に親のtarget_indexを引数で指定してもらい、ログ出力する
  int child_index = 0;
  for (auto& template_child_element : json_data) {
    for (auto& element : template_child_element.items()) {
      if (element.key() == CONFIG_KEY_COORDINATE_V1) {
      } else if (element.key() == CONFIG_KEY_COORDINATE_V2) {
        loadConfigV1_CoordinateV2(element.value(), index, child_index);
      } else if (element.key() == CONFIG_KEY_COORDINATE_V3) {
        loadConfigV1_CoordinateV3(element.value(), index, child_index);
      } else if (element.key() == CONFIG_KEY_SUM_V1) {
        loadConfigV1_SumV1(element.value(), index, child_index);
      } else if (element.key() == CONFIG_KEY_FRAME_DIFF_V1) {
        loadConfigV1_FrameDiffV1(element.value(), index, child_index);
      } else if (element.key() == CONFIG_KEY_DIST_COMPARE_V1) {
        loadConfigV1_DistCompareV1(element.value(), index, child_index);
      } else if (element.key() == CONFIG_KEY_MIX_V1) {
        loadConfigV1_MixV1(element.value(), index, child_index);
      } else if (element.key() == CONFIG_KEY_MARK_COMPARE_V1) {
        loadConfigV1_MarkCompareV1(element.value(), index, child_index);
      } else if (element.key() == CONFIG_KEY_CONTINUE_V1) {
        loadConfigV1_ContinueV1(element.value(), index, child_index);
      } else if (element.key() == CONFIG_KEY_VECTOR_DIST_COMPARE_V1) {
        loadConfigV1_VectorDistCompareV1(element.value(), index, child_index);
      } else {
        LOG(ERROR) << "WARN: loadConfigV1_TemplateConfig() skip config key: " << element.key();
      }
    }
    child_index++;
  }
  
  LOG(INFO) << "loadConfigV1_TemplateConfig() end";
}

void loadConfigV1_CoordinateV2(nlohmann::json& json_data, int index, int child_index) {
  LOG(INFO) << "loadConfigV1_CoordinateV2() start";
  
  bool is_exist_action_key_codes = false;
  bool is_exist_key_points = false;
  bool is_exist_threshold_condition = false;
  bool is_exist_display_color = false;
  bool is_exist_action_flag = false;
  
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_ACTION_CODES) {
      action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
      is_exist_action_key_codes = true;
    } else if (element.key() == CONFIG_KEY_KEY_POINTS) {
      target_key_points.push_back(loadConfigV1_KeyPoints(element.value()));
      is_exist_key_points = true;
    } else if (element.key() == CONFIG_KEY_THRESHOLD_CONDITIONS) {
      threshold_condition_list.push_back(loadConfigV1_ThresholdCondition(element.value()));
      is_exist_threshold_condition = true;
    } else if (element.key() == CONFIG_KEY_DISPLAY_COLOR) {
      int color = loadConfigV1_DisplayColor(element.value());
      cv::Scalar color_scalar = getScalarColorFromInt(color);
      display_color_list.push_back(color_scalar);
      is_exist_display_color = true;
    } else if (element.key() == CONFIG_KEY_ACTION_FLAG) {
      action_flag_list.push_back(loadConfigV1_ActionFlag(element.value()));
      is_exist_action_flag = true;
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_CoordinateV2() skip config key: " << element.key();
    }
  }
  
  if (is_exist_action_key_codes == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_CoordinateV2() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_CODES << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_key_points == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_CoordinateV2() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_KEY_POINTS << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_threshold_condition == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_CoordinateV2() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_THRESHOLD_CONDITIONS << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_display_color == false) {
    LOG(INFO) << "INFO: loadConfigV1_CoordinateV2() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_DISPLAY_COLOR << ") is not exist, set random color.";
    int color = getColorFromPaletto();
    cv::Scalar color_scalar = getScalarColorFromInt(color);
    display_color_list.push_back(color_scalar);
  }
  if (is_exist_action_flag == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_CoordinateV2() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_FLAG << ") is not exist, stop program.";
    exit(-1);
  }
  
  LOG(INFO) << "loadConfigV1_CoordinateV2() end";
}

std::vector<std::vector<int> > loadConfigV1_ActionCodesList(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_ActionCodesList() start";
  
  std::vector<std::vector<int> > action_codes_list;
  
  if (json_data.is_array() == false) {
    // ignore
    LOG(ERROR) << "WARN: loadConfigV1_ActionCodes() element is not array, skip.";
    return action_codes_list;
  }
  
  for (auto& element : json_data) {
    if (element.is_array() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_ActionCodes() child element is not array, skip.";
      continue;
    }
    action_codes_list.push_back(loadConfigV1_ActionCodes(element));
  }
  
  LOG(INFO) << "loadConfigV1_ActionCodesList() end";
  
  return action_codes_list;
}

int toIntFromStringHex(std::string str_element) {
  //if (str_element.size() != 10 && str_element.size() != 4 && str_element.size() != 2) {
  //  // warn
  //  LOG(ERROR) << "WARN: toIntFromStringHex() element string(" << str_element << ") size(" << str_element.size() << ") is not 10('0x00000000') or 4('0x0000') 2('0x00'), set -1.";
  //  return -1;
  //}
  if (str_element.find("0x") != 0) {
    // warn
    LOG(ERROR) << "WARN: toIntFromStringHex() element is not start with '0x', set -1.";
    return -1;
  }
  // 16進数文字列を数値に変換
  return strtol(str_element.c_str(), NULL, 0);
}

int loadConfigV1_ActionCode(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_ActionCode() start";
  
  int action_code = -1;
  
  if (json_data.is_number() == true) {
    action_code = json_data;
  } else if (json_data.is_string() == true) {
    std::string str_element = json_data;
    if (str_element.size() != 10 && str_element.size() != 4 && str_element.size() != 2) {
      // warn
      LOG(ERROR) << "WARN: loadConfigV1_ActionCode() element string(" << str_element << ") size(" << str_element.size() << ") is not 10('0x00000000') or 4('0x0000') 2('0x00'), set -1.";
      return -1;
    }
    if (str_element.find("0x") != 0) {
      // warn
      LOG(ERROR) << "WARN: loadConfigV1_ActionCode() element is not start with '0x', set -1.";
      return -1;
    }
    action_code = toIntFromStringHex(str_element);
  } else {
    // warn
    LOG(ERROR) << "WARN: loadConfigV1_ActionCode() element is not number or string, set -1.";
    return -1;
  }
  
  LOG(INFO) << "loadConfigV1_ActionCode() end";
  
  return action_code;
}

std::vector<int> loadConfigV1_ActionCodes(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_ActionCodes() start";
  
  std::vector<int> action_codes;
  
  for (auto& element : json_data) {
    if (element.is_number() == true) {
      action_codes.push_back(element);
    } else if (element.is_string() == true) {
      std::string str_element = element;
      if (str_element.size() != 10 && str_element.size() != 4 && str_element.size() != 2) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionCodes() element string(" << str_element << ") size(" << str_element.size() << ") is not 10('0x00000000') or 4('0x0000') 2('0x00'), skip.";
        continue;
      }
      if (str_element.find("0x") != 0) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionCodes() element is not start with '0x', skip.";
        continue;
      }
      int int_element = toIntFromStringHex(str_element);
      action_codes.push_back(int_element);
    } else {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_ActionCodes() element is not number or string, skip.";
      continue;
    }
  }
  
  LOG(INFO) << "loadConfigV1_ActionCodes() end";
  
  return action_codes;
}

std::vector<std::vector<int> > loadConfigV1_KeyPointsList(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_KeyPointsList() start";
  
  std::vector<std::vector<int> > key_points_list;
  
  if (json_data.is_array() == false) {
    // ignore
    LOG(ERROR) << "WARN: loadConfigV1_KeyPointsList() element is not array, skip.";
    return key_points_list;
  }
  
  for (auto& element : json_data) {
    if (element.is_array() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPointsList() child element is not array, skip.";
      continue;
    }
    key_points_list.push_back(loadConfigV1_KeyPoints(element));
  }
  
  LOG(INFO) << "loadConfigV1_KeyPointsList() end";
  
  return key_points_list;
}

std::vector<int> loadConfigV1_KeyPoints(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_KeyPoints() start";
  
  std::vector<int> key_points;
  
  int count = 1;
  for (auto& element : json_data) {
    if (element.is_number() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPoints() element is not number, skip.";
      continue;
    }
    if (count > CONFIG_KEY_COORDINATE_V2_KEY_POINTS_MAX_SIZE) {
      LOG(ERROR) << "WARN: loadConfigV1_KeyPoints() element size is max size(" << CONFIG_KEY_COORDINATE_V2_KEY_POINTS_MAX_SIZE << "), skip.";
      continue;
    }
    
    key_points.push_back(element);
    
    count++;
  }
  
  LOG(INFO) << "loadConfigV1_KeyPoints() end";
  
  return key_points;
}

std::vector<std::pair<float, int> > loadConfigV1_ThresholdConditionList(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_ThresholdConditionList() start";
  
  std::vector<std::pair<float, int> > threshold_condition_list;
  
  if (json_data.is_array() == false) {
    // ignore
    LOG(ERROR) << "WARN: loadConfigV1_ThresholdConditionList() element is not array, skip.";
    return threshold_condition_list;
  }
  
  for (auto& element : json_data) {
    if (element.is_object() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_ThresholdConditionList() child element is not object, skip.";
      continue;
    }
    threshold_condition_list.push_back(loadConfigV1_ThresholdCondition(element));
  }
  
  LOG(INFO) << "loadConfigV1_ThresholdConditionList() end";
  
  return threshold_condition_list;
}

std::pair<float, int> loadConfigV1_ThresholdCondition(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_ThresholdCondition() start";
  
  std::pair<float, int> threshold_condition;
  
  int count = 1;
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_THRESHOLD) {
      threshold_condition.first = element.value();
    } else if (element.key() == CONFIG_KEY_CONDITION) {
      threshold_condition.second = element.value();
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_ThresholdCondition() skip config key: " << element.key();
    }
    
    count++;
  }
  
  LOG(INFO) << "loadConfigV1_ThresholdCondition() end";
  
  return threshold_condition;
}

float loadConfigV1_VisibilityThreshold(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_VisibilityThreshold() start";
  
  float visibility_threshold = -1.0;
  
  if (json_data.is_number() == true) {
    visibility_threshold = json_data;
  } else {
    // warn
    LOG(ERROR) << "WARN: loadConfigV1_VisibilityThreshold() element is not number, set -1.0.";
    return -1.0;
  }
  
  LOG(INFO) << "loadConfigV1_VisibilityThreshold() end";
  
  return visibility_threshold;
}

std::vector<int> loadConfigV1_ActionFlagList(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_ActionFlagList() start";
  
  std::vector<int> action_flag_list;
  
  if (json_data.is_array() == false) {
    // ignore
    LOG(ERROR) << "WARN: loadConfigV1_ActionFlagList() element is not array, skip.";
    return action_flag_list;
  }
  
  for (auto& element : json_data) {
    if (element.is_number() == true) {
      action_flag_list.push_back(element);
    } else if (element.is_string() == true) {
      std::string str_element = element;
      if (str_element.size() != 10 && str_element.size() != 4 && str_element.size() != 2) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionFlagList() element string(" << str_element << ") size(" << str_element.size() << ") is not 10('0x00000000') or 4('0x0000') 2('0x00'), skip.";
        continue;
      }
      if (str_element.find("0x") != 0) {
        // ignore
        LOG(ERROR) << "WARN: loadConfigV1_ActionFlagList() element is not start with '0x', skip.";
        continue;
      }
      int int_element = toIntFromStringHex(str_element);
      action_flag_list.push_back(loadConfigV1_ActionFlag(element));
    } else {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_ActionFlagList() element is not number or string, skip.";
      continue;
    }
  }
  
  LOG(INFO) << "loadConfigV1_ActionFlagList() end";
  
  return action_flag_list;
}

int loadConfigV1_ActionFlag(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_ActionFlag() start";
  
  int action_flag = -1;
  
  if (json_data.is_number() == true) {
    action_flag = json_data;
  } else if (json_data.is_string() == true) {
    std::string str_element = json_data;
    if (str_element.size() != 10 && str_element.size() != 4 && str_element.size() != 2) {
      // warn
      LOG(ERROR) << "WARN: loadConfigV1_ActionFlag() element string(" << str_element << ") size(" << str_element.size() << ") is not 10('0x00000000') or 4('0x0000') 2('0x00'), set -1.";
      return -1;
    }
    if (str_element.find("0x") != 0) {
      // warn
      LOG(ERROR) << "WARN: loadConfigV1_ActionFlag() element is not start with '0x', set -1.";
      return -1;
    }
    action_flag = toIntFromStringHex(str_element);
  } else {
    // warn
    LOG(ERROR) << "WARN: loadConfigV1_ActionFlag() element is not number or string, set -1.";
    return -1;
  }
  
  LOG(INFO) << "loadConfigV1_ActionFlag() end";
  
  return action_flag;
}

int loadConfigV1_DisplayColor(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_DisplayColor() start";
  
  int display_color = -1;
  
  if (json_data.is_number() == true) {
    display_color = json_data;
  } else if (json_data.is_string() == true) {
    std::string str_element = json_data;
    //if (str_element.size() != 10 && str_element.size() != 4 && str_element.size() != 2) {
    //  // warn
    //  LOG(ERROR) << "WARN: loadConfigV1_DisplayColor() element string(" << str_element << ") size(" << str_element.size() << ") is not 10('0x00000000') or 4('0x0000') 2('0x00'), set random value.";
    //}
    //if (str_element.find("0x") != 0) {
    //  // warn
    //  LOG(ERROR) << "WARN: loadConfigV1_DisplayColor() element is not start with '0x', set random value.";
    //}
    display_color = toIntFromStringHex(str_element);
  } else {
    // warn
    LOG(ERROR) << "WARN: loadConfigV1_DisplayColor() element is not number or string, set random value.";
  }
  
  if (display_color == -1) {
    LOG(ERROR) << "WARN: loadConfigV1_DisplayColor() display_color is -1, set random value.";
    int display_color = getColorFromPaletto();
  }
  
  LOG(INFO) << "loadConfigV1_DisplayColor() end";
  
  return display_color;
}

void loadConfigV1_CoordinateV3(nlohmann::json& json_data, int index, int child_index) {
  LOG(INFO) << "loadConfigV1_CoordinateV3() start";
  
  bool is_exist_action_key_codes = false;
  bool is_exist_key_points = false;
  bool is_exist_display_color = false;
  bool is_exist_action_flag = false;
  
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_ACTION_CODES) {
      coord_v3_action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
      is_exist_action_key_codes = true;
    } else if (element.key() == CONFIG_KEY_KEY_POINTS) {
      coord_v3_target_key_points.push_back(loadConfigV1_KeyPointsV3(element.value()));
      is_exist_key_points = true;
    } else if (element.key() == CONFIG_KEY_DISPLAY_COLOR) {
      int color = loadConfigV1_DisplayColor(element.value());
      cv::Scalar color_scalar = getScalarColorFromInt(color);
      display_color_list.push_back(color_scalar);
      is_exist_display_color = true;
    } else if (element.key() == CONFIG_KEY_ACTION_FLAG) {
      coord_v3_action_flag_list.push_back(loadConfigV1_ActionFlag(element.value()));
      is_exist_action_flag = true;
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_CoordinateV3() skip config key: " << element.key();
    }
  }
  
  if (is_exist_action_key_codes == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_CoordinateV3() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_CODES << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_key_points == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_CoordinateV3() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_KEY_POINTS << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_display_color == false) {
    LOG(INFO) << "INFO: loadConfigV1_CoordinateV3() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_DISPLAY_COLOR << ") is not exist, set random color.";
    int color = getColorFromPaletto();
    cv::Scalar color_scalar = getScalarColorFromInt(color);
    display_color_list.push_back(color_scalar);
  }
  if (is_exist_action_flag == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_CoordinateV3() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_FLAG << ") is not exist, stop program.";
    exit(-1);
  }
  
  LOG(INFO) << "loadConfigV1_CoordinateV3() end";
}

std::vector<std::vector<int> > loadConfigV1_KeyPointsListV3(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_KeyPointsListV3() start";
  
  std::vector<std::vector<int> > key_points_list;
  
  if (json_data.is_array() == false) {
    // ignore
    LOG(ERROR) << "WARN: loadConfigV1_KeyPointsListV3() element is not array, skip.";
    return key_points_list;
  }
  
  for (auto& element : json_data) {
    if (element.is_array() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPointsListV3() child element is not array, skip.";
      continue;
    }
    key_points_list.push_back(loadConfigV1_KeyPointsV3(element));
  }
  
  LOG(INFO) << "loadConfigV1_KeyPointsListV3() end";
  
  return key_points_list;
}

std::vector<int> loadConfigV1_KeyPointsV3(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_KeyPointsV3() start";
  
  std::vector<int> key_points;
  
  int count = 1;
  for (auto& element : json_data) {
    if (element.is_number() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPointsV3() element is not number, skip.";
      continue;
    }
    
    key_points.push_back(element);
    
    count++;
  }
  
  if (key_points.size() % CONFIG_KEY_COORDINATE_V3_KEY_POINTS_MULTIPLE_SIZE != 0) {
    LOG(ERROR) << "WARN: loadConfigV1_KeyPointsV3() element size(" << key_points.size() << ") is not multiple size(" << CONFIG_KEY_COORDINATE_V3_KEY_POINTS_MULTIPLE_SIZE << "), erase over data.";
    for (int i = 0; i < key_points.size() % CONFIG_KEY_COORDINATE_V3_KEY_POINTS_MULTIPLE_SIZE; i++) {
      key_points.pop_back();
    }
  }
  
  LOG(INFO) << "loadConfigV1_KeyPointsV3() end";
  
  return key_points;
}

void loadConfigV1_SumV1(nlohmann::json& json_data, int index, int child_index) {
  LOG(INFO) << "loadConfigV1_SumV1() start";
  
  bool is_exist_action_key_codes = false;
  bool is_exist_key_points = false;
  bool is_exist_threshold_condition = false;
  bool is_exist_visibility_threshold = false;
  bool is_exist_action_flag = false;
  
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_ACTION_CODES) {
      sum_action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
      is_exist_action_key_codes = true;
    } else if (element.key() == CONFIG_KEY_KEY_POINTS) {
      sum_target_key_points.push_back(loadConfigV1_KeyPointsSumV1(element.value()));
      is_exist_key_points = true;
    } else if (element.key() == CONFIG_KEY_THRESHOLD_CONDITIONS) {
      sum_threshold_condition_list.push_back(loadConfigV1_ThresholdCondition(element.value()));
      is_exist_threshold_condition = true;
    } else if (element.key() == CONFIG_KEY_VISIBILITY_THRESHOLD) {
      sum_visibility_threshold_list.push_back(loadConfigV1_VisibilityThreshold(element.value()));
      is_exist_visibility_threshold = true;
    } else if (element.key() == CONFIG_KEY_ACTION_FLAG) {
      sum_action_flag_list.push_back(loadConfigV1_ActionFlag(element.value()));
      is_exist_action_flag = true;
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_SumV1() skip config key: " << element.key();
    }
  }
  
  if (is_exist_action_key_codes == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_SumV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_CODES << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_key_points == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_SumV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_KEY_POINTS << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_threshold_condition == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_SumV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_THRESHOLD_CONDITIONS << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_visibility_threshold == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_SumV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_VISIBILITY_THRESHOLD << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_action_flag == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_SumV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_FLAG << ") is not exist, stop program.";
    exit(-1);
  }
  // TODO
  
  LOG(INFO) << "loadConfigV1_SumV1() end";
}

std::vector<std::vector<int> > loadConfigV1_KeyPointsListVN(nlohmann::json& json_data, int array_size) {
  LOG(INFO) << "loadConfigV1_KeyPointsListVN() start";
  
  std::vector<std::vector<int> > key_points_list;
  
  if (json_data.is_array() == false) {
    // ignore
    LOG(ERROR) << "WARN: loadConfigV1_KeyPointsListVN() element is not array, skip.";
    return key_points_list;
  }
  
  for (auto& element : json_data) {
    if (element.is_array() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPointsListVN() child element is not array, skip.";
      continue;
    }
    key_points_list.push_back(loadConfigV1_KeyPointsVN(element, array_size));
  }
  
  LOG(INFO) << "loadConfigV1_KeyPointsListVN() end";
  
  return key_points_list;
}

std::vector<int> loadConfigV1_KeyPointsVN(nlohmann::json& json_data, int array_size) {
  LOG(INFO) << "loadConfigV1_KeyPointsVN() start";
  
  std::vector<int> key_points;
  
  int count = 1;
  for (auto& element : json_data) {
    if (element.is_number() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPointsVN() element is not number, skip.";
      continue;
    }
    
    key_points.push_back(element);
    
    count++;
  }
  
  if (key_points.size() % array_size != 0) {
    LOG(ERROR) << "WARN: loadConfigV1_KeyPointsVN() element size(" << key_points.size() << ") is not multiple size(" << array_size << "), erase over data.";
    for (int i = 0; i < key_points.size() % array_size; i++) {
      key_points.pop_back();
    }
  }
  
  LOG(INFO) << "loadConfigV1_KeyPointsVN() end";
  
  return key_points;
}

std::vector<std::vector<int> > loadConfigV1_KeyPointsListSumV1(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_KeyPointsListSumV1() start";
  
  std::vector<std::vector<int> > key_points_list;
  
  if (json_data.is_array() == false) {
    // ignore
    LOG(ERROR) << "WARN: loadConfigV1_KeyPointsListSumV1() element is not array, skip.";
    return key_points_list;
  }
  
  for (auto& element : json_data) {
    if (element.is_array() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPointsListSumV1() child element is not array, skip.";
      continue;
    }
    key_points_list.push_back(loadConfigV1_KeyPointsSumV1(element));
  }
  
  LOG(INFO) << "loadConfigV1_KeyPointsListSumV1() end";
  
  return key_points_list;
}

std::vector<int> loadConfigV1_KeyPointsSumV1(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_KeyPointsSumV1() start";
  
  std::vector<int> key_points;
  
  int count = 1;
  for (auto& element : json_data) {
    if (element.is_number() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPointsSumV1() element is not number, skip.";
      continue;
    }
    
    key_points.push_back(element);
    
    count++;
  }
  
  if (key_points.size() % CONFIG_KEY_SUM_V1_KEY_POINTS_MULTIPLE_SIZE != 0) {
    LOG(ERROR) << "WARN: loadConfigV1_KeyPointsSumV1() element size(" << key_points.size() << ") is not multiple size(" << CONFIG_KEY_SUM_V1_KEY_POINTS_MULTIPLE_SIZE << "), erase over data.";
    for (int i = 0; i < key_points.size() % CONFIG_KEY_SUM_V1_KEY_POINTS_MULTIPLE_SIZE; i++) {
      key_points.pop_back();
    }
  }
  
  LOG(INFO) << "loadConfigV1_KeyPointsSumV1() end";
  
  return key_points;
}

void loadConfigV1_FrameDiffV1(nlohmann::json& json_data, int index, int child_index) {
  LOG(INFO) << "loadConfigV1_FrameDiffV1() start";
  
  bool is_exist_action_key_codes = false;
  bool is_exist_key_points = false;
  bool is_exist_threshold_condition = false;
  bool is_exist_check_vector = false;
  bool is_exist_check_vector_rate = false;
  bool is_exist_action_flag = false;
  
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_ACTION_CODES) {
      frame_diff_v1_action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
      is_exist_action_key_codes = true;
    } else if (element.key() == CONFIG_KEY_KEY_POINTS) {
      frame_diff_v1_key_points.push_back(loadConfigV1_KeyPointsFrameDiffV1(element.value()));
      is_exist_key_points = true;
    } else if (element.key() == CONFIG_KEY_THRESHOLD_CONDITIONS) {
      frame_diff_v1_threshold_condition.push_back(loadConfigV1_ThresholdCondition(element.value()));
      is_exist_threshold_condition = true;
    } else if (element.key() == CONFIG_KEY_CHECK_VECTOR) {
      frame_diff_v1_check_vector.push_back(loadConfigV1_ThresholdConditionList(element.value()));
      is_exist_check_vector = true;
    } else if (element.key() == CONFIG_KEY_CHECK_VECTOR_RATE) {
      frame_diff_v1_check_vector_rate.push_back(loadConfigV1_ThresholdConditionList(element.value()));
      is_exist_check_vector_rate = true;
    } else if (element.key() == CONFIG_KEY_ACTION_FLAG) {
      frame_diff_v1_flags.push_back(loadConfigV1_ActionFlag(element.value()));
      is_exist_action_flag = true;
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_FrameDiffV1() skip config key: " << element.key();
    }
  }
  
  if (is_exist_action_key_codes == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_FrameDiffV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_CODES << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_key_points == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_FrameDiffV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_KEY_POINTS << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_threshold_condition == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_FrameDiffV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_THRESHOLD_CONDITIONS << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_check_vector == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_FrameDiffV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_CHECK_VECTOR << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_check_vector_rate == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_FrameDiffV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_CHECK_VECTOR_RATE << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_action_flag == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_FrameDiffV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_FLAG << ") is not exist, stop program.";
    exit(-1);
  }
  
  LOG(INFO) << "loadConfigV1_FrameDiffV1() end";
}

std::vector<std::vector<int> > loadConfigV1_KeyPointsListFrameDiffV1(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_KeyPointsListFrameDiffV1() start";
  
  std::vector<std::vector<int> > key_points_list;
  
  if (json_data.is_array() == false) {
    // ignore
    LOG(ERROR) << "WARN: loadConfigV1_KeyPointsListFrameDiffV1() element is not array, skip.";
    return key_points_list;
  }
  
  for (auto& element : json_data) {
    if (element.is_array() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPointsListFrameDiffV1() child element is not array, skip.";
      continue;
    }
    key_points_list.push_back(loadConfigV1_KeyPointsFrameDiffV1(element));
  }
  
  LOG(INFO) << "loadConfigV1_KeyPointsListFrameDiffV1() end";
  
  return key_points_list;
}

std::vector<int> loadConfigV1_KeyPointsFrameDiffV1(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_KeyPointsFrameDiffV1() start";
  
  std::vector<int> key_points;
  
  int count = 1;
  for (auto& element : json_data) {
    if (element.is_number() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPointsFrameDiffV1() element is not number, skip.";
      continue;
    }
    
    key_points.push_back(element);
    
    count++;
  }
  
  LOG(INFO) << "loadConfigV1_KeyPointsFrameDiffV1() end";
  
  return key_points;
}

std::vector<std::vector<std::pair<float, int> > > loadConfigV1_ThresholdConditionListList(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_ThresholdConditionListList() start";
  
  std::vector<std::vector<std::pair<float, int> > > threshold_condition_list_list;
  
  if (json_data.is_array() == false) {
    // ignore
    LOG(ERROR) << "WARN: loadConfigV1_ThresholdConditionListList() element is not array, skip.";
    return threshold_condition_list_list;
  }
  
  for (auto& element : json_data) {
    if (element.is_array() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_ThresholdConditionListList() child element is not object, skip.";
      continue;
    }
    threshold_condition_list_list.push_back(loadConfigV1_ThresholdConditionList(element));
  }
  
  LOG(INFO) << "loadConfigV1_ThresholdConditionListList() end";
  
  return threshold_condition_list_list;
}

void loadConfigV1_DistCompareV1(nlohmann::json& json_data, int index, int child_index) {
  LOG(INFO) << "loadConfigV1_DistCompareV1() start";
  
  bool is_exist_action_key_codes = false;
  bool is_exist_key_points = false;
  bool is_exist_action_flag = false;
  
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_ACTION_CODES) {
      landmark_2point_dist_compare_v1_action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
      is_exist_action_key_codes = true;
    } else if (element.key() == CONFIG_KEY_KEY_POINTS) {
      landmark_2point_dist_compare_v1_key_points.push_back(loadConfigV1_KeyPointsV3(element.value()));
      is_exist_key_points = true;
    } else if (element.key() == CONFIG_KEY_ACTION_FLAG) {
      landmark_2point_dist_compare_v1_flag.push_back(loadConfigV1_ActionFlag(element.value()));
      is_exist_action_flag = true;
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_DistCompareV1() skip config key: " << element.key();
    }
  }
  
  if (is_exist_action_key_codes == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_DistCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_CODES << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_key_points == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_DistCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_KEY_POINTS << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_action_flag == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_DistCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_FLAG << ") is not exist, stop program.";
    exit(-1);
  }
  
  LOG(INFO) << "loadConfigV1_DistCompareV1() end";
}

void loadConfigV1_MixV1(nlohmann::json& json_data, int index, int child_index) {
  LOG(INFO) << "loadConfigV1_MixV1() start";
  
  bool is_exist_action_key_codes = false;
  bool is_exist_action_condition = false;
  bool is_exist_action_flag = false;
  
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_ACTION_CODES) {
      mix_v1_action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
      is_exist_action_key_codes = true;
    } else if (element.key() == CONFIG_KEY_ACTION_CONDITIONS) {
      mix_v1_action_condition_list.push_back(loadConfigV1_ActionCodes(element.value()));
      is_exist_action_condition = true;
    } else if (element.key() == CONFIG_KEY_ACTION_FLAG) {
      mix_v1_action_flag.push_back(loadConfigV1_ActionFlag(element.value()));
      is_exist_action_flag = true;
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_MixV1() skip config key: " << element.key();
    }
  }
  
  if (is_exist_action_key_codes == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_MixV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_CODES << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_action_condition == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_MixV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_CONDITIONS << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_action_flag == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_MixV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_FLAG << ") is not exist, stop program.";
    exit(-1);
  }
  
  LOG(INFO) << "loadConfigV1_MixV1() end";
}

std::vector<int> loadConfigV1_KeyPointsV4(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_KeyPointsV4() start";
  
  std::vector<int> key_points;
  
  int count = 1;
  for (auto& element : json_data) {
    if (element.is_number() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPointsV4() element is not number, skip.";
      continue;
    }
    
    key_points.push_back(element);
    
    count++;
  }
  
  if (key_points.size() % CONFIG_KEY_MARK_COMPARE_V1_KEY_POINTS_MULTIPLE_SIZE != 0) {
    LOG(ERROR) << "WARN: loadConfigV1_KeyPointsV4() element size(" << key_points.size() << ") is not multiple size(" << CONFIG_KEY_MARK_COMPARE_V1_KEY_POINTS_MULTIPLE_SIZE << "), erase over data.";
    for (int i = 0; i < key_points.size() % CONFIG_KEY_MARK_COMPARE_V1_KEY_POINTS_MULTIPLE_SIZE; i++) {
      key_points.pop_back();
    }
  }
  
  LOG(INFO) << "loadConfigV1_KeyPointsV4() end";
  
  return key_points;
}

void loadConfigV1_MarkCompareV1(nlohmann::json& json_data, int index, int child_index) {
  LOG(INFO) << "loadConfigV1_MarkCompareV1() start";
  
  bool is_exist_action_key_codes = false;
  //bool is_exist_not_action_key_codes = false;
  bool is_exist_key_points = false;
  //bool is_exist_neutral_key_group_id = false;
  bool is_exist_action_flag = false;
  
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_ACTION_CODES) {
      mark_compare_v1_action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
      is_exist_action_key_codes = true;
    //} else if (element.key() == CONFIG_KEY_NOT_ACTION_CODES) {
    //  mark_compare_v1_not_action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
    //  is_exist_not_action_key_codes = true;
    } else if (element.key() == CONFIG_KEY_KEY_POINTS) {
      mark_compare_v1_key_points.push_back(loadConfigV1_KeyPointsV4(element.value()));
      is_exist_key_points = true;
    //} else if (element.key() == CONFIG_KEY_NEUTRAL_KEY_GROUP_ID) {
    //  mark_compare_v1_neutral_key_group_ids.push_back(loadConfigV1_ActionFlag(element.value()));
    //  is_exist_neutral_key_group_id = true;
    } else if (element.key() == CONFIG_KEY_ACTION_FLAG) {
      mark_compare_v1_action_flag.push_back(loadConfigV1_ActionFlag(element.value()));
      is_exist_action_flag = true;
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_MarkCompareV1() skip config key: " << element.key();
    }
  }
  
  if (is_exist_action_key_codes == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_MarkCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_CODES << ") is not exist, stop program.";
    exit(-1);
  }
  //if (is_exist_not_action_key_codes == false) {
  //  LOG(ERROR) << "ERROR: loadConfigV1_MarkCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_NOT_ACTION_CODES << ") is not exist, stop program.";
  //  exit(-1);
  //}
  if (is_exist_key_points == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_MarkCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_KEY_POINTS << ") is not exist, stop program.";
    exit(-1);
  }
  //if (is_exist_neutral_key_group_id == false) {
  //  LOG(ERROR) << "ERROR: loadConfigV1_MarkCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_NEUTRAL_KEY_GROUP_ID << ") is not exist, stop program.";
  //  exit(-1);
  //}
  if (is_exist_action_flag == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_MarkCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_FLAG << ") is not exist, stop program.";
    exit(-1);
  }
  
  LOG(INFO) << "loadConfigV1_MarkCompareV1() end";
}

std::vector<int> loadConfigV1_VecIntV1(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_VecIntV1() start";
  
  std::vector<int> vec_int;
  
  for (auto& element : json_data) {
    if (element.is_number() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_VecIntV1() element is not number, skip.";
      continue;
    }
    
    vec_int.push_back(element);
  }
  
  LOG(INFO) << "loadConfigV1_VecIntV1() end";
  
  return vec_int;
}

std::vector<std::vector<int> > loadConfigV1_VecVecIntV1(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_VecVecIntV1() start";
  
  std::vector<std::vector<int> > vec_vec_int;
  
  if (json_data.is_array() == false) {
    // ignore
    LOG(ERROR) << "WARN: loadConfigV1_VecVecIntV1() element is not array, skip.";
    return vec_vec_int;
  }
  
  for (auto& element : json_data) {
    if (element.is_array() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_VecVecIntV1() child element is not array, skip.";
      continue;
    }
    vec_vec_int.push_back(loadConfigV1_VecIntV1(element));
  }
  
  LOG(INFO) << "loadConfigV1_VecVecIntV1() end";
  
  return vec_vec_int;
}

void loadConfigV1_ContinueV1(nlohmann::json& json_data, int index, int child_index) {
  LOG(INFO) << "loadConfigV1_ContinueV1() start";
  
  bool is_exist_action_key_codes = false;
  //bool is_exist_not_action_key_codes = false;
  bool is_exist_action_condition = false;
  bool is_exist_condition_timeout_msec = false;
  //bool is_exist_neutral_key_group_id = false;
  bool is_exist_action_flag = false;
  
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_ACTION_CODES) {
      continue_v1_action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
      is_exist_action_key_codes = true;
    //} else if (element.key() == CONFIG_KEY_NOT_ACTION_CODES) {
    //  continue_v1_not_action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
    //  is_exist_not_action_key_codes = true;
    } else if (element.key() == CONFIG_KEY_ACTION_CONDITIONS) {
      continue_v1_action_condition_list.push_back(loadConfigV1_ActionCodes(element.value()));
      is_exist_action_condition = true;
    } else if (element.key() == CONFIG_KEY_CONDITIONS_TIMEOUT_MSEC) {
      continue_v1_condition_timeout_msec_list.push_back(loadConfigV1_ActionCodes(element.value())); // TODO modify★
      is_exist_condition_timeout_msec = true;
    //} else if (element.key() == CONFIG_KEY_NEUTRAL_KEY_GROUP_ID) {
    //  continue_v1_neutral_key_group_ids.push_back(loadConfigV1_ActionFlag(element.value()));
    //  is_exist_neutral_key_group_id = true;
    } else if (element.key() == CONFIG_KEY_ACTION_FLAG) {
      continue_v1_action_flag.push_back(loadConfigV1_ActionFlag(element.value()));
      is_exist_action_flag = true;
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_ContinueV1() skip config key: " << element.key();
    }
  }
  
  if (is_exist_action_key_codes == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_ContinueV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_CODES << ") is not exist, stop program.";
    exit(-1);
  }
  //if (is_exist_not_action_key_codes == false) {
  //  LOG(ERROR) << "ERROR: loadConfigV1_ContinueV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_NOT_ACTION_CODES << ") is not exist, stop program.";
  //  exit(-1);
  //}
  if (is_exist_action_condition == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_ContinueV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_CONDITIONS << ") is not exist, stop program.";
    exit(-1);
  }
  if (is_exist_condition_timeout_msec == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_ContinueV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_CONDITIONS_TIMEOUT_MSEC << ") is not exist, stop program.";
    exit(-1);
  }
  //if (is_exist_neutral_key_group_id == false) {
  //  LOG(ERROR) << "ERROR: loadConfigV1_ContinueV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_NEUTRAL_KEY_GROUP_ID << ") is not exist, stop program.";
  //  exit(-1);
  //}
  if (is_exist_action_flag == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_ContinueV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_FLAG << ") is not exist, stop program.";
    exit(-1);
  }
  
  LOG(INFO) << "loadConfigV1_ContinueV1() end";
}

std::vector<int> loadConfigV1_KeyPointsV5(nlohmann::json& json_data) {
  LOG(INFO) << "loadConfigV1_KeyPointsV5() start";
  
  std::vector<int> key_points;
  
  int count = 1;
  for (auto& element : json_data) {
    if (element.is_number() == false) {
      // ignore
      LOG(ERROR) << "WARN: loadConfigV1_KeyPointsV5() element is not number, skip.";
      continue;
    }
    
    key_points.push_back(element);
    
    count++;
  }
  
  if (key_points.size() % CONFIG_KEY_VECTOR_DIST_COMPARE_V1_KEY_POINTS_MULTIPLE_SIZE != 0) {
    LOG(ERROR) << "WARN: loadConfigV1_KeyPointsV5() element size(" << key_points.size() << ") is not multiple size(" << CONFIG_KEY_VECTOR_DIST_COMPARE_V1_KEY_POINTS_MULTIPLE_SIZE << "), erase over data.";
    for (int i = 0; i < key_points.size() % CONFIG_KEY_VECTOR_DIST_COMPARE_V1_KEY_POINTS_MULTIPLE_SIZE; i++) {
      key_points.pop_back();
    }
  }
  
  LOG(INFO) << "loadConfigV1_KeyPointsV5() end";
  
  return key_points;
}

void loadConfigV1_VectorDistCompareV1(nlohmann::json& json_data, int index, int child_index) {
  LOG(INFO) << "loadConfigV1_VectorDistCompareV1() start";
  
  bool is_exist_action_key_codes = false;
  bool is_exist_not_action_key_codes = false;
  bool is_exist_key_points = false;
  bool is_exist_neutral_key_group_id = false;
  bool is_exist_action_flag = false;
  
  for (auto& element : json_data.items()) {
    if (element.key() == CONFIG_KEY_ACTION_CODES) {
      vector_dists_compare_v1_action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
      is_exist_action_key_codes = true;
    //} else if (element.key() == CONFIG_KEY_NOT_ACTION_CODES) {
    //  vector_dists_compare_v1_not_action_key_code_list.push_back(loadConfigV1_ActionCodes(element.value()));
    //  is_exist_not_action_key_codes = true;
    } else if (element.key() == CONFIG_KEY_KEY_POINTS) {
      vector_dists_compare_v1_key_points.push_back(loadConfigV1_KeyPointsV5(element.value()));
      is_exist_key_points = true;
    //} else if (element.key() == CONFIG_KEY_NEUTRAL_KEY_GROUP_ID) {
    //  vector_dists_compare_v1_neutral_key_group_ids.push_back(loadConfigV1_ActionFlag(element.value()));
    //  is_exist_neutral_key_group_id = true;
    } else if (element.key() == CONFIG_KEY_ACTION_FLAG) {
      vector_dists_compare_v1_flag.push_back(loadConfigV1_ActionFlag(element.value()));
      is_exist_action_flag = true;
    } else {
      LOG(ERROR) << "WARN: loadConfigV1_VectorDistCompareV1() skip config key: " << element.key();
    }
  }
  
  if (is_exist_action_key_codes == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_VectorDistCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_CODES << ") is not exist, stop program.";
    exit(-1);
  }
  //if (is_exist_not_action_key_codes == false) {
  //  LOG(ERROR) << "ERROR: loadConfigV1_VectorDistCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_NOT_ACTION_CODES << ") is not exist, stop program.";
  //  exit(-1);
  //}
  if (is_exist_key_points == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_VectorDistCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_KEY_POINTS << ") is not exist, stop program.";
    exit(-1);
  }
  //if (is_exist_neutral_key_group_id == false) {
  //  LOG(ERROR) << "ERROR: loadConfigV1_VectorDistCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_NEUTRAL_KEY_GROUP_ID << ") is not exist, stop program.";
  //  exit(-1);
  //}
  if (is_exist_action_flag == false) {
    LOG(ERROR) << "ERROR: loadConfigV1_VectorDistCompareV1() index(" << index << ", " << child_index << ") key(" << CONFIG_KEY_ACTION_FLAG << ") is not exist, stop program.";
    exit(-1);
  }
  
  LOG(INFO) << "loadConfigV1_VectorDistCompareV1() end";
}


//////////
// dump //
//////////
void dumpConfigV1() {
  std::string indent_str = "";
  
  int count = 1;
  int index = count - 1;
  
  LOG(INFO) << indent_str << CONFIG_KEY_PC_INFO << ": ";
  dumpConfigV1_PCInfo(indent_str + "  ");
  dumpConfigV1_ActionInfo(indent_str + "  ");
  
  LOG(INFO) << indent_str << CONFIG_KEY_TEMPLATE_CONFIG << ": ";
  dumpConfigV1_CoordinateV2(index, indent_str + "  ");
  dumpConfigV1_CoordinateV3(index, indent_str + "  ");
  dumpConfigV1_SumV1(index, indent_str + "  ");
  dumpConfigV1_FrameDiffV1(index, indent_str + "  ");
  dumpConfigV1_DistCompareV1(index, indent_str + "  ");
  dumpConfigV1_MixV1(index, indent_str + "  ");
  dumpConfigV1_MarkCompareV1(index, indent_str + "  ");
  dumpConfigV1_ContinueV1(index, indent_str + "  ");
  dumpConfigV1_VectorDistCompareV1(index, indent_str + "  ");
}

void dumpConfigV1_PCInfo(std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_MAIN_DISPLAY_WIDTH << ": " << main_display_width;
  LOG(INFO) << indent_str << CONFIG_KEY_MAIN_DISPLAY_HEIGHT << ": " << main_display_height;
  LOG(INFO) << indent_str << CONFIG_KEY_NO_MOUSE_ABSOLUTE_RATE << ": " << no_mouse_absolute_rate;
  LOG(INFO) << indent_str << CONFIG_KEY_CAMERA_ID << ": " << camera_id;
  LOG(INFO) << indent_str << CONFIG_KEY_CAMERA_DISPLAY_WIDTH << ": " << camera_width;
  LOG(INFO) << indent_str << CONFIG_KEY_CAMERA_DISPLAY_HEIGHT << ": " << camera_height;
  LOG(INFO) << indent_str << CONFIG_KEY_CAMERA_FPS << ": " << camera_fps;
  LOG(INFO) << indent_str << CONFIG_KEY_CAMERA_PROC_FPS << ": " << camera_proc_fps;
  LOG(INFO) << indent_str << CONFIG_KEY_CAMERA_LOW_PROC_FPS << ": " << camera_low_proc_fps;
}

void dumpConfigV1_ActionInfo(std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_IS_AUTO_START << ": " << is_auto_start;
  LOG(INFO) << indent_str << CONFIG_KEY_FOCUS_DISPLAY_LEFT << ": " << focus_display_left;
  LOG(INFO) << indent_str << CONFIG_KEY_FOCUS_DISPLAY_TOP << ": " << focus_display_top;
  LOG(INFO) << indent_str << CONFIG_KEY_FOCUS_DISPLAY_RIGHT << ": " << focus_display_right;
  LOG(INFO) << indent_str << CONFIG_KEY_FOCUS_DISPLAY_BOTTOM << ": " << focus_display_bottom;
  LOG(INFO) << indent_str << CONFIG_KEY_IS_DISPLAY_MIRROR << ": " << is_display_mirror;
  LOG(INFO) << indent_str << CONFIG_KEY_DISPLAY_MOSAIC_SCALE << ": " << display_mosaic_scale;
  LOG(INFO) << indent_str << CONFIG_KEY_DISPLAY_FACE_MOSAIC_SCALE << ": " << display_face_mosaic_scale;
  LOG(INFO) << indent_str << CONFIG_KEY_DISPLAY_FACE_MOSAIC_WIDTH << ": " << display_face_mosaic_width;
  LOG(INFO) << indent_str << CONFIG_KEY_DISPLAY_FACE_MOSAIC_HEIGHT << ": " << display_face_mosaic_height;
  LOG(INFO) << indent_str << CONFIG_KEY_FRAME_DIFF_FRAME_MEMORY_MSEC << ": " << frame_diff_landmarks_max_buffer_msec;
  LOG(INFO) << indent_str << CONFIG_KEY_GAME_START_WAIT_MSEC << ": " << game_start_wait_msec;
  LOG(INFO) << indent_str << CONFIG_KEY_PERSON_SIZE_SCALE_COORDINATE << ": " << person_size_scale_coordinate;
  LOG(INFO) << indent_str << CONFIG_KEY_PERSON_SIZE_SCALE_SUM << ": " << person_size_scale_sum;
  LOG(INFO) << indent_str << CONFIG_KEY_PERSON_SIZE_SCALE_FRAME_DIFF << ": " << person_size_scale_frame_diff;
  LOG(INFO) << indent_str << CONFIG_KEY_PERSON_SIZE_SCALE_DIST_COMPARE << ": " << person_size_scale_dist_compare;
  LOG(INFO) << indent_str << CONFIG_KEY_PERSON_SIZE_SCALE_MARK_COMPARE << ": " << person_size_scale_mark_compare;
  LOG(INFO) << indent_str << CONFIG_KEY_PERSON_SIZE_SCALE_VECTOR_DIST_COMPARE << ": " << person_size_scale_vector_dist_compare;
  LOG(INFO) << indent_str << CONFIG_KEY_KEY_GROUP << ": transform data: key_code_group_id_map";
  LOG(INFO) << indent_str << "  key_code  : group_id";
  for (std::map<int, int>::iterator itr = key_code_group_id_map.begin(); itr != key_code_group_id_map.end(); itr++) {
    std::string key_code = getDumpHexFromInt(itr->first);
    //std::string group_id = getDumpHexFromInt(itr->second);
    int group_id = itr->second;
    LOG(INFO) << indent_str << "  " << key_code << ": " << group_id;
  }
  //LOG(INFO) << indent_str << CONFIG_KEY_KEY_GROUP << ": " << latest_running_key_code_group_map;
  LOG(INFO) << indent_str << CONFIG_KEY_IS_ENABLE_CHANGE_X_FRONT_REVERSE << ": " << is_enable_change_x_front_reverse;
  LOG(INFO) << indent_str << CONFIG_KEY_X_FRONT_REVERSE_CHECK_MSEC << ": " << x_front_reverse_check_msec;
  LOG(INFO) << indent_str << CONFIG_KEY_EYES_DIST << ": " << eyes_dist;
  LOG(INFO) << indent_str << CONFIG_KEY_LOGOUT_COORDINATE_TYPES << ": ";
  for (int i = 0; i < logout_type_specify.size(); i++) {
    LOG(INFO) << indent_str << "  " << logout_type_specify[i];
  }
  LOG(INFO) << indent_str << CONFIG_KEY_LOGOUT_COORDINATE_NUMBERS << ": ";
  for (int i = 0; i < logout_number_specify.size(); i++) {
    LOG(INFO) << indent_str << "  " << logout_number_specify[i];
  }
}

void dumpConfigV1_CoordinateV2(int index, std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_COORDINATE_V2 << "[" << index << "]: ";
  dumpConfigV1_ActionCodesList(action_key_code_list, indent_str + "  ");
  dumpConfigV1_KeyPointsList(target_key_points, indent_str + "  ");
  dumpConfigV1_ThresholdConditions(threshold_condition_list, indent_str + "  ");
  dumpConfigV1_DisplayColors(display_color_list, indent_str + "  ");
  dumpConfigV1_ActionFlags(action_flag_list, indent_str + "  ");
}

void dumpConfigV1_ActionCodesList(std::vector<std::vector<int> > action_codes_list, std::string indent_str) {
  for (int i = 0; i < action_codes_list.size(); i++) {
    LOG(INFO) << indent_str << "action_codes_list[" << i << "]: ";
    dumpConfigV1_ActionCodes(action_codes_list[i], indent_str + "  ");
  }
}

void dumpConfigV1_NotActionCodesList(std::vector<std::vector<int> > not_action_codes_list, std::string indent_str) {
  for (int i = 0; i < not_action_codes_list.size(); i++) {
    LOG(INFO) << indent_str << "not_action_codes_list[" << i << "]: ";
    dumpConfigV1_NotActionCodes(not_action_codes_list[i], indent_str + "  ");
  }
}

std::string getDumpHexFromUInt64(uint64_t num) {
  std::stringstream str_stream;
  str_stream << std::hex << num;
  std::string str_hex = str_stream.str();
  while (str_hex.size() < 8) {
    str_hex = "0" + str_hex;
  }
  return str_hex;
}

std::string getDumpHexFromInt(int num) {
  std::stringstream str_stream;
  str_stream << std::hex << num;
  std::string str_hex = str_stream.str();
  while (str_hex.size() < 8) {
    str_hex = "0" + str_hex;
  }
  return str_hex;
}

void dumpConfigV1_ActionCodes(std::vector<int> action_codes, std::string indent_str) {
  for (int i = 0; i < action_codes.size(); i++) {
    //LOG(INFO) << indent_str << "action_codes[" << i << "]: " << action_codes[i];
    std::string str_hex = getDumpHexFromInt(action_codes[i]);
    LOG(INFO) << indent_str << CONFIG_KEY_ACTION_CODES << "[" << i << "]: 0x" << str_hex;
  }
}

void dumpConfigV1_NotActionCodes(std::vector<int> not_action_codes, std::string indent_str) {
  for (int i = 0; i < not_action_codes.size(); i++) {
    //LOG(INFO) << indent_str << "action_codes[" << i << "]: " << action_codes[i];
    std::string str_hex = getDumpHexFromInt(not_action_codes[i]);
    LOG(INFO) << indent_str << CONFIG_KEY_NOT_ACTION_CODES << "[" << i << "]: 0x" << str_hex;
  }
}

void dumpConfigV1_KeyPointsList(std::vector<std::vector<int> > key_points_list, std::string indent_str) {
  for (int i = 0; i < key_points_list.size(); i++) {
    LOG(INFO) << indent_str << CONFIG_KEY_KEY_POINTS << "_list[" << i << "]: ";
    dumpConfigV1_KeyPoints(key_points_list[i], indent_str + "  ");
  }
}

void dumpConfigV1_KeyPoints(std::vector<int> key_points, std::string indent_str) {
  for (int i = 0; i < key_points.size(); i++) {
    LOG(INFO) << indent_str << CONFIG_KEY_KEY_POINTS << "[" << i << "]: " << key_points[i];
  }
}

void dumpConfigV1_ThresholdConditions(std::vector<std::pair<float, int> > threshold_conditions, std::string indent_str) {
  for (int i = 0; i < threshold_conditions.size(); i++) {
    LOG(INFO) << indent_str << CONFIG_KEY_THRESHOLD_CONDITIONS << "[" << i << "]: ";
    dumpConfigV1_ThresholdCondition(threshold_conditions[i], indent_str + "  ");
  }
}

void dumpConfigV1_ThresholdCondition(std::pair<float, int> threshold_condition, std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_THRESHOLD << ":  " << threshold_condition.first;
  LOG(INFO) << indent_str << CONFIG_KEY_CONDITION << ": " << threshold_condition.second;
}

std::string getDumpHexFromColorScalar(cv::Scalar scalar) {
  std::string str_scalar = "";
  //int scalar_size = scalar.length(); // no method, size, length, ???
  int scalar_size = 3;
  for (int i = 0; i < scalar_size; i++) {
    std::stringstream str_stream;
    str_stream << std::hex << (int)(scalar[i]);
    std::string str_hex = str_stream.str();
    while (str_hex.size() < 2) {
      str_hex = "0" + str_hex;
    }
    str_scalar += str_hex;
  }
  return str_scalar;
}

void dumpConfigV1_DisplayColors(std::vector<cv::Scalar> display_colors, std::string indent_str) {
  for (int i = 0; i < display_colors.size(); i++) {
    std::string str_hex = getDumpHexFromColorScalar(display_colors[i]);
    LOG(INFO) << indent_str << CONFIG_KEY_DISPLAY_COLOR << "[" << i << "]: 0x" << str_hex;
  }
}

void dumpConfigV1_ActionFlags(std::vector<int> action_flags, std::string indent_str) {
  for (int i = 0; i < action_flags.size(); i++) {
    LOG(INFO) << indent_str << CONFIG_KEY_ACTION_FLAG << "[" << i << "]: " << action_flags[i];
  }
}

void dumpConfigV1_CoordinateV3(int index, std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_COORDINATE_V3 << "[" << index << "]: ";
  dumpConfigV1_ActionCodesList(coord_v3_action_key_code_list, indent_str + "  ");
  dumpConfigV1_KeyPointsList(coord_v3_target_key_points, indent_str + "  ");
  dumpConfigV1_DisplayColors(coord_v3_display_color_list, indent_str + "  ");
  dumpConfigV1_ActionFlags(coord_v3_action_flag_list, indent_str + "  ");
}

void dumpConfigV1_VisibilityThresholds(std::vector<float> visibility_thresholds, std::string indent_str) {
  for (int i = 0; i < visibility_thresholds.size(); i++) {
    LOG(INFO) << indent_str << CONFIG_KEY_VISIBILITY_THRESHOLD << "[" << i << "]: " << visibility_thresholds[i];
  }
}

void dumpConfigV1_SumV1(int index, std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_SUM_V1 << "[" << index << "]: ";
  dumpConfigV1_ActionCodesList(sum_action_key_code_list, indent_str + "  ");
  dumpConfigV1_KeyPointsList(sum_target_key_points, indent_str + "  ");
  dumpConfigV1_ThresholdConditions(sum_threshold_condition_list, indent_str + "  ");
  dumpConfigV1_VisibilityThresholds(sum_visibility_threshold_list, indent_str + "  ");
  dumpConfigV1_ActionFlags(sum_action_flag_list, indent_str + "  ");
}

void dumpConfigV1_FrameDiffV1(int index, std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_FRAME_DIFF_V1 << "[" << index << "]: ";
  dumpConfigV1_ActionCodesList(frame_diff_v1_action_key_code_list, indent_str + "  ");
  dumpConfigV1_KeyPointsList(frame_diff_v1_key_points, indent_str + "  ");
  dumpConfigV1_ThresholdConditions(frame_diff_v1_threshold_condition, indent_str + "  ");
  LOG(INFO) << indent_str + "  " << CONFIG_KEY_CHECK_VECTOR << ": ";
  dumpConfigV1_ThresholdConditionsList(frame_diff_v1_check_vector, indent_str + "  " + "  ");
  LOG(INFO) << indent_str + "  " << CONFIG_KEY_CHECK_VECTOR_RATE << ": ";
  dumpConfigV1_ThresholdConditionsList(frame_diff_v1_check_vector_rate, indent_str + "  " + "  ");
  dumpConfigV1_ActionFlags(frame_diff_v1_flags, indent_str + "  ");
}

void dumpConfigV1_ThresholdConditionsList(std::vector<std::vector<std::pair<float, int> > > threshold_conditions_list, std::string indent_str) {
  for (int i = 0; i < threshold_conditions_list.size(); i++) {
    LOG(INFO) << indent_str << CONFIG_KEY_THRESHOLD_CONDITIONS << "_list[" << i << "]: ";
    dumpConfigV1_ThresholdConditions(threshold_conditions_list[i], indent_str + "  ");
  }
}

void dumpConfigV1_DistCompareV1(int index, std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_DIST_COMPARE_V1 << "[" << index << "]: ";
  dumpConfigV1_ActionCodesList(landmark_2point_dist_compare_v1_action_key_code_list, indent_str + "  ");
  dumpConfigV1_KeyPointsList(landmark_2point_dist_compare_v1_key_points, indent_str + "  ");
  dumpConfigV1_ActionFlags(landmark_2point_dist_compare_v1_flag, indent_str + "  ");
}

void dumpConfigV1_MixV1(int index, std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_MIX_V1 << "[" << index << "]: ";
  dumpConfigV1_ActionCodesList(mix_v1_action_key_code_list, indent_str + "  ");
  dumpConfigV1_ActionConditionsList(mix_v1_action_condition_list, indent_str + "  ");
  dumpConfigV1_ActionFlags(mix_v1_action_flag, indent_str + "  ");
}

void dumpConfigV1_MarkCompareV1(int index, std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_MARK_COMPARE_V1 << "[" << index << "]: ";
  dumpConfigV1_ActionCodesList(mark_compare_v1_action_key_code_list, indent_str + "  ");
  dumpConfigV1_NotActionCodesList(mark_compare_v1_not_action_key_code_list, indent_str + "  ");
  dumpConfigV1_KeyPointsList(mark_compare_v1_key_points, indent_str + "  ");
  dumpConfigV1_ActionFlags(mark_compare_v1_action_flag, indent_str + "  ");
}

void dumpConfigV1_ContinueV1(int index, std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_CONTINUE_V1 << "[" << index << "]: ";
  dumpConfigV1_ActionCodesList(continue_v1_action_key_code_list, indent_str + "  ");
  dumpConfigV1_NotActionCodesList(continue_v1_not_action_key_code_list, indent_str + "  ");
  dumpConfigV1_ActionConditionsList(continue_v1_action_condition_list, indent_str + "  ");
  LOG(INFO) << indent_str + "  " << CONFIG_KEY_CONDITIONS_TIMEOUT_MSEC << ": ";
  dumpConfigV1_VecVecInt(continue_v1_condition_timeout_msec_list, indent_str + "  " + "  ");
  dumpConfigV1_ActionFlags(continue_v1_action_flag, indent_str + "  ");
}

void dumpConfigV1_VectorDistCompareV1(int index, std::string indent_str) {
  LOG(INFO) << indent_str << CONFIG_KEY_VECTOR_DIST_COMPARE_V1 << "[" << index << "]: ";
  dumpConfigV1_ActionCodesList(vector_dists_compare_v1_action_key_code_list, indent_str + "  ");
  dumpConfigV1_NotActionCodesList(vector_dists_compare_v1_not_action_key_code_list, indent_str + "  ");
  dumpConfigV1_KeyPointsList(vector_dists_compare_v1_key_points, indent_str + "  ");
  dumpConfigV1_ActionFlags(vector_dists_compare_v1_flag, indent_str + "  ");
}

void dumpConfigV1_ActionConditionsList(std::vector<std::vector<int> > action_conditions_list, std::string indent_str) {
  for (int i = 0; i < action_conditions_list.size(); i++) {
    LOG(INFO) << indent_str << CONFIG_KEY_ACTION_CONDITIONS << "_list[" << i << "]: ";
    dumpConfigV1_ActionConditions(action_conditions_list[i], indent_str + "  ");
  }
}

void dumpConfigV1_ActionConditions(std::vector<int> action_conditions, std::string indent_str) {
  for (int i = 0; i < action_conditions.size(); i++) {
    LOG(INFO) << indent_str << CONFIG_KEY_ACTION_CONDITIONS << "[" << i << "]: " << action_conditions[i];
  }
}

void dumpConfigV1_VecVecInt(std::vector<std::vector<int> > vec_vec_int, std::string indent_str) {
  for (int i = 0; i < vec_vec_int.size(); i++) {
    LOG(INFO) << indent_str << "list[" << i << "]: ";
    dumpConfigV1_VecInt(vec_vec_int[i], indent_str + "  ");
  }
}

void dumpConfigV1_VecInt(std::vector<int> vec_int, std::string indent_str) {
  for (int i = 0; i < vec_int.size(); i++) {
    LOG(INFO) << indent_str << "[" << i << "]: " << vec_int[i];
  }
}


//------------------------------------------------------------------------------
// common end
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// personalise start
//------------------------------------------------------------------------------

void fix_person_size_scale() {
  fix_person_size_scale_coordinate();
  fix_person_size_scale_sum();
  fix_person_size_scale_frame_diff();
  fix_person_size_scale_dist_compare();
  fix_person_size_scale_mark_compare();
  fix_person_size_scale_vector_dist_compare();
}

void fix_person_size_scale_coordinate() {
  if (person_size_scale_coordinate == 1.0) {
    return;
  }
  
  LOG(INFO) << "fix person_size_scale_coordinate: " << person_size_scale_coordinate;
  
  std::string indent_str = "";
  
  dumpConfigV1_ThresholdConditions(threshold_condition_list, indent_str + "  ");
  dumpConfigV1_KeyPointsList(coord_v3_target_key_points, indent_str + "  ");
  
  LOG(INFO) << "fix person_size_scale_coordinate start";
  
  for (int i = 0; i < threshold_condition_list.size(); i++) {
    threshold_condition_list[i].first *= person_size_scale_coordinate;
  }
  for (int i = 0; i < coord_v3_target_key_points.size(); i++) {
    coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO] = (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO] * person_size_scale_coordinate);
  }
  
  LOG(INFO) << "fix person_size_scale_coordinate end";
  
  dumpConfigV1_ThresholdConditions(threshold_condition_list, indent_str + "  ");
  dumpConfigV1_KeyPointsList(coord_v3_target_key_points, indent_str + "  ");
}

void fix_person_size_scale_sum() {
  if (person_size_scale_sum == 1.0) {
    return;
  }
  
  LOG(INFO) << "fix person_size_scale_sum: " << person_size_scale_sum;
  
  std::string indent_str = "";
  
  dumpConfigV1_ThresholdConditions(sum_threshold_condition_list, indent_str + "  ");
  
  LOG(INFO) << "fix person_size_scale_sum start";
  
  for (int i = 0; i < landmark_2point_dist_compare_v1_key_points.size(); i++) {
    sum_threshold_condition_list[i].first *= person_size_scale_sum;
  }
  
  LOG(INFO) << "fix person_size_scale_sum end";
  
  dumpConfigV1_ThresholdConditions(sum_threshold_condition_list, indent_str + "  ");
}

void fix_person_size_scale_frame_diff() {
  if (person_size_scale_frame_diff == 1.0) {
    return;
  }
  
  LOG(INFO) << "fix person_size_scale_frame_diff: " << person_size_scale_frame_diff;
  
  std::string indent_str = "";
  
  dumpConfigV1_ThresholdConditions(frame_diff_v1_threshold_condition, indent_str + "  ");
  LOG(INFO) << indent_str + "  " << CONFIG_KEY_CHECK_VECTOR << ": ";
  dumpConfigV1_ThresholdConditionsList(frame_diff_v1_check_vector, indent_str + "  " + "  ");
  
  LOG(INFO) << "fix person_size_scale_frame_diff start";
  
  for (int i = 0; i < frame_diff_v1_threshold_condition.size(); i++) {
    frame_diff_v1_threshold_condition[i].first *= person_size_scale_frame_diff;
  }
  
  for (int i = 0; i < frame_diff_v1_check_vector.size(); i++) {
    for (int j = 0; j < frame_diff_v1_check_vector[i].size(); j++) {
      frame_diff_v1_check_vector[i][j].first *= person_size_scale_frame_diff;
    }
  }
  
  LOG(INFO) << "fix person_size_scale_frame_diff end";
  
  dumpConfigV1_ThresholdConditions(frame_diff_v1_threshold_condition, indent_str + "  ");
  LOG(INFO) << indent_str + "  " << CONFIG_KEY_CHECK_VECTOR << ": ";
  dumpConfigV1_ThresholdConditionsList(frame_diff_v1_check_vector, indent_str + "  " + "  ");
}

void fix_person_size_scale_dist_compare() {
  if (person_size_scale_dist_compare == 1.0) {
    return;
  }
  
  LOG(INFO) << "fix person_size_scale_dist_compare: " << person_size_scale_dist_compare;
  
  std::string indent_str = "";
  
  dumpConfigV1_KeyPointsList(landmark_2point_dist_compare_v1_key_points, indent_str + "  ");
  
  LOG(INFO) << "fix person_size_scale_dist_compare start";
  
  for (int i = 0; i < landmark_2point_dist_compare_v1_key_points.size(); i++) {
    landmark_2point_dist_compare_v1_key_points[i][DIST_COMPARE_THRESHOLD_VALUE_NO] = (int)(landmark_2point_dist_compare_v1_key_points[i][DIST_COMPARE_THRESHOLD_VALUE_NO] * person_size_scale_dist_compare);
  }
  
  LOG(INFO) << "fix person_size_scale_dist_compare end";
  
  dumpConfigV1_KeyPointsList(landmark_2point_dist_compare_v1_key_points, indent_str + "  ");
}

void fix_person_size_scale_mark_compare() {
  if (person_size_scale_mark_compare == 1.0) {
    return;
  }
  
  LOG(INFO) << "fix person_size_scale_mark_compare: " << person_size_scale_mark_compare;
  
  std::string indent_str = "";
  
  dumpConfigV1_KeyPointsList(mark_compare_v1_key_points, indent_str + "  ");
  
  LOG(INFO) << "fix person_size_scale_mark_compare start";
  
  for (int i = 0; i < mark_compare_v1_key_points.size(); i++) {
    mark_compare_v1_key_points[i][MARK_COMPARE_OFFSET_A] = (int)(mark_compare_v1_key_points[i][MARK_COMPARE_OFFSET_A] * person_size_scale_mark_compare);
    mark_compare_v1_key_points[i][MARK_COMPARE_OFFSET_B] = (int)(mark_compare_v1_key_points[i][MARK_COMPARE_OFFSET_B] * person_size_scale_mark_compare);
  }
  
  LOG(INFO) << "fix person_size_scale_mark_compare end";
  
  dumpConfigV1_KeyPointsList(landmark_2point_dist_compare_v1_key_points, indent_str + "  ");
}

void fix_person_size_scale_vector_dist_compare() {
  if (person_size_scale_vector_dist_compare == 1.0) {
    return;
  }
  
  LOG(INFO) << "fix person_size_scale_vector_dist_compare: " << person_size_scale_vector_dist_compare;
  
  std::string indent_str = "";
  
  dumpConfigV1_KeyPointsList(vector_dists_compare_v1_key_points, indent_str + "  ");
  
  LOG(INFO) << "fix person_size_scale_vector_dist_compare start";
  
  for (int i = 0; i < vector_dists_compare_v1_key_points.size(); i++) {
    vector_dists_compare_v1_key_points[i][VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST_OFFSET] = (int)(vector_dists_compare_v1_key_points[i][VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST_OFFSET] * person_size_scale_vector_dist_compare);
    vector_dists_compare_v1_key_points[i][VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND_OFFSET] = (int)(vector_dists_compare_v1_key_points[i][VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND_OFFSET] * person_size_scale_vector_dist_compare);
  }
  
  LOG(INFO) << "fix person_size_scale_vector_dist_compare end";
  
  dumpConfigV1_KeyPointsList(landmark_2point_dist_compare_v1_key_points, indent_str + "  ");
}


//------------------------------------------------------------------------------
// personalise end
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// frame diff start
//------------------------------------------------------------------------------
template<class T> void add_vector_specify_size(std::vector<T>& vec, T& context, int& max_msec) {
  vec.push_back(context);
  while (vec.size() > max_msec) {
    vec.erase(vec.begin());
  }
}

void add_frame_diff_landmarks(uint64_t& timestamp, mediapipe::LandmarkList& landmarks, mediapipe::NormalizedLandmarkList& landmarks_2d, mediapipe::LandmarkList& vec_landmarks, std::vector<mediapipe::LandmarkList>& landmarks_vec, std::vector<mediapipe::NormalizedLandmarkList>& landmarks_2d_vec, std::vector<mediapipe::LandmarkList>& landmarks_vec_vec, std::vector<uint64_t>& timestamps_vec) {
  if (timestamps_vec.empty() == false) {
    uint64_t rest_timestamp = timestamp - frame_diff_landmarks_max_buffer_msec;
    int erase_size = 0;
    for (int i = 0; i < timestamps_vec.size(); i++) {
      if (timestamps_vec[i] > rest_timestamp) {
        break;
      }
      erase_size++;
    }
    for (int i = 0; i < erase_size; i++) {
      timestamps_vec.erase(timestamps_vec.begin());
      landmarks_vec.erase(landmarks_vec.begin());
      landmarks_2d_vec.erase(landmarks_2d_vec.begin());
      landmarks_vec_vec.erase(landmarks_vec_vec.begin());
    }
  }
  //add_vector_specify_size(timestamps_vec, timestamp, frame_diff_landmarks_max_buffer_msec);
  //add_vector_specify_size(landmarks_vec, landmarks, frame_diff_landmarks_max_buffer_msec);
  //add_vector_specify_size(landmarks_2d_vec, landmarks_2d, frame_diff_landmarks_max_buffer_msec);
  //add_vector_specify_size(landmarks_vec_vec, vec_landmarks, frame_diff_landmarks_max_buffer_msec);
  timestamps_vec.push_back(timestamp);
  landmarks_vec.push_back(landmarks);
  landmarks_2d_vec.push_back(landmarks_2d);
  landmarks_vec_vec.push_back(vec_landmarks);
}

//------------------------------------------------------------------------------
// frame diff end
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// image start
//------------------------------------------------------------------------------
auto loadImageForMediapipe(std::string& file_path) {
  cv::Mat	image = cv::imread(file_path);
  
  auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
      mediapipe::ImageFormat::SRGB, image.cols, image.rows,
      mediapipe::ImageFrame::kDefaultAlignmentBoundary);
  cv::Mat image_mat = mediapipe::formats::MatView(input_frame.get());
  
  image.copyTo(image_mat);
  return input_frame;
}

void drawLineHorizon(cv::Mat& image, int size, int height, cv::Scalar color) {
  //int left_max = camera_width;
  int left_max = focus_display_width;
  cv::line(image, cv::Point(0, height), cv::Point(left_max, height), color, size, 4);
}

void drawLineVertical(cv::Mat& image, int size, int width, cv::Scalar color) {
  //int down_max = camera_height;
  int down_max = focus_display_height;
  cv::line(image, cv::Point(width, 0), cv::Point(width, down_max), color, size, 4);
}

cv::Scalar getScalarColorFromInt(int color_int) {
  int color_r = (color_int & 0xff0000) >> 16;
  int color_g = (color_int & 0x00ff00) >> 8;
  int color_b = color_int & 0x0000ff;
  LOG(INFO) << "color(" << color_int << ") to rgb(" << color_r << ", " << color_g << ", " << color_b << ")";
  cv::Scalar color_scalar = cv::Scalar(color_r, color_g, color_b);
  return color_scalar;
}

int getColorFromPaletto() {
  // RAND_MAX: 32767
  // 16777215 = 0xffffff
  int color_r = rand() % 0xff;
  int color_g = rand() % 0xff;
  int color_b = rand() % 0xff;
  int color = (color_r << 16) + (color_g << 8) + color_b;
  LOG(INFO) << "RAND_MAX(" << RAND_MAX << "), rand() color: " << color;
  return color;
}

void drawLineUpDownAndLeftRightThreshold(cv::Mat& image, int size) {
  //int width = camera_width;
  //int height = camera_height;
  int width = focus_display_width;
  int height = focus_display_height;
  
  cv::Rect windows_frame(0, 0, width, camera_height);
  cv::rectangle(image, windows_frame, cv::Scalar(255, 255, 255), size, 4);
  
  // coord v1
  cv::Scalar color_horizon = cv::Scalar(255, 0, 0);
  cv::Scalar color_vertical = cv::Scalar(0, 255, 0);
  if (key_arrow_down_up_threshold.size() >= NO_KEY_MAX) {
    int up_line_heigt = (int)(key_arrow_down_up_threshold[NO_KEY_MIDDLE_OVER_THRESHOLD] * height);
    drawLineHorizon(image, size, up_line_heigt, color_horizon);
    int down_line_heigt = (int)(key_arrow_down_up_threshold[NO_KEY_MIN_OVER_THRESHOLD] * height);
    drawLineHorizon(image, size, down_line_heigt, color_horizon);
  }
  if (key_arrow_left_right_threshold.size() >= NO_KEY_MAX) {
    int right_line_width = (int)(key_arrow_left_right_threshold[NO_KEY_MIDDLE_OVER_THRESHOLD] * width);
    drawLineVertical(image, size, right_line_width, color_vertical);
    int left_line_width = (int)(key_arrow_left_right_threshold[NO_KEY_MAX_OVER_THRESHOLD] * width);
    drawLineVertical(image, size, left_line_width, color_vertical);
  }
  
  // coord v2
  if (target_key_points.empty() == false) {
    for (int i = 0; i < target_key_points.size(); i++) {
      if (target_key_points[i][KEY_POINT_COORDINATE_MODE_NO] == COORDINATE_MODE_LAST_2D) {
        if (target_key_points[i][KEY_POINT_LANDMARK_COORDINATE_NO] == MP_LANDMARK_VALUE_NO_X) {
          // x
          int line_width = (int)(threshold_condition_list[i].first * width);
          drawLineVertical(image, size, line_width, display_color_list[i]);
        } else if (target_key_points[i][KEY_POINT_LANDMARK_COORDINATE_NO] == MP_LANDMARK_VALUE_NO_Y) {
          // y
          int line_heigt = (int)(threshold_condition_list[i].first * height);
          drawLineHorizon(image, size, line_heigt, display_color_list[i]);
        }
      }
    }
  }
  
  // coord v3
  if (coord_v3_target_key_points.empty() == false) {
    for (int i = 0; i < coord_v3_target_key_points.size(); i++) {
      int points_count = coord_v3_target_key_points[i].size();
      // 矩形を描画する。縦と横があれば、両方を考慮する
      int line_left = 0;
      int line_right = width;
      int line_top = 0;
      int line_bottom = height;
      // TODO
      for (int j = 0; j < points_count; j += KEY_POINT_V3_MAX) {
        if (coord_v3_target_key_points[i][KEY_POINT_COORDINATE_MODE_NO + j] == COORDINATE_MODE_LAST_2D) {
          if (coord_v3_target_key_points[i][KEY_POINT_LANDMARK_COORDINATE_NO + j] == MP_LANDMARK_VALUE_NO_X) {
            // x
            int line_width = (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * width / KEY_POINT_THRESHOLD_VALUE_DIVID);
            //LOG(INFO) << "line_width: " << line_width;
            //drawLineVertical(image, size, line_width, coord_v3_display_color_list[i]);
            if (coord_v3_target_key_points[i][KEY_POINT_CONDITION_NO + j] > CONDITION_EQUAL) {
              if (line_left < (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * width / KEY_POINT_THRESHOLD_VALUE_DIVID)) {
                line_left = (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * width / KEY_POINT_THRESHOLD_VALUE_DIVID);
              } else {
                // skip data.
              }
            } else if (coord_v3_target_key_points[i][KEY_POINT_CONDITION_NO + j] < CONDITION_EQUAL) {
              if (line_right > (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * width / KEY_POINT_THRESHOLD_VALUE_DIVID)) {
                line_right = (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * width / KEY_POINT_THRESHOLD_VALUE_DIVID);
              } else {
                // skip data.
              }
            } else {
              // equal
              if (line_left < (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * width / KEY_POINT_THRESHOLD_VALUE_DIVID)) {
                line_left = (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * width / KEY_POINT_THRESHOLD_VALUE_DIVID);
              } else {
                // skip data.
              }
              if (line_right > (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * width / KEY_POINT_THRESHOLD_VALUE_DIVID)) {
                line_right = (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * width / KEY_POINT_THRESHOLD_VALUE_DIVID);
              } else {
                // skip data.
              }
            }
          } else if (coord_v3_target_key_points[i][KEY_POINT_LANDMARK_COORDINATE_NO + j] == MP_LANDMARK_VALUE_NO_Y) {
            // y
            int line_heigt = (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * height / KEY_POINT_THRESHOLD_VALUE_DIVID);
            //LOG(INFO) << "line_heigt: " << line_heigt;
            //drawLineHorizon(image, size, line_heigt, coord_v3_display_color_list[i]);
            if (coord_v3_target_key_points[i][KEY_POINT_CONDITION_NO + j] > CONDITION_EQUAL) {
              if (line_top < (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * height / KEY_POINT_THRESHOLD_VALUE_DIVID)) {
                line_top = (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * height / KEY_POINT_THRESHOLD_VALUE_DIVID);
              } else {
                // skip data.
              }
            } else if (coord_v3_target_key_points[i][KEY_POINT_CONDITION_NO + j] < CONDITION_EQUAL) {
              if (line_bottom > (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * height / KEY_POINT_THRESHOLD_VALUE_DIVID)) {
                line_bottom = (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * height / KEY_POINT_THRESHOLD_VALUE_DIVID);
              } else {
                // skip data.
              }
            } else {
              // equal
              if (line_top < (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * height / KEY_POINT_THRESHOLD_VALUE_DIVID)) {
                line_top = (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * height / KEY_POINT_THRESHOLD_VALUE_DIVID);
              } else {
                // skip data.
              }
              if (line_bottom > (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * height / KEY_POINT_THRESHOLD_VALUE_DIVID)) {
                line_bottom = (int)(coord_v3_target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] * height / KEY_POINT_THRESHOLD_VALUE_DIVID);
              } else {
                // skip data.
              }
            }
          }
        }
      }
      
      if (line_left > line_right) {
        LOG(INFO) << "Condition v3(" << i << ") x is invalid, can not match condition setting.";
      }
      if (line_top > line_bottom) {
        LOG(INFO) << "Condition v3(" << i << ") y is invalid, can not match condition setting.";
      }
      
      //LOG(INFO) << "Condition v3(" << i << ") rect point(" << line_left << ", " << line_top << "), point(" << line_right << ", " << line_bottom << ").";
      //LOG(INFO) << "Condition v3(" << i << ") rect point(" << line_left << ", " << line_top << "), point(" << line_right << ", " << line_bottom << ").";
      int size = 4;
      cv::rectangle(image, cv::Point(line_left, line_top), cv::Point(line_right, line_bottom), coord_v3_display_color_list[i], size);
    }
  }
}

void drawProcInfo(cv::Mat& image) {
  char str_buffer[256];
  int line = 1;
  int pix_width_offset = 5;
  int pix_height = 20;
  float font_size = 0.5;
  int text_line_size = 1;
  cv::Scalar color(255, 255, 255);
  {
    // last_proc_fps
    sprintf(str_buffer, "mp fps: %.2f", last_proc_fps);
    cv::Point position(pix_width_offset, line++ * pix_height);
    cv::putText(image, str_buffer, position, cv::FONT_HERSHEY_SIMPLEX, font_size, color, text_line_size);
  }
  {
    // proc_msec_list[proc_msec_list.size() - 1];
    float proc_msec = -1.0;
    if (proc_msec_list.empty() == false) {
      proc_msec = proc_msec_list[proc_msec_list.size() - 1];
    }
    sprintf(str_buffer, "mp proc msec: %.2f", proc_msec);
    cv::Point position(pix_width_offset, line++ * pix_height);
    cv::putText(image, str_buffer, position, cv::FONT_HERSHEY_SIMPLEX, font_size, color, text_line_size);
  }
  {
    // last_done_fps
    sprintf(str_buffer, "all done fps: %.2f", last_done_fps);
    cv::Point position(pix_width_offset, line++ * pix_height);
    cv::putText(image, str_buffer, position, cv::FONT_HERSHEY_SIMPLEX, font_size, color, text_line_size);
  }
  {
    // done_msec_list[done_msec_list.size() - 1];
    float done_msec = -1.0;
    if (done_msec_list.empty() == false) {
      done_msec = done_msec_list[done_msec_list.size() - 1];
    }
    sprintf(str_buffer, "all done msec: %.2f", done_msec);
    cv::Point position(pix_width_offset, line++ * pix_height);
    cv::putText(image, str_buffer, position, cv::FONT_HERSHEY_SIMPLEX, font_size, color, text_line_size);
  }
}

void drawMakerInfo(cv::Mat& image) {
  char str_buffer[256];
  int line = 1;
  int pix_width_offset = 10;
  int pix_height = 20;
  float font_size = 0.5;
  int text_line_size = 1;
  cv::Scalar color(255, 255, 255);
  int image_width = image.cols;
  int image_height = image.rows;
  {
    // last_proc_fps
    sprintf(str_buffer, "made in O.O.");
    int char_size = strlen(str_buffer);
    cv::Point position(image_width - pix_width_offset * char_size, image_height - line++ * pix_height);
    cv::putText(image, str_buffer, position, cv::FONT_HERSHEY_SIMPLEX, font_size, color, text_line_size);
  }
}

void setMaskImage(cv::Mat& image) {
  cv::Scalar mask_color = cv::Scalar(255, 255, 255);
  int thickness = -1;
  
  if (is_display_mirror == true) {
    // TODO
  }
  
  // left
  cv::rectangle(image, cv::Point(0, 0), cv::Point(focus_display_left, camera_height), mask_color, thickness);
  // top
  cv::rectangle(image, cv::Point(0, 0), cv::Point(camera_width, focus_display_top), mask_color, thickness);
  // right
  cv::rectangle(image, cv::Point(focus_display_right, 0), cv::Point(camera_width, camera_height), mask_color, thickness);
  // bottom
  cv::rectangle(image, cv::Point(0, focus_display_bottom), cv::Point(camera_width, camera_height), mask_color, thickness);
}

bool checkIsOnMaskImage() {
  bool is_on_mack = false;
  
  if (focus_display_left <= 0 || camera_width <= focus_display_left) {
    
  } else {
    is_on_mack = true;
  }
  
  if (focus_display_top <= 0 || camera_height <= focus_display_top) {
    
  } else {
    is_on_mack = true;
  }
  
  if (focus_display_right < 0 || camera_width <= focus_display_right) {
    
  } else {
    is_on_mack = true;
  }
  
  if (focus_display_bottom < 0 || camera_height <= focus_display_bottom) {
    
  } else {
    is_on_mack = true;
  }
  
  LOG(INFO) << "is on mask(" << is_on_mack << ")";
  
  if (focus_display_left <= 0 || camera_width <= focus_display_left) {
    cut_left = 0;
  } else {
    cut_left = focus_display_left;
  }
  
  if (focus_display_top <= 0 || camera_height <= focus_display_top) {
    cut_top = 0;
  } else {
    cut_top = focus_display_top;
  }
  
  if (focus_display_right <= 0 || camera_width <= focus_display_right) {
    cut_right = camera_width;
  } else {
    cut_right = focus_display_right;
  }
  
  if (focus_display_bottom <= 0 || camera_height <= focus_display_bottom) {
    cut_bottom = camera_height;
  } else {
    cut_bottom = focus_display_bottom;
  }
  
  if (is_display_mirror == true) {
    int tmp_cut_left = cut_left;
    int tmp_cut_right = cut_right;
    cut_right = camera_width - tmp_cut_left;
    cut_left = camera_width - tmp_cut_right;
  }
  
  LOG(INFO) << "is display mirror(" << is_display_mirror << ")";
  
  if (is_on_mack == true) {
    //if (is_display_mirror == true) {
    //  focus_display_width = cut_left - cut_right;
    //  focus_display_height = cut_top - cut_bottom;
    //} else {
    //  focus_display_width = cut_right - cut_left;
    //  focus_display_height = cut_bottom - cut_top;
    //}
    focus_display_width = cut_right - cut_left;
    focus_display_height = cut_bottom - cut_top;
  } else {
    focus_display_width = camera_width;
    focus_display_height = camera_height;
  }
  
  LOG(INFO) << "ROI top left point(" << cut_left << ", " << cut_top << "), bottom right point(" << cut_right << ", " << cut_bottom << "), width(" << focus_display_width << "), height(" << focus_display_height << ")";
  
  return is_on_mack;
}

cv::Mat getROIImage(cv::Mat& image) {
  // TODO delete ここでは不要？　すでに実施済み？
  //if (is_display_mirror == true) {
  //  int tmp_cut_left = cut_left;
  //  int tmp_cut_right = cut_right;
  //  cut_right = camera_width - tmp_cut_left;
  //  cut_left = camera_width - tmp_cut_right;
  //}
  
  //cv::Mat roi_image(image, cv::Rect(cut_left, cut_top, cut_right, cut_bottom));
  cv::Mat roi_image(image, cv::Rect(cut_left, cut_top, focus_display_width, focus_display_height));
  return roi_image;
}

void convertMosicImage(cv::Mat& image, int multiple) {
  cv::resize(image, image, cv::Size(), 1.0 / multiple, 1.0 / multiple);
  cv::resize(image, image, cv::Size(), multiple, multiple, cv::INTER_NEAREST);
}

void drawFaceMosic(cv::Mat& image, int multiple, float multiple_width, float multiple_height, mediapipe::NormalizedLandmarkList& landmarks) {
  // TODO
  mediapipe::NormalizedLandmark center_face = landmarks.landmark(POSE_NO_CENTER_EAR);
  int center_x = center_face.x() * image.cols;
  int center_y = center_face.y() * image.rows;
  mediapipe::NormalizedLandmark left_ear = landmarks.landmark(POSE_NO_LEFT_EAR);
  mediapipe::NormalizedLandmark right_ear = landmarks.landmark(POSE_NO_RIGHT_EAR);
  mediapipe::NormalizedLandmark center_eye = landmarks.landmark(POSE_NO_CENTER_EYE);
  mediapipe::NormalizedLandmark center_mouth = landmarks.landmark(POSE_NO_CENTER_MOUTH);
  
  int width_ear = std::abs((right_ear.x() - left_ear.x()) * image.cols);
  int width_eye_mouth = std::abs((center_eye.x() - center_mouth.x()) * image.cols);
  int width = 0;
  if (width_ear > width_eye_mouth) {
    width = width_ear;
  } else {
    width = width_eye_mouth;
  }
  width *= multiple_width;
  
  int height_ear = std::abs((right_ear.y() - left_ear.y()) * image.rows);
  int height_eye_mouth = std::abs((center_eye.y() - center_mouth.y()) * image.rows);;
  int height = 0;
  if (height_ear > height_eye_mouth) {
    height = height_ear;
  } else {
    height = height_eye_mouth;
  }
  height *= multiple_height;
  
  int left = center_x - width / 2;
  int top = center_y - height / 2;
  if (left < 0) {
    left = 0;
  }
  if (top < 0) {
    top = 0;
  }
  LOG(INFO) << "drawFaceMosic() left: " << left << ", top: " << top << ", width: " << width << ", height: " << height;
  cv::Mat roi_image(image, cv::Rect(left, top, width, height));
  LOG(INFO) << "drawFaceMosic() roi_image cols: " << roi_image.cols << ", rows: " << roi_image.rows;
  //if (true) {
  if (false) {
    // error
    // OpenCV(3.4.10) Error: Assertion failed (!ssize.empty()) in cv::resize, file C:\build\3_4_winpack-build-win64-vc15\opencv\modules\imgproc\src\resize.cpp, line 3929
    cv::resize(roi_image, roi_image, cv::Size(), 1.0 / multiple, 1.0 / multiple);
    cv::resize(roi_image, roi_image, cv::Size(), multiple, multiple, cv::INTER_NEAREST);
  } else {
    cv::Mat roi_clone = roi_image.clone();
    cv::blur(roi_clone, roi_image, cv::Size(multiple, multiple));
  }
}

void drawLandmarksPose(cv::Mat& image, mediapipe::NormalizedLandmarkList& landmarks) {
  cv::Scalar line_color = cv::Scalar(255, 255, 255);
  cv::Scalar center_color = cv::Scalar(200, 255, 200);
  cv::Scalar left_color = cv::Scalar(255, 200, 200);
  cv::Scalar right_color = cv::Scalar(200, 200, 255);
  cv::Scalar color = center_color;
  int line_size = 1;
  for (int i = 0; i < landmarks.landmark_size(); i++) {
    switch (i) {
    case 38://POSE_NO_FRONT_BODY // not 2d
    case 39://POSE_NO_FRONT_SHOULDER // not 2d
    case 40://POSE_NO_FRONT_HIP // not 2d
    case 41://POSE_NO_WORLD_CENTER_BODY: // not 2d
    case 46://POSE_NO_FACE_UP // not 2d
    case 51://POSE_NO_LEFT_HAND_FRONT: // not imple
    case 52://POSE_NO_RIGHT_HAND_FRONT: // not imple
      // not draw
      continue;
    case 1://POSE_NO_LEFT_EYE_INNER
    case 2://POSE_NO_LEFT_EYE
    case 3://POSE_NO_LEFT_EYE_OUTER
    case 7://POSE_NO_LEFT_EAR
    case 9://POSE_NO_LEFT_MOUTH
    case 11://POSE_NO_LEFT_SHOULDER
    case 13://POSE_NO_LEFT_ELBOW
    case 15://POSE_NO_LEFT_WRIST
    case 17://POSE_NO_LEFT_PINKY
    case 19://POSE_NO_LEFT_INDEX
    case 21://POSE_NO_LEFT_THUMB
    case 23://POSE_NO_LEFT_HIP
    case 25://POSE_NO_LEFT_KNEE
    case 27://POSE_NO_LEFT_ANKLE
    case 29://POSE_NO_LEFT_HEEL
    case 31://POSE_NO_LEFT_FOOT_INDEX
    case 36://POSE_NO_LEFT_BODY
    case 47://POSE_NO_CENTER_LEFT_FINGER
    case 49://POSE_NO_LEFT_HAND_UP
      color = left_color;
      break;
    case 4://POSE_NO_RIGHT_EYE_INNER
    case 5://POSE_NO_RIGHT_EYE
    case 6://POSE_NO_RIGHT_EYE_OUTER
    case 8://POSE_NO_RIGHT_EAR
    case 10://POSE_NO_RIGHT_MOUTH
    case 12://POSE_NO_RIGHT_SHOULDER
    case 14://POSE_NO_RIGHT_ELBOW
    case 16://POSE_NO_RIGHT_WRIST
    case 18://POSE_NO_RIGHT_PINKY
    case 20://POSE_NO_RIGHT_INDEX
    case 22://POSE_NO_RIGHT_THUMB
    case 24://POSE_NO_RIGHT_HIP
    case 26://POSE_NO_RIGHT_KNEE
    case 28://POSE_NO_RIGHT_ANKLE
    case 30://POSE_NO_RIGHT_HEEL
    case 32://POSE_NO_RIGHT_FOOT_INDEX
    case 37://POSE_NO_RIGHT_BODY
    case 48://POSE_NO_CENTER_RIGHT_FINGER
    case 50://POSE_NO_RIGHT_HAND_UP
      color = right_color;
      break;
    default:
      color = center_color;
      break;
    }
    mediapipe::NormalizedLandmark landmark = landmarks.landmark(i);
    int parent_index = pose_vector_vec[i];
    mediapipe::NormalizedLandmark parent_landmark = landmarks.landmark(parent_index);
    int width = image.cols;
    int height = image.rows;
    
    if (i == parent_index) {
      // not draw line
    } else {
      cv::line(image, cv::Point(landmark.x() * width, landmark.y() * height), cv::Point(parent_landmark.x() * width, parent_landmark.y() * height), line_color, line_size, 4);
    }
    
    int radius = 4;
    int thickness = 2;
    int lineType = 8;
    int shift = 0;
    cv::circle(image, cv::Point(landmark.x() * width, landmark.y() * height), radius, color, thickness, lineType, shift);
    
    switch (i) {
    case 11://POSE_NO_LEFT_SHOULDER
    case 12://POSE_NO_RIGHT_SHOULDER
      {
        int peer_index = i + 12; // hip
        mediapipe::NormalizedLandmark peer_landmark = landmarks.landmark(peer_index);
        cv::line(image, cv::Point(landmark.x() * width, landmark.y() * height), cv::Point(peer_landmark.x() * width, peer_landmark.y() * height), line_color, line_size, 4);
      }
      break;
    default:
      break;
    }
  }
}

void drawLandmarksHand(cv::Mat& image, mediapipe::NormalizedLandmarkList& landmarks) {
  cv::Scalar line_color = cv::Scalar(255, 255, 255);
  cv::Scalar center_color = cv::Scalar(200, 255, 200);
  cv::Scalar thumb_color = cv::Scalar(255, 200, 200);
  cv::Scalar index_color = cv::Scalar(200, 200, 255);
  cv::Scalar middle_color = cv::Scalar(200, 255, 200);
  cv::Scalar ring_color = cv::Scalar(255, 200, 255);
  cv::Scalar pinky_color = cv::Scalar(200, 255, 255);
  cv::Scalar color = center_color;
  int line_size = 1;
  for (int i = 0; i < landmarks.landmark_size(); i++) {
    switch (i) {
    case 22://HAND_NO_SIDE // not 2d
    case 23://HAND_NO_UP // not 2d
    case 24://HAND_NO_FRONT // not 2d
    case 25://HAND_NO_WORLD_CENTER // not 2d
      // not draw
      continue;
    case 1://HAND_NO_THUMB_CMC
    case 2://HAND_NO_THUMB_MCP
    case 3://HAND_NO_THUMB_IP
    case 4://HAND_NO_THUMB_TIP
      color = thumb_color;
      break;
    case 5://HAND_NO_INDEX_FINGER_MCP
    case 6://HAND_NO_INDEX_FINGER_PIP
    case 7://HAND_NO_INDEX_FINGER_DIP
    case 8://HAND_NO_INDEX_FINGER_TIP
      color = index_color;
      break;
    case 9://HAND_NO_MIDDLE_FINGER_MCP
    case 10://HAND_NO_MIDDLE_FINGER_PIP
    case 11://HAND_NO_MIDDLE_FINGER_DIP
    case 12://HAND_NO_MIDDLE_FINGER_TIP
      color = middle_color;
      break;
    case 13://HAND_NO_RING_FINGER_MCP
    case 14://HAND_NO_RING_FINGER_PIP
    case 15://HAND_NO_RING_FINGER_DIP
    case 16://HAND_NO_RING_FINGER_TIP
      color = ring_color;
      break;
    case 17://HAND_NO_PINKY_MCP
    case 18://HAND_NO_PINKY_PIP
    case 19://HAND_NO_PINKY_DIP
    case 20://HAND_NO_PINKY_TIP
      color = pinky_color;
      break;
    default:
      color = center_color;
      break;
    }
    mediapipe::NormalizedLandmark landmark = landmarks.landmark(i);
    int parent_index = hand_vector_vec[i];
    mediapipe::NormalizedLandmark parent_landmark = landmarks.landmark(parent_index);
    int width = image.cols;
    int height = image.rows;
    
    if (i == parent_index) {
      // not draw line
    } else {
      cv::line(image, cv::Point(landmark.x() * width, landmark.y() * height), cv::Point(parent_landmark.x() * width, parent_landmark.y() * height), line_color, line_size, 4);
    }
    
    int radius = 4;
    int thickness = 2;
    int lineType = 8;
    int shift = 0;
    cv::circle(image, cv::Point(landmark.x() * width, landmark.y() * height), radius, color, thickness, lineType, shift);
    
    int peer_index = -1;
    switch (i) {
    case 5://HAND_NO_INDEX_FINGER_MCP
      peer_index = HAND_NO_THUMB_MCP;
      break;
    case 9://HAND_NO_MIDDLE_FINGER_MCP
      peer_index = HAND_NO_INDEX_FINGER_MCP;
      break;
    case 13://HAND_NO_RING_FINGER_MCP
      peer_index = HAND_NO_MIDDLE_FINGER_MCP;
      break;
    case 17://HAND_NO_PINKY_MCP
      peer_index = HAND_NO_RING_FINGER_MCP;
      break;
    default:
      break;
    }
    if (peer_index != -1) {
      mediapipe::NormalizedLandmark peer_landmark = landmarks.landmark(peer_index);
      cv::line(image, cv::Point(landmark.x() * width, landmark.y() * height), cv::Point(peer_landmark.x() * width, peer_landmark.y() * height), line_color, line_size, 4);
    }
  }
}

void drawLandmarksFace(cv::Mat& image, mediapipe::NormalizedLandmarkList& landmarks) {
  int width = image.cols;
  int height = image.rows;
  
  cv::Scalar line_color = cv::Scalar(255, 255, 255);
  cv::Scalar center_color = cv::Scalar(200, 255, 200);
  cv::Scalar pickup_color = cv::Scalar(100, 100, 255);
  cv::Scalar pickup_left_color = cv::Scalar(100, 100, 200);
  cv::Scalar pickup_right_color = cv::Scalar(200, 200, 255);
  cv::Scalar color = center_color;
  int line_size = 1;
  int radius = 1;
  int thickness = 2;
  int lineType = 8;
  int shift = 0;
  
  // debug test
  //if (true) {
  //  color = pickup_color;
  //  std::vector<int> draw_points;
  //  draw_points.push_back(FACE_MESH_NO_NOSE);
  //  draw_points.push_back(FACE_MESH_NO_TOP);
  //  draw_points.push_back(FACE_MESH_NO_BOTTOM);
  //  draw_points.push_back(FACE_MESH_NO_LEFT);
  //  draw_points.push_back(FACE_MESH_NO_RIGHT);
  //  draw_points.push_back(FACE_MESH_NO_CENTER);
  //  draw_points.push_back(FACE_MESH_NO_NOSE_INNER);
  //  for (int i = 0; i < draw_points.size(); i++) {
  //    mediapipe::NormalizedLandmark landmark = landmarks.landmark(draw_points[i]);
  //    cv::circle(image, cv::Point(landmark.x() * width, landmark.y() * height), radius, color, thickness, lineType, shift);
  //  }
  //  drawCircleLines(image, landmarks, face_around_silhouette, true, line_color);
  //  return;
  //}
  
  for (int i = 0; i < landmarks.landmark_size(); i++) {
    switch (i) {
    case -1://FACE_MESH_NO_XXX // not 2d
      // not draw
      continue;
    case 1://FACE_MESH_NO_NOSE
    case 10://FACE_MESH_NO_TOP
    case 152://FACE_MESH_NO_BOTTOM
    case 13://FACE_MESH_NO_LIP_INNER_UPPER
    case 14://FACE_MESH_NO_LIP_INNER_LOWER
    case 478://FACE_MESH_NO_LIP_CENTER
    case 479://FACE_MESH_NO_EYE_CENTER
    case 483://FACE_MESH_NO_CENTER
    case 487://FACE_MESH_NO_NOSE_INNER
      color = pickup_color;
      break;
    case 323://FACE_MESH_NO_LEFT
    case 425://FACE_MESH_NO_LEFT_CHEEK
    case 308://FACE_MESH_NO_LIP_INNER_LEFT_SIDE
    case 386://FACE_MESH_NO_LEFT_EYE_UPPER
    case 374://FACE_MESH_NO_LEFT_EYE_LOWER
    case 362://FACE_MESH_NO_LEFT_EYE_IN_SIDE
    case 263://FACE_MESH_NO_LEFT_EYE_OUT_SIDE
    case 417://FACE_MESH_NO_LEFT_EYE_BROW_UPPER_IN_SIDE
    case 334://FACE_MESH_NO_LEFT_EYE_BROW_UPPER_CENTER
    case 383://FACE_MESH_NO_LEFT_EYE_BROW_UPPER_OUT_SIDE
    case 468://FACE_MESH_NO_LEFT_EYE_IRIS_CENTER
    case 480://FACE_MESH_NO_LEFT_EYE_CENTER
    case 251://FACE_MESH_NO_LEFT_UPPER_FACE
    case 197://FACE_MESH_NO_LEFT_LOWER_FACE
      color = pickup_left_color;
      break;
    case 93://FACE_MESH_NO_RIGHT
    case 205://FACE_MESH_NO_RIGHT_CHEEK
    case 78://FACE_MESH_NO_LIP_INNER_RIGHT_SIDE
    case 159://FACE_MESH_NO_RIGHT_EYE_UPPER
    case 145://FACE_MESH_NO_RIGHT_EYE_LOWER
    case 133://FACE_MESH_NO_RIGHT_EYE_IN_SIDE
    case 33://FACE_MESH_NO_RIGHT_EYE_OUT_SIDE
    case 193://FACE_MESH_NO_RIGHT_EYE_BROW_UPPER_IN_SIDE
    case 105://FACE_MESH_NO_RIGHT_EYE_BROW_UPPER_CENTER
    case 156://FACE_MESH_NO_RIGHT_EYE_BROW_UPPER_OUT_SIDE
    case 473://FACE_MESH_NO_RIGHT_EYE_IRIS_CENTER
    case 481://FACE_MESH_NO_RIGHT_EYE_CENTER
    case 21://FACE_MESH_NO_RIGHT_UPPER_FACE
    case 172://FACE_MESH_NO_RIGHT_LOWER_FACE
      color = pickup_right_color;
      break;
    default:
      color = center_color;
      break;
    }
    mediapipe::NormalizedLandmark landmark = landmarks.landmark(i);
    //int parent_index = face_mesh_vector_vec[i];
    int parent_index = i;
    mediapipe::NormalizedLandmark parent_landmark = landmarks.landmark(parent_index);
    
    if (i == parent_index) {
      // not draw line
    } else {
      cv::line(image, cv::Point(landmark.x() * width, landmark.y() * height), cv::Point(parent_landmark.x() * width, parent_landmark.y() * height), line_color, line_size, 4);
    }
    
    cv::circle(image, cv::Point(landmark.x() * width, landmark.y() * height), radius, color, thickness, lineType, shift);
    
    int peer_index = -1;
    switch (i) {
    case 10://FACE_MESH_NO_TOP
      peer_index = FACE_MESH_NO_BOTTOM;
      break;
    case 323://FACE_MESH_NO_LEFT
      peer_index = FACE_MESH_NO_RIGHT;
      break;
    case 1://FACE_MESH_NO_NOSE
      peer_index = FACE_MESH_NO_NOSE_INNER;
      break;
    case 358://FACE_MESH_NO_NOSE_OUTSIDE_LEFT
      peer_index = FACE_MESH_NO_NOSE_OUTSIDE_RIGHT;
      break;
    default:
      break;
    }
    if (peer_index != -1) {
      mediapipe::NormalizedLandmark peer_landmark = landmarks.landmark(peer_index);
      cv::line(image, cv::Point(landmark.x() * width, landmark.y() * height), cv::Point(peer_landmark.x() * width, peer_landmark.y() * height), line_color, line_size, 4);
    }
  }
  
  // silhouette
  drawCircleLines(image, landmarks, face_around_silhouette, true, line_color);
  drawCircleLines(image, landmarks, face_lipsUpperOuter_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_lipsLowerOuter_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_lipsUpperInner_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_lipsLowerInner_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_rightEyeUpper0_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_rightEyeLower0_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_rightEyeUpper1_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_rightEyeLower1_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_rightEyeUpper2_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_rightEyeLower2_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_rightEyeLower3_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_rightEyebrowUpper_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_rightEyebrowLower_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_rightEyeIris_silhouette, true, line_color);
  drawCircleLines(image, landmarks, face_leftEyeUpper0_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_leftEyeLower0_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_leftEyeUpper1_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_leftEyeLower1_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_leftEyeUpper2_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_leftEyeLower2_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_leftEyeLower3_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_leftEyebrowUpper_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_leftEyebrowLower_silhouette, false, line_color);
  drawCircleLines(image, landmarks, face_leftEyeIris_silhouette, true, line_color);
  //drawCircleLines(image, landmarks, face_midwayBetweenEyes_silhouette, false, line_color);
  //drawCircleLines(image, landmarks, face_noseTip_silhouette, false, line_color);
  //drawCircleLines(image, landmarks, face_noseBottom_silhouette, false, line_color);
  //drawCircleLines(image, landmarks, face_noseRightCorner_silhouette, false, line_color);
  //drawCircleLines(image, landmarks, face_noseLeftCorner_silhouette, false, line_color);
  //drawCircleLines(image, landmarks, face_rightCheek_silhouette, false, line_color);
  //drawCircleLines(image, landmarks, face_leftCheek_silhouette, false, line_color);
}

void drawCircleLines(cv::Mat& image, mediapipe::NormalizedLandmarkList& landmarks, std::vector<int>& silhouette, bool is_circle, cv::Scalar color) {
  int line_size = 1;
  int width = image.cols;
  int height = image.rows;
  
  for (int i = 0; i < silhouette.size(); i++) {
    int target_point = silhouette[i];
    int peer_point = -1;
    if (i == silhouette.size() - 1) {
      if (is_circle == true) {
        peer_point = silhouette[0];
      } else {
        break;
      }
    } else {
      peer_point = silhouette[i + 1];
    }
    if (target_point == peer_point) {
      continue;
    }
    
    mediapipe::NormalizedLandmark target_landmark = landmarks.landmark(target_point);
    mediapipe::NormalizedLandmark peer_landmark = landmarks.landmark(peer_point);
    cv::line(image, cv::Point(target_landmark.x() * width, target_landmark.y() * height), cv::Point(peer_landmark.x() * width, peer_landmark.y() * height), color, line_size, 4);
  }
}

//------------------------------------------------------------------------------
// image end
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// file start
//------------------------------------------------------------------------------

long getFileSize(std::string& file_path) {
  FILE* f = fopen(file_path.c_str(), "rb");
  if (f == NULL) {
    LOG(INFO) << "fopen(" << file_path << ") return value error(NULL).";
    return -1;
  }
  long file_size = -1;
  fseek(f, 0, SEEK_END);
  file_size = ftell(f);
  rewind(f);
  fclose(f);
  return  file_size;
}

char* loadBinaryFile(std::string& file_path) {
  int file_size = getFileSize(file_path);
  if (file_size == -1) {
    LOG(INFO) << "getFileSize(" << file_path << ") return value error(-1).";
    return NULL;
  }
  char* data = (char*)calloc(sizeof(char), file_size + 1);
  FILE* f = fopen(file_path.c_str(), "rb");
  int read_size = fread(data, sizeof(char), file_size, f);
  fclose(f);
  if (read_size != file_size) {
    LOG(INFO) << "fread(" << file_path << ") return value(" << read_size << ") error(not file size(" << file_size << ")).";
    return NULL;
  }
  return data;
}

char* loadTextFile(std::string& file_path) {
  int file_size = getFileSize(file_path);
  if (file_size == -1) {
    LOG(INFO) << "getFileSize(" << file_path << ") return value error(-1).";
    return NULL;
  }
  char* data = (char*)calloc(sizeof(char), file_size + 1);
  FILE* f = fopen(file_path.c_str(), "rb"); // rだとファイルサイズが不一致となる。改行コードのサイズが異なる様子
  int read_size = fread(data, sizeof(char), file_size, f);
  fclose(f);
  if (read_size != file_size) {
    LOG(INFO) << "fread(" << file_path << ") return value(" << read_size << ") error(not file size(" << file_size << ")).";
    return NULL;
  }
  return data;
}

bool check_exist_file(std::string file_path) {
  bool is_exist = std::filesystem::exists(file_path);
  return is_exist;
}


//------------------------------------------------------------------------------
// file end
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// action start
//------------------------------------------------------------------------------

void doKeyEvent(int key_code, int key_flag, int wait_mili_second) {
  if (is_use_keyboard_mouse_event_now == true) {
    // old duple
    keybd_event(key_code, 0, key_flag, 0);
  } else {
    // now
    INPUT input;
    input.type = INPUT_KEYBOARD;
    input.ki.wVk = key_code;
    input.ki.dwFlags = key_flag;
    int ret = SendInput(1, &input, sizeof(INPUT));
    if (ret != 0) {
      LOG(INFO) << "doKeyEvent() SendInput() return value error: " << ret;
    }
  }
}

void doMouseEvent(int event, int dx, int dy, bool is_absolute) {
  INPUT input;
  input.type = INPUT_MOUSE;
  if (event & MOUSEEVENTF_ABSOLUTE) {
    input.mi.dx = (int)(dx * display_rate_x);
    input.mi.dy = (int)(dy * display_rate_y);
  } else {
    input.mi.dx = dx;
    input.mi.dy = dy;
  }
  if (is_absolute == true) {
    input.mi.dwFlags = MOUSEEVENTF_ABSOLUTE;
  } else {
    input.mi.dwFlags = 0;
  }
  input.mi.mouseData = 0;
  input.mi.dwExtraInfo = NULL;
  input.mi.time = 0;
  input.mi.dwFlags |= event;
  
  LOG(INFO) << "mouse_event point dx(" << dx << "), dy(" << dy << "), display_rate_x(" << display_rate_x << "), display_rate_y(" << display_rate_y << ").";
  LOG(INFO) << "mouse_event(" << input.mi.dwFlags << ", " << input.mi.dx << ", " << input.mi.dy << ", , ) do.";
  
  if (is_use_keyboard_mouse_event_now == true) {
    // old
    // mouse_event
    mouse_event(input.mi.dwFlags, input.mi.dx, input.mi.dy, input.mi.mouseData, input.mi.dwExtraInfo);
  } else {
    // now
    // SendInput
    SendInput(1, &input, sizeof(INPUT));
  }
}

void doMultiKeyEvent(int key_code, int key_flag, int wait_mili_second) {
  if (key_code < 0xff) {
    doKeyEvent(key_code, key_flag, 0);
    //keybd_event(key_code, 0, key_flag, 0);
    LOG(INFO) << "keybd_event(" << key_code << ", , " << key_flag << ",) do.";
  } else {
    int key_code_12 = (0xff00 & key_code) >> 8;
    doKeyEvent(key_code_12, key_flag, 0);
    //keybd_event(key_code_12, 0, key_flag, 0);
    LOG(INFO) << "keybd_event(" << key_code_12 << ", , " << key_flag << ",) do.";
    int key_code_34 = (0x00ff & key_code);
    doKeyEvent(key_code_34, key_flag, 0);
    //keybd_event(key_code_34, 0, key_flag, 0);
    LOG(INFO) << "keybd_event(" << key_code_34 << ", , " << key_flag << ",) do.";
  }
}

int doMultiEvent(std::vector<int> codes, int index, int add_flag, int wait_mili_second) {
  int increment_index = 0; // 0 originとする（1にはしない）
  int code = codes[index];
  LOG(INFO) << "doMultiEvent() code(" << code << ") index(" << index << ") codes.size(" << codes.size() << ") start.";
  if (code & SLEEP_FLAG) {
    int sleep_msec = code - SLEEP_FLAG;
    //int flag = add_flag | sleep_default_flag; // TODO no use
    LOG(INFO) << "doMultiEvent() sleep(" << sleep_msec << ") start.";
    Sleep(sleep_msec);
    LOG(INFO) << "doMultiEvent() sleep(" << sleep_msec << ") end.";
  } else if (code & SLEEP_NANO_FLAG) {
    int sleep_nsec = code - SLEEP_FLAG;
    //int flag = add_flag | sleep_default_flag; // TODO no use
    LOG(INFO) << "doMultiEvent() sleep_for(std::chrono::nanoseconds(" << sleep_nsec << ")) start.";
    std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_nsec));
    LOG(INFO) << "doMultiEvent() sleep_for(std::chrono::nanoseconds(" << sleep_nsec << ")) end.";
  } else if (code & MOUSE_FLAG) {
    // mouse event
    if (codes.size() <= index + MOUSE_INDEX_INCREMENT) {
      LOG(INFO) << "doMultiEvent() mouse event: codes.size(" << codes.size() << ") =< index(" << index << ") + MOUSE_INDEX_INCREMENT(" << MOUSE_INDEX_INCREMENT << ")";
      // TODO error
      // return;
    }
    int flag = add_flag | mouse_default_flag;
    int mouse_code = code - MOUSE_FLAG;
    int dx = codes[index + MOUSE_INDEX_D_X];
    int dy = codes[index + MOUSE_INDEX_D_Y];
    bool is_absolute = true;
    is_absolute = false; // not use this flag, use key_code in absolute flag.
    //if (flag & MOUSE_F_NO_ABSOLUTE) {
    //  LOG(INFO) << "doMultiEvent() add_flag(" << add_flag << ") flag(" << flag << ") & MOUSE_F_NO_ABSOLUTE(" << MOUSE_F_NO_ABSOLUTE << ") is true";
    //  is_absolute = false;
    //}
    doMouseEvent(mouse_code, dx, dy, is_absolute);
    //mouseEvent(mouse_code, dx, dy, is_absolute);
    LOG(INFO) << "doMultiEvent() mouseEvent(" << mouse_code << ", " << dx << ", " << dy << ", " << is_absolute << ") flag(" << flag << ") end.";
    increment_index += MOUSE_INDEX_INCREMENT;
  } else if (code & KEYBD_FLAG) {
    // key board event
    int flag = add_flag | key_default_flag;
    int key_code = code - KEYBD_FLAG;
    doMultiKeyEvent(key_code, flag, wait_mili_second);
  } else {
    LOG(INFO) << "doMultiEvent() unknown flag, skip code: " << code;
  }
  return increment_index;
}


int getMousePoint(int& x, int& y) {
  POINT point;
  bool isGet = GetCursorPos(&point);
  if (isGet == false) {
    LOG(INFO) << "GetCursorPos() return value error.";
    return -1;
  }
  x = point.x;
  y = point.y;
  return 0;
}

void mouseEvent(int event, int dx, int dy, bool is_absolute) {
  INPUT input;
  input.type = INPUT_MOUSE;
  if (event & MOUSEEVENTF_ABSOLUTE) {
    input.mi.dx = (int)(dx * display_rate_x);
    input.mi.dy = (int)(dy * display_rate_y);
  } else {
    input.mi.dx = dx;
    input.mi.dy = dy;
  }
  if (is_absolute == true) {
    input.mi.dwFlags = MOUSEEVENTF_ABSOLUTE;
  } else {
    input.mi.dwFlags = 0;
  }
  input.mi.mouseData = 0;
  input.mi.dwExtraInfo = NULL;
  input.mi.time = 0;
  input.mi.dwFlags |= event;
  
  //SendInput(1, &input, sizeof(INPUT));
  LOG(INFO) << "mouse_event point dx(" << dx << "), dy(" << dy << "), display_rate_x(" << display_rate_x << "), display_rate_y(" << display_rate_y << ").";
  LOG(INFO) << "mouse_event(" << input.mi.dwFlags << ", " << input.mi.dx << ", " << input.mi.dy << ", , ) do.";
  mouse_event(input.mi.dwFlags, input.mi.dx, input.mi.dy, input.mi.mouseData, input.mi.dwExtraInfo);
}


//------------------------------------------------------------------------------
// action end
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// mix start
//------------------------------------------------------------------------------

// no use
// coordinate v1
// T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> bool runCalcPoseCoordinateAndAction_v1(T& target_landmark) {
  bool is_match_one = false;
  if (target_landmark.x() >= key_arrow_left_right_threshold[NO_KEY_MAX_OVER_THRESHOLD]) {
    // left
    LOG(INFO) << "coordinate left/right action: left";
    doMultiKeyEvent(KEY_LEFT_ARROW, key_default_flag);
    is_match_one = true;
  } else if (target_landmark.x() >= key_arrow_left_right_threshold[NO_KEY_MIDDLE_OVER_THRESHOLD]) {
    // middle
    LOG(INFO) << "coordinate left/right action: middle";
    doMultiKeyEvent(KEY_LEFT_ARROW, key_default_flag | KEYEVENTF_KEYUP);
    doMultiKeyEvent(KEY_RIGHT_ARROW, key_default_flag | KEYEVENTF_KEYUP);
  } else {
    // right
    LOG(INFO) << "coordinate left/right action: right";
    doMultiKeyEvent(KEY_RIGHT_ARROW, key_default_flag);
    is_match_one = true;
  }
  if (target_landmark.y() >= key_arrow_down_up_threshold[NO_KEY_MIN_OVER_THRESHOLD]) {
    // up
    LOG(INFO) << "coordinate down/up action: down";
    doMultiKeyEvent(KEY_DOWN_ARROW, key_default_flag);
    is_match_one = true;
  } else if (target_landmark.y() >= key_arrow_down_up_threshold[NO_KEY_MIDDLE_OVER_THRESHOLD]) {
    // middle
    LOG(INFO) << "coordinate down/up action: middle";
    doMultiKeyEvent(KEY_UP_ARROW, key_default_flag | KEYEVENTF_KEYUP);
    doMultiKeyEvent(KEY_DOWN_ARROW, key_default_flag | KEYEVENTF_KEYUP);
  } else {
    // down
    LOG(INFO) << "coordinate down/up action: up";
    doMultiKeyEvent(KEY_UP_ARROW, key_default_flag);
    is_match_one = true;
  }
  return is_match_one;
}

// coordinate v2
template <class T> float getLandmarkValueFromCode(T& landmark, /*int& landmarks_array_no, */int& value_no) {
  switch (value_no) {
  case MP_LANDMARK_VALUE_NO_X:
    return landmark.x();
    break;
  case MP_LANDMARK_VALUE_NO_Y:
    return landmark.y();
    break;
  case MP_LANDMARK_VALUE_NO_Z:
    return landmark.z();
    break;
  case MP_LANDMARK_VALUE_NO_VISIBILITY:
    return landmark.visibility();
    break;
  case MP_LANDMARK_VALUE_NO_PRESENCE:
    return landmark.presence();
    break;
  default:
    LOG(INFO) << "getLandmarkValueFromCode() value_no(" << value_no << ") is unknown, return -1.0";
    return -1.0;
    break;
  }
}

float getCoordinateValueFromCode(std::vector<int>& landmark_code) {
  int coordinate_mode_no = landmark_code[KEY_POINT_COORDINATE_MODE_NO];
  int landmarks_array_no = landmark_code[KEY_POINT_LANDMARKS_ARRAY_NO];
  int value_no = landmark_code[KEY_POINT_LANDMARK_COORDINATE_NO];
  switch (coordinate_mode_no) {
  case COORDINATE_MODE_LAST_2D:
    if (last_target_vec_2d.landmark_size() < landmarks_array_no) {
      LOG(INFO) << "last_target_vec_2d.landmark_size(" << last_target_vec_2d.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), return -1.0";
      return -1.0;
    }
    return getLandmarkValueFromCode(last_target_vec_2d.landmark(landmarks_array_no), /*landmarks_array_no, */value_no);
    break;
  case COORDINATE_MODE_LAST_3D:
    if (last_target_vec.landmark_size() < landmarks_array_no) {
      LOG(INFO) << "last_target_vec.landmark_size(" << last_target_vec.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), return -1.0";
      return -1.0;
    }
    return getLandmarkValueFromCode(last_target_vec.landmark(landmarks_array_no), /*landmarks_array_no, */value_no);
    break;
  case COORDINATE_MODE_LAST_3D_VEC_ANGLE:
    if (last_target_vec_angle.landmark_size() < landmarks_array_no) {
      LOG(INFO) << "last_target_vec_angle.landmark_size(" << last_target_vec_angle.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), return -1.0";
      return -1.0;
    }
    return getLandmarkValueFromCode(last_target_vec_angle.landmark(landmarks_array_no), /*landmarks_array_no, */value_no);
    break;
  case COORDINATE_MODE_LAST_CAMERA_DIFF_ANGLE:
    if (last_target_camera_diff_angle.landmark_size() < landmarks_array_no) {
      LOG(INFO) << "last_target_camera_diff_angle.landmark_size(" << last_target_camera_diff_angle.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), return -1.0";
      return -1.0;
    }
    return getLandmarkValueFromCode(last_target_camera_diff_angle.landmark(landmarks_array_no), /*landmarks_array_no, */value_no);
    break;
  case COORDINATE_MODE_BASE_2D:
    if (base_diff_target_vec_2d.landmark_size() < landmarks_array_no) {
      LOG(INFO) << "base_diff_target_vec_2d.landmark_size(" << base_diff_target_vec_2d.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), return -1.0";
      return -1.0;
    }
    return getLandmarkValueFromCode(base_diff_target_vec_2d.landmark(landmarks_array_no), /*landmarks_array_no, */value_no);
    break;
  case COORDINATE_MODE_BASE_3D:
    if (base_diff_target_vec.landmark_size() < landmarks_array_no) {
      LOG(INFO) << "base_diff_target_vec.landmark_size(" << base_diff_target_vec.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), return -1.0";
      return -1.0;
    }
    return getLandmarkValueFromCode(base_diff_target_vec.landmark(landmarks_array_no), /*landmarks_array_no, */value_no);
    break;
  case COORDINATE_MODE_BASE_3D_VEC_ANGLE:
    if (base_diff_target_vec_angle.landmark_size() < landmarks_array_no) {
      LOG(INFO) << "base_diff_target_vec_angle.landmark_size(" << base_diff_target_vec_angle.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), return -1.0";
      return -1.0;
    }
    return getLandmarkValueFromCode(base_diff_target_vec_angle.landmark(landmarks_array_no), /*landmarks_array_no, */value_no);
    break;
  case COORDINATE_MODE_BASE_CAMERA_DIFF_ANGLE:
    if (base_diff_target_camera_diff_angle.landmark_size() < landmarks_array_no) {
      LOG(INFO) << "base_diff_target_camera_diff_angle.landmark_size(" << base_diff_target_camera_diff_angle.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), return -1.0";
      return -1.0;
    }
    return getLandmarkValueFromCode(base_diff_target_camera_diff_angle.landmark(landmarks_array_no), /*landmarks_array_no, */value_no);
    break;
  default:
    LOG(INFO) << "getCoordinateValueFromCode() coordinate_mode_no(" << coordinate_mode_no << ") is unknown, return -1.0";
    return -1.0;
    break;
  }
}

bool isConditionValue(float& value, float& threshold, int& condition_no) {
  switch (condition_no) {
  case CONDITION_LESS_THAN:
    if (value < threshold) {
      return true;
    } else {
      return false;
    }
    break;
  case CONDITION_EQUAL_LESS_THAN:
    if (value <= threshold) {
      return true;
    } else {
      return false;
    }
    break;
  case CONDITION_EQUAL:
    if (value == threshold) {
      return true;
    } else {
      return false;
    }
    break;
  case CONDITION_EQUAL_MORE_THAN:
    if (value >= threshold) {
      return true;
    } else {
      return false;
    }
    break;
  case CONDITION_MORE_THAN:
    if (value > threshold) {
      return true;
    } else {
      return false;
    }
    break;
  default:
    LOG(INFO) << "unknown condition_no(" << condition_no << ") return false.";
    return false;
    break;
  }
}

bool runCalcPoseCoordinateAndAction_v2(std::vector<std::vector<int> >& target_key_code_list, std::vector<std::vector<int> >& target_key_points, std::vector<std::pair<float, int> >& threshold_condition_list, std::vector<int>& target_flag_list, int only_no) {
  bool is_match_one = false;
  int start_action_i = 0;
  int end_action_i = target_key_code_list.size();
  if (only_no != -1) {
    start_action_i = only_no;
    end_action_i = only_no + 1;
  }
  for (int i = start_action_i; i < end_action_i; i++) {
    if (only_no == -1) {
      // 先頭がダミーだとスキップする
      if (target_key_code_list[i][0] == ACTION_CODE_DUMMY) {
        continue;
      }
    }
    float target_value = getCoordinateValueFromCode(target_key_points[i]);
    std::pair<float, int> threshold_condition = threshold_condition_list[i];
    
    bool is_match = isConditionValue(target_value, threshold_condition.first, threshold_condition.second);
    LOG(INFO) << "coord_v2() no(" << i << ") match(" << is_match << ") target_value(" << target_value << ") threshold(" << threshold_condition.first << ") condition(" << threshold_condition.second << ")";
    if (only_no != -1) {
      // 指定された場合、ここで終了する
      if (is_match == true) {
        return true;
      } else {
        return false;
      }
    }
    if (is_match == true) {
      is_match_one = true;
    }
    std::vector<int> target_key_codes = target_key_code_list[i];
    int flag = target_flag_list[i];
    int last_key_code = get_latest_key_code(target_key_codes[target_key_codes.size() - 1]);
    bool is_last_run_no_up = false;
    if (flag & FD_V1_FLAG_NO_UP_LAST_KEY) {
      if (last_key_code == target_key_codes[target_key_codes.size() - 1]) {
        // already run. do nothing.
        continue;
      } else {
        // last run no up, other normal run
        is_last_run_no_up = true;
      }
    }
    for (int j = 0; j < target_key_codes.size(); j++) {
      int target_key_code = target_key_codes[j];
      int jj = 0;
      if (is_match == true) {
        //doMultiKeyEvent(target_key_code, key_default_flag);
        //jj = doMultiEvent(target_key_codes, j, 0);
        jj = push_or_pushing_latest_running_key_code(target_key_codes, j);
        LOG(INFO) << "coord_v2() no(" << i << ", " << j << ") event(" << target_key_code << ") start";
        if (!(flag & FD_V1_FLAG_NO_UP_AFTER_DOWN)) {
          //if (target_key_codes[j] & KEYBD_FLAG) {
          //  //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
          //  doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
          //  clear_latest_running_key_code(target_key_codes[j]);
          //  LOG(INFO) << "coord_v2() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
          //} else if (target_key_codes[j] & MOUSE_FLAG) {
          //  // do nothing
          //}
          
          if (is_last_run_no_up == false) {
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "coord_v2() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
            }
          } else {
            if (j != target_key_codes.size() - 1) {
              // not last is up
              if (target_key_codes[j] & KEYBD_FLAG) {
                //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
                doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
                clear_latest_running_key_code(target_key_codes[j]);
                LOG(INFO) << "coord_v2() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
              } else if (target_key_codes[j] & MOUSE_FLAG) {
                // do nothing
              }
            } else {
              // last is no up, update key
              update_latest_running_key_code(target_key_codes[j]);
            }
          }
        } else {
          if (target_key_codes[j] & KEYBD_FLAG) {
            update_latest_running_key_code(target_key_codes[j]);
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
          }
        }
        if (flag & FD_V1_FLAG_LOOP) {
          queue_key_code.push_back(std::pair<int, int>(target_key_code, flag));
        }
      } else {
        // TODO 必要？ほかのキーが押されると不要になる
        if (!(flag & FD_V1_FLAG_NO_UP_NOT_MATCH)) {
          if (target_key_codes[j] & KEYBD_FLAG) {
            //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
            jj = doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
            clear_latest_running_key_code(target_key_codes[j]);
            LOG(INFO) << "coord_v2() no(" << i << ", " << j << ") not match event(" << target_key_code << ") end";
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
            jj += MOUSE_INDEX_INCREMENT;
          }
        }
      }
      j += jj;
    }
  }
  return is_match_one;
}

// coordinate v3
bool runCalcPoseCoordinateAndAction_v3(std::vector<std::vector<int> >& target_key_code_list, std::vector<std::vector<int> >& target_key_points, std::vector<int>& target_flag_lists, int only_no) {
  bool is_match_one = false;
  int start_action_i = 0;
  int end_action_i = target_key_code_list.size();
  if (only_no != -1) {
    start_action_i = only_no;
    end_action_i = only_no + 1;
  }
  for (int i = start_action_i; i < end_action_i; i++) {
    if (only_no == -1) {
      // 先頭がダミーだとスキップする
      if (target_key_code_list[i][0] == ACTION_CODE_DUMMY) {
        continue;
      }
    }
    bool is_match = true;
    int points_count = target_key_points[i].size();
    for (int j = 0; j < points_count; j += KEY_POINT_V3_MAX) {
      if (j + KEY_POINT_V3_MAX - 1 > points_count) {
        LOG(INFO) << "coord_v3() no(" << i << ", " << j << ") size(" << points_count << ") error, not multiple 5, skip this.";
        continue;
      }
      std::vector<int> sub_target_key_points(target_key_points[i].begin() + j, target_key_points[i].begin() + j + KEY_POINT_V3_MAX);
      float target_value = getCoordinateValueFromCode(sub_target_key_points);
      float threshold_value = (float)target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] / KEY_POINT_THRESHOLD_VALUE_DIVID;
      int condition_no = target_key_points[i][KEY_POINT_CONDITION_NO + j];
      
      is_match = isConditionValue(target_value, threshold_value, condition_no);
      LOG(INFO) << "coord_v3() no(" << i << ", " << j << ") match(" << is_match << ") target_value(" << target_value << ") threshold(" << threshold_value << ") condition no(" << condition_no << ")";
      if (only_no != -1) {
        // 指定された場合、ここで終了する
        if (is_match == true) {
          // ok, check next condition
        } else {
          // ng, stop check condition
          return false;
        }
      } else {
        if (is_match == true) {
          // ok, check next condition
        } else {
          // ng, stop check condition
          break;
        }
      }
    }
    if (is_match == true) {
      is_match_one = true;
    }
    std::vector<int> target_key_codes = target_key_code_list[i];
    int flag = target_flag_lists[i];
    int last_key_code = get_latest_key_code(target_key_codes[target_key_codes.size() - 1]);
    bool is_last_run_no_up = false;
    if (flag & FD_V1_FLAG_NO_UP_LAST_KEY) {
      if (last_key_code == target_key_codes[target_key_codes.size() - 1]) {
        // already run. do nothing.
        continue;
      } else {
        // last run no up, other normal run
        is_last_run_no_up = true;
      }
    }
    if (is_match == true) {
      for (int j = 0; j < target_key_codes.size(); j++) {
        int target_key_code = target_key_codes[j];
        //doMultiKeyEvent(target_key_code, key_default_flag);
        //int jj = doMultiEvent(target_key_codes, j, 0);
        int jj = push_or_pushing_latest_running_key_code(target_key_codes, j);
        LOG(INFO) << "coord_v3() no(" << i << ", " << j << ") event(" << target_key_code << ") start";
        if (!(flag & FD_V1_FLAG_NO_UP_AFTER_DOWN)) {
          //if (target_key_codes[j] & KEYBD_FLAG) {
          //  //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
          //  doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
          //  clear_latest_running_key_code(target_key_codes[j]);
          //  LOG(INFO) << "coord_v3() no(" << i << ", " << j << ") event(" << target_key_code << ") end";
          //} else if (target_key_codes[j] & MOUSE_FLAG) {
          //  // do nothing
          //}
          
          if (is_last_run_no_up == false) {
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "coord_v3() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
            }
          } else {
            if (j != target_key_codes.size() - 1) {
              // not last is up
              if (target_key_codes[j] & KEYBD_FLAG) {
                //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
                doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
                clear_latest_running_key_code(target_key_codes[j]);
                LOG(INFO) << "coord_v3() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
              } else if (target_key_codes[j] & MOUSE_FLAG) {
                // do nothing
              }
            } else {
              // last is no up, update key
              update_latest_running_key_code(target_key_codes[j]);
            }
          }
        } else {
          if (target_key_codes[j] & KEYBD_FLAG) {
            update_latest_running_key_code(target_key_codes[j]);
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
          }
        }
        j += jj;
      }
      coord_v3_last_run_key.clear();
      coord_v3_last_run_key.push_back(i);
      if (flag & FD_V1_FLAG_LOOP) {
        for (int j = 0; j < target_key_codes.size(); j++) {
          int target_key_code = target_key_codes[j];
          queue_key_code.push_back(std::pair<int, int>(target_key_code, flag));
        }
      }
    } else {
      if (!(flag & FD_V1_FLAG_NO_UP_NOT_MATCH)) {
        if (coord_v3_last_run_key.size() >= 1 && coord_v3_last_run_key[0] == i) {
          for (int j = 0; j < target_key_codes.size(); j++) {
            int target_key_code = target_key_codes[j];
            int jj = 0;
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              jj = doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "coord_v3() no(" << i << ", " << j << ") event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
              jj += MOUSE_INDEX_INCREMENT;
            }
            j += jj;
          }
          if (coord_v3_last_run_key.empty() == false) {
            coord_v3_last_run_key.clear();
          }
        } else {
          // not last run, therefore not run keyup
        }
      }
    }
  }
  return is_match_one;
}


// sum v1
// T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class T> float getVectorValueFrom2Landmark(T& landmark, T& landmark_2nd) {
  float d_x = landmark.x() - landmark_2nd.x();
  float d_y = landmark.y() - landmark_2nd.y();
  float d_z = landmark.z() - landmark_2nd.z();
  float d_xyz = (d_x * d_x) + (d_y * d_y) + (d_z * d_z);
  float vector_length = pow(d_xyz, 0.5);
  return vector_length;
}

float getVectorValueFromCode(std::vector<int> landmark_code, float visibility_threshold) {
  float sum_vector = (float)0.0;
  for (int i = 0; i < landmark_code.size(); i += 2) {
    int coordinate_mode_no = landmark_code[KEY_POINT_COORDINATE_MODE_NO + i];
    int landmarks_array_no = landmark_code[KEY_POINT_LANDMARKS_ARRAY_NO + i];
    switch (coordinate_mode_no) {
    case COORDINATE_MODE_LAST_2D:
      if (last_target_vec_2d.landmark_size() < landmarks_array_no) {
        LOG(INFO) << "last_target_vec_2d.landmark_size(" << last_target_vec_2d.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), result is 0.0";
      }
      if (last_2nd_target_vec_2d.landmark_size() < landmarks_array_no) {
        LOG(INFO) << "last_2nd_target_vec_2d.landmark_size(" << last_2nd_target_vec_2d.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), result is 0.0";
      }
      if (visibility_threshold > last_target_vec_2d.landmark(landmarks_array_no).visibility()) {
        break;
      }
      sum_vector += getVectorValueFrom2Landmark(last_target_vec_2d.landmark(landmarks_array_no), last_2nd_target_vec_2d.landmark(landmarks_array_no));
      break;
    case COORDINATE_MODE_LAST_3D:
      if (last_target_vec.landmark_size() < landmarks_array_no) {
        LOG(INFO) << "last_target_vec.landmark_size(" << last_target_vec.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), result is 0.0";
      }
      if (last_2nd_target_vec.landmark_size() < landmarks_array_no) {
        LOG(INFO) << "last_2nd_target_vec.landmark_size(" << last_2nd_target_vec.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), result is 0.0";
      }
      if (visibility_threshold > last_target_vec.landmark(landmarks_array_no).visibility()) {
        break;
      }
      sum_vector += getVectorValueFrom2Landmark(last_target_vec.landmark(landmarks_array_no), last_2nd_target_vec.landmark(landmarks_array_no));
      break;
    case COORDINATE_MODE_LAST_3D_VEC_ANGLE:
      if (last_target_vec_angle.landmark_size() < landmarks_array_no) {
        LOG(INFO) << "last_target_vec_angle.landmark_size(" << last_target_vec_angle.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), result is 0.0";
      }
      if (last_2nd_target_vec_angle.landmark_size() < landmarks_array_no) {
        LOG(INFO) << "last_2nd_target_vec_angle.landmark_size(" << last_2nd_target_vec_angle.landmark_size() << ") is over landmarks_array_no(" << landmarks_array_no << "), result is 0.0";
      }
      sum_vector += getVectorValueFrom2Landmark(last_target_vec_angle.landmark(landmarks_array_no), last_2nd_target_vec_angle.landmark(landmarks_array_no));
      break;
    default:
      LOG(INFO) << "getVectorValueFromCode() coordinate_mode_no(" << coordinate_mode_no << ") is unknown, result is 0.0";
      break;
    }
  }
  return sum_vector;
}

bool runSumAndCalcPoseVectorAndAction_v1(std::vector<std::vector<int> >& sum_target_key_code_list, std::vector<std::vector<int> >& sum_target_key_points, std::vector<std::pair<float, int> >& sum_threshold_condition_list, std::vector<float>& sum_vector_list, std::vector<int>& sum_target_flag_list, int only_no) {
  //LOG(INFO) << "runSumAndCalcPoseVectorAndAction_v1() start";
  if (last_target_vec.landmark_size() == 0) {
    return false;
  }
  if (last_2nd_target_vec.landmark_size() == 0) {
    return false;
  }
  bool is_match_one = false;
  int start_action_i = 0;
  int end_action_i = sum_target_key_code_list.size();
  if (only_no != -1) {
    start_action_i = only_no;
    end_action_i = only_no + 1;
  }
  for (int i = start_action_i; i < end_action_i; i++) {
    if (only_no == -1) {
      // 先頭がダミーだとスキップする
      if (sum_target_key_code_list[i][0] == ACTION_CODE_DUMMY) {
        continue;
      }
    }
    float target_vector_length = getVectorValueFromCode(sum_target_key_points[i], sum_visibility_threshold_list[i]);
    sum_vector_list[i] += target_vector_length;
    
    std::pair<float, int> threshold_condition = sum_threshold_condition_list[i];
    
    bool is_match = isConditionValue(sum_vector_list[i], threshold_condition.first, threshold_condition.second);
    LOG(INFO) << "sum_v1() no(" << i << ") match(" << is_match << ") sum_vector_list[" << i << "](" << sum_vector_list[i] << ") threshold(" << threshold_condition.first << ") condition(" << threshold_condition.second << ")";
    if (only_no != -1) {
      // 指定された場合、ここで終了する
      if (is_match == true) {
        return true;
      } else {
        return false;
      }
    }
    if (is_match == true) {
      is_match_one = true;
      sum_vector_list[i] -= threshold_condition.first;
    }
    std::vector<int> target_key_codes = sum_target_key_code_list[i];
    int flag = sum_target_flag_list[i];
    int last_key_code = get_latest_key_code(target_key_codes[target_key_codes.size() - 1]);
    bool is_last_run_no_up = false;
    if (flag & FD_V1_FLAG_NO_UP_LAST_KEY) {
      if (last_key_code == target_key_codes[target_key_codes.size() - 1]) {
        // already run. do nothing.
        continue;
      } else {
        // last run no up, other normal run
        is_last_run_no_up = true;
      }
    }
    for (int j = 0; j < target_key_codes.size(); j++) {
      int target_key_code = target_key_codes[j];
      int jj = 0;
      if (is_match == true) {
        //doMultiKeyEvent(target_key_code, key_default_flag);
        //jj = doMultiEvent(target_key_codes, j, 0);
        jj = push_or_pushing_latest_running_key_code(target_key_codes, j);
        LOG(INFO) << "sum_v1() no(" << i << ", " << j << ") event(" << target_key_code << ") start";
        // 累積は一度だけ実行
        // TODO キーを押す時間が短い？
        if (!(flag & FD_V1_FLAG_NO_UP_AFTER_DOWN)) {
          //if (target_key_codes[j] & KEYBD_FLAG) {
          //  //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
          //  doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
          //  clear_latest_running_key_code(target_key_codes[j]);
          //  LOG(INFO) << "sum_v1() no(" << i << ", " << j << ") event(" << target_key_code << ") end";
          //} else if (target_key_codes[j] & MOUSE_FLAG) {
          //  // do nothing
          //}
          
          if (is_last_run_no_up == false) {
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "sum_v1() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
            }
          } else {
            if (j != target_key_codes.size() - 1) {
              // not last is up
              if (target_key_codes[j] & KEYBD_FLAG) {
                //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
                doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
                clear_latest_running_key_code(target_key_codes[j]);
                LOG(INFO) << "sum_v1() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
              } else if (target_key_codes[j] & MOUSE_FLAG) {
                // do nothing
              }
            } else {
              // last is no up, update key
              update_latest_running_key_code(target_key_codes[j]);
            }
          }
        } else {
          if (target_key_codes[j] & KEYBD_FLAG) {
            update_latest_running_key_code(target_key_codes[j]);
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
          }
        }
        if (flag & FD_V1_FLAG_LOOP) {
          LOG(INFO) << "sum_v1() push key(" << target_key_code << ", " << flag << ") loop queue.";
          queue_key_code.push_back(std::pair<int, int>(target_key_code, flag));
        }
      } else {
        // 累積だとマッチ以外は何もしない
        //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
        //LOG(INFO) << "sum_v1() no(" << i << ") keybd_event(" << target_key_code << ") end";
      }
      j += jj;
    }
  }
  return is_match_one;
}


// base diff v1
// TODO implement
void setBaseData() {
  base_diff_target_vec_2d = last_target_vec_2d;
  base_diff_target_vec = last_target_vec;
  base_diff_target_vec_angle = last_target_vec_angle;
  base_diff_target_camera_diff_angle = last_target_camera_diff_angle;
}

// TODO implement
void runBaseDiffAction_v1(std::vector<int>& target_key_code_list, std::vector<std::vector<int> >& target_key_points) {
  for (int i = 0; i < target_key_code_list.size(); i++) {
    bool is_match = true;
    int points_count = target_key_points[i].size();
    for (int j = 0; j < points_count; j += KEY_POINT_V3_MAX) {
      std::vector<int> sub_target_key_points(target_key_points[i].begin() + j, target_key_points[i].begin() + j + KEY_POINT_V3_MAX);
      float target_value = getCoordinateValueFromCode(sub_target_key_points);
      std::vector<int> sub_base_key_points(sub_target_key_points);
      sub_base_key_points[KEY_POINT_COORDINATE_MODE_NO] += COORDINATE_MODE_BASE_2D - COORDINATE_MODE_LAST_2D;
      float base_value = getCoordinateValueFromCode(sub_base_key_points);
      float diff_value = base_value - target_value;
      float threshold_value = (float)target_key_points[i][KEY_POINT_THRESHOLD_VALUE_NO + j] / KEY_POINT_THRESHOLD_VALUE_DIVID;
      int condition_no = target_key_points[i][KEY_POINT_CONDITION_NO + j];
      
      is_match = isConditionValue(diff_value, threshold_value, condition_no);
      LOG(INFO) << "base_diff_v1() no(" << i << ", " << j << ") match(" << is_match << ") target_value(" << target_value << ") threshold(" << threshold_value << ") condition no(" << condition_no << ")";
      if (is_match == true) {
        // ok, check next condition
      } else {
        // ng, stop check condition
        break;
      }
    }
    int target_key_code = target_key_code_list[i];
    // from coord v2(not v3)
    if (is_match == true) {
      doMultiKeyEvent(target_key_code, key_default_flag);
      LOG(INFO) << "base_diff_v2() no(" << i << ") keybd_event(" << target_key_code << ") start";
    } else {
      // TODO 必要？ほかのキーが押されると不要になる
      doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
      LOG(INFO) << "base_diff_v2() no(" << i << ") keybd_event(" << target_key_code << ") end";
    }
  }
}


// frame diff v1
template <class T> T calcVecRate(T& landmark) {
  float x = std::abs(landmark.x());
  float y = std::abs(landmark.y());
  float z = std::abs(landmark.z());
  float xyz = x + y + z;
  T result;
  result.set_x(x / xyz);
  result.set_y(y / xyz);
  result.set_z(z / xyz);
  return result;
}

template <class T> T calcVecRateByLen(T& landmark, float length) {
  float x = std::abs(landmark.x());
  float y = std::abs(landmark.y());
  float z = std::abs(landmark.z());
  T result;
  result.set_x(x / length);
  result.set_y(y / length);
  result.set_z(z / length);
  return result;
}

bool isCheckVector(mediapipe::Landmark& sum_vec, std::vector<std::pair<float, int> >& check_vector) {
  float x = sum_vec.x();
  float y = sum_vec.y();
  float z = sum_vec.z();
  bool is_match_x = isConditionValue(x, check_vector[FD_V1_CHECK_VECTOR_X].first, check_vector[FD_V1_CHECK_VECTOR_X].second);
  bool is_match_y = isConditionValue(y, check_vector[FD_V1_CHECK_VECTOR_Y].first, check_vector[FD_V1_CHECK_VECTOR_Y].second);
  bool is_match_z = isConditionValue(z, check_vector[FD_V1_CHECK_VECTOR_Z].first, check_vector[FD_V1_CHECK_VECTOR_Z].second);
  if (is_match_x == false || is_match_y == false || is_match_z == false) {
    return false;
  }
  return true;
}

uint64_t runActionCalcDiff_v1(int only_no, uint64_t arg_input_last_run_timestamp) {
  if (frame_diff_landmarks.size() < 2) {
    LOG(INFO) << "runActionCalcDiff_v1() frame diff landmark.size() < 2, skip.";
    return false;
  }
  // last frameとbuffer内のframeのdiffを計算し、最大が閾値を超えたらアクション実行
  // 以前実行済みだったら、それ以降のframeから計算する
  //   mixの場合、タイムスタンプを考慮しきれない。引数で、実施したtimestampを受け渡しておく
  
  mediapipe::LandmarkList& last_landmarks = frame_diff_landmarks[frame_diff_landmarks.size() - 1];
  uint64_t last_timestamp = frame_diff_timestamps[frame_diff_timestamps.size() - 1];
  //bool is_match_one = false;
  uint64_t last_run_timestamp = 0;
  int start_action_i = 0;
  int end_action_i = frame_diff_v1_action_key_code_list.size();
  if (only_no != -1) {
    start_action_i = only_no;
    end_action_i = only_no + 1;
  }
  for (int i = start_action_i; i < end_action_i; i++) {
    uint64_t last_match_timestamp = 0;
    if (only_no == -1) {
      // 先頭がダミーだとスキップする
      if (frame_diff_v1_action_key_code_list[i][0] == ACTION_CODE_DUMMY) {
        continue;
      }
      last_match_timestamp = frame_diff_v1_last_action_timestamp_list[i];
    } else {
      last_match_timestamp = arg_input_last_run_timestamp;
    }
    std::vector<int> target_key_codes = frame_diff_v1_action_key_code_list[i];
    int flag = frame_diff_v1_flags[i];
    std::pair<float, int> threshold_condition = frame_diff_v1_threshold_condition[i];
    float max_diff = -1.0;
    bool is_match = false;
    //for (int j = 0; j < frame_diff_landmarks.size() - 1; j--) { // modified to back from front
    for (int j = frame_diff_landmarks.size() - 2; j >= 0; j--) { // back
      float sum_vec_len = 0.0;
      mediapipe::Landmark sum_vec;
      mediapipe::LandmarkList& buffer_landmarks = frame_diff_landmarks[j];
      uint64_t buffer_timestamp = frame_diff_timestamps[j];
      if (last_match_timestamp >= buffer_timestamp) {
        LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") last_match_timestamp(" << last_match_timestamp << ") >= target_timestamp(" << buffer_timestamp << "), skip this.";
        continue;
      } else {
        LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") last_match_timestamp(" << last_match_timestamp << ") < target_timestamp(" << buffer_timestamp << "), calc this.";
      }
      for (int k = 0; k < frame_diff_v1_key_points[i].size(); k++) {
        mediapipe::Landmark last_landmark = last_landmarks.landmark(frame_diff_v1_key_points[i][k]);
        mediapipe::Landmark frame_diff_landmark = frame_diff_landmarks[j].landmark(frame_diff_v1_key_points[i][k]);
        mediapipe::Landmark vec_2point_landmark = calc_vector(last_landmark, frame_diff_landmark);
        LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ", " << k << ") 2 point diff vec: x(" << vec_2point_landmark.x() << "), y(" << vec_2point_landmark.y() << "), z(" << vec_2point_landmark.z() << ")";
        float vec_len = calc_vector_length(vec_2point_landmark);
        sum_vec_len += vec_len;
        LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ", " << k << ") sum_vec_len(" << sum_vec_len << "), vec_len(" << vec_len << ")";
        
        add_vector(vec_2point_landmark, sum_vec);
      }
      LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") sum_vec: x(" << sum_vec.x() << "), y(" << sum_vec.y() << "), z(" << sum_vec.z() << "), sum_vec_len(" << sum_vec_len << ")";
      if (is_x_front_reverse == true) {
        sum_vec.set_x(sum_vec.x() * -1);
        LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") reverse sum_vec: x(" << sum_vec.x() << "), y(" << sum_vec.y() << "), z(" << sum_vec.z() << "), sum_vec_len(" << sum_vec_len << ")";
      }
      // sum_vecでXYZ方向をチェック
      bool is_enable_vec = isCheckVector(sum_vec, frame_diff_v1_check_vector[i]);
      if (is_enable_vec == false) {
        //LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") vector is not enable, skip this.";
        continue;
      }
      LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") vector x,y,z length check ok";
      //mediapipe::Landmark sum_vec_rate = calcVecRate(sum_vec);
      mediapipe::Landmark sum_vec_rate = calcVecRateByLen(sum_vec, sum_vec_len);
      LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") sum_vec_rate: x(" << sum_vec_rate.x() << "), y(" << sum_vec_rate.y() << "), z(" << sum_vec_rate.z() << ")";
      // sum_vec_rateで各XYZベクトルの割合をチェック
      bool is_enable_vec_rate = isCheckVector(sum_vec_rate, frame_diff_v1_check_vector_rate[i]);
      if (is_enable_vec_rate == false) {
        //LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") vector rate is not enable, skip this.";
        continue;
      }
      if (max_diff < sum_vec_len) {
        max_diff = sum_vec_len;
      }
      LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") vector x,y,z rate check ok";
      is_match = isConditionValue(max_diff, threshold_condition.first, threshold_condition.second);
      if (only_no != -1) {
        // 指定された場合、ここで終了する
        if (is_match == true) {
          last_run_timestamp = last_timestamp;
          LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") match(" << is_match << ") target_value(" << max_diff << ") threshold(" << threshold_condition.first << ") condition no(" << threshold_condition.second << ")";
          return last_run_timestamp;
        } else {
          continue;
        }
      }
      if (is_match == true) {
        //is_match_one = true;
        last_run_timestamp = last_timestamp;
      //}
      //if (is_match == true) {
        frame_diff_v1_last_action_timestamp_list[i] = last_timestamp;
        LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") match(" << is_match << ") target_value(" << max_diff << ") threshold(" << threshold_condition.first << ") condition no(" << threshold_condition.second << ")";
        break;
      }
    }
    int last_key_code = get_latest_key_code(target_key_codes[target_key_codes.size() - 1]);
    bool is_last_run_no_up = false;
    if (flag & FD_V1_FLAG_NO_UP_LAST_KEY) {
      if (last_key_code == target_key_codes[target_key_codes.size() - 1]) {
        // already run. do nothing.
        continue;
      } else {
        // last run no up, other normal run
        is_last_run_no_up = true;
      }
    }
    if (is_match == true) {
      for (int j = 0; j < target_key_codes.size(); j++) {
        int jj = 0;
        //doMultiKeyEvent(target_key_codes[j], key_default_flag);
        //jj = doMultiEvent(target_key_codes, j, 0);
        jj = push_or_pushing_latest_running_key_code(target_key_codes, j);
        if (!(flag & FD_V1_FLAG_NO_UP_AFTER_DOWN)) {
          //if (target_key_codes[j] & KEYBD_FLAG) {
          //  //doMultiKeyEvent(target_key_codes[j], key_default_flag | KEYEVENTF_KEYUP);
          //  doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
          //  clear_latest_running_key_code(target_key_codes[j]);
          //} else if (target_key_codes[j] & MOUSE_FLAG) {
          //  // do nothing
          //}
          
          if (is_last_run_no_up == false) {
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") down after up event(" << target_key_codes[j] << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
            }
          } else {
            if (j != target_key_codes.size() - 1) {
              // not last is up
              if (target_key_codes[j] & KEYBD_FLAG) {
                //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
                doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
                clear_latest_running_key_code(target_key_codes[j]);
                LOG(INFO) << "frame_diff_v1() no(" << i << ", " << j << ") down after up event(" << target_key_codes[j] << ") end";
              } else if (target_key_codes[j] & MOUSE_FLAG) {
                // do nothing
              }
            } else {
              // last is no up, update key
              update_latest_running_key_code(target_key_codes[j]);
            }
          }
        } else {
          if (target_key_codes[j] & KEYBD_FLAG) {
            update_latest_running_key_code(target_key_codes[j]);
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
          }
        }
        j += jj;
      }
      // TODO フラグで、ほかのコマンドは実行させない設定があるとよい。その場合timestamp全セットさせる。
      //if (flag) {
      //  resetTimestamp(frame_diff_timestamps, last_timestamp);
      //  break;
      //}
      if (flag & FD_V1_FLAG_LOOP) {
        for (int j = 0; j < target_key_codes.size(); j++) {
          queue_key_code.push_back(std::pair<int, int>(target_key_codes[j], flag));
        }
      }
    } else {
      // not match
    }
  }
  //return is_match_one;
  return last_run_timestamp;
}

bool check_x_front_reverse() {
  uint64_t last_timestamp = frame_diff_timestamps[frame_diff_timestamps.size() - 1];
  
  mediapipe::LandmarkList& last_vec_landmarks = frame_diff_vec_landmarks[frame_diff_landmarks.size() - 1];
  if (last_vec_landmarks.landmark(POSE_NO_FRONT_BODY).x() < 0) {
    // reverse, check next frame
  } else {
    // not reverse
    return false;
  }
  
  // 一定期間(x_front_reverse_check_msec)、反対方向を向いたらリバースとする
  int frame_size = frame_diff_vec_landmarks.size();
  bool is_reverse = true;
  for (int i = frame_size - 1; i >= 0; i--) {
    uint64_t timestamp = frame_diff_timestamps[i];
    if (last_timestamp - timestamp > x_front_reverse_check_msec) {
      break;
    }
    if (frame_diff_vec_landmarks[i].landmark(POSE_NO_FRONT_BODY).x() < 0) {
      // reverse, check next frame
    } else {
      // not reverse
      is_reverse = false;
      break;
    }
  }
  
  return is_reverse;
}

bool check_x_front_no_reverse() {
  uint64_t last_timestamp = frame_diff_timestamps[frame_diff_timestamps.size() - 1];
  
  mediapipe::LandmarkList& last_vec_landmarks = frame_diff_vec_landmarks[frame_diff_landmarks.size() - 1];
  if (last_vec_landmarks.landmark(POSE_NO_FRONT_BODY).x() >= 0) {
    // no reverse, check next frame
  } else {
    // reverse
    return false;
  }
  
  int frame_size = frame_diff_vec_landmarks.size();
  bool is_no_reverse = true;
  for (int i = frame_size - 1; i >= 0; i--) {
    uint64_t timestamp = frame_diff_timestamps[i];
    if (last_timestamp - timestamp > x_front_reverse_check_msec) {
      break;
    }
    if (frame_diff_vec_landmarks[i].landmark(POSE_NO_FRONT_BODY).x() >= 0) {
      // no reverse, check next frame
    } else {
      // reverse
      is_no_reverse = false;
      break;
    }
  }
  
  return is_no_reverse;
}

void check_change_x_front_reverse() {
  if (frame_diff_vec_landmarks.size() < 2) {
    LOG(INFO) << "check_x_front_no_reverse() frame diff vec landmark.size() < 2, skip.";
    return;
  }
  bool is_change = false;
  if (is_x_front_reverse == true) {
    is_change = check_x_front_no_reverse();
    if (is_change == true) {
      is_x_front_reverse = false;
      LOG(INFO) << "check_change_x_front_reverse() change x front reverse: " << is_x_front_reverse;
    }
  } else {
    is_change = check_x_front_reverse();
    if (is_change == true) {
      is_x_front_reverse = true;
      LOG(INFO) << "check_change_x_front_reverse() change x front reverse: " << is_x_front_reverse;
    }
  }
}

// landmark 2 point dist compare v1
// TList: mediapipe::LandmarkList, mediapipe::NormalizedLandmarkList, T: mediapipe::Landmark, mediapipe::NormalizedLandmark
template <class TList, class T> float getLandmarkDistFromCode(TList& landamrks, int& landmarks_array_no_1st, int& landmarks_array_no_2nd) {
  T landmark_1st = landamrks.landmark(landmarks_array_no_1st);
  T landmark_2nd = landamrks.landmark(landmarks_array_no_2nd);
  return getVectorValueFrom2Landmark(landmark_1st, landmark_2nd);
}

float get2CoordinateLandmarkFromCode(std::vector<int>& landmark_code) {
  int coordinate_mode_no = landmark_code[DIST_COMPARE_COORDINATE_MODE_NO];
  int landmarks_array_no_1st = landmark_code[DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST];
  int landmarks_array_no_2nd = landmark_code[DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND];
  switch (coordinate_mode_no) {
  case COORDINATE_MODE_LAST_2D:
    if (last_target_vec_2d.landmark_size() < landmarks_array_no_1st) {
      LOG(INFO) << "last_target_vec_2d.landmark_size(" << last_target_vec_2d.landmark_size() << ") is over landmarks_array_no_1st(" << landmarks_array_no_1st << "), return -1.0";
      return -1.0;
    }
    if (last_target_vec_2d.landmark_size() < landmarks_array_no_2nd) {
      LOG(INFO) << "last_target_vec_2d.landmark_size(" << last_target_vec_2d.landmark_size() << ") is over landmarks_array_no_2nd(" << landmarks_array_no_2nd << "), return -1.0";
      return -1.0;
    }
    return getVectorValueFrom2Landmark(last_target_vec_2d.landmark(landmarks_array_no_1st), last_target_vec_2d.landmark(landmarks_array_no_2nd));
    break;
  case COORDINATE_MODE_LAST_3D:
    if (last_target_vec.landmark_size() < landmarks_array_no_1st) {
      LOG(INFO) << "last_target_vec.landmark_size(" << last_target_vec.landmark_size() << ") is over landmarks_array_no_1st(" << landmarks_array_no_1st << "), return -1.0";
      return -1.0;
    }
    if (last_target_vec.landmark_size() < landmarks_array_no_2nd) {
      LOG(INFO) << "last_target_vec.landmark_size(" << last_target_vec.landmark_size() << ") is over landmarks_array_no_2nd(" << landmarks_array_no_2nd << "), return -1.0";
      return -1.0;
    }
    return getVectorValueFrom2Landmark(last_target_vec.landmark(landmarks_array_no_1st), last_target_vec.landmark(landmarks_array_no_2nd));
    break;
  case COORDINATE_MODE_LAST_3D_VEC_ANGLE:
    if (last_target_vec_angle.landmark_size() < landmarks_array_no_1st) {
      LOG(INFO) << "last_target_vec_angle.landmark_size(" << last_target_vec_angle.landmark_size() << ") is over landmarks_array_no_1st(" << landmarks_array_no_1st << "), return -1.0";
      return -1.0;
    }
    if (last_target_vec_angle.landmark_size() < landmarks_array_no_2nd) {
      LOG(INFO) << "last_target_vec_angle.landmark_size(" << last_target_vec_angle.landmark_size() << ") is over landmarks_array_no_2nd(" << landmarks_array_no_2nd << "), return -1.0";
      return -1.0;
    }
    return getVectorValueFrom2Landmark(last_target_vec_angle.landmark(landmarks_array_no_1st), last_target_vec_angle.landmark(landmarks_array_no_2nd));
    break;
  case COORDINATE_MODE_LAST_CAMERA_DIFF_ANGLE:
    if (last_target_camera_diff_angle.landmark_size() < landmarks_array_no_1st) {
      LOG(INFO) << "last_target_camera_diff_angle.landmark_size(" << last_target_camera_diff_angle.landmark_size() << ") is over landmarks_array_no_1st(" << landmarks_array_no_1st << "), return -1.0";
      return -1.0;
    }
    if (last_target_camera_diff_angle.landmark_size() < landmarks_array_no_2nd) {
      LOG(INFO) << "last_target_camera_diff_angle.landmark_size(" << last_target_camera_diff_angle.landmark_size() << ") is over landmarks_array_no_2nd(" << landmarks_array_no_2nd << "), return -1.0";
      return -1.0;
    }
    return getVectorValueFrom2Landmark(last_target_camera_diff_angle.landmark(landmarks_array_no_1st), last_target_camera_diff_angle.landmark(landmarks_array_no_2nd));
    break;
  case COORDINATE_MODE_BASE_2D:
    if (base_diff_target_vec_2d.landmark_size() < landmarks_array_no_1st) {
      LOG(INFO) << "base_diff_target_vec_2d.landmark_size(" << base_diff_target_vec_2d.landmark_size() << ") is over landmarks_array_no_1st(" << landmarks_array_no_1st << "), return -1.0";
      return -1.0;
    }
    if (base_diff_target_vec_2d.landmark_size() < landmarks_array_no_2nd) {
      LOG(INFO) << "base_diff_target_vec_2d.landmark_size(" << base_diff_target_vec_2d.landmark_size() << ") is over landmarks_array_no_2nd(" << landmarks_array_no_2nd << "), return -1.0";
      return -1.0;
    }
    return getVectorValueFrom2Landmark(base_diff_target_vec_2d.landmark(landmarks_array_no_1st), base_diff_target_vec_2d.landmark(landmarks_array_no_2nd));
    break;
  case COORDINATE_MODE_BASE_3D:
    if (base_diff_target_vec.landmark_size() < landmarks_array_no_1st) {
      LOG(INFO) << "base_diff_target_vec.landmark_size(" << base_diff_target_vec.landmark_size() << ") is over landmarks_array_no_1st(" << landmarks_array_no_1st << "), return -1.0";
      return -1.0;
    }
    if (base_diff_target_vec.landmark_size() < landmarks_array_no_2nd) {
      LOG(INFO) << "base_diff_target_vec.landmark_size(" << base_diff_target_vec.landmark_size() << ") is over landmarks_array_no_2nd(" << landmarks_array_no_2nd << "), return -1.0";
      return -1.0;
    }
    return getVectorValueFrom2Landmark(base_diff_target_vec.landmark(landmarks_array_no_1st), base_diff_target_vec.landmark(landmarks_array_no_2nd));
    break;
  case COORDINATE_MODE_BASE_3D_VEC_ANGLE:
    if (base_diff_target_vec_angle.landmark_size() < landmarks_array_no_1st) {
      LOG(INFO) << "base_diff_target_vec_angle.landmark_size(" << base_diff_target_vec_angle.landmark_size() << ") is over landmarks_array_no_1st(" << landmarks_array_no_1st << "), return -1.0";
      return -1.0;
    }
    if (base_diff_target_vec_angle.landmark_size() < landmarks_array_no_2nd) {
      LOG(INFO) << "base_diff_target_vec_angle.landmark_size(" << base_diff_target_vec_angle.landmark_size() << ") is over landmarks_array_no_2nd(" << landmarks_array_no_2nd << "), return -1.0";
      return -1.0;
    }
    return getVectorValueFrom2Landmark(base_diff_target_vec_angle.landmark(landmarks_array_no_1st), base_diff_target_vec_angle.landmark(landmarks_array_no_2nd));
    break;
  case COORDINATE_MODE_BASE_CAMERA_DIFF_ANGLE:
    if (base_diff_target_camera_diff_angle.landmark_size() < landmarks_array_no_1st) {
      LOG(INFO) << "base_diff_target_camera_diff_angle.landmark_size(" << base_diff_target_camera_diff_angle.landmark_size() << ") is over landmarks_array_no_1st(" << landmarks_array_no_1st << "), return -1.0";
      return -1.0;
    }
    if (base_diff_target_camera_diff_angle.landmark_size() < landmarks_array_no_2nd) {
      LOG(INFO) << "base_diff_target_camera_diff_angle.landmark_size(" << base_diff_target_camera_diff_angle.landmark_size() << ") is over landmarks_array_no_2nd(" << landmarks_array_no_2nd << "), return -1.0";
      return -1.0;
    }
    return getVectorValueFrom2Landmark(base_diff_target_camera_diff_angle.landmark(landmarks_array_no_1st), base_diff_target_camera_diff_angle.landmark(landmarks_array_no_2nd));
    break;
  default:
    LOG(INFO) << "getCoordinateValueFromCode() coordinate_mode_no(" << coordinate_mode_no << ") is unknown, return -1.0";
    return -1.0;
    break;
  }
}

bool runLandmark2PointDistCompareAction_v1(std::vector<std::vector<int> >& target_key_code_list, std::vector<std::vector<int> >& target_key_points, std::vector<int>& target_flag_lists, int only_no) {
  bool is_match_one = false;
  int start_action_i = 0;
  int end_action_i = target_key_code_list.size();
  if (only_no != -1) {
    start_action_i = only_no;
    end_action_i = only_no + 1;
  }
  for (int i = start_action_i; i < end_action_i; i++) {
    if (only_no == -1) {
      // 先頭がダミーだとスキップする
      if (target_key_code_list[i][0] == ACTION_CODE_DUMMY) {
        continue;
      }
    }
    bool is_match = true;
    int points_count = target_key_points[i].size();
    for (int j = 0; j < points_count; j += DIST_COMPARE_V1_MAX) {
      std::vector<int> sub_target_key_points(target_key_points[i].begin() + j, target_key_points[i].begin() + j + DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND + 1);
      float target_value = get2CoordinateLandmarkFromCode(sub_target_key_points);
      float threshold_value = (float)target_key_points[i][DIST_COMPARE_THRESHOLD_VALUE_NO + j] / DIST_COMPARE_DIVID_RATE;
      int condition_no = target_key_points[i][DIST_COMPARE_CONDITION_NO + j];
      
      is_match = isConditionValue(target_value, threshold_value, condition_no);
      LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") match(" << is_match << ") target_value(" << target_value << ") threshold(" << threshold_value << ") condition no(" << condition_no << ")";
      if (only_no != -1) {
        // 指定された場合、ここで終了する
        if (is_match == true) {
          // ok, check next condition
        } else {
          // ng, stop check condition
          return false;
        }
      } else {
        if (is_match == true) {
          // ok, check next condition
        } else {
          // ng, stop check condition
          break;
        }
      }
    }
    if (is_match == true) {
      is_match_one = true;
    }
    std::vector<int> target_key_codes = target_key_code_list[i];
    int flag = target_flag_lists[i];
    int last_key_code = get_latest_key_code(target_key_codes[target_key_codes.size() - 1]);
    bool is_last_run_no_up = false;
    if (flag & FD_V1_FLAG_NO_UP_LAST_KEY) {
      if (last_key_code == target_key_codes[target_key_codes.size() - 1]) {
        // already run. do nothing.
        continue;
      } else {
        // last run no up, other normal run
        is_last_run_no_up = true;
      }
    }
    if (is_match == true) {
      for (int j = 0; j < target_key_codes.size(); j++) {
        int target_key_code = target_key_codes[j];
        //doMultiKeyEvent(target_key_code, key_default_flag);
        //int jj = doMultiEvent(target_key_codes, j, 0);
        int jj = push_or_pushing_latest_running_key_code(target_key_codes, j);
        LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") event(" << target_key_code << ") start";
        if (!(flag & FD_V1_FLAG_NO_UP_AFTER_DOWN)) {
          //if (target_key_codes[j] & KEYBD_FLAG) {
          //  //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
          //  doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
          //  clear_latest_running_key_code(target_key_codes[j]);
          //  LOG(INFO) << "coord_v3() no(" << i << ", " << j << ") event(" << target_key_code << ") end";
          //} else if (target_key_codes[j] & MOUSE_FLAG) {
          //  // do nothing
          //}
          
          if (is_last_run_no_up == false) {
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
            }
          } else {
            if (j != target_key_codes.size() - 1) {
              // not last is up
              if (target_key_codes[j] & KEYBD_FLAG) {
                //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
                doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
                clear_latest_running_key_code(target_key_codes[j]);
                LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
              } else if (target_key_codes[j] & MOUSE_FLAG) {
                // do nothing
              }
            } else {
              // last is no up, update key
              update_latest_running_key_code(target_key_codes[j]);
            }
          }
        } else {
          if (target_key_codes[j] & KEYBD_FLAG) {
            update_latest_running_key_code(target_key_codes[j]);
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
          }
        }
        j += jj;
      }
      coord_v3_last_run_key.clear();
      coord_v3_last_run_key.push_back(i);
      if (flag & FD_V1_FLAG_LOOP) {
        for (int j = 0; j < target_key_codes.size(); j++) {
          int target_key_code = target_key_codes[j];
          queue_key_code.push_back(std::pair<int, int>(target_key_code, flag));
        }
      }
    } else {
      if (!(flag & FD_V1_FLAG_NO_UP_NOT_MATCH)) {
        if (coord_v3_last_run_key.size() >= 1 && coord_v3_last_run_key[0] == i) {
          for (int j = 0; j < target_key_codes.size(); j++) {
            int target_key_code = target_key_codes[j];
            int jj = 0;
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              jj = doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
              jj += MOUSE_INDEX_INCREMENT;
            }
            j += jj;
          }
          if (coord_v3_last_run_key.empty() == false) {
            coord_v3_last_run_key.clear();
          }
        } else {
          // not last run, therefore not run keyup
        }
      }
    }
  }
  return is_match_one;
}


// mix v1
bool runActionCalcMix_v1(std::map<uint64_t, uint64_t>& last_run_timestamp, int only_no) {
  std::vector<std::vector<int> > target_key_code_list =  mix_v1_action_key_code_list;
  
  bool is_match_one = false;
  int start_action_i = 0;
  int end_action_i = target_key_code_list.size();
  if (only_no != -1) {
    start_action_i = only_no;
    end_action_i = only_no + 1;
  }
  for (int i = start_action_i; i < end_action_i; i++) {
    if (only_no == -1) {
      // 先頭がダミーだとスキップする
      if (target_key_code_list[i][0] == ACTION_CODE_DUMMY) {
        continue;
      }
    }
    
    bool is_not_match = false;
    std::vector<int> condition_list = mix_v1_action_condition_list[i];
    for (int j = 0; j < condition_list.size(); j++) {
      int condition_flag = condition_list[j];
      int condition_mode = condition_flag & CONDITION_FLAG_MODE;
      int condition_no = condition_flag & CONDITION_FLAG_NO;
      LOG(INFO) << "mix v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") start";
      //if (condition_mode == CONDITION_MODE_COORDINATE_V1) {
      //  bool is_match = runCalcPoseCoordinateAndAction_v1(last_target_vec_2d.landmark(POSE_NO_CENTER_BODY));
      //  if (is_match == true) {
      //    LOG(INFO) << "mix v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
      //  } else {
      //    is_not_match = true;
      //    break;
      //  }
      //}
      if (condition_mode == CONDITION_MODE_COORDINATE_V2) {
        bool is_match = runCalcPoseCoordinateAndAction_v2(action_key_code_list, target_key_points, threshold_condition_list, action_flag_list, condition_no);
        if (is_match == true) {
          LOG(INFO) << "mix v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_COORDINATE_V3) {
        bool is_match = runCalcPoseCoordinateAndAction_v3(coord_v3_action_key_code_list, coord_v3_target_key_points, coord_v3_action_flag_list, condition_no);
        if (is_match == true) {
          LOG(INFO) << "mix v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_SUM_VECTOR_V1) {
        bool is_match = runSumAndCalcPoseVectorAndAction_v1(sum_action_key_code_list, sum_target_key_points, sum_threshold_condition_list, sum_vector_list, sum_action_flag_list, condition_no);
        if (is_match == true) {
          LOG(INFO) << "mix v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_CAMERA_DIFF_V1) {
        // not imple
        bool is_match = true;
        if (is_match == true) {
          LOG(INFO) << "mix v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_FRAME_DIFF_V1) {
        uint64_t last_run_timestamp_map_key = ((uint64_t)i << 48) | ((uint64_t)j << 32) | condition_list[j];
        uint64_t last_run_timestamp_map_value = last_run_timestamp[last_run_timestamp_map_key];
        //bool is_match = runActionCalcDiff_v1(condition_no);
        //if (is_match == true) {
        LOG(INFO) << "mix v1 no(" << i << ", " << j << ") frame_diff_v1 use timestamp(" << last_run_timestamp_map_value << ").";
        uint64_t run_timestamp = runActionCalcDiff_v1(condition_no, last_run_timestamp_map_value);
        //LOG(INFO) << "mix v1 no(" << i << ", " << j << ") last_run_timestamp_map_key(" << last_run_timestamp_map_key << "),  run_timestamp(" << run_timestamp << "), last_run_timestamp_map_value(" << last_run_timestamp_map_value << ")";
        if (run_timestamp != 0) {
          last_run_timestamp[last_run_timestamp_map_key] = run_timestamp;
          LOG(INFO) << "mix v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_DIST_COMPARE_V1) {
        bool is_match = runLandmark2PointDistCompareAction_v1(landmark_2point_dist_compare_v1_action_key_code_list, landmark_2point_dist_compare_v1_key_points, landmark_2point_dist_compare_v1_flag, condition_no);
        if (is_match == true) {
          LOG(INFO) << "mix v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_MARK_COMPARE_V1) {
        bool is_match = runActionCalcMarkCompare_v1(mark_compare_v1_action_key_code_list, mark_compare_v1_key_points, mark_compare_v1_action_flag, condition_no);
        if (is_match == true) {
          LOG(INFO) << "mix v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_VECTOR_DIST_COMPARE_V1) {
        bool is_match = runVectorDistCompareAction_v1(vector_dists_compare_v1_action_key_code_list, vector_dists_compare_v1_not_action_key_code_list, vector_dists_compare_v1_key_points, vector_dists_compare_v1_neutral_key_group_ids, condition_no);
        if (is_match == true) {
          LOG(INFO) << "mix v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
        } else {
          is_not_match = true;
          break;
        }
      }
    }
    
    bool is_match = false;
    if (is_not_match == true) {
      is_match = false;
    } else {
      is_match = true;
    }
    
    LOG(INFO) << "mix v1 no(" << i << ") match(" << is_match << ").";
    
    if (only_no != -1) {
      // 指定された場合、ここで終了する
      return is_match;
    }
    
    if (is_match == true) {
      is_match_one = true;
    }
    
    std::vector<int> target_key_codes = mix_v1_action_key_code_list[i];
    int flag = mix_v1_action_flag[i];
    int last_key_code = get_latest_key_code(target_key_codes[target_key_codes.size() - 1]);
    bool is_last_run_no_up = false;
    if (flag & FD_V1_FLAG_NO_UP_LAST_KEY) {
      if (last_key_code == target_key_codes[target_key_codes.size() - 1]) {
        // already run. do nothing.
        continue;
      } else {
        // last run no up, other normal run
        is_last_run_no_up = true;
      }
    }
    if (is_not_match == false) {
      // match
      //LOG(INFO) << "mix v1 no(" << i << ") match event start";
      for (int j = 0; j < target_key_codes.size(); j++) {
        int target_key_code = target_key_codes[j];
        //doMultiKeyEvent(target_key_code, key_default_flag);
        //int jj = doMultiEvent(target_key_codes, j, 0);
        int jj = push_or_pushing_latest_running_key_code(target_key_codes, j);
        if (!(flag & FD_V1_FLAG_NO_UP_AFTER_DOWN)) {
          //if (target_key_codes[j] & KEYBD_FLAG) {
          //  //doMultiKeyEvent(target_key_codes[j], key_default_flag | KEYEVENTF_KEYUP);
          //  doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
          //  clear_latest_running_key_code(target_key_codes[j]);
          //} else if (target_key_codes[j] & MOUSE_FLAG) {
          //  // do nothing
          //}
          
          if (is_last_run_no_up == false) {
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "mix v1 no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
            }
          } else {
            if (j != target_key_codes.size() - 1) {
              // not last is up
              if (target_key_codes[j] & KEYBD_FLAG) {
                //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
                doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
                clear_latest_running_key_code(target_key_codes[j]);
                LOG(INFO) << "mix v1 no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
              } else if (target_key_codes[j] & MOUSE_FLAG) {
                // do nothing
              }
            } else {
              // last is no up, update key
              update_latest_running_key_code(target_key_codes[j]);
            }
          }
        } else {
          if (target_key_codes[j] & KEYBD_FLAG) {
            update_latest_running_key_code(target_key_codes[j]);
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
          }
        }
        j += jj;
      }
      if (flag & FD_V1_FLAG_LOOP) {
        for (int j = 0; j < target_key_codes.size(); j++) {
          queue_key_code.push_back(std::pair<int, int>(target_key_codes[j], flag));
        }
      }
      //LOG(INFO) << "mix v1 no(" << i << ") match event end";
    } else {
      // not match
      if (!(flag & FD_V1_FLAG_NO_UP_NOT_MATCH)) {
        for (int j = 0; j < target_key_codes.size(); j++) {
          int target_key_code = target_key_codes[j];
          int jj = 0;
          if (target_key_codes[j] & KEYBD_FLAG) {
            //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
            jj = doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
            clear_latest_running_key_code(target_key_codes[j]);
            LOG(INFO) << "mix v1 no(" << i << ", " << j << ") event(" << target_key_code << ") end";
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
            jj += MOUSE_INDEX_INCREMENT;
          }
          j += jj;
        }
      }
    }
  }
  return is_match_one;
}

int verify_check_mix_v1() {
  bool is_invalid = false;
  
  int action_size = mix_v1_action_condition_list.size();
  for (int i = 0; i < action_size; i++) {
    std::vector<int> condition_list = mix_v1_action_condition_list[i];
    for (int j = 0; j < condition_list.size(); j++) {
      int condition_flag = condition_list[j];
      std::string condition_flag_str = getDumpHexFromInt(condition_flag);
      int condition_mode = condition_flag & CONDITION_FLAG_MODE;
      int condition_no = condition_flag & CONDITION_FLAG_NO;
      switch (condition_mode) {
      //case CONDITION_MODE_COORDINATE_V1:
      //  if (.size() <= condition_no) {
      //    is_invalid = true;
      //    LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(coordinate_v1) size(" << .size() << "), please check.";
      //  }
      //  break;
      case CONDITION_MODE_COORDINATE_V2:
        if (action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(coordinate_v2) size(" << action_key_code_list.size() << "), please check.";
        }
        break;
      case CONDITION_MODE_COORDINATE_V3:
        if (coord_v3_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(coordinate_v3) size(" << coord_v3_action_key_code_list.size() << "), please check.";
        }
        break;
      case CONDITION_MODE_SUM_VECTOR_V1:
        if (sum_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(sum_v1) size(" << sum_action_key_code_list.size() << "), please check.";
        }
        break;
      // not imple
      //case CONDITION_MODE_POSE_DIFF_V1:
      //  if (base_diff_v1_action_key_code_list.size() <= condition_no) {
      //    is_invalid = true;
      //    LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(pose_diff_v1) size(" << base_diff_v1_action_key_code_list.size() << "), please check.";
      //  }
      //  break;
      // not imple
      //case CONDITION_MODE_CAMERA_DIFF_V1:
      //  if (base_diff_v1_action_key_code_list.size() <= condition_no) {
      //    is_invalid = true;
      //    LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(camera_diff_v1) size(" << base_diff_v1_action_key_code_list.size() << "), please check.";
      //  }
      //  break;
      case CONDITION_MODE_FRAME_DIFF_V1:
        if (frame_diff_v1_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(frame_diff_v1) size(" << frame_diff_v1_action_key_code_list.size() << "), please check.";
        }
        break;
      case CONDITION_MODE_DIST_COMPARE_V1:
        if (landmark_2point_dist_compare_v1_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(dist_compare_v1) size(" << landmark_2point_dist_compare_v1_action_key_code_list.size() << "), please check.";
        }
        break;
      // own
      case CONDITION_MODE_MIX_V1:
      //  if (mix_v1_action_key_code_list.size() <= condition_no) {
      //    is_invalid = true;
      //    LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(mix_v1) size(" << mix_v1_action_key_code_list.size() << "), please check.";
      //  }
        is_invalid = true;
        LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is own(mix_v1), please check.";
        break;
      case CONDITION_MODE_MARK_COMPARE_V1:
        if (mark_compare_v1_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(dist_compare_v1) size(" << mark_compare_v1_action_key_code_list.size() << "), please check.";
        }
        break;
      // parent
      case CONDITION_MODE_CONTINUE_V1:
      //  if (continue_v1_action_key_code_list.size() <= condition_no) {
      //    is_invalid = true;
      //    LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(continue_v1) size(" << continue_v1_action_key_code_list.size() << "), please check.";
      //  }
        is_invalid = true;
        LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is parent(continue_v1), please check.";
        break;
      case CONDITION_MODE_VECTOR_DIST_COMPARE_V1:
        if (vector_dists_compare_v1_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(vecotr_dist_compare_v1) size(" << landmark_2point_dist_compare_v1_action_key_code_list.size() << "), please check.";
        }
        break;
      default:
        is_invalid = true;
        LOG(ERROR) << "mix v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is unknown, please check.";
        break;
      }
    }
  }
  
  if (is_invalid == true) {
    exit(-1);
  }
}


// mark comapre v1
bool runActionCalcMarkCompare_v1(std::vector<std::vector<int> >& target_key_code_list, std::vector<std::vector<int> >& target_key_points, std::vector<int>& target_flag_lists, int only_no) {
  bool is_match_one = false;
  int start_action_i = 0;
  int end_action_i = target_key_code_list.size();
  if (only_no != -1) {
    start_action_i = only_no;
    end_action_i = only_no + 1;
  }
  for (int i = start_action_i; i < end_action_i; i++) {
    if (only_no == -1) {
      // 先頭がダミーだとスキップする
      if (target_key_code_list[i][0] == ACTION_CODE_DUMMY) {
        continue;
      }
    }
    bool is_match = true;
    int points_count = target_key_points[i].size();
    for (int j = 0; j < points_count; j += MARK_COMPARE_V1_MAX) {
      std::vector<int> sub_target_key_points_a;
      sub_target_key_points_a.push_back(*(target_key_points[i].begin() + MARK_COMPARE_COORDINATE_MODE_NO + j));
      sub_target_key_points_a.push_back(*(target_key_points[i].begin() + MARK_COMPARE_LANDMARKS_ARRAY_NO_A + j));
      sub_target_key_points_a.push_back(*(target_key_points[i].begin() + MARK_COMPARE_COORDINATE_LINE_NO + j));
      float target_value_a = getCoordinateValueFromCode(sub_target_key_points_a);
      target_value_a += (float)(*(target_key_points[i].begin() + MARK_COMPARE_OFFSET_A + j)) / MARK_COMPARE_DIVID_RATE;
      
      std::vector<int> sub_target_key_points_b;
      sub_target_key_points_b.push_back(*(target_key_points[i].begin() + MARK_COMPARE_COORDINATE_MODE_NO + j));
      sub_target_key_points_b.push_back(*(target_key_points[i].begin() + MARK_COMPARE_LANDMARKS_ARRAY_NO_B + j));
      sub_target_key_points_b.push_back(*(target_key_points[i].begin() + MARK_COMPARE_COORDINATE_LINE_NO + j));
      float target_value_b = getCoordinateValueFromCode(sub_target_key_points_b);
      target_value_a += (float)(*(target_key_points[i].begin() + MARK_COMPARE_OFFSET_B + j)) / MARK_COMPARE_DIVID_RATE;
      
      int condition_no = target_key_points[i][MARK_COMPARE_CONDITION_NO + j];
      
      is_match = isConditionValue(target_value_a, target_value_b, condition_no);
      LOG(INFO) << "mark_compare() no(" << i << ", " << j << ") match(" << is_match << ") target_value_a(" << target_value_a << ") target_value_b(" << target_value_b << ") condition no(" << condition_no << ")";
      if (only_no != -1) {
        // 指定された場合、ここで終了する
        if (is_match == true) {
          // ok, check next condition
        } else {
          // ng, stop check condition
          return false;
        }
      } else {
        if (is_match == true) {
          // ok, check next condition
        } else {
          // ng, stop check condition
          break;
        }
      }
    }
    if (is_match == true) {
      is_match_one = true;
    }
    
    std::vector<int> target_key_codes = target_key_code_list[i];
    //std::vector<int> not_target_key_codes = not_target_key_code_list[i];
    int flag = target_flag_lists[i];
    //int key_group_id = mark_compare_v1_neutral_key_group_ids[i]; // TODO args
    //int type = CONDITION_MODE_MARK_COMPARE_V1;
    
    // v0.10.7版に変更(landmark 2 point dist compare v1から流用)
    //runActionCodes(target_key_codes, flag, is_match, type, i, key_group_id);
    int last_key_code = get_latest_key_code(target_key_codes[target_key_codes.size() - 1]);
    bool is_last_run_no_up = false;
    if (flag & FD_V1_FLAG_NO_UP_LAST_KEY) {
      if (last_key_code == target_key_codes[target_key_codes.size() - 1]) {
        // already run. do nothing.
        continue;
      } else {
        // last run no up, other normal run
        is_last_run_no_up = true;
      }
    }
    if (is_match == true) {
      for (int j = 0; j < target_key_codes.size(); j++) {
        int target_key_code = target_key_codes[j];
        //doMultiKeyEvent(target_key_code, key_default_flag);
        //int jj = doMultiEvent(target_key_codes, j, 0);
        int jj = push_or_pushing_latest_running_key_code(target_key_codes, j);
        LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") event(" << target_key_code << ") start";
        if (!(flag & FD_V1_FLAG_NO_UP_AFTER_DOWN)) {
          //if (target_key_codes[j] & KEYBD_FLAG) {
          //  //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
          //  doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
          //  clear_latest_running_key_code(target_key_codes[j]);
          //  LOG(INFO) << "coord_v3() no(" << i << ", " << j << ") event(" << target_key_code << ") end";
          //} else if (target_key_codes[j] & MOUSE_FLAG) {
          //  // do nothing
          //}
          
          if (is_last_run_no_up == false) {
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
            }
          } else {
            if (j != target_key_codes.size() - 1) {
              // not last is up
              if (target_key_codes[j] & KEYBD_FLAG) {
                //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
                doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
                clear_latest_running_key_code(target_key_codes[j]);
                LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
              } else if (target_key_codes[j] & MOUSE_FLAG) {
                // do nothing
              }
            } else {
              // last is no up, update key
              update_latest_running_key_code(target_key_codes[j]);
            }
          }
        } else {
          if (target_key_codes[j] & KEYBD_FLAG) {
            update_latest_running_key_code(target_key_codes[j]);
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
          }
        }
        j += jj;
      }
      coord_v3_last_run_key.clear();
      coord_v3_last_run_key.push_back(i);
      if (flag & FD_V1_FLAG_LOOP) {
        for (int j = 0; j < target_key_codes.size(); j++) {
          int target_key_code = target_key_codes[j];
          queue_key_code.push_back(std::pair<int, int>(target_key_code, flag));
        }
      }
    } else {
      if (!(flag & FD_V1_FLAG_NO_UP_NOT_MATCH)) {
        if (coord_v3_last_run_key.size() >= 1 && coord_v3_last_run_key[0] == i) {
          for (int j = 0; j < target_key_codes.size(); j++) {
            int target_key_code = target_key_codes[j];
            int jj = 0;
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              jj = doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
              jj += MOUSE_INDEX_INCREMENT;
            }
            j += jj;
          }
          if (coord_v3_last_run_key.empty() == false) {
            coord_v3_last_run_key.clear();
          }
        } else {
          // not last run, therefore not run keyup
        }
      }
    }
  }
  return is_match_one;
}

// continue v1
void dump_timestamp_map(std::map<uint64_t, uint64_t>& timestamp_map, std::string indent) {
  std::map<uint64_t, uint64_t>::iterator itr = timestamp_map.begin();
  std::map<uint64_t, uint64_t>::iterator itr_end = timestamp_map.end();
  for (; itr != itr_end; itr++) {
    std::string key_hex = getDumpHexFromUInt64(itr->first);
    LOG(INFO) << indent + "dump_timestamp_map()     key: " << key_hex << ", value: " << itr->second;
  }
}

void dump_timestamp_map_map(std::map<uint64_t, std::map<uint64_t, uint64_t> >& timestamp_map, std::string indent) {
  std::map<uint64_t, std::map<uint64_t, uint64_t> >::iterator itr = timestamp_map.begin();
  std::map<uint64_t, std::map<uint64_t, uint64_t> >::iterator itr_end = timestamp_map.end();
  for (; itr != itr_end; itr++) {
    LOG(INFO) << indent + "dump_timestamp_map_map() key: " << itr->first;
    dump_timestamp_map(itr->second, indent + "  ");
  }
}

bool runActionCalcContinue_v1(int only_no) {
  if (frame_diff_landmarks.size() < 2) {
    LOG(INFO) << "runActionCalcContinue_v1() frame diff landmark.size() < 2, skip.";
    return false;
  }
  uint64_t last_timestamp = frame_diff_timestamps[frame_diff_timestamps.size() - 1];
  
  std::vector<std::vector<int> > target_key_code_list =  continue_v1_action_key_code_list;
  bool is_match_one = false;
  int start_action_i = 0;
  int end_action_i = target_key_code_list.size();
  if (only_no != -1) {
    start_action_i = only_no;
    end_action_i = only_no + 1;
  }
  for (int i = start_action_i; i < end_action_i; i++) {
    if (only_no == -1) {
      // 先頭がダミーだとスキップする
      if (target_key_code_list[i][0] == ACTION_CODE_DUMMY) {
        continue;
      }
    }
    
    // 時間軸を考慮して順番にマッチするかをチェック
    std::vector<int> condition_timeout_msec_list =  continue_v1_condition_timeout_msec_list[i];
    uint64_t parent_condition_key = CONDITION_MODE_CONTINUE_V1 | i;
    int start_no = continue_v1_done_no[i] + 1;
    bool is_not_match = false;
    std::vector<int> condition_list = continue_v1_action_condition_list[i];
    for (int j = start_no; j < condition_list.size(); j++) {
      int condition_flag = condition_list[j];
      int condition_mode = condition_flag & CONDITION_FLAG_MODE;
      int condition_no = condition_flag & CONDITION_FLAG_NO;
      
      uint64_t condition_key = ((uint64_t)i << 48) | ((uint64_t)j << 32) | condition_list[j];
      LOG(INFO) << "continue v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") start";
      uint64_t last_condition_timestamp = 0;
      if (j != 0) {
        // 個別のtimeoutチェック
        uint64_t before_condition_key = ((uint64_t)i << 48) | ((uint64_t)(j - 1) << 32) | condition_list[j - 1];
        last_condition_timestamp = continue_v1_id_timestamp[before_condition_key];
        uint64_t elapsed_time = last_timestamp - last_condition_timestamp;
        if (elapsed_time > condition_timeout_msec_list[j]) {
          // timeout
          continue_v1_done_no[i] = -2;
          is_not_match = true;
          LOG(INFO) << "continue v1 no(" << i << ", " << j << ") timeout(threshold: " << condition_timeout_msec_list[j] << ", value: " << elapsed_time << ")";
          break;
        } else {
          // no timeout
        }
      }
      
      //if (condition_mode == CONDITION_MODE_COORDINATE_V1) {
      //  bool is_match = runCalcPoseCoordinateAndAction_v1(last_target_vec_2d.landmark(POSE_NO_CENTER_BODY));
      //  if (is_match == true) {
      //    LOG(INFO) << "continue v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
      //    continue_v1_done_no[i] = j;
      //    continue_v1_id_timestamp[condition_key] = last_timestamp;
      //    break;
      //  } else {
      //    is_not_match = true;
      //    break;
      //  }
      //}
      if (condition_mode == CONDITION_MODE_COORDINATE_V2) {
        bool is_match = runCalcPoseCoordinateAndAction_v2(action_key_code_list, target_key_points, threshold_condition_list, action_flag_list, condition_no);
        if (is_match == true) {
          LOG(INFO) << "continue v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
          continue_v1_done_no[i] = j;
          continue_v1_id_timestamp[condition_key] = last_timestamp;
          break;
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_COORDINATE_V3) {
        bool is_match = runCalcPoseCoordinateAndAction_v3(coord_v3_action_key_code_list, coord_v3_target_key_points, coord_v3_action_flag_list, condition_no);
        if (is_match == true) {
          LOG(INFO) << "continue v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
          continue_v1_done_no[i] = j;
          continue_v1_id_timestamp[condition_key] = last_timestamp;
          break;
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_SUM_VECTOR_V1) {
        bool is_match = runSumAndCalcPoseVectorAndAction_v1(sum_action_key_code_list, sum_target_key_points, sum_threshold_condition_list, sum_vector_list, sum_action_flag_list, condition_no);
        if (is_match == true) {
          LOG(INFO) << "continue v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
          continue_v1_done_no[i] = j;
          continue_v1_id_timestamp[condition_key] = last_timestamp;
          break;
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_CAMERA_DIFF_V1) {
        // not imple
        bool is_match = true;
        if (is_match == true) {
          LOG(INFO) << "continue v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
          continue_v1_done_no[i] = j;
          continue_v1_id_timestamp[condition_key] = last_timestamp;
          break;
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_FRAME_DIFF_V1) {
        // 連続する場合、前にマッチしたtimestampを活用する
        uint64_t last_run_timestamp_map_key = ((uint64_t)i << 48) | ((uint64_t)j << 32) | condition_list[j];
        //uint64_t last_run_timestamp_map_value = continue_v1_last_run_timestamp[last_run_timestamp_map_key];
        ////bool is_match = runActionCalcDiff_v1(condition_no);
        ////if (is_match == true) {
        //uint64_t run_timestamp = runActionCalcDiff_v1(condition_no, last_run_timestamp_map_value);
        LOG(INFO) << "continue v1 no(" << i << ", " << j << ") use last timestamp: " << last_condition_timestamp;
        uint64_t run_timestamp = runActionCalcDiff_v1(condition_no, last_condition_timestamp);
        //LOG(INFO) << "mix v1 no(" << i << ", " << j << ") last_run_timestamp_map_key(" << last_run_timestamp_map_key << "),  run_timestamp(" << run_timestamp << "), last_run_timestamp_map_value(" << last_run_timestamp_map_value << ")";
        if (run_timestamp != 0) {
          continue_v1_last_run_timestamp[last_run_timestamp_map_key] = run_timestamp;
          LOG(INFO) << "continue v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
          continue_v1_done_no[i] = j;
          continue_v1_id_timestamp[condition_key] = last_timestamp;
          break;
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_DIST_COMPARE_V1) {
        bool is_match = runLandmark2PointDistCompareAction_v1(landmark_2point_dist_compare_v1_action_key_code_list, landmark_2point_dist_compare_v1_key_points, landmark_2point_dist_compare_v1_flag, condition_no);
        if (is_match == true) {
          LOG(INFO) << "continue v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
          continue_v1_done_no[i] = j;
          continue_v1_id_timestamp[condition_key] = last_timestamp;
          break;
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_MIX_V1) {
        // frame diffのtimestampを考慮して実行する
        // timestamp_mapに、最後の順番の実行timestampを、frame_diffの番号の中にセットする
        std::map<uint64_t, uint64_t> mix_v1_last_run_timestamp_map_value = set_continue_to_mix_timestamp(condition_no, last_condition_timestamp);
        dump_timestamp_map(mix_v1_last_run_timestamp_map_value); // test
        bool is_match = runActionCalcMix_v1(mix_v1_last_run_timestamp_map_value, condition_no);
        if (is_match == true) {
          LOG(INFO) << "continue v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
          continue_v1_done_no[i] = j;
          continue_v1_id_timestamp[condition_key] = last_timestamp;
          break;
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_MARK_COMPARE_V1) {
        bool is_match = runActionCalcMarkCompare_v1(mark_compare_v1_action_key_code_list, mark_compare_v1_key_points, mark_compare_v1_action_flag, condition_no);
        if (is_match == true) {
          LOG(INFO) << "continue v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
          continue_v1_done_no[i] = j;
          continue_v1_id_timestamp[condition_key] = last_timestamp;
          break;
        } else {
          is_not_match = true;
          break;
        }
      }
      if (condition_mode == CONDITION_MODE_VECTOR_DIST_COMPARE_V1) {
        bool is_match = runVectorDistCompareAction_v1(vector_dists_compare_v1_action_key_code_list, vector_dists_compare_v1_not_action_key_code_list, vector_dists_compare_v1_key_points, vector_dists_compare_v1_flag, condition_no);
        if (is_match == true) {
          LOG(INFO) << "continue v1 no(" << i << ", " << j << ") condition(" << condition_mode << ", " << condition_no << ") match";
          continue_v1_done_no[i] = j;
          continue_v1_id_timestamp[condition_key] = last_timestamp;
          break;
        } else {
          is_not_match = true;
          break;
        }
      }
    }
    
    bool is_match = false;
    if (is_not_match == true) {
      if (continue_v1_done_no[i] == -2) {
        // timeout
        LOG(INFO) << "continue v1 no(" << i << ") timeout not match(" << is_match << ").";
        // reset
        continue_v1_done_no[i] = -1;
        continue;
      } else if (continue_v1_done_no[i] != -1) {
        // 途中までマッチしている
        // not actionは実行しない
        LOG(INFO) << "continue v1 no(" << i << ") pending not match(" << is_match << ").";
        continue;
      } else {
        // 最初からマッチしていない
        // not actionを実行
        LOG(INFO) << "continue v1 no(" << i << ") not match(" << is_match << ").";
        is_match = false;
      }
    } else {
      if (continue_v1_done_no[i] == condition_list.size() - 1) {
        // 最後までマッチしている
        // actionを実行
        is_match = true;
        // reset
        continue_v1_done_no[i] = -1;
        LOG(INFO) << "continue v1 no(" << i << ") all match(" << is_match << ").";
      } else {
        // 途中までマッチしている
        // not actionは実行しない
        LOG(INFO) << "continue v1 no(" << i << ") pending match(" << is_match << ") then timestamp(" << last_timestamp << ").";
        continue;
      }
    }
    
    LOG(INFO) << "continue v1 no(" << i << ") match(" << is_match << ").";
    
    if (only_no != -1) {
      // 指定された場合、ここで終了する
      return is_match;
    }
    
    if (is_match == true) {
      is_match_one = true;
    }
    
    std::vector<int> target_key_codes = continue_v1_action_key_code_list[i];
    //std::vector<int> not_target_key_codes = continue_v1_not_action_key_code_list[i];
    int flag = continue_v1_action_flag[i];
    //int key_group_id = continue_v1_neutral_key_group_ids[i];
    //int type = CONDITION_MODE_CONTINUE_V1;
    
    // v0.10.7版に変更(mix_v1から流用)
    //runActionCodes(target_key_codes, flag, is_match, type, i, key_group_id);
    int last_key_code = get_latest_key_code(target_key_codes[target_key_codes.size() - 1]);
    bool is_last_run_no_up = false;
    if (flag & FD_V1_FLAG_NO_UP_LAST_KEY) {
      if (last_key_code == target_key_codes[target_key_codes.size() - 1]) {
        // already run. do nothing.
        continue;
      } else {
        // last run no up, other normal run
        is_last_run_no_up = true;
      }
    }
    if (is_not_match == false) {
      // match
      //LOG(INFO) << "mix v1 no(" << i << ") match event start";
      for (int j = 0; j < target_key_codes.size(); j++) {
        int target_key_code = target_key_codes[j];
        //doMultiKeyEvent(target_key_code, key_default_flag);
        //int jj = doMultiEvent(target_key_codes, j, 0);
        int jj = push_or_pushing_latest_running_key_code(target_key_codes, j);
        if (!(flag & FD_V1_FLAG_NO_UP_AFTER_DOWN)) {
          //if (target_key_codes[j] & KEYBD_FLAG) {
          //  //doMultiKeyEvent(target_key_codes[j], key_default_flag | KEYEVENTF_KEYUP);
          //  doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
          //  clear_latest_running_key_code(target_key_codes[j]);
          //} else if (target_key_codes[j] & MOUSE_FLAG) {
          //  // do nothing
          //}
          
          if (is_last_run_no_up == false) {
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "mix v1 no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
            }
          } else {
            if (j != target_key_codes.size() - 1) {
              // not last is up
              if (target_key_codes[j] & KEYBD_FLAG) {
                //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
                doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
                clear_latest_running_key_code(target_key_codes[j]);
                LOG(INFO) << "mix v1 no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
              } else if (target_key_codes[j] & MOUSE_FLAG) {
                // do nothing
              }
            } else {
              // last is no up, update key
              update_latest_running_key_code(target_key_codes[j]);
            }
          }
        } else {
          if (target_key_codes[j] & KEYBD_FLAG) {
            update_latest_running_key_code(target_key_codes[j]);
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
          }
        }
        j += jj;
      }
      if (flag & FD_V1_FLAG_LOOP) {
        for (int j = 0; j < target_key_codes.size(); j++) {
          queue_key_code.push_back(std::pair<int, int>(target_key_codes[j], flag));
        }
      }
      //LOG(INFO) << "mix v1 no(" << i << ") match event end";
    } else {
      // not match
      if (!(flag & FD_V1_FLAG_NO_UP_NOT_MATCH)) {
        for (int j = 0; j < target_key_codes.size(); j++) {
          int target_key_code = target_key_codes[j];
          int jj = 0;
          if (target_key_codes[j] & KEYBD_FLAG) {
            //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
            jj = doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
            clear_latest_running_key_code(target_key_codes[j]);
            LOG(INFO) << "mix v1 no(" << i << ", " << j << ") event(" << target_key_code << ") end";
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
            jj += MOUSE_INDEX_INCREMENT;
          }
          j += jj;
        }
      }
    }
  }
  return is_match_one;
}

std::map<uint64_t, uint64_t> set_continue_to_mix_timestamp(int index, uint64_t timestamp) {
  std::map<uint64_t, uint64_t> timestamp_map;
  
  std::vector<int> condition_list = mix_v1_action_condition_list[index];
  int i = index;
  for (int j = 0; j < condition_list.size(); j++) {
    int condition_flag = condition_list[j];
    int condition_mode = condition_flag & CONDITION_FLAG_MODE;
    //int condition_no = condition_flag & CONDITION_FLAG_NO;
    
    if (condition_mode == CONDITION_MODE_FRAME_DIFF_V1) {
      uint64_t timestamp_map_key = ((uint64_t)i << 48) | ((uint64_t)j << 32) | condition_list[j];
      timestamp_map[timestamp_map_key] = timestamp;
    }
  }
  
  return timestamp_map;
}

int verify_check_continue_v1() {
  bool is_invalid = false;
  
  if (continue_v1_action_condition_list.size() != continue_v1_condition_timeout_msec_list.size()) {
    is_invalid = true;
    LOG(ERROR) << "continue v1 not equal continue_v1_action_condition_list size(" << continue_v1_action_condition_list.size() << ") and continue_v1_condition_timeout_msec_list size(" << continue_v1_condition_timeout_msec_list.size() << "), please check.";
  }
  
  int action_size = continue_v1_action_condition_list.size();
  for (int i = 0; i < action_size; i++) {
    std::vector<int> condition_list = continue_v1_action_condition_list[i];
    for (int j = 0; j < condition_list.size(); j++) {
      int condition_flag = condition_list[j];
      std::string condition_flag_str = getDumpHexFromInt(condition_flag);
      int condition_mode = condition_flag & CONDITION_FLAG_MODE;
      int condition_no = condition_flag & CONDITION_FLAG_NO;
      switch (condition_mode) {
      //case CONDITION_MODE_COORDINATE_V1:
      //  if (.size() <= condition_no) {
      //    is_invalid = true;
      //    LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(coordinate_v1) size(" << .size() << "), please check.";
      //  }
      //  break;
      case CONDITION_MODE_COORDINATE_V2:
        if (action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(coordinate_v2) size(" << action_key_code_list.size() << "), please check.";
        }
        break;
      case CONDITION_MODE_COORDINATE_V3:
        if (coord_v3_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(coordinate_v3) size(" << coord_v3_action_key_code_list.size() << "), please check.";
        }
        break;
      case CONDITION_MODE_SUM_VECTOR_V1:
        if (sum_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(sum_v1) size(" << sum_action_key_code_list.size() << "), please check.";
        }
        break;
      // not imple
      //case CONDITION_MODE_POSE_DIFF_V1:
      //  if (base_diff_v1_action_key_code_list.size() <= condition_no) {
      //    is_invalid = true;
      //    LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(pose_diff_v1) size(" << base_diff_v1_action_key_code_list.size() << "), please check.";
      //  }
      //  break;
      // not imple
      //case CONDITION_MODE_CAMERA_DIFF_V1:
      //  if (base_diff_v1_action_key_code_list.size() <= condition_no) {
      //    is_invalid = true;
      //    LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(camera_diff_v1) size(" << base_diff_v1_action_key_code_list.size() << "), please check.";
      //  }
      //  break;
      case CONDITION_MODE_FRAME_DIFF_V1:
        if (frame_diff_v1_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(frame_diff_v1) size(" << frame_diff_v1_action_key_code_list.size() << "), please check.";
        }
        break;
      case CONDITION_MODE_DIST_COMPARE_V1:
        if (landmark_2point_dist_compare_v1_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(dist_compare_v1) size(" << landmark_2point_dist_compare_v1_action_key_code_list.size() << "), please check.";
        }
        break;
      case CONDITION_MODE_MIX_V1:
        if (mix_v1_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(mix_v1) size(" << mix_v1_action_key_code_list.size() << "), please check.";
        }
        break;
      case CONDITION_MODE_MARK_COMPARE_V1:
        if (mark_compare_v1_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(dist_compare_v1) size(" << mark_compare_v1_action_key_code_list.size() << "), please check.";
        }
        break;
      // own
      case CONDITION_MODE_CONTINUE_V1:
      //  if (continue_v1_action_key_code_list.size() <= condition_no) {
      //    is_invalid = true;
      //    LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(continue_v1) size(" << continue_v1_action_key_code_list.size() << "), please check.";
      //  }
        LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is own(continue_v1), please check.";
        is_invalid = true;
        break;
      case CONDITION_MODE_VECTOR_DIST_COMPARE_V1:
        if (vector_dists_compare_v1_action_key_code_list.size() <= condition_no) {
          is_invalid = true;
          LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is over target(vector_dist_compare_v1) size(" << vector_dists_compare_v1_action_key_code_list.size() << "), please check.";
        }
        break;
      default:
        is_invalid = true;
        LOG(ERROR) << "continue v1 no(" << i << ", " << j << ") condition(0x" << condition_flag_str << ") is unknown, please check.";
        break;
      }
    }
  }
  
  if (is_invalid == true) {
    exit(-1);
  }
}

// vector dist compare v1
bool runVectorDistCompareAction_v1(std::vector<std::vector<int> >& target_key_code_list, std::vector<std::vector<int> >& not_target_key_code_list, std::vector<std::vector<int> >& target_key_points, std::vector<int>& target_flag_lists, int only_no) {
  bool is_match_one = false;
  int start_action_i = 0;
  int end_action_i = target_key_code_list.size();
  if (only_no != -1) {
    start_action_i = only_no;
    end_action_i = only_no + 1;
  }
  for (int i = start_action_i; i < end_action_i; i++) {
    if (only_no == -1) {
      // 先頭がダミーだとスキップする
      if (target_key_code_list[i][0] == ACTION_CODE_DUMMY) {
        continue;
      }
    }
    bool is_match = true;
    int points_count = target_key_points[i].size();
    for (int j = 0; j < points_count; j += VECTOR_DIST_COMPARE_V1_MAX) {
      std::vector<int> sub_target_key_points_1st;
      sub_target_key_points_1st.push_back(COORDINATE_MODE_LAST_3D);
      sub_target_key_points_1st.push_back(target_key_points[i][j + VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST_START]);
      sub_target_key_points_1st.push_back(target_key_points[i][j + VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST_END]);
      float target_value_1st = get2CoordinateLandmarkFromCode(sub_target_key_points_1st);
      
      std::vector<int> sub_target_key_points_2nd;
      sub_target_key_points_2nd.push_back(COORDINATE_MODE_LAST_3D);
      sub_target_key_points_2nd.push_back(target_key_points[i][j + VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND_START]);
      sub_target_key_points_2nd.push_back(target_key_points[i][j + VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND_END]);
      float target_value_2nd = get2CoordinateLandmarkFromCode(sub_target_key_points_2nd);
      
      //LOG(INFO) << "target_value_1st(" << target_value_1st << "), target_value_2nd(" << target_value_2nd << ").";
      
      target_value_1st += (float)target_key_points[i][j + VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST_OFFSET] / VECTOR_DIST_COMPARE_DIVID_RATE;
      target_value_2nd += (float)target_key_points[i][j + VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND_OFFSET] / VECTOR_DIST_COMPARE_DIVID_RATE;
      //LOG(INFO) << "offseted target_value_1st(" << target_value_1st << "), target_value_2nd(" << target_value_2nd << ").";
      
      int condition_no = target_key_points[i][VECTOR_DIST_COMPARE_CONDITION_NO + j];
      
      is_match = isConditionValue(target_value_1st, target_value_2nd, condition_no);
      LOG(INFO) << "vector_dist_compare() no(" << i << ", " << j << ") match(" << is_match << ") target_value_1st(" << target_value_1st << ") target_value_2nd(" << target_value_2nd << ") condition no(" << condition_no << ")";
      if (only_no != -1) {
        // 指定された場合、ここで終了する
        if (is_match == true) {
          // ok, check next condition
        } else {
          // ng, stop check condition
          return false;
        }
      } else {
        if (is_match == true) {
          // ok, check next condition
        } else {
          // ng, stop check condition
          break;
        }
      }
    }
    if (is_match == true) {
      is_match_one = true;
    }
    
    std::vector<int> target_key_codes = target_key_code_list[i];
    //std::vector<int> not_target_key_codes = not_target_key_code_list[i];
    int flag = target_flag_lists[i];
    //int key_group_id = vector_dists_compare_v1_neutral_key_group_ids[i]; // TODO args
    //int type = CONDITION_MODE_VECTOR_DIST_COMPARE_V1;
    int last_key_code = get_latest_key_code(target_key_codes[target_key_codes.size() - 1]);
    bool is_last_run_no_up = false;
    
    // v0.10.7版に変更(dist comapre _v1から流用)
    //runActionCodes(target_key_codes, not_target_key_codes, flag, is_match, type, i, key_group_id);
    if (flag & FD_V1_FLAG_NO_UP_LAST_KEY) {
      if (last_key_code == target_key_codes[target_key_codes.size() - 1]) {
        // already run. do nothing.
        continue;
      } else {
        // last run no up, other normal run
        is_last_run_no_up = true;
      }
    }
    if (is_match == true) {
      for (int j = 0; j < target_key_codes.size(); j++) {
        int target_key_code = target_key_codes[j];
        //doMultiKeyEvent(target_key_code, key_default_flag);
        //int jj = doMultiEvent(target_key_codes, j, 0);
        int jj = push_or_pushing_latest_running_key_code(target_key_codes, j);
        LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") event(" << target_key_code << ") start";
        if (!(flag & FD_V1_FLAG_NO_UP_AFTER_DOWN)) {
          //if (target_key_codes[j] & KEYBD_FLAG) {
          //  //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
          //  doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
          //  clear_latest_running_key_code(target_key_codes[j]);
          //  LOG(INFO) << "coord_v3() no(" << i << ", " << j << ") event(" << target_key_code << ") end";
          //} else if (target_key_codes[j] & MOUSE_FLAG) {
          //  // do nothing
          //}
          
          if (is_last_run_no_up == false) {
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
            }
          } else {
            if (j != target_key_codes.size() - 1) {
              // not last is up
              if (target_key_codes[j] & KEYBD_FLAG) {
                //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
                doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
                clear_latest_running_key_code(target_key_codes[j]);
                LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") down after up event(" << target_key_code << ") end";
              } else if (target_key_codes[j] & MOUSE_FLAG) {
                // do nothing
              }
            } else {
              // last is no up, update key
              update_latest_running_key_code(target_key_codes[j]);
            }
          }
        } else {
          if (target_key_codes[j] & KEYBD_FLAG) {
            update_latest_running_key_code(target_key_codes[j]);
          } else if (target_key_codes[j] & MOUSE_FLAG) {
            // do nothing
          }
        }
        j += jj;
      }
      coord_v3_last_run_key.clear();
      coord_v3_last_run_key.push_back(i);
      if (flag & FD_V1_FLAG_LOOP) {
        for (int j = 0; j < target_key_codes.size(); j++) {
          int target_key_code = target_key_codes[j];
          queue_key_code.push_back(std::pair<int, int>(target_key_code, flag));
        }
      }
    } else {
      if (!(flag & FD_V1_FLAG_NO_UP_NOT_MATCH)) {
        if (coord_v3_last_run_key.size() >= 1 && coord_v3_last_run_key[0] == i) {
          for (int j = 0; j < target_key_codes.size(); j++) {
            int target_key_code = target_key_codes[j];
            int jj = 0;
            if (target_key_codes[j] & KEYBD_FLAG) {
              //doMultiKeyEvent(target_key_code, key_default_flag | KEYEVENTF_KEYUP);
              jj = doMultiEvent(target_key_codes, j, KEYEVENTF_KEYUP);
              clear_latest_running_key_code(target_key_codes[j]);
              LOG(INFO) << "dist_compare() no(" << i << ", " << j << ") event(" << target_key_code << ") end";
            } else if (target_key_codes[j] & MOUSE_FLAG) {
              // do nothing
              jj += MOUSE_INDEX_INCREMENT;
            }
            j += jj;
          }
          if (coord_v3_last_run_key.empty() == false) {
            coord_v3_last_run_key.clear();
          }
        } else {
          // not last run, therefore not run keyup
        }
      }
    }
  }
  return is_match_one;
}



// group key code
int get_group_id(int key_code) {
  if (key_code_group_id_map.find(key_code) == key_code_group_id_map.end()) {
    return -1;
  //} else if (key_code_group_id_map.begin() == key_code_group_id_map.end()) {
  //  return -1;
  } else {
    int group_id = key_code_group_id_map[key_code];
    return group_id;
  }
}

int get_latest_key_code(int key_code) {
  int group_id = key_code_group_id_map[key_code];
  return latest_running_key_code_group_map[group_id];
}

void release_latest_running_key_code(int group_id) {
  int key_code = latest_running_key_code_group_map[group_id];
  if (key_code == 0) {
    return;
  }
  
  if (key_code & KEYBD_FLAG) {
    LOG(INFO) << "release_latest_running_key_code() release key_code(" << key_code << ")";
    doMultiKeyEvent(key_code, key_default_flag | KEYEVENTF_KEYUP);
  } else if (key_code & MOUSE_FLAG) {
    // do nothing
  }
  
  clear_latest_running_key_code_by_group_id(group_id);
}

int push_key_code(std::vector<int> key_codes, int index) {
  return doMultiEvent(key_codes, index, 0);
}

int push_or_pushing_latest_running_key_code(std::vector<int> key_codes, int index) {
  int key_code = key_codes[index];
  int group_id = key_code_group_id_map[key_code];
  int latest_key_code = latest_running_key_code_group_map[group_id];
  if (key_code == latest_key_code) {
    // do nothing, already pushing
    LOG(INFO) << "push_or_pushing_latest_running_key_code() already pushing, do nothing";
    if (key_code & KEYBD_FLAG) {
      return 0;
    } else if (key_code & MOUSE_FLAG) {
      return MOUSE_INDEX_INCREMENT;
    } else {
      return 0;
    }
  } else {
    release_latest_running_key_code(group_id);
    return push_key_code(key_codes, index);
  }
}

void update_latest_running_key_code(int key_code) {
  int group_id = key_code_group_id_map[key_code];
  latest_running_key_code_group_map[group_id] = key_code;
}

void clear_latest_running_key_code(int key_code) {
  int group_id = key_code_group_id_map[key_code];
  clear_latest_running_key_code_by_group_id(group_id);
}

void clear_latest_running_key_code_by_group_id(int group_id) {
  latest_running_key_code_group_map[group_id] = 0;
}

void dump_latest_running_key_code() {
  //LOG(INFO) << "latest_running_key_code_group_map dump start.";
  std::map<int, int>::iterator itr = latest_running_key_code_group_map.begin();
  std::map<int, int>::iterator itr_end = latest_running_key_code_group_map.end();
  for (; itr != itr_end; itr++) {
    LOG(INFO) << "latest_running_key_code_group_map[" << itr->first << "]: " << itr->second;
  }
  //LOG(INFO) << "latest_running_key_code_group_map dump end.";
}


void verify_check_config_no() {
  bool is_invalid = false;
  
  std::string config_name;
  
  // coordinate v2
  config_name = "coordinate_v2";
  //   key_points
  for (int i = 0; i < target_key_points.size(); i++) {
    std::vector<int> target_key_point = target_key_points[i];
    for (int j = 0; j < target_key_point.size(); j++) {
      if (j % KEY_POINT_NO_MAX == KEY_POINT_COORDINATE_MODE_NO) {
        if (target_key_point[j] > COORDINATE_MODE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      } else if (j % KEY_POINT_NO_MAX == KEY_POINT_LANDMARKS_ARRAY_NO) {
#ifdef DETECT_MODE_POSE
        if (target_key_point[j] >= POSE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_HAND
        if (target_key_point[j] >= HAND_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_FACE_MESH
        if (target_key_point[j] >= FACE_MESH_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#endif // DETECT_MODE_FACE_MESH
      } else if (j % KEY_POINT_NO_MAX == KEY_POINT_LANDMARK_COORDINATE_NO) {
        if (target_key_point[j] > MP_LANDMARK_VALUE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      }
    }
  }
  //   threshold_condition
  for (int i = 0; i < threshold_condition_list.size(); i++) {
    std::pair<float, int> threshold_condition = threshold_condition_list[i];
    // threshold:first not check
    if (threshold_condition.second > CONDITION_NO_MAX) {
      LOG(ERROR) << config_name << " no[" << i << "] condition: " << threshold_condition.second << ", is invalid value.";
      is_invalid = true;
    }
  }
  
  // coordinate v3
  config_name = "coordinate_v3";
  //   key_points
  for (int i = 0; i < coord_v3_target_key_points.size(); i++) {
    std::vector<int> target_key_point = coord_v3_target_key_points[i];
    for (int j = 0; j < target_key_point.size(); j++) {
      if (j % KEY_POINT_V3_MAX == KEY_POINT_COORDINATE_MODE_NO) {
        if (target_key_point[j] > COORDINATE_MODE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      } else if (j % KEY_POINT_V3_MAX == KEY_POINT_LANDMARKS_ARRAY_NO) {
#ifdef DETECT_MODE_POSE
        if (target_key_point[j] >= POSE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_HAND
        if (target_key_point[j] >= HAND_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_FACE_MESH
        if (target_key_point[j] >= FACE_MESH_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#endif // DETECT_MODE_FACE_MESH
      } else if (j % KEY_POINT_V3_MAX == KEY_POINT_LANDMARK_COORDINATE_NO) {
        if (target_key_point[j] > MP_LANDMARK_VALUE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      } else if (j % KEY_POINT_V3_MAX == KEY_POINT_THRESHOLD_VALUE_NO) {
        // not check
      } else if (j % KEY_POINT_V3_MAX == KEY_POINT_CONDITION_NO) {
        if (target_key_point[j] > CONDITION_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      }
    }
  }
  
  // sum v1
  config_name = "sum_v1";
  //   key_points
  for (int i = 0; i < sum_target_key_points.size(); i++) {
    std::vector<int> target_key_point = sum_target_key_points[i];
    for (int j = 0; j < target_key_point.size(); j++) {
      if (j % KEY_POINT_SUM_V1_MAX == KEY_POINT_COORDINATE_MODE_NO) {
        if (target_key_point[j] > COORDINATE_MODE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      } else if (j % KEY_POINT_SUM_V1_MAX == KEY_POINT_LANDMARKS_ARRAY_NO) {
#ifdef DETECT_MODE_POSE
        if (target_key_point[j] >= POSE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_HAND
        if (target_key_point[j] >= HAND_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_FACE_MESH
        if (target_key_point[j] >= FACE_MESH_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#endif // DETECT_MODE_FACE_MESH
      }
    }
  }
  //   threshold_condition
  for (int i = 0; i < sum_threshold_condition_list.size(); i++) {
    std::pair<float, int> threshold_condition = sum_threshold_condition_list[i];
    // threshold:first not check
    if (threshold_condition.second > CONDITION_NO_MAX) {
      LOG(ERROR) << config_name << " no[" << i << "] condition: " << threshold_condition.second << ", is invalid value.";
      is_invalid = true;
    }
  }
  
  // frame diff v1
  config_name = "frame_diff_v1";
  //   key_points
  for (int i = 0; i < frame_diff_v1_key_points.size(); i++) {
    std::vector<int> target_key_point = frame_diff_v1_key_points[i];
    for (int j = 0; j < target_key_point.size(); j++) {
#ifdef DETECT_MODE_POSE
      if (target_key_point[j] >= POSE_NO_MAX) {
        LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
        is_invalid = true;
      }
#elif DETECT_MODE_HAND
      if (target_key_point[j] >= HAND_NO_MAX) {
        LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
        is_invalid = true;
      }
#elif DETECT_MODE_FACE_MESH
      if (target_key_point[j] >= FACE_MESH_NO_MAX) {
        LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
        is_invalid = true;
      }
#endif // DETECT_MODE_FACE_MESH
    }
  }
  //   threshold_condition
  for (int i = 0; i < frame_diff_v1_threshold_condition.size(); i++) {
    std::pair<float, int> threshold_condition = frame_diff_v1_threshold_condition[i];
    // threshold:first not check
    if (threshold_condition.second > CONDITION_NO_MAX) {
      LOG(ERROR) << config_name << " no[" << i << "] condition: " << threshold_condition.second << ", is invalid value.";
      is_invalid = true;
    }
  }
  // check_vector
  for (int i = 0; i < frame_diff_v1_check_vector.size(); i++) {
    std::vector<std::pair<float, int> > vector_list = frame_diff_v1_check_vector[i];
    if (vector_list.size() > FD_V1_CHECK_VECTOR_NO_MAX) {
      LOG(WARNING) << config_name << " no[" << i << "] check vector condition size(" << vector_list.size() << ") is invalid.";
    }
    for (int j = 0; j < vector_list.size(); j++) {
      std::pair<float, int> vector = vector_list[j];
      // threshold:first not check
      if (vector.second > CONDITION_NO_MAX) {
        LOG(ERROR) << config_name << " no[" << i << "] check vector condition: " << vector.second << ", is invalid value.";
        is_invalid = true;
      }
    }
  }
  // check_vector_rate
  for (int i = 0; i < frame_diff_v1_check_vector_rate.size(); i++) {
    std::vector<std::pair<float, int> > vector_list = frame_diff_v1_check_vector_rate[i];
    if (vector_list.size() > FD_V1_CHECK_VECTOR_NO_MAX) {
      LOG(WARNING) << config_name << " no[" << i << "] check vector rate condition size(" << vector_list.size() << ") is invalid.";
    }
    for (int j = 0; j < vector_list.size(); j++) {
      std::pair<float, int> vector = vector_list[j];
      // threshold:first not check
      if (vector.second > CONDITION_NO_MAX) {
        LOG(ERROR) << config_name << " no[" << i << "] check vector rate condition: " << vector.second << ", is invalid value.";
        is_invalid = true;
      }
    }
  }
  
  // dist compare v1
  config_name = "dist_compare_v1";
  //   key_points
  for (int i = 0; i < landmark_2point_dist_compare_v1_key_points.size(); i++) {
    std::vector<int> target_key_point = landmark_2point_dist_compare_v1_key_points[i];
    for (int j = 0; j < target_key_point.size(); j++) {
      if (j % DIST_COMPARE_V1_MAX == DIST_COMPARE_COORDINATE_MODE_NO) {
        if (target_key_point[j] > COORDINATE_MODE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      } else if (j % DIST_COMPARE_V1_MAX == DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST || j % DIST_COMPARE_V1_MAX == DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND) {
#ifdef DETECT_MODE_POSE
        if (target_key_point[j] >= POSE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_HAND
        if (target_key_point[j] >= HAND_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_FACE_MESH
        if (target_key_point[j] >= FACE_MESH_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#endif // DETECT_MODE_FACE_MESH
      } else if (j % DIST_COMPARE_V1_MAX == DIST_COMPARE_THRESHOLD_VALUE_NO) {
        // not check
      } else if (j % DIST_COMPARE_V1_MAX == DIST_COMPARE_CONDITION_NO) {
        if (target_key_point[j] > CONDITION_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      }
    }
  }
  
  // mark compare v1
  config_name = "mark_compare_v1";
  //   key_points
  for (int i = 0; i < mark_compare_v1_key_points.size(); i++) {
    std::vector<int> target_key_point = mark_compare_v1_key_points[i];
    for (int j = 0; j < target_key_point.size(); j++) {
      if (j % MARK_COMPARE_V1_MAX == DIST_COMPARE_COORDINATE_MODE_NO) {
        if (target_key_point[j] > COORDINATE_MODE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      } else if (j % MARK_COMPARE_V1_MAX == MARK_COMPARE_COORDINATE_LINE_NO) {
        if (target_key_point[j] > MP_LANDMARK_VALUE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      } else if (j % MARK_COMPARE_V1_MAX == MARK_COMPARE_LANDMARKS_ARRAY_NO_A || j % MARK_COMPARE_V1_MAX == MARK_COMPARE_LANDMARKS_ARRAY_NO_B) {
#ifdef DETECT_MODE_POSE
        if (target_key_point[j] >= POSE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_HAND
        if (target_key_point[j] >= HAND_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_FACE_MESH
        if (target_key_point[j] >= FACE_MESH_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#endif // DETECT_MODE_FACE_MESH
      } else if (j % MARK_COMPARE_V1_MAX == MARK_COMPARE_OFFSET_A || j % MARK_COMPARE_V1_MAX == MARK_COMPARE_OFFSET_B) {
        // not check
      } else if (j % MARK_COMPARE_V1_MAX == MARK_COMPARE_CONDITION_NO) {
        if (target_key_point[j] > CONDITION_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      }
    }
  }
  
  // vector dist compare v1
  config_name = "vector_dist_compare_v1";
  //   key_points
  for (int i = 0; i < vector_dists_compare_v1_key_points.size(); i++) {
    std::vector<int> target_key_point = vector_dists_compare_v1_key_points[i];
    for (int j = 0; j < target_key_point.size(); j++) {
      if (j % VECTOR_DIST_COMPARE_V1_MAX == VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST_START || 
          j % VECTOR_DIST_COMPARE_V1_MAX == VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST_END || 
          j % VECTOR_DIST_COMPARE_V1_MAX == VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND_START || 
          j % VECTOR_DIST_COMPARE_V1_MAX == VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND_END) {
#ifdef DETECT_MODE_POSE
        if (target_key_point[j] >= POSE_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_HAND
        if (target_key_point[j] >= HAND_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#elif DETECT_MODE_FACE_MESH
        if (target_key_point[j] >= FACE_MESH_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
#endif // DETECT_MODE_FACE_MESH
      } else if (j % DIST_COMPARE_V1_MAX == VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_1ST_OFFSET) {
        // not check
      } else if (j % DIST_COMPARE_V1_MAX == VECTOR_DIST_COMPARE_LANDMARKS_ARRAY_NO_2ND_OFFSET) {
        // not check
      } else if (j % DIST_COMPARE_V1_MAX == VECTOR_DIST_COMPARE_CONDITION_NO) {
        if (target_key_point[j] > CONDITION_NO_MAX) {
          LOG(ERROR) << config_name << " no[" << i << "] key_points[" << j << "]: " << target_key_point[j] << ", is invalid value.";
          is_invalid = true;
        }
      }
    }
  }
  
  // logout
  //   type
  for (int i = 0; i < logout_type_specify.size(); i++) {
    if (logout_type_specify[i] > COORDINATE_MODE_NO_MAX) {
      LOG(ERROR) << CONFIG_KEY_LOGOUT_COORDINATE_TYPES << " no[" << i << "]: " << logout_type_specify[i] << ", is invalid value.";
      is_invalid = true;
    }
  }
  //   number
  for (int i = 0; i < logout_number_specify.size(); i++) {
#ifdef DETECT_MODE_POSE
    if (logout_number_specify[i] >= POSE_NO_MAX) {
      LOG(ERROR) << CONFIG_KEY_LOGOUT_COORDINATE_NUMBERS << " no[" << i << "]: " << logout_number_specify[i] << ", is invalid value.";
      is_invalid = true;
    }
#elif DETECT_MODE_HAND
    if (logout_number_specify[i] >= HAND_NO_MAX) {
      LOG(ERROR) << CONFIG_KEY_LOGOUT_COORDINATE_NUMBERS << " no[" << i << "]: " << logout_number_specify[i] << ", is invalid value.";
      is_invalid = true;
    }
#elif DETECT_MODE_FACE_MESH
    if (logout_number_specify[i] >= FACE_MESH_NO_MAX) {
      LOG(ERROR) << CONFIG_KEY_LOGOUT_COORDINATE_NUMBERS << " no[" << i << "]: " << logout_number_specify[i] << ", is invalid value.";
      is_invalid = true;
    }
#endif // DETECT_MODE_FACE_MESH
  }
  
  if (is_invalid == true) {
    LOG(ERROR) << "verify check config each no, exist invalid data, please confirm.";
    exit(-1);
  }
}


// owner site
void show_owner_site_page(std::string path) {
  LOG(INFO) << "show owner site start: " << path;
  //system("start chrome.exe " + path);
  //ShellExecute(NULL, _T("open"), _T(path.c_str()), NULL, NULL, SW_SHOWDEFAULT); // need #include <tchar.h>
  ShellExecute(NULL, TEXT("open"), TEXT(path.c_str()), NULL, NULL, SW_SHOWDEFAULT);
  LOG(INFO) << "show owner site end: " << path;
}

bool is_open_owner_site(float rate) {
  int srand_seed = time(NULL) % 100;
  LOG(INFO) << "srand_seed: " << srand_seed;
  std::srand(srand_seed);
  
  int rand_int = std::rand() % 100;
  float rand_rate = (float)rand_int / 100.0;
  if (rate > rand_rate) {
    LOG(INFO) << "owner site show_rate(" << rate << ") > now_rate(" << rand_rate << "), owner site open.";
    return true;
  } else {
    LOG(INFO) << "owner site show_rate(" << rate << ") < now_rate(" << rand_rate << "), owner site not open.";
    return false;
  }
}

void show_owner_site_page() {
  float show_rate = owner_site_open_rate;
  bool is_owner_site = is_open_owner_site(show_rate);
  if (is_owner_site == true) {
    std::string path = OWNER_SITE_PATH;
    show_owner_site_page(path);
  }
}

// owner file check
std::string calc_md5_hash(std::string path) {
  char* file_context_ptr = loadTextFile(path);
  std::string file_context(file_context_ptr);
  free(file_context_ptr);
  int file_size = file_context.size();
  const char* file_context_cptr = file_context.c_str();
  
  HCRYPTPROV h_crypt_prov = 0;
  HCRYPTHASH h_crypt_hash = 0;
  
  CryptAcquireContext(&h_crypt_prov, NULL, NULL, PROV_RSA_FULL, CRYPT_VERIFYCONTEXT);
  CryptCreateHash(h_crypt_prov, CALG_MD5, 0, 0, &h_crypt_hash);
  CryptHashData(h_crypt_hash, (BYTE*)file_context_cptr, file_size, 0);
  char hash_c_array[HASH_SIZE + 1];
  DWORD read_size = HASH_SIZE;
  CryptGetHashParam(h_crypt_hash, HP_HASHVAL, (BYTE*)hash_c_array, &read_size, 0);
  CryptDestroyHash(h_crypt_hash);
  CryptReleaseContext(h_crypt_prov, 0);
  
  hash_c_array[HASH_SIZE] = '\0';
  char hash_hex_array[HASH_SIZE * 2 + 1];
  sprintf(hash_hex_array, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", 
    (unsigned char)hash_c_array[0], (unsigned char)hash_c_array[1], (unsigned char)hash_c_array[2], (unsigned char)hash_c_array[3], 
    (unsigned char)hash_c_array[4], (unsigned char)hash_c_array[5], (unsigned char)hash_c_array[6], (unsigned char)hash_c_array[7], 
    (unsigned char)hash_c_array[8], (unsigned char)hash_c_array[9], (unsigned char)hash_c_array[10], (unsigned char)hash_c_array[11], 
    (unsigned char)hash_c_array[12], (unsigned char)hash_c_array[13], (unsigned char)hash_c_array[14], (unsigned char)hash_c_array[15]
    );
  //LOG(INFO) << "hash_hex_array: " << hash_hex_array;
  hash_hex_array[HASH_SIZE * 2] = '\0';
  std::string hash(hash_hex_array);
  //LOG(INFO) << "hash: " << hash;
  
  return hash;
}

bool check_key_file_and_hash(std::string expect_hash, std::string path) {
  std::string key_path_hash = calc_md5_hash(path);
  //LOG(ERROR) << "key path hash: " << key_path_hash; // TODO comment out
  bool is_ok = false;
  if (key_path_hash == expect_hash) {
    is_ok = true;
  }
  return is_ok;
}

// refer
//   https://learn.microsoft.com/ja-jp/cpp/c-runtime-library/reference/signal?view=msvc-170
//   https://learn.microsoft.com/ja-jp/windows/console/ctrl-c-and-ctrl-break-signals
void signal_func(int signal_no) {
  if (signal_no == SIGABRT) {
    // 異常終了
    LOG(INFO) << "signal(" << signal_no << "): SIGABRT";
    //show_owner_site_page();
    exit(0);
  } else if (signal_no == SIGFPE) {
    // 浮動小数点エラー
    LOG(INFO) << "signal(" << signal_no << "): SIGFPE";
    //show_owner_site_page();
    exit(0);
  } else if (signal_no == SIGILL) {
    // 無効な命令
    LOG(INFO) << "signal(" << signal_no << "): SIGILL";
    //show_owner_site_page();
    exit(0);
  } else if (signal_no == SIGINT) {
    // Ctrl + C シグナル
    LOG(INFO) << "signal(" << signal_no << "): SIGINT";
    //show_owner_site_page();
    // force show owner site
    std::string path = OWNER_SITE_PATH;
    show_owner_site_page(path);
    exit(0);
  } else if (signal_no == SIGSEGV) {
    // ストレージへの無効なアクセス
    LOG(INFO) << "signal(" << signal_no << "): SIGSEGV";
    //show_owner_site_page();
    exit(0);
  } else if (signal_no == SIGTERM) {
    // 終了要求
    LOG(INFO) << "signal(" << signal_no << "): SIGTERM";
    //show_owner_site_page();
    // force show owner site
    std::string path = OWNER_SITE_PATH;
    show_owner_site_page(path);
    exit(0);
  } else if (signal_no == SIGBREAK) {
    // Ctrl + Break
    LOG(INFO) << "signal(" << signal_no << "): SIGBREAK";
    //show_owner_site_page();
    // force show owner site
    std::string path = OWNER_SITE_PATH;
    show_owner_site_page(path);
    exit(0);
  }
}

void regist_signal_func() {
  // 異常終了
  //signal(SIGABRT, signal_func);
  //ret = signal(SIGABRT, signal_func);
  //if (ret == SIG_ERR) {
  //  LOG(INFO) << "regist signal(SIGABRT) func error(" << ret << ")";
  //}
  
  // 浮動小数点エラー
  //signal(SIGFPE, signal_func);
  //ret = signal(SIGFPE, signal_func);
  //if (ret == SIG_ERR) {
  //  LOG(INFO) << "regist signal(SIGFPE) func error(" << ret << ")";
  //}
  
  // 無効な命令
  //signal(SIGILL, signal_func);
  //ret = signal(SIGILL, signal_func);
  //if (ret == SIG_ERR) {
  //  LOG(INFO) << "regist signal(SIGILL) func error(" << ret << ")";
  //}
  
  // Ctrl + C シグナル
  signal(SIGINT, signal_func);
  //ret = signal(SIGINT, signal_func);
  //if (ret == SIG_ERR) {
  //  LOG(INFO) << "regist signal(SIGINT) func error(" << ret << ")";
  //}
  
  // ストレージへの無効なアクセス
  //signal(SIGSEGV, signal_func);
  //ret = signal(SIGSEGV, signal_func);
  //if (ret == SIG_ERR) {
  //  LOG(INFO) << "regist signal(SIGSEGV) func error(" << ret << ")";
  //}
  
  // 終了要求
  signal(SIGTERM, signal_func);
  //ret = signal(SIGTERM, signal_func);
  //if (ret == SIG_ERR) {
  //  LOG(INFO) << "regist signal(SIGTERM) func error(" << ret << ")";
  //}
  
  // Ctrl + Break
  signal(SIGBREAK, signal_func);
  //ret = signal(SIGBREAK, signal_func);
  //if (ret == SIG_ERR) {
  //  LOG(INFO) << "regist signal(SIGBREAK) func error(" << ret << ")";
  //}
  
  // linux(windows build error)
  //signal(SIGQUIT, signal_func);
  //ret = signal(SIGQUIT, signal_func);
  //if (ret == SIG_ERR) {
  //  LOG(INFO) << "regist signal(SIGQUIT) func error(" << ret << ")";
  //}
  
  // linux(windows build error)
  //signal(SIGHUP, signal_func);
  //ret = signal(SIGHUP, signal_func);
  //if (ret == SIG_ERR) {
  //  LOG(INFO) << "regist signal(SIGHUP) func error(" << ret << ")";
  //}
}

void verify_check() {
  //   mix_v1
  verify_check_mix_v1();
  //   continue v1
  verify_check_continue_v1();
  //   key_points
  verify_check_config_no();
}

//------------------------------------------------------------------------------
// mix end
//------------------------------------------------------------------------------



////////////////////////////////////////////////////////////////////////////////
// modifyer add end
////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv) {
  
  
  google::InitGoogleLogging(argv[0]);
  
  
  bool is_hash_file_exist = check_exist_file(OWNER_SITE_PATH);
  if (is_hash_file_exist == false) {
    LOG(ERROR) << "ERROR: ower file(" << OWNER_SITE_PATH << ") don't exist, please confirm.";
    exit(0);
  }
  bool is_hash_ok = check_key_file_and_hash(OWNER_SITE_EXPECT_HASH, OWNER_SITE_PATH);
  if (is_hash_ok == false) {
    LOG(ERROR) << "ERROR: owner file(" << OWNER_SITE_PATH << ") is invalid, please confirm.";
    exit(0);
  }
  regist_signal_func();
  
  
  float flag_init_value = 1.0;
  bool is_left = true;
  bool is_right = false;
  
#ifdef DETECT_MODE_POSE
  logout_number_specify.push_back(POSE_NO_NOSE);
  //logout_number_specify.push_back(POSE_NO_CENTER_NOSE);
  //logout_number_specify.push_back(POSE_NO_CENTER_SHOULDER);
  logout_number_specify.push_back(POSE_NO_CENTER_BODY);
  //logout_number_specify.push_back(POSE_NO_CENTER_HIP);
  //logout_number_specify.push_back(POSE_NO_LEFT_SHOULDER);
  //logout_number_specify.push_back(POSE_NO_LEFT_BODY);
  //logout_number_specify.push_back(POSE_NO_LEFT_HIP);
  //logout_number_specify.push_back(POSE_NO_RIGHT_SHOULDER);
  //logout_number_specify.push_back(POSE_NO_RIGHT_BODY);
  //logout_number_specify.push_back(POSE_NO_RIGHT_HIP);
  //logout_number_specify.push_back(POSE_NO_FRONT_SHOULDER);
  //logout_number_specify.push_back(POSE_NO_FRONT_BODY);
  //logout_number_specify.push_back(POSE_NO_FRONT_HIP);
  //logout_number_specify.push_back(POSE_NO_LEFT_SHOULDER);
  //logout_number_specify.push_back(POSE_NO_LEFT_ELBOW);
  //logout_number_specify.push_back(POSE_NO_LEFT_WRIST);
  //logout_number_specify.push_back(POSE_NO_RIGHT_SHOULDER);
  //logout_number_specify.push_back(POSE_NO_RIGHT_ELBOW);
  //logout_number_specify.push_back(POSE_NO_RIGHT_WRIST);
  
  //logout_number_specify.push_back(POSE_NO_LEFT_HIP);
  //logout_number_specify.push_back(POSE_NO_LEFT_KNEE);
  //logout_number_specify.push_back(POSE_NO_LEFT_ANKLE);
  //logout_number_specify.push_back(POSE_NO_RIGHT_HIP);
  //logout_number_specify.push_back(POSE_NO_RIGHT_KNEE);
  //logout_number_specify.push_back(POSE_NO_RIGHT_ANKLE);
  logout_number_specify.push_back(POSE_NO_WORLD_CENTER_BODY);
#elif DETECT_MODE_HAND
  //logout_number_specify.push_back(HAND_NO_WRIST);
  //logout_number_specify.push_back(HAND_NO_THUMB_TIP);
  logout_number_specify.push_back(HAND_NO_INDEX_FINGER_TIP);
  //logout_number_specify.push_back(HAND_NO_MIDDLE_FINGER_TIP);
  //logout_number_specify.push_back(HAND_NO_RING_FINGER_TIP);
  //logout_number_specify.push_back(HAND_NO_PINKY_TIP);
  logout_number_specify.push_back(HAND_NO_CENTER);
  logout_number_specify.push_back(HAND_NO_WORLD_CENTER);
#elif DETECT_MODE_FACE_MESH
  logout_number_specify.push_back(FACE_MESH_NO_NOSE);
  //logout_number_specify.push_back(FACE_MESH_NO_TOP);
  //logout_number_specify.push_back(FACE_MESH_NO_BOTTOM);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_CHEEK);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_CHEEK);
  //logout_number_specify.push_back(FACE_MESH_NO_LIP_INNER_UPPER);
  //logout_number_specify.push_back(FACE_MESH_NO_LIP_INNER_LOWER);
  //logout_number_specify.push_back(FACE_MESH_NO_LIP_INNER_LEFT_SIDE);
  //logout_number_specify.push_back(FACE_MESH_NO_LIP_INNER_RIGHT_SIDE);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_EYE_UPPER);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_EYE_LOWER);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_EYE_IN_SIDE);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_EYE_OUT_SIDE);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_EYE_BROW_UPPER_IN_SIDE);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_EYE_BROW_UPPER_CENTER);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_EYE_BROW_UPPER_OUT_SIDE);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_EYE_UPPER);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_EYE_LOWER);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_EYE_IN_SIDE);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_EYE_OUT_SIDE);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_EYE_BROW_UPPER_IN_SIDE);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_EYE_BROW_UPPER_CENTER);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_EYE_BROW_UPPER_OUT_SIDE);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_EYE_IRIS_CENTER);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_EYE_IRIS_CENTER);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_UPPER_FACE);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_LOWER_FACE);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_UPPER_FACE);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_LOWER_FACE);
  //
  //logout_number_specify.push_back(FACE_MESH_NO_LIP_CENTER);
  //logout_number_specify.push_back(FACE_MESH_NO_EYE_CENTER);
  //logout_number_specify.push_back(FACE_MESH_NO_LEFT_EYE_CENTER);
  //logout_number_specify.push_back(FACE_MESH_NO_RIGHT_EYE_CENTER);
  logout_number_specify.push_back(FACE_MESH_NO_WORLD_CENTER);
  logout_number_specify.push_back(FACE_MESH_NO_CENTER);
  //logout_number_specify.push_back(FACE_MESH_NO_SIDE); // same right
  //logout_number_specify.push_back(FACE_MESH_NO_UP); // same top
  logout_number_specify.push_back(FACE_MESH_NO_FRONT);
#endif // DETECT_MODE_FACE_MESH
  logout_number_specify.clear();
  
  
  float common_action_rate = 1.0;
  
  int srand_seed = time(NULL) % 100;
  LOG(INFO) << "first srand_seed: " << srand_seed;
  std::srand(srand_seed);
  
  absl::ParseCommandLine(argc, argv);
  
  
  if (absl::GetFlag(FLAGS_input_config_path).empty() == false) {
    LOG(INFO) << "input config path: " << absl::GetFlag(FLAGS_input_config_path);
    config_path = absl::GetFlag(FLAGS_input_config_path);
  }
  loadConfigV1(config_path);
  dumpConfigV1();
  
  // init sum
  for (int i = 0; i < sum_action_key_code_list.size(); i++) {
    sum_vector_list.push_back((float)0.0);
  }
  // init key code group
  // config file load
  for (int i = 0; i < key_code_group_id_map.size(); i++) {
    latest_running_key_code_group_map[i] = 0;
  }
  // init timestamp
  //   frame diff v1
  if (frame_diff_v1_action_key_code_list.empty() == false) {
    frame_diff_v1_last_action_timestamp_list.resize(frame_diff_v1_action_key_code_list.size());
  }
  //   mix v1
  for (int i = 0; i < mix_v1_action_condition_list.size(); i++) {
    for (int j = 0; j < mix_v1_action_condition_list[i].size(); j++) {
      uint64_t map_key = ((uint64_t)i << 48) | ((uint64_t)j << 32) | mix_v1_action_condition_list[i][j];
      mix_v1_last_run_timestamp[map_key] = 0;
      //LOG(INFO) << "mix_v1_last_run_timestamp init ((uint64_t)i << 48): " << ((uint64_t)i << 48);
      //LOG(INFO) << "mix_v1_last_run_timestamp init ((uint64_t)j << 32): " << ((uint64_t)j << 32);
      //LOG(INFO) << "mix_v1_last_run_timestamp init key: " << map_key;
    }
  }
  //   continue v1
  for (int i = 0; i < continue_v1_action_condition_list.size(); i++) {
    for (int j = 0; j < continue_v1_action_condition_list[i].size(); j++) {
      uint64_t map_key = ((uint64_t)i << 48) | ((uint64_t)j << 32) | continue_v1_action_condition_list[i][j];
      continue_v1_last_run_timestamp[map_key] = 0;
      //LOG(INFO) << "continue_v1_last_run_timestamp init ((uint64_t)i << 48): " << ((uint64_t)i << 48);
      //LOG(INFO) << "continue_v1_last_run_timestamp init ((uint64_t)j << 32): " << ((uint64_t)j << 32);
      //LOG(INFO) << "continue_v1_last_run_timestamp init key: " << map_key;
    }
  }
  // init continue v1
  for (int i = 0; i < continue_v1_action_key_code_list.size(); i++) {
    continue_v1_done_no.push_back(-1);
  }
  // init config
  if (no_mouse_absolute_rate == 0) {
    display_rate_x = DISPLAY_RATE / main_display_width;
    display_rate_y = DISPLAY_RATE / main_display_height;
  } else {
    display_rate_x = 1.0;
    display_rate_y = 1.0;
  }
  // init display color
  while (action_key_code_list.size() > display_color_list.size()) {
    int color = getColorFromPaletto();
    cv::Scalar color_scalar = getScalarColorFromInt(color);
    display_color_list.push_back(color_scalar);
  }
  while (coord_v3_action_key_code_list.size() > coord_v3_display_color_list.size()) {
    int color = getColorFromPaletto();
    cv::Scalar color_scalar = getScalarColorFromInt(color);
    coord_v3_display_color_list.push_back(color_scalar);
  }
  // init vector
  init_face_mesh_vector_vec();
  // verify check
  verify_check();
  
  
  LOG(INFO) << "*************************************************************";
  LOG(INFO) << "config_path: " << config_path;
  LOG(INFO) << "*************************************************************";
  
  fix_person_size_scale();
  
  int device_count = 1;
  if (absl::GetFlag(FLAGS_input_device_count).empty() == false) {
    LOG(INFO) << "input device count: " << absl::GetFlag(FLAGS_input_device_count);
    device_count = std::stoi(absl::GetFlag(FLAGS_input_device_count));
  }
  graphs_vec.resize(device_count);
  
  int user_count = 1;
  if (absl::GetFlag(FLAGS_input_user_count).empty() == false) {
    LOG(INFO) << "input user count: " << absl::GetFlag(FLAGS_input_user_count);
    user_count = std::stoi(absl::GetFlag(FLAGS_input_user_count));
  }
  
  const bool is_load_camera_id = !absl::GetFlag(FLAGS_input_camera_id).empty();
  if (is_load_camera_id) {
    LOG(INFO) << "FLAGS_input_camera_id: " << absl::GetFlag(FLAGS_input_camera_id);
    int arg_camera_id = std::stoi(absl::GetFlag(FLAGS_input_camera_id));
    LOG(INFO) << "use arg_camera_id(" << arg_camera_id << "), not use config_file/default camera_id(" << camera_id << ")";
    camera_id = arg_camera_id;
  }
  
  const bool is_load_display_mosaic_scale = !absl::GetFlag(FLAGS_input_display_mosaic_scale).empty();
  if (is_load_display_mosaic_scale) {
    LOG(INFO) << "FLAGS_input_display_mosaic_scale: " << absl::GetFlag(FLAGS_input_display_mosaic_scale);
    int arg_display_mosaic_scale = std::stoi(absl::GetFlag(FLAGS_input_display_mosaic_scale));
    LOG(INFO) << "use arg_display_mosaic_scale(" << arg_display_mosaic_scale << "), not use config_file/default display_mosaic_scale(" << display_mosaic_scale << ")";
    display_mosaic_scale = arg_display_mosaic_scale;
  }
  
  if (absl::GetFlag(FLAGS_input_proc_fps).empty() == false) {
    LOG(INFO) << "input arg proc fps: " << absl::GetFlag(FLAGS_input_proc_fps);
    camera_proc_fps = std::stof(absl::GetFlag(FLAGS_input_proc_fps));
  }
  
  const bool arg_is_auto_start = !absl::GetFlag(FLAGS_is_auto_start).empty();
  if (arg_is_auto_start == true) {
    LOG(INFO) << "FLAGS_is_auto_start: " << absl::GetFlag(FLAGS_is_auto_start);
    int int_arg_is_auto_start = std::stoi(absl::GetFlag(FLAGS_is_auto_start));
    LOG(INFO) << "use int_arg_is_auto_start(" << int_arg_is_auto_start << "), not use config_file/default display_mosaic_scale(" << display_mosaic_scale << ")";
    if (int_arg_is_auto_start == 0) {
      is_auto_start = false;
    } else {
      is_auto_start = true;
    }
  }
  
  absl::Status result;
  std::string calculator_graph_config_contents = default_config_context;
  if (false) {
    if (absl::GetFlag(FLAGS_calculator_graph_config_file).empty() == false) {
      LOG(INFO) << "input calculator graph config file: " << absl::GetFlag(FLAGS_calculator_graph_config_file);
      result = mediapipe::file::GetContents(
          absl::GetFlag(FLAGS_calculator_graph_config_file),
          &calculator_graph_config_contents);
      if (result.ok() == false) {
        LOG(INFO) << "graph config contexts Initialize error.";
      }
    }
  }
  //LOG(INFO) << "Get calculator graph config contents: " << calculator_graph_config_contents;
  
  // init config
  for (int d = 0; d < device_count; d++) {
    std::vector<mediapipe::CalculatorGraph*> graphs = graphs_vec[d];
    
    absl::Status is_run_status;
    for (int i = 0; i < user_count; i++) {
      
      graphs.emplace_back(new mediapipe::CalculatorGraph());
      
      mediapipe::CalculatorGraphConfig config =
          mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
              calculator_graph_config_contents);
      
      LOG(INFO) << "Initialize the calculator graph.";
      //mediapipe::CalculatorGraph graph;
      result = graphs[i]->Initialize(config);
      if (result.ok() == false) {
        LOG(INFO) << "graph Initialize error.";
      }
    }
    
    
    absl::Status run_status = RunMPPGraphsAtCameraDevice(graphs);
    if (!run_status.ok()) {
      LOG(ERROR) << "Failed to run the graph" << d << ": " << run_status.message();
      return EXIT_FAILURE;
    } else {
      LOG(INFO) << "Success!";
    }
  }
  
  
  return 0;
}
