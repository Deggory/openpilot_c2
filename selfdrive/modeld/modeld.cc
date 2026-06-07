#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <cmath>
#include <algorithm>

#include <eigen3/Eigen/Dense>

#include "cereal/messaging/messaging.h"
#include "cereal/visionipc/visionipc_client.h"
#include "selfdrive/common/clutil.h"
#include "selfdrive/common/modeldata.h"
#include "selfdrive/common/params.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/modeld/models/driving.h"

ExitHandler do_exit;

#ifdef USE_K230_KMODEL
namespace {

constexpr int K230_DEFAULT_WIDTH = 512;
constexpr int K230_DEFAULT_HEIGHT = 256;
constexpr int K230_DEFAULT_SENSOR_WIDTH = 1920;
constexpr int K230_DEFAULT_SENSOR_HEIGHT = 1080;
constexpr float K230_DEFAULT_SOURCE_FX = 910.0f;
constexpr float K230_DEFAULT_SOURCE_FY = 910.0f;
constexpr float K230_DEFAULT_SOURCE_CX = 256.0f;
constexpr float K230_DEFAULT_SOURCE_CY = 47.6f;

float getenv_optional_float(const char *key, float fallback) {
  return util::getenv(key).empty() ? fallback : util::getenv(key, fallback);
}

Eigen::Matrix<float, 3, 4> k230_zero_extrinsics() {
  Eigen::Matrix<float, 3, 4> extrinsics;
  extrinsics << 0.0f, -1.0f,  0.0f, 0.0f,
                0.0f,  0.0f, -1.0f, 1.22f,
                1.0f,  0.0f,  0.0f, 0.0f;
  return extrinsics;
}

mat3 k230_source_intrinsics(int source_width, int source_height) {
  const int crop_x = std::max(0, util::getenv("K230_CAM_CROP_X", 0));
  const int crop_y = std::max(0, util::getenv("K230_CAM_CROP_Y", 0));
  const int crop_width = std::max(2, util::getenv("K230_CAM_CROP_WIDTH", K230_DEFAULT_SENSOR_WIDTH));
  const int crop_height = std::max(2, util::getenv("K230_CAM_CROP_HEIGHT", K230_DEFAULT_SENSOR_HEIGHT));

  const float default_sensor_fx = K230_DEFAULT_SOURCE_FX * K230_DEFAULT_SENSOR_WIDTH / K230_DEFAULT_WIDTH;
  const float default_sensor_fy = K230_DEFAULT_SOURCE_FY * K230_DEFAULT_SENSOR_HEIGHT / K230_DEFAULT_HEIGHT;
  const float default_sensor_cx = K230_DEFAULT_SOURCE_CX * K230_DEFAULT_SENSOR_WIDTH / K230_DEFAULT_WIDTH;
  const float default_sensor_cy = K230_DEFAULT_SOURCE_CY * K230_DEFAULT_SENSOR_HEIGHT / K230_DEFAULT_HEIGHT;

  const float sensor_fx = getenv_optional_float("K230_SENSOR_FX", default_sensor_fx);
  const float sensor_fy = getenv_optional_float("K230_SENSOR_FY", default_sensor_fy);
  const float sensor_cx = getenv_optional_float("K230_SENSOR_CX", default_sensor_cx);
  const float sensor_cy = getenv_optional_float("K230_SENSOR_CY", default_sensor_cy);

  const float sx = static_cast<float>(source_width) / crop_width;
  const float sy = static_cast<float>(source_height) / crop_height;
  float preset_fx = sensor_fx * sx;
  float preset_fy = sensor_fy * sy;
  float preset_cx = (sensor_cx - crop_x) * sx;
  float preset_cy = (sensor_cy - crop_y) * sy;

  const std::string preset = util::getenv("K230_SOURCE_PRESET");
  if (preset == "wide") {
    preset_fx = 455.0f;
    preset_fy = 455.0f;
    preset_cx = 256.0f;
    preset_cy = 104.0f;
  } else if (preset == "cal") {
    preset_fx = 606.7f;
    preset_fy = 606.7f;
    preset_cx = 256.0f;
    preset_cy = 47.6f;
  } else if (!preset.empty() && preset != "medmodel") {
    LOGE("unknown K230_SOURCE_PRESET=%s", preset.c_str());
  }

  const float fx = getenv_optional_float("K230_SOURCE_FX", preset_fx);
  const float fy = getenv_optional_float("K230_SOURCE_FY", preset_fy);
  const float cx = getenv_optional_float("K230_SOURCE_CX", preset_cx);
  const float cy = getenv_optional_float("K230_SOURCE_CY", preset_cy);

  LOGW("K230 source intrinsics %.3f %.3f %.3f %.3f for %dx%d crop=%dx%d+%d+%d",
       fx, fy, cx, cy, source_width, source_height, crop_width, crop_height, crop_x, crop_y);
  return (mat3){{
    fx, 0.0f, cx,
    0.0f, fy, cy,
    0.0f, 0.0f, 1.0f,
  }};
}

float k230_transform_inbounds_ratio(const mat3 &projection, int source_width, int source_height) {
  constexpr int model_width = 512;
  constexpr int model_height = 256;
  constexpr int step = 8;

  int valid = 0;
  int total = 0;
  for (int y = 0; y < model_height; y += step) {
    for (int x = 0; x < model_width; x += step) {
      const float x0 = projection.v[0] * x + projection.v[1] * y + projection.v[2];
      const float y0 = projection.v[3] * x + projection.v[4] * y + projection.v[5];
      const float w0 = projection.v[6] * x + projection.v[7] * y + projection.v[8];
      if (std::fabs(w0) > 1e-6f) {
        const float sx = x0 / w0;
        const float sy = y0 / w0;
        if (std::isfinite(sx) && std::isfinite(sy) &&
            sx >= -1.0f && sx < source_width &&
            sy >= -1.0f && sy < source_height) {
          ++valid;
        }
      }
      ++total;
    }
  }
  return total > 0 ? static_cast<float>(valid) / total : 0.0f;
}

}  // namespace
#endif

mat3 update_calibration(const Eigen::Matrix<float, 3, 4> &extrinsics, const mat3 &cam_intrinsics_mat, bool bigmodel_frame) {
  /*
     import numpy as np
     from common.transformations.model import medmodel_frame_from_road_frame
     medmodel_frame_from_ground = medmodel_frame_from_road_frame[:, (0, 1, 3)]
     ground_from_medmodel_frame = np.linalg.inv(medmodel_frame_from_ground)
  */
  static const auto ground_from_medmodel_frame = (Eigen::Matrix<float, 3, 3>() <<
     0.00000000e+00, 0.00000000e+00, 1.00000000e+00,
    -1.09890110e-03, 0.00000000e+00, 2.81318681e-01,
    -1.84808520e-20, 9.00738606e-04, -4.28751576e-02).finished();

  static const auto ground_from_sbigmodel_frame = (Eigen::Matrix<float, 3, 3>() <<
     0.00000000e+00,  7.31372216e-19,  1.00000000e+00,
    -2.19780220e-03,  4.11497335e-19,  5.62637363e-01,
    -5.46146580e-20,  1.80147721e-03, -2.73464241e-01).finished();

  static const mat3 yuv_transform = get_model_yuv_transform();
  const auto cam_intrinsics = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>(cam_intrinsics_mat.v);

  auto ground_from_model_frame = bigmodel_frame ? ground_from_sbigmodel_frame : ground_from_medmodel_frame;
  auto camera_frame_from_road_frame = cam_intrinsics * extrinsics;
  Eigen::Matrix<float, 3, 3> camera_frame_from_ground;
  camera_frame_from_ground.col(0) = camera_frame_from_road_frame.col(0);
  camera_frame_from_ground.col(1) = camera_frame_from_road_frame.col(1);
  camera_frame_from_ground.col(2) = camera_frame_from_road_frame.col(3);

  auto warp_matrix = camera_frame_from_ground * ground_from_model_frame;
  mat3 transform = {};
  for (int i=0; i<3*3; i++) {
    transform.v[i] = warp_matrix(i / 3, i % 3);
  }
  return matmul3(yuv_transform, transform);
}

static uint64_t get_ts(const VisionIpcBufExtra &extra) {
  return Hardware::TICI() ? extra.timestamp_sof : extra.timestamp_eof;
}

void run_model(ModelState &model, VisionIpcClient &vipc_client_main, VisionIpcClient &vipc_client_extra,
               bool main_wide_camera, bool use_extra_client, int main_width, int main_height) {
  // messaging
  PubMaster pm({"modelV2", "cameraOdometry"});
  SubMaster sm({"lateralPlan", "roadCameraState", "liveCalibration"});

  // setup filter to track dropped frames
  FirstOrderFilter frame_dropped_filter(0., 10., 1. / MODEL_FREQ);

  uint32_t frame_id = 0, last_vipc_frame_id = 0;
  double last = 0;
  uint32_t run_count = 0;

  mat3 model_transform_main = {};
  mat3 model_transform_extra = {};
  bool live_calib_seen = false;

#ifdef USE_K230_KMODEL
  const mat3 main_cam_intrinsics = k230_source_intrinsics(main_width, main_height);
  const mat3 extra_cam_intrinsics = main_cam_intrinsics;
  const float min_warp_inbounds = util::getenv("K230_MIN_WARP_INBOUNDS", 0.85f);
#else
  (void)main_width;
  (void)main_height;
  const mat3 main_cam_intrinsics = main_wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  const mat3 extra_cam_intrinsics = Hardware::TICI() ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
#endif
  const Eigen::Matrix<float, 3, 4> zero_extrinsics =
#ifdef USE_K230_KMODEL
      k230_zero_extrinsics();
#else
      Eigen::Matrix<float, 3, 4>::Zero();
#endif
  model_transform_main = update_calibration(zero_extrinsics, main_cam_intrinsics, false);
  model_transform_extra = update_calibration(zero_extrinsics, extra_cam_intrinsics, true);

  VisionBuf *buf_main = nullptr;
  VisionBuf *buf_extra = nullptr;

  VisionIpcBufExtra meta_main = {0};
  VisionIpcBufExtra meta_extra = {0};

  while (!do_exit) {
    // Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    while (get_ts(meta_main) < get_ts(meta_extra) + 25000000ULL) {
      buf_main = vipc_client_main.recv(&meta_main);
      if (buf_main == nullptr)  break;
    }

    if (buf_main == nullptr) {
      LOGE("vipc_client_main no frame");
      continue;
    }

    if (use_extra_client) {
      // Keep receiving extra frames until frame id matches main camera
      do {
        buf_extra = vipc_client_extra.recv(&meta_extra);
      } while (buf_extra != nullptr && get_ts(meta_main) > get_ts(meta_extra) + 25000000ULL);

      if (buf_extra == nullptr) {
        LOGE("vipc_client_extra no frame");
        continue;
      }

      if (std::abs((int64_t)meta_main.timestamp_sof - (int64_t)meta_extra.timestamp_sof) > 10000000ULL) {
        LOGE("frames out of sync! main: %d (%.5f), extra: %d (%.5f)",
          meta_main.frame_id, double(meta_main.timestamp_sof) / 1e9,
          meta_extra.frame_id, double(meta_extra.timestamp_sof) / 1e9);
      }
    } else {
      // Use single camera
      buf_extra = buf_main;
      meta_extra = meta_main;
    }

    // TODO: path planner timeout?
    sm.update(0);
    int desire = ((int)sm["lateralPlan"].getLateralPlan().getDesire());
    frame_id = sm["roadCameraState"].getRoadCameraState().getFrameId();
    if (sm.updated("liveCalibration")) {
      auto extrinsic_matrix = sm["liveCalibration"].getLiveCalibration().getExtrinsicMatrix();
      Eigen::Matrix<float, 3, 4> extrinsic_matrix_eigen;
      for (int i = 0; i < 4*3; i++) {
        extrinsic_matrix_eigen(i / 4, i % 4) = extrinsic_matrix[i];
      }

      const mat3 candidate_transform_main = update_calibration(extrinsic_matrix_eigen, main_cam_intrinsics, false);
      const mat3 candidate_transform_extra = update_calibration(extrinsic_matrix_eigen, extra_cam_intrinsics, true);
#ifdef USE_K230_KMODEL
      const float inbounds = k230_transform_inbounds_ratio(candidate_transform_main, main_width, main_height);
      if (util::getenv("K230_LOG_WARP_BOUNDS", 0) != 0) {
        LOGW("K230 main warp inbounds %.1f%% for source %dx%d", inbounds * 100.0f, main_width, main_height);
      }
      if (inbounds >= min_warp_inbounds) {
        model_transform_main = candidate_transform_main;
        model_transform_extra = candidate_transform_extra;
      } else {
        LOGE("rejecting K230 calibration warp inbounds %.1f%% below %.1f%%",
             inbounds * 100.0f, min_warp_inbounds * 100.0f);
      }
#else
      model_transform_main = candidate_transform_main;
      model_transform_extra = candidate_transform_extra;
#endif
      live_calib_seen = true;
    }

    float vec_desire[DESIRE_LEN] = {0};
    if (desire >= 0 && desire < DESIRE_LEN) {
      vec_desire[desire] = 1.0;
    }

    // tracked dropped frames
    uint32_t vipc_dropped_frames = meta_main.frame_id - last_vipc_frame_id - 1;
    float frames_dropped = frame_dropped_filter.update((float)std::min(vipc_dropped_frames, 10U));
    if (run_count < 10) { // let frame drops warm up
      frame_dropped_filter.reset(0);
      frames_dropped = 0.;
    }
    run_count++;

    float frame_drop_ratio = frames_dropped / (1 + frames_dropped);

    double mt1 = millis_since_boot();
    ModelOutput *model_output = model_eval_frame(&model, buf_main, buf_extra, model_transform_main, model_transform_extra, vec_desire);
    double mt2 = millis_since_boot();
    float model_execution_time = (mt2 - mt1) / 1000.0;

    model_publish(pm, meta_main.frame_id, meta_extra.frame_id, frame_id, frame_drop_ratio, *model_output, meta_main.timestamp_eof, model_execution_time,
                  kj::ArrayPtr<const float>(model.output.data(), model.output.size()), live_calib_seen);
    posenet_publish(pm, meta_main.frame_id, vipc_dropped_frames, *model_output, meta_main.timestamp_eof, live_calib_seen);

    //printf("model process: %.2fms, from last %.2fms, vipc_frame_id %u, frame_id, %u, frame_drop %.3f\n", mt2 - mt1, mt1 - last, extra.frame_id, frame_id, frame_drop_ratio);
    last = mt1;
    last_vipc_frame_id = meta_main.frame_id;
  }
}

int main(int argc, char **argv) {
#ifndef USE_K230_KMODEL
  if (!Hardware::PC()) {
    int ret;
    ret = util::set_realtime_priority(54);
    assert(ret == 0);
    util::set_core_affinity({Hardware::EON() ? 2 : 7});
    assert(ret == 0);
  }
#endif

  bool main_wide_camera = Hardware::TICI() ? Params().getBool("EnableWideCamera") : false;
  bool use_extra_client = Hardware::TICI() && !main_wide_camera;

  // cl init
  cl_device_id device_id = nullptr;
  cl_context context = nullptr;
#ifndef USE_K230_KMODEL
  device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));
#endif

  // init the models
  ModelState model;
  model_init(&model, device_id, context);
  LOGW("models loaded, modeld starting");

  VisionIpcClient vipc_client_main = VisionIpcClient("camerad", main_wide_camera ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_ROAD, true, device_id, context);
  VisionIpcClient vipc_client_extra = VisionIpcClient("camerad", VISION_STREAM_WIDE_ROAD, false, device_id, context);

  while (!do_exit && !vipc_client_main.connect(false)) {
    util::sleep_for(100);
  }

  while (!do_exit && use_extra_client && !vipc_client_extra.connect(false)) {
    util::sleep_for(100);
  }

  // run the models
  // vipc_client.connected is false only when do_exit is true
  if (!do_exit) {
    const VisionBuf *b = &vipc_client_main.buffers[0];
    LOGW("connected main cam with buffer size: %d (%d x %d)", b->len, b->width, b->height);

    if (use_extra_client) {
      const VisionBuf *wb = &vipc_client_extra.buffers[0];
      LOGW("connected extra cam with buffer size: %d (%d x %d)", wb->len, wb->width, wb->height);
    }

    run_model(model, vipc_client_main, vipc_client_extra, main_wide_camera, use_extra_client,
              b->width, b->height);
  }

  model_free(&model);
  if (context != nullptr) {
    CL_CHECK(clReleaseContext(context));
  }
  return 0;
}
