/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ADCCollisionSensor.h"
#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

ADCCollisionSensor::~ADCCollisionSensor() = default;

// ─────────────────────────────────────────────────────────────────────────────
void ADCCollisionSensor::init(mc_control::MCGlobalController & controller,
                               const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("[ADCCollisionSensor] init called with configuration:\n{}",
                    config.dump(true, true));

  // ── Config ───────────────────────────────────────────────────────────────
  voltage_topic_        = config("voltage_topic",        std::string("/collision/voltage"));
  verbose_              = config("verbose",               false);
  threshold_offset_     = config("threshold_offset",     1.0);   // Volts
  threshold_filtering_  = config("threshold_filtering",  0.05);  // LPF alpha

  // LpfThreshold::setValues(offset, filtering, jointNumber)
  // jointNumber=1 because we have a single scalar ADC channel
  lpf_.setValues(threshold_offset_, threshold_filtering_, 1);

  // ── ROS 2 subscriber ─────────────────────────────────────────────────────
  node_ = mc_rtc::ROSBridge::get_node_handle();

  voltage_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    voltage_topic_,
    rclcpp::QoS(1).best_effort(),
    [this](const std_msgs::msg::Float64::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      voltage_in_ = msg->data;
    });

  // ── Datastore ─────────────────────────────────────────────────────────────
  auto & ds = controller.controller().datastore();

  if(!ds.has("ADC_voltage"))
    ds.make<double>("ADC_voltage", 0.0);

  if(!ds.has("ADC_collision"))
    ds.make<bool>("ADC_collision", false);

  if(!ds.has("Obstacle detected"))
    ds.make<bool>("Obstacle detected", false);

  // ── Logger ────────────────────────────────────────────────────────────────
  auto & logger = controller.controller().logger();

  logger.addLogEntry("ADCCollisionSensor_voltage",
                     [this]() { return voltage_in_; });

  logger.addLogEntry("ADCCollisionSensor_threshold_high",
                     [this]() { return threshold_high_; });

  logger.addLogEntry("ADCCollisionSensor_threshold_low",
                     [this]() { return threshold_low_; });

  logger.addLogEntry("ADCCollisionSensor_collision",
                     [this]() { return collision_; });

  // ── GUI ───────────────────────────────────────────────────────────────────
  controller.controller().gui()->addElement(
    {"Plugins", "ADCCollisionSensor"},
    mc_rtc::gui::NumberInput(
      "threshold_filtering",
      [this]() { return threshold_filtering_; },
      [this](double v)
      {
        threshold_filtering_ = v;
        lpf_.setFiltering(v);
      }),
    mc_rtc::gui::NumberInput(
      "threshold_offset (V)",
      [this]() { return threshold_offset_; },
      [this](double v)
      {
        threshold_offset_ = v;
        lpf_.setOffset(v);
      }),
    mc_rtc::gui::Checkbox(
      "Verbose",
      [this]() { return verbose_; },
      [this]() { verbose_ = !verbose_; }),
    mc_rtc::gui::Label("voltage",
      [this]() { return std::to_string(voltage_in_); }),
    mc_rtc::gui::Label("threshold_high",
      [this]() { return std::to_string(threshold_high_); }),
    mc_rtc::gui::Label("threshold_low",
      [this]() { return std::to_string(threshold_low_); }),
    mc_rtc::gui::Label("collision",
      [this]() { return collision_ ? std::string("YES") : std::string("no"); })
  );

  mc_rtc::log::info("[ADCCollisionSensor] init done — topic: {}, "
                    "offset: {:.3f} V, filtering: {:.4f}",
                    voltage_topic_, threshold_offset_, threshold_filtering_);
}

// ─────────────────────────────────────────────────────────────────────────────
void ADCCollisionSensor::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("[ADCCollisionSensor] reset called");

  // Re-zero the LPF state by re-initialising with the same parameters
  lpf_.setValues(threshold_offset_, threshold_filtering_, 1);

  collision_      = false;
  prev_collision_ = false;
  threshold_high_ = 0.0;
  threshold_low_  = 0.0;

  controller.controller().gui()->removeElements(
    {"Plugins", "ADCCollisionSensor"}, this);
}

// ─────────────────────────────────────────────────────────────────────────────
void ADCCollisionSensor::before(mc_control::MCGlobalController & controller)
{
  // ── 1. Grab latest voltage (thread-safe) ─────────────────────────────────
  double v;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    v = voltage_in_;
  }

  // ── 2. Compute adaptive threshold bounds ─────────────────────────────────
  // adaptiveThreshold(signal, true)  → filtered + offset  (upper bound)
  // adaptiveThreshold(signal, false) → filtered - offset  (lower bound)
  // NOTE: both calls feed the same sample into the filter; the second call
  // will double-step the filter state.  To avoid this we call high first,
  // cache it, then compute low from the same filtered value directly.
  threshold_high_ = lpf_.adaptiveThreshold(v, true);
  // For the lower bound we derive it without stepping the filter again:
  // low = high - 2*offset
  threshold_low_  = threshold_high_ - 2.0 * threshold_offset_;

  // ── 3. Collision decision ─────────────────────────────────────────────────
  collision_ = (v > threshold_high_) || (v < threshold_low_);

  // ── 4. Edge logging ───────────────────────────────────────────────────────
  if(collision_ && !prev_collision_)
  {
    mc_rtc::log::info("[ADCCollisionSensor] *** COLLISION DETECTED *** "
                      "v={:.3f} V  high={:.3f}  low={:.3f}",
                      v, threshold_high_, threshold_low_);
  }
  if(verbose_ && !collision_ && prev_collision_)
  {
    mc_rtc::log::info("[ADCCollisionSensor] collision cleared, v={:.3f} V", v);
  }
  prev_collision_ = collision_;

  // ── 5. Datastore ─────────────────────────────────────────────────────────
  auto & ds = controller.controller().datastore();
  ds.get<double>("ADC_voltage")     = v;
  ds.get<bool>("ADC_collision")     = collision_;
  ds.get<bool>("Obstacle detected") = collision_;
}

// ─────────────────────────────────────────────────────────────────────────────
void ADCCollisionSensor::after(mc_control::MCGlobalController & /*controller*/)
{
  // Reserved for post-controller diagnostics.
}

// ─────────────────────────────────────────────────────────────────────────────
mc_control::GlobalPlugin::GlobalPluginConfiguration ADCCollisionSensor::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after  = false;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ADCCollisionSensor", mc_plugin::ADCCollisionSensor)
