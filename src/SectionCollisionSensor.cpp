/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "SectionCollisionSensor.h"
#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

// ─────────────────────────────────────────────────────────────────────────────
// Voltage thresholds separating the 7 contact zones (Volts).
// Zone boundaries: 0 | 1.0 | 2.5 | 4.0 | 5.5 | 7.0 | 8.5 | 10.0
// ─────────────────────────────────────────────────────────────────────────────
static constexpr double SEGMENT_THRESHOLDS[] = {1.0, 2.5, 4.0, 5.5, 7.0, 8.5};
static constexpr int NUM_THRESHOLDS =
    static_cast<int>(sizeof(SEGMENT_THRESHOLDS) / sizeof(SEGMENT_THRESHOLDS[0]));

int SectionCollisionSensor::voltageToSegment(double voltage) noexcept
{
  for(int i = 0; i < NUM_THRESHOLDS; ++i)
  {
    if(voltage < SEGMENT_THRESHOLDS[i]) { return i; }
  }
  return 6;
}

SectionCollisionSensor::~SectionCollisionSensor() = default;

// ─────────────────────────────────────────────────────────────────────────────
void SectionCollisionSensor::init(mc_control::MCGlobalController & controller,
                               const mc_rtc::Configuration & config)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  // Make sure to have obstacle detection datastore entry
  if(!ctl.controller().datastore().has("Obstacle detected"))
  {
    ctl.controller().datastore().make<bool>("Obstacle detected", false);
  }
  if(!ctl.controller().datastore().has("ADC_segment"))
  {
    ctl.controller().datastore().make<int>("ADC_segment", 0);
  }

  // ── Config ────────────────────────────────────────────────────────────────
  std::string voltage_topic = config("voltage_topic", std::string("/collision/voltage"));
  activate_verbose_    = config("verbose",             true);
  threshold_offset_ = config("threshold_offset",     5.0);   // Volts
  threshold_filtering_ = config("threshold_filtering",  1.0);  // LPF alpha

  // LpfThreshold::setValues(offset, filtering, jointNumber)
  // jointNumber=1 because we have a single scalar ADC channel
  lpf_.setValues(threshold_offset_, threshold_filtering_, 1);

  // ── ROS 2 subscriber ─────────────────────────────────────────────────────
  if(!ctl.controller().datastore().has("ros_spin"))
  {
    ctl.controller().datastore().make<bool>("ros_spin", false);
  }
  node_ = mc_rtc::ROSBridge::get_node_handle();
  if(!ctl.controller().datastore().get<bool>("ros_spin"))
  {
    spinThread_ = std::thread(std::bind(&SectionCollisionSensor::rosSpinner, this));
    ctl.controller().datastore().assign("ros_spin", true);
  }

  voltage_sub_.subscribe(node_, voltage_topic);
  voltage_sub_.maxTime(ctl.timestep());

  addGui(ctl);
  addLog(ctl);

  mc_rtc::log::info("[SectionCollisionSensor] init called with configuration:\n{}", config.dump(true, true));
}

// ─────────────────────────────────────────────────────────────────────────────
void SectionCollisionSensor::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("[SectionCollisionSensor] reset called");
}

// ─────────────────────────────────────────────────────────────────────────────
void SectionCollisionSensor::before(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  // ── 1. Grab latest voltage ────────────────────────────────────────────────
  voltage_in_ = voltage_sub_.data().value();

  // ── 2. Compute adaptive threshold bounds ─────────────────────────────────
  threshold_high_ = lpf_.adaptiveThreshold(voltage_in_, true);
  threshold_low_  = lpf_.adaptiveThreshold(voltage_in_, false);
  double v_filtered = lpf_.getFilteredSignal();
  contact_segment_ = voltageToSegment(v_filtered);

  // ── 3. Map filtered voltage → segment ────────────────────────────────────
  threshold_high_ = lpf_.adaptiveThreshold(voltage_in_, true);
  threshold_low_  = lpf_.adaptiveThreshold(voltage_in_, false);
  double v_filtered = lpf_.filter(voltage_in_);  // or however LpfThreshold filters
  contact_segment_ = voltageToSegment(v_filtered);

  // ── 4. Edge logging ───────────────────────────────────────────────────────
  if(contact_segment_ != prev_contact_segment_)
  {
    if(activate_verbose_)
    {
      mc_rtc::log::info(
        "[SectionCollisionSensor] segment changed: {} → {}  (v_raw={:.3f} V, v_filt={:.3f} V)",   //<------------- Check
        prev_contact_segment_, contact_segment_, voltage_in_, v_filtered);                    //<------------- Check
    }
    prev_contact_segment_ = contact_segment_;                                                 //<------------- Check
  }

  // ── 5. Datastore ─────────────────────────────────────────────────────────
  if(collision_stop_activated_)
  {
    ctl.controller().datastore().get<bool>("Obstacle detected") = (contact_segment_ > 0);
    ctl.controller().datastore().get<int>("ADC_segment")        = contact_segment_;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void SectionCollisionSensor::after(mc_control::MCGlobalController & /*controller*/)
{
  // Reserved for post-controller diagnostics.
}

// ─────────────────────────────────────────────────────────────────────────────
mc_control::GlobalPlugin::GlobalPluginConfiguration SectionCollisionSensor::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after  = false;
  out.should_always_run = true;
  return out;
}

void SectionCollisionSensor::rosSpinner(void)
{
  mc_rtc::log::info("[SectionCollisionSensor][ROS Spinner] thread created for voltage subscriber");
  rclcpp::Rate r(1000); // Hz
  while(rclcpp::ok() and !stop_thread)
  {
    rclcpp::spin_some(node_);
    r.sleep();
  }
  mc_rtc::log::info("[SectionCollisionSensor][ROS Spinner] spinner destroyed");
}

void SectionCollisionSensor::addGui(mc_control::MCGlobalController & controller) //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  ctl.controller().gui()->addElement(
    {"Plugins", "SectionCollisionSensor"},
    mc_rtc::gui::NumberInput(
      "threshold_filtering",
      [this]() { return threshold_filtering_; },
      [this](double v)
      {
        threshold_filtering_ = v;
        lpf_.setFiltering(v);
      }),
    mc_rtc::gui::Label("voltage",
      [this]() { return std::to_string(voltage_in_); }),
    mc_rtc::gui::Label("segment (0=none, 1-6=contact)",
      [this]() { return std::to_string(contact_segment_); }),
    mc_rtc::gui::Label("obstacle detected",
      [this]() { return (contact_segment_ > 0) ? std::string("YES") : std::string("no"); }),
    mc_rtc::gui::Checkbox("Collision stop", collision_stop_activated_),
    mc_rtc::gui::Checkbox("Verbose", activate_verbose_)
  );
}

void SectionCollisionSensor::addLog(mc_control::MCGlobalController & controller)
{
  auto & logger = controller.controller().logger();
  logger.addLogEntry("SectionCollisionSensor_voltage",
                     [this]() { return voltage_in_; });
  logger.addLogEntry("SectionCollisionSensor_segment",
                     [this]() { return contact_segment_; });
  logger.addLogEntry("SectionCollisionSensor_obstacle_detected",
                     [this]() { return contact_segment_ > 0; });
  logger.addLogEntry("SectionCollisionSensor_threshold_filtering",
                     [this]() { return threshold_filtering_; });
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("SectionCollisionSensor", mc_plugin::SectionCollisionSensor)
