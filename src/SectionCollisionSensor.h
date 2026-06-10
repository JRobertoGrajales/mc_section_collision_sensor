/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
 
#include "LpfThreshold.h"
#include "ROS2Subscriber.h" 

#include <mc_control/GlobalPlugin.h>
 
namespace mc_plugin
{

/**
 * SectionCollisionSensor — mc_rtc GlobalPlugin
 *
 * Monitors the voltage output of an EL3102 ADC channel published as a
 * ROS 2 Float64 topic by the standalone EtherCAT ROS node.  Applies an
 * IIR low-pass filter and maps the result to one of 7 contact segments:
 *
 *   Voltage range    Segment   Nominal voltage
 *   ─────────────    ───────   ───────────────
 *   0.0 – 1.0 V       0       no contact
 *   1.0 – 2.5 V       1       1.75 V
 *   2.5 – 4.0 V       2       3.25 V
 *   4.0 – 5.5 V       3       4.75 V
 *   5.5 – 7.0 V       4       6.25 V
 *   7.0 – 8.5 V       5       7.75 V
 *   8.5 – 10.0 V      6       9.25 V
 *
 * Datastore keys written:
 *   "ADC_voltage"       (double) — raw voltage, updated every cycle
 *   "ADC_segment"       (int)    — contact segment 0-6
 *   "Obstacle detected" (bool)   — true when segment >= 1
 */

struct SectionCollisionSensor : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller,
            const mc_rtc::Configuration & config) override;
 
  void reset(mc_control::MCGlobalController & controller) override;
 
  void before(mc_control::MCGlobalController & controller) override;
 
  void after(mc_control::MCGlobalController & controller) override;

  void addGui(mc_control::MCGlobalController & controller);
  void addLog(mc_control::MCGlobalController & controller);
 
  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;
 
  ~SectionCollisionSensor() override;
 
private:
  /** Map a filtered voltage to a contact segment (0 = no contact, 1-6 = active). */
  static int voltageToSegment(double voltage) noexcept;

  // GUI
  bool activate_verbose_ = true;

  // ── ROS 2 ────────────────────────────────────────────────────────────────
  std::shared_ptr<rclcpp::Node> node_;
  std::thread spinThread_;
  std::mutex mutex_;
  ROSFloatSubscriber voltage_sub_;
  void rosSpinner(void);
  bool stop_thread = false;

  double voltage_in_ = 0.0;
 
  // ── Signal processing ────────────────────────────────────────────────────
  LpfThreshold lpf_;
  double threshold_filtering_ = 1.0; ///< LPF coefficient (0–1)
 
  // ── State ─────────────────────────────────────────────────────────────────
  int  contact_segment_      = 0;   ///< current segment 0-6
  int  prev_contact_segment_ = 0;   ///< previous cycle segment
  bool collision_stop_activated_ = false;
};
 
} // namespace mc_plugin
