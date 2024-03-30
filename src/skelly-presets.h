#pragma once

#include "skelly.h"

// Some presets for different walk cycles
inline void moveDefault(MoveOptions& move) {
  move = MoveOptions{};
}
inline void moveTightrope(MoveOptions& move) {
  move = MoveOptions{};
  move.max_speed = 120;
  move.max_leg_pct = 0.9;
  move.stance_w_pct = 0;
  move.hand_height_pct = 0.25;
  move.arm_span_pct = 0.9;
}
inline void movePreppy(MoveOptions& move) {
  move = MoveOptions{};
  move.max_speed = 240;
  move.max_leg_pct = 0.9;
  move.step_height = 15;
  move.bounce = 5;
  move.arm_span_pct = 0;
  move.hand_height_pct = 0.5;
}
inline void moveSnow(MoveOptions& move) {
  move = MoveOptions{};
  move.max_speed = 100;
  move.max_leg_pct = 0.9;
  move.step_height = 50;
  move.arm_span_pct = 0.25;
  move.hand_height_pct = 0.65;
  move.hands_forward = 15;
}
inline void moveRunway(MoveOptions& move) {
  move = MoveOptions{};
  move.stance_w_pct = 0.2;
  move.hip_sway = 12;
  move.hip_spin = 10;
  move.shoulder_spin = 12;
  move.arm_span_pct = 0.5;
  move.hand_height_pct = 0.9;
  move.hands_forward = -15;
}
inline void moveCrouch(MoveOptions& move) {
  move = MoveOptions{};
  move.max_speed = 120;
  move.max_leg_pct = 0.65;
  move.stance_w_pct = 1.2;
  move.step_height = 15;
  move.arm_span_pct = 0.2;
  move.hand_height_pct = 0.5;
  move.hands_forward = 15;
}
inline void moveFlanders(MoveOptions& move) {
  move = MoveOptions{};
  move.max_speed = 165;
  move.max_leg_pct = 0.90;
  move.bounce = 4;
  move.hip_sway = 23;
  move.hip_spin = 4.5;
  move.hand_height_pct = 0.53;
  move.hands_forward = 0;
}

// Some presets for different skeleton sizes
inline void sizeDefault(SkellySizes& sizes) {
  sizes = SkellySizes{};
}
inline void sizeTall(SkellySizes& sizes) {
  sizes = SkellySizes{};
  sizes.height = 250;
  sizes.leg = 140;
  sizes.arm = 110;
}
inline void sizeBig(SkellySizes& sizes) {
  sizes = SkellySizes{};
  sizes.height = 210;
  sizes.leg = 100;
  sizes.bone_w = 10;
  sizes.pelvis_w = 60;
  sizes.shoulders_w = 100;
  sizes.arm = 100;
}
inline void sizeDwarf(SkellySizes& sizes) {
  sizes = SkellySizes{};
  sizes.height = 114;
  sizes.leg = 40;
  sizes.pelvis_w = 25;
  sizes.shoulders_w = 40;
  sizes.arm = 50;
}
inline void sizeChimp(SkellySizes& sizes) {
  sizes = SkellySizes{};
  sizes.height = 150;
  sizes.leg = 50;
  sizes.arm = 100;
  sizes.bicep_pct = 0.4;
}
