#pragma once

#include "skelly.h"

// Some presets for different walk cycles
inline void moveDefault(WalkOptions& walk, MoveMods& mods) {
  walk = WalkOptions{};
  mods = MoveMods{};
}
inline void moveTightrope(WalkOptions& walk, MoveMods& mods) {
  walk = WalkOptions{};
  walk.speed = 120;
  walk.stance_w_pct = 0;
  walk.hand_height_pct = 0.25;
  walk.arm_span_pct = 0.9;
  mods.crouch_pct = 0.95;
}
inline void movePreppy(WalkOptions& walk, MoveMods& mods) {
  walk = WalkOptions{};
  walk.speed = 240;
  walk.step_height = 15;
  walk.bounce = 5;
  walk.arm_span_pct = 0;
  walk.hand_height_pct = 0.5;
  mods.crouch_pct = 0.95;
}
inline void moveSnow(WalkOptions& walk, MoveMods& mods) {
  walk = WalkOptions{};
  walk.speed = 100;
  walk.step_height = 50;
  walk.arm_span_pct = 0.25;
  walk.hand_height_pct = 0.65;
  walk.hands_forward = 15;
  mods.crouch_pct = 0.95;
}
inline void moveRunway(WalkOptions& walk, MoveMods& mods) {
  walk = WalkOptions{};
  walk.stance_w_pct = 0.2;
  walk.hip_sway = 12;
  walk.hip_spin = 10;
  walk.shoulder_spin = 12;
  walk.arm_span_pct = 0.5;
  walk.hand_height_pct = 0.9;
  walk.hands_forward = -15;
}
inline void moveCrouch(WalkOptions& walk, MoveMods& mods) {
  walk = WalkOptions{};
  walk.speed = 120;
  walk.stance_w_pct = 1.2;
  walk.step_height = 15;
  walk.arm_span_pct = 0.2;
  walk.hand_height_pct = 0.5;
  walk.hands_forward = 15;
  mods.crouch_pct = 0.7;
}
inline void moveFlanders(WalkOptions& walk, MoveMods& mods) {
  walk = WalkOptions{};
  walk.speed = 165;
  walk.bounce = 4;
  walk.hip_sway = 23;
  walk.hip_spin = 4.5;
  walk.hand_height_pct = 0.53;
  walk.hands_forward = 0;
  mods.crouch_pct = 0.95;
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
