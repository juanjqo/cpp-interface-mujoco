/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

// Dear ImGui configuration (IMGUI_USER_CONFIG, see CMakeLists.txt)

#pragma once

// Each interface draws its GUI in its own viewer thread, so two interfaces draw at the same time:
// the current ImGui context is per thread, not global.
struct ImGuiContext;
extern thread_local ImGuiContext* DQMujocoImGuiContext;
#define GImGui DQMujocoImGuiContext

// The library never writes files on its own (imgui.ini, imgui_log.txt) nor opens URLs
#define IMGUI_DISABLE_DEFAULT_SHELL_FUNCTIONS
#define IMGUI_DISABLE_OBSOLETE_FUNCTIONS
