/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

#include "DQ_MujocoGui.h"
#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_opengl3.h"
#include "ImGuizmo.h"
#include "imoguizmo.hpp"
#include "GLFW/glfw3.h"  // key codes and the OpenGL 1.1 functions (glClear, ...)
#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <functional>

// The current ImGui context of each thread (see dq_mujoco_imconfig.h)
thread_local ImGuiContext* DQMujocoImGuiContext = nullptr;

// The font of the GUI, embedded by CMakeLists.txt
extern const unsigned char dq_mujoco_gui_font[];
extern const unsigned int dq_mujoco_gui_font_size;

namespace DQ_robotics_extensions
{

namespace
{
/**
 * @brief opengl_backend_mutex serializes the calls to ImGui's OpenGL backend. Its OpenGL loader is
 *        global: ImGui_ImplOpenGL3_Shutdown() of one interface clears the function pointers that
 *        the viewer thread of another one is using (the other entry points load them again).
 */
std::mutex& opengl_backend_mutex()
{
    static std::mutex mutex;
    return mutex;
}

//--------------------------------------------------------------------------------------------------
// Style: a dark theme in the style of the editors of game engines and 3D tools
//--------------------------------------------------------------------------------------------------
constexpr float FONT_SIZE = 15.0f;

ImVec4 color(const unsigned int& rgb, const float& alpha = 1.0f)
{
    return ImVec4(((rgb >> 16) & 0xFF) / 255.0f, ((rgb >> 8) & 0xFF) / 255.0f, (rgb & 0xFF) / 255.0f, alpha);
}

const unsigned int ACCENT = 0x4FA3E0;
const unsigned int SELECTION = 0xFFB020;   // amber: stands out against the blue floor and robots
const unsigned int BACKGROUND = 0x1F2125;  // also behind the 3D view while it is drawn
const ImU32 AXIS_COLORS[4] = {IM_COL32(200, 70, 70, 255),    // x: red
                              IM_COL32(95, 165, 60, 255),    // y: green
                              IM_COL32(65, 120, 205, 255),   // z: blue
                              IM_COL32(130, 130, 140, 255)}; // w (quaternions)

/**
 * @brief glfw_to_imgui_key translates a GLFW key code (the keys that the GUI uses).
 */
ImGuiKey glfw_to_imgui_key(const int& key)
{
    if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z)
        return static_cast<ImGuiKey>(ImGuiKey_A + (key - GLFW_KEY_A));
    if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9)
        return static_cast<ImGuiKey>(ImGuiKey_0 + (key - GLFW_KEY_0));
    if (key >= GLFW_KEY_F1 && key <= GLFW_KEY_F12)
        return static_cast<ImGuiKey>(ImGuiKey_F1 + (key - GLFW_KEY_F1));
    switch (key)
    {
    case GLFW_KEY_TAB:           return ImGuiKey_Tab;
    case GLFW_KEY_LEFT:          return ImGuiKey_LeftArrow;
    case GLFW_KEY_RIGHT:         return ImGuiKey_RightArrow;
    case GLFW_KEY_UP:            return ImGuiKey_UpArrow;
    case GLFW_KEY_DOWN:          return ImGuiKey_DownArrow;
    case GLFW_KEY_PAGE_UP:       return ImGuiKey_PageUp;
    case GLFW_KEY_PAGE_DOWN:     return ImGuiKey_PageDown;
    case GLFW_KEY_HOME:          return ImGuiKey_Home;
    case GLFW_KEY_END:           return ImGuiKey_End;
    case GLFW_KEY_INSERT:        return ImGuiKey_Insert;
    case GLFW_KEY_DELETE:        return ImGuiKey_Delete;
    case GLFW_KEY_BACKSPACE:     return ImGuiKey_Backspace;
    case GLFW_KEY_SPACE:         return ImGuiKey_Space;
    case GLFW_KEY_ENTER:         return ImGuiKey_Enter;
    case GLFW_KEY_KP_ENTER:      return ImGuiKey_KeypadEnter;
    case GLFW_KEY_ESCAPE:        return ImGuiKey_Escape;
    case GLFW_KEY_MINUS:         return ImGuiKey_Minus;
    case GLFW_KEY_PERIOD:        return ImGuiKey_Period;
    case GLFW_KEY_COMMA:         return ImGuiKey_Comma;
    case GLFW_KEY_SLASH:         return ImGuiKey_Slash;
    case GLFW_KEY_LEFT_CONTROL:  return ImGuiKey_LeftCtrl;
    case GLFW_KEY_RIGHT_CONTROL: return ImGuiKey_RightCtrl;
    case GLFW_KEY_LEFT_SHIFT:    return ImGuiKey_LeftShift;
    case GLFW_KEY_RIGHT_SHIFT:   return ImGuiKey_RightShift;
    case GLFW_KEY_LEFT_ALT:      return ImGuiKey_LeftAlt;
    case GLFW_KEY_RIGHT_ALT:     return ImGuiKey_RightAlt;
    case GLFW_KEY_LEFT_SUPER:    return ImGuiKey_LeftSuper;
    case GLFW_KEY_RIGHT_SUPER:   return ImGuiKey_RightSuper;
    default:                     return ImGuiKey_None;
    }
}

/**
 * @brief add_modifiers gives ImGui the state of the modifier keys (GLFW_MOD_* flags).
 */
void add_modifiers(ImGuiIO& io, const int& glfw_mods)
{
    io.AddKeyEvent(ImGuiMod_Ctrl, (glfw_mods & GLFW_MOD_CONTROL) != 0);
    io.AddKeyEvent(ImGuiMod_Shift, (glfw_mods & GLFW_MOD_SHIFT) != 0);
    io.AddKeyEvent(ImGuiMod_Alt, (glfw_mods & GLFW_MOD_ALT) != 0);
    io.AddKeyEvent(ImGuiMod_Super, (glfw_mods & GLFW_MOD_SUPER) != 0);
}

std::string body_label(const mjModel* m, const int& body)
{
    const char* name = mj_id2name(m, mjOBJ_BODY, body);
    return (name && name[0]) ? name : "(body " + std::to_string(body) + ")";
}

std::string object_label(const mjModel* m, const mjtObj& type, const int& id, const char* kind)
{
    const char* name = mj_id2name(m, type, id);
    return (name && name[0]) ? name : "(" + std::string(kind) + " " + std::to_string(id) + ")";
}

bool contains_case_insensitive(const std::string& text, const char* pattern)
{
    auto lower = [](unsigned char c) { return static_cast<char>(std::tolower(c)); };
    std::string a(text), b(pattern);
    std::transform(a.begin(), a.end(), a.begin(), lower);
    std::transform(b.begin(), b.end(), b.begin(), lower);
    return a.find(b) != std::string::npos;
}

bool has_free_joint(const mjModel* m, const int& body)
{
    for (int j = m->body_jntadr[body]; j >= 0 && j < m->body_jntadr[body] + m->body_jntnum[body]; j++)
        if (m->jnt_type[j] == mjJNT_FREE)
            return true;
    return false;
}

enum class Icon { PLAY, PAUSE, STEP, RESET, SELECT, MOVE, ROTATE };

/**
 * @brief icon_button draws a button with a vector icon (the font has no icons).
 */
bool icon_button(const char* id, const Icon& icon, const char* tooltip, const bool& enabled = true,
                 const bool& selected = false)
{
    const float h = ImGui::GetFrameHeight();
    ImGui::BeginDisabled(!enabled);
    const bool pressed = ImGui::InvisibleButton(id, ImVec2(h * 1.4f, h));
    const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
    ImGui::EndDisabled();

    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 a = ImGui::GetItemRectMin();
    const ImVec2 b = ImGui::GetItemRectMax();
    if (selected)
        draw->AddRectFilled(a, b, ImGui::GetColorU32(ImGuiCol_Header), ImGui::GetStyle().FrameRounding);
    else if (hovered && enabled)
        draw->AddRectFilled(a, b, ImGui::GetColorU32(ImGui::IsItemActive() ? ImGuiCol_ButtonActive
                                                                           : ImGuiCol_ButtonHovered),
                            ImGui::GetStyle().FrameRounding);
    const ImU32 col = ImGui::GetColorU32(enabled ? ImGuiCol_Text : ImGuiCol_TextDisabled);
    const ImVec2 c = (a + b) * 0.5f;
    const float r = h * 0.26f;
    switch (icon)
    {
    case Icon::PLAY:
        draw->AddTriangleFilled(c + ImVec2(-0.75f * r, -r), c + ImVec2(-0.75f * r, r), c + ImVec2(r, 0), col);
        break;
    case Icon::PAUSE:
        draw->AddRectFilled(c + ImVec2(-0.8f * r, -r), c + ImVec2(-0.25f * r, r), col, 1);
        draw->AddRectFilled(c + ImVec2(0.25f * r, -r), c + ImVec2(0.8f * r, r), col, 1);
        break;
    case Icon::STEP:
        draw->AddTriangleFilled(c + ImVec2(-r, -r), c + ImVec2(-r, r), c + ImVec2(0.55f * r, 0), col);
        draw->AddRectFilled(c + ImVec2(0.55f * r, -r), c + ImVec2(r, r), col, 1);
        break;
    case Icon::RESET:
    {
        const float thickness = std::max(1.5f, 0.28f * r);
        draw->PathArcTo(c, 0.85f * r, IM_PI * 0.15f, IM_PI * 1.75f, 20);
        draw->PathStroke(col, thickness);
        // Arrow head at the start of the arc, pointing along it
        const ImVec2 tip = c + ImVec2(std::cos(IM_PI * 0.15f), std::sin(IM_PI * 0.15f)) * (0.85f * r);
        draw->AddTriangleFilled(tip + ImVec2(-0.6f * r, -0.05f * r), tip + ImVec2(0.45f * r, -0.35f * r),
                                tip + ImVec2(0.25f * r, 0.6f * r), col);
        break;
    }
    case Icon::SELECT:  // a mouse pointer
    {
        const ImVec2 points[7] = {c + ImVec2(-0.65f * r, -1.05f * r), c + ImVec2(-0.65f * r, 0.75f * r),
                                  c + ImVec2(-0.2f * r, 0.3f * r),   c + ImVec2(0.15f * r, 1.05f * r),
                                  c + ImVec2(0.45f * r, 0.9f * r),   c + ImVec2(0.1f * r, 0.15f * r),
                                  c + ImVec2(0.75f * r, 0.15f * r)};
        draw->AddConcavePolyFilled(points, 7, col);
        break;
    }
    case Icon::MOVE:  // four arrows
    {
        const float t = std::max(1.5f, 0.22f * r);
        draw->AddLine(c + ImVec2(-0.75f * r, 0), c + ImVec2(0.75f * r, 0), col, t);
        draw->AddLine(c + ImVec2(0, -0.75f * r), c + ImVec2(0, 0.75f * r), col, t);
        const float l = 1.1f * r, w = 0.38f * r, s = 0.62f * r;
        draw->AddTriangleFilled(c + ImVec2(l, 0), c + ImVec2(s, -w), c + ImVec2(s, w), col);
        draw->AddTriangleFilled(c + ImVec2(-l, 0), c + ImVec2(-s, w), c + ImVec2(-s, -w), col);
        draw->AddTriangleFilled(c + ImVec2(0, -l), c + ImVec2(w, -s), c + ImVec2(-w, -s), col);
        draw->AddTriangleFilled(c + ImVec2(0, l), c + ImVec2(-w, s), c + ImVec2(w, s), col);
        break;
    }
    case Icon::ROTATE:  // an orbit with an arrow
    {
        const float t = std::max(1.5f, 0.22f * r);
        draw->AddEllipse(c, ImVec2(1.0f * r, 0.5f * r), col, 0.0f, 24, t);
        draw->AddCircleFilled(c, 0.22f * r, col);
        const ImVec2 tip = c + ImVec2(0.15f * r, 0.5f * r);
        draw->AddTriangleFilled(tip + ImVec2(0.35f * r, 0), tip + ImVec2(-0.15f * r, -0.32f * r),
                                tip + ImVec2(-0.15f * r, 0.32f * r), col);
        break;
    }
    }
    if (hovered && tooltip)
        ImGui::SetTooltip("%s", tooltip);
    return pressed && enabled;
}

/**
 * @brief vector_row shows a labeled row of read-only values, each with a colored axis tag.
 */
void vector_row(const char* label, const mjtNum* values, const int& n, const char* const* names,
                const ImU32* colors)
{
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    const float label_width = ImGui::GetFontSize() * 6.0f;
    ImGui::SameLine(label_width);
    const float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
    const float width = std::max(10.0f, (ImGui::GetContentRegionAvail().x - spacing * (n - 1)) / n);
    const float h = ImGui::GetFrameHeight();
    const float rounding = ImGui::GetStyle().FrameRounding;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    for (int i = 0; i < n; i++)
    {
        if (i > 0)
            ImGui::SameLine(0, spacing);
        const ImVec2 p = ImGui::GetCursorScreenPos();
        const float tag = h * 0.9f;
        draw->AddRectFilled(p, p + ImVec2(width, h), ImGui::GetColorU32(ImGuiCol_FrameBg), rounding);
        draw->AddRectFilled(p, p + ImVec2(tag, h), colors[i], rounding, ImDrawFlags_RoundLeft);
        const ImVec2 name_size = ImGui::CalcTextSize(names[i]);
        draw->AddText(p + ImVec2((tag - name_size.x) * 0.5f, (h - name_size.y) * 0.5f),
                      IM_COL32(255, 255, 255, 235), names[i]);
        char text[32];
        std::snprintf(text, sizeof(text), "%.4f", values[i]);
        const ImVec2 text_size = ImGui::CalcTextSize(text);
        draw->PushClipRect(p + ImVec2(tag, 0), p + ImVec2(width, h), true);
        draw->AddText(p + ImVec2(tag + ImGui::GetStyle().FramePadding.x, (h - text_size.y) * 0.5f),
                      ImGui::GetColorU32(ImGuiCol_Text), text);
        draw->PopClipRect();
        ImGui::Dummy(ImVec2(width, h));
    }
}

//--------------------------------------------------------------------------------------------------
// Matrices for ImGuizmo: 4x4, column-major (OpenGL)
//--------------------------------------------------------------------------------------------------

/**
 * @brief camera_matrices computes the view and projection matrices of the camera of the 3D view,
 *        as mjr_render() sets them.
 */
void camera_matrices(const mjvGLCamera& camera, const float& aspect_ratio, float view[16], float projection[16])
{
    const float* f = camera.forward;
    const float* u = camera.up;
    const float* p = camera.pos;
    const float r[3] = {f[1] * u[2] - f[2] * u[1], f[2] * u[0] - f[0] * u[2], f[0] * u[1] - f[1] * u[0]};
    auto dot = [](const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    const float v[16] = {r[0], u[0], -f[0], 0,
                         r[1], u[1], -f[1], 0,
                         r[2], u[2], -f[2], 0,
                         -dot(r, p), -dot(u, p), dot(f, p), 1};
    std::copy(v, v + 16, view);

    // mjr_render() matches the width of the frustum to the aspect ratio of the viewport
    const float half_width = camera.frustum_width ? camera.frustum_width
                                                  : 0.5f * aspect_ratio * (camera.frustum_top - camera.frustum_bottom);
    const float l = camera.frustum_center - half_width, rt = camera.frustum_center + half_width;
    const float b = camera.frustum_bottom, t = camera.frustum_top;
    const float n = camera.frustum_near, fr = camera.frustum_far;
    std::fill(projection, projection + 16, 0.0f);
    if (camera.orthographic)
    {
        projection[0] = 2 / (rt - l);
        projection[5] = 2 / (t - b);
        projection[10] = -2 / (fr - n);
        projection[12] = -(rt + l) / (rt - l);
        projection[13] = -(t + b) / (t - b);
        projection[14] = -(fr + n) / (fr - n);
        projection[15] = 1;
    }
    else
    {
        projection[0] = 2 * n / (rt - l);
        projection[5] = 2 * n / (t - b);
        projection[8] = (rt + l) / (rt - l);
        projection[9] = (t + b) / (t - b);
        projection[10] = -(fr + n) / (fr - n);
        projection[11] = -1;
        projection[14] = -2 * fr * n / (fr - n);
    }
}

void pose_to_matrix(const mjtNum pos[3], const mjtNum quat[4], float matrix[16])
{
    mjtNum rotation[9];  // row-major
    mju_quat2Mat(rotation, quat);
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
            matrix[4 * col + row] = static_cast<float>(rotation[3 * row + col]);
        matrix[12 + row] = static_cast<float>(pos[row]);
        matrix[4 * row + 3] = 0;
    }
    matrix[15] = 1;
}

void matrix_to_pose(const float matrix[16], mjtNum pos[3], mjtNum quat[4])
{
    mjtNum rotation[9];
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
            rotation[3 * row + col] = matrix[4 * col + row];
        pos[row] = matrix[12 + row];
    }
    mju_mat2Quat(quat, rotation);
    mju_normalize4(quat);
}

/**
 * @brief status_dot draws a filled circle aligned with the current line of text.
 */
void status_dot(const ImU32& col)
{
    const float r = ImGui::GetFontSize() * 0.28f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float line = ImGui::GetTextLineHeight();
    ImGui::GetWindowDrawList()->AddCircleFilled(p + ImVec2(r, line * 0.5f), r, col);
    ImGui::Dummy(ImVec2(2 * r, line));
}
}  // namespace


//--------------------------------------------------------------------------------------------------
// State
//--------------------------------------------------------------------------------------------------

MujocoGui::Movable MujocoGui::movable(const mjModel* m, const int& body)
{
    if (body <= 0 || body >= m->nbody)
        return Movable::NONE;
    if (m->body_mocapid[body] >= 0)
        return Movable::KINEMATIC;
    if (m->body_jntnum[body] == 1 && m->jnt_type[m->body_jntadr[body]] == mjJNT_FREE)
        return Movable::FREE;
    if (m->body_parentid[body] == 0 && m->body_jntnum[body] == 0)
        return Movable::FIXED;
    return Movable::NONE;
}

void MujocoGui::State::copy(const mjModel* m, const mjData* d)
{
    time = d->time;
    qpos.assign(d->qpos, d->qpos + m->nq);
    qvel.assign(d->qvel, d->qvel + m->nv);
    xpos.assign(d->xpos, d->xpos + 3 * m->nbody);
    xquat.assign(d->xquat, d->xquat + 4 * m->nbody);
    sensordata.assign(d->sensordata, d->sensordata + m->nsensordata);
}


//--------------------------------------------------------------------------------------------------
// Construction
//--------------------------------------------------------------------------------------------------

MujocoGui::MujocoGui(const float& content_scale)
    : content_scale_(content_scale > 0 ? content_scale : 1.0f),
      last_frame_(std::chrono::steady_clock::now())
{
    IMGUI_CHECKVERSION();
    ImGuiContext* previous = ImGui::GetCurrentContext();
    context_ = ImGui::CreateContext();
    ImGui::SetCurrentContext(context_);

    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;  // the library writes no files: the layout starts as the default one
    io.LogFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigWindowsMoveFromTitleBarOnly = true;
    io.BackendPlatformName = "dq_mujoco_glfw";

    ImFontConfig font;
    font.FontDataOwnedByAtlas = false;  // static data
    io.Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(dq_mujoco_gui_font),
                                   static_cast<int>(dq_mujoco_gui_font_size), FONT_SIZE, &font);
    _apply_style();

    ImGui::SetCurrentContext(previous);
}

MujocoGui::~MujocoGui()
{
    ImGui::SetCurrentContext(context_);
    if (renderer_initialized_)
    {
        std::lock_guard<std::mutex> lock(opengl_backend_mutex());
        ImGui_ImplOpenGL3_Shutdown();
    }
    ImGui::DestroyContext(context_);
}

void MujocoGui::_apply_style()
{
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowPadding = ImVec2(10, 8);
    style.FramePadding = ImVec2(6, 4);
    style.ItemSpacing = ImVec2(8, 5);
    style.ItemInnerSpacing = ImVec2(5, 4);
    style.IndentSpacing = 14;
    style.ScrollbarSize = 11;
    style.GrabMinSize = 9;
    style.WindowBorderSize = 1;
    style.ChildBorderSize = 0;
    style.PopupBorderSize = 1;
    style.FrameBorderSize = 0;
    style.TabBorderSize = 0;
    style.TabBarOverlineSize = 2;
    style.WindowRounding = 0;
    style.ChildRounding = 2;
    style.FrameRounding = 3;
    style.PopupRounding = 3;
    style.ScrollbarRounding = 6;
    style.GrabRounding = 2;
    style.TabRounding = 2;
    style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
    style.WindowMenuButtonPosition = ImGuiDir_None;
    style.SeparatorTextBorderSize = 1;
    style.SeparatorTextPadding = ImVec2(0, 3);
    style.DockingSeparatorSize = 2;
    style.DockingNodeHasCloseButton = false;  // each tab has its own
    style.TreeLinesFlags = ImGuiTreeNodeFlags_DrawLinesToNodes;
    style.TreeLinesSize = 1;

    ImVec4* c = style.Colors;
    c[ImGuiCol_Text]                      = color(0xD5D8DC);
    c[ImGuiCol_TextDisabled]              = color(0x80858D);
    c[ImGuiCol_WindowBg]                  = color(BACKGROUND);
    c[ImGuiCol_ChildBg]                   = color(BACKGROUND, 0.0f);
    c[ImGuiCol_PopupBg]                   = color(0x26292E, 0.98f);
    c[ImGuiCol_Border]                    = color(0x33363C);
    c[ImGuiCol_BorderShadow]              = color(0x000000, 0.0f);
    c[ImGuiCol_FrameBg]                   = color(0x2A2D32);
    c[ImGuiCol_FrameBgHovered]            = color(0x32363C);
    c[ImGuiCol_FrameBgActive]             = color(0x393E45);
    c[ImGuiCol_TitleBg]                   = color(0x17181B);
    c[ImGuiCol_TitleBgActive]             = color(0x17181B);
    c[ImGuiCol_TitleBgCollapsed]          = color(0x17181B);
    c[ImGuiCol_MenuBarBg]                 = color(0x17181B);
    c[ImGuiCol_ScrollbarBg]               = color(BACKGROUND, 0.0f);
    c[ImGuiCol_ScrollbarGrab]             = color(0x3A3E45);
    c[ImGuiCol_ScrollbarGrabHovered]      = color(0x464B53);
    c[ImGuiCol_ScrollbarGrabActive]       = color(0x50565F);
    c[ImGuiCol_CheckMark]                 = color(ACCENT);
    c[ImGuiCol_SliderGrab]                = color(ACCENT, 0.8f);
    c[ImGuiCol_SliderGrabActive]          = color(ACCENT);
    c[ImGuiCol_Button]                    = color(0x2D3035);
    c[ImGuiCol_ButtonHovered]             = color(0x383C42);
    c[ImGuiCol_ButtonActive]              = color(0x2F5F86);
    c[ImGuiCol_Header]                    = color(0x2C4F70);
    c[ImGuiCol_HeaderHovered]             = color(0x2E3238);
    c[ImGuiCol_HeaderActive]              = color(0x33597E);
    c[ImGuiCol_Separator]                 = color(0x33363C);
    c[ImGuiCol_SeparatorHovered]          = color(ACCENT, 0.6f);
    c[ImGuiCol_SeparatorActive]           = color(ACCENT);
    c[ImGuiCol_ResizeGrip]                = color(0x3A3E45, 0.0f);
    c[ImGuiCol_ResizeGripHovered]         = color(ACCENT, 0.6f);
    c[ImGuiCol_ResizeGripActive]          = color(ACCENT);
    c[ImGuiCol_InputTextCursor]           = color(0xD5D8DC);
    c[ImGuiCol_Tab]                       = color(0x17181B);
    c[ImGuiCol_TabHovered]                = color(0x2E3238);
    c[ImGuiCol_TabSelected]               = color(BACKGROUND);
    c[ImGuiCol_TabSelectedOverline]       = color(ACCENT);
    c[ImGuiCol_TabDimmed]                 = color(0x17181B);
    c[ImGuiCol_TabDimmedSelected]         = color(BACKGROUND);
    c[ImGuiCol_TabDimmedSelectedOverline] = color(0x3A3E45);
    c[ImGuiCol_DockingPreview]            = color(ACCENT, 0.45f);
    c[ImGuiCol_DockingEmptyBg]            = color(BACKGROUND);
    c[ImGuiCol_PlotLines]                 = color(ACCENT);
    c[ImGuiCol_PlotLinesHovered]          = color(0x7FC0F0);
    c[ImGuiCol_PlotHistogram]             = color(0x3A6E99);
    c[ImGuiCol_PlotHistogramHovered]      = color(ACCENT);
    c[ImGuiCol_TableHeaderBg]             = color(0x24272B);
    c[ImGuiCol_TableBorderStrong]         = color(0x33363C);
    c[ImGuiCol_TableBorderLight]          = color(0x2A2D32);
    c[ImGuiCol_TableRowBg]                = color(0x000000, 0.0f);
    c[ImGuiCol_TableRowBgAlt]             = color(0xFFFFFF, 0.025f);
    c[ImGuiCol_TextLink]                  = color(ACCENT);
    c[ImGuiCol_TextSelectedBg]            = color(ACCENT, 0.35f);
    c[ImGuiCol_TreeLines]                 = color(0x3A3E45);
    c[ImGuiCol_DragDropTarget]            = color(ACCENT);
    c[ImGuiCol_NavCursor]                 = color(ACCENT);
    c[ImGuiCol_ModalWindowDimBg]          = color(0x000000, 0.5f);

    // High-DPI displays: larger fonts and sizes
    style.ScaleAllSizes(content_scale_);
    style.FontScaleDpi = content_scale_;
    style.FontSizeBase = FONT_SIZE;
}


//--------------------------------------------------------------------------------------------------
// Main thread: events and requests
//--------------------------------------------------------------------------------------------------

void MujocoGui::_push(const Event& event)
{
    std::lock_guard<std::mutex> lock(events_mutex_);
    // The events are only dropped if no frame is drawn for a long time (e.g., the GUI is hidden)
    if (events_.size() > 4096)
        events_.clear();
    events_.push_back(event);
}

void MujocoGui::on_mouse_button(const int& button, const bool& down, const int& glfw_mods)
{
    _push({Event::Type::MOUSE_BUTTON, button, down ? 1 : 0, static_cast<double>(glfw_mods), 0});
}

void MujocoGui::on_mouse_move(const double& x, const double& y)
{
    _push({Event::Type::MOUSE_MOVE, 0, 0, x, y});
}

void MujocoGui::on_scroll(const double& dx, const double& dy)
{
    _push({Event::Type::SCROLL, 0, 0, dx, dy});
}

void MujocoGui::on_key(const int& glfw_key, const int& glfw_mods, const bool& down)
{
    _push({Event::Type::KEY, glfw_key, glfw_mods, down ? 1.0 : 0.0, 0});
}

void MujocoGui::on_char(const unsigned int& codepoint)
{
    _push({Event::Type::CHAR, static_cast<int>(codepoint), 0, 0, 0});
}

void MujocoGui::on_focus(const bool& focused)
{
    _push({Event::Type::FOCUS, focused ? 1 : 0, 0, 0, 0});
}

MujocoGui::Requests MujocoGui::take_requests()
{
    std::lock_guard<std::mutex> lock(requests_mutex_);
    Requests requests = requests_;
    requests_ = Requests();
    return requests;
}

void MujocoGui::_request(const Requests& request)
{
    std::lock_guard<std::mutex> lock(requests_mutex_);
    if (request.toggle_simulation)
        requests_.toggle_simulation = !requests_.toggle_simulation;
    if (request.step)
        requests_.step = true;
    if (request.reset)
    {
        requests_.reset = true;
        requests_.reset_keyframe = request.reset_keyframe;
    }
}


//--------------------------------------------------------------------------------------------------
// Viewer thread: frames
//--------------------------------------------------------------------------------------------------

void MujocoGui::_process_events()
{
    std::vector<Event> events;
    {
        std::lock_guard<std::mutex> lock(events_mutex_);
        events.swap(events_);
    }
    ImGuiIO& io = ImGui::GetIO();
    for (const Event& e : events)
    {
        switch (e.type)
        {
        case Event::Type::MOUSE_BUTTON:
            // The modifiers first: Ctrl + drag is the perturbation of a body
            add_modifiers(io, static_cast<int>(e.x));
            // GLFW and ImGui number the left, right and middle buttons alike
            if (e.a >= 0 && e.a < ImGuiMouseButton_COUNT)
                io.AddMouseButtonEvent(e.a, e.b != 0);
            break;
        case Event::Type::MOUSE_MOVE:
            io.AddMousePosEvent(static_cast<float>(e.x), static_cast<float>(e.y));
            break;
        case Event::Type::SCROLL:
            io.AddMouseWheelEvent(static_cast<float>(e.x), static_cast<float>(e.y));
            break;
        case Event::Type::KEY:
        {
            // X11 gives the modifiers before the key: a modifier key changes its own (as in
            // imgui_impl_glfw)
            int mods = e.b;
            const bool down = e.x != 0;
            auto set = [&](const int& flag) { mods = down ? (mods | flag) : (mods & ~flag); };
            if (e.a == GLFW_KEY_LEFT_CONTROL || e.a == GLFW_KEY_RIGHT_CONTROL)
                set(GLFW_MOD_CONTROL);
            else if (e.a == GLFW_KEY_LEFT_SHIFT || e.a == GLFW_KEY_RIGHT_SHIFT)
                set(GLFW_MOD_SHIFT);
            else if (e.a == GLFW_KEY_LEFT_ALT || e.a == GLFW_KEY_RIGHT_ALT)
                set(GLFW_MOD_ALT);
            else if (e.a == GLFW_KEY_LEFT_SUPER || e.a == GLFW_KEY_RIGHT_SUPER)
                set(GLFW_MOD_SUPER);
            add_modifiers(io, mods);
            const ImGuiKey key = glfw_to_imgui_key(e.a);
            if (key != ImGuiKey_None)
                io.AddKeyEvent(key, e.x != 0);
            break;
        }
        case Event::Type::CHAR:
            io.AddInputCharacter(static_cast<unsigned int>(e.a));
            break;
        case Event::Type::FOCUS:
            io.AddFocusEvent(e.a != 0);
            break;
        }
    }
}

mjrRect MujocoGui::begin_frame(const Frame& frame)
{
    ImGui::SetCurrentContext(context_);
    std::unique_lock<std::mutex> backend_lock(opengl_backend_mutex());
    if (!renderer_initialized_)
    {
#ifdef __APPLE__
        ImGui_ImplOpenGL3_Init("#version 120");  // MuJoCo needs a legacy (OpenGL 2.1) context on macOS
#else
        ImGui_ImplOpenGL3_Init(nullptr);
#endif
        renderer_initialized_ = true;
    }

    ImGuiIO& io = ImGui::GetIO();
    const int window_width = std::max(1, frame.window_width);
    const int window_height = std::max(1, frame.window_height);
    io.DisplaySize = ImVec2(static_cast<float>(window_width), static_cast<float>(window_height));
    const float scale_x = static_cast<float>(frame.framebuffer_width) / window_width;
    const float scale_y = static_cast<float>(frame.framebuffer_height) / window_height;
    io.DisplayFramebufferScale = ImVec2(scale_x, scale_y);
    const auto now = std::chrono::steady_clock::now();
    io.DeltaTime = std::max(1e-4f, std::chrono::duration<float>(now - last_frame_).count());
    last_frame_ = now;
    _process_events();

    ImGui_ImplOpenGL3_NewFrame();
    backend_lock.unlock();
    ImGui::NewFrame();
    ImGuizmo::BeginFrame();

    // The bars first: the dock space fills the rest of the window
    _shortcuts(frame);
    _menu_bar(frame);
    _status_bar(frame);
    const ImGuiID dockspace_id = ImHashStr("DQMujocoDockSpace");
    if (reset_layout_)
    {
        _dock_layout(dockspace_id);
        reset_layout_ = false;
    }
    ImGui::DockSpaceOverViewport(dockspace_id, ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);

    if (selected_body_ >= frame.m->nbody)
        selected_body_ = -1;  // the scene was edited
    _scene_panel(frame);
    _properties_panel(frame);
    _joints_panel(frame);
    _sensors_panel(frame);
    _visualization_panel(frame);
    wants_mouse_ = io.WantCaptureMouse;

    // The 3D view is drawn in the free (central) area of the dock space
    ImRect view_rect(ImVec2(0, 0), io.DisplaySize);
    if (ImGuiDockNode* central = ImGui::DockBuilderGetCentralNode(dockspace_id))
        if (central->Size.x > 0 && central->Size.y > 0)
            view_rect = ImRect(central->Pos, central->Pos + central->Size);

    // The gizmo, the navigation gizmo and the toolbar, over the 3D view
    const float overlay_rect[4] = {view_rect.Min.x, view_rect.Min.y, view_rect.Max.x, view_rect.Max.y};
    _viewport_overlay(frame, overlay_rect);
    _viewport_toolbar(overlay_rect);
    wants_mouse_ = io.WantCaptureMouse || gizmo_hovered_;

    // A click in the 3D view selects the body under the cursor. Only a click: a drag moves the
    // camera, and a press on a panel (or the gizmo) is for the panel.
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        click_in_view_ = !io.WantCaptureMouse && !gizmo_hovered_ && !io.KeyCtrl && view_rect.Contains(io.MousePos);
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
    {
        if (click_in_view_ && io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Left]
                                  < io.MouseDragThreshold * io.MouseDragThreshold)
        {
            const ImVec2 p = io.MouseClickedPos[ImGuiMouseButton_Left];
            pick_.relx = (p.x - view_rect.Min.x) / view_rect.GetWidth();
            pick_.rely = (view_rect.Max.y - p.y) / view_rect.GetHeight();  // from the bottom
            pick_.aspect_ratio = view_rect.GetWidth() / view_rect.GetHeight();
            pick_requested_ = true;
        }
        click_in_view_ = false;
    }

    // Ctrl + drag in the 3D view: MuJoCo's perturbation of the selected body
    perturb_ = PerturbInput();
    if (perturb_button_ < 0)
    {
        if (selected_body_ > 0 && io.KeyCtrl && !io.WantCaptureMouse && !gizmo_hovered_
            && view_rect.Contains(io.MousePos))
        {
            for (const ImGuiMouseButton button : {ImGuiMouseButton_Left, ImGuiMouseButton_Right})
            {
                if (!ImGui::IsMouseClicked(button))
                    continue;
                perturb_button_ = button;
                perturb_.start = true;
                perturb_.body = selected_body_;
                perturb_.active = button == ImGuiMouseButton_Left ? mjPERT_ROTATE : mjPERT_TRANSLATE;
                // Held at the clicked point, if the body was selected in the 3D view, or at its origin
                if (picked_body_ == selected_body_)
                    mju_copy3(perturb_.localpos, picked_point_);
                break;
            }
        }
    }
    else if (!ImGui::IsMouseDown(perturb_button_) || selected_body_ <= 0)
    {
        perturb_.end = true;
        perturb_button_ = -1;
    }
    else if (ImGui::IsMousePosValid() && (io.MouseDelta.x != 0 || io.MouseDelta.y != 0))
    {
        if (perturb_button_ == ImGuiMouseButton_Right)
            perturb_.action = io.KeyShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else
            perturb_.action = io.KeyShift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        perturb_.dx = io.MouseDelta.x / view_rect.GetHeight();
        perturb_.dy = io.MouseDelta.y / view_rect.GetHeight();
    }
    has_selection_ = selected_body_ > 0;

    // In pixels, for mjr_render(). OpenGL counts the rows from the bottom.
    mjrRect view = {static_cast<int>(view_rect.Min.x * scale_x),
                    frame.framebuffer_height - static_cast<int>(view_rect.Max.y * scale_y),
                    static_cast<int>(view_rect.GetWidth() * scale_x),
                    static_cast<int>(view_rect.GetHeight() * scale_y)};

    // Clear the whole window: mjr_render() only draws the 3D view
    const ImVec4 background = color(BACKGROUND);
    glViewport(0, 0, frame.framebuffer_width, frame.framebuffer_height);
    glDisable(GL_SCISSOR_TEST);
    glClearColor(background.x, background.y, background.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    return view;
}

void MujocoGui::render()
{
    ImGui::SetCurrentContext(context_);
    ImGui::Render();
    std::lock_guard<std::mutex> lock(opengl_backend_mutex());
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void MujocoGui::select_body(const int& body, const mjtNum local_point[3])
{
    selected_body_ = body > 0 ? body : -1;  // the world (e.g., the floor) clears the selection
    reveal_selection_ = selected_body_ > 0;
    has_selection_ = selected_body_ > 0;
    picked_body_ = selected_body_;
    mju_copy3(picked_point_, local_point);
}

bool MujocoGui::take_pick(Pick& pick)
{
    if (!pick_requested_)
        return false;
    pick = pick_;
    pick_requested_ = false;
    return true;
}

void MujocoGui::highlight_selection(const mjModel* m, mjvScene& scn) const
{
    const int root = selected_body_;
    if (root <= 0 || root >= m->nbody)
        return;
    const ImVec4 highlight = color(SELECTION);
    for (int i = 0; i < scn.ngeom; i++)
    {
        mjvGeom& geom = scn.geoms[i];
        if (geom.objtype != mjOBJ_GEOM || geom.objid < 0 || geom.objid >= m->ngeom)
            continue;  // decorations: frames, contacts, labels, ...
        // MuJoCo numbers the bodies in tree order (a parent before its children)
        int body = m->geom_bodyid[geom.objid];
        while (body > root)
            body = m->body_parentid[body];
        if (body != root)
            continue;
        // Mostly the selection color, so that the hue does not mix with that of the geom (blue and
        // amber would make gray), with some of the geom's color to keep its shading and texture
        geom.emission += 0.2f;
        geom.rgba[0] = 0.35f * geom.rgba[0] + 0.65f * highlight.x;
        geom.rgba[1] = 0.35f * geom.rgba[1] + 0.65f * highlight.y;
        geom.rgba[2] = 0.35f * geom.rgba[2] + 0.65f * highlight.z;
    }
}

void MujocoGui::_viewport_overlay(const Frame& frame, const float view_rect[4])
{
    gizmo_ = GizmoInput();
    gizmo_hovered_ = false;
    const mjModel* m = frame.m;
    const ImVec2 origin(view_rect[0], view_rect[1]);
    const ImVec2 size(view_rect[2] - view_rect[0], view_rect[3] - view_rect[1]);
    if (size.x <= 0 || size.y <= 0)
        return;

    // The camera of the 3D view
    const mjvGLCamera camera = mjv_averageCamera(frame.scn.camera, frame.scn.camera + 1);
    float view[16], projection[16];
    camera_matrices(camera, size.x / size.y, view, projection);

    // ImGuizmo draws in a transparent window over the 3D view (below the panels)
    ImGui::SetNextWindowPos(origin);
    ImGui::SetNextWindowSize(size);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs
                                   | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoSavedSettings
                                   | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoBringToFrontOnFocus
                                   | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoNav;
    ImGui::Begin("##viewport_overlay", nullptr, flags);
    ImGuizmo::SetDrawlist();
    ImGuizmo::SetRect(origin.x, origin.y, size.x, size.y);
    ImGuizmo::SetOrthographic(camera.orthographic != 0);
    ImGuizmo::AllowAxisFlip(false);
    ImGuizmo::SetGizmoSizeClipSpace(0.16f);
    ImGuizmo::Style& style = ImGuizmo::GetStyle();
    style.TranslationLineThickness = 3.0f * content_scale_;
    style.TranslationLineArrowSize = 7.0f * content_scale_;
    style.RotationLineThickness = 3.0f * content_scale_;
    style.RotationOuterLineThickness = 2.5f * content_scale_;
    const ImVec4 x_color = ImGui::ColorConvertU32ToFloat4(AXIS_COLORS[0]);
    const ImVec4 y_color = ImGui::ColorConvertU32ToFloat4(AXIS_COLORS[1]);
    const ImVec4 z_color = ImGui::ColorConvertU32ToFloat4(AXIS_COLORS[2]);
    style.Colors[ImGuizmo::DIRECTION_X] = style.Colors[ImGuizmo::PLANE_X] = x_color;
    style.Colors[ImGuizmo::DIRECTION_Y] = style.Colors[ImGuizmo::PLANE_Y] = y_color;
    style.Colors[ImGuizmo::DIRECTION_Z] = style.Colors[ImGuizmo::PLANE_Z] = z_color;
    style.Colors[ImGuizmo::SELECTION] = color(SELECTION);

    // The gizmo of the selected body, if the gizmo can move it (not while it is dragged with Ctrl)
    const int b = selected_body_;
    const bool has_pose = b > 0 && 3 * b + 2 < static_cast<int>(frame.state.xpos.size());
    const bool show_gizmo = tool_ != Tool::SELECT && has_pose && perturb_button_ < 0
                            && movable(m, b) != Movable::NONE;
    bool dragging = false;
    if (show_gizmo)
    {
        float matrix[16];
        pose_to_matrix(frame.state.xpos.data() + 3 * b, frame.state.xquat.data() + 4 * b, matrix);
        ImGuizmo::PushID(b);
        if (ImGuizmo::Manipulate(view, projection, tool_ == Tool::MOVE ? ImGuizmo::TRANSLATE : ImGuizmo::ROTATE,
                                 gizmo_local_ ? ImGuizmo::LOCAL : ImGuizmo::WORLD, matrix))
        {
            gizmo_.moved = true;
            gizmo_.body = b;
            matrix_to_pose(matrix, gizmo_.pos, gizmo_.quat);
        }
        dragging = ImGuizmo::IsUsing();
        gizmo_hovered_ = ImGuizmo::IsOver() || dragging;
        ImGuizmo::PopID();
    }
    if (dragging)
        gizmo_body_ = b;
    else if (gizmo_dragging_)
    {
        gizmo_.end = true;
        gizmo_.body = gizmo_body_;
    }
    gizmo_dragging_ = dragging;
    ImGui::End();

    // The navigation gizmo (ImOGuizmo), in the top-right corner: the axes of the world, as in
    // Blender. Click an axis to look along it, or drag it to turn the camera (not for the cameras of
    // the model, which are fixed).
    if (show_navigation_ && frame.cam.type != mjCAMERA_FIXED)
    {
        const float gizmo_size = 120.0f * content_scale_;
        const ImVec2 position(origin.x + size.x - gizmo_size - 4 * content_scale_, origin.y + 4 * content_scale_);
        ImGui::SetNextWindowPos(position);
        ImGui::SetNextWindowSize(ImVec2(gizmo_size, gizmo_size));
        ImGui::Begin("##navigation", nullptr, flags);
        ImOGuizmo::SetRect(position.x, position.y, gizmo_size);
        ImOGuizmo::SetDrawList();
        ImOGuizmo::config.positiveRadiusScale = 0.095f;  // the labels (X, Y, Z) fit in the circles
        ImOGuizmo::config.negativeRadiusScale = 0.06f;
        ImOGuizmo::config.lineThicknessScale = 0.02f;
        auto darker = [](const ImU32& c) {
            const ImVec4 v = ImGui::ColorConvertU32ToFloat4(c);
            return ImGui::GetColorU32(ImVec4(0.55f * v.x, 0.55f * v.y, 0.55f * v.z, 1.0f));
        };
        ImOGuizmo::config.xCircleFrontColor = AXIS_COLORS[0];
        ImOGuizmo::config.yCircleFrontColor = AXIS_COLORS[1];
        ImOGuizmo::config.zCircleFrontColor = AXIS_COLORS[2];
        ImOGuizmo::config.xCircleBackColor = darker(AXIS_COLORS[0]);
        ImOGuizmo::config.yCircleBackColor = darker(AXIS_COLORS[1]);
        ImOGuizmo::config.zCircleBackColor = darker(AXIS_COLORS[2]);

        // It reacts to a release over an axis: only if the press was on it (not after a drag of the
        // camera that ends there)
        const ImGuiIO& io = ImGui::GetIO();
        const ImVec2 center = position + ImVec2(gizmo_size, gizmo_size) * 0.5f;
        const float radius = 0.5f * gizmo_size * ImOGuizmo::config.hoverCircleRadiusScale;
        const ImVec2 offset = io.MousePos - center;
        const bool inside = offset.x * offset.x + offset.y * offset.y <= radius * radius;
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            navigation_pressed_ = inside && !gizmo_hovered_;
        const bool button = ImGui::IsMouseDown(ImGuiMouseButton_Left) || ImGui::IsMouseReleased(ImGuiMouseButton_Left);
        const bool interactive = navigation_pressed_ || !button;

        float new_view[16];
        std::copy(view, view + 16, new_view);
        // The length of its axes follows the projection: a square one with a field of view of 90
        // degrees (as ImOGuizmo recommends), instead of the camera's
        const float near = 0.1f, far = 1000.0f;
        const float gizmo_projection[16] = {1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, -(far + near) / (far - near), -1,
                                            0, 0, -2 * far * near / (far - near), 0};
        // The pivot is the point the camera looks at; MuJoCo's z axis is up (y, z, x: right handed)
        if (ImOGuizmo::DrawGizmo(new_view, gizmo_projection,
                                 interactive ? static_cast<float>(frame.cam.distance) : 0.0f,
                                 ImOGuizmo::CoordinateSystem::YZX))
        {
            // The new direction of the camera, as MuJoCo's azimuth and elevation
            const double fx = -new_view[2], fy = -new_view[6], fz = -new_view[10];
            const double norm = std::sqrt(fx * fx + fy * fy + fz * fz);
            if (norm > 1e-6)
            {
                frame.cam.azimuth = std::atan2(fy, fx) * 180.0 / mjPI;
                frame.cam.elevation = std::clamp(std::asin(fz / norm) * 180.0 / mjPI, -89.5, 89.5);
            }
        }
        gizmo_hovered_ = gizmo_hovered_ || (interactive && inside) || ImOGuizmo::internal::interaction.dragging;
        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
            navigation_pressed_ = false;
        ImGui::End();
    }
}

void MujocoGui::_viewport_toolbar(const float view_rect[4])
{
    // The tools of the gizmo, in the top-left corner of the 3D view
    const float margin = 6.0f * content_scale_;
    ImGui::SetNextWindowPos(ImVec2(view_rect[0] + margin, view_rect[1] + margin));
    ImGui::SetNextWindowBgAlpha(0.85f);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize
                                   | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing
                                   | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 4));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f * content_scale_);
    if (ImGui::Begin("##viewport_toolbar", nullptr, flags))
    {
        if (icon_button("##select", Icon::SELECT, "Select (Q)", true, tool_ == Tool::SELECT))
            tool_ = Tool::SELECT;
        if (icon_button("##move", Icon::MOVE, "Move (W)", true, tool_ == Tool::MOVE))
            tool_ = Tool::MOVE;
        if (icon_button("##rotate", Icon::ROTATE, "Rotate (E)", true, tool_ == Tool::ROTATE))
            tool_ = Tool::ROTATE;
        ImGui::Separator();
        if (ImGui::Button(gizmo_local_ ? "Local" : "World"))
            gizmo_local_ = !gizmo_local_;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The gizmo follows the axes of the %s", gizmo_local_ ? "body" : "world");
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
}

void MujocoGui::_dock_layout(const unsigned int& dockspace_id)
{
    // Scene and visualization on the left, properties, joints and sensors on the right, and the
    // 3D view in the center
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->WorkSize);
    // Widths for the contents of the panels, but at most a share of the window
    const float width = std::max(1.0f, viewport->WorkSize.x);
    const float left_ratio = std::clamp(260.0f * content_scale_ / width, 0.15f, 0.25f);
    const float right_ratio = std::clamp(340.0f * content_scale_ / (width * (1 - left_ratio)), 0.2f, 0.4f);
    ImGuiID center = dockspace_id;
    const ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, left_ratio, nullptr, &center);
    const ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, right_ratio, nullptr, &center);
    ImGuiID left_top = left;
    const ImGuiID left_bottom = ImGui::DockBuilderSplitNode(left_top, ImGuiDir_Down, 0.45f, nullptr, &left_top);
    ImGuiID right_top = right;
    const ImGuiID right_bottom = ImGui::DockBuilderSplitNode(right_top, ImGuiDir_Down, 0.50f, nullptr, &right_top);
    ImGui::DockBuilderDockWindow("Scene", left_top);
    ImGui::DockBuilderDockWindow("Visualization", left_bottom);
    ImGui::DockBuilderDockWindow("Properties", right_top);
    ImGui::DockBuilderDockWindow("Joints", right_bottom);
    ImGui::DockBuilderDockWindow("Sensors", right_bottom);
    ImGui::DockBuilderFinish(dockspace_id);
}

void MujocoGui::_shortcuts(const Frame& frame)
{
    // Not while the user types in a text field
    if (ImGui::GetIO().WantTextInput)
        return;
    if (ImGui::IsKeyPressed(ImGuiKey_Space, false))
        _request({true, false, false, -1});
    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow, true) && !frame.statistics.running)
        _request({false, true, false, -1});
    if (ImGui::IsKeyPressed(ImGuiKey_Backspace, false))
        _request({false, false, true, -1});
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false))
        selected_body_ = -1;
    // Tools of the gizmo, as in Omniverse
    if (!ImGui::GetIO().KeyCtrl)
    {
        if (ImGui::IsKeyPressed(ImGuiKey_Q, false))
            tool_ = Tool::SELECT;
        if (ImGui::IsKeyPressed(ImGuiKey_W, false))
            tool_ = Tool::MOVE;
        if (ImGui::IsKeyPressed(ImGuiKey_E, false))
            tool_ = Tool::ROTATE;
    }
}


//--------------------------------------------------------------------------------------------------
// Bars
//--------------------------------------------------------------------------------------------------

void MujocoGui::_menu_bar(const Frame& frame)
{
    const mjModel* m = frame.m;
    const bool running = frame.statistics.running;
    if (!ImGui::BeginMainMenuBar())
        return;

    if (ImGui::BeginMenu("Simulation"))
    {
        if (ImGui::MenuItem(running ? "Pause" : "Play", "Space"))
            _request({true, false, false, -1});
        if (ImGui::MenuItem("Step", "Right", false, !running))
            _request({false, true, false, -1});
        ImGui::Separator();
        if (ImGui::MenuItem("Reset", "Backspace"))
            _request({false, false, true, -1});
        if (ImGui::BeginMenu("Reset to keyframe", m->nkey > 0))
        {
            for (int k = 0; k < m->nkey; k++)
                if (ImGui::MenuItem(object_label(m, mjOBJ_KEY, k, "keyframe").c_str()))
                    _request({false, false, true, k});
            ImGui::EndMenu();
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View"))
    {
        ImGui::MenuItem("Scene", nullptr, &show_scene_);
        ImGui::MenuItem("Properties", nullptr, &show_properties_);
        ImGui::MenuItem("Joints", nullptr, &show_joints_);
        ImGui::MenuItem("Sensors", nullptr, &show_sensors_);
        ImGui::MenuItem("Visualization", nullptr, &show_visualization_);
        ImGui::Separator();
        ImGui::MenuItem("Navigation gizmo", nullptr, &show_navigation_);
        ImGui::Separator();
        if (ImGui::MenuItem("Reset layout"))
        {
            show_scene_ = show_properties_ = show_joints_ = show_sensors_ = show_visualization_ = true;
            reset_layout_ = true;
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Camera"))
    {
        mjvCamera& cam = frame.cam;
        if (ImGui::MenuItem("Free", nullptr, cam.type == mjCAMERA_FREE))
        {
            cam.type = mjCAMERA_FREE;
            cam.trackbodyid = -1;
            frame.tracking_body.clear();
        }
        const bool can_follow = selected_body_ > 0;
        const std::string follow = can_follow ? "Follow " + body_label(m, selected_body_) : "Follow the selected body";
        if (ImGui::MenuItem(follow.c_str(), nullptr,
                            cam.type == mjCAMERA_TRACKING && cam.trackbodyid == selected_body_, can_follow))
        {
            const char* name = mj_id2name(m, mjOBJ_BODY, selected_body_);
            cam.type = mjCAMERA_TRACKING;
            cam.trackbodyid = selected_body_;
            cam.fixedcamid = -1;
            frame.tracking_body = name ? name : "";
        }
        if (m->ncam > 0)
        {
            ImGui::SeparatorText("Cameras of the model");
            for (int i = 0; i < m->ncam; i++)
            {
                if (ImGui::MenuItem(object_label(m, mjOBJ_CAMERA, i, "camera").c_str(), nullptr,
                                    cam.type == mjCAMERA_FIXED && cam.fixedcamid == i))
                {
                    cam.type = mjCAMERA_FIXED;
                    cam.fixedcamid = i;
                    frame.tracking_body.clear();
                }
            }
        }
        ImGui::EndMenu();
    }

    // Transport controls, in the middle of the bar
    const float h = ImGui::GetFrameHeight();
    const float width = 3 * h * 1.4f + 2 * ImGui::GetStyle().ItemSpacing.x;
    ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(), (ImGui::GetWindowWidth() - width) * 0.5f));
    if (icon_button("##play", running ? Icon::PAUSE : Icon::PLAY, running ? "Pause (Space)" : "Play (Space)"))
        _request({true, false, false, -1});
    if (icon_button("##step", Icon::STEP, "Step (Right)", !running))
        _request({false, true, false, -1});
    if (icon_button("##reset", Icon::RESET, "Reset (Backspace)"))
        _request({false, false, true, -1});

    ImGui::EndMainMenuBar();
}

void MujocoGui::_status_bar(const Frame& frame)
{
    const Statistics& s = frame.statistics;
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings
                                   | ImGuiWindowFlags_MenuBar;
    if (ImGui::BeginViewportSideBar("##status_bar", ImGui::GetMainViewport(), ImGuiDir_Down,
                                    ImGui::GetFrameHeight(), flags))
    {
        if (ImGui::BeginMenuBar())
        {
            status_dot(s.running ? IM_COL32(95, 190, 110, 255) : IM_COL32(220, 165, 60, 255));
            ImGui::TextUnformatted(s.running ? "Running" : "Paused");
            ImGui::TextDisabled("|");
            ImGui::Text("t = %.3f s", frame.state.time);
            if (s.running)
            {
                ImGui::TextDisabled("|");
                ImGui::Text("RTF %.2f", s.real_time_factor);
            }
            ImGui::TextDisabled("|");
            ImGui::Text("Step %.0f us", s.step_time_us);
            ImGui::TextDisabled("|");
            ImGui::Text("%.0f FPS", s.fps);
            if (perturb_button_ >= 0 && selected_body_ > 0)
            {
                ImGui::TextDisabled("|");
                ImGui::TextColored(color(SELECTION), "%s %s",
                                   perturb_button_ == ImGuiMouseButton_Right ? "Moving" : "Rotating",
                                   body_label(frame.m, selected_body_).c_str());
            }

            // The name of the model, on the right
            const char* model = frame.m->names;  // the first name is the model's
            if (model && model[0])
            {
                const float w = ImGui::CalcTextSize(model).x + ImGui::GetStyle().WindowPadding.x;
                ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowWidth() - w));
                ImGui::TextDisabled("%s", model);
            }
            ImGui::EndMenuBar();
        }
    }
    ImGui::End();
}


//--------------------------------------------------------------------------------------------------
// Panels
//--------------------------------------------------------------------------------------------------

void MujocoGui::_scene_panel(const Frame& frame)
{
    if (!show_scene_)
        return;
    if (ImGui::Begin("Scene", &show_scene_))
    {
        const mjModel* m = frame.m;
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::InputTextWithHint("##filter", "Search bodies", filter_, sizeof(filter_));

        auto badge = [&](const int& body) {
            const char* kind = has_free_joint(m, body) ? "free" : (m->body_mocapid[body] >= 0 ? "kinematic" : nullptr);
            if (!kind)
                return;
            ImGui::SameLine();
            ImGui::TextDisabled("%s", kind);
        };

        if (ImGui::BeginChild("##bodies"))
        {
            // A body selected in the 3D view: open its ancestors and scroll to it
            std::vector<char> reveal(m->nbody, 0);
            if (reveal_selection_ && selected_body_ > 0 && selected_body_ < m->nbody)
                for (int b = m->body_parentid[selected_body_]; b > 0; b = m->body_parentid[b])
                    reveal[b] = 1;
            auto scroll_to_selection = [&](const int& b) {
                if (reveal_selection_ && b == selected_body_)
                    ImGui::SetScrollHereY(0.5f);
            };

            if (filter_[0])
            {
                // Search: a flat list of the matching bodies
                for (int b = 0; b < m->nbody; b++)
                {
                    const std::string label = body_label(m, b);
                    if (!contains_case_insensitive(label, filter_))
                        continue;
                    ImGui::PushID(b);
                    if (ImGui::Selectable(label.c_str(), selected_body_ == b))
                        selected_body_ = b;
                    scroll_to_selection(b);
                    badge(b);
                    ImGui::PopID();
                }
            }
            else
            {
                // The kinematic tree, from the world body
                std::vector<std::vector<int>> children(m->nbody);
                for (int b = 1; b < m->nbody; b++)
                    children[m->body_parentid[b]].push_back(b);
                std::function<void(int)> draw = [&](const int& b) {
                    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick
                                               | ImGuiTreeNodeFlags_SpanAvailWidth;
                    if (children[b].empty())
                        flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
                    if (b == 0 || m->body_parentid[b] == 0)
                        flags |= ImGuiTreeNodeFlags_DefaultOpen;
                    if (selected_body_ == b)
                        flags |= ImGuiTreeNodeFlags_Selected;
                    if (reveal[b])
                        ImGui::SetNextItemOpen(true);
                    const bool open = ImGui::TreeNodeEx(reinterpret_cast<void*>(static_cast<intptr_t>(b)), flags,
                                                        "%s", body_label(m, b).c_str());
                    if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
                        selected_body_ = b;
                    scroll_to_selection(b);
                    badge(b);
                    if (open && !children[b].empty())
                    {
                        for (const int& child : children[b])
                            draw(child);
                        ImGui::TreePop();
                    }
                };
                draw(0);
            }
            reveal_selection_ = false;
        }
        ImGui::EndChild();
    }
    ImGui::End();
}

void MujocoGui::_properties_panel(const Frame& frame)
{
    if (!show_properties_)
        return;
    if (ImGui::Begin("Properties", &show_properties_))
    {
        const mjModel* m = frame.m;
        const int b = selected_body_;
        if (b < 0 || 3 * b + 2 >= static_cast<int>(frame.state.xpos.size()))
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
            ImGui::TextWrapped("Select a body in the Scene panel.");
            ImGui::PopStyleColor();
            ImGui::End();
            return;
        }

        ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 1.2f);
        ImGui::TextUnformatted(body_label(m, b).c_str());
        ImGui::PopFont();
        ImGui::TextDisabled("Body %d", b);
        if (b > 0)
        {
            ImGui::SameLine();
            ImGui::TextDisabled("  Parent: %s", body_label(m, m->body_parentid[b]).c_str());
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Pose", ImGuiTreeNodeFlags_DefaultOpen))
        {
            const mjtNum* p = frame.state.xpos.data() + 3 * b;
            const mjtNum* q = frame.state.xquat.data() + 4 * b;
            static const char* const xyz[3] = {"X", "Y", "Z"};
            static const char* const wxyz[4] = {"W", "X", "Y", "Z"};
            static const ImU32 quaternion_colors[4] = {AXIS_COLORS[3], AXIS_COLORS[0], AXIS_COLORS[1], AXIS_COLORS[2]};
            vector_row("Position", p, 3, xyz, AXIS_COLORS);
            vector_row("Rotation", q, 4, wxyz, quaternion_colors);
        }

        if (ImGui::CollapsingHeader("Body", ImGuiTreeNodeFlags_DefaultOpen))
        {
            const char* kind = b == 0 ? "world" : has_free_joint(m, b) ? "free (floating base)"
                                                : m->body_mocapid[b] >= 0 ? "kinematic" : "fixed or jointed";
            if (ImGui::BeginTable("##body", 2, ImGuiTableFlags_SizingStretchProp))
            {
                auto row = [](const char* name, const char* format, auto... values) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TextDisabled("%s", name);
                    ImGui::TableNextColumn();
                    ImGui::Text(format, values...);
                };
                row("Type", "%s", kind);
                row("Mass", "%.4g kg", m->body_mass[b]);
                row("Subtree mass", "%.4g kg", m->body_subtreemass[b]);
                row("Geoms", "%d", m->body_geomnum[b]);
                row("Joints", "%d", m->body_jntnum[b]);
                ImGui::EndTable();
            }
            if (b > 0)
            {
                mjvCamera& cam = frame.cam;
                const bool followed = cam.type == mjCAMERA_TRACKING && cam.trackbodyid == b;
                if (ImGui::Button(followed ? "Stop following" : "Follow with the camera", ImVec2(-FLT_MIN, 0)))
                {
                    if (followed)
                    {
                        cam.type = mjCAMERA_FREE;
                        cam.trackbodyid = -1;
                        frame.tracking_body.clear();
                    }
                    else
                    {
                        const char* name = mj_id2name(m, mjOBJ_BODY, b);
                        cam.type = mjCAMERA_TRACKING;
                        cam.trackbodyid = b;
                        cam.fixedcamid = -1;
                        frame.tracking_body = name ? name : "";
                    }
                }
                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
                switch (movable(m, b))
                {
                case Movable::FREE:      ImGui::TextWrapped("Gizmo (W, E): places the body."); break;
                case Movable::KINEMATIC: ImGui::TextWrapped("Gizmo (W, E): moves the kinematic body."); break;
                case Movable::FIXED:     ImGui::TextWrapped("Gizmo (W, E): moves the body and its children."); break;
                case Movable::NONE:      ImGui::TextWrapped("Moved by its joints: no gizmo."); break;
                }
                ImGui::TextWrapped("Ctrl + drag in the 3D view: left rotates the body, right moves it "
                                   "(Shift: horizontally).");
                ImGui::PopStyleColor();
            }
        }
    }
    ImGui::End();
}

void MujocoGui::_joints_panel(const Frame& frame)
{
    if (!show_joints_)
        return;
    if (ImGui::Begin("Joints", &show_joints_))
    {
        const mjModel* m = frame.m;
        const State& s = frame.state;
        const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable
                                      | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp;
        if (m->njnt == 0)
            ImGui::TextDisabled("The model has no joints.");
        else if (ImGui::BeginTable("##joints", 3, flags))
        {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Joint", ImGuiTableColumnFlags_None, 1.1f);
            ImGui::TableSetupColumn("Position", ImGuiTableColumnFlags_None, 1.2f);
            ImGui::TableSetupColumn("Velocity", ImGuiTableColumnFlags_None, 0.7f);
            ImGui::TableHeadersRow();
            for (int j = 0; j < m->njnt; j++)
            {
                const int type = m->jnt_type[j];
                if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
                    continue;  // free and ball joints: see the pose of their body
                const int qadr = m->jnt_qposadr[j];
                const int vadr = m->jnt_dofadr[j];
                if (qadr >= static_cast<int>(s.qpos.size()) || vadr >= static_cast<int>(s.qvel.size()))
                    continue;
                const double q = s.qpos[qadr];
                const char* unit = type == mjJNT_HINGE ? "rad" : "m";

                ImGui::PushID(j);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::Selectable(object_label(m, mjOBJ_JOINT, j, "joint").c_str(),
                                      m->jnt_bodyid[j] == selected_body_, ImGuiSelectableFlags_SpanAllColumns))
                    selected_body_ = m->jnt_bodyid[j];
                ImGui::TableNextColumn();
                char text[32];
                std::snprintf(text, sizeof(text), "%.3f", q);
                if (m->jnt_limited[j] && m->jnt_range[2 * j + 1] > m->jnt_range[2 * j])
                {
                    // Position within the limits
                    const double low = m->jnt_range[2 * j], high = m->jnt_range[2 * j + 1];
                    const float fraction = static_cast<float>(std::clamp((q - low) / (high - low), 0.0, 1.0));
                    ImGui::ProgressBar(fraction, ImVec2(-FLT_MIN, 0), text);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%.4f %s, limits [%.4f, %.4f]", q, unit, low, high);
                }
                else
                {
                    ImGui::TextUnformatted(text);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%.4f %s", q, unit);
                }
                ImGui::TableNextColumn();
                ImGui::Text("%.3f", s.qvel[vadr]);
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void MujocoGui::_sensors_panel(const Frame& frame)
{
    if (!show_sensors_)
        return;
    if (ImGui::Begin("Sensors", &show_sensors_))
    {
        const mjModel* m = frame.m;
        const State& s = frame.state;
        const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable
                                      | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp;
        if (m->nsensor == 0)
            ImGui::TextDisabled("The model has no sensors.");
        else if (ImGui::BeginTable("##sensors", 2, flags))
        {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Sensor", ImGuiTableColumnFlags_None, 1.0f);
            ImGui::TableSetupColumn("Values", ImGuiTableColumnFlags_None, 2.0f);
            ImGui::TableHeadersRow();
            for (int i = 0; i < m->nsensor; i++)
            {
                const int adr = m->sensor_adr[i];
                const int dim = m->sensor_dim[i];
                if (adr + dim > static_cast<int>(s.sensordata.size()))
                    continue;
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(object_label(m, mjOBJ_SENSOR, i, "sensor").c_str());
                ImGui::TableNextColumn();
                std::string values;
                char value[32];
                for (int k = 0; k < dim; k++)
                {
                    std::snprintf(value, sizeof(value), k ? "  %.3f" : "%.3f", s.sensordata[adr + k]);
                    values += value;
                }
                ImGui::TextUnformatted(values.c_str());
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void MujocoGui::_visualization_panel(const Frame& frame)
{
    if (!show_visualization_)
        return;
    if (ImGui::Begin("Visualization", &show_visualization_))
    {
        mjvOption& opt = frame.opt;
        mjvScene& scn = frame.scn;

        // Columns of checkboxes, as many as fit
        const int columns = std::clamp(static_cast<int>(ImGui::GetContentRegionAvail().x / (ImGui::GetFontSize() * 9.0f)), 1, 3);
        auto checkboxes = [columns](const char* id, const int* flags, const int& n, const char* (*names)[3],
                                    mjtByte* values) {
            if (!ImGui::BeginTable(id, columns))
                return;
            for (int i = 0; i < n; i++)
            {
                ImGui::TableNextColumn();
                bool value = values[flags[i]] != 0;
                if (ImGui::Checkbox(names[flags[i]][0], &value))
                    values[flags[i]] = value ? 1 : 0;
            }
            ImGui::EndTable();
        };

        ImGui::SeparatorText("Rendering");
        static const int rendering[] = {mjRND_SHADOW, mjRND_REFLECTION, mjRND_SKYBOX, mjRND_HAZE, mjRND_FOG,
                                        mjRND_WIREFRAME};
        checkboxes("##rendering", rendering, IM_ARRAYSIZE(rendering), mjRNDSTRING, scn.flags);

        ImGui::SeparatorText("Show");
        static const int elements[] = {mjVIS_JOINT, mjVIS_ACTUATOR, mjVIS_COM, mjVIS_INERTIA, mjVIS_CONTACTPOINT,
                                       mjVIS_CONTACTFORCE, mjVIS_TRANSPARENT, mjVIS_CONVEXHULL, mjVIS_CAMERA,
                                       mjVIS_LIGHT, mjVIS_TENDON, mjVIS_RANGEFINDER, mjVIS_CONSTRAINT,
                                       mjVIS_STATIC, mjVIS_TEXTURE, mjVIS_PERTOBJ, mjVIS_PERTFORCE};
        checkboxes("##elements", elements, IM_ARRAYSIZE(elements), mjVISSTRING, opt.flags);

        ImGui::SeparatorText("Frames and labels");
        const float width = ImGui::GetFontSize() * 7.0f;
        ImGui::SetNextItemWidth(width);
        ImGui::Combo("Frames", &opt.frame, mjFRAMESTRING, mjNFRAME);
        ImGui::SetNextItemWidth(width);
        ImGui::Combo("Labels", &opt.label, mjLABELSTRING, mjNLABEL);

        ImGui::SeparatorText("Groups");
        auto groups = [](const char* name, mjtByte* values) {
            ImGui::PushID(name);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(name);
            ImGui::SameLine(ImGui::GetFontSize() * 4.0f);
            for (int g = 0; g < mjNGROUP; g++)
            {
                if (g > 0)
                    ImGui::SameLine();
                bool value = values[g] != 0;
                char label[8];
                std::snprintf(label, sizeof(label), "%d", g);
                if (ImGui::Checkbox(label, &value))
                    values[g] = value ? 1 : 0;
            }
            ImGui::PopID();
        };
        groups("Geoms", opt.geomgroup);
        groups("Sites", opt.sitegroup);
        groups("Joints", opt.jointgroup);
    }
    ImGui::End();
}

}  // namespace DQ_robotics_extensions
