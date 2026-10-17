/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

#include "DQ_MujocoSceneBuilderImpl.h"
#include <cstdint>
#include <cstdio>
#include <limits>
#include <set>
#include <stdexcept>

namespace DQ_robotics_extensions
{

using namespace DQ_robotics;
using namespace Eigen;

namespace
{
/**
 * @brief set_pose writes a unit dual quaternion as the position and quaternion (w, x, y, z) of a
 *        MuJoCo element.
 */
void set_pose(double pos[3], double quat[4], const DQ& pose, const std::string& function_name)
{
    if (!is_unit(pose))
        throw std::invalid_argument(function_name + ". The pose must be a unit dual quaternion!");
    const VectorXd p = pose.translation().vec3();
    const VectorXd r = pose.P().vec4();
    for (int k = 0; k < 3; k++)
        pos[k] = p(k);
    for (int k = 0; k < 4; k++)
        quat[k] = r(k);
}

DQ pose_from_qpos(const mjtNum* qpos)
{
    const DQ r = DQ(qpos[3], qpos[4], qpos[5], qpos[6]).normalize();
    const DQ p = qpos[0]*i_ + qpos[1]*j_ + qpos[2]*k_;
    return r + 0.5*E_*p*r;
}

std::vector<double> qpos_from_pose(const DQ& pose)
{
    const VectorXd p = pose.translation().vec3();
    const VectorXd r = pose.P().vec4();
    return {p(0), p(1), p(2), r(0), r(1), r(2), r(3)};
}

std::string name_of(const mjModel* m, const mjtObj& type, const int& id)
{
    const char* name = mj_id2name(m, type, id);
    return name ? name : "";
}

std::set<mjsElement*> elements_of(const mjSpec* spec, const mjtObj& type)
{
    std::set<mjsElement*> elements;
    for (mjsElement* e = mjs_firstElement(spec, type); e; e = mjs_nextElement(spec, e))
        elements.insert(e);
    return elements;
}

// The elements that refer to files. MuJoCo looks for them in the meshdir of the compiler (meshes,
// skins, height fields) or in its texturedir (textures).
const mjtObj file_types[] = {mjOBJ_MESH, mjOBJ_SKIN, mjOBJ_HFIELD, mjOBJ_TEXTURE};

// The file names of an element (in C++, mjString is std::string), empty ones included
std::vector<std::string*> files_of(mjsElement* element)
{
    switch (element->elemtype)
    {
    case mjOBJ_MESH:   return {mjs_asMesh(element)->file};
    case mjOBJ_SKIN:   return {mjs_asSkin(element)->file};
    case mjOBJ_HFIELD: return {mjs_asHField(element)->file};
    case mjOBJ_TEXTURE:
    {
        mjsTexture* texture = mjs_asTexture(element);
        std::vector<std::string*> files = {texture->file};
        for (std::string& file : *texture->cubefiles)
            files.push_back(&file);
        return files;
    }
    default:           return {};
    }
}

std::string strip_path(const std::string& file)
{
    const std::size_t n = file.find_last_of("/\\");
    return n == std::string::npos ? file : file.substr(n + 1);
}

std::string join_path(const std::string& directory, const std::string& file)
{
    if (directory.empty())
        return file;
    const char last = directory.back();
    return (last == '/' || last == '\\') ? directory + file : directory + "/" + file;
}

// The path of a file of a robot in the scene: robot_assets/<name>_<hash>.<extension>. The hash
// (64-bit FNV-1a of the content) makes it the same for the same file, and different for different
// files, also in the file name (MuJoCo's virtual file system falls back to matching file names).
std::string scene_path(const std::string& original, const std::vector<unsigned char>& content)
{
    std::uint64_t hash = 14695981039346656037ULL;
    for (const unsigned char& byte : content)
    {
        hash ^= byte;
        hash *= 1099511628211ULL;
    }
    char hash_text[17];
    std::snprintf(hash_text, sizeof(hash_text), "%016llx", static_cast<unsigned long long>(hash));

    const std::string name = strip_path(original);
    const std::size_t dot = name.find_last_of('.');
    const std::string stem = dot == std::string::npos ? name : name.substr(0, dot);
    const std::string extension = dot == std::string::npos ? "" : name.substr(dot);
    return "robot_assets/" + stem + "_" + hash_text + extension;
}

/**
 * @brief relocate_files points the files of a robot, parsed into child, to their paths in the
 *        scene (see scene_path()), since the scene, when saved, does not keep the directories of
 *        the attached models (meshdir, texturedir).
 * @return The files, by their path in the scene, with their content in robot_model.assets.
 */
std::map<std::string, const std::vector<unsigned char>*> relocate_files(mjSpec* child,
                                                                        const DQ_MujocoModel& robot_model,
                                                                        const std::string& function_name)
{
    std::map<std::string, const std::vector<unsigned char>*> files;
    for (const mjtObj& type : file_types)
    {
        const std::string directory = mjs_getString(type == mjOBJ_TEXTURE ? child->compiler.texturedir
                                                                          : child->compiler.meshdir);
        for (mjsElement* e = mjs_firstElement(child, type); e; e = mjs_nextElement(child, e))
            for (std::string* file : files_of(e))
            {
                if (file->empty())
                    continue;
                const std::string original = join_path(directory, child->strippath ? strip_path(*file) : *file);
                const auto it = robot_model.assets.find(original);
                if (it == robot_model.assets.end())
                    throw std::invalid_argument(function_name + ". The file \"" + original + "\" of the robot "
                                                "is not in the assets of its model.");
                const std::string path = scene_path(original, it->second);
                files[path] = &it->second;
                *file = path;
            }
    }
    mjs_setString(child->compiler.meshdir, "");
    mjs_setString(child->compiler.texturedir, "");
    mjs_setString(child->modelfiledir, "");
    return files;
}

// A virtual file system with the given files, deleted with it
struct ScopedVFS
{
    mjVFS vfs;
    ScopedVFS() { mj_defaultVFS(&vfs); }
    ~ScopedVFS() { mj_deleteVFS(&vfs); }
    ScopedVFS(const ScopedVFS&) = delete;
    ScopedVFS& operator=(const ScopedVFS&) = delete;

    bool add(const std::string& name, const void* buffer, const std::size_t& size)
    {
        return size <= static_cast<std::size_t>(std::numeric_limits<int>::max())
               && mj_addBufferVFS(&vfs, name.c_str(), buffer, static_cast<int>(size)) == 0;
    }
};
}

//--------------------------------------------------------------------------------------------------
// Impl
//--------------------------------------------------------------------------------------------------

DQ_MujocoSceneBuilder::Impl::Impl(const std::string& name)
{
    spec_ = mj_makeSpec();
    if (!spec_)
        throw std::runtime_error("Failed to create the MuJoCo model specification");
    mjs_setString(spec_->modelname, name.c_str());

    // Radians, also in the saved XML (the robots are converted when their XML is parsed)
    spec_->compiler.degree = 0;
    spec_->option.integrator = mjINT_IMPLICITFAST;

    // The look of the examples: a gradient sky, a soft headlight and a bit of haze
    mjVisual& visual = spec_->visual;
    for (int k = 0; k < 3; k++)
    {
        visual.headlight.diffuse[k] = 0.6f;
        visual.headlight.ambient[k] = 0.3f;
        visual.headlight.specular[k] = 0.0f;
    }
    const float haze[4] = {0.15f, 0.25f, 0.35f, 1.0f};
    std::copy(haze, haze + 4, visual.rgba.haze);
    visual.global.azimuth = 135;
    visual.global.elevation = -25;

    mjsTexture* sky = mjs_addTexture(spec_);
    mjs_setName(sky->element, "skybox");
    sky->type = mjTEXTURE_SKYBOX;
    sky->builtin = mjBUILTIN_GRADIENT;
    const double rgb1[3] = {0.3, 0.5, 0.7};
    std::copy(rgb1, rgb1 + 3, sky->rgb1);
    std::fill(sky->rgb2, sky->rgb2 + 3, 0.0);
    // A smooth gradient does not need a high resolution, and the texture is generated at each
    // compilation (also by edit_scene())
    sky->width = 256;
    sky->height = 1536;
}

DQ_MujocoSceneBuilder::Impl::Impl(mjSpec* spec, const mjModel* model)
    : spec_(spec), owns_spec_(false)
{
    const int keyframe_id = mj_name2id(model, mjOBJ_KEY, home_keyframe_name_);
    if (keyframe_id >= 0)
        read_home(model, keyframe_id);
}

DQ_MujocoSceneBuilder::Impl::~Impl()
{
    if (owns_spec_ && spec_)
        mj_deleteSpec(spec_);
}

mjsBody* DQ_MujocoSceneBuilder::Impl::world() const
{
    return mjs_findBody(spec_, "world");
}

void DQ_MujocoSceneBuilder::Impl::check_name_is_free(const mjtObj& type, const std::string& name) const
{
    if (name.empty())
        throw std::invalid_argument("The name cannot be empty.");
    if (mjs_findElement(spec_, type, name.c_str()))
        throw std::invalid_argument("The name \"" + name + "\" is already used in the scene.");
}

void DQ_MujocoSceneBuilder::Impl::add_primitive(const std::string& name, const DQ& pose, const mjtGeom& type,
                                                const double size[3], const BODY& body,
                                                const Vector4d& rgba, const double& mass)
{
    const std::string function_name = "Bad call in DQ_MujocoSceneBuilder::add_primitive";
    check_name_is_free(mjOBJ_BODY, name);
    check_name_is_free(mjOBJ_GEOM, name);
    for (int k = 0; k < 3; k++)
        if (size[k] < 0)
            throw std::invalid_argument(function_name + ". The sizes must be positive.");
    if (body == BODY::FREE && mass <= 0)
        throw std::invalid_argument(function_name + ". The mass of a FREE object must be positive.");

    mjsBody* b = mjs_addBody(world(), nullptr);
    mjs_setName(b->element, name.c_str());
    set_pose(b->pos, b->quat, pose, function_name);
    if (body == BODY::FREE)
        mjs_addFreeJoint(b);  // unnamed: the object is addressed by its body name
    if (body == BODY::KINEMATIC)
        b->mocap = 1;  // its pose is part of the state (mjData::mocap_pos/mocap_quat)

    mjsGeom* geom = mjs_addGeom(b, nullptr);
    mjs_setName(geom->element, name.c_str());
    geom->type = type;
    std::copy(size, size + 3, geom->size);
    for (int k = 0; k < 4; k++)
        geom->rgba[k] = static_cast<float>(rgba(k));
    if (body == BODY::FREE)
        geom->mass = mass;
}

DQ_MujocoRobotDescription DQ_MujocoSceneBuilder::Impl::add_robot(const DQ_MujocoRobotDescription& robot,
                                                                 const std::string& prefix, const DQ& pose,
                                                                 const std::string& parent_body)
{
    const std::string function_name = "Bad call in DQ_MujocoSceneBuilder::add_robot";
    if (robot.model.xml.empty())
        throw std::invalid_argument(function_name + ". The model of the robot is empty.");
    // MuJoCo names the default classes of an attached model after the prefix
    if (prefix.empty())
        throw std::invalid_argument(function_name + ". The prefix cannot be empty.");
    if (!is_unit(pose))
        throw std::invalid_argument(function_name + ". The pose must be a unit dual quaternion!");
    mjsBody* parent = parent_body.empty() ? world() : mjs_findBody(spec_, parent_body.c_str());
    if (!parent)
        throw std::invalid_argument(function_name + ". There is no body \"" + parent_body + "\" in the scene.");
    const bool in_the_world = parent == world();

    // Parse the robot into its own specification (the attach moves its elements into the scene)
    const std::string file_name = robot.model.name.empty() ? "robot.xml" : robot.model.name;
    ScopedVFS vfs;
    struct Cleanup
    {
        mjSpec* spec = nullptr;
        mjModel* model = nullptr;
        ~Cleanup()
        {
            if (model) mj_deleteModel(model);
            if (spec) mj_deleteSpec(spec);
        }
    } cleanup;
    if (!vfs.add(file_name, robot.model.xml.data(), robot.model.xml.size()))
        throw std::runtime_error(function_name + ". Failed to read the model of the robot.");
    for (const auto& [name, content] : robot.model.assets)
        if (!vfs.add(name, content.data(), content.size()))
            throw std::runtime_error(function_name + ". Failed to read the file \"" + name + "\" of the robot.");
    char error[1000] = "";
    mjSpec* child = mj_parseXML(file_name.c_str(), &vfs.vfs, error, sizeof(error));
    if (!child)
        throw std::runtime_error(function_name + ". Failed to parse the model of the robot: " + error);
    cleanup.spec = child;

    // Compile the robot alone, to read its home keyframe by name
    mjModel* model = mj_compile(child, &vfs.vfs);
    if (!model)
        throw std::runtime_error(function_name + ". Failed to compile the model of the robot: "
                                 + mjs_getError(child));
    cleanup.model = model;
    int keyframe_id = -1;
    if (!robot.home_keyframe.empty())
    {
        keyframe_id = mj_name2id(model, mjOBJ_KEY, robot.home_keyframe.c_str());
        if (keyframe_id < 0)
            throw std::invalid_argument(function_name + ". The robot has no keyframe \""
                                        + robot.home_keyframe + "\".");
    }

    // The robot is attached by its root body (the robot's floor, lights, etc. are left out)
    int root_id = -1;
    int roots = 0;
    for (int b = 1; b < model->nbody; b++)
        if (model->body_parentid[b] == 0)
        {
            root_id = b;
            roots++;
        }
    if (roots != 1)
        throw std::invalid_argument(function_name + ". The robot must have exactly one body attached to the "
                                    "world, but it has " + std::to_string(roots) + ".");
    const std::string root_name = name_of(model, mjOBJ_BODY, root_id);
    if (root_name.empty())
        throw std::invalid_argument(function_name + ". The root body of the robot must have a name.");
    // MuJoCo only allows free joints in bodies attached to the world
    if (!in_the_world)
        for (int j = 0; j < model->njnt; j++)
            if (model->jnt_type[j] == mjJNT_FREE)
                throw std::invalid_argument(function_name + ". A robot with a floating base can only be "
                                            "added to the world, not to the body \"" + parent_body + "\".");
    check_name_is_free(mjOBJ_BODY, prefix + root_name);

    const std::map<std::string, const std::vector<unsigned char>*> files
        = relocate_files(child, robot.model, function_name);
    std::map<mjtObj, std::set<mjsElement*>> file_elements_before;
    for (const mjtObj& type : file_types)
        file_elements_before[type] = elements_of(spec_, type);
    const std::set<mjsElement*>& textures_before = file_elements_before[mjOBJ_TEXTURE];
    const std::set<mjsElement*> materials_before = elements_of(spec_, mjOBJ_MATERIAL);
    mjsFrame* frame = mjs_addFrame(parent, nullptr);
    set_pose(frame->pos, frame->quat, pose, function_name);
    if (!mjs_attach(frame->element, mjs_findBody(child, root_name.c_str())->element, prefix.c_str(), ""))
    {
        const std::string message = mjs_getError(spec_);
        mjs_delete(spec_, frame->element);
        throw std::runtime_error(function_name + ". Failed to attach the robot: " + message);
    }

    // The attach brings all the assets of the robot's model, also those of its floor and sky
    // (e.g., a large skybox texture), which make the scene slow to compile and to render. Only the
    // materials and textures used by the bodies of the robot are kept.
    std::set<std::string> used_materials;
    std::set<std::string> used_textures;
    auto use_material = [&](const int& material_id) {
        if (material_id < 0)
            return;
        used_materials.insert(prefix + name_of(model, mjOBJ_MATERIAL, material_id));
        for (int role = 0; role < mjNTEXROLE; role++)
        {
            const int texture_id = model->mat_texid[material_id * mjNTEXROLE + role];
            if (texture_id >= 0)
                used_textures.insert(prefix + name_of(model, mjOBJ_TEXTURE, texture_id));
        }
    };
    for (int g = 0; g < model->ngeom; g++)
        if (model->geom_bodyid[g] != 0)
            use_material(model->geom_matid[g]);
    for (int s = 0; s < model->nsite; s++)
        if (model->site_bodyid[s] != 0)
            use_material(model->site_matid[s]);
    std::vector<mjsElement*> unused;
    for (mjsElement* e : elements_of(spec_, mjOBJ_MATERIAL))
        if (!materials_before.count(e) && !used_materials.count(mjs_getString(mjs_getName(e))))
            unused.push_back(e);
    for (mjsElement* e : elements_of(spec_, mjOBJ_TEXTURE))
    {
        const std::string name = mjs_getString(mjs_getName(e));
        if (!textures_before.count(e) && (name.empty() || !used_textures.count(name)))
            unused.push_back(e);
    }
    for (mjsElement* e : unused)
        mjs_delete(spec_, e);

    // The files of the robot that the scene uses
    for (const mjtObj& type : file_types)
        for (mjsElement* e : elements_of(spec_, type))
            if (!file_elements_before[type].count(e))
                for (const std::string* file : files_of(e))
                {
                    const auto it = files.find(*file);
                    if (it != files.end())
                        assets_.emplace(it->first, *it->second);
                }

    // The keyframes of the robot are replaced by the "home" keyframe of the scene. Its free bodies
    // keep their keyframe poses in the robot's frame, so they are moved to where it was placed.
    for (int k = 0; k < model->nkey; k++)
    {
        const std::string key_name = name_of(model, mjOBJ_KEY, k);
        if (!key_name.empty())
            dropped_keyframes_.push_back(prefix + key_name);
    }
    if (keyframe_id >= 0)
    {
        const mjtNum* key_qpos = model->key_qpos + keyframe_id * model->nq;
        const mjtNum* key_ctrl = model->key_ctrl + keyframe_id * model->nu;
        for (int j = 0; j < model->njnt; j++)
        {
            const int adr = model->jnt_qposadr[j];
            if (model->jnt_type[j] == mjJNT_FREE)
            {
                const std::string body_name = name_of(model, mjOBJ_BODY, model->jnt_bodyid[j]);
                if (!body_name.empty())
                    home_free_pose_[prefix + body_name] = qpos_from_pose(pose*pose_from_qpos(key_qpos + adr));
            }
            else
            {
                const std::string joint_name = name_of(model, mjOBJ_JOINT, j);
                const int size = model->jnt_type[j] == mjJNT_BALL ? 4 : 1;
                if (!joint_name.empty())
                    home_qpos_[prefix + joint_name] = std::vector<double>(key_qpos + adr, key_qpos + adr + size);
            }
        }
        for (int a = 0; a < model->nu; a++)
        {
            const std::string actuator_name = name_of(model, mjOBJ_ACTUATOR, a);
            if (!actuator_name.empty())
                home_ctrl_[prefix + actuator_name] = key_ctrl[a];
        }
    }

    // The names of the robot in the scene
    DQ_MujocoRobotDescription placed;
    placed.floating_base = robot.floating_base.empty() ? "" : prefix + robot.floating_base;
    for (const auto& name : robot.joint_names)
        placed.joint_names.push_back(prefix + name);
    for (const auto& name : robot.gripper_joint_names)
        placed.gripper_joint_names.push_back(prefix + name);
    placed.home_keyframe = home_keyframe_name_;
    for (const auto& name : robot.end_effector_sites)
        placed.end_effector_sites.push_back(prefix + name);
    return placed;
}

void DQ_MujocoSceneBuilder::Impl::remove(const std::string& name)
{
    if (name.empty() || name == "world")
        throw std::invalid_argument("Bad call in DQ_MujocoSceneBuilder::remove. Invalid name \"" + name + "\".");
    for (const mjtObj& type : {mjOBJ_BODY, mjOBJ_SITE, mjOBJ_LIGHT, mjOBJ_GEOM})
    {
        if (mjsElement* element = mjs_findElement(spec_, type, name.c_str()))
        {
            mjs_delete(spec_, element);
            home_free_pose_.erase(name);
            return;
        }
    }
    throw std::invalid_argument("Bad call in DQ_MujocoSceneBuilder::remove. There is no object, site or "
                                "light \"" + name + "\" in the scene.");
}

void DQ_MujocoSceneBuilder::Impl::read_home(const mjModel* model, const int& keyframe_id)
{
    const mjtNum* key_qpos = model->key_qpos + keyframe_id * model->nq;
    const mjtNum* key_ctrl = model->key_ctrl + keyframe_id * model->nu;
    for (int j = 0; j < model->njnt; j++)
    {
        const int adr = model->jnt_qposadr[j];
        if (model->jnt_type[j] == mjJNT_FREE)
        {
            const std::string body_name = name_of(model, mjOBJ_BODY, model->jnt_bodyid[j]);
            if (!body_name.empty())
                home_free_pose_[body_name] = std::vector<double>(key_qpos + adr, key_qpos + adr + 7);
        }
        else
        {
            const std::string joint_name = name_of(model, mjOBJ_JOINT, j);
            const int size = model->jnt_type[j] == mjJNT_BALL ? 4 : 1;
            if (!joint_name.empty())
                home_qpos_[joint_name] = std::vector<double>(key_qpos + adr, key_qpos + adr + size);
        }
    }
    for (int a = 0; a < model->nu; a++)
    {
        const std::string actuator_name = name_of(model, mjOBJ_ACTUATOR, a);
        if (!actuator_name.empty())
            home_ctrl_[actuator_name] = key_ctrl[a];
    }
}

void DQ_MujocoSceneBuilder::Impl::fix_keyframes(mjSpec* spec, const mjModel* model) const
{
    for (const auto& name : dropped_keyframes_)
        if (mjsElement* key = mjs_findElement(spec, mjOBJ_KEY, name.c_str()))
            mjs_delete(spec, key);
    if (mjsElement* key = mjs_findElement(spec, mjOBJ_KEY, home_keyframe_name_))
        mjs_delete(spec, key);
    add_home(spec, model);
}

void DQ_MujocoSceneBuilder::Impl::add_home(mjSpec* spec, const mjModel* model) const
{
    std::vector<double> qpos(model->qpos0, model->qpos0 + model->nq);
    std::vector<double> ctrl(model->nu, 0.0);
    for (int j = 0; j < model->njnt; j++)
    {
        const int adr = model->jnt_qposadr[j];
        if (model->jnt_type[j] == mjJNT_FREE)
        {
            auto it = home_free_pose_.find(name_of(model, mjOBJ_BODY, model->jnt_bodyid[j]));
            if (it != home_free_pose_.end())
                std::copy(it->second.begin(), it->second.end(), qpos.begin() + adr);
        }
        else
        {
            auto it = home_qpos_.find(name_of(model, mjOBJ_JOINT, j));
            const std::size_t size = model->jnt_type[j] == mjJNT_BALL ? 4 : 1;
            if (it != home_qpos_.end() && it->second.size() == size)
                std::copy(it->second.begin(), it->second.end(), qpos.begin() + adr);
        }
    }
    for (int a = 0; a < model->nu; a++)
    {
        auto it = home_ctrl_.find(name_of(model, mjOBJ_ACTUATOR, a));
        if (it != home_ctrl_.end())
            ctrl[a] = it->second;
    }

    mjsKey* key = mjs_addKey(spec);
    mjs_setName(key->element, home_keyframe_name_);
    mjs_setDouble(key->qpos, qpos.data(), static_cast<int>(qpos.size()));
    if (!ctrl.empty())
        mjs_setDouble(key->ctrl, ctrl.data(), static_cast<int>(ctrl.size()));
}

std::string DQ_MujocoSceneBuilder::Impl::to_xml() const
{
    const std::string function_name = "Bad call in DQ_MujocoSceneBuilder::build";
    // A copy, so that the builder can still be changed and built again
    struct SpecCopy
    {
        mjSpec* spec;
        ~SpecCopy() { if (spec) mj_deleteSpec(spec); }
    } copy{mj_copySpec(spec_)};
    if (!copy.spec)
        throw std::runtime_error(function_name + ". Failed to copy the scene.");

    ScopedVFS vfs;
    for (const auto& [name, content] : assets_)
        if (!vfs.add(name, content.data(), content.size()))
            throw std::runtime_error(function_name + ". Failed to read the file \"" + name + "\".");

    mjModel* model = mj_compile(copy.spec, &vfs.vfs);
    if (!model)
        throw std::runtime_error(function_name + ". Invalid scene: " + mjs_getError(copy.spec));
    fix_keyframes(copy.spec, model);
    mj_deleteModel(model);
    model = mj_compile(copy.spec, &vfs.vfs);
    if (!model)
        throw std::runtime_error(function_name + ". Invalid scene: " + mjs_getError(copy.spec));
    mj_deleteModel(model);

    char error[1000] = "";
    const int size = mj_saveXMLString(copy.spec, nullptr, 0, error, sizeof(error));
    if (size < 0)
        throw std::runtime_error(function_name + ". Failed to write the scene: " + error);
    std::vector<char> xml(size + 1, '\0');
    if (mj_saveXMLString(copy.spec, xml.data(), static_cast<int>(xml.size()), error, sizeof(error)) != 0)
        throw std::runtime_error(function_name + ". Failed to write the scene: " + error);
    return std::string(xml.data());
}

//--------------------------------------------------------------------------------------------------
// DQ_MujocoSceneBuilder
//--------------------------------------------------------------------------------------------------

/**
 * @brief Start an empty scene, with a sky and a headlight, and the default physics options of the
 *        builder (timestep 0.002 s, IMPLICITFAST integrator, gravity -9.81 m/s^2 along z).
 * @param name Name of the model (shown as the title of the window).
 */
DQ_MujocoSceneBuilder::DQ_MujocoSceneBuilder(const std::string& name)
    : impl_(std::make_unique<Impl>(name))
{
}

DQ_MujocoSceneBuilder::DQ_MujocoSceneBuilder(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl))
{
}

DQ_MujocoSceneBuilder::~DQ_MujocoSceneBuilder() = default;
DQ_MujocoSceneBuilder::DQ_MujocoSceneBuilder(DQ_MujocoSceneBuilder&&) noexcept = default;
DQ_MujocoSceneBuilder& DQ_MujocoSceneBuilder::operator=(DQ_MujocoSceneBuilder&&) noexcept = default;

/**
 * @brief Set the timestep of the simulation
 * @param timestep Timestep in seconds (default: 0.002).
 */
void DQ_MujocoSceneBuilder::set_timestep(const double& timestep)
{
    if (timestep <= 0)
        throw std::invalid_argument(error_msg_layout_ + "set_timestep. The timestep must be positive.");
    impl_->spec_->option.timestep = timestep;
}

/**
 * @brief Set the numerical integrator (default: IMPLICITFAST)
 */
void DQ_MujocoSceneBuilder::set_integrator(const INTEGRATOR& integrator)
{
    switch (integrator)
    {
    case INTEGRATOR::EULER:        impl_->spec_->option.integrator = mjINT_EULER; break;
    case INTEGRATOR::RK4:          impl_->spec_->option.integrator = mjINT_RK4; break;
    case INTEGRATOR::IMPLICIT:     impl_->spec_->option.integrator = mjINT_IMPLICIT; break;
    case INTEGRATOR::IMPLICITFAST: impl_->spec_->option.integrator = mjINT_IMPLICITFAST; break;
    }
}

/**
 * @brief Set the gravity
 * @param gravity Gravity in the world frame, in m/s^2 (default: (0, 0, -9.81)).
 */
void DQ_MujocoSceneBuilder::set_gravity(const Vector3d& gravity)
{
    for (int k = 0; k < 3; k++)
        impl_->spec_->option.gravity[k] = gravity(k);
}

/**
 * @brief Add an infinite floor (a plane at z = 0) with a checker pattern
 * @param name Name of the floor geometry.
 */
void DQ_MujocoSceneBuilder::add_floor(const std::string& name)
{
    impl_->check_name_is_free(mjOBJ_GEOM, name);
    mjSpec* spec = impl_->spec_;
    const std::string texture_name = name + "_texture";
    const std::string material_name = name + "_material";

    mjsTexture* texture = mjs_addTexture(spec);
    mjs_setName(texture->element, texture_name.c_str());
    texture->type = mjTEXTURE_2D;
    texture->builtin = mjBUILTIN_CHECKER;
    texture->mark = mjMARK_EDGE;
    const double rgb1[3] = {0.2, 0.3, 0.4};
    const double rgb2[3] = {0.1, 0.2, 0.3};
    std::copy(rgb1, rgb1 + 3, texture->rgb1);
    std::copy(rgb2, rgb2 + 3, texture->rgb2);
    std::fill(texture->markrgb, texture->markrgb + 3, 0.8);
    texture->width = 300;
    texture->height = 300;

    mjsMaterial* material = mjs_addMaterial(spec, nullptr);
    mjs_setName(material->element, material_name.c_str());
    mjs_setInStringVec(material->textures, mjTEXROLE_RGB, texture_name.c_str());
    material->texuniform = 1;
    material->texrepeat[0] = 5;
    material->texrepeat[1] = 5;
    material->reflectance = 0.2f;

    mjsGeom* floor = mjs_addGeom(impl_->world(), nullptr);
    mjs_setName(floor->element, name.c_str());
    floor->type = mjGEOM_PLANE;
    floor->size[0] = 0;  // infinite
    floor->size[1] = 0;
    floor->size[2] = 0.05;
    mjs_setString(floor->material, material_name.c_str());
}

/**
 * @brief Add a light
 * @param name Name of the light.
 * @param position Position of the light in the world, as a pure quaternion.
 * @param direction Direction of the light in the world, as a pure quaternion (default: down).
 * @param directional If true, a directional light (like the sun, from far away); otherwise a spot
 *        light at the position.
 */
void DQ_MujocoSceneBuilder::add_light(const std::string& name, const DQ& position, const DQ& direction,
                                      const bool& directional)
{
    impl_->check_name_is_free(mjOBJ_LIGHT, name);
    const VectorXd p = position.vec3();
    const VectorXd d = direction.vec3();
    if (d.norm() == 0)
        throw std::invalid_argument(error_msg_layout_ + "add_light. The direction cannot be zero.");
    mjsLight* light = mjs_addLight(impl_->world(), nullptr);
    mjs_setName(light->element, name.c_str());
    for (int k = 0; k < 3; k++)
    {
        light->pos[k] = p(k);
        light->dir[k] = d(k);
    }
    light->type = directional ? mjLIGHT_DIRECTIONAL : mjLIGHT_SPOT;
    light->castshadow = 1;
}

/**
 * @brief Add a box
 * @param name Name of the object (its body and its geometry).
 * @param pose Pose of the center of the box in the world.
 * @param half_sizes Half of the size of the box along its x, y and z axes, in meters.
 * @param body FIXED (welded to the world, the default), KINEMATIC (moved by the program with
 *        DQ_MujocoInterface::set_body_pose()) or FREE (a floating base that falls, collides and
 *        can be pushed).
 * @param rgba Color, with each component between 0 and 1.
 * @param mass Mass in kg (only used by FREE objects).
 */
void DQ_MujocoSceneBuilder::add_box(const std::string& name, const DQ& pose, const Vector3d& half_sizes,
                                    const BODY& body, const Vector4d& rgba, const double& mass)
{
    const double size[3] = {half_sizes(0), half_sizes(1), half_sizes(2)};
    impl_->add_primitive(name, pose, mjGEOM_BOX, size, body, rgba, mass);
}

/**
 * @brief Add a sphere (see add_box() for the common parameters)
 * @param radius Radius in meters.
 */
void DQ_MujocoSceneBuilder::add_sphere(const std::string& name, const DQ& pose, const double& radius,
                                       const BODY& body, const Vector4d& rgba, const double& mass)
{
    const double size[3] = {radius, 0, 0};
    impl_->add_primitive(name, pose, mjGEOM_SPHERE, size, body, rgba, mass);
}

/**
 * @brief Add a cylinder along the z axis of its pose (see add_box() for the common parameters)
 * @param radius Radius in meters.
 * @param half_length Half of the length in meters.
 */
void DQ_MujocoSceneBuilder::add_cylinder(const std::string& name, const DQ& pose, const double& radius,
                                         const double& half_length, const BODY& body, const Vector4d& rgba,
                                         const double& mass)
{
    const double size[3] = {radius, half_length, 0};
    impl_->add_primitive(name, pose, mjGEOM_CYLINDER, size, body, rgba, mass);
}

/**
 * @brief Add a capsule along the z axis of its pose (see add_box() for the common parameters)
 * @param radius Radius in meters.
 * @param half_length Half of the length of the cylindrical part, in meters.
 */
void DQ_MujocoSceneBuilder::add_capsule(const std::string& name, const DQ& pose, const double& radius,
                                        const double& half_length, const BODY& body, const Vector4d& rgba,
                                        const double& mass)
{
    const double size[3] = {radius, half_length, 0};
    impl_->add_primitive(name, pose, mjGEOM_CAPSULE, size, body, rgba, mass);
}

/**
 * @brief Add a site in the world: a frame without physics, e.g., a visual target to move with
 *        DQ_MujocoInterface::set_site_pose()
 * @param name Name of the site.
 * @param pose Pose of the site in the world.
 * @param size Radius of the sphere that shows the site, in meters.
 * @param rgba Color of the sphere.
 */
void DQ_MujocoSceneBuilder::add_site(const std::string& name, const DQ& pose, const double& size,
                                     const Vector4d& rgba)
{
    impl_->check_name_is_free(mjOBJ_SITE, name);
    if (size <= 0)
        throw std::invalid_argument(error_msg_layout_ + "add_site. The size must be positive.");
    mjsSite* site = mjs_addSite(impl_->world(), nullptr);
    mjs_setName(site->element, name.c_str());
    set_pose(site->pos, site->quat, pose, error_msg_layout_ + "add_site");
    site->type = mjGEOM_SPHERE;
    site->size[0] = size;
    for (int k = 0; k < 4; k++)
        site->rgba[k] = static_cast<float>(rgba(k));
}

/**
 * @brief Add a robot, e.g., one of the robots library
 * @param robot The robot (GenericQuadrupedRobot::description(), ...).
 * @param prefix Prefix added to every name of the robot, so that several robots can be added (e.g.,
 *        "arm_" turns the joint "joint1" into "arm_joint1"). It cannot be empty.
 * @param pose Where the robot is placed: the pose of the robot's world frame in the scene, or in the
 *        frame of parent_body. For a floating-base robot, its home configuration is moved there too.
 * @param parent_body The body that the robot is mounted on (e.g., the floating base of a
 *        quadruped, "dog_trunk", for an arm on its back), which then carries it. Empty (the
 *        default): the robot is placed in the world. Only robots without a floating base can be
 *        mounted on a body.
 * @return The description of the robot in the scene: its prefixed names, and "home" as its home
 *         keyframe (the keyframe of the scene, which also has the other robots at home). Its model
 *         is empty: load the scene with build().
 *
 * Only the root body of the robot (and what hangs from it, its actuators, etc.) is added: the
 * floor, lights and world sites of the robot's model are not. The physics options of the scene are
 * used, not those of the robot.
 *
 * To use the kinematics of a robot placed at pose x, move its reference frame, e.g.,
 * @code
 * auto arm = scene.add_robot(GenericManipulatorRobot::description(), "arm_", x);
 * DQ_SerialManipulatorDH kinematics = GenericManipulatorRobot::kinematics();
 * kinematics.set_reference_frame(x*kinematics.get_reference_frame());
 * @endcode
 * For a robot mounted at pose x on a body, its reference frame moves with the body: use
 * x_body*x*kinematics.get_reference_frame(), with x_body = DQ_MujocoInterface::get_body_pose() of the
 * body (or get_base_pose(), for a floating base), updated as the body moves.
 *
 * The files of the robot (meshes, textures) must be in the assets of its model, as in the robots of
 * the robots library. They are added to the scene under robot_assets/, once even if the robot is
 * added several times.
 *
 * @throws std::invalid_argument If the prefix is empty, there is no parent_body, the robot does not
 *         have exactly one body attached to its world, has a floating base and a parent_body, has
 *         no keyframe named home_keyframe, uses a file that is not in its assets, or if its names
 *         clash with the scene
 */
DQ_MujocoRobotDescription DQ_MujocoSceneBuilder::add_robot(const DQ_MujocoRobotDescription& robot,
                                                           const std::string& prefix, const DQ& pose,
                                                           const std::string& parent_body)
{
    return impl_->add_robot(robot, prefix, pose, parent_body);
}

/**
 * @brief Remove an object, a site, a light, the floor, or a robot
 * @param name Its name. For a robot, the name of its root body with the prefix (e.g., "arm_base" or,
 *        for a floating-base robot, its floating base, e.g., "dog_trunk"): its joints and actuators
 *        are removed with it.
 * @throws std::invalid_argument If there is nothing with that name
 */
void DQ_MujocoSceneBuilder::remove(const std::string& name)
{
    impl_->remove(name);
}

/**
 * @brief Build the scene
 * @return The scene as a DQ_MujocoModel, to load with DQ_MujocoInterface::load_model(). Its xml is
 *         the MJCF of the scene, and its assets are the files of the robots (meshes, textures),
 *         under robot_assets/. To save the scene, write the xml and each asset at its path,
 *         relative to the xml file.
 * @throws std::runtime_error If the scene is invalid (with MuJoCo's message)
 *
 * The builder is not changed: it can be changed and built again.
 */
DQ_MujocoModel DQ_MujocoSceneBuilder::build() const
{
    DQ_MujocoModel model;
    model.name = "scene.xml";
    model.xml = impl_->to_xml();
    model.assets = impl_->assets_;
    return model;
}

}
