# Patches ImGuizmo after FetchContent downloads it (see dependencies.cmake): its state is one global
# context, but each DQ_MujocoInterface draws its GUI in its own viewer thread, at the same time as
# the others. The context becomes per thread, as the current ImGui context (dq_mujoco_imconfig.h).
#   cmake -DFILE=<ImGuizmo.cpp> -P PatchImGuizmo.cmake
file(READ ${FILE} source)
if(source MATCHES "static thread_local Context gContext;")
    return()  # already patched
endif()
string(FIND "${source}" "static Context gContext;" position)
if(position EQUAL -1)
    message(FATAL_ERROR "PatchImGuizmo.cmake: \"static Context gContext;\" not found in ${FILE}. "
                        "Check the patch when updating ImGuizmo.")
endif()
string(REPLACE "static Context gContext;" "static thread_local Context gContext;" source "${source}")
file(WRITE ${FILE} "${source}")
