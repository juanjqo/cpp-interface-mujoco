# Patches ImGuizmo and ImOGuizmo after FetchContent downloads them (see dependencies.cmake): their
# state is in global variables, but each DQ_MujocoInterface draws its GUI in its own viewer thread,
# at the same time as the others. The variables become per thread, as the current ImGui context
# (dq_mujoco_imconfig.h).
#   cmake -DLIBRARY=<ImGuizmo|ImOGuizmo> -DFILE=<source file> -P PatchThreadLocal.cmake
if(LIBRARY STREQUAL "ImGuizmo")
    set(global "static Context gContext")               # its context
elseif(LIBRARY STREQUAL "ImOGuizmo")
    set(global "static struct")                         # its configuration and interaction state
else()
    message(FATAL_ERROR "PatchThreadLocal.cmake: unknown LIBRARY \"${LIBRARY}\"")
endif()
string(REPLACE "static " "static thread_local " per_thread "${global}")

file(READ ${FILE} source)
string(FIND "${source}" "${global}" position)
if(position EQUAL -1)
    string(FIND "${source}" "${per_thread}" patched)
    if(patched EQUAL -1)
        message(FATAL_ERROR "PatchThreadLocal.cmake: \"${global}\" not found in ${FILE}. Check the "
                            "patch when updating ${LIBRARY}.")
    endif()
    return()  # already patched
endif()
string(REPLACE "${global}" "${per_thread}" source "${source}")
file(WRITE ${FILE} "${source}")
