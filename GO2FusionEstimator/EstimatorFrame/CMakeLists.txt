cmake_minimum_required(VERSION 3.16)
project(EstimatorFrame_SMX LANGUAGES C CXX)

# =============================
# 全局路径约定
# - 可执行（.exe/.out）：工程根目录
# - 共享库（Windows: .dll / Linux: .so）：Estimator/ 目录
# - 其余产物（.lib/.a/.pdb 等）：Temp/bin
# =============================
set(ROOT ${CMAKE_SOURCE_DIR})
set(TBIN ${ROOT}/Temp/bin)

# =============================
# 源文件（自动跟踪新增）
# =============================
file(GLOB_RECURSE C_SOURCES_ALL   CONFIGURE_DEPENDS
  ${ROOT}/Estimator/*.c
  ${ROOT}/Estimator/C_Estimators/*.c
)
file(GLOB_RECURSE CPP_SOURCES_ALL CONFIGURE_DEPENDS
  ${ROOT}/Estimator/Cpp_Estimators/*.cpp
)

# =========================================================
# 目标 1：共享库 EstimatorPortN
#  - Windows : Output/EstimatorPortN.dll (+ import lib 到 Output/)
#  - Linux   : Output/EstimatorPortN.so
# =========================================================
add_library(EstimatorPortN SHARED
  ${C_SOURCES_ALL}
  ${CPP_SOURCES_ALL}
)

# 头文件（含 Eigen，如存在则降噪）
target_include_directories(EstimatorPortN PRIVATE
  ${ROOT}/Estimator
  ${ROOT}/Estimator/C_Estimators
  ${ROOT}/Estimator/Cpp_Estimators
)
if(EXISTS "${ROOT}/Estimator/Cpp_Estimators/Eigen")
  target_include_directories(EstimatorPortN SYSTEM PRIVATE
    ${ROOT}/Estimator/Cpp_Estimators/Eigen)
endif()

# 语言标准（目标范围）
target_compile_features(EstimatorPortN PRIVATE c_std_11 cxx_std_17)

# 产物路径
set_target_properties(EstimatorPortN PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY            ${ROOT}/Output      # Windows .dll
  LIBRARY_OUTPUT_DIRECTORY            ${ROOT}/Output      # Linux  .so
  ARCHIVE_OUTPUT_DIRECTORY            ${TBIN}             # Windows .lib / Linux .a
  PDB_OUTPUT_DIRECTORY                ${TBIN}
  COMPILE_PDB_OUTPUT_DIRECTORY        ${TBIN}
  DEBUG_POSTFIX ""                                        # 不加 _d
  OUTPUT_NAME EstimatorPortN                              # Linux: EstimatorPortN.so
)

# 编译/链接选项
if(UNIX AND NOT APPLE)
  set_target_properties(EstimatorPortN PROPERTIES PREFIX "")
endif()

if(MSVC)
  target_compile_options(EstimatorPortN PRIVATE /MP /W3 /EHsc)
  target_link_options(EstimatorPortN  PRIVATE /INCREMENTAL:NO)  # 禁止生成 .ilk
else()
  target_compile_options(EstimatorPortN PRIVATE -Wall -Wextra -O2)
  set_target_properties(EstimatorPortN PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# Release 去除 Eigen 调试断言
target_compile_definitions(EstimatorPortN PRIVATE $<$<CONFIG:Release>:EIGEN_NO_DEBUG>)

# Windows: 复制导入库 EstimatorPortN.lib -> Output/（与 dll 放一起）
if(MSVC)
  add_custom_command(TARGET EstimatorPortN POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_LINKER_FILE:EstimatorPortN>   # 通常是 Temp/bin/EstimatorPortN.lib
            ${ROOT}/Output/
    COMMENT "Copy EstimatorPortN.lib to Output/ (Windows)"
  )
endif()

# =========================================================
# 目标 2：C_Demo（直接编入全部估计器源码）
# =========================================================
add_executable(C_Demo
  ${ROOT}/Demo/C_Demo.c
  ${C_SOURCES_ALL}
  ${CPP_SOURCES_ALL}
)

target_include_directories(C_Demo PRIVATE
  ${ROOT}/Estimator
  ${ROOT}/Estimator/C_Estimators
  ${ROOT}/Estimator/Cpp_Estimators
)
if(EXISTS "${ROOT}/Estimator/Cpp_Estimators/Eigen")
  target_include_directories(C_Demo SYSTEM PRIVATE
    ${ROOT}/Estimator/Cpp_Estimators/Eigen)
endif()

target_compile_features(C_Demo PRIVATE c_std_11 cxx_std_17)

set_target_properties(C_Demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY            ${ROOT}/Output
  ARCHIVE_OUTPUT_DIRECTORY            ${TBIN}
  LIBRARY_OUTPUT_DIRECTORY            ${TBIN}
  PDB_OUTPUT_DIRECTORY                ${TBIN}
  COMPILE_PDB_OUTPUT_DIRECTORY        ${TBIN}
  DEBUG_POSTFIX ""
)

if(MSVC)
  target_compile_options(C_Demo PRIVATE /MP /W3 /EHsc)
  target_link_options(C_Demo  PRIVATE /INCREMENTAL:NO)
else()
  target_compile_options(C_Demo PRIVATE -Wall -Wextra -O2)
endif()

target_compile_definitions(C_Demo PRIVATE $<$<CONFIG:Release>:EIGEN_NO_DEBUG>)

# =========================================================
# 目标 3：Cpp_Demo（直接编入全部估计器源码）
# =========================================================
add_executable(Cpp_Demo
  ${ROOT}/Demo/Cpp_Demo.cpp
  ${C_SOURCES_ALL}
  ${CPP_SOURCES_ALL}
)

target_include_directories(Cpp_Demo PRIVATE
  ${ROOT}/Estimator
  ${ROOT}/Estimator/C_Estimators
  ${ROOT}/Estimator/Cpp_Estimators
)
if(EXISTS "${ROOT}/Estimator/Cpp_Estimators/Eigen")
  target_include_directories(Cpp_Demo SYSTEM PRIVATE
    ${ROOT}/Estimator/Cpp_Estimators/Eigen)
endif()

target_compile_features(Cpp_Demo PRIVATE c_std_11 cxx_std_17)

set_target_properties(Cpp_Demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY            ${ROOT}/Output
  ARCHIVE_OUTPUT_DIRECTORY            ${TBIN}
  LIBRARY_OUTPUT_DIRECTORY            ${TBIN}
  PDB_OUTPUT_DIRECTORY                ${TBIN}
  COMPILE_PDB_OUTPUT_DIRECTORY        ${TBIN}
  DEBUG_POSTFIX ""
)

if(MSVC)
  target_compile_options(Cpp_Demo PRIVATE /MP /W3 /EHsc)
  target_link_options(Cpp_Demo  PRIVATE /INCREMENTAL:NO)
else()
  target_compile_options(Cpp_Demo PRIVATE -Wall -Wextra -O2)
endif()

target_compile_definitions(Cpp_Demo PRIVATE $<$<CONFIG:Release>:EIGEN_NO_DEBUG>)

# =========================================================
# 目标 4：C_DLL_Demo（Windows 运行时加载 EstimatorPortN.dll）
#  - 对应你的 Windows DLL 运行时加载示例（LoadLibrary/GetProcAddress）
#  - Windows 运行时：PATH 需包含 %CD%\Estimator
#  - Linux   请使用 C_SO_Demo（dlopen/dlsym）
# =========================================================
add_executable(C_DLL_Demo
  ${ROOT}/Demo/C_DLL_Demo.c
)

target_include_directories(C_DLL_Demo PRIVATE
  ${ROOT}/Estimator
)

set_target_properties(C_DLL_Demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY            ${ROOT}/Output
  ARCHIVE_OUTPUT_DIRECTORY            ${TBIN}
  LIBRARY_OUTPUT_DIRECTORY            ${TBIN}
  PDB_OUTPUT_DIRECTORY                ${TBIN}
  COMPILE_PDB_OUTPUT_DIRECTORY        ${TBIN}
  DEBUG_POSTFIX ""
)

if(MSVC)
  target_compile_options(C_DLL_Demo PRIVATE /W3 /EHsc)
  target_link_options(C_DLL_Demo  PRIVATE /INCREMENTAL:NO)
else()
  target_compile_options(C_DLL_Demo PRIVATE -Wall -Wextra -O2)
endif()
# 注意：不链接 EstimatorPortN（运行时加载）

# =========================================================
# 目标 5：Cpp_DLL_Demo（Windows 运行时加载 EstimatorPortN.dll）
#  - 对应你的 build_cpp_dll_demo.bat：cl Cpp_DLL_Demo.cpp（运行时加载）
#  - Windows 运行时：PATH 需包含 %CD%\Estimator
#  - Linux   请使用 Cpp_SO_Demo（dlopen/dlsym）
# =========================================================
add_executable(Cpp_DLL_Demo
  ${ROOT}/Demo/Cpp_DLL_Demo.cpp
)

target_include_directories(Cpp_DLL_Demo PRIVATE
  ${ROOT}/Estimator
)

set_target_properties(Cpp_DLL_Demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY            ${ROOT}/Output
  ARCHIVE_OUTPUT_DIRECTORY            ${TBIN}
  LIBRARY_OUTPUT_DIRECTORY            ${TBIN}
  PDB_OUTPUT_DIRECTORY                ${TBIN}
  COMPILE_PDB_OUTPUT_DIRECTORY        ${TBIN}
  DEBUG_POSTFIX ""
)

if(MSVC)
  target_compile_options(Cpp_DLL_Demo PRIVATE /W3 /EHsc)
  target_link_options(Cpp_DLL_Demo  PRIVATE /INCREMENTAL:NO)
else()
  target_compile_options(Cpp_DLL_Demo PRIVATE -Wall -Wextra -O2)
endif()
# 注意：不链接 EstimatorPortN（运行时加载）

# =========================================================
# 目标 6：C_SO_Demo（Linux 运行时加载 EstimatorPortN.so）
#  - 使用 dlopen/dlsym；需链接 -ldl
# =========================================================
add_executable(C_SO_Demo
  ${ROOT}/Demo/C_SO_Demo.c
)
target_include_directories(C_SO_Demo PRIVATE
  ${ROOT}/Estimator
)
set_target_properties(C_SO_Demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY            ${ROOT}/Output
  ARCHIVE_OUTPUT_DIRECTORY            ${TBIN}
  LIBRARY_OUTPUT_DIRECTORY            ${TBIN}
  DEBUG_POSTFIX ""
)
if(UNIX AND NOT APPLE)
  target_link_libraries(C_SO_Demo PRIVATE dl)
endif()
if(MSVC)
  target_compile_options(C_SO_Demo PRIVATE /W3)
else()
  target_compile_options(C_SO_Demo PRIVATE -Wall -Wextra -O2)
endif()

# =========================================================
# 目标 7：Cpp_SO_Demo（Linux 运行时加载 EstimatorPortN.so）
#  - 使用 dlopen/dlsym；需链接 -ldl
# =========================================================
add_executable(Cpp_SO_Demo
  ${ROOT}/Demo/Cpp_SO_Demo.cpp
)
target_include_directories(Cpp_SO_Demo PRIVATE
  ${ROOT}/Estimator
)
set_target_properties(Cpp_SO_Demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY            ${ROOT}/Output
  ARCHIVE_OUTPUT_DIRECTORY            ${TBIN}
  LIBRARY_OUTPUT_DIRECTORY            ${TBIN}
  DEBUG_POSTFIX ""
)
if(UNIX AND NOT APPLE)
  target_link_libraries(Cpp_SO_Demo PRIVATE dl)
endif()
if(MSVC)
  target_compile_options(Cpp_SO_Demo PRIVATE /W3 /EHsc)
else()
  target_compile_options(Cpp_SO_Demo PRIVATE -Wall -Wextra -O2)
endif()
