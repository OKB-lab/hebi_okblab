# cmake/FindEIGEN3.cmake
# Ubuntuのヘッダ配置に合わせて探索
find_path(EIGEN3_INCLUDE_DIR Eigen/Core
  PATHS /usr/include/eigen3 #/usr/local/include/eigen3
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(EIGEN3 DEFAULT_MSG EIGEN3_INCLUDE_DIR)

if(EIGEN3_FOUND)
  set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
  # 必要ならインタフェースターゲットも用意
#   if(NOT TARGET EIGEN3::Eigen)
#     add_library(EIGEN3::Eigen INTERFACE)
#     target_include_directories(EIGEN3::Eigen INTERFACE "${EIGEN3_INCLUDE_DIR}")
#   endif()
endif()
