#-----------------------------------------------------------------------------
# GearAutoOpt 模块: 齿轮自动建模 + 求解 + 多目标优化
#-----------------------------------------------------------------------------

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

#-----------------------------------------------------------------------------
# 源码扫描 (递归子目录: data/ db/ solver/ runner/ opt/ gui/)
#-----------------------------------------------------------------------------
file(GLOB_RECURSE _ui     "*.ui")
file(GLOB_RECURSE _header "*.h")
file(GLOB_RECURSE _source "*.cpp")
# GLOB 只在 configure 时更新：显式加入新文件，避免未重新 cmake 时漏链
set(_gear_opt_log_cpp "${CMAKE_CURRENT_SOURCE_DIR}/data/GearOptLog.cpp")
set(_gear_opt_main_thread_runner_cpp "${CMAKE_CURRENT_SOURCE_DIR}/runner/GearOptMainThreadRunner.cpp")
set(_gear_opt_mesh_indep_cpp "${CMAKE_CURRENT_SOURCE_DIR}/runner/MeshIndependenceRunner.cpp")
set(_gear_opt_bridge_cpp "${CMAKE_CURRENT_SOURCE_DIR}/data/GearOptGeometryBridge.cpp")
set(_gear_opt_compare_cpp "${CMAKE_CURRENT_SOURCE_DIR}/gui/GearOptCompareTable.cpp")
set(_gear_opt_opt_config_cpp "${CMAKE_CURRENT_SOURCE_DIR}/gui/OptimizationConfigDialog.cpp")
set(_gear_opt_metric_def_cpp "${CMAKE_CURRENT_SOURCE_DIR}/data/GearMetricDefinition.cpp")
set(_gear_opt_db_cpp "${CMAKE_CURRENT_SOURCE_DIR}/db/GearOptResultDatabase.cpp")
set(_gear_opt_vtk_cpp "${CMAKE_CURRENT_SOURCE_DIR}/solver/CalculiXResultVtkExport.cpp")
set(_gear_opt_backfill_cpp "${CMAKE_CURRENT_SOURCE_DIR}/tools/GearOptBackfill.cpp")
set(_gear_opt_failed_retry_cpp "${CMAKE_CURRENT_SOURCE_DIR}/tools/GearOptFailedCaseRetry.cpp")
set(_gear_opt_failed_retry_runner_cpp "${CMAKE_CURRENT_SOURCE_DIR}/tools/FailedCaseRetryRunner.cpp")
set(_gear_opt_surrogate_cpp
	"${CMAKE_CURRENT_SOURCE_DIR}/surrogate/GearSurrogateModel.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/surrogate/GearInfillSelector.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/surrogate/GearSurrogateMetrics.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/surrogate/GearSurrogateSampleValidation.cpp"
)
set(_gear_opt_verbose_log_cpp "${CMAKE_CURRENT_SOURCE_DIR}/runner/GearSurrogateVerboseLog.cpp")
if(EXISTS "${_gear_opt_bridge_cpp}")
	list(APPEND _source "${_gear_opt_bridge_cpp}")
	list(REMOVE_DUPLICATES _source)
endif()
if(EXISTS "${_gear_opt_log_cpp}")
	list(APPEND _source "${_gear_opt_log_cpp}")
	list(REMOVE_DUPLICATES _source)
endif()
if(EXISTS "${_gear_opt_main_thread_runner_cpp}")
	list(APPEND _source "${_gear_opt_main_thread_runner_cpp}")
	list(REMOVE_DUPLICATES _source)
endif()
if(EXISTS "${_gear_opt_mesh_indep_cpp}")
	list(APPEND _source "${_gear_opt_mesh_indep_cpp}")
	list(REMOVE_DUPLICATES _source)
endif()
if(EXISTS "${_gear_opt_compare_cpp}")
	list(APPEND _source "${_gear_opt_compare_cpp}")
	list(REMOVE_DUPLICATES _source)
endif()
if(EXISTS "${_gear_opt_opt_config_cpp}")
	list(APPEND _source "${_gear_opt_opt_config_cpp}")
	list(REMOVE_DUPLICATES _source)
endif()
if(EXISTS "${_gear_opt_metric_def_cpp}")
	list(APPEND _source "${_gear_opt_metric_def_cpp}")
	list(REMOVE_DUPLICATES _source)
endif()
if(EXISTS "${_gear_opt_db_cpp}")
	list(APPEND _source "${_gear_opt_db_cpp}")
	list(REMOVE_DUPLICATES _source)
endif()
if(EXISTS "${_gear_opt_vtk_cpp}")
	list(APPEND _source "${_gear_opt_vtk_cpp}")
	list(REMOVE_DUPLICATES _source)
endif()
foreach(_surrogate_cpp IN LISTS _gear_opt_surrogate_cpp)
	if(EXISTS "${_surrogate_cpp}")
		list(APPEND _source "${_surrogate_cpp}")
	endif()
endforeach()
if(EXISTS "${_gear_opt_verbose_log_cpp}")
	list(APPEND _source "${_gear_opt_verbose_log_cpp}")
endif()
if(EXISTS "${_gear_opt_failed_retry_cpp}")
	list(APPEND _source "${_gear_opt_failed_retry_cpp}")
endif()
if(EXISTS "${_gear_opt_failed_retry_runner_cpp}")
	list(APPEND _source "${_gear_opt_failed_retry_runner_cpp}")
endif()
list(REMOVE_DUPLICATES _source)
if(EXISTS "${_gear_opt_backfill_cpp}")
	list(REMOVE_ITEM _source "${_gear_opt_backfill_cpp}")
endif()
qt5_wrap_ui(_interface ${_ui})

#-----------------------------------------------------------------------------
# 动态库目标
#-----------------------------------------------------------------------------
add_library(GearAutoOpt
	${_interface}
	${_header}
	${_source}
)

set_target_properties(GearAutoOpt PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

target_compile_definitions(GearAutoOpt PRIVATE
	"GEARAUTOOPT_API"
	"_USE_MATH_DEFINES"   # MSVC 默认不定义 M_PI 等数学常量
)
if(MSVC)
    target_compile_options(GearAutoOpt PRIVATE /utf-8)
endif()

#-----------------------------------------------------------------------------
# 依赖
#-----------------------------------------------------------------------------
list(APPEND _depend_library
	Common
	DataProperty
	Geometry
	GeometryCommand
	BCBase
	Material
	ModelData
	ModuleBase
	ConfigOptions
	IO
	GmshModule
	SolverControl
)

list(APPEND _runtimes_libraries
	Qt5::Widgets Qt5::Xml Qt5::Gui Qt5::Core Qt5::Sql
	VTK::CommonCore VTK::CommonDataModel VTK::IOXML
)

target_link_libraries(GearAutoOpt PRIVATE
	${_runtimes_libraries}
	${_depend_library}
	# runner 子类引 GeoCommandCreateGear.h，该头文件依赖 OCC；
	# GeometryCommand 用 PRIVATE 链接 OCC，include 路径不传播，这里显式补上。
	OpenCASCADE::TKernel
	OpenCASCADE::TKMath
	OpenCASCADE::TKBRep
	OpenCASCADE::TKG2d
	OpenCASCADE::TKG3d
	OpenCASCADE::TKTopAlgo
)

add_dependencies(GearAutoOpt ${_depend_library})

if(EXISTS "${_gear_opt_backfill_cpp}")
	add_executable(GearOptBackfill "${_gear_opt_backfill_cpp}")
	set_target_properties(GearOptBackfill PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
	if(MSVC)
		target_compile_options(GearOptBackfill PRIVATE /utf-8)
	endif()
	target_link_libraries(GearOptBackfill PRIVATE GearAutoOpt Qt5::Core Qt5::Sql)
endif()

set(FastCAE_GearAutoOpt_Runtimes_Libraries ${_runtimes_libraries} PARENT_SCOPE)
