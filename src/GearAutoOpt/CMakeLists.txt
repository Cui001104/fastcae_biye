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

set(FastCAE_GearAutoOpt_Runtimes_Libraries ${_runtimes_libraries} PARENT_SCOPE)
