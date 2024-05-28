cmake_minimum_required(VERSION 3.14)

include(cmake/prelude.cmake)

project(
    swarm_optimization
    VERSION 0.0.1
    DESCRIPTION "Diploma project for optimizing the trajectory of a swarm of UAV"
    HOMEPAGE_URL "https://example.com/"
    LANGUAGES CXX
)

include(cmake/project-is-top-level.cmake)
include(cmake/variables.cmake)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# ---- Auto MOC, UIC, RCC for Qt ----
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ---- Looking for Qt
find_package(
	Qt5
	COMPONENTS Core Gui Widgets
	REQUIRED
)

# ---- Declare library ----
add_library(
    swarm_optimization_lib OBJECT
    source/PSO.hpp
)

target_include_directories(
    swarm_optimization_lib ${warning_guard}
    PUBLIC "\$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/source>"
)

target_compile_features(swarm_optimization_lib PUBLIC cxx_std_20)

target_link_libraries(
	swarm_optimization_lib
	PUBLIC Qt5::Core
	PUBLIC Qt5::Gui
	PUBLIC Qt5::Widgets
)

#[===[
  The executable file, when compiled under Windows, will be created with the WinMain() entry point, not just main().
  This makes it a GUI executable file, not a console application.
  They will also allow you to remove the opening console when launching the executable file from the GUI.
]===]

if(WIN32 AND MINGW)
	set(EXE_TYPE WIN32)
else ()
	set(EXE_TYPE "")
endif()


# ---- Declare executable ----
add_executable(
	swarm_optimization_exe ${EXE_TYPE}
	source/main.cpp
)
add_executable(swarm_optimization::exe ALIAS swarm_optimization_exe)

set_property(
	TARGET swarm_optimization_exe
	PROPERTY
	OUTPUT_NAME swarm_optimization
)

target_compile_features(
	swarm_optimization_exe
	PRIVATE cxx_std_20
)

target_link_libraries(
  swarm_optimization_exe 
  PRIVATE swarm_optimization_lib
)

# ---- Install rules ----
if(NOT CMAKE_SKIP_INSTALL_RULES)
  include(cmake/install-rules.cmake)
endif()

# ---- Project graph ----
if(PROJECT_IS_TOP_LEVEL)
	add_custom_target(
		graphviz
		COMMAND ${CMAKE_COMMAND} -E make_directory GraphDependencies
		COMMAND ${CMAKE_COMMAND} "--graphviz=${PROJECT_NAME}_GRAPH.dot" .
		COMMAND dot -Tpdf ${PROJECT_NAME}_GRAPH.dot -o ./GraphDependencies/${PROJECT_NAME}_GRAPH.pdf
		COMMAND ${CMAKE_COMMAND} -E rm "${CMAKE_BINARY_DIR}/*.dot*"
		WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
	)
endif()

# ---- Копирование ресурсов времени выполнения ----
add_custom_target(
	optimization_results
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/source/plot_csv.py ${CMAKE_CURRENT_BINARY_DIR}/plot_csv.py
	COMMAND python3 ${CMAKE_CURRENT_BINARY_DIR}/plot_csv.py
)

# ---- Find and copy DLLs ----
if(WIN32 AND MINGW)
	# Path to Qt-plugins
	get_target_property(qmake_executable_path Qt5::qmake IMPORTED_LOCATION)
	cmake_path(REMOVE_FILENAME qmake_executable_path)
	cmake_path(SET qt_plugins_path NORMALIZE ${qmake_executable_path})
	cmake_path(APPEND qt_plugins_path "../plugins/platforms")
	cmake_path(SET qt_plugins_path NORMALIZE ${qt_plugins_path})

	# Building and executing (post-build) bash-script
	configure_file(${CMAKE_SOURCE_DIR}/Copy_DLLs.sh ${CMAKE_CURRENT_BINARY_DIR}/Copy_DLLs.sh @ONLY)
	add_custom_command(
    TARGET swarm_optimization_exe
    POST_BUILD COMMAND bash "${CMAKE_CURRENT_BINARY_DIR}/Copy_DLLs.sh"
  )
endif()

# ---- Developer mode ----
if(NOT swarm_optimization_DEVELOPER_MODE)
  return()
elseif(NOT PROJECT_IS_TOP_LEVEL)
  message(
      AUTHOR_WARNING
      "Developer mode is intended for developers of swarm_optimization"
  )
endif()

include(cmake/dev-mode.cmake)
