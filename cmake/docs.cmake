# ---- Declare documentation target ----
find_package(Doxygen)
if(DOXYGEN_FOUND)
	set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/docs/Doxyfile.in)
	set(DOXYGEN_OUT ${PROJECT_SOURCE_DIR}/docs/Doxyfile)
	configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

	add_custom_target(
		docs
		COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
		COMMENT "Build documentation using Doxygen."
		WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/docs"
	)
else(DOXYGEN_FOUND)
	message("To build the documentation, Doxygen must be installed.")
endif(DOXYGEN_FOUND)