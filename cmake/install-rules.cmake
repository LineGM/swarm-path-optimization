install(
    TARGETS swarm_optimization_exe
    RUNTIME COMPONENT swarm_optimization_Runtime
)

if(PROJECT_IS_TOP_LEVEL)
  include(CPack)
endif()
