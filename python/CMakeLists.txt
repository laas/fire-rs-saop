find_program(PYTHON "python3")

if (PYTHON)
    set(SETUP_PY "${CMAKE_CURRENT_SOURCE_DIR}/setup.py")
    set(OUTPUT      "${CMAKE_CURRENT_BINARY_DIR}/timestamp")
    file(GLOB_RECURSE CYTHON_SOURCES *.pyx)

    add_custom_command(
      OUTPUT ${OUTPUT}
      COMMAND cd ${CMAKE_CURRENT_SOURCE_DIR} && ${PYTHON} ${SETUP_PY} build_ext --inplace
      COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT}
      DEPENDS ${CYTHON_SOURCES})

    add_custom_target(firers-cython ALL DEPENDS ${OUTPUT})
endif()
