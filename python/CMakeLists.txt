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

# if (PYTHON)
#     file(GLOB_RECURSE PYTHON_SOURCES *.pyx)
#     MESSAGE(STATUS "${PYTHON_SOURCES}")
#     set(SETUP_PY_IN "${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in")
#     set(SETUP_PY    "${CMAKE_CURRENT_BINARY_DIR}/setup.py")
#     set(DEPS        "${PYTHON_SOURCES}")
#     set(DEPS        "${CMAKE_CURRENT_SOURCE_DIR}/fire_rs/__init__.py")
#     set(PY_SRC        "${CMAKE_CURRENT_SOURCE_DIR}/fire_rs/")
#     set(PY_BIN        "${CMAKE_BINARY_SOURCE_DIR}/fire_rs/")
#     set(OUTPUT      "${CMAKE_CURRENT_BINARY_DIR}/build/timestamp")

#     configure_file(${SETUP_PY_IN} ${SETUP_PY})
#     #file(COPY ${PY_SRC} DESTINATION ${PY_BIN})

#     add_custom_command(OUTPUT ${OUTPUT}
#                        COMMAND ${PYTHON} ${SETUP_PY} build
#                        COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT}
#                        DEPENDS "/home/saop/code/python/fire_rs/display.py" ${DEPS})

#     add_custom_target(target ALL DEPENDS ${OUTPUT})

#     install(CODE "execute_process(COMMAND ${PYTHON} ${SETUP_PY} install)")
# endif()
