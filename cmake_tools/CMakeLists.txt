# TODO: Check Version
find_package (PythonInterp COMPONENTS Interpreter Development)

macro(add_cl_gen cl_gen_name file_path)
    add_custom_command(
        COMMAND ${PYTHON_EXECUTABLE} ${file_path}
        DEPENDS ${file_path}
        OUTPUT ${generated_path}/${cl_gen_name}.hh ${generated_path}/${cl_gen_name}.clh ${cl_gen_name}_x
        COMMENT "Generating code for ${cl_gen_name}"
    )

    add_custom_target(
        ${cl_gen_name} ALL
        DEPENDS ${cl_gen_name}_x
    )
endmacro()

macro(add_test test_name test_file test_libs)
    add_executable(
        ${test_name}
        ${test_file}
    )
    set_target_properties(
        ${test_name}
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test
    )
    set(AUGLIBS ${GTEST_LIBRARIES} pthread ${test_libs})
    target_link_libraries(${test_name} ${AUGLIBS})
endmacro(add_test)
