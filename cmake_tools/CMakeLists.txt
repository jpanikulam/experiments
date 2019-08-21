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