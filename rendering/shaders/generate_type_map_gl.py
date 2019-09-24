
mapping = {
    "GL_FLOAT": (1, "GLfloat", "float", "glUniform1f"),
    "GL_FLOAT_VEC2": (2, "GLfloat", "VecNf<2>", "glUniform2fv"),
    "GL_FLOAT_VEC3": (3, "GLfloat", "VecNf<3>", "glUniform3fv"),
    "GL_FLOAT_VEC4": (4, "GLfloat", "VecNf<4>", "glUniform4fv"),

    "GL_INT": (1, "GLint"),
    "GL_INT_VEC2": (2, "GLint"),
    "GL_INT_VEC3": (3, "GLint"),
    "GL_INT_VEC4": (4, "GLint"),
    "GL_UNSIGNED_INT": (1, "GLuint"),
    "GL_UNSIGNED_INT_VEC2": (2, "GLuint"),
    "GL_UNSIGNED_INT_VEC3": (3, "GLuint"),
    "GL_UNSIGNED_INT_VEC4": (4, "GLuint"),

    "GL_BOOL": (1, "GLboolean"),
    "GL_BOOL_VEC2": (2, "GLboolean"),
    "GL_BOOL_VEC3": (3, "GLboolean"),
    "GL_BOOL_VEC4": (4, "GLboolean"),

    "GL_FLOAT_MAT2": (4, "GLfloat", "MatNf<2, 2>", "glUniformMatrix2fv"),
    "GL_FLOAT_MAT2x3": (6, "GLfloat", "MatNf<2, 3>", "glUniformMatrix2x3fv"),
    "GL_FLOAT_MAT2x4": (8, "GLfloat", "MatNf<2, 4>", "glUniformMatrix2x4fv"),
    "GL_FLOAT_MAT3": (9, "GLfloat", "MatNf<3, 3>", "glUniformMatrix3fv"),
    "GL_FLOAT_MAT3x2": (6, "GLfloat", "MatNf<3, 2>", "glUniformMatrix3x2fv"),
    "GL_FLOAT_MAT3x4": (12, "GLfloat", "MatNf<3, 4>", "glUniformMatrix3x4fv"),
    "GL_FLOAT_MAT4": (16, "GLfloat", "MatNf<4, 4>", "glUniformMatrix4fv"),
    "GL_FLOAT_MAT4x2": (8, "GLfloat", "MatNf<4, 2>", "glUniformMatrix4x2fv"),
    "GL_FLOAT_MAT4x3": (12, "GLfloat", "MatNf<4, 3>", "glUniformMatrix4x3fv"),
}


list_content_type = {
    "GL_FLOAT": "GL_FLOAT",
    "GL_FLOAT_VEC2": "GL_FLOAT",
    "GL_FLOAT_VEC3": "GL_FLOAT",
    "GL_FLOAT_VEC4": "GL_FLOAT",

    "GL_INT": "GL_INT",
    "GL_INT_VEC2": "GL_INT",
    "GL_INT_VEC3": "GL_INT",
    "GL_INT_VEC4": "GL_INT",
    "GL_UNSIGNED_INT": "GL_UNSIGNED_INT",
    "GL_UNSIGNED_INT_VEC2": "GL_UNSIGNED_INT",
    "GL_UNSIGNED_INT_VEC3": "GL_UNSIGNED_INT",
    "GL_UNSIGNED_INT_VEC4": "GL_UNSIGNED_INT",

    "GL_BOOL": "GL_BOOL",
    "GL_BOOL_VEC2": "GL_BOOL",
    "GL_BOOL_VEC3": "GL_BOOL",
    "GL_BOOL_VEC4": "GL_BOOL",

    "GL_FLOAT_MAT2": "GL_FLOAT",
    "GL_FLOAT_MAT2x3": "GL_FLOAT",
    "GL_FLOAT_MAT2x4": "GL_FLOAT",
    "GL_FLOAT_MAT3": "GL_FLOAT",
    "GL_FLOAT_MAT3x2": "GL_FLOAT",
    "GL_FLOAT_MAT3x4": "GL_FLOAT",
    "GL_FLOAT_MAT4": "GL_FLOAT",
    "GL_FLOAT_MAT4x2": "GL_FLOAT",
    "GL_FLOAT_MAT4x3": "GL_FLOAT",

}


def make_uniform(type_enum, xdt):
    n_elements, type_name, cc_type, function = xdt
    txt = "void Shader::set(const std::string& name, const {}& arg) const ".format(cc_type) + "{"

    txt += """
  const std::string err_str = name + " was not available";
  if (debug_mode_) {{
    if (0u == uniform_from_name_.count(name)) {{
        jcc::Warning() << "Not using " << name << " in shader" << std::endl;
        return;
    }}
  }} else {{
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }}
""".format()
    txt += "\n  const auto& desc = uniform_from_name_.at(name);"
    txt += '\n  JASSERT_EQ(desc.type, static_cast<int>({type_enum}), "Mismatched argument type");'.format(
        type_enum=type_enum
    )

    if n_elements > 1:
        if 'MatNf' in cc_type:
            txt += "\n  {fnc}(desc.location, 1, false, arg.data());".format(fnc=function)
        else:
            txt += "\n  {fnc}(desc.location, 1, arg.data());".format(fnc=function)
    else:
        txt += "\n  {fnc}(desc.location, arg);".format(fnc=function)

    txt += "\n}"
    return txt


def line(txt, depth=0):
    return ("  " * depth) + txt + ";\n"


def make_attribute(type_enum, xdt):
    n_elements, type_name, cc_type, function = xdt
    txt = "void VertexArrayObject::set(const std::string& name, const std::vector<{}>& arg) ".format(cc_type) + "{"

    txt += """
  const std::string err_str = name + " was not available";
  if (debug_mode_) {{
    if (0u == attribute_from_name_.count(name)) {{
        jcc::Warning() << "Not using " << name << " in shader" << std::endl;
        return;
    }}
  }} else {{
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }}
""".format()
    txt += "\n  const auto& desc = attribute_from_name_.at(name);"
    txt += '\n  JASSERT_EQ(desc.type, static_cast<int>({type_enum}), "Mismatched argument type");'.format(
        type_enum=type_enum
    )

    txt += line("glBindVertexArray(vao_)", 1)

    txt += line("GLuint vbo_id", 1)
    txt += line("glGenBuffers(1, &vbo_id)", 1)
    txt += line("glBindBuffer(GL_ARRAY_BUFFER, vbo_id)", 1)

    buffer_data_args_text = "arg.size() * {n_elements} * sizeof({type_name})".format(
        n_elements=n_elements,
        type_name=type_name
    )

    txt += line("glBufferData(GL_ARRAY_BUFFER, " + buffer_data_args_text + ", arg.data(), GL_STATIC_DRAW)", 1)
    txt += line("constexpr GLint SIZE = 3", 1)
    txt += line("constexpr bool NORMALIZED = false", 1)
    txt += line("constexpr int STRIDE = 0", 1)
    txt += line("glVertexAttribPointer(desc.location, SIZE, {type_enum}, NORMALIZED, STRIDE, 0)".format(
        type_enum=list_content_type[type_enum]), 1
    )

    txt += line("glEnableVertexAttribArray(desc.location)", 1)
    txt += line("allocated_buffers_.push_back(vbo_id)", 1)
    txt += "}"
    return txt


def uniforms():
    for type_enum, data in mapping.items():
        if len(data) > 2:
            print make_uniform(type_enum, data)


def attributes():
    for type_enum, data in mapping.items():
        if len(data) > 2:
            print make_attribute(type_enum, data)


if __name__ == '__main__':
    # uniforms()
    attributes()
