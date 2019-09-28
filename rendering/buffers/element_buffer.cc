#include "rendering/buffers/element_buffer.hh"

namespace jcc {

ElementBuffer::ElementBuffer() {
  glGenBuffers(1, &buffer_id_);
  bind();
}

ElementBuffer::ElementBuffer(const std::vector<Vec3ui32>& indices) {
  glGenBuffers(1, &buffer_id_);
  bind();

  // TODO: maybe this will change?
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * (3 * 4), indices.data(),
               GL_STATIC_DRAW);
  n_faces_ = indices.size();
}

ElementBuffer::~ElementBuffer() {
  if (buffer_id_ != 0) {
    destroy();
  }
}

void ElementBuffer::draw_elements() const {
  bind();
  constexpr int VERTICES_PER_FACE = 3;
  glDrawElements(GL_TRIANGLES, n_faces_ * VERTICES_PER_FACE, GL_UNSIGNED_INT, (void*)0);
}

void ElementBuffer::bind() const {
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer_id_);
}
void ElementBuffer::destroy() {
  glDeleteBuffers(1, &buffer_id_);
  buffer_id_ = 0;
  n_faces_ = 0;
}

}  // namespace jcc
