#pragma once

//
// Retains no ownership over the object
//
template <class T>
struct Out {
  explicit Out(T& obj) : _obj(obj) {
  }
  T& operator*() {
    return _obj;
  }
  T* operator->() {
    return &_obj;
  }

 private:
  T& _obj;
};

template <class T>
Out<T> out(T& obj) {
  return Out<T>(obj);
}