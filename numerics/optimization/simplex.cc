//%ignore()
#include "eigen.hh"

namespace numerics {

class VariableHandle {
 public:
  VariableHandle(int id) : id_(id) {
  }

 private:
  int id_;
};

class LinearProgram {
  struct Value {
    VariableHandle handle;
    double scaling;
  };

  struct ImplicitLinearConstraint {
    std::vector<Value> values;
    int greater_than;
  };

  LinearProgram() = default;

  template <int N>
  int sum(const std::array<Value, N>& vals) {

  }

  VariableHandle new_variable() {
    return VariableHandle{++handles_};
  }

  void greater_than(const VariableHandle& a, const) {
  }

 private:
  int handles_ = 0;
};

void go() {
  LinearProgram::Value x1{1, 1.0};
  LinearProgram::Value x2{2, 1.0};
  LinearProgram::Value x3{3, 1.0};

  VariableHandle
}
}  // namespace numerics

int main() {
  numerics::go();
}
