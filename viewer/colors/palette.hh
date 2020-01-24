#pragma once

namespace viewer {

// One could imagine a palette type
// TODO
class Palette {
  // A palette defines

  virtual jcc::Vec4 base03() const;
  virtual jcc::Vec4 base02() const;
  virtual jcc::Vec4 base01() const;
  virtual jcc::Vec4 base00() const;
  virtual jcc::Vec4 base0() const;
  virtual jcc::Vec4 base1() const;
  virtual jcc::Vec4 base2() const;
  virtual jcc::Vec4 base3() const;
  virtual jcc::Vec4 yellow() const;
  virtual jcc::Vec4 orange() const;
  virtual jcc::Vec4 red() const;
  virtual jcc::Vec4 magenta() const;
  virtual jcc::Vec4 violet() const;
  virtual jcc::Vec4 blue() const;
  virtual jcc::Vec4 cyan() const;
  virtual jcc::Vec4 green() const;
};

}  // namespace viewer