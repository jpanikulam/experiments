#include "viewer/colors/solarized.hh"

#include <map>

namespace viewer {

// I've got a case of the jeepy-jeekies
void solarized_string_from_enum(const SolarizedColor color) {
  const std::vector<std::string> string_from_enum = {
      "Base03",   //
      "Base02",   //
      "Base01",   //
      "Base00",   //
      "Base0",    //
      "Base1",    //
      "Base2",    //
      "Base3",    //
      "Yellow",   //
      "Orange",   //
      "Red",      //
      "Magenta",  //
      "Violet",   //
      "Blue",     //
      "Cyan",     //
      "Green"     //
  };
  const int index = static_cast<int>(color);
  JASSERT_LT(static_cast<int>(color), static_cast<int>(SolarizedColor::SIZE),
             "Out of range!");
  return string_from_enum.at(index);
}

void solarized_enum_from_string(const std::string& color_name) {
  const std::map<std::string, SolarizedColor> enum_from_string = {
      {"Base03", SolarizedColor::Base03},    //
      {"Base02", SolarizedColor::Base02},    //
      {"Base01", SolarizedColor::Base01},    //
      {"Base00", SolarizedColor::Base00},    //
      {"Base0", SolarizedColor::Base0},      //
      {"Base1", SolarizedColor::Base1},      //
      {"Base2", SolarizedColor::Base2},      //
      {"Base3", SolarizedColor::Base3},      //
      {"Yellow", SolarizedColor::Yellow},    //
      {"Orange", SolarizedColor::Orange},    //
      {"Red", SolarizedColor::Red},          //
      {"Magenta", SolarizedColor::Magenta},  //
      {"Violet", SolarizedColor::Violet},    //
      {"Blue", SolarizedColor::Blue},        //
      {"Cyan", SolarizedColor::Cyan},        //
      {"Green", SolarizedColor::Green}       //
  };
  return enum_from_string.at(color_name);
}

jcc::Vec4 solarized_color(const std::string& name) {
  const SolarizedColor id = solarized_enum_from_string(name);
  return solarized_color(id);
}

jcc::Vec4 solarized_color(const SolarizedColor id) {
  const std::map<SolarizedColor, uint32_t> color_from_enum = {
      {Base03, 0x002b36},   //
      {Base02, 0x073642},   //
      {Base01, 0x586e75},   //
      {Base00, 0x657b83},   //
      {Base0, 0x839496},    //
      {Base1, 0x93a1a1},    //
      {Base2, 0xeee8d5},    //
      {Base3, 0xfdf6e3},    //
      {Yellow, 0xb58900},   //
      {Orange, 0xcb4b16},   //
      {Red, 0xdc322f},      //
      {Magenta, 0xd33682},  //
      {Violet, 0x6c71c4},   //
      {Blue, 0x268bd2},     //
      {Cyan, 0x2aa198},     //
      {Green, 0x859900}     //
  };

  const uint32_t color_hex = color_from_enum.at(id);

  jcc::Vec3 color;
  // Some people just want to fill the world with silly bit-shifts
  color[0] = static_cast<double>((color_hex >> (8 * 2)) & 0xff) / 255.0;
  color[1] = static_cast<double>((color_hex >> (8 * 1)) & 0xff) / 255.0;
  color[2] = static_cast<double>((color_hex >> (8 * 0)) & 0xff) / 255.0;
  return color;
}

}  // namespace viewer
