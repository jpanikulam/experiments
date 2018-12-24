#include "estimation/jet/jet_ekf.hh"
#include "estimation/filter_impl.hh"

namespace estimation {

template class Ekf<jet_filter::State>;
template class ObservationModel<jet_filter::State, jet_filter::AccelMeasurement>;

}  // namespace estimation