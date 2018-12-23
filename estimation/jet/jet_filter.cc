#include "estimation/jet/jet_filter.hh"

namespace estimation {
namespace jet_filter {

class JetFilter() {
  void measure_imu(const ImuMeasurement& meas) {
    imu_measurements_.push(meas);
  }

  void measure_fiducial(const FiducialMeasurement& meas) {
    fiducial_measurements_.push(meas);
  }

 private:
  void observe_imu() const;
  void observe_fiducial() const;

  void handle(int i) {
    switch (i) {
      case 0:
        const FilterUpdate update = observe_imu(x, imu_measurements_.top());
        imu_measurements_.pop();
        break;
      case 1:
        const FilterUpdate update = observe_fiducial(x, fiducial_measurements_.top());
        fiducial_measurements_.pop();
        break;
    }
  }

 private:
  std::priority_queue<ImuMeasurement> imu_measurements_;
  std::priority_queue<FiducialMeasurement> fiducial_measurements_;

  std::priority_queue<Measurement> measurements_;
}

}  // namespace jet_filter
}  // namespace estimation
