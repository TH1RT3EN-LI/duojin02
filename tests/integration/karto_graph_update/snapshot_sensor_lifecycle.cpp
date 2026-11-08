#include <karto_sdk/Karto.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>

static void require(bool value, const char * message) {
  if (!value) throw std::runtime_error(message);
}
static karto::Sensor * registered(const karto::Name & name) {
  try {return karto::SensorManager::GetInstance()->GetSensorByName(name);}
  catch (const karto::Exception &) {return nullptr;}
}
int main(int argc, char ** argv) {
  if (argc != 2) return 2;
  const auto directory = std::filesystem::path(argv[1]);
  std::filesystem::create_directories(directory);
  const auto file = (directory / "sensors.data").string();
  const karto::Name name("snapshot_sensor_identity");
  auto live = std::make_unique<karto::Dataset>();
  auto * sensor = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name);
  live->Add(sensor);
  live->SaveToFile(file);
  {
    karto::Dataset rejected_stage;
    rejected_stage.LoadFromFile(file);
    require(registered(name) == sensor, "Load must not alter live registry");
    require(rejected_stage.GetLasers().front() != sensor, "Staging sensor must be a distinct object");
  }
  require(registered(name) == sensor, "Rejected same-name stage removed live sensor");
  auto replacement = std::make_unique<karto::Dataset>();
  replacement->LoadFromFile(file);
  auto * next = dynamic_cast<karto::Sensor *>(replacement->GetLasers().front());
  karto::SensorManager::GetInstance()->RegisterSensor(next, true);
  live.reset();
  require(registered(name) == next, "Old dataset destruction removed replacement sensor");
  replacement->Clear();
  require(registered(name) == nullptr, "Current dataset did not release its own sensor");
  replacement->Clear();
  replacement.reset();
  {
    karto::Dataset unregistered_stage;
    unregistered_stage.LoadFromFile(file);
    require(registered(name) == nullptr, "Unregistered staging unexpectedly registered sensor");
  }
  require(registered(name) == nullptr, "Absent-name staging destruction changed registry");
  std::ofstream(directory / "result.json") <<
    "{\"passed\":true,\"same_name_rejected_stage_preserves_live\":true,"
    "\"old_dataset_preserves_replacement\":true,\"repeated_clear_safe\":true,"
    "\"absent_name_stage_destruction_safe\":true}\n";
}
