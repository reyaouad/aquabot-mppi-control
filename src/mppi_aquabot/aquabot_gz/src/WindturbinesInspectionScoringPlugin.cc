#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/msgs/float.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include "WindturbinesInspectionScoringPlugin.hh"
#include "vrx/WaypointMarkers.hh"


#include "nlohmann/json.hpp"
#include <fstream>

using namespace gz;
using namespace vrx;

/// \brief Private ScoringPlugin data class.
class WindturbinesInspectionScoringPlugin::Implementation
{
  /// \brief The position of the pinger.
  public: math::Vector3d pingerPosition;

  /// \brief Phase Search : If the time is over, the search is finished
  public: double phaseSearchMaxTime = 600.0; // 4 minutes

  /// \brief Phase Rally : Min distance to the critical windturbine
  public: double phaseRallyMinDistance = 15.0;

  /// \brief Phase Stabilize : Total time
  public: double phaseStabilizeTotalTime = 180.0;

  /// \brief Phase TurnAround : Number of 360° turns, -1 = infinite
  public: double phaseTurnAroundNumberTurns = -1.0;

  /// \brief Phases Stabilize and TurnAround : Target distance
  public: double phaseInspectionTargetDistance = 10.0;

  /// \brief Phase Stabilize and TurnAround : Period to save mean values
  public: int phaseStabilizeTurnAroundMeanPeriod = 30;

  /// \brief Minimum distance with collision objects. (only actors boats)
  public: double collisionMinDistance = 5;

  /// \brief Finish if collision.
  public: bool collisionFinish = true;

  /// \brief Collision penality
  public: double collisionPenality = 0;

  /// \brief Windturbines objects
  public: std::vector<std::string> windturbinesNames;

  /// \brief Critical windturbine marker name
  public: std::string criticalMarkerName;

  /// \brief Windturbines status
  public: std::vector<std::string> windturbinesStatus;

  /// \brief Ais windturbines period
  public: double aisWindturbinesPeriod = 3.0;

  /// \brief Transport node.
  public: transport::Node node{transport::NodeOptions()};

  /// \brief Entity of the vehicle used.
  public: sim::Entity playerEntity;

  /// \brief Entity of the windturbines used.
  public: std::vector<sim::Entity> windturbinesEntity;

  /// \brief Entity of the critical windturbine marker.
  public: sim::Entity criticalMarkerEntity;

  /// \brief Waypoint visualization markers
  public: WaypointMarkers waypointMarkers{"pinger_marker"};

  /// \brief Debug topic where heading is published
  public: std::string topicCriticalHeading = "/vrx/windturbinesinspection/debug/critical_windturbine_heading";

  /// \brief Debug topic where critical windturbine distance is published
  public: std::string topicCriticalWindturineDistance = "/vrx/windturbinesinspection/debug/critical_windturbine_distance";

  /// \brief Debug topic where critical windturbine orientation is published
  public: std::string topiccriticalWindturbineBearingFromBoat = "/vrx/windturbinesinspection/debug/critical_windturbine_bearing_from_boat";

  /// \brief Debug topic where critical target pose distance is published
  public: std::string topicCriticalTargetPoseDistance = "/vrx/windturbinesinspection/debug/critical_target_pose_distance";

  /// \brief Debug topic where critical turn around angle is published
  public: std::string topicCriticalTurnAroundAngle = "/vrx/windturbinesinspection/debug/critical_turn_around_angle";

  /// \brief Debug topic where Windturbines distances is published
  public: std::string topicWindturbinesDistances = "/vrx/windturbinesinspection/debug/windturbines_distances";

  /// \brief Debug topic where Windturbines checkup status ok is published
  public: std::string topicWindturbineCheckupStatusOk = "/vrx/windturbinesinspection/debug/windturbine_checkup_status_ok";

  /// \brief Debug topic where Windturbines checkup status not received is published
  public: std::string topicWindturbineCheckupStatusNotReceived = "/vrx/windturbinesinspection/debug/windturbine_checkup_status_not_received";

  /// \brief Topic where current phase is published
  public: std::string topicCurrentPhase = "/vrx/windturbinesinspection/current_phase";

  /// \brief Topic where windturbines positions is published
  public: std::string topicAisWindturbinesPositions = "/aquabot/ais_sensor/windturbines_positions";

  /// \brief Topic where windturbines checkups are received
  public: std::string topicCheckUp = "/vrx/windturbinesinspection/windturbine_checkup";

  /// \brief Debug publisher for the critical heading.
  public: transport::Node::Publisher criticalHeadingPub;

  /// \brief Debug publisher for the critical windturbine distance.
  public: transport::Node::Publisher criticalDistancePub;

  /// \brief Debug publisher for the critical windturbine bearing.
  public: transport::Node::Publisher criticalWindturbineBearingFromBoatPub;

  /// \brief Debug publisher for the critical target pose distance.
  public: transport::Node::Publisher criticalTargetPoseDistancePub;

  /// \brief Debug publisher for the critical turn around angle.
  public: transport::Node::Publisher criticalTurnAroundAnglePub;

  /// \brief Debug publisher for the windturbines distances.
  public: transport::Node::Publisher windturbinesDistancesPub;

  /// \brief Debug publisher for the windturbines checkup status ok.
  public: transport::Node::Publisher windturbineCheckupStatusOkPub;

  /// \brief Debug publisher for the windturbines checkup status not received.
  public: transport::Node::Publisher windturbineCheckupStatusNotReceivedPub;

  /// \brief Publisher for the bool second task started.
  public: transport::Node::Publisher currentPhasePub;

  /// \brief Publisher for the windturbines positions.
  public: transport::Node::Publisher aisWindturbinesPositionsPub;

  /// \brief Publisher to set the pinger position.
  public: transport::Node::Publisher setPingerPub;

  /// \brief Number of collisions
  public: int collisions = 0;

  /// \brief Time when phase search finished.
  public: std::chrono::duration<double> timePhaseSearch
    = std::chrono::duration<double>::zero();

  /// \brief Time to finish rally finished.
  public: std::chrono::duration<double> timePhaseRally 
    = std::chrono::duration<double>::zero();

  /// \brief Time to finish stabilize
  public: std::chrono::duration<double> timePhaseStabilize
    = std::chrono::duration<double>::zero();

  /// \brief Time to finish turn around
  public: std::chrono::duration<double> timePhaseTurnAround
    = std::chrono::duration<double>::zero();

  /// \brief Last collision time
  public: std::chrono::duration<double> lastCollisionTime 
    = std::chrono::duration<double>::zero();

  /// \brief Last time process speed
  public: std::chrono::duration<double> lastTimeProcessSpeed
    = std::chrono::duration<double>::zero();

  /// \brief Last time publish AIS windturbines
  public: std::chrono::duration<double> lastTimePubAisWindturbines
    = std::chrono::duration<double>::zero();

  /// \brief Last time public debug topics
  public: std::chrono::duration<double> lastTimeProcessPublish
    = std::chrono::duration<double>::zero();

  /// \brief Last time we published a pinger position.
  public: std::chrono::duration<double> lastSetPingerPositionTime
    = std::chrono::duration<double>::zero();

  /// \brief Last time saved values for mean values
  public: std::chrono::duration<double> lastTimeSavedMeanValues
    = std::chrono::duration<double>::zero();

  /// \brief Last player position for speed computation
  public: math::Pose3d lastPlayerPose;

  /// \brief Last log csv data
  public: WindturbinesInspectionDataStruct lastDataSaved;

  /// \brief Spherical coordinate conversions. 
  public: math::SphericalCoordinates sc; 

  /// \brief World sim
  public: sim::World world;

  /// \brief Log Info bool
  public: bool logCsvInfo = true;

  /// \brief Log CSV file
  public: std::ofstream logCsvFile;

  /// \brief Topic CheckUp CSV file
  public: std::ofstream logCsvCheckUpFile;

  /// \brief Log TXT file
  public: std::ofstream logTxtFile;

  /// \brief Register a new windturbine checkup received.
  /// \param[in] _msg The message containing a windturbine checkup.
  public: void OnWindturbineCheckUpReceived(const msgs::StringMsg &_msg);

  /// \brief Set the pinger location.
  public: void SetPingerPosition();

  /// \brief Current vector of perception requests to be processed.
  public: std::vector<msgs::StringMsg> requests;

  /// \brief Mutex to protect the requests.
  public: std::mutex mutex;
};

//////////////////////////////////////////////////
void WindturbinesInspectionScoringPlugin::Implementation::OnWindturbineCheckUpReceived(const msgs::StringMsg &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->requests.push_back(_msg);
}

//////////////////////////////////////////////////
void WindturbinesInspectionScoringPlugin::Implementation::SetPingerPosition()
{
  msgs::Vector3d position;
  position.set_x(this->pingerPosition.X());
  position.set_y(this->pingerPosition.Y());
  position.set_z(this->pingerPosition.Z());
  this->setPingerPub.Publish(position);
}

/////////////////////////////////////////////////
WindturbinesInspectionScoringPlugin::WindturbinesInspectionScoringPlugin()
  : ScoringPlugin(),
  dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  gzmsg << "WindturbinesInspectionScoringPlugin scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void WindturbinesInspectionScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  // Initialize spherical coordinates instance.
  auto worldEntity = _ecm.EntityByComponents(sim::components::World());
  sim::World world(worldEntity);
  this->dataPtr->world = world;
  this->dataPtr->sc = this->dataPtr->world.SphericalCoordinates(_ecm).value();

  // Optional log csv info.
  if (_sdf->HasElement("log_csv_info"))
    this->dataPtr->logCsvInfo = _sdf->Get<bool>("log_csv_info");

  // Initialize log files
  auto currentTime = std::chrono::system_clock::now();
  std::time_t currentTime_t = std::chrono::system_clock::to_time_t(currentTime);
  struct tm *localTime = std::localtime(&currentTime_t);
  // Format the date and time in the format YY-MM-DD_HH-mm-ss
  std::stringstream formattedTime;
  formattedTime << std::put_time(localTime, "%y-%m-%d_%H_%M_%S");
  std::string formattedTimeString = formattedTime.str();

  std::string colcon_install_path = std::getenv("COLCON_PREFIX_PATH");
  std::string csvFileName = colcon_install_path + std::string("/../log/wintubinesinspection_") 
    + this->dataPtr->world.Name(_ecm).value_or("no_name") + "_" + formattedTimeString + ".csv";
  std::string csvCheckUpFileName = colcon_install_path + std::string("/../log/wintubinesinspection_checkup_") 
    + this->dataPtr->world.Name(_ecm).value_or("no_name") + "_" + formattedTimeString + ".csv";
  std::string txtFileName = colcon_install_path + std::string("/../log/wintubinesinspection_") 
    + this->dataPtr->world.Name(_ecm).value_or("no_name") + "_" + formattedTimeString + ".txt";
  gzmsg << "WindturbinesInspectionScoringPlugin log csv saved to " << csvFileName << std::endl;
  gzmsg << "WindturbinesInspectionScoringPlugin log csv checkup savec to " << csvCheckUpFileName << std::endl;
  gzmsg << "WindturbinesInspectionScoringPlugin log txt savec to " << txtFileName << std::endl;

  // Create log txt file
  this->dataPtr->logTxtFile.open(txtFileName);

  // Write txt header
  this->dataPtr->logTxtFile << "[INFO] Windturbines Inspection Scoring Plugin" << std::endl;

  // Set the topic to be used to publish the sensor message.
  std::string setPositionTopicName = "/pinger/set_pinger_position";
  if (_sdf->HasElement("set_position_topic_name"))
  {
    setPositionTopicName = _sdf->Get<std::string>("set_position_topic_name");
  }

  // Required pinger position.
  if (!_sdf->HasElement("pinger_position"))
  {
    gzerr << "Unable to find <pinger_position>" << std::endl;
    return;
  }

  this->dataPtr->pingerPosition = _sdf->Get<math::Vector3d>("pinger_position");
  this->dataPtr->logTxtFile << "[CONF] pinger_position = (" + std::to_string(this->dataPtr->pingerPosition.X()) + "," 
            + std::to_string(this->dataPtr->pingerPosition.Y()) + "," 
            + std::to_string(this->dataPtr->pingerPosition.Z()) + ")" << std::endl;
  
  // Optional pinger marker.
  if (_sdf->HasElement("markers"))
  {
    auto markersElement = _sdf->Clone()->GetElement("markers");

    this->dataPtr->waypointMarkers.Load(markersElement);
    if (!this->dataPtr->waypointMarkers.DrawMarker(0,
           this->dataPtr->pingerPosition.X(), this->dataPtr->pingerPosition.Y(),
           this->dataPtr->pingerPosition.Z()))
    {
      gzerr << "Error creating visual marker" << std::endl;
    }
  }
  
  if (_sdf->HasElement("phase_search_max_time"))
    this->dataPtr->phaseSearchMaxTime = _sdf->Get<double>("phase_search_max_time");
  this->dataPtr->logTxtFile << "[CONF] phase_search_max_time = " + std::to_string(this->dataPtr->phaseSearchMaxTime) << std::endl;

  if (_sdf->HasElement("phase_rally_min_distance"))
    this->dataPtr->phaseRallyMinDistance = _sdf->Get<double>("phase_rally_min_distance");
  this->dataPtr->logTxtFile << "[CONF] phase_rally_min_distance = " + std::to_string(this->dataPtr->phaseRallyMinDistance) << std::endl;

  if (_sdf->HasElement("phase_stabilize_total_time"))
    this->dataPtr->phaseStabilizeTotalTime = _sdf->Get<double>("phase_stabilize_total_time");
  this->dataPtr->logTxtFile << "[CONF] phase_stabilize_total_time = " + std::to_string(this->dataPtr->phaseStabilizeTotalTime) << std::endl;

  if (_sdf->HasElement("phase_turnaround_number_turns"))
    this->dataPtr->phaseTurnAroundNumberTurns = _sdf->Get<double>("phase_turnaround_number_turns");
  this->dataPtr->logTxtFile << "[CONF] phase_turnaround_number_turns = " + std::to_string(this->dataPtr->phaseTurnAroundNumberTurns) << std::endl;

  if (_sdf->HasElement("phase_inspection_target_distance"))
    this->dataPtr->phaseInspectionTargetDistance = _sdf->Get<double>("phase_inspection_target_distance");
  this->dataPtr->logTxtFile << "[CONF] phase_inspection_target_distance = " + std::to_string(this->dataPtr->phaseInspectionTargetDistance) << std::endl;

  if (_sdf->HasElement("phase_stabilize_turnaround_mean_period"))
    this->dataPtr->phaseStabilizeTurnAroundMeanPeriod = _sdf->Get<int>("phase_stabilize_turnaround_mean_period");
  this->dataPtr->logTxtFile << "[CONF] phase_stabilize_turnaround_mean_period = " + std::to_string(this->dataPtr->phaseStabilizeTurnAroundMeanPeriod) << std::endl;

  if (_sdf->HasElement("collision_min_distance"))
    this->dataPtr->collisionMinDistance = _sdf->Get<double>("collision_min_distance");
  this->dataPtr->logTxtFile << "[CONF] collision_min_distance = " + std::to_string(this->dataPtr->collisionMinDistance) << std::endl;

  if (_sdf->HasElement("collision_finish"))
    this->dataPtr->collisionFinish = _sdf->Get<bool>("collision_finish");
  this->dataPtr->logTxtFile << "[CONF] collision_finish = " + std::to_string(this->dataPtr->collisionFinish) << std::endl;

  if (_sdf->HasElement("collision_penality"))
    this->dataPtr->collisionPenality = _sdf->Get<double>("collision_penality");
  this->dataPtr->logTxtFile << "[CONF] collision_penality = " + std::to_string(this->dataPtr->collisionPenality) << std::endl;

  if (_sdf->HasElement("windturbines"))
  {
    auto windturbinesElement = _sdf->Clone()->GetElement("windturbines");
    if(windturbinesElement->HasElement("windturbine"))
    {
      auto objectElem = windturbinesElement->GetElement("windturbine");
      while (objectElem)
      {
        auto objectName = objectElem->Get<std::string>("name");
        this->dataPtr->windturbinesNames.push_back(objectName);
        auto objectStatus = objectElem->Get<std::string>("status");
        this->dataPtr->windturbinesStatus.push_back(objectStatus);
        this->dataPtr->logTxtFile << "[CONF] windturbine = " + objectName + ", status = " + objectStatus << std::endl;
        objectElem = objectElem->GetNextElement("windturbine");
      }
    }
  }

  if (_sdf->HasElement("critical_marker_name"))
    this->dataPtr->criticalMarkerName = _sdf->Get<std::string>("critical_marker_name");
  this->dataPtr->logTxtFile << "[CONF] critical_marker_name = " + this->dataPtr->criticalMarkerName << std::endl;


  // Declare publishers
  transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(1u);

  // Debug topic critical wintubine heading : Heading between player and critical windturbine (not published on competitive mode)
  this->dataPtr->criticalHeadingPub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topicCriticalHeading, opts);

  // Debug topic critical wintubine distance : Distance between player and critical windturbine (not published on competitive mode)
  this->dataPtr->criticalDistancePub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topicCriticalWindturineDistance, opts);

  // Debug topic critical wintubine orientation : Orientation between player and critical windturbine (not published on competitive mode)
  this->dataPtr->criticalWindturbineBearingFromBoatPub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topiccriticalWindturbineBearingFromBoat, opts);

  // Debug topic critical target pose distance : Distance between player and critical target pose (not published on competitive mode)
  this->dataPtr->criticalTargetPoseDistancePub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topicCriticalTargetPoseDistance, opts);
    
  // Debug topic critical turn around angle : Angle between player and critical windturbine (not published on competitive mode)
  this->dataPtr->criticalTurnAroundAnglePub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->topicCriticalTurnAroundAngle, opts);

  // Debug topic windturbine distances : Distances between player and windturbines (not published on competitive mode)
  this->dataPtr->windturbinesDistancesPub = this->dataPtr->node.Advertise<msgs::Float_V>(
    this->dataPtr->topicWindturbinesDistances, opts);

  // Debug topic wintubine inspection status : Checkup status 0 = NOT RECEIVED, 1 = GOOD, 2 = BAD (not published on competitive mode)
  this->dataPtr->windturbineCheckupStatusOkPub = this->dataPtr->node.Advertise<msgs::UInt32>(
    this->dataPtr->topicWindturbineCheckupStatusOk, opts);

  // Debug topic wintubine inspection status : Checkup status 0 = NOT RECEIVED, 1 = GOOD, 2 = BAD (not published on competitive mode)
  this->dataPtr->windturbineCheckupStatusNotReceivedPub = this->dataPtr->node.Advertise<msgs::UInt32>(
    this->dataPtr->topicWindturbineCheckupStatusNotReceived, opts);

  // Topic current phase : UInt32 with id of the current phase
  this->dataPtr->currentPhasePub = this->dataPtr->node.Advertise<msgs::UInt32>(
    this->dataPtr->topicCurrentPhase, opts);

  // Topic windturbines positions : GPS Position of the target
  this->dataPtr->aisWindturbinesPositionsPub = this->dataPtr->node.Advertise<msgs::Pose>(
    this->dataPtr->topicAisWindturbinesPositions, opts);

  // Topic set pinger position : Set the pinger position
  this->dataPtr->setPingerPub = this->dataPtr->node.Advertise<msgs::Vector3d>(setPositionTopicName, opts);

  // Create log csv file
  if (this->dataPtr->logCsvInfo)
  {
    this->dataPtr->logCsvFile.open(csvFileName);

    // Write csv header
    std::string line = "Time,Phase";
    line += ",Player X,Player Y,Player Yaw,Player Speed,Player Distance";
    line += ",Target Pose X,Target Pose Y,Target Pose Yaw,Target Pose Distance";
    line += ",Critical Windturbine X,Critical Windturbine Y";
    line += ",Critical Windturbine Distance,Critical Windturbine Heading";
    line += ",Critical Windturbine Bearing From Boat,Critical Windturbine Number Turns";
    int i = 0;
    for(auto windturbineName : this->dataPtr->windturbinesNames)
    {
      std::string name = "Windturbine " + std::to_string(i);
      line += "," + name + " X," + name + " Y," + name + " Distance," + name + " Checkup Status";
      i++;
    }
    this->dataPtr->logCsvFile << line << std::endl;
  }

  gzmsg << "WindturbinesInspectionScoringPlugin::Configured" << std::endl;
}

//////////////////////////////////////////////////
void WindturbinesInspectionScoringPlugin::PreUpdate( const sim::UpdateInfo &_info, sim::EntityComponentManager &_ecm)
{
  // Don't update when paused
  if (_info.paused)
    return;

  ScoringPlugin::PreUpdate(_info, _ecm);

  if (this->ScoringPlugin::TaskState() != "running")
    return;

  WindturbinesInspectionDataStruct currentData = this->dataPtr->lastDataSaved;
  currentData.time = this->ElapsedTime();

  // Set the pinger position again every 10 seconds.
  std::chrono::duration<double> elapsedTime = 
    currentData.time - this->dataPtr->lastSetPingerPositionTime;

  if (elapsedTime >= std::chrono::duration<double>(10.0) && 
    currentData.phase > WindturbinesInspectionState::Phase_Search)
  {
    this->dataPtr->SetPingerPosition();
    this->dataPtr->lastSetPingerPositionTime = currentData.time;
  }

  // Update positions
  // The vehicles might not be ready yet, let's try to get them
  if (!this->dataPtr->playerEntity)
  {
    auto entity = _ecm.EntityByComponents(
      sim::components::Name(ScoringPlugin::VehicleName()));
    if (entity == sim::kNullEntity)
      return;

    this->dataPtr->playerEntity = entity;
  }

  currentData.player = _ecm.Component<sim::components::Pose>(
    this->dataPtr->playerEntity)->Data();

  std::vector<sim::Entity> windturbines;
  if(this->dataPtr->windturbinesEntity.size() == 0)
  {
    for(auto windturbineName : this->dataPtr->windturbinesNames)
    {
      auto entity = _ecm.EntityByComponents(
        sim::components::Name(windturbineName));
      if (entity == sim::kNullEntity)
        return;
      windturbines.push_back(entity);
      this->dataPtr->windturbinesEntity.push_back(entity);
    }
    this->dataPtr->windturbinesEntity = windturbines;
  }

  if(currentData.windturbines.size() != this->dataPtr->windturbinesEntity.size())
  {
    currentData.windturbines.clear();
    for(auto entity : this->dataPtr->windturbinesEntity)
    {
      auto worldPoseComponent = _ecm.Component<sim::components::Pose>(entity);
      if (worldPoseComponent)
      {
        currentData.windturbines.push_back(worldPoseComponent->Data());
      }
    }
  }

  if (!this->dataPtr->criticalMarkerEntity)
  {
    auto entity = _ecm.EntityByComponents(
      sim::components::Name(this->dataPtr->criticalMarkerName));
    if (entity == sim::kNullEntity)
      return;
    this->dataPtr->criticalMarkerEntity = entity;
    currentData.criticalMarkerPose = _ecm.Component<sim::components::Pose>(
      this->dataPtr->criticalMarkerEntity)->Data();
    math::Vector3d criticalTargetPoseVector = currentData.criticalMarkerPose.CoordPositionAdd(math::Vector3d(-this->dataPtr->phaseInspectionTargetDistance, 0, 0));
    currentData.criticalTargetPose = math::Pose3d(criticalTargetPoseVector, currentData.criticalMarkerPose.Rot());
  }

  // Update distances and angles
  currentData.playerDistance = currentData.player.Pos().Distance(math::Vector3d());
  currentData.criticalWindturbineDistance = currentData.player.Pos().Distance(this->dataPtr->pingerPosition);
  currentData.criticalTargetPoseDistance = currentData.player.Pos().Distance(currentData.criticalTargetPose.Pos());
  math::Vector3d vectPlayerToCritical = this->dataPtr->pingerPosition - currentData.player.Pos();
  if(vectPlayerToCritical.X() != 0)
  {
    currentData.criticalWindturbineBearingFromBoat = atan(vectPlayerToCritical.Y() / vectPlayerToCritical.X());
    if(vectPlayerToCritical.X() < 0)
    {
      if(vectPlayerToCritical.Y() > 0) {
        currentData.criticalWindturbineBearingFromBoat += M_PI;
      } else {
        currentData.criticalWindturbineBearingFromBoat -= M_PI;
      }
    }
  }

  currentData.criticalWindturbineHeading =  currentData.criticalWindturbineBearingFromBoat - currentData.player.Rot().Yaw();
  while(currentData.criticalWindturbineHeading > (2 * M_PI))
    currentData.criticalWindturbineHeading -= 2 * M_PI;
  while(currentData.criticalWindturbineHeading < -(2 * M_PI))
    currentData.criticalWindturbineHeading += 2 * M_PI;

  // Process current phase
  if(currentData.phase == WindturbinesInspectionState::Phase_Initial)
  {
    currentData.phase = WindturbinesInspectionState::Phase_Search;
    this->dataPtr->logTxtFile << "[PHASE] Phase initial finished : Time = " << std::to_string(currentData.time.count()) << std::endl;
  }
  else if(currentData.phase == WindturbinesInspectionState::Phase_Search)
  {
    bool checkupFinished = ProcessSearch(_ecm, currentData);
    if((currentData.time - this->dataPtr->timePhaseSearch).count() > this->dataPtr->phaseSearchMaxTime
      || checkupFinished)
    {
      currentData.phase = WindturbinesInspectionState::Phase_RallyCritical;
      this->dataPtr->timePhaseSearch = currentData.time;
      this->dataPtr->logTxtFile << "[PHASE] Phase search finished : Time = " << std::to_string(this->dataPtr->timePhaseSearch.count()) << std::endl;
    }
  }
  else if(currentData.phase == WindturbinesInspectionState::Phase_RallyCritical)
  {
    // Check distance to critical windturbine
    if(currentData.criticalWindturbineDistance <= this->dataPtr->phaseRallyMinDistance)
    {
      currentData.phase = WindturbinesInspectionState::Phase_Stabilize;
      this->dataPtr->timePhaseRally = currentData.time - this->dataPtr->timePhaseSearch;
      this->dataPtr->logTxtFile << "[PHASE] Phase rally finished : Time = " << std::to_string(this->dataPtr->timePhaseRally.count()) << std::endl;
    }
  }
  else if(currentData.phase == WindturbinesInspectionState::Phase_Stabilize)
  {
    ProcessMeanValues(currentData);
    if((currentData.time - this->dataPtr->timePhaseRally - this->dataPtr->timePhaseSearch).count() > this->dataPtr->phaseStabilizeTotalTime)
    {
      ProcessMeanValues(currentData, true);
      currentData.phase = WindturbinesInspectionState::Phase_TurnAround;
      this->dataPtr->timePhaseStabilize = currentData.time - this->dataPtr->timePhaseRally - this->dataPtr->timePhaseSearch;
      this->dataPtr->logTxtFile << "[PHASE] Phase stabilize finished : Time = " << std::to_string(this->dataPtr->timePhaseStabilize.count()) << std::endl;
    }
  }
  else if(currentData.phase == WindturbinesInspectionState::Phase_TurnAround)
  {
    ProcessMeanValues(currentData);
    if(ProcessTurnAround(_ecm, currentData))
    {
      ProcessMeanValues(currentData, true);
      currentData.phase = WindturbinesInspectionState::Phase_Task_Finished;
      this->dataPtr->timePhaseTurnAround = currentData.time - this->dataPtr->timePhaseStabilize - this->dataPtr->timePhaseRally - this->dataPtr->timePhaseSearch;
      this->dataPtr->logTxtFile << "[PHASE] Phase turn around finished : Time = " << std::to_string(this->dataPtr->timePhaseTurnAround.count()) << std::endl;
    }
  }
  else if(currentData.phase == WindturbinesInspectionState::Phase_Task_Finished)
  {
    this->dataPtr->logTxtFile << "[PHASE] Task finished succesfully : " 
      << "Total Time = " << std::to_string(this->ElapsedTime().count()) << std::endl;
    this->Finish();
  }

  // Process periodic tasks
  // Update speed only at 1 Hz
  if((currentData.time - this->dataPtr->lastTimeProcessSpeed).count() > PROCESS_SPEED_TASK_PERIOD)
  {
    auto diffTime = (currentData.time - this->dataPtr->lastTimeProcessSpeed).count();
    currentData.playerSpeed = currentData.player.Pos().Distance(this->dataPtr->lastPlayerPose.Pos()) / diffTime;
    this->dataPtr->lastPlayerPose = currentData.player;
    this->dataPtr->lastTimeProcessSpeed = currentData.time;
  }

  static auto turbine_idx{0};
  if( (currentData.time - this->dataPtr->lastTimePubAisWindturbines).count() > this->dataPtr->aisWindturbinesPeriod)
  {
    if(currentData.windturbines.size())
    {
      // Windturbines AIS
      const auto &cur{currentData.windturbines[turbine_idx]};

#if GZ_MATH_MAJOR_VERSION >= 9
      auto cartVec{math::CoordinateVector3::Metric(-cur.Pos().X(), -cur.Pos().Y(), cur.Pos().Z())};
      auto latlon = this->dataPtr->sc.SphericalFromLocalPosition(cartVec).value();
      const math::Quaternion<double> orientation = cur.Rot();
      math::Pose3d pose({latlon.Lat()->Degree(), latlon.Lon()->Degree(), latlon.Z().value()}, orientation);
#else
      math::Vector3d cartVec(-cur.Pos().X(), -cur.Pos().Y(), cur.Pos().Z());
      math::Vector3d latlon = this->dataPtr->sc.SphericalFromLocalPosition(cartVec);
      const math::Quaternion<double> orientation = cur.Rot();
      math::Pose3d pose(latlon, orientation);
#endif



      msgs::Pose windturbinesPoseMsg;
      msgs::Set(&windturbinesPoseMsg,pose);
      this->dataPtr->aisWindturbinesPositionsPub.Publish(windturbinesPoseMsg);
      this->dataPtr->lastTimePubAisWindturbines = currentData.time;
      turbine_idx = (turbine_idx + 1) % currentData.windturbines.size();
    }
  }

  // Publish debug topics, log csv and current phase topic
  if( (currentData.time - this->dataPtr->lastTimeProcessPublish).count() > PROCESS_PUBLISH_TASK_PERIOD)
  {
    currentData.windturbinesDistances.clear();
    for(auto windturbinePose : currentData.windturbines)
    {
      currentData.windturbinesDistances.push_back(currentData.player.Pos().Distance(windturbinePose.Pos()));
    }

    msgs::UInt32 currentPhaseMsg;
    currentPhaseMsg.set_data(uint(currentData.phase));
    this->dataPtr->currentPhasePub.Publish(currentPhaseMsg);

    OnSendTopicDebug(currentData);
    OnLogDataCsv(currentData);

    this->dataPtr->lastTimeProcessPublish = currentData.time;
  }

  // Save data every loop
  this->dataPtr->lastDataSaved = currentData;
}

//////////////////////////////////////////////////
void WindturbinesInspectionScoringPlugin::OnCollision()
{
  if((this->ElapsedTime() - this->dataPtr->lastCollisionTime).count() < 1)
  {
    this->dataPtr->lastCollisionTime = this->ElapsedTime();
  } else {
    this->dataPtr->lastCollisionTime = this->ElapsedTime();
    this->dataPtr->collisions++;
  }

  this->dataPtr->logTxtFile << "[EVENT] Collision detected : Number = " << std::to_string(this->dataPtr->collisions) << std::endl;

  if(this->dataPtr->collisionFinish)
  {
    this->Finish();
  }
}

//////////////////////////////////////////////////
void WindturbinesInspectionScoringPlugin::OnFinished()
{
  if(this->dataPtr->lastDataSaved.phase == WindturbinesInspectionState::Phase_TurnAround && this->dataPtr->phaseTurnAroundNumberTurns < 0)
  {
    ProcessMeanValues(this->dataPtr->lastDataSaved, true);
    this->dataPtr->lastDataSaved.phase = WindturbinesInspectionState::Phase_Task_Finished;
    this->dataPtr->timePhaseTurnAround = this->dataPtr->lastDataSaved.time - this->dataPtr->timePhaseStabilize - this->dataPtr->timePhaseRally - this->dataPtr->timePhaseSearch;
    this->dataPtr->logTxtFile << "[PHASE] Phase turn around finished : Time = " << std::to_string(this->dataPtr->timePhaseTurnAround.count()) << std::endl;
  }

  if(this->dataPtr->lastDataSaved.phase == WindturbinesInspectionState::Phase_Task_Finished)
  {
    this->dataPtr->logTxtFile << "[FINISH] Task finished succesfully : Time = " << std::to_string(this->ElapsedTime().count()) << std::endl;
  } else {
    this->dataPtr->logTxtFile << "[FINISH] Task finished with error : Time = " << std::to_string(this->ElapsedTime().count()) << std::endl;
    this->dataPtr->logTxtFile << "[FINISH] GAME OVER, TRY AGAIN !" << std::endl;
  }

  // Print score details
  OnLogDataCsv(this->dataPtr->lastDataSaved);

  // Close log files
  this->dataPtr->logCsvFile.close();
  this->dataPtr->logTxtFile.close();

  // Call parent OnFinished
  ScoringPlugin::OnFinished();
}

//////////////////////////////////////////////////
void WindturbinesInspectionScoringPlugin::OnRunning()
{
  this->dataPtr->node.Subscribe(this->dataPtr->topicCheckUp,
    &WindturbinesInspectionScoringPlugin::Implementation::OnWindturbineCheckUpReceived, this->dataPtr.get());
  ScoringPlugin::OnRunning();
}

//////////////////////////////////////////////////
void WindturbinesInspectionScoringPlugin::OnLogDataCsv(WindturbinesInspectionDataStruct& data)
{
  if (this->dataPtr->logCsvInfo)
  {
    // Write csv header
    std::string line = std::to_string(data.time.count()) + "," + std::to_string(data.phase);
    line += "," + std::to_string(data.player.Pos().X()) + "," + std::to_string(data.player.Pos().Y()) 
      + "," + std::to_string(data.player.Rot().Yaw()) + "," + std::to_string(data.playerSpeed) 
      + "," + std::to_string(data.playerDistance);
    line += "," + std::to_string(data.criticalTargetPose.Pos().X()) + "," + std::to_string(data.criticalTargetPose.Pos().Y()) 
      + "," + std::to_string(data.criticalMarkerPose.Rot().Yaw()) + "," + std::to_string(data.criticalTargetPoseDistance);
    line += "," + std::to_string(this->dataPtr->pingerPosition.X()) + "," + std::to_string(this->dataPtr->pingerPosition.Y());
    line += "," + std::to_string(data.criticalWindturbineDistance) + "," + std::to_string(data.criticalWindturbineHeading);
    line += "," + std::to_string(data.criticalWindturbineBearingFromBoat) + "," + std::to_string(data.criticalInspectionTurn);
    int i = 0;
    for(auto windturbinePose : data.windturbines)
    {
      line += "," + std::to_string(windturbinePose.Pos().X()) + "," + std::to_string(windturbinePose.Pos().Y()) 
        + "," + std::to_string(data.player.Pos().Distance(windturbinePose.Pos())) + "," + std::to_string(data.windturbinesCheckUpStatus[i]);
      i++;
    }
    
    this->dataPtr->logCsvFile << line << std::endl;
  }
}

//////////////////////////////////////////////////
void WindturbinesInspectionScoringPlugin::OnSendTopicDebug(WindturbinesInspectionDataStruct& data)
{
  msgs::Float criticalWindturbineHeadingMsg;
  criticalWindturbineHeadingMsg.set_data(data.criticalWindturbineHeading);
  this->dataPtr->criticalHeadingPub.Publish(criticalWindturbineHeadingMsg);

  msgs::Float criticalWindturbineDistanceMsg;
  criticalWindturbineDistanceMsg.set_data(data.criticalWindturbineDistance);
  this->dataPtr->criticalDistancePub.Publish(criticalWindturbineDistanceMsg);

  msgs::Float criticalWindturbineBearingFromBoatMsg;
  criticalWindturbineBearingFromBoatMsg.set_data(data.criticalWindturbineBearingFromBoat);
  this->dataPtr->criticalWindturbineBearingFromBoatPub.Publish(criticalWindturbineBearingFromBoatMsg);

  msgs::Float criticalTargetPoseDistanceMsg;
  criticalTargetPoseDistanceMsg.set_data(data.criticalTargetPoseDistance);
  this->dataPtr->criticalTargetPoseDistancePub.Publish(criticalTargetPoseDistanceMsg);

  msgs::Float criticalTurnAroundAngleMsg;
  criticalTurnAroundAngleMsg.set_data(data.criticalInspectionTurn);
  this->dataPtr->criticalTurnAroundAnglePub.Publish(criticalTurnAroundAngleMsg);

  msgs::Float_V windturbinesDistancesMsg;
  for(auto distance : data.windturbinesDistances)
  {
    windturbinesDistancesMsg.add_data(distance);
  }

  this->dataPtr->windturbinesDistancesPub.Publish(windturbinesDistancesMsg);

  msgs::UInt32 windturbineCheckupStatusOkMsg;
  msgs::UInt32 windturbineCheckupStatusNotReceivedMsg;
  uint32_t countOk = 0, countNotReceived = 0;
  for(auto status : data.windturbinesCheckUpStatus)
  {
    if(status == 1)
    {
      countOk++;
    }
    else if(status == 0)
    {
      countNotReceived++;
    }
  }

  windturbineCheckupStatusOkMsg.set_data(countOk);
  windturbineCheckupStatusNotReceivedMsg.set_data(countNotReceived);

  this->dataPtr->windturbineCheckupStatusOkPub.Publish(windturbineCheckupStatusOkMsg);
  this->dataPtr->windturbineCheckupStatusNotReceivedPub.Publish(windturbineCheckupStatusNotReceivedMsg);
}

//////////////////////////////////////////////////
bool WindturbinesInspectionScoringPlugin::ProcessSearch(sim::EntityComponentManager &_ecm, WindturbinesInspectionDataStruct& data)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if(data.windturbinesCheckUpStatus.size() == 0)
  {
    data.windturbinesCheckUpStatus.resize(this->dataPtr->windturbinesStatus.size(), 0);
  }

  // Check requests and update data
  for(auto request : this->dataPtr->requests)
  {
    // Convert request.data from json to retreive id and status
    // request.data is like "{"id":0,"state":"OK"}"
    nlohmann::json j = nlohmann::json::parse(request.data());
    int id = 0;
    std::string state = "";
    if(j.find("id") != j.end())
    {
      id = j["id"];
    }
    if(j.find("state") != j.end())
    {
      state = j["state"];
    }
    if(id < data.windturbinesCheckUpStatus.size() && id >= 0 && data.windturbinesCheckUpStatus[id] == 0)
    {
      std::string expectedStatus = this->dataPtr->windturbinesStatus[id];
      if (state.compare(expectedStatus) == 0)
      {
        data.windturbinesCheckUpStatus[id] = 1;
      }
      else
      {
        data.windturbinesCheckUpStatus[id] = 2;
      }
      this->dataPtr->logTxtFile << "[PROCESS] ProcessSearch : Received windturbine " << std::to_string(id) << ", checkup state = " << state << ", expected = " << expectedStatus << std::endl;
    }
  }

  bool finished = true;
  for(auto status : data.windturbinesCheckUpStatus)
  {
    if(status == 0)
    {
      finished = false;
      break;
    }
  }

  this->dataPtr->requests.clear();
  return finished;
}

//////////////////////////////////////////////////
bool WindturbinesInspectionScoringPlugin::ProcessTurnAround(sim::EntityComponentManager &_ecm, WindturbinesInspectionDataStruct& data)
{ 
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  
  double delta = (data.criticalWindturbineBearingFromBoat - this->dataPtr->lastDataSaved.criticalWindturbineBearingFromBoat);
  while (std::abs(delta) >= M_PI)
  {
    if (delta > M_PI)
      delta -= 2 * M_PI;
    else if (delta < -M_PI)
      delta += 2 * M_PI;
  }

  data.criticalInspectionTurn += delta;
  if(this->dataPtr->phaseTurnAroundNumberTurns > 0.0)
  {
    return (std::abs(data.criticalInspectionTurn) >= (this->dataPtr->phaseTurnAroundNumberTurns * 2 * M_PI));
  } 
  else 
  {
    return false;
  }
}

//////////////////////////////////////////////////
void WindturbinesInspectionScoringPlugin::ProcessMeanValues(WindturbinesInspectionDataStruct& data, bool finished)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if((data.time - this->dataPtr->lastTimeSavedMeanValues).count() < 0.5 && !finished)
  {
    return;
  }
  
  const double timeStart = (data.time - (this->dataPtr->timePhaseRally + this->dataPtr->timePhaseSearch + this->dataPtr->timePhaseStabilize)).count();
  const int currentCount = (int(floor(timeStart)) / this->dataPtr->phaseStabilizeTurnAroundMeanPeriod);
  const double timeCurrentMean = timeStart - (this->dataPtr->phaseStabilizeTurnAroundMeanPeriod * (currentCount));

  // If current phase is Stabilize or TurnAround, calculate mean values
  if(data.phase == WindturbinesInspectionState::Phase_Stabilize)
  {
    if(data.phaseStabilizeMeanTargetDistance.size() <= currentCount || finished)
    {
      data.phaseStabilizeMeanTargetDistance.push_back(data.criticalTargetPoseDistance);
      data.phaseStabilizeMeanWindturbineHeadingError.push_back(abs(data.criticalWindturbineHeading));
      data.phaseStabilizeMeanWindturbineDistance.push_back(data.criticalWindturbineDistance);
      if(data.phaseStabilizeMeanTargetDistance.size() > 1)
      {
        int index = data.phaseStabilizeMeanTargetDistance.size()-2;
        this->dataPtr->logTxtFile << "[PHASE] Stabilize Mean values " << std::to_string(index) << " : " 
          << "Target Distance = " << std::to_string(data.phaseStabilizeMeanTargetDistance[index]) << ", "
          << "Windturbine Heading Error = " << std::to_string(data.phaseStabilizeMeanWindturbineHeadingError[index]) << ", "
          << "Windturbine Distance = " << std::to_string(data.phaseStabilizeMeanWindturbineDistance[index]) << ","
          << "Time mean = " << std::to_string(timeStart-(this->dataPtr->phaseStabilizeTurnAroundMeanPeriod*index)) << std::endl;
      }
    }
    else
    {
      const double timeDiff = (data.time - this->dataPtr->lastTimeSavedMeanValues).count();

      data.phaseStabilizeMeanWindturbineDistance[currentCount] = (data.phaseStabilizeMeanWindturbineDistance[currentCount] * timeCurrentMean
        + data.criticalWindturbineDistance * timeDiff) / (timeCurrentMean + timeDiff);
      data.phaseStabilizeMeanTargetDistance[currentCount] = (data.phaseStabilizeMeanTargetDistance[currentCount] * timeCurrentMean
        + data.criticalTargetPoseDistance * timeDiff) / (timeCurrentMean + timeDiff);
      data.phaseStabilizeMeanWindturbineHeadingError[currentCount] = (data.phaseStabilizeMeanWindturbineHeadingError[currentCount] * timeCurrentMean
        + abs(data.criticalWindturbineHeading) * timeDiff) / (timeCurrentMean + timeDiff);
    }
  }
  else if(data.phase == WindturbinesInspectionState::Phase_TurnAround)
  {
    if(data.phaseTurnAroundMeanWindturbineHeadingError.size() <= currentCount || finished)
    {
      data.phaseTurnAroundMeanWindturbineHeadingError.push_back(abs(data.criticalWindturbineHeading));
      data.phaseTurnAroundMeanWindturbineDistance.push_back(data.criticalWindturbineDistance);
      if(data.phaseTurnAroundMeanWindturbineHeadingError.size() > 1)
      {
        int index = data.phaseTurnAroundMeanWindturbineHeadingError.size()-2;
        this->dataPtr->logTxtFile << "[PHASE] TurnAround Mean values " << std::to_string(index) << " : " 
          << "Windturbine Heading Error = " << std::to_string(data.phaseTurnAroundMeanWindturbineHeadingError[index]) << ", "
          << "Windturbine Distance = " << std::to_string(data.phaseTurnAroundMeanWindturbineDistance[index]) << ","
          << "Current Number Turns = " << std::to_string(data.criticalInspectionTurn) << ","
          << "Time mean = " << std::to_string(timeStart-(this->dataPtr->phaseStabilizeTurnAroundMeanPeriod*index)) << std::endl;      }
    }
    else
    {
      const double timeDiff = (data.time - this->dataPtr->lastTimeSavedMeanValues).count();

      data.phaseTurnAroundMeanWindturbineDistance[currentCount] = (data.phaseTurnAroundMeanWindturbineDistance[currentCount] * timeCurrentMean
        + data.criticalWindturbineDistance * timeDiff) / (timeCurrentMean + timeDiff);
      data.phaseTurnAroundMeanWindturbineHeadingError[currentCount] = (data.phaseTurnAroundMeanWindturbineHeadingError[currentCount] * timeCurrentMean
        + abs(data.criticalWindturbineHeading) * timeDiff) / (timeCurrentMean + timeDiff);
    }
  }

  this->dataPtr->lastTimeSavedMeanValues = data.time;
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(vrx::WindturbinesInspectionScoringPlugin,
              sim::System,
              vrx::WindturbinesInspectionScoringPlugin::ISystemConfigure,
              vrx::WindturbinesInspectionScoringPlugin::ISystemPreUpdate)
