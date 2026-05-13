#ifndef WINDTURBINES_INSPECTION_SCORING_PLUGIN_HH
#define WINDTURBINES_INSPECTION_SCORING_PLUGIN_HH

#include <gz/transport.hh>
#include "vrx/ScoringPlugin.hh"

#define PROCESS_SPEED_TASK_PERIOD 1.0
#define PROCESS_PUBLISH_TASK_PERIOD 1.0

using namespace gz;

enum WindturbinesInspectionState
{
  Phase_Initial = 0,
  Phase_Search = 1,
  Phase_RallyCritical = 2,
  Phase_Stabilize = 3,
  Phase_TurnAround = 4,
  Phase_Task_Finished = 5
};

struct WindturbinesInspectionDataStruct {
  std::chrono::duration<double> time = std::chrono::duration<double>::zero();
  WindturbinesInspectionState phase = WindturbinesInspectionState::Phase_Initial;
  math::Pose3d player = math::Pose3d::Zero;
  double playerSpeed = 0;
  double playerDistance = 0;
  math::Pose3d criticalMarkerPose = math::Pose3d::Zero;
  math::Pose3d criticalTargetPose = math::Pose3d::Zero;
  double criticalWindturbineDistance = 0;
  double criticalWindturbineBearingFromBoat = 0;
  double criticalWindturbineHeading = 0;
  double criticalTargetPoseDistance = 0;
  std::vector<double> windturbinesDistances;
  std::vector<uint32_t> windturbinesCheckUpStatus;
  uint64_t collisions = 0;
  uint16_t windturbineCriticalId = 0;
  std::vector<math::Pose3d> windturbines;
  double criticalInspectionTurn = 0;
  std::vector<double> phaseStabilizeMeanTargetDistance;
  std::vector<double> phaseStabilizeMeanWindturbineHeadingError;
  std::vector<double> phaseStabilizeMeanWindturbineDistance;
  std::vector<double> phaseTurnAroundMeanWindturbineHeadingError;
  std::vector<double> phaseTurnAroundMeanWindturbineDistance;
};

namespace vrx
{
  /// \brief A plugin for computing the score of the windturbines inspection task.
  ///  This plugin derives from the generic ScoringPlugin class. Refer to that
  /// plugin for an explanation of the four states defined (Initial, Ready,
  /// Running and Finished) as well as other required SDF elements.
  ///
  /// Windturbines inspection task is a two part task. In the first part, the WAM-V
  /// must raise windturbines state fast as possible. In the second part, the WAM-V must
  /// find and turn around the critical windturbine.
  class WindturbinesInspectionScoringPlugin : public ScoringPlugin
  {
    /// \brief Constructor.
    public: WindturbinesInspectionScoringPlugin();

    /// \brief Destructor.
    public: ~WindturbinesInspectionScoringPlugin() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited.
    protected: void OnCollision() override;

    protected: void OnFinished() override;

    private: void OnRunning() override;

    // Log data csv
    private: 
    void OnLogDataCsv(WindturbinesInspectionDataStruct& data);
    void OnSendTopicDebug(WindturbinesInspectionDataStruct& data);
    bool ProcessSearch(sim::EntityComponentManager &_ecm, WindturbinesInspectionDataStruct& data);
    bool ProcessTurnAround(sim::EntityComponentManager &_ecm, WindturbinesInspectionDataStruct& data);
    void ProcessMeanValues(WindturbinesInspectionDataStruct& data, bool finished = false);

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif // WINDTURBINES_INSPECTION_SCORING_PLUGIN_HH
