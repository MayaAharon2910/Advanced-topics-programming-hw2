#pragma once

#include "StrongPosition3D.h"
#include "Map3D.h"
#include "ScanResult.h"
#include "Config.h"
#include <mp-units/systems/si/unit_symbols.h>
#include <vector>
#include <deque>
#include <set>
#include <tuple>

class ILidarSensor;
class IPositionSensor;
class IMovementDriver;

enum ScanDirection {
    FORWARD = 0,
    RIGHT = 90,
    BACK = 180,
    LEFT = 270
};

enum class DroneCommandType {
    Rotate,
    Advance,
    Elevate,
    Scan,
    GetLocation,
    Finished
};

struct DroneCommand {
    DroneCommandType type;
    double value_cm = 0.0;
    double angle_deg = 0.0;
    bool has_angles = false;
    double azimuth_deg = 0.0;
    double elevation_deg = 0.0;
    bool succeeded = true;
    bool collision_detected = false;
    double executed_azimuth_deg = 0.0;
    double executed_elevation_deg = 0.0;
    size_t scan_beams = 0;
    int scan_hits = 0;
    int scan_open = 0;
};

class Drone {
public:
    // Creates a drone algorithm instance wired to explicit sensor and movement interfaces.
    Drone(size_t map_width,
          size_t map_height,
          size_t map_depth,
          const Config& config,
          const MissionConfig& mission_config,
          ILidarSensor& lidar,
          IPositionSensor& position_sensor,
          IMovementDriver& driver);

    // Executes exactly one autonomous action through the injected interfaces.
    DroneCommand executeNextAction();
    // Returns the next command required by the exploration state machine.
    DroneCommand getNextCommand();
    // Updates the internal occupancy map from LiDAR scan results.
    void processScanResults(const std::vector<ScanResult>& results);
    // Stores the latest measured pose and marks the current voxel as free.
    void updatePose(const Pose3D& pose);

    // Returns the drone's current reconstructed occupancy map.
    const Map3D& getMap() const { return internal_map_; }
    // Returns the drone's current position estimate.
    StrongPosition3D getPosition() const { return current_position_; }
    // Returns the drone's current heading estimate in degrees.
    mp_units::quantity<mp_units::si::unit_symbols::deg> getOrientation() const { return orientation_; }

private:
    enum class ExplorationState {
        Scanning,
        Planning,
        Moving,
        Finished
    };

    enum class BfsGoalMode {
        Frontier6,
        Frontier26
    };

    StrongPosition3D current_position_;
    mp_units::quantity<mp_units::si::unit_symbols::deg> orientation_;
    Map3D internal_map_;
    Config config_;
    MissionConfig mission_config_;
    ILidarSensor* lidar_;
    IPositionSensor* position_sensor_;
    IMovementDriver* driver_;
    mp_units::quantity<mp_units::si::unit_symbols::cm> sphere_radius_;

    // Emits scan commands or advances to planning after a scan batch completes.
    DroneCommand nextScanningCommand();
    // Chooses local movement, BFS recovery, targeted scans, or mission finish.
    DroneCommand nextPlanningCommand();
    // Emits queued movement commands and returns to scanning when movement ends.
    DroneCommand nextMovingCommand();
    // Queues a full scan batch around the current pose.
    void enqueueScanCommands();

    // Finds a BFS path to the nearest reachable frontier matching the goal mode.
    std::vector<StrongPosition3D> bfs_to_goal(BfsGoalMode goal_mode) const;
    // Returns true when the drone sphere can safely occupy the given position.
    bool targetIsSafeForDrone(const StrongPosition3D& position) const;
    // Returns true when a voxel is a valid unvisited frontier goal for BFS.
    bool isBfsGoal(int x, int y, int z, BfsGoalMode goal_mode) const;
    // Returns true when the current rounded voxel was already fully scanned.
    bool currentPositionWasVisited() const;
    // Marks the current scan batch complete and moves the state machine to planning.
    void finishCurrentScanBatch();
    // Records the current rounded voxel as visited and keeps it marked free.
    void markCurrentPositionVisited();
    // Rewrites all visited voxels as free after scan updates.
    void preserveVisitedPositionsAsFree();
    // Processes scan results using the expected beam pattern for the executed scan direction.
    void processScanResultsForExecutedScan(const std::vector<ScanResult>& results,
                                           double absolute_azimuth_deg,
                                           double elevation_deg);
    // Removes the current reached step from the active BFS path.
    void consumeCurrentPathStep();

    // Queues a one-step local move to a nearby safe unvisited voxel if possible.
    bool enqueueSweepMove();
    // Queues a targeted scan toward unknown space around a blocked target.
    bool enqueueTargetedScanForTarget(const StrongPosition3D& target_position);
    // Queues a targeted scan toward an unknown neighbor of the current voxel.
    bool enqueueTargetedScanAroundCurrentPosition();

    std::deque<DroneCommand> pending_commands_;
    std::vector<StrongPosition3D> current_path_;
    std::set<std::tuple<int, int, int>> visited_positions_;
    bool scan_batch_completion_pending_ = false;
    ExplorationState state_ = ExplorationState::Scanning;
    bool require_location_update_ = true;
};
