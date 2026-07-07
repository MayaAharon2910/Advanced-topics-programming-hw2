// =============================================================================
// staff_inputs_test.cpp - Integration tests using the official staff input files
// The course staff released a full composition (inputs/sim_compose.yaml)
// with 5 simulations x 6 missions x 2 drones x 2 lidars = 24 runs.
// Running all 24 runs with max_steps up to 10 000 exceeds the 1-minute
// integration test budget, so we split coverage across two tests:
//   FullCompositionRunsWithoutCrashing - loads and parses the full composition,
//     then runs the first run only (1 sim x 1 mission x 1 drone x 1 lidar)
//     as a fast pipeline-stability smoke test.
//   HouseScenarioMapLoadsWithSemanticValues - isolated regression guard for the
//     Map3DImpl clamping fix: the house map uses uint8 values > 1 and must be
//     treated as Occupied, not Unmapped.
// =============================================================================

#include <gtest/gtest.h>

#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/YamlConfig.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace drone_mapper {

namespace {
std::filesystem::path stagingDir() {
    return std::filesystem::current_path() / "tmp_staff_inputs_output";
}
} // namespace

/*
 * What it does: checks that applyMapOffset() shifts a local position and a
 *               local bounds box into world coordinates correctly.
 * Setup: a fixed map offset (100, 50, 150 cm) plus one sample local position
 *        and one sample local bounds box.
 * Checks: EXPECT_DOUBLE_EQ on every resulting x/y/z of both the shifted
 *         position and the shifted bounds.
 */
TEST(Integration, SimulationFactoryAppliesMapOffsetToInitialPoseAndBounds) {
    const Position3D offset{100.0 * cm, 50.0 * cm, 150.0 * cm};
    const Position3D local_pose{10.0 * cm, 20.0 * cm, 30.0 * cm};

    const auto world_pose = SimulationRunFactoryImpl::applyMapOffset(local_pose, offset);
    EXPECT_DOUBLE_EQ(world_pose.x.force_numerical_value_in(cm), 110.0);
    EXPECT_DOUBLE_EQ(world_pose.y.force_numerical_value_in(cm), 70.0);
    EXPECT_DOUBLE_EQ(world_pose.z.force_numerical_value_in(cm), 180.0);

    types::MappingBounds local_bounds{};
    local_bounds.min_x = 0.0 * cm;
    local_bounds.max_x = 50.0 * cm;
    local_bounds.min_y = 0.0 * cm;
    local_bounds.max_y = 60.0 * cm;
    local_bounds.min_height = 0.0 * cm;
    local_bounds.max_height = 40.0 * cm;

    const auto world_bounds = SimulationRunFactoryImpl::applyMapOffset(local_bounds, offset);
    EXPECT_DOUBLE_EQ(world_bounds.min_x.force_numerical_value_in(cm), 100.0);
    EXPECT_DOUBLE_EQ(world_bounds.max_x.force_numerical_value_in(cm), 150.0);
    EXPECT_DOUBLE_EQ(world_bounds.min_y.force_numerical_value_in(cm), 50.0);
    EXPECT_DOUBLE_EQ(world_bounds.max_y.force_numerical_value_in(cm), 110.0);
    EXPECT_DOUBLE_EQ(world_bounds.min_height.force_numerical_value_in(cm), 150.0);
    EXPECT_DOUBLE_EQ(world_bounds.max_height.force_numerical_value_in(cm), 190.0);
}

/*
 * What it does: loads the staff's full composition YAML and runs just the
 *               first sim/mission/drone/lidar combo as a fast smoke test.
 * Setup: parses inputs/sim_compose.yaml, checks it has at least one sim
 *        group/drone/lidar, then builds a 1x1x1x1 subset from the first
 *        entries with max_steps capped to 200.
 * Checks: no exception thrown, exactly 1 run produced, the run isn't a
 *         silent map-load failure, score <= 100, and runtime stays under 60s.
 */
TEST(Integration, FullCompositionRunsWithoutCrashing) {
    // Parse the full composition - verifies the YAML is well-formed.
    const auto comp = yaml::parseSimulationComposition("inputs/sim_compose.yaml");

    ASSERT_FALSE(comp.simulation_mission_groups.empty())
        << "Staff composition must have at least one simulation group";
    ASSERT_FALSE(comp.drones.empty())
        << "Staff composition must specify at least one drone";
    ASSERT_FALSE(comp.lidars.empty())
        << "Staff composition must specify at least one lidar";

    // Use the house scenario (index 0) with its first mission capped to 200 steps.
    // The house map is the canonical staff map and is verified separately in
    // HouseScenarioMapLoadsWithSemanticValues. We cap steps so this smoke test
    // finishes quickly even if the drone encounters obstacles immediately.
    const auto& [chosen_sim, chosen_missions] = comp.simulation_mission_groups.front();
    ASSERT_FALSE(chosen_missions.empty());

    auto chosen_mission = chosen_missions.front();
    chosen_mission.max_steps = std::min(chosen_mission.max_steps, std::size_t{200});

    types::SimulationCompositionData subset{};
    subset.composition_file = comp.composition_file;
    subset.simulation_mission_groups.emplace_back(chosen_sim, std::vector{chosen_mission});
    subset.drones.push_back(comp.drones.front());
    subset.lidars.push_back(comp.lidars.front());

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    const auto staging = stagingDir();
    std::filesystem::create_directories(staging);

    const auto t0 = std::chrono::steady_clock::now();
    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(subset, staging));
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    ASSERT_EQ(report.runs.size(), 1U)
        << "Subset composition should produce exactly 1 run";

    // The score must not be a silent map-load failure. A map-load failure looks
    // like: score==-1 AND the only MissionRunResult has steps==0 AND the error
    // code is NOT DRONE_HITS_OBSTACLE (a collision on step 1+ is fine — it means
    // the map loaded and the pipeline reached the run loop).
    const auto& run = report.runs.front();
    const bool map_load_failed = [&] {
        if (run.mission_score != -1.0) return false;
        if (run.mission_results.empty()) return true;      // no result at all → load fail
        const auto& first = run.mission_results.front();
        if (first.steps > 0) return false;                 // ran at least one step → ok
        if (first.errors.empty()) return true;             // 0 steps, no error code → load fail
        return first.errors.front().code != "DRONE_HITS_OBSTACLE";
    }();

    EXPECT_FALSE(map_load_failed)
        << "Run returned -1 with 0 steps: the staff map probably failed to load. "
        << "Error: " << [&] {
               if (run.mission_results.empty()) return std::string{"no mission result"};
               if (run.mission_results.front().errors.empty()) return std::string{"no error code"};
               return run.mission_results.front().errors.front().code;
           }();
    EXPECT_LE(run.mission_score, 100.0);

    EXPECT_LT(elapsed_ms, 60'000)
        << "Single staff run took " << elapsed_ms << "ms - exceeds the 1-minute limit";

    std::filesystem::remove_all(staging);
}

/*
 * What it does: regression guard for the Map3DImpl clamping fix - the staff's
 *               house map stores solid voxels as uint8 2/3/4/18/45, not 1.
 * Setup: runs the fastest house mission with the small drone and short lidar,
 *        max_steps capped to 200.
 * Checks: no exception thrown, exactly 1 run produced, score is in [0, 100]
 *         (not the -1 error score, which would mean the map failed to load).
 */
TEST(Integration, HouseScenarioMapLoadsWithSemanticValues) {
    const auto comp = yaml::parseSimulationComposition("inputs/sim_compose.yaml");

    ASSERT_FALSE(comp.simulation_mission_groups.empty());
    const auto& [sim, missions] = comp.simulation_mission_groups.front();
    ASSERT_FALSE(missions.empty());
    ASSERT_FALSE(comp.drones.empty());
    ASSERT_FALSE(comp.lidars.empty());

    // Use the mission with fewest steps from this group.
    auto chosen_mission2 = *std::min_element(
        missions.begin(), missions.end(),
        [](const auto& a, const auto& b) { return a.max_steps < b.max_steps; });
    // Cap max_steps so this test finishes in a few seconds even if the drone
    // hits obstacles immediately (which it does on the house map with walls).
    chosen_mission2.max_steps = std::min(chosen_mission2.max_steps, std::size_t{200});

    types::SimulationCompositionData isolated{};
    isolated.composition_file = comp.composition_file;
    isolated.simulation_mission_groups.emplace_back(sim, std::vector{chosen_mission2});
    isolated.drones.push_back(comp.drones.front());
    isolated.lidars.push_back(comp.lidars.back());

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    const auto staging = stagingDir();
    std::filesystem::create_directories(staging);

    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(isolated, staging));

    ASSERT_EQ(report.runs.size(), 1U);
    // Strictly require a valid (non-error) score — see the note in
    // FullCompositionRunsWithoutCrashing. The capped run may end with
    // "max_steps", but it must still load the map and produce a real score.
    EXPECT_GE(report.runs.front().mission_score, 0.0)
        << "Run returned the error score (-1): the staff house map probably failed to load";
    EXPECT_LE(report.runs.front().mission_score, 100.0);

    std::filesystem::remove_all(staging);
}

/*
 * What it does: replicates the grader's validation fixture layout - a
 *               composition file with generic, unquoted relative paths
 *               (simulation/simulation.yaml, mission/mission.yaml, etc.) in
 *               sub-folders next to the composition file, not the CWD.
 * Setup: builds that exact folder structure in a temp dir (tiny 5x5x5
 *        single-voxel map copied to map/map.npy), then parses and runs it
 *        with CWD left at the project root.
 * Checks: map_filename resolves to a file that actually exists, parsing and
 *         running both succeed, exactly 1 run is produced, and its score is
 *         in [0, 100] (not the -1 error score from a failed map load).
 */
TEST(Integration, GraderLayoutFixtureResolvesPathsRelativeToComposition) {
    namespace fs = std::filesystem;
    const fs::path root = fs::current_path() / "tmp_grader_layout_fixture";
    fs::remove_all(root);
    fs::create_directories(root / "simulation");
    fs::create_directories(root / "mission");
    fs::create_directories(root / "drone");
    fs::create_directories(root / "lidar");
    fs::create_directories(root / "map");

    // Tiny 5x5x5 map (10cm voxels -> 50cm world cube) with a single occupied
    // voxel at grid (2,4,2). Copied under the generic name the fixture implies.
    ASSERT_TRUE(fs::exists("data_maps/single_voxel_x2_y4_z2.npy"))
        << "Test must run from the project root (data_maps/ not found)";
    fs::copy_file("data_maps/single_voxel_x2_y4_z2.npy", root / "map" / "map.npy",
                  fs::copy_options::overwrite_existing);

    auto write = [](const fs::path& p, const std::string& text) {
        std::ofstream out(p);
        out << text;
    };

    // Verbatim structure of validation_tests/fixtures/valid/composition.yaml
    // (generic names, unquoted paths).
    write(root / "composition.yaml",
          "simulation_compositions:\n"
          "  simulations:\n"
          "    - simulation_config: simulation/simulation.yaml\n"
          "      mission_configs:\n"
          "        - mission/mission.yaml\n"
          "  drone_configs:\n"
          "    - drone/drone.yaml\n"
          "  lidar_configs:\n"
          "    - lidar/lidar.yaml\n");

    write(root / "simulation" / "simulation.yaml",
          "simulation_config:\n"
          "  map_filename: \"map/map.npy\"\n"
          "  map_resolution_cm: 10\n"
          "  initial_drone_position:\n"
          "    x_cm: 25\n"
          "    y_cm: 15\n"
          "    height_cm: 25\n"
          "  initial_angle_deg: 0\n"
          "  map_axes_offset:\n"
          "    x_offset: 0\n"
          "    y_offset: 0\n"
          "    height_offset: 0\n");

    write(root / "mission" / "mission.yaml",
          "mission_config:\n"
          "  max_steps: 60\n"
          "  boundaries:\n"
          "    x_boundary:\n"
          "      min_cm: 0\n"
          "      max_cm: 50\n"
          "    y_boundary:\n"
          "      min_cm: 0\n"
          "      max_cm: 50\n"
          "    height_boundary:\n"
          "      min_cm: 0\n"
          "      max_cm: 50\n"
          "  gps_resolution_cm: 10\n");

    write(root / "drone" / "drone.yaml",
          "drone_config:\n"
          "  dimensions_cm: 8\n"
          "  max_rotate_deg: 90\n"
          "  max_advance_cm: 30\n"
          "  max_elevate_cm: 20\n");

    write(root / "lidar" / "lidar.yaml",
          "lidar_config:\n"
          "  z_min_cm: 5\n"
          "  z_max_cm: 80\n"
          "  d_cm: 2.5\n"
          "  fov_circles: 4\n");

    // Parse with CWD at the project root — every reference must resolve
    // relative to the composition file, exactly like the grader's fixture.
    types::SimulationCompositionData comp{};
    ASSERT_NO_THROW(comp = yaml::parseSimulationComposition(root / "composition.yaml"));
    ASSERT_EQ(comp.simulation_mission_groups.size(), 1U);
    ASSERT_EQ(comp.drones.size(), 1U);
    ASSERT_EQ(comp.lidars.size(), 1U);

    // The map path must already be resolved to an existing file at parse time.
    const auto& parsed_sim = std::get<0>(comp.simulation_mission_groups.front());
    EXPECT_TRUE(fs::exists(parsed_sim.map_filename))
        << "map_filename was not resolved relative to the composition directory: "
        << parsed_sim.map_filename;

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    const fs::path staging = fs::current_path() / "tmp_grader_layout_output";
    fs::create_directories(staging);

    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(comp, staging));

    ASSERT_EQ(report.runs.size(), 1U);
    EXPECT_GE(report.runs.front().mission_score, 0.0)
        << "Grader-layout run returned the error score (-1): map path resolution failed";
    EXPECT_LE(report.runs.front().mission_score, 100.0);

    fs::remove_all(root);
    fs::remove_all(staging);
}

} // namespace drone_mapper
