# Standard Test Scenario Generation for MASS Collision Avoidance

Data and reference implementation accompanying the manuscript:

> **Standard Test Scenario Generation Technology for Collision Avoidance of Maritime Autonomous Surface Ships Based on ROS**
> X. Chen, Z. Cheng, N. Li, X. Liu, J. Chen
> *Journal of Marine Science and Engineering (JMSE)*, under review.

This repository provides the standardized test scenario datasets and the target-ship parameter generation algorithm described in the paper, enabling reproducible benchmarking of MASS collision avoidance algorithms.

## Overview

The framework generates deterministic, COLREGs-compliant collision avoidance test scenarios by solving an inverse-kinematic problem: given a desired DCPA (distance at closest point of approach) and TCPA (time to closest point of approach), the method computes the target ship initial position, course, and speed in closed form. COLREGs right-of-way rules are embedded through an elliptical safety domain, and multi-ship scenarios are constructed by superposition.

## Repository Structure

```
.
├── data/
│   ├── 17_scenario_config.json         # configuration of the 17 test scenarios
│   ├── scenario_library_index.json     # index of the scenario library
│   ├── scenario_library/               # individual scenario definitions (JSON)
│   ├── target_ship_parameters.csv      # generated parameters of 62 target ships
│   └── execution_deviation.csv         # specified vs. achieved DCPA/TCPA
├── src/
│   ├── scenario_generator_ros.py       # ROS 2 implementation of the generator
│   └── generate_scenario_index.py      # scenario library index builder
├── figures/
│   └── Figure1.png ... Figure7.png     # manuscript figures (PNG, 300 dpi)
├── LICENSE
└── README.md
```

## Data Description

### `data/17_scenario_config.json`
Configuration of the 17 simulation scenarios, each with an ID, name, number of target ships (`ship_num`), encounter type(s), and behavior mode. Scenarios span single-target two-ship encounters to six-target multi-ship interactions.

### `data/scenario_library/` and `scenario_library_index.json`
A structured scenario library with a JSON index, providing reusable, standardized encounter definitions.

### `data/target_ship_parameters.csv`
Generated initial parameters of the 62 target ships across the 17 scenarios (Table A of the paper's supplementary material). Columns:

| Column | Unit | Description |
|--------|------|-------------|
| `scenario` | — | scenario ID (1–17) |
| `target_ship` | — | target ship index (tg_1, ...) |
| `encounter_type` | — | crossing-standon (port) / crossing-giveway (starboard) / head-on / overtaking |
| `speed_kn` | kn | target ship speed |
| `initial_distance_m` | m | initial distance from the own ship |
| `relative_bearing_deg` | deg | initial relative bearing |
| `course_deg` | deg | target ship course |

### `data/execution_deviation.csv`
Execution deviation between specified and achieved DCPA/TCPA in the Gazebo simulation (Table B of the supplementary material). Columns: `scenario`, `target_ship`, `set_dcpa_nm`, `actual_dcpa_nm`, `dcpa_deviation_nm`, `set_tcpa_min`, `actual_tcpa_min`, `tcpa_deviation_min`.

The inverse-kinematic generator is exact in closed form; the tabulated deviations reflect ship dynamics (target acceleration transients and own-ship heading oscillation) rather than generation error.

## Code

### `src/scenario_generator_ros.py`
The ROS 2 node that computes target ship initial parameters satisfying given DCPA/TCPA constraints and COLREGs sector requirements, and publishes them to the simulation environment.

**Dependencies:** ROS 2 (Foxy or later), `numpy`, and a Gazebo/VRX-based ship simulation environment. The script is provided as a reference implementation of the algorithm in Section 3 of the paper; it is intended to be run within a matching ROS 2 workspace.

### `src/generate_scenario_index.py`
Builds the scenario library index from individual scenario definitions.

## Figures
The seven manuscript figures are provided as high-resolution PNG (300 dpi) files in `figures/`.

## Requirements
- Python 3.8+
- numpy
- ROS 2 (Foxy or later) for the simulation node
- Gazebo / VRX maritime simulation environment (for execution)

## License
- **Code** (`src/`): MIT License — see `LICENSE`.
- **Data** (`data/`, `figures/`): Creative Commons Attribution 4.0 International (CC BY 4.0).

## Citation
If you use these data or code, please cite the manuscript above. The formal citation and DOI will be updated upon publication.
