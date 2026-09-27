# Naming conventions

One term per concept. Name a new thing by finding its row below, not by inventing a word.
(Plan §2; the same table is in `CLAUDE.md`.)

| Concept | Term | ID form (yaml, topics) | Node/link/model form |
|---|---|---|---|
| The whole main line | **line** | `main` | `Line` |
| A work location on the main line | **station** | `st_place_spider` | `Station_PlaceSpider` |
| Parallel copies of a station | **cell** | `st_test.cell_3` | `Station_Test_Cell3` |
| Carrier riding the conveyor | **pallet** | `pallet` | `Pallet_<n>` |
| Returnable fixture carrying a subassembly | **carrier** | `carrier_spider`, `carrier_pod` | `Carrier_Spider` |
| Part source at a fixed pick pose | **feeder** | `fd_mid_plate` | `Feeder_MidPlate` |
| WIP between a sub-line and the main line | **buffer** | `buf_spider` | `Buffer_Spider` |
| Pick-and-place robot | **gantry** | `gantry_<n>` (`gantry_1`) | `Gantry_<n>` (`Gantry_1`) |
| Region served by one gantry | **zone** | zone list inside `gantries.<id>` | — |
| Screwdriving unit (fixed pattern) | **gang head** | `gh_arm_sandwich` | `GangHead_ArmSandwich` |
| Screwdriving unit (variable position) | **scara** | `scara_1` | `Scara_1` |
| Interchangeable gantry tooling | **tool** | `tool_plate` | `Tool_Plate` |
| Named frame on a part used for grasp | **pick frame** | `pick` | `<Part>_pick` |
| Named datum on a plate/pallet | **datum** | `datum_round`, `datum_diamond` | `<Part>_datum_round` |
| An operation the sequencer can request | **capability** | `place`, `drive`, `advance`, `test` | — |

Rules:
- Station IDs use the **verb** of what happens there (`st_drive_arm_sandwich`), not the equipment.
- yaml/topic IDs are `snake_case`; Gazebo model/link names are `PascalCase` derived mechanically.
- "Arm" means the drone arm only. Robots are "gantry", "gang head", "scara". Never "robot arm".
- A gantry's joints are `gantry_<n>_<axis>` (`gantry_1_x`), its links `Gantry_<n>_<axis>` and
  `Gantry_<n>_Tcp`, its controller `gantry_<n>_controller`, its world rail `Rail_Gantry_<n>`.
