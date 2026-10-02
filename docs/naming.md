# Naming conventions

One term per concept. Name a new thing by finding its row below, not by inventing a word.
(Plan §2; the same table is in `CLAUDE.md`.)

| Concept | Term | ID form (yaml, topics) | Node/link/model form |
|---|---|---|---|
| The whole main line | **line** | `main` | `Line` |
| A work location on the main line | **station** | `st_place_spider` | `Station_PlaceSpider` |
| Parallel copies of a station | **cell** | `st_test.cell_3` | `Station_Test_Cell3` |
| Carrier riding the conveyor | **pallet** | `pallet` | `Pallet_<n>` |
| Queued pallet position between two stations | **queue slot** | count in `layout.buffer_slots_between_stations` | `Pallet_Slot_<n>_<k>` (after station index n, k-th) |
| Returnable fixture carrying a subassembly | **carrier** | `carrier_spider`, `carrier_pod` | `Carrier_Spider` |
| Part source at a fixed pick pose | **feeder** | `fd_mid_plate` | `Feeder_MidPlate` |
| WIP between a sub-line and the main line | **buffer** | `buf_spider` | `Buffer_Spider` |
| Pick-and-place robot, one per station | **scara** | `scara_<station without st_>` (`scara_place_spider`) | `Scara_<Name>` (`Scara_PlaceSpider`); layout `Scara_<Name>_Base`, `Scara_<Name>_Envelope` |
| Run of consecutive stations built, checked and guarded together | **milestone** | `milestones.<name>` (`frame`) | `Guard_<Milestone>` (`Guard_Frame`) |
| Screwdriving unit (fixed pattern) | **gang head** | `gh_arm_sandwich` | `GangHead_ArmSandwich`; on a slide, `GangHead_<Name>_Slide` |
| Interchangeable SCARA tooling | **tool** | `tool_plate` | `Tool_Plate` |
| SCARA tool taking a whole set in one pick | **gang gripper** | `<part>_gang` (`standoff_gang`) | `Tool_StandoffGang` |
| SCARA tool lifting the 4 arms out of their carrier in their X pattern | **spider gripper** | `spider_gripper` | `Tool_SpiderGripper` |
| SCARA tool taking a left and right part together, one pocket each | **pair gripper** | `<part>_pair` (`side_plate_pair`) | `Tool_SidePlatePair` |
| Named frame on a part used for grasp | **pick frame** | `pick` | `<Part>_pick` |
| Named datum on a plate/pallet | **datum** | `datum_round`, `datum_diamond` | `<Part>_datum_round` |
| Pallet pin locating one arm through its tooling hole | **arm pin** | `drone.arm_tooling_holes` | `Pin_Arm_<n>` |
| An operation the sequencer can request | **capability** | `place`, `drive`, `advance`, `test` | — |

Rules:
- Station IDs use the **verb** of what happens there (`st_drive_arm_sandwich`), not the equipment.
- yaml/topic IDs are `snake_case`; Gazebo model/link names are `PascalCase` derived mechanically.
- "Arm" means the drone arm only. Robots are "scara" and "gang head". Never "robot arm" (a
  SCARA's own segments are "links").
- A SCARA's joints are `scara_<station>_j1..j4` (shoulder, elbow, Z, wrist:
  `scara_place_spider_j1`), its links `Scara_<Name>_{base,link1,link2,quill,wrist}` and
  `Scara_<Name>_Tcp`, its controller `scara_<station>_controller` (D-012).
