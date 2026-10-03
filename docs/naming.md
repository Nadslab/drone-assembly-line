# Naming conventions

One term per concept. Name a new thing by finding its row below, not by inventing a word.
(Plan §2; the same table is in `CLAUDE.md`.)

| Concept | Term | ID form (yaml, topics) | Node/link/model form |
|---|---|---|---|
| The whole main line | **line** | `main` | `Line` |
| A work location on the main line | **station** | `st_build_frame` | `Station_BuildFrame` |
| Parallel copies of a station | **cell** | `st_test.cell_3` | `Station_Test_Cell3` |
| Carrier riding the conveyor | **pallet** | `pallet` | `Pallet_<n>` |
| Queued pallet position between two stations | **queue slot** | count derived per gap at `layout.pallet_block.length + layout.slot_gap` (`spec.queue_gaps`) | `Pallet_Slot_<n>_<k>` (after station index n, k-th) |
| Returnable fixture carrying a subassembly | **carrier** | `carrier_spider`, `carrier_pod` | `Carrier_Spider` |
| Chute or conveyor taking an empty carrier back to its sub-line | **return path** | `rt_<carrier>` (`rt_carrier_spider`) | `ReturnPath_CarrierSpider`, its end `ReturnSink_CarrierSpider` |
| Part source at a fixed pick pose | **feeder** | `fd_mid_plate` | `Feeder_MidPlate` |
| WIP between a sub-line and the main line | **buffer** | `buf_spider` | `Buffer_Spider` |
| Pick-and-place robot, one per station | **scara** | `scara_<station without st_>` (`scara_build_frame`) | `Scara_<Name>` (`Scara_BuildFrame`); layout `Scara_<Name>_Base` (pedestal), `Scara_<Name>_<Part>` (vendor arm), `Scara_<Name>_Envelope` |
| SCARA product used by a scara | **robot model** | `robot_models.<id>` (`epson_ls10b_700_z300`) | — |
| Run of consecutive stations built and checked together | **milestone** | `milestones.<name>` (`frame`) | partition `Guard_Partition_<A>_<B>` at its boundary; the line's one fence `Guard_Fence` |
| Screwdriving unit (fixed pattern) | **gang head** | `gh_arm_sandwich` | `GangHead_ArmSandwich`; on a slide, `GangHead_<Name>_Slide` (envelope) |
| Rail and post carrying a gang head on its slide | **slide frame** | `gang_heads.<id>.slide.frame` | `GangHead_<Name>_Rail`, `GangHead_<Name>_Post` |
| Where a SCARA waits while its station's gang head works | **park pose** | `scaras.<id>.park_pose` | close-up `reports/render/<station>_park.png` |
| Vendor's published reference move the motion is calibrated to | **standard cycle** | `robot_models.<id>.standard_cycle` | — |
| Interchangeable SCARA tooling | **tool** | `tool_plate_vacuum` (`tools:`) | `Tool_PlateVacuum` |
| SCARA end-effector carrying two tools 180° apart, switched by a J4 turn | **dual-tool** | `scaras.<id>.tools: [<tool>, <tool>]` | — |
| SCARA tool taking a whole set in one pick | **gang gripper** | `<part>_gang` (`standoff_gang`) | `Tool_StandoffGang` |
| SCARA tool taking a left and right part together, one pocket each | **pair gripper** | `<part>_pair` (`side_plate_pair`) | `Tool_SidePlatePair` |
| Named frame on a part used for grasp | **pick frame** | `pick` | `<Part>_pick` |
| Named datum on a plate/pallet | **datum** | `datum_round`, `datum_diamond` | `<Part>_datum_round` |
| Pallet pin locating one arm through its tooling hole | **arm pin** | `drone.arm_tooling_holes` | `Pin_Arm_<n>` |
| An operation the sequencer can request | **capability** | `place`, `drive`, `remove` (lift a carrier off onto its return path), `actuate` (a station device, e.g. arm pins), `advance`, `test` | — |

Rules:
- Station IDs use the **verb** of what happens there (`st_build_frame`, `st_seat_esc`), not the equipment.
- yaml/topic IDs are `snake_case`; Gazebo model/link names are `PascalCase` derived mechanically.
- "Arm" means the drone arm only. Robots are "scara" and "gang head". Never "robot arm" (a
  SCARA's own segments are "links").
- A SCARA's joints are `scara_<station>_j1..j4` (shoulder, elbow, Z, wrist:
  `scara_build_frame_j1`), its links `Scara_<Name>_{base,link1,link2,quill,wrist}` and
  `Scara_<Name>_Tcp`, its controller `scara_<station>_controller` (D-012).
