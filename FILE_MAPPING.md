# Complete File Mapping - TKC China Reverse Project

## 📂 Directory Tree Structure

```
tkc_china_reverse/
│
├── 📄 Core Python Scripts
│   ├── TKC.py (1670 lines) ⭐ MAIN VERSION
│   ├── TKC_original.py (973 lines) 📜 ORIGINAL
│   └── TKC_abuedited.py (1652 lines) ✏️ EDITED VERSION
│
├── 📋 Configuration Files
│   ├── test_new_logic.json
│   ├── devicemodel.model
│   ├── robot_calibrationfile.cp
│   └── p0001.palletobject
│
├── 📁 Task Files (.task)
│   ├── simple_auto_22_pallets.task
│   ├── neel_new_logic_auto360_testing.task
│   ├── new_neel_logic_testing.task
│   └── Eircmap_China Script Test(22 Pallets) .task
│
├── 🗺️ Map Files
│   └── map/
│       └── usa_eric_test_1_new.smap
│
├── 📸 Image Files (Visual Documentation)
│   ├── length_width_container_ramp_pallets.png ⚠️ IMPORTANT DIAGRAM
│   ├── Screenshot 2026-01-06 135521.png
│   ├── Screenshot 2026-01-06 135538.png
│   ├── Screenshot 2026-01-13 134800.png
│   ├── WhatsApp Image 2026-01-13 at 1.52.03 PM.jpeg
│   ├── WhatsApp Image 2026-01-13 at 1.52.05 PM (1).jpeg
│   ├── WhatsApp Image 2026-01-13 at 1.52.05 PM.jpeg
│   └── WhatsApp Image 2026-01-13 at 1.52.06 PM.jpeg
│
└── 📦 syspy/ (System Python Modules)
    ├── battery_Can/
    │   ├── __init__.py
    │   ├── canpass_aarch64.py
    │   ├── canpass_base.py
    │   ├── canpass_x86.py
    │   └── port_config.json
    │
    ├── battery_Serial/
    │   ├── __init__.py
    │   ├── battery_base.py
    │   ├── serialpass_aarch64.py
    │   └── serialpass_x86.py
    │
    ├── canLogger/
    │   ├── CanData.py
    │   ├── candump.py
    │   ├── CanFrame_pb2.py
    │   ├── kinco.py
    │   ├── Receive.py
    │   └── Recode2Log.py
    │
    ├── dmx512/
    │   ├── __init__.py
    │   ├── dmx512_aarch64.py
    │   ├── dmx512_base.py
    │   └── dmx512_x86.py
    │
    ├── lib/
    │   ├── __init__.py
    │   ├── char_utility.py
    │   ├── misc_utility.py
    │   ├── pass_through.py
    │   ├── rpc_client.py
    │   ├── rpc_server.py
    │   ├── udp_client.py
    │   └── udp_debug.py
    │
    └── protobuf/
        ├── __init__.py
        ├── CanFrame_pb2.py
        ├── descriptor_aarch64_pb2.py
        ├── descriptor_pb2.py
        ├── message_battery_aarch64_pb2.py
        ├── message_battery_pb2.py
        ├── message_controller_aarch64_pb2.py
        ├── message_controller_pb2.py
        ├── message_dmx512_arm_pb2.py
        ├── message_dmx512_pb2.py
        ├── message_header_aarch64_pb2.py
        ├── message_header_pb2.py
        ├── message_motorinfos_aarch64_pb2.py
        ├── message_motorinfos_pb2.py
        ├── message_movetask_aarch64_pb2.py
        ├── message_movetask_pb2.py
        ├── message_navigation_aarch64_pb2.py
    │   ├── message_navigation_pb2.py
    │   ├── message_odometer_aarch64_pb2.py
    │   ├── message_odometer_pb2.py
    │   ├── wrappers_aarch64_pb2.py
    │   ├── wrappers_pb2.py
    │   └── proto/
    │       └── [22 .proto files + 1 .options file]
    │
    └── __pycache__/ (compiled Python files)
```

---

## 📊 File Comparison Matrix

| Feature | TKC.py | TKC_original.py | TKC_abuedited.py |
|---------|--------|-----------------|------------------|
| **Lines of Code** | 1670 | 973 | 1652 |
| **PathPlanningConfig** | ✅ Yes | ❌ No | ✅ Yes |
| **BezierRetreat Class** | ✅ Yes | ❌ No | ✅ Yes |
| **StraightApproachWithSideShift** | ✅ Yes | ❌ No | ✅ Yes |
| **Pallet Counting** | ✅ Yes | ❌ No | ✅ Yes |
| **Dual Logic System** | ✅ Yes | ❌ No | ✅ Yes |
| **Startup Banner** | ✅ Yes | ❌ No | ✅ Yes |
| **CubicBezier2Load (Old)** | ✅ Yes | ✅ Yes | ✅ Yes |
| **360° Detection** | ✅ Yes | ✅ Yes | ✅ Yes |
| **Recognition System** | ✅ Yes | ✅ Yes | ✅ Yes |

---

## 🔍 Class Structure Map

### TKC.py / TKC_abuedited.py (Full Version)

```
Module (Main Controller)
├── Operations
│   ├── load() → Task List Creation
│   ├── unload() → Task List Creation
│   ├── zero() → Motor Reset
│   ├── side() → Side Motor Control
│   ├── lift() → Lift Motor Control
│   ├── tilt() → Tilt Motor Control
│   ├── rec() → Recognition
│   └── Mid360AreaDetect() → 360° Detection
│
└── Task Execution
    ├── ForkMotor (Motor Control)
    ├── Rec (Pallet Recognition)
    ├── GoToPre (Navigation)
    ├── Mid360AreaDetect (360° Detection)
    └── CubicBezier2Load (Pickup Logic)
        ├── OLD: Bezier Curve Approach
        └── NEW: Two-Phase Approach
            ├── BezierRetreat (Phase 1)
            └── StraightApproachWithSideShift (Phase 2)
```

### TKC_original.py (Original Version)

```
Module (Main Controller)
├── Operations
│   └── [Same as above]
│
└── Task Execution
    ├── ForkMotor
    ├── Rec
    ├── GoToPre
    ├── Mid360AreaDetect
    └── CubicBezier2Load
        └── OLD: Bezier Curve Approach Only
```

---

## 📝 Configuration File Details

### test_new_logic.json
```json
{
    "operation": "Script",
    "script_args": {
        "operation": "load",        // Main operation
        "truckLoad": 1,              // Load in truck mode
        "use360": 1,                 // Enable 360° detection
        "clearGBData": 1             // Clear global data
    },
    "script_name": "TKC_abuedited.py"  // Script to use
}
```

### Task Files Purpose
- **simple_auto_22_pallets.task**: Simple automation for 22 pallets
- **neel_new_logic_auto360_testing.task**: Testing new logic with 360°
- **new_neel_logic_testing.task**: Testing new logic
- **Eircmap_China Script Test(22 Pallets) .task**: China map test with 22 pallets

---

## 🎯 Key File Relationships

```
TKC.py (Main)
    ↓ uses
PathPlanningConfig
    ↓ configures
BezierRetreat + StraightApproachWithSideShift
    ↓ called by
CubicBezier2Load
    ↓ part of
Module.load() task list
    ↓ executes
Task sequence for pallet pickup
```

---

## 📸 Image Files - Expected Content

1. **length_width_container_ramp_pallets.png**
   - Container dimensions (2.48m × 12.45m)
   - Ramp layout and dimensions
   - Pallet positioning diagram
   - Safety zones visualization

2. **Screenshot 2026-01-06 135521.png & 135538.png**
   - UI screenshots from testing
   - May show configuration or runtime state

3. **Screenshot 2026-01-13 134800.png**
   - More recent test screenshot
   - Possibly showing new logic in action

4. **WhatsApp Images (4 files from Jan 13, 2026)**
   - Communication images
   - Likely contain:
     - Configuration details
     - Test results
     - Diagrams or notes
     - Problem descriptions or solutions

**⚠️ IMPORTANT**: These images contain critical visual information that should be reviewed for complete understanding of the system configuration and requirements.

---

## 🔗 Dependency Graph

```
TKC.py
├── rbk (Robot Base Kit)
│   ├── MoveStatus
│   ├── BasicModule
│   └── ParamServer
├── rbkSim (Simulation)
│   └── SimModule
├── robot
│   └── ModuleTool
├── goPath (Navigation)
│   └── Module
└── syspy/ (System Modules)
    ├── battery_Can/ (Battery CAN)
    ├── battery_Serial/ (Battery Serial)
    ├── canLogger/ (CAN Logging)
    ├── dmx512/ (Lighting)
    ├── lib/ (Utilities)
    └── protobuf/ (Protocol Buffers)
```

---

## 📌 File Usage Guide

### For Development
- **TKC.py**: Main development file (use this)
- **TKC_original.py**: Reference for original logic
- **TKC_abuedited.py**: Backup/alternative version

### For Testing
- **test_new_logic.json**: Test configuration
- **Task files**: Pre-configured test scenarios

### For Understanding
- **Image files**: Visual documentation
- **PROJECT_ANALYSIS.md**: Complete analysis (this document)
- **FILE_MAPPING.md**: File structure reference

---

## ✅ File Checklist

- [x] TKC.py - Main script analyzed
- [x] TKC_original.py - Original version analyzed
- [x] TKC_abuedited.py - Edited version analyzed
- [x] test_new_logic.json - Configuration analyzed
- [x] Task files - Identified (4 files)
- [x] Image files - Identified (8 files)
- [x] syspy/ structure - Mapped
- [x] Map files - Identified
- [x] Configuration files - Identified

---

## 🎓 Learning Path

1. **Start**: Read `TKC_original.py` to understand basic structure
2. **Compare**: Review differences in `TKC.py`
3. **Understand**: Study `PathPlanningConfig` parameters
4. **Trace**: Follow `Module.load()` → `CubicBezier2Load` → new logic
5. **Visualize**: Review image files for context
6. **Test**: Use `test_new_logic.json` configuration
7. **Debug**: Use extensive logging in new logic

---

**Last Updated**: Analysis completed after thorough code review
**Total Files Analyzed**: 3 main Python files + configuration + structure
**Lines of Code Reviewed**: ~4,300+ lines
