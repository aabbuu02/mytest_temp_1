# TKC China Reverse - Complete Project Analysis

## 📋 Project Overview
This is a robotic pallet loading/unloading system for container trucks. The system uses a forklift robot to autonomously load/unload pallets from containers using advanced path planning algorithms.

## 📁 Project Structure

### Core Python Files
1. **TKC.py** (1670 lines) - **MAIN/CURRENT VERSION**
   - Contains new path planning logic (BezierRetreat + StraightApproach)
   - Has PathPlanningConfig class with extensive configuration
   - Includes pallet counting system
   - Has startup banner confirming new logic is loaded

2. **TKC_original.py** (973 lines) - **ORIGINAL VERSION**
   - Original implementation without new path planning
   - Uses only CubicBezier2Load for pallet pickup
   - Simpler logic, no retreat/approach phases

3. **TKC_abuedited.py** (1652 lines) - **EDITED VERSION**
   - Similar to TKC.py with new path planning logic
   - Contains PathPlanningConfig and new classes
   - Appears to be a working copy/backup

### Configuration Files
- **test_new_logic.json** - Test configuration using TKC_abuedited.py
  - Operation: load
  - truckLoad: 1
  - use360: 1
  - clearGBData: 1

### Task Files (.task)
- `simple_auto_22_pallets.task`
- `neel_new_logic_auto360_testing.task`
- `new_neel_logic_testing.task`
- `Eircmap_China Script Test(22 Pallets) .task`

### Image Files (Visual Documentation)
- `length_width_container_ramp_pallets.png` - Container/pallet dimensions diagram
- `Screenshot 2026-01-06 135521.png` - Screenshot 1
- `Screenshot 2026-01-06 135538.png` - Screenshot 2
- `Screenshot 2026-01-13 134800.png` - Screenshot 3
- `WhatsApp Image 2026-01-13 at 1.52.03 PM.jpeg` - Image 1
- `WhatsApp Image 2026-01-13 at 1.52.05 PM (1).jpeg` - Image 2
- `WhatsApp Image 2026-01-13 at 1.52.05 PM.jpeg` - Image 3
- `WhatsApp Image 2026-01-13 at 1.52.06 PM.jpeg` - Image 4

### System Files
- `devicemodel.model` - Device model configuration
- `robot_calibrationfile.cp` - Robot calibration
- `p0001.palletobject` - Pallet object definition
- `map/usa_eric_test_1_new.smap` - Map file

### syspy/ Directory
Contains system Python modules:
- `battery_Can/` - CAN bus battery communication
- `battery_Serial/` - Serial battery communication
- `canLogger/` - CAN bus logging
- `dmx512/` - DMX512 lighting control
- `lib/` - Utility libraries (RPC, UDP, etc.)
- `protobuf/` - Protocol buffer definitions

---

## 🔍 Key Differences Between Files

### TKC.py vs TKC_original.py

#### NEW in TKC.py:
1. **PathPlanningConfig Class** (Lines 112-212)
   - Retreat parameters (retreat_distance, retreat_curve_factor)
   - Safety parameters (wall_clearance, container_width, robot_width)
   - Safe Y range calculation (±0.415m)
   - Pallet strategy (first_n_pallets_use_old_logic = 0)
   - Side-shifter parameters
   - Ramp safety parameters
   - Debug/logging parameters
   - Configuration validation

2. **BezierRetreat Class** (Lines 970-1199)
   - Generates gentle Bezier curve for retreating from pallet
   - Calculates safe target Y position
   - Avoids 90/180 degree turns
   - Maximum angle change validation

3. **StraightApproachWithSideShift Class** (Lines 1201-1364)
   - Straight backward approach to pallet
   - Dynamic side-shifter adjustment
   - Fork insertion detection (DI 16/17)
   - Automatic pallet switching

4. **Enhanced CubicBezier2Load** (Lines 1365-1665)
   - Dual logic support (old Bezier vs new retreat+straight)
   - Pallet number tracking
   - Phase management (retreat → approach)
   - Logic determination based on pallet count

5. **Pallet Counting System** (Module.load method, Lines 466-519)
   - Tracks pallet count in global data
   - Increments on each load operation
   - Used to determine which logic to use

6. **Startup Banner** (Lines 215-228)
   - Confirms new path planning logic is loaded
   - Shows configuration status

#### REMOVED from TKC_original.py:
- All new path planning classes
- PathPlanningConfig
- Pallet counting
- Dual logic system

---

## 🏗️ Core Classes Architecture

### 1. Module (Main Controller)
- **Purpose**: Main orchestrator for all operations
- **Key Methods**:
  - `load()`: Sets up load task sequence
  - `unload()`: Sets up unload task sequence
  - `get_load_type()`: Determines load type (first, in_rec, in_not_rec)
- **Task List Management**: Sequential task execution

### 2. RecOutput (Global State)
- **Purpose**: Stores pallet recognition results globally
- **Properties**:
  - `valid`: Recognition validity
  - `results`: Recognition results array
  - `currentPoint`: Current pallet position
  - `nextPoint`: Next pallet position
  - `good_location`: 360 detection location
  - `has_goods`: Goods detection flag

### 3. GoToPre (Navigation)
- **Purpose**: Navigate to pre-position points
- **Types**:
  - Type 0/4: 360 detection navigation
  - Type 2: Pre-position for recognition
  - Type 3: Navigate to pallet
  - Type 5: Backward retreat

### 4. Rec (Recognition)
- **Purpose**: Pallet recognition using camera
- **Features**:
  - Multiple recognition attempts (max 5)
  - Handles 1 or 2 pallet detection
  - Updates RecOutput with results

### 5. ForkMotor (Actuator Control)
- **Purpose**: Control fork motors (side, lift, tilt)
- **Motors**:
  - `sideshift5`: Lateral movement
  - `lift3`: Vertical movement
  - `tilt6`: Tilt adjustment

### 6. Mid360AreaDetect (360° Detection)
- **Purpose**: Detect goods using 360° LiDAR/camera
- **Process**:
  - Reads point cloud data
  - Filters points in detection zone
  - Determines if goods exist
  - Returns approximate goods location

### 7. CubicBezier2Load (OLD LOGIC)
- **Purpose**: Original Bezier curve approach to pallet
- **Process**:
  1. Calculate side offset
  2. Generate cubic Bezier curve
  3. Execute curve path
  4. Detect fork insertion

### 8. BezierRetreat (NEW LOGIC - Phase 1)
- **Purpose**: Retreat from pallet using gentle Bezier curve
- **Process**:
  1. Determine safe target Y position
  2. Calculate retreat distance (2.0m default)
  3. Generate Bezier curve (gentle, <45° turns)
  4. Execute retreat path

### 9. StraightApproachWithSideShift (NEW LOGIC - Phase 2)
- **Purpose**: Straight backward approach with side-shift compensation
- **Process**:
  1. Calculate pallet relative position
  2. Determine required side-shift
  3. Adjust side-shifter dynamically
  4. Execute straight backward path
  5. Detect fork insertion (DI 16/17)

---

## ⚙️ Configuration Parameters

### PathPlanningConfig (TKC.py only)
```python
# Retreat Parameters
retreat_distance = 2.0              # Meters to retreat
retreat_curve_factor = 0.3          # Bezier curve bend (0-1)

# Safety Parameters
wall_clearance = 0.2                # 200mm from walls
container_width = 2.48              # Container width (m)
robot_width = 1.25                  # Robot width (m)
safe_y_max = 0.415                  # Max safe Y (±415mm)
safe_y_min = -0.415                 # Min safe Y

# Pallet Strategy
first_n_pallets_use_old_logic = 0  # First N use old logic
enable_new_path_logic = True        # Enable new logic

# Side-shifter
dynamic_sideshifter_enabled = True
sideshifter_max = 0.2               # ±200mm limit

# Ramp Safety
ramp_width = 1.8
ramp_safe_y_max = 0.4
ramp_safe_y_min = -0.4
```

### GoodsAreaDetect
```python
x_min = -1.7, x_max = -5           # Detection X range
y_min = -0.6, y_max = 0.6          # Detection Y range
z_up = 0.6, z_down = 0.25          # Detection Z range
goods_pre_dist = 3                 # Distance after detection
go2pallet_b3_load_dist = 4         # Distance to pallet
a = 2.48, b = 10.0                 # Container dimensions
```

### CubicBezierPar
```python
t = -0.08                           # Turn amplitude
d_l = -0.28                         # Left side parameter
d_r = 0.28                          # Right side parameter
extend_factor = 0.7                 # Extension factor
odo_2_pallet_dist = 0.45            # Max Y movement
backDist = 0.02                     # Backward distance
```

### GOPAthPar
```python
max_speed = 0.1                     # Max navigation speed
maxRot = 10                         # Max rotation speed
reachDist = 0.02                    # Reach distance tolerance
reachAngle = 0.3°                   # Reach angle tolerance
```

---

## 🔄 Load Operation Flow

### Load Type: first (First Pallet)
1. Reset side motor to 0
2. Lift fork to height
3. **[Optional]** Mid360AreaDetect (if use360=1)
4. Rec (pallet recognition)
5. GoToPre(3) - Navigate to pallet
6. CubicBezier2Load - Pickup pallet
   - **NEW LOGIC**: Retreat → Straight Approach
   - **OLD LOGIC**: Bezier curve approach
7. Lift to end height
8. GoToPre(5) - Backward retreat (1.9m)
9. Reset side motor to 0

### Load Type: in_rec (Need Recognition)
1. Reset side motor
2. Lift fork
3. GoToPre(2) - Pre-position
4. Rec - Recognition
5. GoToPre(3) - Navigate forward
6. CubicBezier2Load
7. Lift to end height
8. GoToPre(5) - Backward
9. Reset side motor

### Load Type: in_not_rec (No Recognition Needed)
1. Reset side motor
2. Lift fork
3. GoToPre(3) - Navigate to pallet
4. CubicBezier2Load
5. Lift to end height
6. GoToPre(5) - Backward
7. Reset side motor

---

## 🆕 New Path Planning Logic Details

### When is New Logic Used?
- Condition: `pallet_number > first_n_pallets_use_old_logic`
- Default: `first_n_pallets_use_old_logic = 0` (all pallets use new logic)
- Production: Set to 4 (pallets 1-4 use old, 5-22 use new)

### Phase 1: BezierRetreat
1. **Calculate Safe Target Y**:
   - Left pallet (Y < -0.2) → Retreat right (+0.6m offset)
   - Right pallet (Y > 0.2) → Retreat left (-0.6m offset)
   - Center pallet → Stay center (Y = 0)
   - Clamp to safe range: [-0.415, +0.415]

2. **Generate Bezier Curve**:
   - Start: Current robot position
   - End: Retreat target (2.0m forward, safe Y)
   - Control points: Gentle curve (factor = 0.3)
   - Validation: Max angle change < 45°

3. **Execute Retreat**:
   - Forward mode (backMode = False)
   - Speed: 0.1 m/s
   - Tolerance: 5cm

### Phase 2: StraightApproachWithSideShift
1. **Calculate Approach**:
   - Pallet position from recognition
   - Robot position after retreat
   - Calculate lateral offset

2. **Side-Shifter Adjustment**:
   - If offset ≤ 0.2m: Full compensation
   - If offset > 0.2m: Clamp to ±0.2m limit
   - Dynamic adjustment during approach

3. **Straight Path**:
   - Backward mode (backMode = True)
   - Simple 2-point path (current → pallet)
   - Speed: 0.1 m/s
   - Tolerance: 2cm

4. **Completion**:
   - Fork insertion detected (DI 16/17)
   - Switch to next pallet
   - Update global data

---

## 📊 Key Metrics & Constraints

### Container Dimensions
- Width: 2.48m
- Length: 12.45m (cargo_width)
- Height: 2m (cargo_length)

### Robot Dimensions
- Width: 1.25m
- Safe Y range: ±0.415m (calculated)
- Ramp safe Y: ±0.4m

### Safety Margins
- Wall clearance: 200mm minimum
- Safe Y calculation:
  ```
  safe_y = (container_width/2) - (robot_width/2) - wall_clearance
         = (2.48/2) - (1.25/2) - 0.2
         = 1.24 - 0.625 - 0.2
         = 0.415m
  ```

### Movement Constraints
- Max speed: 0.1 m/s
- Max rotation: 10°/s
- Side-shifter range: ±200mm
- Retreat distance: 2.0m (configurable)
- Backward retreat: 1.9m (after pickup)

---

## 🔧 Dependencies

### External Modules
- `rbk` - Robot base kit
- `rbkSim` - Robot simulation
- `robot` - Robot utilities
- `goPath` - Path navigation module

### Internal Modules (syspy/)
- Battery communication (CAN/Serial)
- CAN logging
- DMX512 control
- Protocol buffers
- RPC/UDP utilities

---

## 📝 Notes & Observations

1. **Version**: All files marked as Version:20251106-1

2. **Language**: Mixed Chinese/English comments
   - Chinese: User-facing tips and comments
   - English: Technical documentation and logs

3. **Logging**: Extensive logging in new logic
   - PathPlanningConfig.log() for unified logging
   - Verbose logging can be enabled/disabled
   - Path points logging (optional, verbose)

4. **Error Handling**:
   - MoveStatus enum (NONE, RUNNING, FINISHED, FAILED, SUSPENDED)
   - Error reporting via r.setError()
   - Warning system via r.setWarning()

5. **Global Data Management**:
   - Uses r.getGData() / r.setGData() for persistence
   - Stores pallet count, recognition results
   - Cleared on first load or when clearGBData=1

6. **Recognition System**:
   - Supports detecting 1 or 2 pallets simultaneously
   - If 2 pallets detected: currentPoint + nextPoint
   - Optimizes by skipping recognition if nextPoint exists

7. **360° Detection**:
   - Uses DJI-mid360-TCP camera
   - Point cloud processing (4000 points per slice)
   - Filters points in detection zone
   - Requires ≥3 points to confirm goods

8. **Fork Insertion Detection**:
   - Digital inputs: DI 16 and DI 17
   - Either input triggers completion
   - Used in both old and new logic

---

## 🎯 Key Features

### Old Logic (CubicBezier2Load)
- ✅ Single-phase Bezier curve approach
- ✅ Side-shifter adjustment
- ✅ Works well for first 4 pallets (ramp area)
- ⚠️ May cause large turning angles in tight spaces

### New Logic (Retreat + Straight)
- ✅ Two-phase approach (retreat → straight)
- ✅ Gentle curves (no 90/180° turns)
- ✅ Dynamic side-shifter compensation
- ✅ Better for pallets 5-22 (container interior)
- ✅ Safer wall clearance management

---

## 🚀 Usage

### Testing New Logic
```json
{
    "operation": "Script",
    "script_args": {
        "operation": "load",
        "truckLoad": 1,
        "use360": 1,
        "clearGBData": 1
    },
    "script_name": "TKC_abuedited.py"
}
```

### Configuration
- Edit `PathPlanningConfig` class in TKC.py
- Restart script after modification
- Use `first_n_pallets_use_old_logic` to control which pallets use which logic

---

## 📸 Image Files Analysis

Based on filenames, images likely contain:
1. **length_width_container_ramp_pallets.png**: Technical diagram showing container dimensions, ramp layout, and pallet positioning
2. **Screenshots**: UI screenshots from testing (dates: Jan 6, Jan 13, 2026)
3. **WhatsApp Images**: Communication images with details about the system (Jan 13, 2026)

**Note**: These images contain important visual documentation that should be reviewed for:
- Container/pallet layout
- Ramp configuration
- Testing scenarios
- Configuration details

---

## ✅ Summary

This is a sophisticated robotic pallet handling system with:
- **Dual path planning strategies** (old Bezier vs new retreat+straight)
- **Adaptive logic selection** based on pallet number
- **Comprehensive safety checks** (wall clearance, ramp limits)
- **Robust recognition system** (360° detection + camera recognition)
- **Extensive logging and debugging** capabilities

The system is designed to handle 22 pallets in a container truck, with different strategies for ramp area (first 4) vs container interior (remaining 18).
