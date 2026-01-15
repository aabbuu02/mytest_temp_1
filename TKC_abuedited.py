import rbk as RBK
from rbk import MoveStatus, BasicModule, ParamServer
from rbkSim import SimModule
from datetime import datetime
from robot import ModuleTool
import math, time, json, goPath

# Version:20251106-1
"""
####BEGIN DEFAULT ARGS####
{
    "operation":{
        "value": "zero",
        "default_value":["Mid360AreaDetect","load","unload","zero","rec","B3"],
        "tips": "Operation mode selection (load/unload/zero/rec/etc)",
        "type": "complex"        
    },
    "side":{
        "value": "",
        "tips":"Fork lateral movement (side shift)",
        "type":"float",
        "unit": "m"
    },
    "tilt":{
        "value": "",
        "tips":"Fork forward tilt",
        "type":"float",
        "unit": "m"
    },
    "lift":{
        "value": "",
        "tips":"Fork lift height",
        "type":"float",
        "unit": "m"
    },
    "end_height":{
        "value": "",
        "tips":"Fork lift end height (after pickup)",
        "type":"float",
        "unit": "m"
    },
    "truckLoad":{
        "value": "",
        "tips":"load in truck",
        "type":"float",
        "unit": "m"
    },
    "use360":{
        "value": "",
        "tips":"Whether to use 360 degree detection for goods",
        "type":"float",
        "unit": "m"
    },
    "clearGBData":{
        "value": "",
        "tips":"Clear global data (reset pallet tracking)",
        "type":"float",
        "unit": "m"
    }
}
####END DEFAULT ARGS####
"""


class GOPAthPar:
    max_speed = 0.1
    maxRot = 10
    reachDist = 0.02
    reachAngle = math.radians(0.3)


class GoodsAreaDetect:
    """Area for 360 degree goods detection: rectangular space behind robot, reference point is odometry center"""
    x_min = -1.7                    #-1.5
    x_max = -5                      #-5
    y_min = -0.6                    #-0.8
    y_max = 0.6                     #0.8
    z_up = 0.6                      #0.5
    z_down = 0.25                   #0.25
    """
    goods_pre_dist: After 360 detection, move robot to maintain distance from goods, default 3m.
    Adjust based on actual conditions: camera should be able to recognize pallet.
    If >3.5m may not recognize, if <1m may not recognize.
    """
    goods_pre_dist = 3
    """go2pallet: Distance to navigate to pallet front without using 360 detection"""
    go2pallet_b3_load_dist = 4

    """Container dimensions (width and length)"""
    a = 2.48
    b = 10.0

    """If loading from platform, distance from pallet to reference point"""
    load_pallet_in_platform_dist = 0.1

    """Distance to approach container when starting fork insertion"""
    B_dist = 0.15
    enable_B_dist = 2  # Activation condition for B_dist: distance between pallet center and robot odometry center


class CubicBezierPar:
    """
    Third-order Bezier curve control points and parameters, similar to drawing Bezier curves in roboshop
    """
    t = -0.08  # Smaller value = smaller turning amplitude
    d_l = -0.28  # Smaller value = smaller starting angle, for picking from robot's left side
    d_r = 0.28  # Smaller value = smaller starting angle, for picking from robot's right side
    extend_factor = 0.7
    odo_2_pallet_dist = 0.45  # Maximum allowed distance from odometry center to pallet, limits robot Y movement in container. Larger value = more Y movement allowed
    backDist = 0.02  # Backward distance for approach


class PathPlanningConfig:
    """
    New Path Planning Configuration
    For Bezier retreat + straight approach strategy
    
    LOG: All parameters are configurable, restart script after modification
    """
    # ============ Retreat Parameters ============
    retreat_distance = 2.0              # Distance to retreat before straight approach (meters)
                                        # LOG: Increase for more retreat, requires more space
    
    retreat_curve_factor = 0.3          # Bezier curve bend factor (0-1)
                                        # LOG: Smaller value = gentler curve, less turning angle
                                        # NOTE: User requires no 90/180 degree turns, keep <0.4
    
    max_rotation_angle = 15.0           # Maximum rotation angle during curves (degrees)
                                        # LOG: Prevent large rotations that could cause wall collisions
                                        # LOG: Keep robot rotation minimal to avoid touching container walls
    
    # ============ Safety Parameters ============
    # ============ Outside Testing Mode ============
    outside_testing_mode = True         # Set to True for testing outside container (wider safe bounds)
                                        # LOG: When True, uses wider safe Y bounds (±1.0m) for outside testing
                                        # LOG: When False, uses container safe bounds (±0.365m)
                                        # LOG: Change to False when testing inside container

    # ============ Robot / Fork Orientation ============
    forks_at_front = True               # True: forks are at robot FRONT (common forklift)
                                        # False: forks are at robot BACK (some AGV layouts)
                                        # LOG: This flips retreat/approach driving direction (backMode) safely
    
    wall_clearance = 0.25               # Safety margin from container walls (meters)
                                        # LOG: 250mm minimum clearance to prevent wall collision (USER REQUIREMENT)
    
    container_width = 2.48              # Container width (meters)
    robot_width = 1.25                  # Robot body width (meters)
    
    # Calculate safe Y range
    # For container: (container_width/2) - (robot_width/2) - wall_clearance
    # = (2.48/2) - (1.25/2) - 0.25 = 1.24 - 0.625 - 0.25 = 0.365
    # For outside: Use much wider bounds since no container walls
    outside_safe_y_max = 5.0            # meters (outside testing)
    outside_safe_y_min = -5.0           # meters (outside testing)
    safe_y_max = outside_safe_y_max if outside_testing_mode else 0.365  # Maximum safe Y position (meters)
    safe_y_min = outside_safe_y_min if outside_testing_mode else -0.365  # Minimum safe Y position (meters)
                                        # LOG: Outside mode: ±5.0m, Container mode: ±365mm (with 250mm wall clearance)
    
    # ============ Pallet Strategy ============
    first_n_pallets_use_old_logic = 0   # First N pallets use ramp-specific logic
                                        # LOG: Set to 0 to test new logic for ALL pallets (recommended for outside testing)
                                        # LOG: Set to 4 for production (pallets 1-4 ramp area, 5-22 container area)
                                        # LOG: First 4 pallets are on ramp, need special handling
                                        # LOG: For outside single pallet test, keep at 0 to use new 4-phase logic

    
    enable_new_path_logic = True        # Enable new path planning logic
                                        # LOG: Set to False to use old logic for all (testing)
    
    # ============ Side-shifter Parameters ============
    dynamic_sideshifter_enabled = True  # Enable dynamic side-shift adjustment
                                        # LOG: Dynamically adjust side-shifter during straight approach
    
    sideshifter_max = 0.21              # Maximum side-shifter range (±210mm)
                                        # LOG: Hardware limit ±210mm (USER REQUIREMENT)

    # Side-shift direction mapping (IMPORTANT)
    # Your hardware: +0.21m = forks move RIGHT, -0.21m = forks move LEFT.
    # Set this so that positive command in code still means "move towards pallet".
    # - True  => +value moves forks LEFT
    # - False => +value moves forks RIGHT  (YOUR ROBOT)
    sideshifter_positive_is_left = False
    
    # ============ Ramp Safety Parameters ============
    ramp_width = 1.8                    # Ramp width (meters)
    ramp_safe_y_max = 0.4               # Safe Y position on ramp (meters)
    ramp_safe_y_min = -0.4              # Safe Y position on ramp (meters)
                                        # LOG: Ramp narrower than container, stricter Y limits
    
    ramp_center_scan_point = None       # Center point of ramp for scanning (will be set from map)
                                        # LOG: Robot scans at ramp center, then aligns to ramp width
    
    # ============ Debug Parameters ============
    verbose_logging = True              # Verbose logging output
                                        # LOG: Set to True to see detailed path planning logs
    
    log_path_points = False             # Log path points
                                        # LOG: Set to True to log all Bezier points (verbose)
    
    @staticmethod
    def log(message, level="INFO"):
        """Unified logging output"""
        if PathPlanningConfig.verbose_logging:
            prefix = f"[PathPlanning-{level}]"
            print(f"{prefix} {message}")
            # Try to force flush
            import sys
            sys.stdout.flush()

    @staticmethod
    def validate_config():
        """Validate configuration parameters"""
        errors = []
        
        # Skip container-specific validation in outside testing mode
        if not PathPlanningConfig.outside_testing_mode:
            # Check safe range (only for container mode)
            calculated_safe_y = (PathPlanningConfig.container_width / 2) - \
                               (PathPlanningConfig.robot_width / 2) - \
                               PathPlanningConfig.wall_clearance
            
            if abs(calculated_safe_y - PathPlanningConfig.safe_y_max) > 0.01:
                errors.append(f"safe_y_max incorrect: {PathPlanningConfig.safe_y_max} "
                             f"should be: {calculated_safe_y:.3f}")
            
            # Check wall clearance meets requirement (only for container mode)
            if PathPlanningConfig.wall_clearance < 0.25:
                errors.append(f"wall_clearance too small: {PathPlanningConfig.wall_clearance}m, "
                             f"minimum required: 0.25m (250mm)")
        else:
            # Outside testing mode - just log the configuration
            PathPlanningConfig.log("Outside testing mode enabled - using wider safe bounds", "INFO")
            PathPlanningConfig.log(f"  Safe Y range: [{PathPlanningConfig.safe_y_min:.3f}, "
                                  f"{PathPlanningConfig.safe_y_max:.3f}]m", "INFO")
        
        # Check retreat distance (applies to both modes)
        if PathPlanningConfig.retreat_distance < 1.0:
            errors.append(f"retreat_distance too small: {PathPlanningConfig.retreat_distance}m, "
                         f"recommend >= 1.5m")
        
        # Check curve factor (applies to both modes)
        if PathPlanningConfig.retreat_curve_factor >= 0.5:
            errors.append(f"retreat_curve_factor too large: {PathPlanningConfig.retreat_curve_factor}, "
                         f"may cause large turning angles")
        
        if errors:
            PathPlanningConfig.log("Configuration validation failed:", "ERROR")
            for error in errors:
                PathPlanningConfig.log(f"  - {error}", "ERROR")
            return False
        else:
            PathPlanningConfig.log("Configuration validated successfully", "INFO")
            if not PathPlanningConfig.outside_testing_mode:
                PathPlanningConfig.log(f"  - Safe Y range: [{PathPlanningConfig.safe_y_min:.3f}, "
                                      f"{PathPlanningConfig.safe_y_max:.3f}]m", "INFO")
            PathPlanningConfig.log(f"  - Retreat distance: {PathPlanningConfig.retreat_distance}m", "INFO")
            PathPlanningConfig.log(f"  - Curve factor: {PathPlanningConfig.retreat_curve_factor}", "INFO")
            return True


class LoadType(int):
    first = 0  # First pickup: use 360 detection for goods, then recognize and pick
    second = 1  # Goods at container opening
    in_rec = 2  # No 360 detection, need recognition for pickup, simpler than first (one less step)
    in_not_rec = 3  # No 360 detection, no recognition needed (previous recognition detected 2 pallets, use that result directly for efficiency)


class MotorName(str):
    side = "sideshift5"
    lift = "lift3"
    tilt = "tilt6"


class RecOutput:
    """Global variables for pallet recognition and tracking"""
    valid = False  # Whether pallet recognition is valid
    results = []  # Results from most recent pallet recognition
    nextPoint = None  # If data exists, coordinates of next pallet
    currentPoint = None  # If data exists, coordinates of current nearest pallet
    good_location = []  # After 360 detection, approximate goods coordinates if goods detected
    has_goods = False  # Whether goods detected by 360 degree sensor

    def to_dict(self):
        # Since class attributes are shared, we only serialize instance attributes (if any)
        # Here we assume these are instance attributes (should be initialized in __init__)
        # But current code defines class attributes, so we demonstrate conversion of "assumed" instance attributes
        return {
            'valid': self.valid,
            'results': self.results,
            'nextPoint': self.nextPoint,
            'currentPoint': self.currentPoint,
            'good_location': self.good_location,
            'has_goods': self.has_goods
        }

    def to_object(self, d={}):
        if d is None:
            return
        if "valid" in d:
            self.valid = d["valid"]
        if "results" in d:
            self.results = d["results"]
        if "nextPoint" in d:
            self.nextPoint = d["nextPoint"]
        if "currentPoint" in d:
            self.currentPoint = d["currentPoint"]
        if "good_location" in d:
            self.good_location = d["good_location"]
        if "has_goods" in d:
            self.has_goods = d["has_goods"]

    def currentPoint_switch_nextPoint(self):
        """
        When currentPoint pallet task is complete, call this function to update currentPoint
        """
        self.good_location = [self.currentPoint["x"], self.currentPoint["y"], self.currentPoint["yaw"]]
        self.currentPoint = None
        if self.nextPoint:
            self.currentPoint = self.nextPoint
            self.nextPoint = None


class Module(BasicModule):
    def __init__(self, r: SimModule, args):
        super(Module, self).__init__()
        # Container parameters
        self.truckLoad = None
        self.end_height = None
        self.load_type = None
        self.id = None
        self.cargo_length = 2
        self.cargo_width = 12.45
        self.entry_point_id = ""
        self.rec_file = "plt/p0001.plt"
        self.init_loc = None

        self.use360 = 0
        self.tilt_pos = None
        self.lift_height = None
        self.side_pos = None
        self.task_list = []
        self.task_id = 0
        self.operation_status = MoveStatus.NONE
        p = ParamServer(__file__)
        self.timeout = p.loadParam("timeout", type="int", default=600, maxValue=300, minValue=0, unit="s",
                                   comment=" Operation timeout time")
        self.init = True
        self.status = MoveStatus.NONE
        self.report_info = dict()
        self.start_time = time.time()
        self.goods_id = ""
        self.operation = None
        self.operations = self._init_operations()
        self.handle = None
        self.GData = RecOutput()
        self.is_clear_GB_data = None
        self.lift_speed, self.side_speed, self.tilt_speed = 0.9, 0.5, 0.1

    def run(self, r: SimModule, args):
        self.status = MoveStatus.RUNNING
        if self.init:
            """==== Input parameter initialization function ==== """
            # 验证路径规划配置 Validate path planning configuration
            PathPlanningConfig.log("=" * 80, "INFO")
            PathPlanningConfig.log("TKC 脚本启动 TKC Script Starting", "INFO")
            PathPlanningConfig.log("=" * 80, "INFO")
            if not PathPlanningConfig.validate_config():
                r.setError("路径规划配置验证失败 Path planning configuration validation failed! 请检查参数 Please check parameters.")
                self.status = MoveStatus.FAILED
                return self.status
            
            r.clearWarning(54402)
            r.clearWarning(56010)
            r.clearWarning(57003)
            r.clearWarning(54014)
            self.init_args(r, args)
            if not self.handle:
                """==== Initialize operation, determine task content ==== """
                self.handle = self.get_handle(r, args)
                if not self.handle:
                    r.setError(f"找不到输入的操作:{self.operation}，检检查脚本输入参数。")
                    self.status = MoveStatus.FAILED
                    return self.status
            self.init = False
        if not self.init and self.status != MoveStatus.FINISHED:
            """==== Initialization complete, start running task ==== """
            self.handle_robot(r)
        self.status = self.operation_status
        self.setAndCheckGoodsStatus(r)
        """==== Loop end, report data (print) ==== """
        self.report_data(r)
        r.setInfo(json.dumps(self.report_info))
        r.logInfo(json.dumps(self.report_info))
        return self.status

    def setAndCheckGoodsStatus(self, r):
        if self.operation == "load":
            if self.status == MoveStatus.RUNNING:
                if r.hasGoods():
                    r.setError(f"fork has goods:{self.operation}")
                    self.status = MoveStatus.FAILED
            if self.status == MoveStatus.FINISHED:
                r.forkGoods(True, self.rec_file)
        if self.operation == "unload":
            if self.status == MoveStatus.FINISHED:
                r.clearGoodsShape()

    def init_args(self, r: SimModule, args: dict):
        self.start_time = time.time()
        """==== Initialize operation, process script input parameters as needed ==== """
        self.operation = args.get("operation")
        self.side_pos = args.get("side")
        self.lift_height = args.get("lift", 0.080)
        self.truckLoad = args.get("truckLoad", 0)
        self.end_height = args.get("end_height", self.lift_height + 0.1) if self.lift_height else None
        self.tilt_pos = args.get("tilt")
        self.use360 = args.get("use360", 0)
        self.init_loc = [r.loc()['x'], r.loc()['y'], r.loc()['angle']]
        self.is_clear_GB_data = args.get("clearGBData", None)
        self.report_info["args"] = args
        # Clear global data
        if any([self.is_clear_GB_data, self.use360, not self.truckLoad]):
            for _ in range(3):
                r.setGData({})
        if g_data := r.getGData():
            self.GData.to_object(g_data)
        """==== Initialize operation, get navigation task parameters as needed ==== """
        self.get_move_task_params(r)  # Get task goodsId

    def handle_robot(self, r: SimModule):
        if self.operation_status == MoveStatus.NONE:
            self.operation_status = MoveStatus.RUNNING
        else:
            self.run_tak_list(r)

    def run_tak_list(self, r):
        if self.task_id < len(self.task_list):
            if self.task_list[self.task_id].status == MoveStatus.NONE:
                self.task_list[self.task_id].reset(r)
            elif self.task_list[self.task_id].status == MoveStatus.FINISHED:
                self.task_id = self.task_id + 1
            elif self.task_list[self.task_id].status == MoveStatus.FAILED:
                self.operation_status = MoveStatus.FAILED
            else:
                self.task_list[self.task_id].run(r, self)
        else:
            self.operation_status = MoveStatus.FINISHED

    def get_handle(self, r: SimModule, args: dict):
        handler = self.operations.get(self.operation or next((op for op in args if op in self.operations), None))
        if handler:
            handler(r)
        return handler

    def check_timeout(self, r: SimModule):
        if time.time() - self.start_time > self.timeout:
            r.setError(f"running time out")
            self.status = MoveStatus.FAILED

    def suspend(self, r: SimModule):
        # =====Handle business when task is suspended=====
        r.setNotice(f"suspend task")
        self.status = MoveStatus.SUSPENDED

    def _init_operations(self):
        """Initialize operation mapping"""
        return {
            "load": self.load, "unload": self.unload, "side": self.side,
            "lift": self.lift, "tilt": self.tilt, "zero": self.zero,
            "rec": self.rec, "Mid360AreaDetect": self.Mid360AreaDetect,
        }

    # Simplified operation methods
    def Mid360AreaDetect(self, r):
        self.task_list = [Mid360AreaDetect(r)]

    def side(self, r):
        self.task_list = [ForkMotor(r, MotorName.side, self.side_pos, self.side_speed)]

    def rec(self, r):
        self.task_list = [Rec(r, self.rec_file)]

    def tilt(self, r):
        self.task_list = [ForkMotor(r, MotorName.tilt, self.tilt_pos)]

    def lift(self, r):
        self.task_list = [ForkMotor(r, MotorName.lift, self.lift_height, self.lift_speed)]

    def zero(self, r):
        self.task_list = [ForkMotor(r, MotorName.tilt, 0.0, self.tilt_speed),
                          ForkMotor(r, MotorName.side, 0.0, self.side_speed),
                          ForkMotor(r, MotorName.lift, 0.095, self.lift_speed)]

    def load(self, r: SimModule):
        # Pallet count tracking: Increment count at start of each load operation
        g_data_dict = r.getGData()
        if not isinstance(g_data_dict, dict):
            g_data_dict = {}
        
        pallet_count = g_data_dict.get('pallet_count', 0) + 1
        g_data_dict['pallet_count'] = pallet_count
        r.setGData(g_data_dict)
        
        PathPlanningConfig.log(f"=" * 60, "INFO")
        PathPlanningConfig.log(f"Starting load operation, Pallet number: {pallet_count}", "INFO")
        PathPlanningConfig.log(f"=" * 60, "INFO")
        
        self.load_type = self.get_load_type(r)
        if self.load_type == LoadType.first:
            self.task_list = [
                ForkMotor(r, MotorName.side, 0, self.side_speed),
                ForkMotor(r, MotorName.lift, self.lift_height, self.lift_speed),
                Rec(r, self.rec_file),  # Step 3: Pallet recognition
                GoToPre(r, 3),  # Step 4: Navigate to pallet
                CubicBezier2Load(r),  # Step 5: Plan curve path for pickup (includes new precise pickup)
                ForkMotor(r, MotorName.lift, self.end_height, self.lift_speed),  # Step 6: Lift pallet
                GoToPre(r, 6),  # Step 7: Move forward 1m (after lift) - NEW
                ForkMotor(r, MotorName.side, 0, self.side_speed)  # Step 8: Reset sideshifter to center
            ]
            if self.use360:
                self.task_list.insert(2, Mid360AreaDetect(r, self.entry_point_id))  # Step 1: 360 degree goods detection
        elif self.load_type == LoadType.second:
            self.task_list = []
        elif self.load_type == LoadType.in_rec:
            self.task_list = [
                ForkMotor(r, MotorName.side, 0, self.side_speed),
                ForkMotor(r, MotorName.lift, self.lift_height, self.lift_speed),
                GoToPre(r, 2),  # Step 1: Reach pre-position point (based on previous pallet position)
                Rec(r, self.rec_file),  # Step 2: Pallet recognition
                GoToPre(r, 3),  # Step 3: Move forward a distance
                CubicBezier2Load(r),  # Step 4: Plan curve path for pickup (includes new precise pickup)
                ForkMotor(r, MotorName.lift, self.end_height, self.lift_speed),  # Step 5: Lift pallet
                GoToPre(r, 6),  # Step 6: Move forward 1m (after lift) - NEW
                ForkMotor(r, MotorName.side, 0, self.side_speed)  # Step 7: Reset sideshifter to center
            ]
        elif self.load_type == LoadType.in_not_rec:  # No recognition needed, direct pickup
            self.task_list = [
                ForkMotor(r, MotorName.side, 0, self.side_speed),
                ForkMotor(r, MotorName.lift, self.lift_height, self.lift_speed),
                GoToPre(r, 3),  # Step 1: Reach pre-position point
                CubicBezier2Load(r),  # Step 2: Plan curve path for pickup (includes new precise pickup)
                ForkMotor(r, MotorName.lift, self.end_height, self.lift_speed),  # Step 3: Lift pallet
                GoToPre(r, 6),  # Step 4: Move forward 1m (after lift) - NEW
                ForkMotor(r, MotorName.side, 0, self.side_speed)  # Step 5: Reset sideshifter to center
            ]
        else:
            self.status = MoveStatus.FAILED
            r.setError("Load type incorrect")

    def get_load_type(self, r: SimModule):
        current_point = self.GData.currentPoint
        good_location = self.GData.good_location
        has_goods = self.GData.has_goods
        if not has_goods and not current_point and not good_location:
            self.load_type = LoadType.first
        elif current_point and good_location:
            self.load_type = LoadType.in_not_rec
        elif not current_point and good_location:
            self.load_type = LoadType.in_rec
        else:
            self.load_type = -1
        return self.load_type

    def unload(self, r: SimModule):
        self.task_list = []
        if self.side_pos is not None:
            self.task_list.append(ForkMotor(r, MotorName.side, self.side_pos, self.side_speed))
        if self.tilt_pos is not None:
            self.task_list.append(ForkMotor(r, MotorName.tilt, self.tilt_pos, self.tilt_speed))
        if self.lift_height is not None:
            self.task_list.append(ForkMotor(r, MotorName.lift, self.lift_height, self.lift_speed))

    def cancel(self, r: SimModule):
        r.resetRec()
        self.status = MoveStatus.NONE

    def get_move_task_params(self, r):
        """
        Get moveTask parameters goods_id
        """
        move_task = r.moveTask()
        for p in move_task['params']:
            if p['key'] == 'goodsId':
                self.goods_id = p['string_value']
            if p['key'] == 'id':
                self.entry_point_id = p['string_value']

    def stop_robot(self, r: SimModule):
        r.setNextSpeed(json.dumps({"x": 0, "y": 0, "rotate": 0}))
        return r.getNextSpeed()['x'] == 0 and r.getNextSpeed()['y'] == 0 and r.getNextSpeed()['rotate'] == 0

    def get_LM_angle(self, r):
        l = r.getLM(self.entry_point_id, True)
        if l[3] == -1:
            return r.loc()['angle']
        return l[2]

    def report_data(self, r):
        self.report_info['take_time'] = time.time() - self.start_time
        self.report_info['task_len'] = len(self.task_list)
        self.report_info['task_id'] = self.task_id
        self.report_info['GData'] = self.GData.to_dict()
        self.report_info['load_type'] = self.load_type


class GoToPre:
    """
    Navigate to pre-position point, different go_type cases:
    0 (4): Use 360 detection for obstacles, reach pre-position point, then recognize
    1: Use previous pallet position, go to pickup point without recognition, then curve pickup
    2: Use previous pallet position, reach pre-position point, recognize, then go to pickup point again
    3: After successful recognition, go to pickup point, then curve pickup
    5: After lifting fork, retreat straight 1.5m backward
    6: After lifting fork, move forward straight 1m (NEW)
    """

    def __init__(self, r: SimModule, go_type=0, back_distance=1.9):
        self.go_type = go_type
        self.go = goPath.Module(r, dict())
        self.status = MoveStatus.NONE
        self.init = False
        self.start_time = 0
        self.tpm_info = []
        self.back_distance = back_distance
        self.x, self.y, self.yaw, self.backMode = 0.001, 0.001, 0.01, 0
        self.coordinate, self.reachDist, self.reachAngle = "robot", 0.01, 0.1
        self.maxRot = GOPAthPar.maxRot
        self.maxSpeed = GOPAthPar.max_speed
        self.useOdo = 0

    def run(self, r: SimModule, m: Module):
        self._init_move(r, m)
        self._execute_move(r, m)
        self._report_status(m)

    def _init_move(self, r: SimModule, m: Module):
        if self.init: return True
        r.resetPath()
        self.start_time = time.time()
        loc = [r.loc()['x'], r.loc()['y'], r.loc()['angle']]
        good_location = m.GData.good_location
        init_handlers = {
            0: lambda: self._init_type0(r, m, loc),
            4: lambda: self._init_type4(loc, good_location),
            2: lambda: self._init_type2(loc, good_location),
            3: lambda: self._init_type3(m, loc),
            5: lambda: self._init_type5(loc),
            6: lambda: self._init_type6(loc)  # Forward 1m after lift
        }
        if self.go_type in init_handlers:
            init_handlers[self.go_type]()
        self.go_args = {
            'x': self.x, 'y': self.y, 'theta': self.yaw,
            'coordinate': self.coordinate, 'backMode': self.backMode,
            # 'maxRot': self.maxRot, 'maxSpeed': self.maxSpeed, 'useOdo': self.useOdo,
            # 'reachDist': self.reachDist, 'reachAngle': self.reachAngle
        }
        self.init = True

    def _init_type0(self, r: SimModule, m: Module, loc):
        self.x, self.y, self.yaw, self.backMode = -1, 0.001, 0.01, 1
        distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(loc[:2], m.init_loc[:2])))
        if distance > 15:
            self.status = MoveStatus.FAILED
            r.setError(f"mid360 goods detection retreat distance exceeded {distance}")

    def _init_type4(self, loc, good_location):
        target_local = [GoodsAreaDetect.goods_pre_dist, 0, 0]
        target_world = RBK.Pos2World(target_local, good_location)

        d1 = math.dist(loc[:2], good_location[:2])
        d2 = math.dist(target_world[:2], good_location[:2])
        self.backMode = 1 if d1 > d2 else 0

        self.tpm_info = [d1, d2, target_local, self.backMode]
        self.x, self.y, self.yaw, self.coordinate = *target_world, 'world'
        self.reachAngle = 0.05

    def _init_type5(self, loc):
        # Set backward retreat parameters
        self.x, self.y, self.yaw = self.back_distance, 0.001, 0.001
        self.coordinate, self.backMode = 'robot', 0
        self.reachDist, self.reachAngle = GOPAthPar.reachDist, GOPAthPar.reachAngle
    
    def _init_type6(self, loc):
        # Set forward movement parameters (forward 1m after lift) - NEW
        # Forward movement: positive X in robot frame (backMode = False)
        self.x, self.y, self.yaw = 1.0, 0.001, 0.001  # Forward 1m
        self.coordinate, self.backMode = 'robot', 0  # Forward mode (backMode = 0 means forward, not backward)
        self.reachDist, self.reachAngle = GOPAthPar.reachDist, GOPAthPar.reachAngle

    def _init_type2(self, loc, good_location):
        good2base = RBK.Pos2Base(good_location, loc)
        target_world = RBK.Pos2World([good2base[0] + 1.5, 0, 0], loc)
        self.x, self.y, self.yaw, self.coordinate, self.backMode = *target_world, 'world', 1

    def _init_type3(self, m: Module, loc):
        current_point = m.GData.currentPoint
        pallet_loc = [current_point['x'], current_point['y'], current_point['yaw']]
        loc2pallet = RBK.Pos2Base(loc, pallet_loc)

        if abs(loc2pallet[1]) < 0.23:
            self.status = MoveStatus.FINISHED

        before_y = loc2pallet[1]
        if before_y >= GoodsAreaDetect.enable_B_dist: before_y -= GoodsAreaDetect.B_dist
        if before_y <= -GoodsAreaDetect.enable_B_dist: before_y += GoodsAreaDetect.B_dist

        target_local = [GoodsAreaDetect.go2pallet_b3_load_dist, before_y, 0]
        target_world = RBK.Pos2World(target_local, pallet_loc)

        d1, d2 = math.dist(loc[:2], pallet_loc[:2]), math.dist(target_world[:2], pallet_loc[:2])
        if abs(d1 - d2) < 0.05:
            self.status = MoveStatus.FINISHED

        self.backMode = 1 if d1 > d2 else 0
        target_base = RBK.Pos2Base(target_world, loc)

        self.tpm_info = [d1, d2, target_local, loc2pallet, target_base]
        self.x, self.y, self.yaw = target_base[0], target_base[1], 0.001
        self.reachAngle = 0.05

    def _execute_move(self, r: SimModule, m: Module):
        if not self.init: return
        self.go.run(r, self.go_args)

        if self.go.status == MoveStatus.FINISHED:
            if self.go_type == 0 and not r.getGData().get("has_goods", False):
                m.task_list.insert(m.task_id, Mid360AreaDetect(r))
            self.status = MoveStatus.FINISHED
        elif self.go.status == MoveStatus.FAILED:
            self.status = MoveStatus.FAILED

    def _report_status(self, m: Module):
        state = {
            "status": self.status,
            "go_args": self.go_args,
            "go_type": self.go_type,
            # "tmp": self.tpm_info,
            "take_time": time.time() - self.start_time
        }
        m.report_info[f"GoToPre{m.task_id}"] = state

    def reset(self, r: SimModule):
        r.resetPath()
        self.status = MoveStatus.RUNNING


class Rec:
    def __init__(self, r: SimModule, pallet_file):
        self.results = []
        self.p_num = 0
        self.start_time = None
        self.do_duration_time = 0.2
        self.filename = pallet_file
        self.rec_times = 0
        self.max_rec_times = 5
        self.status = MoveStatus.NONE
        self.init = False
        self.rec_status = -1

    def run(self, r: SimModule, m: Module):
        if not self.init:
            PathPlanningConfig.log(f"=" * 60, "INFO")
            PathPlanningConfig.log(f"Starting pallet recognition", "INFO")
            PathPlanningConfig.log(f"  Recognition file: {self.filename}", "INFO")
            r.resetRec()
            self.init = True
            self.status = MoveStatus.RUNNING
            self.start_time = time.time()
            self.do_duration_time = 0
        else:
            if time.time() - self.start_time >= self.do_duration_time:
                rec_status = r.getRecStatus()
                self.rec_status = rec_status
                PathPlanningConfig.log(f"  Recognition status: {rec_status} (0=idle, 1=running, 2=success, 3=failed)", "DEBUG")
                
                if rec_status == 3:
                    self.rec_times = self.rec_times + 1
                    PathPlanningConfig.log(f"  Recognition failed, attempt {self.rec_times}/{self.max_rec_times}", "WARN")
                    if self.rec_times > self.max_rec_times:
                        PathPlanningConfig.log(f"  Recognition failed after {self.max_rec_times} attempts!", "ERROR")
                        self.status = MoveStatus.FAILED
                        r.setError(f"Pallet recognition failed after {self.max_rec_times} attempts")
                    r.resetRec()
                elif rec_status == 0 or rec_status == 1:
                    r.doRecWithAngle(self.filename, 0.0)
                elif rec_status == 2:
                    self.p_num = r.getRecResultSize()
                    PathPlanningConfig.log(f"  Recognition successful! Detected {self.p_num} pallet(s)", "INFO")
                    
                    for num in range(1, self.p_num + 1):
                        result = r.getRecResults(num)
                        self.results.append(result)
                        PathPlanningConfig.log(f"    Pallet {num}: X={result.get('x', 'N/A'):.3f}, Y={result.get('y', 'N/A'):.3f}, "
                                              f"Yaw={math.degrees(result.get('yaw', 0)):.1f}°", "INFO")
                    
                    self.status = MoveStatus.FINISHED
                    self.update_RecOutput(r, m)
                    r.setGData(m.GData.to_dict())
                    
                    # Log which pallet was selected
                    if m.GData.currentPoint:
                        PathPlanningConfig.log(f"  Selected pallet (nearest): X={m.GData.currentPoint.get('x', 'N/A'):.3f}, "
                                              f"Y={m.GData.currentPoint.get('y', 'N/A'):.3f}", "INFO")
                    if m.GData.nextPoint:
                        PathPlanningConfig.log(f"  Next pallet stored: X={m.GData.nextPoint.get('x', 'N/A'):.3f}, "
                                              f"Y={m.GData.nextPoint.get('y', 'N/A'):.3f}", "INFO")
                    PathPlanningConfig.log(f"Recognition complete", "INFO")
                    PathPlanningConfig.log(f"=" * 60, "INFO")
        cur_state = dict()
        cur_state['pallet num'] = self.p_num
        cur_state['rec_status'] = self.rec_status
        cur_state['take_time'] = time.time() - self.start_time
        m.report_info[f'Rec-{m.task_id}'] = cur_state

    def reset(self, r: SimModule):
        r.resetRec()
        self.status = MoveStatus.RUNNING

    def update_RecOutput(self, r: SimModule, m: Module):
        """Update recognition output: select nearest pallet if 2 detected"""
        if not self.results:
            PathPlanningConfig.log(f"ERROR: No recognition results to update!", "ERROR")
            r.setError("update_RecOutput: No recognition results")
            return
        
        PathPlanningConfig.log(f"Updating recognition output with {len(self.results)} pallet(s)", "INFO")
        
        if len(self.results) == 1:
            # Single pallet detected - use it
            m.GData.results = self.results
            m.GData.valid = True
            m.GData.currentPoint = self.results[0]
            m.GData.nextPoint = []
            PathPlanningConfig.log(f"  Single pallet detected - using it as currentPoint", "INFO")
        elif len(self.results) == 2:
            # Two pallets detected - select nearest one
            m.GData.results = self.results
            m.GData.valid = True
            m.GData.currentPoint, m.GData.nextPoint = self.get_currentPoint(r)
            PathPlanningConfig.log(f"  Two pallets detected - selected nearest as currentPoint", "INFO")
            PathPlanningConfig.log(f"    CurrentPoint (nearest): X={m.GData.currentPoint.get('x', 'N/A'):.3f}, "
                                  f"Y={m.GData.currentPoint.get('y', 'N/A'):.3f}", "INFO")
            PathPlanningConfig.log(f"    NextPoint (farther): X={m.GData.nextPoint.get('x', 'N/A'):.3f}, "
                                  f"Y={m.GData.nextPoint.get('y', 'N/A'):.3f}", "INFO")

    def calculate_distance(self, rec, point2):
        """Calculate Euclidean distance between two points"""
        point = [rec['x'], rec['y'], rec['yaw']]
        # Extract coordinates
        x1, y1, yaw1 = point
        x2, y2, yaw2 = point2
        # Calculate 2D plane distance (ignore yaw, as yaw is angle, not position coordinate)
        distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        return distance

    def get_currentPoint(self, r):
        """Select nearest pallet when 2 pallets detected"""
        loc = [r.loc()['x'], r.loc()['y'], r.loc()['angle']]
        PathPlanningConfig.log(f"  Selecting nearest pallet from {len(self.results)} results", "INFO")
        PathPlanningConfig.log(f"  Robot position: X={loc[0]:.3f}, Y={loc[1]:.3f}", "INFO")
        
        # Calculate 2D distance for each object (based on origin [a, b]), and bind original data
        # Sort by distance: nearest first (reverse=True means farthest first, so [1] is nearest)
        sorted_results = sorted(self.results,
                                key=lambda obj: math.sqrt((obj["x"] - loc[0])**2 + (obj["y"] - loc[1])** 2) if abs(obj["x"] - loc[0]) <= 0.12 else abs(obj["y"] - loc[1]),
                                reverse=True)
        
        nearest = sorted_results[1]  # Nearest pallet
        farther = sorted_results[0]  # Farther pallet
        
        # Calculate distances for logging
        dist_nearest = math.sqrt((nearest["x"] - loc[0])**2 + (nearest["y"] - loc[1])**2)
        dist_farther = math.sqrt((farther["x"] - loc[0])**2 + (farther["y"] - loc[1])**2)
        
        PathPlanningConfig.log(f"  Nearest pallet: distance={dist_nearest:.3f}m, X={nearest['x']:.3f}, Y={nearest['y']:.3f}", "INFO")
        PathPlanningConfig.log(f"  Farther pallet: distance={dist_farther:.3f}m, X={farther['x']:.3f}, Y={farther['y']:.3f}", "INFO")
        PathPlanningConfig.log(f"  Selected nearest pallet for pickup", "INFO")
        
        return nearest, farther  # Return (nearest, farther) - nearest becomes currentPoint


class ForkMotor:
    def __init__(self, r: SimModule, motor, position, speed):
        """
        Args:
            motor(string): Motor name
        """
        self.start_time = None
        self.status = MoveStatus.NONE
        self.motor = motor
        self.init = False
        self.position = position
        self.speed = 0.1
        self.lift_speed = 0.9
        self.side_speed = 0.5
        self.tilt_speed = 0.1

    def reset(self, r: SimModule):
        r.resetMotor(self.motor)
        self.init = False
        self.status = MoveStatus.RUNNING

    def run(self, r: SimModule, m: Module, pos=None):
        if not self.init:
            self.start_time = time.time()
            self.status = MoveStatus.RUNNING
            self.init = True
            if pos is not None:
                self.position = pos
            r.resetMotor(self.motor)
            r.setMotorPosition(self.motor, self.position, self.speed, -1)
        else:
            if r.isMotorReached(self.motor):
                self.status = MoveStatus.FINISHED
            r.publishSpeed()
        cur_state = dict()
        cur_state["status"] = self.status
        cur_state["position"] = self.position
        cur_state["take_time"] = time.time() - self.start_time
        m.report_info[f"ForkMotor_{m.task_id}_{self.motor}"] = cur_state


class Mid360AreaDetect:
    def __init__(self, r: SimModule, site_id=None):
        """
        Detect if container has goods using mid360 sensor and return coordinates of nearest goods
        """
        self.point_num = 0
        self.node_num1 = None
        self.status = MoveStatus.NONE
        self.polygon1 = []  # Four vertex coordinates of container projection on plane
        self.init = False
        self.count_num = 0
        self.nodes = []
        self.point = []
        self.id = site_id
        self.node_slice = None
        self.site_length = 0.475
        self.ap_loc = None  # AP point coordinates
        self.cur_state = dict()
        self.start_time = None

    def reset(self, r: SimModule):

        self.status = MoveStatus.RUNNING

    def run(self, r: SimModule, m: Module):
        if not self.init:
            self.status = MoveStatus.RUNNING
            self.start_time = time.time()
            self.init = True
            self.ap_loc = [-2, 0]
            self.polygon1 = (
                ((self.ap_loc[0] + self.site_length), self.ap_loc[1] + m.cargo_length / 2),
                ((self.ap_loc[0] + self.site_length), self.ap_loc[1] - m.cargo_length / 2),
                ((self.ap_loc[0] + self.site_length - m.cargo_width / 2), self.ap_loc[1] + m.cargo_length / 2),
                ((self.ap_loc[0] + self.site_length - m.cargo_width / 2), self.ap_loc[1] - m.cargo_length / 2)
            )
            # self.cur_state["self.polygon1"] = self.polygon1
            self.read_cloud(r)
        self.rec_good(r, m)
        self.cur_state["status"] = self.status
        self.cur_state["take_time"] = time.time() - self.start_time
        a = "Mid360AreaDetect" + str(m.task_id)
        m.report_info[a] = self.cur_state

    def read_cloud(self, r):
        all_cloud = r.allCameraCloud()["allcloud"]  # Robot coordinate system
        # self.cur_state["all_cloud"] = all_cloud
        for i in all_cloud:
            if i["device"]["device_name"] == "DJI-mid360-TCP":
                self.nodes = self.nodes + i["cloud"]
        # self.cur_state["nodes"] = self.nodes
        if len(self.nodes) > 0:
            self.node_slice = [self.nodes[i:i + 4000] for i in range(0, len(self.nodes), 4000)]
        else:
            self.status = MoveStatus.FAILED
            r.setError("DJI-mid360-TCP camera has no data")
            r.logInfo(f"no DJI-mid360-TCP camera")

    def rec_good(self, r, m):
        print("rec_good:", self.count_num)
        if self.count_num != len(self.node_slice):
            for n in self.node_slice[self.count_num]:
                if GoodsAreaDetect.z_up > n["z"] > GoodsAreaDetect.z_down and GoodsAreaDetect.x_max < n[
                    "x"] < GoodsAreaDetect.x_min and GoodsAreaDetect.y_min < n["y"] < GoodsAreaDetect.y_max:
                    if self.is_point_inside_rectangle((n["x"], n["y"]), self.polygon1):  # Filter out ramp points and extra points
                        if not self.point:
                            self.point = [n["x"], n["y"], n["z"]]
                        else:

                            if abs(n["x"]) < abs(self.point[0]):
                                self.point = [n["x"], n["y"], n["z"]]
                        self.point_num += 1
            self.count_num = self.count_num + 1
        else:
            # self.cur_state["point"] = self.point
            self.cur_state["num"] = len(self.point)
            self.cur_state["num"] = self.point_num
            # print("Points meeting conditions:", len(self.point))
            if self.point_num >= 3:
                # Goods detected, output nearest coordinates
                self.cur_state["has_goods"] = True
                self.cur_state["good_location"] = [self.point[0], 0, 0]
                self.cur_state["point"] = self.point
                m.GData.has_goods = True
                loc = [r.loc()['x'], r.loc()['y'], r.loc()['angle']]
                m.GData.good_location = RBK.Pos2World([self.point[0], 0, 0], loc)
                m.task_list.insert(m.task_id, GoToPre(r, 4))
                r.setGData(m.GData.to_dict())
                self.status = MoveStatus.FINISHED
            else:
                m.GData.has_goods = False
                self.status = MoveStatus.FINISHED
                r.setGData(m.GData.to_dict())
                m.task_list.insert(m.task_id, GoToPre(r, 0))

    def is_point_inside_rectangle(self, point, rectangle):
        """Check if a point is inside a rectangle
        Args:
        point: tuple, point coordinates, e.g. (x, y)
        rectangle: list, four vertex coordinates of rectangle, e.g. [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        return:
        bool: whether point is inside rectangle or on edge
        """
        x, y = point
        x1, y1 = rectangle[0]
        x2, y2 = rectangle[1]
        x3, y3 = rectangle[2]
        x4, y4 = rectangle[3]
        # Condition for point inside rectangle: point X coordinate between min and max X of rectangle
        # Point Y coordinate between min and max Y of rectangle
        return min(x1, x2, x3, x4) <= x <= max(x1, x2, x3, x4) and min(y1, y2, y3, y4) <= y <= max(y1, y2, y3, y4)

    def get_latest_point(self):
        result = []
        current_streak = 0
        for i, point in enumerate(self.point):
            if i == 0:
                # Handle first point, cannot compare with previous point
                prev_point = None
            else:
                prev_point = self.point[i - 1]
            # Check if current point and previous point have same Y value
            if point.y != prev_point.y:
                current_streak = 0
            else:
                # Calculate X difference, considering floating point precision
                dx = abs(point.x - prev_point.x)
                if dx < 0.1:  # Adjust threshold based on actual requirements
                    current_streak += 1
                    # If 50 or more consecutive points reached, consider as line and extract sub-array to result
                    if current_streak >= 50:
                        # Ensure sub-array contains first 50 consecutive points meeting conditions
                        line_points = self.point[i - 49:i + 1]
                        result.append(line_points)
        return result


class BezierRetreat:
    """
    Bezier Retreat Class
    Retreat from pallet position to safe zone using gentle Bezier curve
    
    LOG: This class generates retreat path, ensuring robot moves away from walls
    """
    
    def __init__(self, r: SimModule):
        self.status = MoveStatus.NONE
        self.init = False
        self.retreat_distance = PathPlanningConfig.retreat_distance
        self.xs, self.ys = [], []
        self.target_world = [0, 0, 0]
        self.start_time = None
        self.pallet_pos = None
        self.robot_start_pos = None
        self.safe_target_y = 0
        
        PathPlanningConfig.log("BezierRetreat initialized", "INFO")
    
    def calculate_retreat_path(self, pallet_pos, robot_pos):
        """
        Calculate Bezier retreat path
        Args:
            pallet_pos: [x, y, yaw] pallet position
            robot_pos: [x, y, yaw] current robot position
        Returns:
            xs, ys: Bezier curve points
            target_world: target position
        """
        PathPlanningConfig.log(f"Starting retreat path calculation", "INFO")
        PathPlanningConfig.log(f"  Pallet pos: X={pallet_pos[0]:.3f}, Y={pallet_pos[1]:.3f}, "
                              f"Yaw={math.degrees(pallet_pos[2]):.1f}°", "INFO")
        PathPlanningConfig.log(f"  Robot pos: X={robot_pos[0]:.3f}, Y={robot_pos[1]:.3f}, "
                              f"Yaw={math.degrees(robot_pos[2]):.1f}°", "INFO")
        
        self.pallet_pos = pallet_pos
        self.robot_start_pos = robot_pos
        
        # Determine safe target Y position
        self.safe_target_y = self.determine_safe_target(pallet_pos[1], robot_pos[1])
        
        # Calculate retreat target (world coordinates)
        # Retreat direction: away from pallet
        target_local = [self.retreat_distance, self.safe_target_y, 0]
        target_world = RBK.Pos2World(target_local, robot_pos)
        
        PathPlanningConfig.log(f"  Retreat target: X={target_world[0]:.3f}, Y={target_world[1]:.3f}", "INFO")
        PathPlanningConfig.log(f"  Safe Y target: {self.safe_target_y:.3f}m", "INFO")
        
        # Generate Bezier curve
        P0 = robot_pos[:2]
        P3 = target_world[:2]
        self.xs, self.ys = self.generate_retreat_bezier(P0, P3, pallet_pos)
        
        self.target_world = target_world
        
        PathPlanningConfig.log(f"  Generated {len(self.xs)} path points", "INFO")
        
        if PathPlanningConfig.log_path_points:
            PathPlanningConfig.log(f"  Path points: {list(zip(self.xs[::100], self.ys[::100]))}", "DEBUG")
        
        return self.xs, self.ys, target_world
    
    def determine_safe_target(self, pallet_y, robot_y):
        """
        Determine safe Y position
        Args:
            pallet_y: pallet Y coordinate
            robot_y: current robot Y coordinate
        Returns:
            safe_y: safe target Y position
        """
        PathPlanningConfig.log(f"Calculating safe target", "DEBUG")
        PathPlanningConfig.log(f"  Pallet Y: {pallet_y:.3f}m", "DEBUG")
        PathPlanningConfig.log(f"  Robot Y: {robot_y:.3f}m", "DEBUG")
        
        # Strategy:
        # If pallet on LEFT (negative Y), retreat RIGHT (positive/center)
        # If pallet on RIGHT (positive Y), retreat LEFT (negative/center)
        
        retreat_offset = 0.6  # lateral offset during retreat
        
        if pallet_y < -0.2:  # Left side pallet
            # Retreat toward right/center
            safe_y = min(pallet_y + retreat_offset, PathPlanningConfig.safe_y_max)
            PathPlanningConfig.log(f"  Left pallet, retreat right: {safe_y:.3f}m", "DEBUG")
        elif pallet_y > 0.2:  # Right side pallet
            # Retreat toward left/center
            safe_y = max(pallet_y - retreat_offset, PathPlanningConfig.safe_y_min)
            PathPlanningConfig.log(f"  Right pallet, retreat left: {safe_y:.3f}m", "DEBUG")
        else:  # Center pallet
            safe_y = 0  # Stay in center
            PathPlanningConfig.log(f"  Center pallet, stay center: {safe_y:.3f}m", "DEBUG")
        
        # Ensure within safe bounds
        safe_y = max(PathPlanningConfig.safe_y_min, 
                     min(safe_y, PathPlanningConfig.safe_y_max))
        
        # Check if exceeds safe range
        if abs(safe_y) > PathPlanningConfig.safe_y_max:
            PathPlanningConfig.log(f"  WARNING: Target Y {safe_y:.3f}m exceeds safe range "
                                  f"[{PathPlanningConfig.safe_y_min:.3f}, {PathPlanningConfig.safe_y_max:.3f}]", "WARN")
        
        return safe_y
    
    def generate_retreat_bezier(self, P0, P3, pallet_pos):
        """
        Generate gentle Bezier retreat curve
        Args:
            P0: start point [x, y]
            P3: end point [x, y]
            pallet_pos: pallet position (for curve optimization)
        Returns:
            xs, ys: curve points
        """
        PathPlanningConfig.log(f"Generating Bezier curve", "DEBUG")
        PathPlanningConfig.log(f"  P0 (start): [{P0[0]:.3f}, {P0[1]:.3f}]", "DEBUG")
        PathPlanningConfig.log(f"  P3 (end): [{P3[0]:.3f}, {P3[1]:.3f}]", "DEBUG")
        
        dx, dy = P3[0] - P0[0], P3[1] - P0[1]
        length = math.hypot(dx, dy)
        
        if length == 0:
            PathPlanningConfig.log(f"  WARNING: start and end are same", "WARN")
            return [P0[0]], [P0[1]]
        
        PathPlanningConfig.log(f"  Distance: {length:.3f}m", "DEBUG")
        
        # Control points design: gentle curve, avoid large turning angles
        # Use retreat_curve_factor to control bend amount
        factor = PathPlanningConfig.retreat_curve_factor
        
        # P1: Control point near start
        # P2: Control point near end
        P1 = [P0[0] + dx * factor, P0[1] + dy * (factor * 0.5)]
        P2 = [P3[0] - dx * factor, P3[1] - dy * (factor * 0.5)]
        
        PathPlanningConfig.log(f"  P1 (control1): [{P1[0]:.3f}, {P1[1]:.3f}]", "DEBUG")
        PathPlanningConfig.log(f"  P2 (control2): [{P2[0]:.3f}, {P2[1]:.3f}]", "DEBUG")
        PathPlanningConfig.log(f"  Curve factor: {factor}", "DEBUG")
        
        # Generate curve points
        xs = self._bezier_curve(P0[0], P1[0], P2[0], P3[0])
        ys = self._bezier_curve(P0[1], P1[1], P2[1], P3[1])
        
        # Verify curve doesn't cause large turns
        max_angle_change = self._calculate_max_angle_change(xs, ys)
        PathPlanningConfig.log(f"  Max angle change: {max_angle_change:.1f}°", "INFO")
        
        if max_angle_change > 45:
            PathPlanningConfig.log(f"  WARNING: Large angle change detected! "
                                  f"Consider reducing retreat_curve_factor", "WARN")
        
        return xs, ys
    
    def _bezier_curve(self, p0, p1, p2, p3):
        """Cubic Bezier curve formula"""
        return [p0 * (1 - t) ** 3 + 3 * p1 * (1 - t) ** 2 * t + 
                3 * p2 * (1 - t) * t ** 2 + p3 * t ** 3
                for t in (i * 0.001 for i in range(1001))]
    
    def _calculate_max_angle_change(self, xs, ys):
        """
        Calculate maximum angle change in path
        To verify curve doesn't cause sharp turns
        """
        if len(xs) < 3:
            return 0
        
        max_angle = 0
        step = 50  # Check every 50 points
        
        for i in range(step, len(xs) - step, step):
            # Calculate forward and backward vectors
            dx1 = xs[i] - xs[i - step]
            dy1 = ys[i] - ys[i - step]
            dx2 = xs[i + step] - xs[i]
            dy2 = ys[i + step] - ys[i]
            
            # Calculate angle change
            angle1 = math.atan2(dy1, dx1)
            angle2 = math.atan2(dy2, dx2)
            angle_change = abs(math.degrees(angle2 - angle1))
            
            # Normalize to 0-180
            if angle_change > 180:
                angle_change = 360 - angle_change
            
            max_angle = max(max_angle, angle_change)
        
        return max_angle
    
    def run(self, r: SimModule, m: Module):
        """Execute retreat path"""
        if not self.init:
            PathPlanningConfig.log(f"Starting retreat execution", "INFO")
            self.start_time = time.time()
            r.resetPath()
            
            # Set path parameters
            r.setPathReachAngle(0.1)
            r.setPathMaxSpeed(0.1)
            r.setPathBackMode(False)  # Forward mode
            r.setPathOnWorld(self.xs, self.ys, self.target_world[2])
            
            PathPlanningConfig.log(f"  Path set, points: {len(self.xs)}", "INFO")
            self.init = True
        
        # Execute path
        r.goPath()
        
        # Check if finished
        current_pos = [r.loc()['x'], r.loc()['y']]
        distance_to_target = math.hypot(current_pos[0] - self.target_world[0],
                                       current_pos[1] - self.target_world[1])
        
        if distance_to_target < 0.05:  # 5cm tolerance
            r.resetPath()
            self.status = MoveStatus.FINISHED
            elapsed = time.time() - self.start_time
            PathPlanningConfig.log(f"Retreat finished, time: {elapsed:.2f}s", "INFO")
            PathPlanningConfig.log(f"  Final pos: X={current_pos[0]:.3f}, Y={current_pos[1]:.3f}", "INFO")
        else:
            self.status = MoveStatus.RUNNING
    
    def reset(self, r: SimModule):
        self.status = MoveStatus.RUNNING


class StraightApproachWithSideShift:
    """
    Straight Approach with Dynamic Side-shift Class
    After retreat, approach pallet straight backward while dynamically adjusting side-shifter
    
    LOG: This class handles straight approach to pallet, using side-shifter to compensate lateral offset
    """
    
    def __init__(self, r: SimModule):
        self.status = MoveStatus.NONE
        self.init = False
        self.target_world = [0, 0, 0]
        self.sideshifter_target = 0
        self.side_motor = None
        self.start_time = None
        self.approach_distance = 0
        self.pallet_pos = None
        self.robot_start_pos = None
        self.reatch = False
        
        PathPlanningConfig.log("StraightApproachWithSideShift initialized", "INFO")
    
    def calculate_approach_path(self, pallet_pos, robot_pos):
        """
        Calculate straight approach path
        Args:
            pallet_pos: [x, y, yaw] pallet position from recognition
            robot_pos: [x, y, yaw] robot position after retreat
        Returns:
            target_world: target position at pallet
            sideshifter_adjustment: required side-shift
        """
        PathPlanningConfig.log(f"Starting straight approach calculation", "INFO")
        PathPlanningConfig.log(f"  Pallet pos: X={pallet_pos[0]:.3f}, Y={pallet_pos[1]:.3f}, "
                              f"Yaw={math.degrees(pallet_pos[2]):.1f}°", "INFO")
        PathPlanningConfig.log(f"  Robot pos: X={robot_pos[0]:.3f}, Y={robot_pos[1]:.3f}, "
                              f"Yaw={math.degrees(robot_pos[2]):.1f}°", "INFO")
        
        self.pallet_pos = pallet_pos
        self.robot_start_pos = robot_pos
        
        # Calculate pallet relative to robot
        pallet_to_robot = RBK.Pos2Base(pallet_pos, robot_pos)
        
        PathPlanningConfig.log(f"  Pallet relative: X={pallet_to_robot[0]:.3f}, "
                              f"Y={pallet_to_robot[1]:.3f}, Yaw={math.degrees(pallet_to_robot[2]):.1f}°", "INFO")
        
        # Target: pallet position + small offset
        target_local = [pallet_to_robot[0] + CubicBezierPar.backDist, 
                        pallet_to_robot[1], 
                        0]
        target_world = RBK.Pos2World(target_local, robot_pos)
        
        # Calculate required side-shift
        lateral_offset = pallet_to_robot[1]
        sideshifter_adjustment = self.calculate_sideshifter_adjustment(lateral_offset)
        
        self.target_world = target_world
        self.sideshifter_target = sideshifter_adjustment
        self.approach_distance = abs(pallet_to_robot[0])
        
        PathPlanningConfig.log(f"  Target pos: X={target_world[0]:.3f}, Y={target_world[1]:.3f}", "INFO")
        PathPlanningConfig.log(f"  Approach distance: {self.approach_distance:.3f}m", "INFO")
        PathPlanningConfig.log(f"  Lateral offset: {lateral_offset:.3f}m", "INFO")
        PathPlanningConfig.log(f"  Side-shift adjustment: {sideshifter_adjustment:.3f}m", "INFO")
        
        return target_world, sideshifter_adjustment
    
    def calculate_sideshifter_adjustment(self, lateral_offset):
        """
        Calculate side-shifter adjustment
        Args:
            lateral_offset: lateral distance to pallet (m)
        Returns:
            sideshifter_value: adjustment value (m)
        """
        # Map lateral offset to side-shift sign based on hardware convention
        # We interpret lateral_offset > 0 as "pallet is to robot's +Y side" in robot base frame.
        # Convert to actuator command sign so that side-shifter moves forks toward pallet.
        cmd_offset = lateral_offset if PathPlanningConfig.sideshifter_positive_is_left else -lateral_offset

        # Clamp to side-shifter limits
        max_shift = PathPlanningConfig.sideshifter_max
        
        if abs(cmd_offset) <= max_shift:
            adjustment = cmd_offset
            PathPlanningConfig.log(f"  Side-shifter can fully compensate: {adjustment:.3f}m", "DEBUG")
        elif cmd_offset > 0:
            adjustment = max_shift
            PathPlanningConfig.log(f"  Side-shifter at max: {adjustment:.3f}m "
                                  f"(needed: {cmd_offset:.3f}m)", "WARN")
        else:
            adjustment = -max_shift
            PathPlanningConfig.log(f"  Side-shifter at min: {adjustment:.3f}m "
                                  f"(needed: {cmd_offset:.3f}m)", "WARN")
        
        return adjustment
    
    def run(self, r: SimModule, m: Module):
        """Execute straight approach with parallel sideshifter adjustment"""
        if not self.init:
            PathPlanningConfig.log(f"Starting straight approach execution", "INFO")
            PathPlanningConfig.log(f"  Using recognition data for pallet position", "INFO")
            self.start_time = time.time()
            r.resetPath()
            
            # Initialize side-shifter motor (will adjust parallel during movement)
            self.side_motor = ForkMotor(r, MotorName.side, self.sideshifter_target, 0.5)
            PathPlanningConfig.log(f"  Sideshifter initialized for parallel adjustment: {self.sideshifter_target:.3f}m", "INFO")
            
            # Set straight path driving direction based on fork orientation:
            # - forks_at_front=True  => approach should drive FORWARD (backMode=False)
            # - forks_at_front=False => approach should drive BACKWARD (backMode=True)
            r.setPathBackMode(not PathPlanningConfig.forks_at_front)
            r.setPathMaxSpeed(0.1)
            r.setPathReachDist(0.02)
            r.setPathReachAngle(0.1)
            
            # Set target point - use simple straight path (to pallet from recognition)
            current_pos = [r.loc()['x'], r.loc()['y']]
            xs = [current_pos[0], self.target_world[0]]
            ys = [current_pos[1], self.target_world[1]]
            r.setPathOnWorld(xs, ys, self.target_world[2])
            
            distance = math.hypot(xs[1] - xs[0], ys[1] - ys[0])
            PathPlanningConfig.log(f"  Path set: from ({current_pos[0]:.3f}, {current_pos[1]:.3f}) "
                                  f"to ({self.target_world[0]:.3f}, {self.target_world[1]:.3f})", "INFO")
            PathPlanningConfig.log(f"  Approach distance: {distance:.3f}m", "INFO")
            PathPlanningConfig.log(f"  Sideshifter will adjust PARALLEL during backward movement", "INFO")
            
            self.init = True
        
        # Execute side-shifter adjustment PARALLEL with path movement
        # This ensures sideshifter adjusts while robot moves, not before
        if PathPlanningConfig.dynamic_sideshifter_enabled and self.side_motor:
            if self.side_motor.status == MoveStatus.FAILED:
                self.status = MoveStatus.FAILED
                PathPlanningConfig.log(f"Side-shifter failed!", "ERROR")
                r.setError("Sideshifter adjustment failed during backward approach")
                return
            # Run sideshifter in parallel (doesn't block path execution)
            self.side_motor.run(r, m, self.sideshifter_target)
            
            # Log sideshifter progress
            if self.side_motor.status == MoveStatus.RUNNING:
                current_side_pos = r.getMotorPosition(MotorName.side) if hasattr(r, 'getMotorPosition') else "unknown"
                PathPlanningConfig.log(f"  Sideshifter adjusting: target={self.sideshifter_target:.3f}m, current={current_side_pos}", "DEBUG")
        
        # Execute straight path towards pallet
        if self.status != MoveStatus.FAILED:
            r.goPath()
            
            # Log path progress
            current_pos = [r.loc()['x'], r.loc()['y']]
            distance_remaining = math.hypot(current_pos[0] - self.target_world[0],
                                          current_pos[1] - self.target_world[1])
            if distance_remaining < 0.1:  # Log when close
                PathPlanningConfig.log(f"  Approaching pallet: {distance_remaining:.3f}m remaining", "DEBUG")
        
        # Check for fork insertion signal (DI 16 or 17)
        di_status = r.Di().get('node', [])
        fork_signal_16 = any(node['status'] for node in di_status if node['id'] == 16)
        fork_signal_17 = any(node['status'] for node in di_status if node['id'] == 17)
        
        if fork_signal_16 or fork_signal_17:
            self.reatch = True
            PathPlanningConfig.log(f"Fork insertion detected: DI16={fork_signal_16}, DI17={fork_signal_17}", "INFO")
            PathPlanningConfig.log(f"  Pallet pickup successful!", "INFO")
        
        # Completion condition
        if self.reatch:
            r.resetPath()
            self.status = MoveStatus.FINISHED
            elapsed = time.time() - self.start_time
            current_pos = [r.loc()['x'], r.loc()['y']]
            current_side = r.getMotorPosition(MotorName.side) if hasattr(r, 'getMotorPosition') else "unknown"
            
            PathPlanningConfig.log(f"Backward approach finished, time: {elapsed:.2f}s", "INFO")
            PathPlanningConfig.log(f"  Final robot pos: X={current_pos[0]:.3f}, Y={current_pos[1]:.3f}", "INFO")
            PathPlanningConfig.log(f"  Final sideshifter pos: {current_side}", "INFO")
            PathPlanningConfig.log(f"  Target was: X={self.target_world[0]:.3f}, Y={self.target_world[1]:.3f}", "INFO")
            
            # Switch to next pallet
            m.GData.currentPoint_switch_nextPoint()
            r.setGData(m.GData.to_dict())
            PathPlanningConfig.log(f"  Switched to next pallet in recognition data", "INFO")
        else:
            self.status = MoveStatus.RUNNING
    
    def reset(self, r: SimModule):
        self.status = MoveStatus.RUNNING


class ForwardBezierRetreat:
    """
    Forward Bezier Retreat - Move robot FORWARD (away from pallet) using Bezier curve
    Fork side is the BACK of robot, so forward = away from pallet
    """
    
    def __init__(self, r: SimModule):
        self.status = MoveStatus.NONE
        self.init = False
        self.retreat_distance = PathPlanningConfig.retreat_distance
        self.xs, self.ys = [], []
        self.target_world = [0, 0, 0]
        self.start_time = None
        self.pallet_pos = None
        self.robot_start_pos = None
        
        PathPlanningConfig.log("ForwardBezierRetreat initialized", "INFO")
    
    def calculate_forward_retreat_path(self, pallet_pos, robot_pos):
        """
        Calculate forward retreat path (moving away from pallet)
        Args:
            pallet_pos: [x, y, yaw] pallet position
            robot_pos: [x, y, yaw] current robot position
        Returns:
            xs, ys: Bezier curve points
            target_world: target position
        """
        PathPlanningConfig.log(f"Calculating forward retreat path (away from pallet)", "INFO")
        
        self.pallet_pos = pallet_pos
        self.robot_start_pos = robot_pos
        
        # Calculate pallet relative to robot
        pallet_to_robot = RBK.Pos2Base(pallet_pos, robot_pos)
        
        # Retreat direction depends on fork orientation:
        # - forks_at_front=True: pallet is typically in FRONT, so retreat = drive backward (negative X)
        # - forks_at_front=False: pallet is typically behind, so retreat = drive forward (positive X)
        retreat_x = -self.retreat_distance if PathPlanningConfig.forks_at_front else self.retreat_distance
        retreat_local = [retreat_x, 0, 0]
        target_world = RBK.Pos2World(retreat_local, robot_pos)
        
        # Ensure target Y stays within safe bounds
        target_y = max(PathPlanningConfig.safe_y_min,
                      min(target_world[1], PathPlanningConfig.safe_y_max))
        target_world[1] = target_y
        
        # Generate Bezier curve from current position to retreat target
        P0 = robot_pos[:2]
        P3 = target_world[:2]
        
        self.xs, self.ys = self._generate_bezier_curve(P0, P3)
        self.target_world = [target_world[0], target_y, robot_pos[2]]
        
        PathPlanningConfig.log(f"  Forward retreat target: X={target_world[0]:.3f}, Y={target_y:.3f}", "INFO")
        PathPlanningConfig.log(f"  Generated {len(self.xs)} path points", "INFO")
        
        return self.xs, self.ys, self.target_world
    
    def _generate_bezier_curve(self, P0, P3):
        """Generate gentle Bezier curve with rotation limits to prevent wall collisions"""
        dx, dy = P3[0] - P0[0], P3[1] - P0[1]
        length = math.hypot(dx, dy)
        
        if length == 0:
            return [P0[0]], [P0[1]]
        
        factor = PathPlanningConfig.retreat_curve_factor
        
        # Control points for gentle curve
        P1 = [P0[0] + dx * factor, P0[1] + dy * (factor * 0.5)]
        P2 = [P3[0] - dx * factor, P3[1] - dy * (factor * 0.5)]
        
        xs = self._bezier_curve(P0[0], P1[0], P2[0], P3[0])
        ys = self._bezier_curve(P0[1], P1[1], P2[1], P3[1])
        
        # Verify rotation doesn't exceed limits
        max_angle = self._calculate_max_rotation(xs, ys)
        PathPlanningConfig.log(f"  Max rotation in curve: {max_angle:.1f}° (limit: {PathPlanningConfig.max_rotation_angle}°)", "INFO")
        
        if max_angle > PathPlanningConfig.max_rotation_angle:
            PathPlanningConfig.log(f"  WARNING: Curve rotation {max_angle:.1f}° exceeds limit! Adjusting...", "WARN")
            # Reduce curve factor to decrease rotation
            factor = factor * 0.7
            P1 = [P0[0] + dx * factor, P0[1] + dy * (factor * 0.5)]
            P2 = [P3[0] - dx * factor, P3[1] - dy * (factor * 0.5)]
            xs = self._bezier_curve(P0[0], P1[0], P2[0], P3[0])
            ys = self._bezier_curve(P0[1], P1[1], P2[1], P3[1])
            PathPlanningConfig.log(f"  Adjusted curve factor to {factor:.3f}", "INFO")
        
        return xs, ys
    
    def _calculate_max_rotation(self, xs, ys):
        """Calculate maximum rotation angle in path"""
        if len(xs) < 3:
            return 0
        
        max_angle = 0
        step = 50  # Check every 50 points
        
        for i in range(step, len(xs) - step, step):
            # Calculate direction vectors
            dx1 = xs[i] - xs[i - step]
            dy1 = ys[i] - ys[i - step]
            dx2 = xs[i + step] - xs[i]
            dy2 = ys[i + step] - ys[i]
            
            # Calculate angle change
            angle1 = math.atan2(dy1, dx1)
            angle2 = math.atan2(dy2, dx2)
            angle_change = abs(math.degrees(angle2 - angle1))
            
            # Normalize to 0-180
            if angle_change > 180:
                angle_change = 360 - angle_change
            
            max_angle = max(max_angle, angle_change)
        
        return max_angle
    
    def _bezier_curve(self, p0, p1, p2, p3):
        """Cubic Bezier curve formula"""
        return [p0 * (1 - t) ** 3 + 3 * p1 * (1 - t) ** 2 * t + 
                3 * p2 * (1 - t) * t ** 2 + p3 * t ** 3
                for t in (i * 0.001 for i in range(1001))]
    
    def run(self, r: SimModule, m: Module):
        """Execute forward retreat path"""
        if not self.init:
            PathPlanningConfig.log(f"Starting forward retreat execution", "INFO")
            self.start_time = time.time()
            r.resetPath()
            
            # Set path parameters
            r.setPathReachAngle(0.1)
            r.setPathMaxSpeed(0.1)
            # If forks_at_front=True, retreat should drive backward to move away from pallet
            r.setPathBackMode(PathPlanningConfig.forks_at_front)
            r.setPathOnWorld(self.xs, self.ys, self.target_world[2])
            
            PathPlanningConfig.log(f"  Forward path set, points: {len(self.xs)}", "INFO")
            self.init = True
        
        # Execute path
        r.goPath()
        
        # Check if finished
        current_pos = [r.loc()['x'], r.loc()['y']]
        distance_to_target = math.hypot(current_pos[0] - self.target_world[0],
                                       current_pos[1] - self.target_world[1])
        
        if distance_to_target < 0.05:  # 5cm tolerance
            r.resetPath()
            self.status = MoveStatus.FINISHED
            elapsed = time.time() - self.start_time
            PathPlanningConfig.log(f"Forward retreat finished, time: {elapsed:.2f}s", "INFO")
        else:
            self.status = MoveStatus.RUNNING
    
    def reset(self, r: SimModule):
        self.status = MoveStatus.RUNNING


class PrecisePalletPickup:
    """
    Precise Pallet Pickup with Multi-Phase Approach
    Flow: Forward Bezier Retreat → Lateral Alignment → Sideshifter Adjust → Backward Straight Approach
    
    USER REQUIREMENT:
    1. After recognition, robot moves FORWARD (away from pallet) using Bezier curve
    2. Robot aligns itself laterally (right or left)
    3. Robot adjusts sideshifter according to need
    4. Robot moves BACKWARD (towards pallet) to pick precisely
    5. Fork insertion detected → pickup complete
    """
    
    def __init__(self, r: SimModule):
        self.status = MoveStatus.NONE
        self.init = False
        self.start_time = None
        
        # Phase tracking
        self.current_phase = "none"  # none, forward_retreat, lateral_align, sideshifter_adjust, backward_approach
        
        # Phase objects
        self.forward_retreat = None
        self.lateral_align = None
        self.sideshifter_motor = None
        self.backward_approach = None
        
        # Data
        self.pallet_pos = None
        self.robot_start_pos = None
        self.lateral_target_y = 0
        self.sideshifter_target = 0
        self.target_world = [0, 0, 0]
        self.pallet_number = 0
        self.is_ramp_area = False
        
        PathPlanningConfig.log("PrecisePalletPickup initialized", "INFO")
    
    def calculate_pickup_path(self, pallet_pos, robot_pos, pallet_number=0, is_ramp_area=False):
        """
        Calculate complete pickup path using recognition data
        Args:
            pallet_pos: [x, y, yaw] pallet position from recognition output
            robot_pos: [x, y, yaw] current robot position
            pallet_number: Current pallet number (for ramp logic)
            is_ramp_area: Whether in ramp area (first 4 pallets)
        """
        PathPlanningConfig.log(f"=" * 60, "INFO")
        PathPlanningConfig.log(f"Calculating precise pickup path (using recognition data)", "INFO")
        PathPlanningConfig.log(f"  Pallet number: {pallet_number}", "INFO")
        PathPlanningConfig.log(f"  Is ramp area: {is_ramp_area}", "INFO")
        PathPlanningConfig.log(f"  Pallet pos (from recognition): X={pallet_pos[0]:.3f}, Y={pallet_pos[1]:.3f}, "
                              f"Yaw={math.degrees(pallet_pos[2]):.1f}°", "INFO")
        PathPlanningConfig.log(f"  Robot pos: X={robot_pos[0]:.3f}, Y={robot_pos[1]:.3f}, "
                              f"Yaw={math.degrees(robot_pos[2]):.1f}°", "INFO")
        
        self.pallet_pos = pallet_pos
        self.robot_start_pos = robot_pos
        
        # Calculate pallet relative to robot
        pallet_to_robot = RBK.Pos2Base(pallet_pos, robot_pos)
        PathPlanningConfig.log(f"  Pallet relative to robot: X={pallet_to_robot[0]:.3f}, Y={pallet_to_robot[1]:.3f}", "INFO")
        
        # Phase 1: Forward retreat target (move away from pallet)
        # Retreat forward (positive X in robot frame) using Bezier curve
        retreat_distance = PathPlanningConfig.retreat_distance
        retreat_local = [retreat_distance, 0, 0]  # Forward in robot frame
        retreat_world = RBK.Pos2World(retreat_local, robot_pos)
        
        # Phase 2: Lateral alignment target
        # For ramp area (first 4 pallets): align to ramp width, don't fall off
        # For container area: use container safe bounds
        pallet_y = pallet_pos[1]
        robot_y = robot_pos[1]
        
        if is_ramp_area:
            # Ramp-specific logic: align to ramp width, ensure robot doesn't fall off
            PathPlanningConfig.log(f"  RAMP AREA: Using ramp-specific alignment", "INFO")
            ramp_safe_y_max = PathPlanningConfig.ramp_safe_y_max
            ramp_safe_y_min = PathPlanningConfig.ramp_safe_y_min
            
            # Calculate lateral movement needed
            lateral_offset = pallet_y - robot_y
            
            # Clamp to ramp safe Y range (narrower than container)
            target_y = robot_y + lateral_offset
            target_y = max(ramp_safe_y_min, min(target_y, ramp_safe_y_max))
            
            PathPlanningConfig.log(f"  Ramp safe Y range: [{ramp_safe_y_min:.3f}, {ramp_safe_y_max:.3f}]m", "INFO")
            PathPlanningConfig.log(f"  Ramp alignment target Y: {target_y:.3f}m", "INFO")
        else:
            # Container area: use container safe bounds
            PathPlanningConfig.log(f"  CONTAINER AREA: Using container safe bounds", "INFO")
            lateral_offset = pallet_y - robot_y
            
            # Clamp to safe Y range (with 250mm wall clearance)
            target_y = robot_y + lateral_offset
            target_y = max(PathPlanningConfig.safe_y_min, 
                          min(target_y, PathPlanningConfig.safe_y_max))
            
            PathPlanningConfig.log(f"  Container safe Y range: [{PathPlanningConfig.safe_y_min:.3f}, {PathPlanningConfig.safe_y_max:.3f}]m", "INFO")
            PathPlanningConfig.log(f"  Container alignment target Y: {target_y:.3f}m", "INFO")
        
        self.lateral_target_y = target_y
        
        # Phase 3: Sideshifter adjustment
        # Calculate how much sideshifter needs to compensate
        final_lateral_offset = pallet_y - target_y
        self.sideshifter_target = self._calculate_sideshifter_adjustment(final_lateral_offset)
        
        PathPlanningConfig.log(f"  Final lateral offset: {final_lateral_offset:.3f}m", "INFO")
        PathPlanningConfig.log(f"  Sideshifter adjustment: {self.sideshifter_target:.3f}m (max ±{PathPlanningConfig.sideshifter_max:.3f}m)", "INFO")
        
        # Phase 4: Backward approach target
        # Target is pallet position with small offset (from recognition data)
        target_local = [pallet_to_robot[0] + CubicBezierPar.backDist, 
                        final_lateral_offset, 
                        0]
        self.target_world = RBK.Pos2World(target_local, [retreat_world[0], target_y, robot_pos[2]])
        
        PathPlanningConfig.log(f"  Forward retreat target: X={retreat_world[0]:.3f}, Y={retreat_world[1]:.3f}", "INFO")
        PathPlanningConfig.log(f"  Backward approach target: X={self.target_world[0]:.3f}, Y={self.target_world[1]:.3f}", "INFO")
        PathPlanningConfig.log(f"=" * 60, "INFO")
        
        return retreat_world, target_y, self.sideshifter_target, self.target_world
    
    def _calculate_sideshifter_adjustment(self, lateral_offset):
        """Calculate sideshifter adjustment, clamped to ±210mm"""
        max_shift = PathPlanningConfig.sideshifter_max
        
        if abs(lateral_offset) <= max_shift:
            return lateral_offset
        elif lateral_offset > 0:
            return max_shift
        else:
            return -max_shift
    
    def run(self, r: SimModule, m: Module):
        """Execute multi-phase pickup"""
        if not self.init:
            PathPlanningConfig.log(f"Initializing precise pickup", "INFO")
            self.start_time = time.time()
            self._initialize_phases(r, m)
            self.init = True
        
        # Phase 1: Forward Bezier Retreat
        if self.current_phase == "forward_retreat":
            if self.forward_retreat.status == MoveStatus.NONE:
                PathPlanningConfig.log(f"Phase 1: Starting forward Bezier retreat", "INFO")
            
            self.forward_retreat.run(r, m)
            
            if self.forward_retreat.status == MoveStatus.FINISHED:
                PathPlanningConfig.log(f"Phase 1 complete: Forward retreat finished", "INFO")
                self.current_phase = "lateral_align"
                self._initialize_lateral_align(r, m)
            elif self.forward_retreat.status == MoveStatus.FAILED:
                self.status = MoveStatus.FAILED
                PathPlanningConfig.log(f"Phase 1 failed: Forward retreat failed!", "ERROR")
            else:
                self.status = MoveStatus.RUNNING
            return
        
        # Phase 2: Lateral Alignment
        if self.current_phase == "lateral_align":
            if self.lateral_align.status == MoveStatus.NONE:
                PathPlanningConfig.log(f"Phase 2: Starting lateral alignment", "INFO")
                PathPlanningConfig.log(f"  Current robot Y: {r.loc()['y']:.3f}m", "INFO")
                PathPlanningConfig.log(f"  Target Y: {self.lateral_target_y:.3f}m", "INFO")
                PathPlanningConfig.log(f"  Y movement needed: {self.lateral_target_y - r.loc()['y']:.3f}m", "INFO")
            
            self.lateral_align.run(r, self.lateral_align.go_args)
            
            if self.lateral_align.status == MoveStatus.FINISHED:
                current_y = r.loc()['y']
                PathPlanningConfig.log(f"Phase 2 complete: Lateral alignment finished", "INFO")
                PathPlanningConfig.log(f"  Final robot Y: {current_y:.3f}m (target was {self.lateral_target_y:.3f}m)", "INFO")
                PathPlanningConfig.log(f"  Y error: {abs(current_y - self.lateral_target_y):.3f}m", "INFO")
                self.current_phase = "sideshifter_adjust"
                self._initialize_sideshifter(r, m)
            elif self.lateral_align.status == MoveStatus.FAILED:
                self.status = MoveStatus.FAILED
                PathPlanningConfig.log(f"Phase 2 failed: Lateral alignment failed!", "ERROR")
                r.setError("Lateral alignment phase failed")
            else:
                self.status = MoveStatus.RUNNING
            return
        
        # Phase 3: Sideshifter Adjustment (parallel with backward approach)
        # NOTE: Sideshifter adjusts PARALLEL during backward movement, not before
        # This phase is skipped - sideshifter adjustment happens in Phase 4
        if self.current_phase == "sideshifter_adjust":
            # Skip this phase - sideshifter will adjust parallel in backward approach
            PathPlanningConfig.log(f"Phase 3: Sideshifter will adjust parallel during backward approach", "INFO")
            self.current_phase = "backward_approach"
            self._initialize_backward_approach(r, m)
            return
        
        # Phase 4: Backward Straight Approach (with parallel sideshifter adjustment)
        if self.current_phase == "backward_approach":
            if self.backward_approach.status == MoveStatus.NONE:
                PathPlanningConfig.log(f"Phase 4: Starting backward straight approach", "INFO")
                PathPlanningConfig.log(f"  Sideshifter will adjust PARALLEL during movement", "INFO")
                PathPlanningConfig.log(f"  Sideshifter target: {self.sideshifter_target:.3f}m", "INFO")
            
            # Sideshifter adjusts parallel during backward movement (handled in StraightApproachWithSideShift)
            self.backward_approach.run(r, m)
            
            if self.backward_approach.status == MoveStatus.FINISHED:
                self.status = MoveStatus.FINISHED
                elapsed = time.time() - self.start_time
                current_pos = [r.loc()['x'], r.loc()['y']]
                PathPlanningConfig.log(f"=" * 60, "INFO")
                PathPlanningConfig.log(f"Phase 4 complete: Backward approach finished", "INFO")
                PathPlanningConfig.log(f"Precise pickup complete! Total time: {elapsed:.2f}s", "INFO")
                PathPlanningConfig.log(f"  Final robot position: X={current_pos[0]:.3f}, Y={current_pos[1]:.3f}", "INFO")
                PathPlanningConfig.log(f"  Target was: X={self.target_world[0]:.3f}, Y={self.target_world[1]:.3f}", "INFO")
                PathPlanningConfig.log(f"=" * 60, "INFO")
                
                # Switch to next pallet
                m.GData.currentPoint_switch_nextPoint()
                r.setGData(m.GData.to_dict())
                PathPlanningConfig.log(f"Switched to next pallet in recognition data", "INFO")
            elif self.backward_approach.status == MoveStatus.FAILED:
                self.status = MoveStatus.FAILED
                PathPlanningConfig.log(f"Phase 4 failed: Backward approach failed!", "ERROR")
                r.setError("Backward approach phase failed")
            else:
                self.status = MoveStatus.RUNNING
            return
    
    def _initialize_phases(self, r: SimModule, m: Module):
        """Initialize all phases using recognition output data"""
        # LOG: Get recognition data
        PathPlanningConfig.log(f"Initializing phases with recognition data", "INFO")
        
        # Get pallet position from recognition output (currentPoint)
        if not m.GData.currentPoint:
            PathPlanningConfig.log(f"ERROR: No currentPoint in recognition data!", "ERROR")
            self.status = MoveStatus.FAILED
            r.setError("No pallet recognition data available for precise pickup")
            return
        
        # Extract pallet position from recognition output
        current_point = m.GData.currentPoint
        pallet_pos = [current_point['x'], current_point['y'], current_point['yaw']]
        robot_pos = [r.loc()['x'], r.loc()['y'], r.loc()['angle']]
        
        # LOG: Recognition data details
        PathPlanningConfig.log(f"Recognition data used:", "INFO")
        PathPlanningConfig.log(f"  Pallet X: {pallet_pos[0]:.3f}m", "INFO")
        PathPlanningConfig.log(f"  Pallet Y: {pallet_pos[1]:.3f}m", "INFO")
        PathPlanningConfig.log(f"  Pallet Yaw: {math.degrees(pallet_pos[2]):.1f}°", "INFO")
        PathPlanningConfig.log(f"  Robot X: {robot_pos[0]:.3f}m", "INFO")
        PathPlanningConfig.log(f"  Robot Y: {robot_pos[1]:.3f}m", "INFO")
        PathPlanningConfig.log(f"  Robot Yaw: {math.degrees(robot_pos[2]):.1f}°", "INFO")
        
        # Get pallet number to determine if in ramp area
        g_data_dict = r.getGData()
        self.pallet_number = g_data_dict.get('pallet_count', 0)
        self.is_ramp_area = (self.pallet_number <= PathPlanningConfig.first_n_pallets_use_old_logic)
        
        PathPlanningConfig.log(f"  Pallet number: {self.pallet_number}", "INFO")
        PathPlanningConfig.log(f"  Is ramp area (first {PathPlanningConfig.first_n_pallets_use_old_logic} pallets): {self.is_ramp_area}", "INFO")
        
        # Calculate path using recognition data (with ramp/container logic)
        retreat_world, lateral_y, sideshifter, approach_target = self.calculate_pickup_path(
            pallet_pos, robot_pos, self.pallet_number, self.is_ramp_area
        )
        
        # Initialize Phase 1: Forward Bezier Retreat
        # Create a forward retreat Bezier path (moving away from pallet)
        self.forward_retreat = ForwardBezierRetreat(r)
        xs, ys, target = self.forward_retreat.calculate_forward_retreat_path(pallet_pos, robot_pos)
        
        # Store retreat target for later use
        self.retreat_target = retreat_world
        
        # Start with forward retreat phase
        self.current_phase = "forward_retreat"
        
        PathPlanningConfig.log(f"All phases initialized using recognition data", "INFO")
    
    def _initialize_lateral_align(self, r: SimModule, m: Module):
        """Initialize lateral alignment phase"""
        r.resetPath()
        self.lateral_align = goPath.Module(r, dict())
        
        current_pos = [r.loc()['x'], r.loc()['y'], r.loc()['angle']]
        target_world = [current_pos[0], self.lateral_target_y, current_pos[2]]
        
        # Calculate relative position for lateral movement
        target_base = RBK.Pos2Base(target_world, current_pos)
        
        self.lateral_align.go_args = {
            'x': target_base[0],
            'y': target_base[1],
            'theta': 0.001,
            'coordinate': 'robot',
            'backMode': 0  # Forward mode
        }
    
    def _initialize_sideshifter(self, r: SimModule, m: Module):
        """Initialize sideshifter adjustment (will be used parallel during backward approach)"""
        PathPlanningConfig.log(f"Preparing sideshifter for parallel adjustment: {self.sideshifter_target:.3f}m", "INFO")
        self.sideshifter_motor = ForkMotor(r, MotorName.side, self.sideshifter_target, 0.5)
    
    def _initialize_backward_approach(self, r: SimModule, m: Module):
        """Initialize backward approach phase using recognition data"""
        PathPlanningConfig.log(f"Initializing backward approach with recognition data", "INFO")
        
        self.backward_approach = StraightApproachWithSideShift(r)
        robot_pos = [r.loc()['x'], r.loc()['y'], r.loc()['angle']]
        
        # Use recognition data (currentPoint) for pallet position
        if not m.GData.currentPoint:
            PathPlanningConfig.log(f"ERROR: No currentPoint for backward approach!", "ERROR")
            self.status = MoveStatus.FAILED
            return
        
        pallet_pos = [m.GData.currentPoint['x'], m.GData.currentPoint['y'], m.GData.currentPoint['yaw']]
        PathPlanningConfig.log(f"  Using recognition pallet pos: X={pallet_pos[0]:.3f}, Y={pallet_pos[1]:.3f}", "INFO")
        
        target, sideshifter = self.backward_approach.calculate_approach_path(pallet_pos, robot_pos)
        
        PathPlanningConfig.log(f"Backward approach initialized", "INFO")
    
    def reset(self, r: SimModule):
        self.status = MoveStatus.RUNNING


class CubicBezier2Load:
    """Third-order Bezier curve fork insertion for pallet pickup"""

    def __init__(self, r: SimModule):
        self.status = MoveStatus.NONE
        self.init = False
        self.need_side = True
        self.reatch = False
        self.side = 0
        self.side_motor = ForkMotor(r, MotorName.side, 0.0, 0.5)
        self.start_time = None
        self.C_msg = None
        self.target_world = [0, 0, 0]
        self.control_point = None
        self.xs, self.ys = [], []  # Store Bezier curve point sets
        
        # New path planning related
        self.pallet_number = 0  # Pallet number
        self.use_new_logic = False  # Whether to use new logic
        self.retreat_phase = None  # Retreat phase
        self.approach_phase = None  # Approach phase
        self.precise_pickup = None  # Precise pickup (new multi-phase system)
        self.current_phase = "none"  # Current phase: none, retreat, approach, old_logic, precise_pickup
        
        PathPlanningConfig.log("CubicBezier2Load initialized", "INFO")

    def run(self, r: SimModule, m: Module):
        self._validate_input(r, m)
        
        # Initialize: determine which logic to use
        if not self.init:
            self._determine_logic(r, m)
        
        # Execute based on logic
        if self.use_new_logic:
            # Use new precise pickup system (multi-phase)
            self._execute_precise_pickup(r, m)
        else:
            self._execute_old_logic(r, m)
        
        self._report_status(m)

    def _validate_input(self, r: SimModule, m: Module):
        if not m.GData.currentPoint:
            self.status = MoveStatus.FAILED
            r.setError(f'No currentPoint in recognition data: {m.GData.to_dict()}')

    def _initialize(self, r: SimModule, m: Module):
        if self.init: return
        self.start_time = time.time()
        r.resetPath()

        current_point = m.GData.currentPoint
        pallet_pos = [current_point['x'], current_point['y'], current_point['yaw']]
        loc_robot = [r.loc()['x'], r.loc()['y'], 0]
        # loc_robot = [r.loc()['x'], r.loc()['y'], r.loc()['yaw']]

        # Calculate lateral movement distance
        pallet_to_lm = RBK.Pos2Base(pallet_pos, loc_robot)
        self.side = self._calculate_side_offset(pallet_to_lm[1])

        # Calculate target position
        y_offset = self._calculate_y_offset(pallet_to_lm[1])
        target_local = [pallet_to_lm[0] + CubicBezierPar.backDist, y_offset, 0]
        self.target_world = RBK.Pos2World(target_local, loc_robot)

        # Generate Bezier curve
        curve_params = self._get_curve_params(pallet_to_lm)
        P0, P3 = loc_robot[:2], self.target_world[:2]
        self.xs, self.ys, self.control_point = self._generate_bezier_curve(P0, P3, curve_params)

        # Set path parameters - Fix: add missing path settings
        r.setPathReachAngle(0.1)
        r.setPathMaxSpeed(0.1)
        r.setPathBackMode(True)
        r.setPathOnWorld(self.xs, self.ys, self.target_world[2])  # Fix: set Bezier curve path

        self.C_msg = {
            # "Pallet coordinates": pallet_pos, "Pallet to robot": pallet_to_lm,
            "target_world": self.target_world, "control_point": self.control_point,
            # **curve_params
        }
        self.init = True

    def _calculate_side_offset(self, lateral_distance):
        if lateral_distance > 0.2: return 0.195
        if lateral_distance < -0.2: return -0.195
        return lateral_distance

    def _calculate_y_offset(self, lateral_distance):
        if abs(lateral_distance) <= 0.2: return 0
        y = lateral_distance - (0.31 if lateral_distance > 0 else -0.31)
        max_dist = CubicBezierPar.odo_2_pallet_dist
        return max(-max_dist, min(y, max_dist))

    def _get_curve_params(self, pallet_to_robot):
        if abs(pallet_to_robot[1]) < 0.23:
            return {"t": 0.5, "d": 0.0, "c_extend_factor": 0}
        return {
            "t": CubicBezierPar.t,
            "d": CubicBezierPar.d_r if pallet_to_robot[1] > 0 else CubicBezierPar.d_l,
            "c_extend_factor": CubicBezierPar.extend_factor
        }

    def _generate_bezier_curve(self, P0, P3, params):
        p1, p2 = self._compute_control_points(P0, P3, params)
        xs = self._bezier_curve(P0[0], p1[0], p2[0], P3[0])
        ys = self._bezier_curve(P0[1], p1[1], p2[1], P3[1])
        return xs, ys, [P0, p1, p2, P3]

    def _compute_control_points(self, P0, P3, params):
        dx, dy = P3[0] - P0[0], P3[1] - P0[1]
        length = math.hypot(dx, dy)
        if length == 0: return P0, P3
        u_perp = [-dy / length, dx / length]
        extend = params["c_extend_factor"]
        P1_base = [P0[0] + dx * (0.5 - params["t"]), P0[1] + dy * (0.5 - params["t"])]
        P2_base = [P3[0] - dx * (0.5 - params["t"]), P3[1] - dy * (0.5 - params["t"])]
        P1 = [P1_base[0] + params["d"] * u_perp[0] * extend, P1_base[1] + params["d"] * u_perp[1] * extend]
        P2 = [P2_base[0] - params["d"] * u_perp[0] * extend, P2_base[1] - params["d"] * u_perp[1] * extend]
        return P1, P2

    def _bezier_curve(self, p0, p1, p2, p3):
        return [p0 * (1 - t) ** 3 + 3 * p1 * (1 - t) ** 2 * t + 3 * p2 * (1 - t) * t ** 2 + p3 * t ** 3
                for t in (i * 0.001 for i in range(1001))]

    def _execute_movement(self, r: SimModule, m: Module):
        if not self.init: return
        # Execute lateral movement
        if self.need_side:
            if self.side_motor.status == MoveStatus.FAILED:
                self.status = MoveStatus.FAILED
                return
            self.side_motor.run(r, m, self.side)
        # Execute path tracking
        if self.status != MoveStatus.FAILED:
            r.goPath()
        # Detect arrival signal
        if any(node['status'] for node in r.Di().get('node', [])
               if node['id'] in (16, 17)):
            self.reatch = True
        if self.reatch:
            r.resetPath()
            self.status = MoveStatus.FINISHED
            m.GData.currentPoint_switch_nextPoint()
            r.setGData(m.GData.to_dict())

    def _report_status(self, m: Module):
        state = {
            "msg": self.C_msg,
            "status": self.status,
            "take_time": time.time() - self.start_time
        }
        m.report_info[f"CubicBezier2Load_{m.task_id}"] = state

    def _determine_logic(self, r: SimModule, m: Module):
        """
        Determine which path planning logic to use
        """
        # Get pallet number (from runtime GData, not recognition output)
        self.pallet_number = self._get_pallet_number(r)
        
        # Determine if using new logic
        # Conditions:
        # 1. New logic enabled
        # 2. Pallet number > first N pallets OR Outside Testing Mode (forced)
        
        # LOGGING HEADER
        PathPlanningConfig.log("=" * 60, "INFO")
        PathPlanningConfig.log(f"DECIDING PATH PLANNING LOGIC", "INFO")
        
        # Check if we should FORCE new logic for outside testing
        force_new_logic = PathPlanningConfig.outside_testing_mode
        
        if force_new_logic:
            PathPlanningConfig.log(f"Outside Testing Mode = True => FORCING NEW LOGIC", "INFO")
            self.use_new_logic = True
        else:
            self.use_new_logic = (
                PathPlanningConfig.enable_new_path_logic
                and self.pallet_number > PathPlanningConfig.first_n_pallets_use_old_logic
            )
            PathPlanningConfig.log(f"Container Mode Check: Enable={PathPlanningConfig.enable_new_path_logic}, "
                                  f"Pallet#={self.pallet_number} > OldLimit={PathPlanningConfig.first_n_pallets_use_old_logic} "
                                  f"=> UseNew={self.use_new_logic}", "INFO")

        PathPlanningConfig.log(f"FINAL DECISION: Logic [{'NEW (precise pickup)' if self.use_new_logic else 'OLD (standard pickup)'}]", "INFO")
        PathPlanningConfig.log("=" * 60, "INFO")
        
        if self.use_new_logic:
            self.current_phase = "precise_pickup"
            PathPlanningConfig.log(f"Starting new precise pickup flow", "INFO")
            PathPlanningConfig.log(f"  Phase 1: Forward Bezier retreat (away from pallet)", "INFO")
            PathPlanningConfig.log(f"  Phase 2: Lateral alignment (right/left)", "INFO")
            PathPlanningConfig.log(f"  Phase 3: Sideshifter adjustment (parallel during movement)", "INFO")
            PathPlanningConfig.log(f"  Phase 4: Backward straight approach (to pallet)", "INFO")
        else:
            self.current_phase = "old_logic"
            PathPlanningConfig.log(f"Using old logic (Bezier pickup)", "INFO")
    
    def _get_pallet_number(self, r: SimModule):
        """
        Get current pallet number
        Get from global data or initialize
        """
        g_data_dict = r.getGData()
        if not isinstance(g_data_dict, dict):
            g_data_dict = {}
        pallet_count = int(g_data_dict.get('pallet_count', 0) or 0)

        PathPlanningConfig.log(f"Current pallet count (from r.getGData): {pallet_count}", "DEBUG")

        return pallet_count
    
    def _execute_precise_pickup(self, r: SimModule, m: Module):
        """
        Execute precise pickup logic
        Flow: Forward retreat -> Lateral alignment -> Sideshifter adjustment -> Backward approach
        """
        if not self.init:
            PathPlanningConfig.log(f"=" * 60, "INFO")
            PathPlanningConfig.log(f"Initializing precise pickup system", "INFO")
            PathPlanningConfig.log(f"  Using recognition data from currentPoint", "INFO")
            
            # Verify recognition data exists
            if not m.GData.currentPoint:
                PathPlanningConfig.log(f"ERROR: No recognition data available!", "ERROR")
                self.status = MoveStatus.FAILED
                r.setError("No pallet recognition data available for precise pickup")
                return
            
            # Log recognition data details
            rec_data = m.GData.currentPoint
            PathPlanningConfig.log(f"  Recognition data:", "INFO")
            PathPlanningConfig.log(f"    Pallet X: {rec_data.get('x', 'N/A'):.3f}m", "INFO")
            PathPlanningConfig.log(f"    Pallet Y: {rec_data.get('y', 'N/A'):.3f}m", "INFO")
            PathPlanningConfig.log(f"    Pallet Yaw: {math.degrees(rec_data.get('yaw', 0)):.1f}°", "INFO")
            if 'palletWidth' in rec_data:
                PathPlanningConfig.log(f"    Pallet Width: {rec_data.get('palletWidth', 'N/A')}", "INFO")
            
            self.start_time = time.time()
            self.precise_pickup = PrecisePalletPickup(r)
            self.init = True
            PathPlanningConfig.log(f"Precise pickup system initialized", "INFO")
            PathPlanningConfig.log(f"=" * 60, "INFO")
        
        # Execute precise pickup (handles all phases internally)
        self.precise_pickup.run(r, m)
        
        if self.precise_pickup.status == MoveStatus.FINISHED:
            self.status = MoveStatus.FINISHED
            elapsed = time.time() - self.start_time
            PathPlanningConfig.log(f"=" * 60, "INFO")
            PathPlanningConfig.log(f"Precise pickup complete! Total time: {elapsed:.2f}s", "INFO")
            PathPlanningConfig.log(f"=" * 60, "INFO")
        elif self.precise_pickup.status == MoveStatus.FAILED:
            self.status = MoveStatus.FAILED
            elapsed = time.time() - self.start_time
            PathPlanningConfig.log(f"=" * 60, "ERROR")
            PathPlanningConfig.log(f"Precise pickup failed! Time: {elapsed:.2f}s", "ERROR")
            PathPlanningConfig.log(f"  Check logs above for failure phase", "ERROR")
            PathPlanningConfig.log(f"=" * 60, "ERROR")
            r.setError("Precise pickup failed - check logs for details")
        else:
            self.status = MoveStatus.RUNNING
    
    
    def _execute_old_logic(self, r: SimModule, m: Module):
        """
        Execute old path planning logic (original Bezier pickup)
        NOTE: Outside testing should not trigger ramp warnings.
        """
        # Initialize
        self._initialize(r, m)
        
        # Execute movement
        self._execute_movement(r, m)
        
        # Ramp safety check (first N pallets only) - disable in outside testing
        if (not PathPlanningConfig.outside_testing_mode) and (
            self.pallet_number <= PathPlanningConfig.first_n_pallets_use_old_logic
        ):
            robot_y = r.loc()['y']
            if abs(robot_y) > PathPlanningConfig.ramp_safe_y_max:
                PathPlanningConfig.log(f"警告 WARNING: 机器人 Y Robot Y={robot_y:.2f}m 超出斜坡安全区域 exceeds ramp safe zone "
                                      f"[{PathPlanningConfig.ramp_safe_y_min:.2f}, {PathPlanningConfig.ramp_safe_y_max:.2f}]", "WARN")
                r.setWarning(f"Robot Y={robot_y:.2f}m exceeds ramp safe zone!")

    def reset(self, r: SimModule):
        self.status = MoveStatus.RUNNING


if __name__ == '__main__':
    pass

