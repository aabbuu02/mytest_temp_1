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
        "tips": "机构动作选项",
        "type": "complex"        
    },
    "side":{
        "value": "",
        "tips":"货叉横移",
        "type":"float",
        "unit": "m"
    },
    "tilt":{
        "value": "",
        "tips":"货叉前倾",
        "type":"float",
        "unit": "m"
    },
    "lift":{
        "value": "",
        "tips":"货叉升降",
        "type":"float",
        "unit": "m"
    },
    "end_height":{
        "value": "",
        "tips":"货叉抬升结束高度",
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
        "tips":"是否使用360检测货物",
        "type":"float",
        "unit": "m"
    },
    "clearGBData":{
        "value": "",
        "tips":"清楚全局数据",
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
    """使用360检测货物的范围，机器人后方一个长方体的空间，参看点为里程中心"""
    x_min = -1.7                    #-1.5
    x_max = -5                      #-5
    y_min = -0.6                    #-0.8
    y_max = 0.6                     #0.8
    z_up = 0.6                      #0.5
    z_down = 0.25                   #0.25
    """
    goods_pre_dist，使用360检测货物后，移动机器人，和货物保持一定距离,默认3m，
    根据实际调整，只要相机能识别栈板即可，超过3.5可能识别不到，小于1m，可能识别不到
    """
    goods_pre_dist = 3
    """go2pallet表示不用360，导航到栈板前面的距离"""
    go2pallet_b3_load_dist = 4

    """定义货箱尺寸"""
    a = 2.48
    b = 10.0

    """如果是平台取货，栈板距离基准点的位置"""
    load_pallet_in_platform_dist = 0.1

    """进叉起始位置需要靠近货箱的距离"""
    B_dist = 0.15
    enable_B_dist = 2  # B_dist 的起效条件 enable_B_dist表示 栈板中心 和 机器人 odo 的距离


class CubicBezierPar:
    """
    三阶贝塞尔曲线控制点，控制参数，类似于在 roboshop 画三阶贝塞尔曲线
    """
    t = -0.08  # 越小，转弯幅度越小
    d_l = -0.28  # 越小，起步角度越小,车的左边取货
    d_r = 0.28  # 越小，起步角度越小，车子右边取货
    extend_factor = 0.7
    odo_2_pallet_dist = 0.45  # 允许的 里程中心 -> 栈板 的最大距离，用于限制车子在车厢内的 y 方向的移动范围，值越大，y 方向能移动的距离越大
    backDist = 0.02  # 后退距离


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
    
    # ============ Safety Parameters ============
    wall_clearance = 0.2                # Safety margin from container walls (meters)
                                        # LOG: 200mm minimum clearance to prevent wall collision
    
    container_width = 2.48              # Container width (meters)
    robot_width = 1.25                  # Robot body width (meters)
    
    # Calculate safe Y range
    # Formula: (container_width/2) - (robot_width/2) - wall_clearance
    # = (2.48/2) - (1.25/2) - 0.2 = 1.24 - 0.625 - 0.2 = 0.415
    safe_y_max = 0.415                  # Maximum safe Y position (meters)
    safe_y_min = -0.415                 # Minimum safe Y position (meters)
                                        # LOG: Robot center must stay within ±415mm range
    
    # ============ Pallet Strategy ============
    first_n_pallets_use_old_logic = 0  # First N pallets use old logic
                                        # LOG: Set to 0 to test new logic for ALL pallets
                                        # LOG: Set to 4 for production (pallets 1-4 old, 5-22 new)

    
    enable_new_path_logic = True        # Enable new path planning logic
                                        # LOG: Set to False to use old logic for all (testing)
    
    # ============ Side-shifter Parameters ============
    dynamic_sideshifter_enabled = True  # Enable dynamic side-shift adjustment
                                        # LOG: Dynamically adjust side-shifter during straight approach
    
    sideshifter_max = 0.2               # Maximum side-shifter range (±200mm)
                                        # LOG: Hardware limit ±200mm
    
    # ============ Ramp Safety Parameters ============
    ramp_width = 1.8                    # Ramp width (meters)
    ramp_safe_y_max = 0.4               # Safe Y position on ramp (meters)
    ramp_safe_y_min = -0.4              # Safe Y position on ramp (meters)
                                        # LOG: Ramp narrower than container, stricter Y limits
    
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
    
    @staticmethod
    def validate_config():
        """Validate configuration parameters"""
        errors = []
        
        # Check safe range
        calculated_safe_y = (PathPlanningConfig.container_width / 2) - \
                           (PathPlanningConfig.robot_width / 2) - \
                           PathPlanningConfig.wall_clearance
        
        if abs(calculated_safe_y - PathPlanningConfig.safe_y_max) > 0.01:
            errors.append(f"safe_y_max incorrect: {PathPlanningConfig.safe_y_max} "
                         f"should be: {calculated_safe_y:.3f}")
        
        # Check retreat distance
        if PathPlanningConfig.retreat_distance < 1.0:
            errors.append(f"retreat_distance too small: {PathPlanningConfig.retreat_distance}m, "
                         f"recommend >= 1.5m")
        
        # Check curve factor
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
            PathPlanningConfig.log(f"  - Safe Y range: [{PathPlanningConfig.safe_y_min:.3f}, "
                                  f"{PathPlanningConfig.safe_y_max:.3f}]m", "INFO")
            PathPlanningConfig.log(f"  - Retreat distance: {PathPlanningConfig.retreat_distance}m", "INFO")
            PathPlanningConfig.log(f"  - Curve factor: {PathPlanningConfig.retreat_curve_factor}", "INFO")
            return True


# ============================================================================
# STARTUP BANNER - Confirms this file is loaded
# ============================================================================
print("=" * 80)
print("🚀 ABU'S NEW PATH PLANNING LOGIC LOADED!")
print("=" * 80)
print(f"📦 File: TKC.py (Updated Version)")
print(f"✅ PathPlanningConfig: Loaded")
print(f"✅ BezierRetreat: Loaded")
print(f"✅ StraightApproachWithSideShift: Loaded")
print(f"⚙️  first_n_pallets_use_old_logic = {PathPlanningConfig.first_n_pallets_use_old_logic}")
print(f"⚙️  enable_new_path_logic = {PathPlanningConfig.enable_new_path_logic}")
print("=" * 80)
# ============================================================================



class LoadType(int):
    first = 0  # 第一次取货，亦谓之使用360检测货物，然后识别取货
    second = 1  # 货物在货箱口
    in_rec = 2  # 不使用360检测，需要识别取货，比 first 流程简单，少了一步检测360
    in_not_rec = 3  # 不使用360检测，不需要识别，因为上一次识别，同时识别到了两个栈板，提高效率，直接拿结果，不需要识别


class MotorName(str):
    side = "sideshift5"
    lift = "lift3"
    tilt = "tilt6"


class RecOutput:
    """全局变量"""
    valid = False  # 栈板识别是否有效
    results = []  # 最近一次栈板识别的结果
    nextPoint = None  # 如果有数据，表示当前下一个栈板的坐标
    currentPoint = None  # 如果有数据，表示当前最近栈板的坐标
    good_location = []  # 360识别是否有货后，输出货物的坐标（大概）
    has_goods = False  # 360识别是否有货

    def to_dict(self):
        # 由于类属性是共享的，我们只序列化实例属性（如果有的话）
        # 这里我们假设这些属性是实例属性而不是类属性（即它们应该在 __init__ 中初始化）
        # 但由于你当前的代码定义的是类属性，我们仅演示如何转换这些“假设的”实例属性
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
        当 currentPoint 的栈板任务完成，调用此函数，更新 currentPoint

        """
        self.good_location = [self.currentPoint["x"], self.currentPoint["y"], self.currentPoint["yaw"]]
        self.currentPoint = None
        if self.nextPoint:
            self.currentPoint = self.nextPoint
            self.nextPoint = None


class Module(BasicModule):
    def __init__(self, r: SimModule, args):
        super(Module, self).__init__()
        # 车厢参数
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
                                   comment=" 运行超时时间")
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
            """==== 输入参数初始化函数 ==== """
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
                """==== 初始化操作，确定任务内容 ==== """
                self.handle = self.get_handle(r, args)
                if not self.handle:
                    r.setError(f"找不到输入的操作:{self.operation}，检检查脚本输入参数。")
                    self.status = MoveStatus.FAILED
                    return self.status
            self.init = False
        if not self.init and self.status != MoveStatus.FINISHED:
            """==== 初始化完成，开始运行任务 ==== """
            self.handle_robot(r)
        self.status = self.operation_status
        self.setAndCheckGoodsStatus(r)
        """==== 循环结束，上报数据（打印） ==== """
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
        """==== 初始化操作，处理脚本的输入参数 按需取用 ==== """
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
        # 清理全局数据
        if any([self.is_clear_GB_data, self.use360, not self.truckLoad]):
            for _ in range(3):
                r.setGData({})
        if g_data := r.getGData():
            self.GData.to_object(g_data)
        """==== 初始化操作，获取导航任务参数，按需取用 ==== """
        self.get_move_task_params(r)  # 获取任务的货物 goodsId

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
        # =====处理任务暂停时的业务=====
        r.setNotice(f"suspend task")
        self.status = MoveStatus.SUSPENDED

    def _init_operations(self):
        """初始化操作映射"""
        return {
            "load": self.load, "unload": self.unload, "side": self.side,
            "lift": self.lift, "tilt": self.tilt, "zero": self.zero,
            "rec": self.rec, "Mid360AreaDetect": self.Mid360AreaDetect,
        }

    # 简化的操作方法
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
        # 托盘计数追踪 Pallet count tracking
        # 在每次取货操作开始时增加计数 Increment count at start of each load operation
        g_data_dict = r.getGData()
        if not isinstance(g_data_dict, dict):
            g_data_dict = {}
        
        pallet_count = g_data_dict.get('pallet_count', 0) + 1
        g_data_dict['pallet_count'] = pallet_count
        r.setGData(g_data_dict)
        
        PathPlanningConfig.log(f"开始取货操作 Starting load operation, 托盘编号 Pallet number: {pallet_count}", "INFO")
        
        self.load_type = self.get_load_type(r)
        if self.load_type == LoadType.first:
            self.task_list = [
                ForkMotor(r, MotorName.side, 0, self.side_speed),
                ForkMotor(r, MotorName.lift, self.lift_height, self.lift_speed),
                Rec(r, self.rec_file),  # 3 识别
                GoToPre(r, 3),
                CubicBezier2Load(r),  # 5 规划曲线取货
                ForkMotor(r, MotorName.lift, self.end_height, self.lift_speed),  # 6 抬升
                GoToPre(r, 5),  # 7 后退
                ForkMotor(r, MotorName.side, 0, self.side_speed)  # 9 横移回零
            ]
            if self.use360:
                self.task_list.insert(2, Mid360AreaDetect(r, self.entry_point_id))  # 1 360检测货物
        elif self.load_type == LoadType.second:
            self.task_list = []
        elif self.load_type == LoadType.in_rec:
            self.task_list = [
                ForkMotor(r, MotorName.side, 0, self.side_speed),
                ForkMotor(r, MotorName.lift, self.lift_height, self.lift_speed),
                GoToPre(r, 2),  # 1 到达前置点,这是根据上一托的位置判定
                Rec(r, self.rec_file),  # 2 识别
                GoToPre(r, 3),  # 3 往前走一段距离
                CubicBezier2Load(r),  # 4 规划曲线取货
                ForkMotor(r, MotorName.lift, self.end_height, self.lift_speed),  # 5 抬升
                GoToPre(r, 5),  # 6 后退
                ForkMotor(r, MotorName.side, 0, self.side_speed)  # 8 横移回零
            ]
        elif self.load_type == LoadType.in_not_rec:  # 不识别直接取货
            self.task_list = [
                ForkMotor(r, MotorName.side, 0, self.side_speed),
                ForkMotor(r, MotorName.lift, self.lift_height, self.lift_speed),
                GoToPre(r, 3),  # 1 到达前置点
                CubicBezier2Load(r),  # 2 规划曲线取货
                ForkMotor(r, MotorName.lift, self.end_height, self.lift_speed),  # 3 抬升
                GoToPre(r, 5),  # 4 后退
                ForkMotor(r, MotorName.side, 0, self.side_speed)  # 8 横移回零
            ]
        else:
            self.status = MoveStatus.FAILED
            r.setError("取货类型不对")

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
        获取moveTask参数 goods_id
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
    到达前置点，有三种情况，go_type，
    0（4）、（到达前置点）使用 360 检测障碍物，到达前置点，然后识别
    1、（走到取货点）拿上一次栈板位置，走到取货点，不识别，然后曲线取货
    2、（到达前置点）拿上一次栈板位置，到达前置点，识别后，再次走到取货点
    3、（走到取货点）识别成功后，走到取货点，然后曲线取货
    5、 抬起货叉后，后退直线1.5m
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
            5: lambda: self._init_type5(loc)
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
            r.setError(f"mid360 检测货物后退距离超过了 {distance}")

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
        # 设置后退参数
        self.x, self.y, self.yaw = self.back_distance, 0.001, 0.001
        self.coordinate, self.backMode = 'robot', 0
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
            r.resetRec()
            self.init = True
            self.status = MoveStatus.RUNNING
            self.start_time = time.time()
            self.do_duration_time = 0
        else:
            if time.time() - self.start_time >= self.do_duration_time:
                rec_status = r.getRecStatus()
                self.rec_status = rec_status
                if rec_status == 3:
                    self.rec_times = self.rec_times + 1
                    if self.rec_times > self.max_rec_times:
                        self.status = MoveStatus.FAILED
                    r.resetRec()
                elif rec_status == 0 or rec_status == 1:
                    r.doRecWithAngle(self.filename, 0.0)
                elif rec_status == 2:
                    self.p_num = r.getRecResultSize()
                    for num in range(1, self.p_num + 1):
                        self.results.append(r.getRecResults(num))
                    self.status = MoveStatus.FINISHED
                    self.update_RecOutput(r, m)
                    r.setGData(m.GData.to_dict())
        cur_state = dict()
        cur_state['pallet num'] = self.p_num
        cur_state['rec_status'] = self.rec_status
        cur_state['take_time'] = time.time() - self.start_time
        m.report_info[f'Rec-{m.task_id}'] = cur_state

    def reset(self, r: SimModule):
        r.resetRec()
        self.status = MoveStatus.RUNNING

    def update_RecOutput(self, r: SimModule, m: Module):
        if not self.results:
            r.setError("update_RecOutput")
            return
        if len(self.results) == 1:
            m.GData.results = self.results
            m.GData.valid = True
            m.GData.currentPoint = self.results[0]
            m.GData.nextPoint = []
        if len(self.results) == 2:
            m.GData.results = self.results
            m.GData.valid = True
            m.GData.currentPoint, m.GData.nextPoint = self.get_currentPoint(r)

    def calculate_distance(self, rec, point2):
        """# 定义一个函数来计算两个点之间的欧几里得距离"""
        point = [rec['x'], rec['y'], rec['yaw']]
        # 提取坐标
        x1, y1, yaw1 = point
        x2, y2, yaw2 = point2
        # 计算二维平面上的距离（忽略yaw，因为yaw是角度，不是位置坐标）
        distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        return distance

    def get_currentPoint(self, r):
        loc = [r.loc()['x'], r.loc()['y'], r.loc()['angle']]
        # 计算每个对象的二维距离（基于原点 [a, b]），并绑定原始数据
        sorted_results = sorted(self.results,
                                key=lambda obj: math.sqrt((obj["x"] - loc[0])**2 + (obj["y"] - loc[1])** 2) if abs(obj["x"] - loc[0]) <= 0.12 else abs(obj["y"] - loc[1]),
                                reverse=True)
        return sorted_results[1], sorted_results[0]


class ForkMotor:
    def __init__(self, r: SimModule, motor, position, speed):
        """
        Args:
            motor(string): 电机名
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
        通过mid360检测货箱是否有货物以及返回最近货物的坐标
        """
        self.point_num = 0
        self.node_num1 = None
        self.status = MoveStatus.NONE
        self.polygon1 = []  # 车厢在平面投影的四个顶点坐标
        self.init = False
        self.count_num = 0
        self.nodes = []
        self.point = []
        self.id = site_id
        self.node_slice = None
        self.site_length = 0.475
        self.ap_loc = None  # ap点坐标
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
        all_cloud = r.allCameraCloud()["allcloud"]  # 机器人坐标系
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
                    if self.is_point_inside_rectangle((n["x"], n["y"]), self.polygon1):  # 剔除斜坡的点和多余的点
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
            # print("符合条件的点:", len(self.point))
            if self.point_num >= 3:
                # 有货物，输出最近的坐标
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
        """判断一个点是否在矩形内部
        Args:
        point: tuple，表示点的坐标，例如(x，y)
        rectangle: list，表示矩形的四个顶点坐标，例如[(x1，y1)，(x2，y2),(x3，y3)，(x4，y4)
        return:
        boo1:表示点是否在矩形内部或者边缘
        """
        x, y = point
        x1, y1 = rectangle[0]
        x2, y2 = rectangle[1]
        x3, y3 = rectangle[2]
        x4, y4 = rectangle[3]
        # 点在矩形内部的条件是:点的横坐标在矩形的最小横坐标和最大横坐标之间
        # 点的纵坐标在矩形的最小纵坐标和最大纵坐标之间
        return min(x1, x2, x3, x4) <= x <= max(x1, x2, x3, x4) and min(y1, y2, y3, y4) <= y <= max(y1, y2, y3, y4)

    def get_latest_point(self):
        result = []
        current_streak = 0
        for i, point in enumerate(self.point):
            if i == 0:
                # 处理第一个点，无法与前一个点比较
                prev_point = None
            else:
                prev_point = self.point[i - 1]
            # 检查当前点和前一点的y值是否相同
            if point.y != prev_point.y:
                current_streak = 0
            else:
                # 计算x差值，考虑浮点精度问题
                dx = abs(point.x - prev_point.x)
                if dx < 0.1:  # 根据实际需求调整阈值
                    current_streak += 1
                    # 如果达到或超过50个连续点，则认为是直线提取子数组并添加到result中
                    if current_streak >= 50:
                        # 确保子数组包含前50个连续的满足条件的点
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
        # Clamp to side-shifter limits
        max_shift = PathPlanningConfig.sideshifter_max
        
        if abs(lateral_offset) <= max_shift:
            adjustment = lateral_offset
            PathPlanningConfig.log(f"  Side-shifter can fully compensate: {adjustment:.3f}m", "DEBUG")
        elif lateral_offset > 0:
            adjustment = max_shift
            PathPlanningConfig.log(f"  Side-shifter at max: {adjustment:.3f}m "
                                  f"(needed: {lateral_offset:.3f}m)", "WARN")
        else:
            adjustment = -max_shift
            PathPlanningConfig.log(f"  Side-shifter at min: {adjustment:.3f}m "
                                  f"(needed: {lateral_offset:.3f}m)", "WARN")
        
        return adjustment
    
    def run(self, r: SimModule, m: Module):
        """Execute straight approach"""
        if not self.init:
            PathPlanningConfig.log(f"Starting straight approach execution", "INFO")
            self.start_time = time.time()
            r.resetPath()
            
            # Initialize side-shifter motor
            self.side_motor = ForkMotor(r, MotorName.side, self.sideshifter_target, 0.5)
            
            # Set straight path (backward mode)
            r.setPathBackMode(True)
            r.setPathMaxSpeed(0.1)
            r.setPathReachDist(0.02)
            r.setPathReachAngle(0.1)
            
            # Set target point - use simple straight path
            current_pos = [r.loc()['x'], r.loc()['y']]
            xs = [current_pos[0], self.target_world[0]]
            ys = [current_pos[1], self.target_world[1]]
            r.setPathOnWorld(xs, ys, self.target_world[2])
            
            PathPlanningConfig.log(f"  Path set: from ({current_pos[0]:.3f}, {current_pos[1]:.3f}) "
                                  f"to ({self.target_world[0]:.3f}, {self.target_world[1]:.3f})", "INFO")
            
            # Start side-shifter adjustment
            if PathPlanningConfig.dynamic_sideshifter_enabled:
                PathPlanningConfig.log(f"  Starting dynamic side-shift: {self.sideshifter_target:.3f}m", "INFO")
            
            self.init = True
        
        # Execute side-shifter adjustment
        if PathPlanningConfig.dynamic_sideshifter_enabled and self.side_motor:
            if self.side_motor.status == MoveStatus.FAILED:
                self.status = MoveStatus.FAILED
                PathPlanningConfig.log(f"Side-shifter failed!", "ERROR")
                return
            self.side_motor.run(r, m, self.sideshifter_target)
        
        # Execute straight path
        if self.status != MoveStatus.FAILED:
            r.goPath()
        
        # Check for fork insertion signal
        if any(node['status'] for node in r.Di().get('node', [])
               if node['id'] in (16, 17)):
            self.reatch = True
            PathPlanningConfig.log(f"Fork insertion detected (DI 16/17)", "INFO")
        
        # Completion condition
        if self.reatch:
            r.resetPath()
            self.status = MoveStatus.FINISHED
            elapsed = time.time() - self.start_time
            current_pos = [r.loc()['x'], r.loc()['y']]
            
            PathPlanningConfig.log(f"Straight approach finished, time: {elapsed:.2f}s", "INFO")
            PathPlanningConfig.log(f"  Final pos: X={current_pos[0]:.3f}, Y={current_pos[1]:.3f}", "INFO")
            
            # Switch to next pallet
            m.GData.currentPoint_switch_nextPoint()
            r.setGData(m.GData.to_dict())
            PathPlanningConfig.log(f"  Switched to next pallet", "INFO")
        else:
            self.status = MoveStatus.RUNNING
    
    def reset(self, r: SimModule):
        self.status = MoveStatus.RUNNING



class CubicBezier2Load:
    """3阶贝塞尔曲线进叉取货"""

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
        self.xs, self.ys = [], []  # 添加贝塞尔曲线点集存储
        
        # 新路径规划相关 New path planning related
        self.pallet_number = 0  # 托盘编号 Pallet number
        self.use_new_logic = False  # 是否使用新逻辑 Whether to use new logic
        self.retreat_phase = None  # 后退阶段 Retreat phase
        self.approach_phase = None  # 接近阶段 Approach phase
        self.current_phase = "none"  # 当前阶段 Current phase: none, retreat, approach, old_logic
        
        PathPlanningConfig.log("CubicBezier2Load 初始化 initialized", "INFO")

    def run(self, r: SimModule, m: Module):
        self._validate_input(r, m)
        
        # 初始化：确定使用哪种逻辑 Initialize: determine which logic to use
        if not self.init:
            self._determine_logic(r, m)
        
        # 根据逻辑执行 Execute based on logic
        if self.use_new_logic:
            self._execute_new_logic(r, m)
        else:
            self._execute_old_logic(r, m)
        
        self._report_status(m)

    def _validate_input(self, r: SimModule, m: Module):
        if not m.GData.currentPoint:
            self.status = MoveStatus.FAILED
            r.setError(f'没有 currentPoint {m.GData.to_dict()}')

    def _initialize(self, r: SimModule, m: Module):
        if self.init: return
        self.start_time = time.time()
        r.resetPath()

        current_point = m.GData.currentPoint
        pallet_pos = [current_point['x'], current_point['y'], current_point['yaw']]
        loc_robot = [r.loc()['x'], r.loc()['y'], 0]
        # loc_robot = [r.loc()['x'], r.loc()['y'], r.loc()['yaw']]

        # 计算横移距离
        pallet_to_lm = RBK.Pos2Base(pallet_pos, loc_robot)
        self.side = self._calculate_side_offset(pallet_to_lm[1])

        # 计算目标位置
        y_offset = self._calculate_y_offset(pallet_to_lm[1])
        target_local = [pallet_to_lm[0] + CubicBezierPar.backDist, y_offset, 0]
        self.target_world = RBK.Pos2World(target_local, loc_robot)

        # 生成贝塞尔曲线
        curve_params = self._get_curve_params(pallet_to_lm)
        P0, P3 = loc_robot[:2], self.target_world[:2]
        self.xs, self.ys, self.control_point = self._generate_bezier_curve(P0, P3, curve_params)

        # 设置路径参数 - 修复：添加缺失的路径设置
        r.setPathReachAngle(0.1)
        r.setPathMaxSpeed(0.1)
        r.setPathBackMode(True)
        r.setPathOnWorld(self.xs, self.ys, self.target_world[2])  # 修复：设置贝塞尔曲线路径

        self.C_msg = {
            # "栈板坐标": pallet_pos, "栈板2机器人": pallet_to_lm,
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
        # 执行横移
        if self.need_side:
            if self.side_motor.status == MoveStatus.FAILED:
                self.status = MoveStatus.FAILED
                return
            self.side_motor.run(r, m, self.side)
        # 执行路径跟踪
        if self.status != MoveStatus.FAILED:
            r.goPath()
        # 检测到位信号
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
        确定使用哪种路径规划逻辑 Determine which path planning logic to use
        """
        # 获取托盘编号 Get pallet number
        self.pallet_number = self._get_pallet_number(m)
        
        # 判断是否使用新逻辑 Determine if using new logic
        # 条件 Conditions:
        # 1. 新逻辑已启用 New logic enabled
        # 2. 托盘编号 > 前N个 Pallet number > first N
        self.use_new_logic = (PathPlanningConfig.enable_new_path_logic and 
                             self.pallet_number > PathPlanningConfig.first_n_pallets_use_old_logic)
        
        PathPlanningConfig.log(f"=" * 60, "INFO")
        PathPlanningConfig.log(f"托盘编号 Pallet number: {self.pallet_number}", "INFO")
        PathPlanningConfig.log(f"使用逻辑 Logic: {'新逻辑 NEW (后退+直线) (retreat+straight)' if self.use_new_logic else '旧逻辑 OLD (贝塞尔取货) (Bezier pickup)'}", "INFO")
        PathPlanningConfig.log(f"=" * 60, "INFO")
        
        if self.use_new_logic:
            self.current_phase = "retreat"
            PathPlanningConfig.log(f"开始新逻辑流程 Starting new logic flow", "INFO")
            PathPlanningConfig.log(f"  阶段1: 后退 Phase 1: Retreat", "INFO")
            PathPlanningConfig.log(f"  阶段2: 直线接近 Phase 2: Straight approach", "INFO")
        else:
            self.current_phase = "old_logic"
            PathPlanningConfig.log(f"使用旧逻辑（贝塞尔取货）Using old logic (Bezier pickup)", "INFO")
    
    def _get_pallet_number(self, m: Module):
        """
        获取当前托盘编号 Get current pallet number
        从全局数据中获取或初始化 Get from global data or initialize
        """
        g_data_dict = m.GData.to_dict()
        pallet_count = g_data_dict.get('pallet_count', 0)
        
        PathPlanningConfig.log(f"当前托盘计数 Current pallet count: {pallet_count}", "DEBUG")
        
        return pallet_count
    
    def _execute_new_logic(self, r: SimModule, m: Module):
        """
        执行新路径规划逻辑 Execute new path planning logic
        流程 Flow: 后退 Retreat -> 直线接近 Straight approach
        """
        if not self.init:
            PathPlanningConfig.log(f"初始化新逻辑 Initializing new logic", "INFO")
            self._initialize_new_logic(r, m)
            self.init = True
        
        # 阶段1: 后退 Phase 1: Retreat
        if self.current_phase == "retreat":
            if self.retreat_phase.status == MoveStatus.NONE:
                PathPlanningConfig.log(f"开始后退阶段 Starting retreat phase", "INFO")
            
            self.retreat_phase.run(r, m)
            
            if self.retreat_phase.status == MoveStatus.FINISHED:
                PathPlanningConfig.log(f"后退阶段完成，切换到接近阶段 Retreat finished, switching to approach phase", "INFO")
                self.current_phase = "approach"
                # 初始化接近阶段 Initialize approach phase
                self._initialize_approach_phase(r, m)
            elif self.retreat_phase.status == MoveStatus.FAILED:
                self.status = MoveStatus.FAILED
                PathPlanningConfig.log(f"后退阶段失败 Retreat phase failed!", "ERROR")
            else:
                self.status = MoveStatus.RUNNING
            return
        
        # 阶段2: 直线接近 Phase 2: Straight approach
        if self.current_phase == "approach":
            if self.approach_phase.status == MoveStatus.NONE:
                PathPlanningConfig.log(f"开始接近阶段 Starting approach phase", "INFO")
            
            self.approach_phase.run(r, m)
            
            if self.approach_phase.status == MoveStatus.FINISHED:
                self.status = MoveStatus.FINISHED
                PathPlanningConfig.log(f"接近阶段完成，新逻辑流程结束 Approach finished, new logic flow complete", "INFO")
            elif self.approach_phase.status == MoveStatus.FAILED:
                self.status = MoveStatus.FAILED
                PathPlanningConfig.log(f"接近阶段失败 Approach phase failed!", "ERROR")
            else:
                self.status = MoveStatus.RUNNING
            return
    
    def _initialize_new_logic(self, r: SimModule, m: Module):
        """
        初始化新路径规划逻辑 Initialize new path planning logic
        """
        PathPlanningConfig.log(f"初始化新逻辑组件 Initializing new logic components", "INFO")
        
        # 获取托盘和机器人位置 Get pallet and robot positions
        current_point = m.GData.currentPoint
        pallet_pos = [current_point['x'], current_point['y'], current_point['yaw']]
        robot_pos = [r.loc()['x'], r.loc()['y'], r.loc()['angle']]
        
        # 创建并初始化后退阶段 Create and initialize retreat phase
        self.retreat_phase = BezierRetreat(r)
        xs, ys, target = self.retreat_phase.calculate_retreat_path(pallet_pos, robot_pos)
        
        PathPlanningConfig.log(f"后退阶段已初始化 Retreat phase initialized", "INFO")
        
        # 接近阶段将在后退完成后初始化 Approach phase will be initialized after retreat
        self.approach_phase = None
    
    def _initialize_approach_phase(self, r: SimModule, m: Module):
        """
        初始化接近阶段 Initialize approach phase
        在后退完成后调用 Called after retreat completes
        """
        PathPlanningConfig.log(f"初始化接近阶段 Initializing approach phase", "INFO")
        
        # 获取当前机器人位置（后退后）Get current robot position (after retreat)
        robot_pos = [r.loc()['x'], r.loc()['y'], r.loc()['angle']]
        
        # 获取托盘位置（从识别数据）Get pallet position (from recognition data)
        current_point = m.GData.currentPoint
        pallet_pos = [current_point['x'], current_point['y'], current_point['yaw']]
        
        # 创建并初始化接近阶段 Create and initialize approach phase
        self.approach_phase = StraightApproachWithSideShift(r)
        target, sideshifter = self.approach_phase.calculate_approach_path(pallet_pos, robot_pos)
        
        PathPlanningConfig.log(f"接近阶段已初始化 Approach phase initialized", "INFO")
    
    def _execute_old_logic(self, r: SimModule, m: Module):
        """
        执行旧路径规划逻辑（原贝塞尔取货）Execute old path planning logic (original Bezier pickup)
        保留原有逻辑，添加斜坡安全检查 Preserve original logic, add ramp safety check
        """
        # 初始化 Initialize
        self._initialize(r, m)
        
        # 执行移动 Execute movement
        self._execute_movement(r, m)
        
        # 斜坡安全检查（仅前4个托盘）Ramp safety check (first 4 pallets only)
        if self.pallet_number <= PathPlanningConfig.first_n_pallets_use_old_logic:
            robot_y = r.loc()['y']
            if abs(robot_y) > PathPlanningConfig.ramp_safe_y_max:
                PathPlanningConfig.log(f"警告 WARNING: 机器人 Y Robot Y={robot_y:.2f}m 超出斜坡安全区域 exceeds ramp safe zone "
                                      f"[{PathPlanningConfig.ramp_safe_y_min:.2f}, {PathPlanningConfig.ramp_safe_y_max:.2f}]", "WARN")
                r.setWarning(f"Robot Y={robot_y:.2f}m exceeds ramp safe zone!")

    def reset(self, r: SimModule):
        self.status = MoveStatus.RUNNING


if __name__ == '__main__':
    pass

