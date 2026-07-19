# -*- coding: utf-8 -*-
"""
K230 单功能视觉程序

功能：
  1. 检测 A4 靶子外侧黑框并计算靶心。
  2. 完整框丢失时，用可见白纸区域给出粗搜索方向。
  3. 计算靶心/引导点相对画面中心的像素偏差。
  4. 显示并通过 UART3 发送偏差。

通信：X<dx>Y<dy>Z<status>E\r\n
状态：Z=1完整黑框，Z=2白纸粗引导，Z=0完全丢失。
"""
import time
import os
import math
import gc
time.sleep_ms(500)
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA
import sys

sys.path.append('/sdcard/examples/libs')
from libs.YOLO import YOLOv8

print("[BOOT] partial-safe script start")


# ======================== 图像与显示 ========================

# A4瞄准继续使用960x720逻辑坐标，保持STM32 PID收到的误差尺度不变。
AIM_W, AIM_H = 960, 720
# YOLO独立使用640x480，降低KPU预处理、帧缓冲和内存带宽压力。
AI_W, AI_H = 640, 480
VIEW_W, VIEW_H = 320, 240     # 通道1：小尺寸黑框检测与LCD显示
LCD_W, LCD_H = 800, 480
DISPLAY_X = (LCD_W - VIEW_W) // 2
DISPLAY_Y = (LCD_H - VIEW_H) // 2

sensor = Sensor(width=1280, height=960)
sensor.reset()
print("[BOOT] sensor reset ok")
sensor.set_framesize(chn=CAM_CHN_ID_0, width=AI_W, height=AI_H)
sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_0)
sensor.set_framesize(chn=CAM_CHN_ID_1, width=VIEW_W, height=VIEW_H)
sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

Display.init(Display.ST7701, width=LCD_W, height=LCD_H, to_ide=False)
MediaManager.init()
sensor.run()
print("[BOOT] display/media/sensor ok")


# ======================== UART3 ========================

def get_uart3_pins():
    """庐山派 K230-CanMV 的 UART3 固定引出到 GPIO50/51。

    不再使用 os.uname()[-1] 自动判断：CanMV 的 machine 字段在
    不同固件中可能只返回 "K230"，会误选 GPIO32/33，导致STM32始终收不到数据。
    """
    return 50, 51


uart_tx_pin, uart_rx_pin = get_uart3_pins()
fpioa = FPIOA()
fpioa.set_function(uart_tx_pin, FPIOA.UART3_TXD)
fpioa.set_function(uart_rx_pin, FPIOA.UART3_RXD)
uart = UART(UART.UART3, baudrate=115200, bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

last_tx_log_tick = 0
task_mode = 1
uart_command_buffer = ""
stable_digit = -1
digit_candidate = -1
digit_candidate_count = 0


def update_task_mode():
    """接收 STM32 的 #<task_id>\n 命令并切换视觉任务。"""
    global task_mode, uart_command_buffer
    global stable_digit, digit_candidate, digit_candidate_count
    global frame_cache, frame_scan_wait, frame_misses
    global partial_cache, partial_scan_wait, partial_misses
    global task5_waypoint_index, task5_arrival_count
    global task5_tracing, task5_complete
    global task5_filtered_corners, task5_filter_misses
    global task5_valid_count, task5_resume_blank
    global task5_frozen_corners, task5_corner_tx_index
    global task5_frozen_plane_corners
    global task5_aim_count, task5_aim_done
    global task5_area_cm2, task5_long_cm, task5_short_cm
    if not uart.any():
        return
    data = uart.read()
    if not data:
        return
    try:
        text = data.decode()
    except Exception:
        return
    for ch in text:
        if ch == '#':
            uart_command_buffer = "#"
        elif uart_command_buffer:
            if ch == '\n' or ch == '\r':
                try:
                    new_mode = int(uart_command_buffer[1:])
                    if 0 <= new_mode <= 7:
                        task_mode = new_mode
                        print("[MODE] task {}".format(task_mode))
                        stable_digit = -1
                        digit_candidate = -1
                        digit_candidate_count = 0
                        frame_cache = None
                        frame_scan_wait = 0
                        frame_misses = 0
                        partial_cache = None
                        partial_scan_wait = 0
                        partial_misses = 0
                        task5_waypoint_index = 0
                        task5_arrival_count = 0
                        task5_tracing = False
                        task5_complete = False
                        task5_filtered_corners = None
                        task5_filter_misses = 0
                        task5_valid_count = 0
                        task5_resume_blank = False
                        task5_frozen_corners = None
                        task5_frozen_plane_corners = None
                        task5_corner_tx_index = 0
                        task5_aim_count = 0
                        task5_aim_done = False
                        task5_area_cm2 = 0.0
                        task5_long_cm = 0.0
                        task5_short_cm = 0.0
                except Exception:
                    pass
                uart_command_buffer = ""
            elif len(uart_command_buffer) < 4:
                uart_command_buffer += ch
            else:
                uart_command_buffer = ""


# ======================== YOLO 数字识别（任务6） ========================

KMODEL_PATH = "/sdcard/examples/kmodel/digit_detect.kmodel"
DIGIT_LABELS = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
DIGIT_MODEL_SIZE = [320, 320]
DIGIT_CONF_THRESHOLD = 0.2
DIGIT_NMS_THRESHOLD = 0.45
DIGIT_STABLE_FRAMES = 5

digit_yolo = YOLOv8(
    task_type="detect",
    mode="video",
    kmodel_path=KMODEL_PATH,
    labels=DIGIT_LABELS,
    rgb888p_size=[AI_W, AI_H],
    model_input_size=DIGIT_MODEL_SIZE,
    display_size=[AI_W, AI_H],
    conf_thresh=DIGIT_CONF_THRESHOLD,
    nms_thresh=DIGIT_NMS_THRESHOLD,
    max_boxes_num=20,
    debug_mode=0
)
digit_yolo.config_preprocess()


def best_digit_result(results):
    """返回 (digit, confidence, cx, cy)，兼容官方 YOLOv8 detect 输出。"""
    if not results or len(results) < 3 or not results[0]:
        return None
    boxes, class_ids, scores = results[0], results[1], results[2]
    best = None
    for i in range(len(boxes)):
        try:
            x, y, w, h = boxes[i]
            score = float(scores[i])
            class_id = int(class_ids[i])
        except Exception:
            continue
        if class_id < 0 or class_id > 9:
            continue
        if best is None or score > best[1]:
            best = (class_id, score, int(x + w / 2), int(y + h / 2))
    return best


def update_digit_stability(detected):
    """同一数字连续多帧才回传，避免Task6写错瞬时误识别结果。"""
    global stable_digit, digit_candidate, digit_candidate_count
    if not detected:
        digit_candidate = -1
        digit_candidate_count = 0
        stable_digit = -1
        return -1
    digit = detected[0]
    if digit == digit_candidate:
        digit_candidate_count += 1
    else:
        digit_candidate = digit
        digit_candidate_count = 1
    if digit_candidate_count >= DIGIT_STABLE_FRAMES:
        stable_digit = digit
    return stable_digit


def draw_digit_results(view, results, detected):
    """把640x480 AI检测框缩放绘制到320x240显示通道。"""
    if results and len(results) >= 3 and results[0]:
        boxes, class_ids, scores = results[0], results[1], results[2]
        for i in range(len(boxes)):
            try:
                x, y, w, h = boxes[i]
                class_id = int(class_ids[i])
                score = float(scores[i])
            except Exception:
                continue
            vx = int(x * VIEW_W / AI_W)
            vy = int(y * VIEW_H / AI_H)
            vw = max(1, int(w * VIEW_W / AI_W))
            vh = max(1, int(h * VIEW_H / AI_H))
            view.draw_rectangle(vx, vy, vw, vh,
                                color=(0, 255, 0), thickness=2)
            if 0 <= class_id <= 9:
                draw_digit(view, vx + 3, max(2, vy + 3),
                           class_id, (255, 255, 0))
    if detected:
        digit, confidence, _, _ = detected
        draw_digit(view, 150, 12, digit, (255, 255, 0))
    else:
        view.draw_rectangle(1, 1, VIEW_W - 2, VIEW_H - 2,
                            color=(255, 0, 0), thickness=3)


# ======================== 检测参数 ========================

# 黑框在320x240通道快速检测，再映射到960x720坐标。
TARGET_FRAME_ROI = (4, 4, VIEW_W - 8, VIEW_H - 8)
TARGET_FRAME_THRESHOLD = 10000
TARGET_FRAME_MIN_AREA = int(VIEW_W * VIEW_H * 0.08)
TARGET_FRAME_MAX_AREA = int(VIEW_W * VIEW_H * 0.92)
TARGET_FRAME_ASPECT_MIN = 1.12
TARGET_FRAME_ASPECT_MAX = 1.85
TARGET_FRAME_BLACK_MAX = 115
TARGET_FRAME_BLACK_SPREAD = 45
TARGET_FRAME_BLACK_MIN_RATIO = 0.45

FRAME_SEARCH_INTERVAL = 1     # 未锁定时每帧搜索
FRAME_RECHECK_INTERVAL = 2    # 每2帧更新黑框，降低靶心跟随延迟
FRAME_KEEP_MISSES = 1

# 精简的部分目标识别：使用老固件已验证过的 find_blobs，不使用LSD线段检测。
PARTIAL_WHITE_LAB = (60, 100, -25, 25, -25, 25)
PARTIAL_MIN_PIXELS = 500
PARTIAL_MIN_AREA = 1000
PARTIAL_EDGE_MARGIN = 6
PARTIAL_GUIDE_OFFSET = 36
PARTIAL_RECHECK_INTERVAL = 2
PARTIAL_KEEP_MISSES = 1

IMG_CX = AIM_W // 2
IMG_CY = AIM_H // 2

frame_cache = None
frame_scan_wait = 0
frame_misses = 0
partial_cache = None
partial_scan_wait = 0
partial_misses = 0
partial_error_reported = 0

# ======================== Task5 任意角度黑色矩形跟踪 ========================

INNER_BLACK_LAB = (0, 62, -35, 35, -35, 35)
INNER_RECT_FIND_THRESHOLD = 5000
INNER_RECT_MIN_PIXELS = 120
INNER_RECT_MIN_AREA = 350
# 必须小于外侧靶框面积，避免把A4黑框当成Task5目标。
INNER_RECT_MAX_AREA = int(VIEW_W * VIEW_H * 0.12)
# 黑板有安装孔和强反光，不能再按“纯黑实心块”要求高填充率。
INNER_RECT_MIN_DENSITY = 0.18
INNER_RECT_DARK_MAX = 150
INNER_RECT_MIN_DARK_RATIO = 0.40
TASK5_REACH_THRESHOLD = 12       # 960x720逻辑坐标，约等于显示通道4像素
TASK5_REACH_FRAMES = 2
# 实际描绘轨迹必须位于黑纸外侧。这里是320x240检测画面中的外扩距离；
# LCD调试图使用的像素外扩；最终控制轨迹改为靶面厘米坐标外扩。
TASK5_OUTSET_PX = 14
TASK5_FRAME_WIDTH_CM = 29.7
TASK5_FRAME_HEIGHT_CM = 21.0
TASK5_OUTSET_CM = 1.0
TASK5_CM_TX_SCALE = 100
# 角点跨帧低通滤波：越小越稳但跟随越慢，建议范围0.25~0.45。
TASK5_SMOOTH_ALPHA = 0.30
TASK5_SMOOTH_ALPHA_MOVING = 0.60
TASK5_MOVING_THRESHOLD_PX = 4.0
# 连续漏检超过该值才按真正丢失处理；短暂漏检由STM32的C2去抖跨过。
TASK5_FILTER_KEEP_MISSES = 2
# 与上一帧相差过大的候选直接拒绝，防止运动模糊时跳到圆环/外框。
TASK5_MAX_CENTER_JUMP_PX = 28
TASK5_MIN_AREA_RATIO = 0.60
TASK5_MAX_AREA_RATIO = 1.65
# 丢失后必须连续识别多帧才恢复描边，恢复期间保持关光。
TASK5_REACQUIRE_FRAMES = 3
# 新方案：云台静止时连续识别这些帧后冻结四角，绘画期间不再边动边识别。
TASK5_FREEZE_FRAMES = 6
TASK5_AIM_THRESHOLD = 12
TASK5_AIM_FRAMES = 3

# 左上角起步，顺时针沿四边回到左上角。每边细分5段，
# 减小单次目标跳变，降低PID突然加速和转角过冲。
TASK5_WAYPOINTS = (
    (0.0, 0.0), (0.2, 0.0), (0.4, 0.0),
    (0.6, 0.0), (0.8, 0.0), (1.0, 0.0),
    (1.0, 0.2), (1.0, 0.4), (1.0, 0.6),
    (1.0, 0.8), (1.0, 1.0),
    (0.8, 1.0), (0.6, 1.0), (0.4, 1.0),
    (0.2, 1.0), (0.0, 1.0),
    (0.0, 0.8), (0.0, 0.6), (0.0, 0.4),
    (0.0, 0.2), (0.0, 0.0),
)
task5_waypoint_index = 0
task5_arrival_count = 0
task5_tracing = False
task5_complete = False
task5_filtered_corners = None
task5_filter_misses = 0
task5_valid_count = 0
task5_resume_blank = False
task5_frozen_corners = None
task5_frozen_plane_corners = None
task5_corner_tx_index = 0
task5_aim_count = 0
task5_aim_done = False
task5_area_cm2 = 0.0
task5_long_cm = 0.0
task5_short_cm = 0.0


# ======================== 靶子黑框 ========================

def view_to_cam(point):
    """把320x240检测坐标映射到960x720坐标。"""
    return (point[0] * AIM_W // VIEW_W,
            point[1] * AIM_H // VIEW_H)


def cam_to_view(point):
    """把960x720识别坐标映射到320x240显示通道。"""
    return (point[0] * VIEW_W // AIM_W,
            point[1] * VIEW_H // AIM_H)

def polygon_area(points):
    area2 = 0
    for i in range(len(points)):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % len(points)]
        area2 += x1 * y2 - x2 * y1
    return abs(area2) // 2


def polygon_area_float(points):
    """浮点鞋带公式，Task5靶面坐标下结果单位为cm²。"""
    area2 = 0.0
    for i in range(len(points)):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % len(points)]
        area2 += x1 * y2 - x2 * y1
    return abs(area2) * 0.5


def task5_rectangle_size(points):
    """用两组对边的平均值求黑色矩形长、短边，单位为cm。"""
    lengths = []
    for i in range(4):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % 4]
        dx = x2 - x1
        dy = y2 - y1
        lengths.append((dx * dx + dy * dy) ** 0.5)
    side_a = (lengths[0] + lengths[2]) * 0.5
    side_b = (lengths[1] + lengths[3]) * 0.5
    return max(side_a, side_b), min(side_a, side_b)


def quad_center(corners):
    """返回四边形两条对角线交点。"""
    x1, y1 = corners[0]
    x2, y2 = corners[2]
    x3, y3 = corners[1]
    x4, y4 = corners[3]
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denominator) < 0.001:
        return (sum(p[0] for p in corners) // 4,
                sum(p[1] for p in corners) // 4)
    cross12 = x1 * y2 - y1 * x2
    cross34 = x3 * y4 - y3 * x4
    cx = (cross12 * (x3 - x4) - (x1 - x2) * cross34) / denominator
    cy = (cross12 * (y3 - y4) - (y1 - y2) * cross34) / denominator
    return int(round(cx)), int(round(cy))


def sort_quad_corners(points):
    """把任意顺序的四角统一成：左上、右上、右下、左下。"""
    cx = sum(p[0] for p in points) / 4
    cy = sum(p[1] for p in points) / 4
    ordered = sorted(points,
                     key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    start = min(range(4),
                key=lambda i: ordered[i][0] + ordered[i][1])
    return ordered[start:] + ordered[:start]


def align_quad_corners(points, reference):
    """把新四角与上一帧一一对应，防止起点/方向突然翻转。"""
    best = None
    best_score = None
    forward = list(points)
    backward = list(reversed(points))
    for sequence in (forward, backward):
        for shift in range(4):
            candidate = sequence[shift:] + sequence[:shift]
            score = 0.0
            for i in range(4):
                dx = candidate[i][0] - reference[i][0]
                dy = candidate[i][1] - reference[i][1]
                score += dx * dx + dy * dy
            if best_score is None or score < best_score:
                best_score = score
                best = candidate
    return best


def filter_task5_corners(corners):
    """对Task5四角做跨帧指数滤波，抑制检测像素抖动。"""
    global task5_filtered_corners
    if task5_filtered_corners is None:
        task5_filtered_corners = [
            (float(p[0]), float(p[1])) for p in corners
        ]
    else:
        aligned = align_quad_corners(corners, task5_filtered_corners)
        total_move = 0.0
        for i in range(4):
            dx = aligned[i][0] - task5_filtered_corners[i][0]
            dy = aligned[i][1] - task5_filtered_corners[i][1]
            total_move += math.sqrt(dx * dx + dy * dy)
        mean_move = total_move / 4.0
        # 静止时强滤波；云台运动时提高响应，避免固定alpha造成严重滞后。
        if mean_move > TASK5_MOVING_THRESHOLD_PX:
            alpha = TASK5_SMOOTH_ALPHA_MOVING
        else:
            alpha = TASK5_SMOOTH_ALPHA
        filtered = []
        for i in range(4):
            old_x, old_y = task5_filtered_corners[i]
            new_x, new_y = aligned[i]
            filtered.append((
                old_x + alpha * (new_x - old_x),
                old_y + alpha * (new_y - old_y)
            ))
        task5_filtered_corners = filtered
    return [(int(round(p[0])), int(round(p[1])))
            for p in task5_filtered_corners]


def select_task5_candidate(candidates):
    """结合上一帧位置/面积选择候选，并拒绝运动模糊造成的大跳变。"""
    if not candidates:
        return None
    if task5_filtered_corners is None:
        return max(candidates, key=lambda item: item[0])[1]

    reference = [(p[0], p[1]) for p in task5_filtered_corners]
    ref_center_x = sum(p[0] for p in reference) / 4.0
    ref_center_y = sum(p[1] for p in reference) / 4.0
    ref_area = max(1.0, float(polygon_area(reference)))
    best = None
    best_score = None

    for visual_score, raw_corners in candidates:
        corners = align_quad_corners(raw_corners, reference)
        center_x = sum(p[0] for p in corners) / 4.0
        center_y = sum(p[1] for p in corners) / 4.0
        dx = center_x - ref_center_x
        dy = center_y - ref_center_y
        center_jump = math.sqrt(dx * dx + dy * dy)
        area_ratio = float(polygon_area(corners)) / ref_area
        if center_jump > TASK5_MAX_CENTER_JUMP_PX:
            continue
        if area_ratio < TASK5_MIN_AREA_RATIO or \
           area_ratio > TASK5_MAX_AREA_RATIO:
            continue
        continuity_penalty = 1.0 + center_jump * 0.20 + \
                             abs(area_ratio - 1.0) * 8.0
        score = visual_score / continuity_penalty
        if best_score is None or score > best_score:
            best_score = score
            best = corners
    return best


def quad_point_float(corners, u, v):
    """四边形双线性插值，保留浮点精度。"""
    tl, tr, br, bl = corners
    top_x = tl[0] + (tr[0] - tl[0]) * u
    top_y = tl[1] + (tr[1] - tl[1]) * u
    bottom_x = bl[0] + (br[0] - bl[0]) * u
    bottom_y = bl[1] + (br[1] - bl[1]) * u
    return (top_x + (bottom_x - top_x) * v,
            top_y + (bottom_y - top_y) * v)


def quad_point(corners, u, v):
    """在旋转四边形内双线性插值；四角顺序为TL、TR、BR、BL。"""
    x, y = quad_point_float(corners, u, v)
    return int(round(x)), int(round(y))


def inverse_quad_point(corners, point):
    """由图像点反求它在外框中的(u,v)，用于消除靶面透视。"""
    u, v = 0.5, 0.5
    for _ in range(8):
        x, y = quad_point_float(corners, u, v)
        tl, tr, br, bl = corners
        du_x = (tr[0] - tl[0]) * (1.0 - v) + \
               (br[0] - bl[0]) * v
        du_y = (tr[1] - tl[1]) * (1.0 - v) + \
               (br[1] - bl[1]) * v
        dv_x = (bl[0] - tl[0]) * (1.0 - u) + \
               (br[0] - tr[0]) * u
        dv_y = (bl[1] - tl[1]) * (1.0 - u) + \
               (br[1] - tr[1]) * u
        det = du_x * dv_y - du_y * dv_x
        if abs(det) < 0.0001:
            break
        fx = x - point[0]
        fy = y - point[1]
        u -= (fx * dv_y - fy * dv_x) / det
        v -= (du_x * fy - du_y * fx) / det
    return u, v


def view_to_target_cm(point, frame_corners):
    u, v = inverse_quad_point(frame_corners, point)
    return ((u - 0.5) * TASK5_FRAME_WIDTH_CM,
            (v - 0.5) * TASK5_FRAME_HEIGHT_CM)


def target_cm_to_view(point, frame_corners):
    u = point[0] / TASK5_FRAME_WIDTH_CM + 0.5
    v = point[1] / TASK5_FRAME_HEIGHT_CM + 0.5
    return quad_point(frame_corners, u, v)


def expand_quad_plane(corners, distance):
    """在靶面厘米坐标中沿四条边外法线精确外扩。"""
    area2 = 0.0
    unit_normals = []
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % 4]
        area2 += x1 * y2 - x2 * y1
        dx, dy = x2 - x1, y2 - y1
        length = math.sqrt(dx * dx + dy * dy)
        if length < 0.001:
            return list(corners)
        unit_normals.append((dx, dy, length))
    positive = area2 > 0
    normals = []
    for dx, dy, length in unit_normals:
        if positive:
            normals.append((dy / length, -dx / length))
        else:
            normals.append((-dy / length, dx / length))
    expanded = []
    for i in range(4):
        pnx, pny = normals[(i - 1) % 4]
        nnx, nny = normals[i]
        denominator = max(0.15, 1.0 + pnx * nnx + pny * nny)
        expanded.append((
            corners[i][0] + distance * (pnx + nnx) / denominator,
            corners[i][1] + distance * (pny + nny) / denominator))
    return expanded


def expand_quad(corners, distance):
    """把凸四边形四条边向外平移distance像素，返回扩张后的四角。

    不是简单放大水平外接框；每个角由相邻两条边的外法线共同确定，
    因此黑纸任意旋转时，轨迹仍与真实四边平行并位于其外侧。
    """
    area2 = 0.0
    normals = []
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % 4]
        area2 += x1 * y2 - x2 * y1
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx * dx + dy * dy)
        if length < 0.001:
            return list(corners)
        normals.append((dx, dy, length))

    # 图像坐标Y轴向下：正有向面积的连续角点使用左法线作为外侧。
    positive = area2 > 0
    unit_normals = []
    for dx, dy, length in normals:
        if positive:
            nx, ny = dy / length, -dx / length
        else:
            nx, ny = -dy / length, dx / length
        unit_normals.append((nx, ny))

    expanded = []
    for i in range(4):
        prev_nx, prev_ny = unit_normals[(i - 1) % 4]
        next_nx, next_ny = unit_normals[i]
        denominator = 1.0 + prev_nx * next_nx + prev_ny * next_ny
        if denominator < 0.15:
            denominator = 0.15
        offset_x = distance * (prev_nx + next_nx) / denominator
        offset_y = distance * (prev_ny + next_ny) / denominator
        x = int(round(corners[i][0] + offset_x))
        y = int(round(corners[i][1] + offset_y))
        # 目标点不能跑出摄像头画面，否则PID永远无法把它移到中心。
        x = max(2, min(VIEW_W - 3, x))
        y = max(2, min(VIEW_H - 3, y))
        expanded.append((x, y))
    return expanded


def pixel_is_dark(pixel):
    if isinstance(pixel, tuple):
        return sum(pixel) / max(1, len(pixel)) <= INNER_RECT_DARK_MAX
    return pixel <= INNER_RECT_DARK_MAX


def quad_dark_ratio(img, corners):
    """避开边缘，在矩形内部采样；允许反光孔占掉部分黑色像素。"""
    dark = 0
    total = 0
    for v in (0.25, 0.5, 0.75):
        for u in (0.25, 0.5, 0.75):
            x, y = quad_point(corners, u, v)
            if 0 <= x < VIEW_W and 0 <= y < VIEW_H:
                total += 1
                if pixel_is_dark(img.get_pixel(x, y)):
                    dark += 1
    return dark / max(1, total)


def draw_quad(img, corners, color, thickness=3):
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % 4]
        img.draw_line(x1, y1, x2, y2, color=color, thickness=thickness)


def is_neutral_black(pixel):
    if isinstance(pixel, tuple):
        high = max(pixel)
        return high <= TARGET_FRAME_BLACK_MAX and \
               high - min(pixel) <= TARGET_FRAME_BLACK_SPREAD
    return pixel <= TARGET_FRAME_BLACK_MAX


def edge_black_ratio(img, corners):
    black_count = 0
    sample_count = 0
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % 4]
        for n in range(1, 9):
            x = int(round(x1 + (x2 - x1) * n / 9))
            y = int(round(y1 + (y2 - y1) * n / 9))
            found = False
            for oy in (-2, 0, 2):
                py = max(0, min(VIEW_H - 1, y + oy))
                for ox in (-2, 0, 2):
                    px = max(0, min(VIEW_W - 1, x + ox))
                    if is_neutral_black(img.get_pixel(px, py)):
                        found = True
                        break
                if found:
                    break
            black_count += 1 if found else 0
            sample_count += 1
    return black_count / max(1, sample_count)


def detect_target_frame(img):
    global frame_cache, frame_scan_wait, frame_misses

    if frame_scan_wait > 0:
        frame_scan_wait -= 1
        return frame_cache

    candidates = []
    for rect in img.find_rects(roi=TARGET_FRAME_ROI,
                               threshold=TARGET_FRAME_THRESHOLD):
        corners = list(rect.corners())
        area = polygon_area(corners)
        if not (TARGET_FRAME_MIN_AREA <= area <= TARGET_FRAME_MAX_AREA):
            continue

        sides = []
        for i in range(4):
            p1 = corners[i]
            p2 = corners[(i + 1) % 4]
            sides.append(math.sqrt((p2[0] - p1[0]) ** 2 +
                                   (p2[1] - p1[1]) ** 2))
        side_a = (sides[0] + sides[2]) / 2
        side_b = (sides[1] + sides[3]) / 2
        aspect = max(side_a, side_b) / max(1.0, min(side_a, side_b))
        if not (TARGET_FRAME_ASPECT_MIN <= aspect <= TARGET_FRAME_ASPECT_MAX):
            continue

        black_ratio = edge_black_ratio(img, corners)
        if black_ratio >= TARGET_FRAME_BLACK_MIN_RATIO:
            score = area * black_ratio / (1.0 + abs(aspect - 1.414) * 3.0)
            candidates.append((score, corners))

    if candidates:
        small_corners = max(candidates, key=lambda item: item[0])[1]
        frame_cache = [view_to_cam(p) for p in small_corners]
        frame_misses = 0
        frame_scan_wait = FRAME_RECHECK_INTERVAL - 1
        return frame_cache

    frame_misses += 1
    if frame_cache and frame_misses <= FRAME_KEEP_MISSES:
        frame_scan_wait = FRAME_RECHECK_INTERVAL - 1
        return frame_cache
    frame_cache = None
    frame_scan_wait = FRAME_SEARCH_INTERVAL - 1
    return None


# ======================== 精简部分目标引导 ========================

def detect_partial_paper(img):
    """完整黑框失败时，用最大白纸区域给云台提供粗搜索方向。

    该点只是 Z=2 引导点，STM32 不会用它判定已经对准。
    """
    global partial_cache, partial_scan_wait, partial_misses
    global partial_error_reported

    if partial_scan_wait > 0:
        partial_scan_wait -= 1
        return partial_cache

    try:
        blobs = img.find_blobs([PARTIAL_WHITE_LAB],
                               roi=TARGET_FRAME_ROI,
                               pixels_threshold=PARTIAL_MIN_PIXELS,
                               area_threshold=PARTIAL_MIN_AREA,
                               merge=True, margin=8)
    except Exception as exc:
        if not partial_error_reported:
            print("[PARTIAL ERROR] {}".format(exc))
            partial_error_reported = 1
        partial_cache = None
        partial_scan_wait = PARTIAL_RECHECK_INTERVAL - 1
        return None

    if blobs:
        paper = max(blobs, key=lambda blob: blob.pixels())
        left = paper.x() <= PARTIAL_EDGE_MARGIN
        right = paper.x() + paper.w() >= VIEW_W - PARTIAL_EDGE_MARGIN
        top = paper.y() <= PARTIAL_EDGE_MARGIN
        bottom = paper.y() + paper.h() >= VIEW_H - PARTIAL_EDGE_MARGIN

        # 白纸在哪个画面边缘被截断，真实靶心通常就在对应方向之外。
        if left and not right:
            guide_x = VIEW_W // 2 - PARTIAL_GUIDE_OFFSET
        elif right and not left:
            guide_x = VIEW_W // 2 + PARTIAL_GUIDE_OFFSET
        else:
            guide_x = paper.cx()

        if top and not bottom:
            guide_y = VIEW_H // 2 - PARTIAL_GUIDE_OFFSET
        elif bottom and not top:
            guide_y = VIEW_H // 2 + PARTIAL_GUIDE_OFFSET
        else:
            guide_y = paper.cy()

        guide_x = max(0, min(VIEW_W - 1, guide_x))
        guide_y = max(0, min(VIEW_H - 1, guide_y))
        partial_cache = (view_to_cam((guide_x, guide_y)), paper.rect())
        partial_misses = 0
        partial_scan_wait = PARTIAL_RECHECK_INTERVAL - 1
        return partial_cache

    partial_misses += 1
    if partial_cache and partial_misses <= PARTIAL_KEEP_MISSES:
        partial_scan_wait = PARTIAL_RECHECK_INTERVAL - 1
        return partial_cache
    partial_cache = None
    partial_scan_wait = PARTIAL_RECHECK_INTERVAL - 1
    return None


def detect_inner_black_rect(img):
    """检测任意旋转角度的黑色矩形，返回TL/TR/BR/BL四个真实角点。"""
    candidates = []
    try:
        rects = img.find_rects(roi=TARGET_FRAME_ROI,
                               threshold=INNER_RECT_FIND_THRESHOLD)
        for rect in rects:
            corners = sort_quad_corners(list(rect.corners()))
            area = polygon_area(corners)
            if area < INNER_RECT_MIN_AREA or area > INNER_RECT_MAX_AREA:
                continue

            side_lengths = []
            for i in range(4):
                dx = corners[(i + 1) % 4][0] - corners[i][0]
                dy = corners[(i + 1) % 4][1] - corners[i][1]
                side_lengths.append(math.sqrt(dx * dx + dy * dy))
            long_side = max(side_lengths)
            short_side = min(side_lengths)
            if short_side < 10 or long_side / max(1, short_side) > 4.8:
                continue

            dark_ratio = quad_dark_ratio(img, corners)
            if dark_ratio < INNER_RECT_MIN_DARK_RATIO:
                continue
            candidates.append((area * (0.5 + dark_ratio), corners))
    except Exception as exc:
        print("[TASK5 RECT ERROR] {}".format(exc))

    selected = select_task5_candidate(candidates)
    if selected is not None:
        return selected

    # 反光过强导致矩形边缘不连续时，以放宽后的黑色连通域兜底。
    try:
        blobs = img.find_blobs([INNER_BLACK_LAB],
                               roi=TARGET_FRAME_ROI,
                               pixels_threshold=INNER_RECT_MIN_PIXELS,
                               area_threshold=INNER_RECT_MIN_AREA,
                               merge=True, margin=8)
    except Exception as exc:
        print("[TASK5 DETECT ERROR] {}".format(exc))
        return None

    candidates = []
    for blob in blobs:
        w = blob.w()
        h = blob.h()
        area = w * h
        if area < INNER_RECT_MIN_AREA or area > INNER_RECT_MAX_AREA:
            continue
        if w < 12 or h < 12:
            continue
        # 这里只是兜底路径；安装孔和反光会明显降低填充率。
        density = blob.pixels() / max(1, area)
        if density < INNER_RECT_MIN_DENSITY:
            continue
        aspect = w / max(1, h)
        if aspect < 0.22 or aspect > 4.8:
            continue
        # min_corners()给出连通域的最小旋转外接矩形，兜底时也不能退回
        # 水平外接框，否则目标一旋转，激光路径就会离开真实边缘。
        try:
            corners = sort_quad_corners(list(blob.min_corners()))
        except Exception:
            continue
        rotated_area = polygon_area(corners)
        if rotated_area < INNER_RECT_MIN_AREA or \
           rotated_area > INNER_RECT_MAX_AREA:
            continue
        if quad_dark_ratio(img, corners) < INNER_RECT_MIN_DARK_RATIO:
            continue
        candidates.append((blob.pixels() * density, corners))

    return select_task5_candidate(candidates)


def task5_target_from_rect(trace_corners):
    """沿黑纸外侧扩张轨迹计算航点，并映射到960x720坐标。"""
    u, v = TASK5_WAYPOINTS[task5_waypoint_index]
    point = quad_point(trace_corners, u, v)
    return view_to_cam(point), point

# ======================== 轻量误差显示 ========================

SEVEN_SEGMENTS = (
    (1, 1, 1, 1, 1, 1, 0), (0, 1, 1, 0, 0, 0, 0),
    (1, 1, 0, 1, 1, 0, 1), (1, 1, 1, 1, 0, 0, 1),
    (0, 1, 1, 0, 0, 1, 1), (1, 0, 1, 1, 0, 1, 1),
    (1, 0, 1, 1, 1, 1, 1), (1, 1, 1, 0, 0, 0, 0),
    (1, 1, 1, 1, 1, 1, 1), (1, 1, 1, 1, 0, 1, 1),
)


def draw_digit(img, x, y, digit, color):
    w, mid, bottom = 10, 10, 20
    lines = ((x + 2, y, x + 8, y), (x + w, y + 2, x + w, y + 8),
             (x + w, y + 12, x + w, y + 18),
             (x + 2, y + bottom, x + 8, y + bottom),
             (x, y + 12, x, y + 18), (x, y + 2, x, y + 8),
             (x + 2, y + mid, x + 8, y + mid))
    for i in range(7):
        if SEVEN_SEGMENTS[digit][i]:
            line = lines[i]
            img.draw_line(line[0], line[1], line[2], line[3],
                          color=color, thickness=2)


def draw_value(img, x, y, axis, value, color):
    if axis == 'X':
        img.draw_line(x, y, x + 8, y + 20, color=color, thickness=2)
        img.draw_line(x + 8, y, x, y + 20, color=color, thickness=2)
    else:
        img.draw_line(x, y, x + 4, y + 9, color=color, thickness=2)
        img.draw_line(x + 8, y, x + 4, y + 9, color=color, thickness=2)
        img.draw_line(x + 4, y + 9, x + 4, y + 20,
                      color=color, thickness=2)
    x += 14
    img.draw_line(x, y + 10, x + 8, y + 10, color=color, thickness=2)
    if value >= 0:
        img.draw_line(x + 4, y + 6, x + 4, y + 14,
                      color=color, thickness=2)
    x += 14
    value = max(-999, min(999, int(value)))
    for ch in str(abs(value)):
        draw_digit(img, x, y, int(ch), color)
        x += 14


def draw_task5_value(img, x, y, label, value, color):
    """七段数码形式显示A/L/W及一位小数。"""
    # 字母
    if label == 'A':
        img.draw_line(x, y + 20, x + 5, y, color=color, thickness=2)
        img.draw_line(x + 5, y, x + 10, y + 20, color=color, thickness=2)
        img.draw_line(x + 2, y + 11, x + 8, y + 11,
                      color=color, thickness=2)
    elif label == 'L':
        img.draw_line(x, y, x, y + 20, color=color, thickness=2)
        img.draw_line(x, y + 20, x + 10, y + 20,
                      color=color, thickness=2)
    else:  # W
        img.draw_line(x, y, x + 2, y + 20, color=color, thickness=2)
        img.draw_line(x + 2, y + 20, x + 5, y + 11,
                      color=color, thickness=2)
        img.draw_line(x + 5, y + 11, x + 8, y + 20,
                      color=color, thickness=2)
        img.draw_line(x + 8, y + 20, x + 10, y,
                      color=color, thickness=2)
    x += 15
    img.draw_line(x, y + 7, x + 8, y + 7, color=color, thickness=2)
    img.draw_line(x, y + 13, x + 8, y + 13, color=color, thickness=2)
    x += 13
    value10 = max(0, min(9999, int(value * 10.0 + 0.5)))
    integer_part = value10 // 10
    decimal_part = value10 % 10
    for ch in str(integer_part):
        draw_digit(img, x, y, int(ch), color)
        x += 14
    img.draw_rectangle(x, y + 18, 3, 3, color=color, fill=True)
    x += 7
    draw_digit(img, x, y, decimal_part, color)


def draw_task5_measurements(img, area_cm2, long_cm, short_cm):
    """左上角显示面积、长边和短边：A(cm²)、L/W(cm)。"""
    color = (255, 255, 255)
    img.draw_rectangle(2, 2, 112, 79, color=(0, 0, 0), fill=True)
    draw_task5_value(img, 5, 5, 'A', area_cm2, color)
    draw_task5_value(img, 5, 31, 'L', long_cm, color)
    draw_task5_value(img, 5, 57, 'W', short_cm, color)


def draw_frame(img, corners):
    for i in range(4):
        p1 = cam_to_view(corners[i])
        p2 = cam_to_view(corners[(i + 1) % 4])
        img.draw_line(p1[0], p1[1], p2[0], p2[1],
                      color=(0, 255, 255), thickness=3)


# ======================== 主循环 ========================

print("=== K230 A4 Aim Start ===")
print("[UART3] TX=GPIO{} RX=GPIO{}".format(uart_tx_pin, uart_rx_pin))
uart.write("X0Y0Z0E\r\n")

try:
    while True:
        os.exitpoint()
        update_task_mode()

        if task_mode == 0:
            # 菜单空闲模式：停止AI推理并明确告诉STM32当前没有视觉目标。
            uart.write("X0Y0Z0D-1E\r\n")
            time.sleep_ms(30)
            continue

        if task_mode == 5:
            view = sensor.snapshot(chn=CAM_CHN_ID_1)
            raw_corners = None

            if not task5_aim_done:
                # 第一阶段：关激光瞄准A4外框中心，云台停止后再标定黑纸。
                aim_frame = detect_target_frame(view)
                if aim_frame is None:
                    task5_aim_count = 0
                    uart.write("X0Y0Z0C2E\r\n")
                    view.draw_rectangle(1, 1, VIEW_W - 2, VIEW_H - 2,
                                        color=(255, 0, 0), thickness=3)
                else:
                    aim_x = int(round(sum(p[0] for p in aim_frame) / 4))
                    aim_y = int(round(sum(p[1] for p in aim_frame) / 4))
                    err_x = aim_x - IMG_CX
                    err_y = aim_y - IMG_CY
                    uart.write("X{}Y{}Z1C3E\r\n".format(err_x, err_y))
                    if abs(err_x) <= TASK5_AIM_THRESHOLD and \
                       abs(err_y) <= TASK5_AIM_THRESHOLD:
                        task5_aim_count += 1
                    else:
                        task5_aim_count = 0
                    draw_frame(view, aim_frame)
                    view.draw_cross(cam_to_view((aim_x, aim_y))[0],
                                    cam_to_view((aim_x, aim_y))[1],
                                    color=(255, 0, 0), size=10, thickness=2)
                    if task5_aim_count >= TASK5_AIM_FRAMES:
                        task5_aim_done = True
                        uart.write("X0Y0Z1C8E\r\n")  # C8=靶心瞄准完成
                        task5_valid_count = 0
                        task5_filtered_corners = None
                        frame_cache = None
                        frame_scan_wait = 0
                        frame_misses = 0

                Display.show_image(view, x=DISPLAY_X, y=DISPLAY_Y)
                gc.collect()
                continue

            if task5_frozen_corners is None:
                # 同时识别A4外框和内部黑纸，用外框建立真实厘米坐标系。
                raw_corners = detect_inner_black_rect(view)
                frame_cam = detect_target_frame(view)
                frame_view = None
                if frame_cam is not None:
                    frame_view = sort_quad_corners(
                        [cam_to_view(p) for p in frame_cam])
                uart.write("X0Y0Z0C2E\r\n")
                if raw_corners is None or frame_view is None:
                    task5_valid_count = 0
                    task5_filter_misses += 1
                    if task5_filter_misses > TASK5_FILTER_KEEP_MISSES:
                        task5_filtered_corners = None
                    view.draw_rectangle(1, 1, VIEW_W - 2, VIEW_H - 2,
                                        color=(255, 0, 0), thickness=3)
                else:
                    task5_filter_misses = 0
                    corners = filter_task5_corners(raw_corners)
                    task5_valid_count += 1
                    plane_corners_abs = [
                        view_to_target_cm(p, frame_view) for p in corners
                    ]
                    task5_long_cm, task5_short_cm = task5_rectangle_size(
                        plane_corners_abs)
                    # 题目要求矩形面积直接按“长×宽”计算。
                    task5_area_cm2 = task5_long_cm * task5_short_cm
                    trace_plane_abs = expand_quad_plane(
                        plane_corners_abs, TASK5_OUTSET_CM)
                    reference_cm = view_to_target_cm(
                        (VIEW_W * 0.5, VIEW_H * 0.5), frame_view)
                    # STM32开环零点是冻结瞬间的相机朝向，因此发送相对
                    # 当前画面中心的厘米位移，而不是相对A4中心的绝对坐标。
                    trace_plane = [
                        (p[0] - reference_cm[0], p[1] - reference_cm[1])
                        for p in trace_plane_abs
                    ]
                    trace_corners = [
                        target_cm_to_view(p, frame_view)
                        for p in trace_plane_abs
                    ]
                    draw_quad(view, frame_view, (0, 0, 255), thickness=2)
                    draw_quad(view, raw_corners, (0, 128, 0), thickness=1)
                    draw_quad(view, corners, (255, 255, 0), thickness=2)
                    draw_quad(view, trace_corners, (0, 255, 255), thickness=3)
                    draw_task5_measurements(
                        view, task5_area_cm2,
                        task5_long_cm, task5_short_cm)
                    if task5_valid_count >= TASK5_FREEZE_FRAMES:
                        # 同时冻结LCD图像角点和靶面厘米角点。
                        task5_frozen_corners = [
                            (int(p[0]), int(p[1])) for p in trace_corners
                        ]
                        task5_frozen_plane_corners = [
                            (float(p[0]), float(p[1])) for p in trace_plane
                        ]
                        task5_corner_tx_index = 0
            else:
                # C4~C7发送TL/TR/BR/BL的靶面厘米坐标×100。
                corner = task5_frozen_corners[task5_corner_tx_index]
                plane_corner = task5_frozen_plane_corners[
                    task5_corner_tx_index]
                err_x = int(round(plane_corner[0] * TASK5_CM_TX_SCALE))
                err_y = int(round(plane_corner[1] * TASK5_CM_TX_SCALE))
                corner_status = 4 + task5_corner_tx_index
                uart.write("X{}Y{}Z1C{}E\r\n".format(
                    err_x, err_y, corner_status))
                task5_corner_tx_index = (task5_corner_tx_index + 1) % 4

                draw_quad(view, task5_frozen_corners,
                          (0, 255, 255), thickness=3)
                view.draw_cross(corner[0], corner[1],
                                color=(255, 0, 0), size=8, thickness=2)
                draw_task5_measurements(
                    view, task5_area_cm2,
                    task5_long_cm, task5_short_cm)

            Display.show_image(view, x=DISPLAY_X, y=DISPLAY_Y)
            now = time.ticks_ms()
            if time.ticks_diff(now, last_tx_log_tick) >= 500:
                print("[TASK5] frozen={} valid={} tx_corner={}".format(
                    task5_frozen_corners is not None,
                    task5_valid_count, task5_corner_tx_index))
                last_tx_log_tick = now
            gc.collect()
            continue

        if task_mode == 6:
            try:
                ai_img = sensor.snapshot(chn=CAM_CHN_ID_0)
                results = digit_yolo.run(ai_img.to_numpy_ref())
                detected = best_digit_result(results)
                confirmed_digit = update_digit_stability(detected)

                # Task6仍检测A4黑框，STM32先瞄准靶心，再开始开环书写。
                view = sensor.snapshot(chn=CAM_CHN_ID_1)
                frame = detect_target_frame(view)
                if frame:
                    center = quad_center(frame)
                    draw_frame(view, frame)
                    target_status = 1
                else:
                    partial = detect_partial_paper(view)
                    if partial:
                        center, partial_rect = partial
                        view.draw_rectangle(partial_rect[0], partial_rect[1],
                                            partial_rect[2], partial_rect[3],
                                            color=(255, 128, 0), thickness=2)
                        target_status = 2
                    else:
                        center = (IMG_CX, IMG_CY)
                        target_status = 0
                err_x = center[0] - IMG_CX
                err_y = center[1] - IMG_CY

                if confirmed_digit >= 0:
                    packet = "X{}Y{}Z{}D{}E\r\n".format(
                        err_x, err_y, target_status, confirmed_digit)
                else:
                    packet = "X{}Y{}Z{}D-1E\r\n".format(
                        err_x, err_y, target_status)
                uart.write(packet)

                # 任务6也必须刷新LCD，否则画面会停在切换前最后一帧。
                draw_digit_results(view, results, detected)
                Display.show_image(view, x=DISPLAY_X, y=DISPLAY_Y)

                now = time.ticks_ms()
                if time.ticks_diff(now, last_tx_log_tick) >= 500:
                    if confirmed_digit >= 0:
                        digit, confidence, _, _ = detected
                        print("[DIGIT] {} conf={:.2f} {}".format(
                            digit, confidence, packet.strip()))
                    else:
                        print("[DIGIT] searching... mem={}".format(
                            gc.mem_free()))
                    last_tx_log_tick = now

                del ai_img
                del results
                gc.collect()
            except Exception as exc:
                uart.write("X0Y0Z0D-1E\r\n")
                print("[TASK6 ERROR] {}".format(exc))
                try:
                    sys.print_exception(exc)
                except Exception:
                    pass
                gc.collect()
                time.sleep_ms(100)
            continue

        view = sensor.snapshot(chn=CAM_CHN_ID_1)

        frame = detect_target_frame(view)

        if frame:
            center = quad_center(frame)
            # 靶心只由A4纸外侧黑框四角确定。
            # 不再用红色圆环blob平移黑框，避免把圆环片段误当靶心。
            draw_frame(view, frame)
            target_status = 1
            partial_cache = None
            partial_scan_wait = 0
            partial_misses = 0
        else:
            partial = detect_partial_paper(view)
            if partial:
                center, partial_rect = partial
                view.draw_rectangle(partial_rect[0], partial_rect[1],
                                    partial_rect[2], partial_rect[3],
                                    color=(255, 128, 0), thickness=3)
                target_status = 2
            else:
                center = (IMG_CX, IMG_CY)
                target_status = 0

        center_view = cam_to_view(center)
        view.draw_cross(center_view[0], center_view[1],
                        color=(255, 0, 0), size=10, thickness=2)

        # Z=1为精确靶心，Z=2为粗引导点；统一使用“目标-画面中心”。
        err_x = center[0] - IMG_CX
        err_y = center[1] - IMG_CY
        image_center_view = cam_to_view((IMG_CX, IMG_CY))
        view.draw_cross(image_center_view[0], image_center_view[1],
                        color=(0, 128, 255), size=9, thickness=2)
        view.draw_line(image_center_view[0], image_center_view[1],
                       center_view[0], center_view[1],
                       color=(255, 255, 0), thickness=2)

        # 每轮主循环只发送一帧；发送频率与视觉处理帧率一致。
        packet = "X{}Y{}Z{}E\r\n".format(
            int(err_x), int(err_y), target_status)
        uart.write(packet)
        now = time.ticks_ms()
        if time.ticks_diff(now, last_tx_log_tick) >= 500:
            print("[UART3 TX]", packet.strip())
            last_tx_log_tick = now

        if target_status == 1:
            color = (255, 255, 0)
        elif target_status == 2:
            color = (0, 255, 255)
        else:
            color = (255, 80, 0)
        draw_value(view, 8, 18, 'X', err_x, color)
        draw_value(view, 125, 18, 'Y', err_y, color)
        if target_status == 0:
            view.draw_rectangle(1, 1, VIEW_W - 2, VIEW_H - 2,
                                color=(255, 0, 0), thickness=3)

        Display.show_image(view, x=DISPLAY_X, y=DISPLAY_Y)

except KeyboardInterrupt:
    print("User Stop")
except BaseException as e:
    print("Error: {}".format(e))
finally:
    try:
        digit_yolo.deinit()
    except Exception:
        pass
    try:
        sensor.stop()
    except Exception:
        pass
    try:
        Display.deinit()
    except Exception:
        pass
    try:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
    except Exception:
        pass
    try:
        MediaManager.deinit()
    except Exception:
        pass
    try:
        uart.deinit()
    except Exception:
        pass
    print("=== End ===")
