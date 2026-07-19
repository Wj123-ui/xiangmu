# -*- coding: utf-8 -*-
"""
K230 单功能视觉程序

功能：
  1. 检测 A4 靶子外侧黑框并计算靶心。
  2. 检测红色激光点。
  3. 显示并通过 UART3 发送激光相对靶心的像素误差。

通信：X<dx>Y<dy>Z<found>E\r\n
偏差：dx/dy = 靶心坐标 - 激光点坐标，found=1表示找到靶框。
"""
import time
import os
import math

from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA


# ======================== 图像与显示 ========================

CAM_W, CAM_H = 640, 480       # 通道0：高分辨率识别
VIEW_W, VIEW_H = 320, 240     # 通道1：小尺寸屏幕显示
VIEW_DIV = 2                  # 识别坐标除以2映射到显示坐标
LCD_W, LCD_H = 800, 480
DISPLAY_X = (LCD_W - VIEW_W) // 2
DISPLAY_Y = (LCD_H - VIEW_H) // 2

sensor = Sensor(width=1280, height=960)
sensor.reset()
sensor.set_framesize(chn=CAM_CHN_ID_0, width=CAM_W, height=CAM_H)
sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
sensor.set_framesize(chn=CAM_CHN_ID_1, width=VIEW_W, height=VIEW_H)
sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

Display.init(Display.ST7701, width=LCD_W, height=LCD_H, to_ide=False)
MediaManager.init()
sensor.run()


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

TX_INTERVAL_MS = 20
last_tx_tick = 0
last_tx_log_tick = 0


def send_aim_data(dx=0, dy=0, found=0, force=False):
    global last_tx_tick, last_tx_log_tick
    now = time.ticks_ms()
    if not force and time.ticks_diff(now, last_tx_tick) < TX_INTERVAL_MS:
        return
    # 新协议例：X15Y-8Z1E\r\n
    packet = "X{}Y{}Z{}E\r\n".format(
        int(dx), int(dy), 1 if found else 0)
    uart.write(packet)
    last_tx_tick = now
    if force or time.ticks_diff(now, last_tx_log_tick) >= 500:
        print("[UART3 TX]", packet.strip())
        last_tx_log_tick = now


# ======================== 检测参数 ========================

# 需要根据现场光照在 IDE 阈值编辑器中标定。
# 激光可能因曝光变成红、橙红或带白色高光，因此范围适当放宽。
LASER_LAB = (42, 100, 18, 127, -35, 100)
TARGET_LAB = (25, 60, 20, 127, -20, 40)

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
FRAME_RECHECK_INTERVAL = 5    # 低分辨率通道快速校正黑框
FRAME_KEEP_MISSES = 1

IMG_CX = CAM_W // 2
IMG_CY = CAM_H // 2

frame_cache = None
frame_scan_wait = 0
frame_misses = 0


# ======================== 靶子黑框 ========================

def polygon_area(points):
    area2 = 0
    for i in range(len(points)):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % len(points)]
        area2 += x1 * y2 - x2 * y1
    return abs(area2) // 2


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
        # 黑框在320x240通道检测，立即映射到640x480识别坐标。
        detected = [(p[0] * VIEW_DIV, p[1] * VIEW_DIV)
                    for p in small_corners]
        frame_cache = detected
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


def frame_roi(corners, margin=60):
    x1 = max(0, min(p[0] for p in corners) - margin)
    y1 = max(0, min(p[1] for p in corners) - margin)
    x2 = min(CAM_W, max(p[0] for p in corners) + margin + 1)
    y2 = min(CAM_H, max(p[1] for p in corners) + margin + 1)
    return x1, y1, max(1, x2 - x1), max(1, y2 - y1)


# ======================== 激光与红色圆环 ========================

def detect_laser(img, roi):
    full_roi = (0, 0, CAM_W, CAM_H)
    search_rois = (roi,) if roi == full_roi else (roi, full_roi)

    for search_roi in search_rois:
        blobs = img.find_blobs([LASER_LAB], roi=search_roi,
                               pixels_threshold=3,
                               area_threshold=3, merge=True)
        candidates = [b for b in blobs
                      if 1 <= b.w() <= 70 and 1 <= b.h() <= 70 and
                         3 <= b.pixels() <= 4000]
        if not candidates:
            continue

        # 激光通常比纸上的红色圆环亮；亮度优先，紧凑度作为次要评分。
        def laser_score(blob):
            pixel = img.get_pixel(blob.cx(), blob.cy())
            brightness = max(pixel) if isinstance(pixel, tuple) else pixel
            compact = blob.pixels() * blob.pixels() // max(1, blob.area())
            return brightness * 10000 + compact

        best = max(candidates, key=laser_score)
        return best.cx(), best.cy()

    return None


def detect_red_center(img, laser, roi):
    blobs = img.find_blobs([TARGET_LAB], roi=roi,
                           pixels_threshold=50,
                           area_threshold=50, merge=True)
    if not blobs:
        return None
    if laser:
        filtered = [b for b in blobs
                    if (b.cx() - laser[0]) ** 2 +
                       (b.cy() - laser[1]) ** 2 > 12 ** 2]
        if filtered:
            blobs = filtered
    best = min(blobs,
               key=lambda b: (b.cx() - IMG_CX) ** 2 +
                             (b.cy() - IMG_CY) ** 2)
    return best.cx(), best.cy()


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


def draw_frame(img, corners):
    for i in range(4):
        p1 = corners[i]
        p2 = corners[(i + 1) % 4]
        img.draw_line(p1[0] // VIEW_DIV, p1[1] // VIEW_DIV,
                      p2[0] // VIEW_DIV, p2[1] // VIEW_DIV,
                      color=(0, 255, 255), thickness=3)


# ======================== 主循环 ========================

print("=== K230 A4 Aim Start ===")
print("[UART3] TX=GPIO{} RX=GPIO{}".format(uart_tx_pin, uart_rx_pin))
send_aim_data(0, 0, 0, force=True)

try:
    while True:
        os.exitpoint()
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        view = sensor.snapshot(chn=CAM_CHN_ID_1)

        frame = detect_target_frame(view)
        roi = frame_roi(frame) if frame else (0, 0, CAM_W, CAM_H)
        laser = detect_laser(img, roi)
        red_center = detect_red_center(img, laser, roi)

        if frame:
            center = quad_center(frame)
            if red_center:
                dx = red_center[0] - center[0]
                dy = red_center[1] - center[1]
                if dx * dx + dy * dy <= 100 * 100:
                    for i in range(4):
                        frame[i] = frame[i][0] + dx, frame[i][1] + dy
                    center = red_center
            draw_frame(view, frame)
            frame_ok = True
        else:
            center = red_center if red_center else (IMG_CX, IMG_CY)
            frame_ok = False

        view.draw_cross(center[0] // VIEW_DIV, center[1] // VIEW_DIV,
                        color=(255, 0, 0), size=10, thickness=2)

        if laser:
            # 协议定义：靶心相对激光点的偏差。
            err_x = center[0] - laser[0]
            err_y = center[1] - laser[1]
            send_aim_data(err_x, err_y, 1 if frame_ok else 0)
            view.draw_cross(laser[0] // VIEW_DIV, laser[1] // VIEW_DIV,
                            color=(0, 255, 0), size=9, thickness=2)
            view.draw_line(laser[0] // VIEW_DIV, laser[1] // VIEW_DIV,
                           center[0] // VIEW_DIV, center[1] // VIEW_DIV,
                           color=(255, 255, 0), thickness=2)
        else:
            # 最终回退：把画面中心视为虚拟激光点。
            err_x = center[0] - IMG_CX
            err_y = center[1] - IMG_CY
            send_aim_data(err_x, err_y, 1 if frame_ok else 0)
            view.draw_cross(IMG_CX // VIEW_DIV, IMG_CY // VIEW_DIV,
                            color=(0, 128, 255), size=9, thickness=2)
            view.draw_line(IMG_CX // VIEW_DIV, IMG_CY // VIEW_DIV,
                           center[0] // VIEW_DIV, center[1] // VIEW_DIV,
                           color=(255, 80, 0), thickness=2)

        color = (255, 255, 0) if frame_ok and laser else (255, 80, 0)
        draw_value(view, 8, 18, 'X', err_x, color)
        draw_value(view, 125, 18, 'Y', err_y, color)
        if not frame_ok:
            view.draw_rectangle(1, 1, VIEW_W - 2, VIEW_H - 2,
                                color=(255, 0, 0), thickness=3)

        Display.show_image(view, x=DISPLAY_X, y=DISPLAY_Y)

except KeyboardInterrupt:
    print("User Stop")
except BaseException as e:
    print("Error: {}".format(e))
finally:
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
