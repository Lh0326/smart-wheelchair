"""Ladar-Ai 共享工具函数。"""

# 8 方位名称
ZONE_NAMES = [
    "front_left", "front", "front_right", "right",
    "rear_right", "rear", "rear_left", "left",
]

# 方位中文名
DIRECTION_CN = {
    "front_left": "左前方",
    "front": "正前方",
    "front_right": "右前方",
    "right": "右侧",
    "rear_right": "右后方",
    "rear": "正后方",
    "rear_left": "左后方",
    "left": "左侧",
}

_CN_DIGITS = "零一二三四五六七八九"

# 方位角度边界（度数）
# ROS2 LaserScan: 0°=正前方, 正角度=逆时针=左侧, 负角度=顺时针=右侧
_ZONE_RANGES_DEG = [
    ("front",       337.5,  22.5),
    ("front_left",   22.5,  67.5),   # 逆时针正角度 = 左侧
    ("left",         67.5, 112.5),
    ("rear_left",   112.5, 157.5),
    ("rear",        157.5, 202.5),
    ("rear_right",  202.5, 247.5),
    ("right",       247.5, 292.5),   # 顺时针负角度 = 右侧
    ("front_right", 292.5, 337.5),
]


def num_to_chinese(n: float) -> str:
    """将数字转为中文读法。如 2.5 -> '二点五', 0.2 -> '零点二', 15 -> '十五'"""
    if n == 0:
        return "零"
    integer_part = int(n)
    decimal_part = round(n - integer_part, 2)
    result = ""
    if integer_part >= 10:
        tens = integer_part // 10
        if tens > 1:
            result += _CN_DIGITS[tens]
        result += "十"
        ones = integer_part % 10
        if ones > 0:
            result += _CN_DIGITS[ones]
    elif integer_part > 0:
        result += _CN_DIGITS[integer_part]
    if decimal_part > 0:
        if integer_part == 0:
            result += "零"
        result += "点"
        decimal_str = f"{decimal_part:.2f}".rstrip("0").lstrip("0.")
        for ch in decimal_str:
            if ch.isdigit():
                result += _CN_DIGITS[int(ch)]
    return result


def direction_to_chinese(zone_name: str) -> str:
    """方位英文名转中文。"""
    return DIRECTION_CN.get(zone_name, zone_name)


def angle_to_zone(angle_rad: float) -> str:
    """将弧度角映射到 8 方位分区名。"""
    import math
    deg = math.degrees(angle_rad) % 360.0
    for name, start, end in _ZONE_RANGES_DEG:
        if start > end:  # 跨 360°
            if deg >= start or deg < end:
                return name
        else:
            if start <= deg < end:
                return name
    return "front"
