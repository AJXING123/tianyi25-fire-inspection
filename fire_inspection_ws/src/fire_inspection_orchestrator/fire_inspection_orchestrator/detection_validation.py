"""检测消息的纯 Python 校验，不依赖 ROS。"""

import json
import math


_REQUIRED_FIELDS = ("cls", "conf", "cx", "cy", "w", "h")


def _finite_number(value):
    """只接受有限的数字，不接受 bool、字符串、NaN 或 Infinity。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(value)


def parse_detections(payload):
    """解析并校验检测 JSON。

    返回：
      []：合法的空检测列表；
      list[dict]：合法检测列表；
      None：消息格式错误或存在不合法检测项。

    只做数据结构与数值校验，不负责类别筛选或业务判定。
    """
    if not isinstance(payload, str):
        return None

    try:
        data = json.loads(payload)
    except (json.JSONDecodeError, TypeError, ValueError):
        return None

    if not isinstance(data, list):
        return None

    for detection in data:
        if not isinstance(detection, dict):
            return None

        if any(field not in detection for field in _REQUIRED_FIELDS):
            return None

        cls = detection["cls"]
        if not isinstance(cls, str) or not cls.strip():
            return None

        for field in ("conf", "cx", "cy", "w", "h"):
            if not _finite_number(detection[field]):
                return None

        if not 0.0 <= detection["conf"] <= 1.0:
            return None

        if not 0.0 <= detection["cx"] <= 1.0:
            return None

        if not 0.0 <= detection["cy"] <= 1.0:
            return None

        if not 0.0 <= detection["w"] <= 1.0:
            return None

        if not 0.0 <= detection["h"] <= 1.0:
            return None

    return data
