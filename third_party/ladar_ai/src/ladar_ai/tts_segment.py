"""TTS 句级切分：按中英文标点把长文本切成短段用于流式合成。
import os as _os
def _find_ws_root():
    r = _os.environ.get("WS_ROOT")
    if r: return r
    d = _os.path.dirname(_os.path.abspath(__file__))
    for _ in range(6):
        if _os.path.exists(_os.path.join(d, "env.sh")): return d
        d = _os.path.dirname(d)
    return d
_WS_ROOT = _find_ws_root()
_MODELS_ROOT = _os.environ.get("MODELS_ROOT", _os.path.join(_WS_ROOT, "models"))


句点 . 不切分小数点（数字之间的 . 保留），避免把"距离 2.5 米"
误切成 ["距离 2", "5 米"]。
"""
import re

# 切分用标点：中文句号/感叹号/问号/逗号/分号 + 英文对应 + 换行 + 句点
_PUNCT_BASE = "。！？!?；;，,"
_SENTENCE_END = _PUNCT_BASE + "\n."

# 切分模式：包含所有标点（含英文句点 .）
_SPLIT_PATTERN = re.compile(rf"[{re.escape(_SENTENCE_END)}]")

# 小数点模式：仅匹配数字之间的 .（前后都是数字）
_DECIMAL_DOT = re.compile(r"(?<=\d)\.(?=\d)")


def segment_text(text: str) -> list:
    """按标点把文本切成短段。

    切分规则：
    - 中文句号/感叹号/问号/逗号/分号 + 英文对应 + 换行：直接切分
    - 英文句点 .：仅当两侧都是数字时不切分（保护小数点如 2.5、v1.0、0.8）
      其他位置的 . 按句末标点切分

    Parameters
    ----------
    text : str
        原始文本。

    Returns
    -------
    list[str]
        切分后的非空短段列表（去掉标点本身，前后空白剥离）。
    """
    if not text:
        return []

    # 先保护小数点：把数字之间的 . 用占位符替换，切分后再还原
    # 占位符选择一个文本中不会出现的字符
    _DECIMAL_PLACEHOLDER = "\x00"
    protected = _DECIMAL_DOT.sub(_DECIMAL_PLACEHOLDER, text)

    # 按所有标点（含句点 .）切分
    raw_parts = _SPLIT_PATTERN.split(protected)

    # 还原小数点 + 剥离空白 + 过滤空段
    return [
        part.replace(_DECIMAL_PLACEHOLDER, ".").strip()
        for part in raw_parts
        if part.strip()
    ]
