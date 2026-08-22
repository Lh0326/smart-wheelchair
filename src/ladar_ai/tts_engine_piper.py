"""Piper TTS 引擎：本地轻量中文 TTS（目标首字延迟 < 400ms）。

通过 sherpa-onnx OfflineTts 的 VITS 后端加载 Piper zh_CN-huayan-medium 模型。
作为 TTSEngine 的主引擎，Kokoro 作为兜底。
"""
import os
import logging
from typing import Tuple

import numpy as np

try:
    import sherpa_onnx
except ImportError:
    sherpa_onnx = None

logger = logging.getLogger(__name__)

_PIPER_SAMPLE_RATE = 22050  # Piper 中文模型固定输出采样率


class PiperTTSEngine:
    """Piper TTS 引擎：加载本地 ONNX 模型，提供同步 generate 接口。"""

    def __init__(self, model_dir: str, num_threads: int = 2):
        self._tts = None
        self._model_dir = model_dir
        self._num_threads = num_threads

        if sherpa_onnx is None:
            logger.error("sherpa_onnx 未安装，Piper 引擎不可用")
            return

        onnx_file = os.path.join(model_dir, "zh_CN-huayan-medium.onnx")
        if not os.path.isfile(onnx_file):
            logger.warning("Piper 模型不存在: %s", onnx_file)
            return

        tokens_file = os.path.join(model_dir, "tokens.txt")
        if not os.path.isfile(tokens_file):
            logger.warning("Piper tokens.txt 不存在: %s", tokens_file)
            return

        espeak_data = os.path.join(model_dir, "espeak-ng-data")
        if not os.path.isdir(espeak_data):
            logger.warning("espeak-ng-data 不存在: %s（Piper 模型目录应自带）", espeak_data)
            espeak_data = ""  # 让 OfflineTts 报错时根因明确

        try:
            cfg = sherpa_onnx.OfflineTtsConfig(
                model=sherpa_onnx.OfflineTtsModelConfig(
                    vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                        model=onnx_file,
                        tokens=tokens_file,
                        data_dir=espeak_data if os.path.isdir(espeak_data) else "",
                    ),
                    num_threads=num_threads,
                    debug=False,
                ),
                max_num_sentences=1,
            )
            self._tts = sherpa_onnx.OfflineTts(cfg)
            logger.info("Piper 引擎加载成功: %s", onnx_file)
        except Exception as e:
            logger.error("Piper 引擎加载失败: %s", e)

    def is_loaded(self) -> bool:
        return self._tts is not None

    def generate(self, text: str, sid: int = 0, speed: float = 1.0) -> Tuple[np.ndarray, int]:
        """同步合成。

        Returns
        -------
        (audio, sample_rate)
            audio: float32 numpy array, 22050 Hz
            sample_rate: 22050
        """
        if self._tts is None or not text or not text.strip():
            return np.array([], dtype=np.float32), _PIPER_SAMPLE_RATE

        try:
            result = self._tts.generate(text, sid=sid, speed=speed)
            audio = np.array(result.samples, dtype=np.float32)
            return audio, result.sample_rate
        except Exception as e:
            logger.error("Piper generate 失败: %s", e)
            return np.array([], dtype=np.float32), _PIPER_SAMPLE_RATE
