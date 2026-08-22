# focus_detector.py
"""专注度检测 pipeline：EMG 屏蔽 → 空间滤波 → 特征 → 分类 → 平滑。

关键修复（评估文档 P0/P1）：
  - EMG 反向门控（F9）：EMG ratio > emg_safe_threshold 时强制 score=50
    + fallback_reason='emg'，不让 SVM 决策。修复"皱眉被误判为专注"反模式。
  - fallback_reason 字段（F8）：UI/日志能区分四种 neutral 来源
    ('emg' / 'low_confidence' / 'severe_artifact' / None)
  - feature_mode 跟随 SVM 模型（F5）：默认 reduced 30 维。
"""
from dataclasses import dataclass, field
from typing import Optional, Dict, List
import numpy as np

from .emg_handler import EMGHandler
from .spatial_filter import CARFilter, LaplacianFilter
from .feature_extractor import FeatureExtractor
from .classifiers import SVMClassifier
from .confidence_smoother import ConfidenceSmoother


@dataclass
class FocusResult:
    """专注度检测结果。

    fallback_reason 区分 4 种 state='neutral' 的来源：
      - None：SVM 正常输出，state 来自 ConfidenceSmoother 滞回
      - 'emg'：EMG ratio 超过 safe-fallback 阈值，强制降级（P0 反作弊）
      - 'low_confidence'：连续 3 个置信度弃权（ConfidenceSmoother）
      - 'severe_artifact'：EMGHandler 标记重度伪迹（沿用上次结果）
    """
    score: float                       # 0-100
    state: str                         # focused/neutral/relaxed
    confidence: float                  # 0-1
    p_focus: float
    p_relax: float
    features: Dict[str, float] = field(default_factory=dict)
    top_contributors: List[tuple] = field(default_factory=list)
    emg_level: float = 0.0
    artifact_rejected: bool = False
    fallback_reason: Optional[str] = None  # P1/F8


class FocusDetector:
    """编排 5 个组件的 pipeline。"""

    def __init__(self, fs: int = 500,
                 model_type: str = 'svm',
                 model_path: Optional[str] = None,
                 use_laplacian: bool = False,
                 baseline_alpha: Optional[np.ndarray] = None,
                 feature_mode: str = 'reduced',
                 emg_safe_threshold: float = float('inf')):
        """
        emg_safe_threshold：超过此 EMG ratio 强制 safe-fallback（neutral）。
          默认 inf（取消 EMG 反向门控，用户要求"取消 EMG 阈值"）。
          想恢复时设回 3.5。
        """
        self.fs = fs
        self.emg_handler = EMGHandler(
            fs=fs,
            light_thresh=1.0,           # 软去除阈值（保留，改进信号质量）
            severe_thresh=float('inf'), # 丢弃阈值：取消（用户要求"取消 EMG 阈值"）
            beta_light_thresh=1.5,      # beta 软去除（保留）
            beta_severe_thresh=float('inf'),  # beta 丢弃：取消
        )
        self.car = CARFilter()
        self.laplacian = LaplacianFilter() if use_laplacian else None
        self.smoother = ConfidenceSmoother()
        self.emg_safe_threshold = emg_safe_threshold

        # 分类器加载
        if model_type == 'svm':
            if model_path:
                self.classifier = SVMClassifier.load(model_path)
                # 训练时持久化的 baseline_alpha 优先；显式传入次之；默认 ones
                effective_baseline = (
                    self.classifier.baseline_alpha
                    if self.classifier.baseline_alpha is not None
                    else baseline_alpha
                )
                # feature_mode 跟随 model（保证训练/推理对齐）
                effective_mode = self.classifier.feature_mode or feature_mode
                # F4: 注入训练时学习的噪声模板到 EMGHandler
                if self.classifier.noise_template is not None:
                    nt = self.classifier.noise_template
                    if isinstance(nt, dict) and nt.get('freqs') is not None:
                        self.emg_handler.set_noise_template(
                            np.asarray(nt['freqs']),
                            np.asarray(nt['mag']),
                        )
            else:
                self.classifier = None  # 校准前
                effective_baseline = baseline_alpha
                effective_mode = feature_mode
        else:
            raise ValueError(f"未知 model_type: {model_type}")

        self.feature_mode = effective_mode
        self.extractor = FeatureExtractor(
            fs=fs, baseline_alpha=effective_baseline, mode=effective_mode,
        )

        self._feature_names = self.extractor.feature_names()
        self._last_result: Optional[FocusResult] = None

    def update(self, window: np.ndarray) -> FocusResult:
        """输入 (n_samples, 8) 窗口，输出 FocusResult。"""
        # 1. EMG 软去除
        cleaned, emg_ratio = self.emg_handler.process(window)
        artifact_rejected = self.emg_handler.last_rejected

        # 1.5 EMG 反向门控（P0/F9）：高 EMG ratio 强制 safe-fallback
        # 不让 SVM 在高 EMG 时输出高 p_focus（避免"皱眉=专注"反模式）
        if emg_ratio >= self.emg_safe_threshold and not artifact_rejected:
            result = FocusResult(
                score=50.0, state='neutral', confidence=0.0,
                p_focus=0.5, p_relax=0.5,
                emg_level=emg_ratio, artifact_rejected=False,
                fallback_reason='emg',
            )
            self._last_result = result
            return result

        if artifact_rejected:
            # 重度伪迹 → 沿用上次结果或返回 safe neutral
            if self._last_result is not None:
                last = self._last_result
                result = FocusResult(
                    score=last.score,
                    state=last.state,
                    confidence=last.confidence,
                    p_focus=last.p_focus,
                    p_relax=last.p_relax,
                    features=last.features,
                    emg_level=emg_ratio,
                    artifact_rejected=True,
                    fallback_reason='severe_artifact',
                )
                return result
            # 第一次就重度污染
            return FocusResult(
                score=50.0, state='neutral', confidence=0.5,
                p_focus=0.5, p_relax=0.5,
                emg_level=emg_ratio, artifact_rejected=True,
                fallback_reason='severe_artifact',
            )

        # 2. 空间滤波
        filtered = self.car.apply(cleaned)
        if self.laplacian is not None:
            filtered = self.laplacian.apply(filtered)

        # 3. 特征提取
        feat = self.extractor.extract(filtered)
        feat_dict = dict(zip(self._feature_names, feat.tolist()))

        # 4. 分类
        if self.classifier is None:
            p_focus, p_relax = 0.5, 0.5
        else:
            p_focus, p_relax = self.classifier.predict_proba(feat)

        # 5. 置信度平滑
        score, state, confidence = self.smoother.update(p_focus)

        # 6. 计算特征贡献（简化版：基于 |feature - median|)
        top = sorted(feat_dict.items(), key=lambda kv: -abs(kv[1]))[:5]

        # 7. 标记 fallback_reason（P1/F8）
        fallback_reason = None
        if confidence < 0.15:  # ConfidenceSmoother 内部 margin 阈值
            fallback_reason = 'low_confidence'

        result = FocusResult(
            score=score, state=state, confidence=confidence,
            p_focus=p_focus, p_relax=p_relax,
            features=feat_dict, top_contributors=top,
            emg_level=emg_ratio, artifact_rejected=False,
            fallback_reason=fallback_reason,
        )
        self._last_result = result
        return result
