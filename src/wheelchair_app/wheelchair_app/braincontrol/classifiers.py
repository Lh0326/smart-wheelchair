# classifiers.py
"""Classifier Protocol + SVM 实现。Phase 2 可加 EEGNetClassifier。"""
from typing import Optional, Protocol, Tuple
import numpy as np
from joblib import dump, load
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline


class Classifier(Protocol):
    """专注度分类器接口。"""
    def fit(self, X: np.ndarray, y: np.ndarray) -> None: ...
    def predict_proba(self, features: np.ndarray) -> Tuple[float, float]:
        """返回 (p_focus, p_relax)，二者之和为 1。"""
        ...
    def save(self, path: str) -> None: ...
    @classmethod
    def load(cls, path: str) -> "Classifier": ...


class SVMClassifier:
    """RBF 核 SVM + StandardScaler，subject-dependent。

    持久化格式：joblib dump 一个 dict，含 'model'（sklearn Pipeline）、
    可选的 'baseline_alpha'（shape=(n_channels,)）、'feature_mode'
    ('reduced' / 'full') 和 'noise_template'（F4 修复：EMG 频谱减法
    用 relax 段学习模板）。加载时自动兼容旧版（直接 dump Pipeline）。
    """

    def __init__(self, C: float = 1.0, gamma: str = 'scale',
                 baseline_alpha: Optional[np.ndarray] = None,
                 feature_mode: str = 'reduced',
                 noise_template: Optional[dict] = None):
        """
        noise_template: dict {'freqs': np.ndarray, 'mag': np.ndarray}，
            从 relax 段学习的 EMG 噪声模板（F4 修复）。
        """
        self.model = Pipeline([
            ('scaler', StandardScaler()),
            ('svm', SVC(kernel='rbf', C=C, gamma=gamma,
                       class_weight='balanced', probability=True,
                       random_state=42))
        ])
        self.baseline_alpha = (
            np.asarray(baseline_alpha, dtype=np.float64)
            if baseline_alpha is not None else None
        )
        self.feature_mode = feature_mode
        self.noise_template = noise_template

    def fit(self, X: np.ndarray, y: np.ndarray) -> None:
        self.model.fit(X, y)

    def predict_proba(self, features: np.ndarray) -> Tuple[float, float]:
        proba = self.model.predict_proba(features.reshape(1, -1))[0]
        classes = self.model.named_steps['svm'].classes_
        focus_idx = list(classes).index(1) if 1 in classes else 1
        relax_idx = 1 - focus_idx
        return float(proba[focus_idx]), float(proba[relax_idx])

    def save(self, path: str) -> None:
        dump({
            'model': self.model,
            'baseline_alpha': self.baseline_alpha,
            'feature_mode': self.feature_mode,
            'noise_template': self.noise_template,
        }, path)

    @classmethod
    def load(cls, path: str) -> "SVMClassifier":
        instance = cls.__new__(cls)
        data = load(path)
        if isinstance(data, dict) and 'model' in data:
            instance.model = data['model']
            ba = data.get('baseline_alpha')
            instance.baseline_alpha = (
                np.asarray(ba, dtype=np.float64) if ba is not None else None
            )
            instance.feature_mode = data.get('feature_mode', 'full')
            instance.noise_template = data.get('noise_template')
        else:
            # 兼容旧版格式：直接 dump 的 sklearn Pipeline
            instance.model = data
            instance.baseline_alpha = None
            instance.feature_mode = 'full'
            instance.noise_template = None
        return instance


class EEGNetClassifier:
    """Phase 2 实现。当前抛 NotImplementedError。"""
    def __init__(self, *args, **kwargs):
        raise NotImplementedError("EEGNetClassifier 留待 Phase 2")
