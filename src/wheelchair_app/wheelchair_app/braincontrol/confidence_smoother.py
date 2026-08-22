# confidence_smoother.py
"""置信度滞回平滑器：EWMA + 滞回阈值 + 置信度弃权。"""


class ConfidenceSmoother:
    """状态机：
        - score > high_thresh → 'focused'
        - score < low_thresh  → 'relaxed'
        - 中间带 → 保持上一状态
        - 连续 uncertain_count >= 3 → 强制 'neutral'
    """

    def __init__(self,
                 low_thresh: float = 0.4,
                 high_thresh: float = 0.6,
                 alpha: float = 0.3,
                 margin: float = 0.15):
        self.low_thresh = low_thresh
        self.high_thresh = high_thresh
        self.alpha = alpha
        self.margin = margin
        # 初始 50：中性起步，让 EWMA 自然收敛。之前用 90 是让旧测试
        # 通过的 hack（违反"初始无偏好"语义）。真实推理首窗口结果
        # 不应预设为"接近 focused"。
        self.score = 50.0
        self.state = 'neutral'
        self.uncertain_count = 0

    def update(self, p_focus: float) -> tuple:
        """输入 SVM 概率，更新内部状态。返回 (score, state, confidence)。"""
        p_relax = 1.0 - p_focus
        gap = abs(p_focus - p_relax)
        # EWMA 平滑
        self.score = self.alpha * (p_focus * 100) + (1 - self.alpha) * self.score
        # 置信度弃权：gap 严格 >0（排除死区 p_focus == p_relax == 0.5，
        # 那是滞回保持带）且 < margin（弱信号但不确定）→ 计数
        if 0.0 < gap < self.margin:
            self.uncertain_count += 1
        else:
            self.uncertain_count = 0
        # 状态转移
        if self.uncertain_count >= 3:
            self.state = 'neutral'
        elif self.score / 100 > self.high_thresh:
            self.state = 'focused'
        elif self.score / 100 < self.low_thresh:
            self.state = 'relaxed'
        # 中间带 → 不改 state（滞回）
        confidence = max(p_focus, p_relax)
        return self.score, self.state, confidence

    def reset(self):
        self.score = 50.0
        self.state = 'neutral'
        self.uncertain_count = 0
