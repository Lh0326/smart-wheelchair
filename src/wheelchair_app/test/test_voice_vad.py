from wheelchair_app.nodes.voice_node import compute_adaptive_vad_threshold


def test_adaptive_vad_uses_quiet_noise_floor():
    threshold = compute_adaptive_vad_threshold([0.010, 0.012, 0.011, 0.050])
    assert 0.018 <= threshold < 0.030


def test_adaptive_vad_handles_noisy_microphone():
    threshold = compute_adaptive_vad_threshold([0.030, 0.032, 0.035, 0.080])
    assert 0.050 <= threshold <= 0.080


def test_adaptive_vad_falls_back_without_samples():
    assert compute_adaptive_vad_threshold([]) == 0.025
