"""验证 braincontrol 包所有运行时模块可以正常 import。"""


def test_import_ads1299():
    from wheelchair_app.braincontrol import ads1299
    assert hasattr(ads1299, 'ADS1298Data')


def test_import_focus_pipeline():
    from wheelchair_app.braincontrol import (
        focus_detector, emg_handler, feature_extractor, classifiers,
        confidence_smoother, spatial_filter, artifact_rejector, signal_quality
    )
    assert hasattr(focus_detector, 'FocusDetector')


def test_import_imu_pipeline():
    from wheelchair_app.braincontrol import (
        imu_reader, imu_handler, head_pose_calculator, tilt_indicator
    )
    assert hasattr(imu_reader, 'ESP32ImuReader')


def test_import_control_layer():
    from wheelchair_app.braincontrol import (
        control_types, control_state_machine, motion_commander
    )
    assert hasattr(control_types, 'MotionCommand')
    assert hasattr(control_state_machine, 'ControlStateMachine')
    assert hasattr(motion_commander, 'MotionCommander')


def test_import_event_detectors():
    from wheelchair_app.braincontrol import (
        clench_detector, clench_features, frown_detector
    )
    assert hasattr(clench_detector, 'ClenchDetector')


def test_models_exist():
    import os
    from wheelchair_app.braincontrol import __path__ as pkg_path
    # pkg_path[0] 直接就是 braincontrol/ 目录本身（不是它的父目录）
    pkg_root = pkg_path[0] if isinstance(pkg_path, list) else pkg_path
    assert os.path.exists(os.path.join(pkg_root, 'models', 'focus_svm.joblib'))
    assert os.path.exists(os.path.join(pkg_root, 'models', 'clench_svm.joblib'))
