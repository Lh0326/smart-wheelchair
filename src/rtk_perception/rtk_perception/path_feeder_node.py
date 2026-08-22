"""path_feeder_node：订阅 /nav_path，调 controller_server FollowPath action。

职责：
  - 收到新 /nav_path → cancel 当前 goal → 发新 goal
  - /nav_path 1 秒无更新 → cancel 当前 goal（避免 TEB 用旧路径）
  - goal 完成或失败 → 等下一个 /nav_path
  - 频率限制：最多 0.5 秒发一次 goal
  - **目的地去重**：同一最终目的地只发送一次 FollowPath goal；忽略轮椅前进
    造成的路径前缀裁剪，避免中间点附近重置 TEB 和下位机加速斜坡

并发安全：
  - 锁只用于保护共享状态（_latest_path/_current_goal_handle/_last_goal_sent_sec）
  - 阻塞 IO（wait_for_server, spin_until_future_complete）在锁外执行
"""
from __future__ import annotations

import signal
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from std_msgs.msg import Bool


def path_signature(
    path: Path,
    sample_count: int = 8,
    resolution_m: float = 0.001,
    ignore_start_pose: bool = False,
) -> bytes:
    """计算路径内容的稳定签名(用于内容去重)。

    全部 poses 比较代价高(可能几百点),均匀采样 sample_count 个关键点即可。
    每点 x/y 按 resolution_m 量化，过滤定位/投影抖动。

    返回 bytes(便于直接比较/hash)。空路径返回 b''。
    """
    if not path.poses:
        return b""
    resolution_m = max(float(resolution_m), 0.001)
    poses = path.poses[1:] if ignore_start_pose and len(path.poses) > 1 else path.poses
    n = len(poses)
    if n <= sample_count:
        sampled = poses
    else:
        # 均匀采样(含首尾)
        idxs = [int(i * (n - 1) / (sample_count - 1)) for i in range(sample_count)]
        sampled = [poses[i] for i in idxs]

    parts = []
    for p in sampled:
        x_q = int(round(p.pose.position.x / resolution_m))
        y_q = int(round(p.pose.position.y / resolution_m))
        parts.append(f"{x_q},{y_q}")
    return "|".join(parts).encode()


def destination_signature(path: Path, resolution_m: float = 0.25) -> bytes:
    """按最终目的地生成稳定签名。

    path_to_baselink 会随轮椅前进持续裁掉已经走过的路径前缀，因此完整路径
    签名会在经过中间路径点时变化。若据此替换 FollowPath goal，controller
    会先输出零速并重置 TEB；下位机约 1.8 秒的加速斜坡又会从零重新开始，
    最终表现为中间点附近反复走停。

    同一次导航任务的最终 pose 保持不变，只有用户选择新终点时才应替换
    FollowPath goal。坐标量化用于过滤 GNSS/投影的小幅抖动。
    """
    if not path.poses:
        return b""
    resolution_m = max(float(resolution_m), 0.01)
    destination = path.poses[-1].pose.position
    x_q = int(round(destination.x / resolution_m))
    y_q = int(round(destination.y / resolution_m))
    return f"goal:{x_q},{y_q}".encode()


def signature_retry_blocked(
    blocked_signature: bytes | None,
    blocked_until_sec: float,
    current_signature: bytes,
    now_sec: float,
) -> bool:
    """同一路径仅在 ABORT 退避窗口内禁止重发。"""
    return (
        blocked_signature is not None
        and current_signature == blocked_signature
        and now_sec < blocked_until_sec
    )


class PathFeederNode(Node):
    def __init__(self):
        super().__init__("path_feeder_node")

        self.declare_parameter("path_timeout_sec", 1.0)
        self.declare_parameter("goal_min_interval_sec", 0.5)
        self.declare_parameter("controller_server_name", "controller_server")
        self.declare_parameter("nav_control_heartbeat_sec", 0.2)
        self.declare_parameter("path_signature_resolution_m", 0.10)
        self.declare_parameter("abort_retry_backoff_sec", 3.0)
        self.declare_parameter("goal_delay_sec", 2.0)

        self._path_timeout = float(self.get_parameter("path_timeout_sec").value)
        self._goal_min_interval = float(self.get_parameter("goal_min_interval_sec").value)
        self._nav_control_heartbeat_sec = float(
            self.get_parameter("nav_control_heartbeat_sec").value
        )
        self._path_signature_resolution_m = float(
            self.get_parameter("path_signature_resolution_m").value
        )
        self._abort_retry_backoff_sec = float(
            self.get_parameter("abort_retry_backoff_sec").value
        )
        self._goal_delay_sec = float(self.get_parameter("goal_delay_sec").value)
        controller_name = str(self.get_parameter("controller_server_name").value)

        # FollowPath action client
        # nav2 controller_server 默认在 /follow_path 注册 action（不带 controller_server 前缀）
        # 参考: https://docs.nav2.org/configuration/packages/configuring-controller-server.html
        self._action_client = ActionClient(
            self, FollowPath, "follow_path"
        )

        # 订阅 /nav_path（QoS 与 path_to_baselink publisher 对齐）
        nav_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Path, "/nav_path", self._nav_path_cb, nav_qos)

        # 前端"清除终点"按钮 → 取消 controller_server 当前 FollowPath goal
        # path_to_baselink_node 也订阅同一 topic 清自己缓存，但只有 path_feeder 取消 action
        # 才能让 TEB 真正停下（轮椅静止原地）。
        from std_msgs.msg import Empty
        self.create_subscription(Empty, "/clear_goal", self._on_clear_goal, 10)
        self._nav_control_pub = self.create_publisher(Bool, "/nav_control_active", 10)

        # 共享状态（受 _lock 保护）
        self._latest_path: Path | None = None
        self._latest_path_time_sec = 0.0
        self._last_goal_sent_sec = 0.0
        self._last_sent_signature: bytes | None = None  # 上次成功发出的路径签名(去重)
        self._current_goal_handle = None
        self._current_goal_signature: bytes | None = None
        self._pending_goal = False  # I2: 标记是否有 goal 在路上（已发出但还未收到 response）
        self._pending_goal_signature: bytes | None = None
        self._blocked_signature: bytes | None = None
        self._blocked_until_sec = 0.0
        # USB Hub OCP 错峰：新签名首次到达后等 goal_delay_sec 再发 goal，
        # 让 networkx 全局规划 + path_to_baselink 转换的 CPU 峰值先过去
        self._pending_sig: bytes | None = None
        self._pending_sig_ready_sec: float = 0.0
        self._nav_control_active = False
        self._last_nav_control_pub_sec = 0.0
        self._lock = threading.Lock()

        # 10Hz 检查：是否需要发新 goal 或 cancel 过期 goal
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"PathFeederNode started. action=/follow_path (controller_server={controller_name}), "
            f"path_timeout={self._path_timeout}s, "
            f"goal_min_interval={self._goal_min_interval}s"
        )
        self._publish_nav_control_active(False, "startup")

    def _publish_nav_control_active(self, active: bool, reason: str, force: bool = False):
        """发布真实底盘自主导航执行门控。

        该话题只表达"TEB 正在跟踪有效路径"，不是运动指令。真实底盘节点仍以
        /cmd_vel_safe 为运动意图源，以 HWT /heading_imu 为航向反馈源。
        """
        if self._nav_control_active == active and not force:
            return
        state_changed = self._nav_control_active != active
        self._nav_control_active = active
        msg = Bool()
        msg.data = active
        self._nav_control_pub.publish(msg)
        self._last_nav_control_pub_sec = self.get_clock().now().nanoseconds * 1e-9
        if state_changed:
            self.get_logger().info(f"nav_control_active={active} ({reason})")

    def _nav_path_cb(self, msg: Path):
        if not msg.poses:
            return
        with self._lock:
            self._latest_path = msg
            self._latest_path_time_sec = self.get_clock().now().nanoseconds * 1e-9

    def _on_clear_goal(self, msg):
        """前端"清除终点"按钮 → cancel 当前 FollowPath goal + 清空缓存路径。

        通知 controller_server 停止执行当前路径（TEB 收到 cancel 后停止发 cmd_vel）。
        同时清空 self._latest_path 避免下个 tick 又把同一路径发回去。
        """
        self.get_logger().info("clear_goal 收到，cancel 当前 FollowPath goal")
        with self._lock:
            self._latest_path = None
            self._latest_path_time_sec = 0.0
            self._last_sent_signature = None  # 让下次新终点能正常发出
            self._blocked_signature = None
            self._blocked_until_sec = 0.0
            self._pending_sig = None
            self._pending_sig_ready_sec = 0.0
        self._publish_nav_control_active(False, "clear_goal")
        self._cancel_current_goal_async()

    def _tick(self):
        """10Hz 决策循环。锁内只读状态、做决策；锁外做 action IO。"""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (
            self._nav_control_active
            and now_sec - self._last_nav_control_pub_sec >= self._nav_control_heartbeat_sec
        ):
            self._publish_nav_control_active(True, "heartbeat", force=True)

        # === 锁内：读状态 + 决定动作 ===
        with self._lock:
            if self._latest_path is None:
                return

            path_age = now_sec - self._latest_path_time_sec
            path_expired = path_age > self._path_timeout
            goal_in_flight = self._current_goal_handle is not None or self._pending_goal
            recent_sent = now_sec - self._last_goal_sent_sec < self._goal_min_interval
            path_to_send = self._latest_path
            last_sig = self._last_sent_signature

        # === 锁外：执行 IO（不持锁）===

        # 路径过期 + 有 in-flight goal → cancel
        if path_expired and goal_in_flight:
            self.get_logger().warn(
                "Path expired, canceling current goal",
                throttle_duration_sec=2.0,
            )
            self._publish_nav_control_active(False, "path_expired")
            self._cancel_current_goal_async()
            # 过期取消后,清空签名让下次新路径能正常发送
            with self._lock:
                self._last_sent_signature = None
            return

        # 路径过期但无 goal → 啥也不做
        if path_expired:
            return

        # 频率限制
        if recent_sent:
            return

        # 导航任务去重只看最终目的地。/nav_path 是随当前位置滚动裁剪的路径，
        # 完整路径签名会在通过每个中间点时变化；替换 goal 会让 TEB 输出零速
        # 并使下位机 1.8s 加速斜坡重新开始，造成明显的“走一下停一下”。
        current_sig = destination_signature(
            path_to_send,
            resolution_m=self._path_signature_resolution_m,
        )
        if signature_retry_blocked(
            self._blocked_signature,
            self._blocked_until_sec,
            current_sig,
            now_sec,
        ):
            return
        if last_sig is not None and current_sig == last_sig:
            return  # 路径未变化,跳过

        # USB Hub OCP 错峰：新签名首次到达后等 goal_delay_sec 再发，
        # 让 networkx 全局规划 + path_to_baselink 转换 CPU 峰值先过去。
        # _nav_path_cb 周期推送会使 _latest_path_time_sec 一直刷新，
        # 所以用签名判等识别"首次见到的新签名"，而非 path_age。
        with self._lock:
            if self._pending_sig != current_sig:
                self._pending_sig = current_sig
                self._pending_sig_ready_sec = now_sec + self._goal_delay_sec
                self.get_logger().info(
                    f"新路径签名首次到达，等 {self._goal_delay_sec}s grace period "
                    f"再发 goal（USB Hub OCP 错峰）"
                )
                return
            if now_sec < self._pending_sig_ready_sec:
                return  # grace period 未到
            # grace period 已过，清 pending 后继续发
            self._pending_sig = None
            self._pending_sig_ready_sec = 0.0

        # 有 in-flight goal → 先 cancel
        if goal_in_flight:
            self._cancel_current_goal_async()

        # 发新 goal（非阻塞）
        if self._send_goal(path_to_send, current_sig):
            # I1 修复：只在 _send_goal 真正发出后才推进时间戳 + 记录签名
            with self._lock:
                self._last_goal_sent_sec = now_sec
                self._last_sent_signature = current_sig

    def _cancel_current_goal_async(self):
        """异步 cancel 当前 goal(不阻塞 timer 线程)。

        优化:rclpy.spin_until_future_complete 在 timer/action 回调中是 nested spin,
        会导致 busy loop + CPU 占用 70%+。改用 Future.add_done_callback。
        """
        with self._lock:
            handle = self._current_goal_handle
            self._current_goal_handle = None
            self._current_goal_signature = None
            # 不清 _pending_goal：cancel 后若 pending 还在路上，response_cb 会处理

        if handle is None:
            return
        self._publish_nav_control_active(False, "cancel_current_goal")
        try:
            future = handle.cancel_goal_async()
            future.add_done_callback(self._on_cancel_done)
        except Exception as e:
            self.get_logger().warn(f"cancel_goal_async 失败: {e}")

    def _on_cancel_done(self, future):
        """cancel 异步完成回调。"""
        try:
            result = future.result()
            self.get_logger().info(f"Cancel done: code={result.return_code}")
        except Exception as e:
            self.get_logger().warn(f"Cancel future 异常: {e}")

    def _send_goal(self, path: Path, signature: bytes) -> bool:
        """非阻塞发 goal。返回 True 表示已投递（response 还在路上），False 表示 server 未就绪。"""
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn(
                "controller_server action not available",
                throttle_duration_sec=5.0,
            )
            return False

        with self._lock:
            self._pending_goal = True
            self._pending_goal_signature = signature

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda response_future, goal_sig=signature:
            self._goal_response_cb(response_future, goal_sig)
        )
        return True

    def _goal_response_cb(self, future, goal_sig=None):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f"Goal response error: {e}")
            with self._lock:
                if goal_sig is None or self._pending_goal_signature == goal_sig:
                    self._pending_goal = False
                    self._pending_goal_signature = None
                if goal_sig is None or self._last_sent_signature == goal_sig:
                    self._last_sent_signature = None
            return

        with self._lock:
            if goal_sig is None:
                goal_sig = self._pending_goal_signature
            if self._pending_goal_signature == goal_sig:
                self._pending_goal = False
                self._pending_goal_signature = None

        if not handle.accepted:
            self.get_logger().warn("Goal rejected by controller_server")
            with self._lock:
                if self._last_sent_signature == goal_sig:
                    self._last_sent_signature = None
            self._publish_nav_control_active(False, "goal_rejected")
            return

        with self._lock:
            # I2: 若期间已被 cancel/替换，立即 cancel 新 handle
            stale = self._current_goal_handle is not None
            if not stale:
                self._current_goal_handle = handle
                self._current_goal_signature = goal_sig

        if stale:
            self.get_logger().warn("Received response for stale goal, canceling")
            try:
                cancel_future = handle.cancel_goal_async()
                cancel_future.add_done_callback(self._on_cancel_done)
            except Exception as e:
                self.get_logger().warn(f"Stale cancel async 失败: {e}")
            return

        self._publish_nav_control_active(True, "goal_accepted")
        handle.get_result_async().add_done_callback(
            lambda result_future, goal_handle=handle, goal_sig=goal_sig:
            self._result_cb(result_future, goal_handle, goal_sig)
        )

    def _result_cb(self, future, goal_handle=None, goal_sig=None):
        # cancel/替换旧 goal 后，它的 result 可能晚于新 goal 返回。旧回调绝不能
        # 清除新 goal handle 或发布 nav_control_active=False。
        with self._lock:
            if goal_handle is not None and self._current_goal_handle is not goal_handle:
                self.get_logger().info("忽略已替换 FollowPath goal 的迟到 result")
                return
            if goal_sig is None:
                goal_sig = self._current_goal_signature
            self._current_goal_handle = None
            self._current_goal_signature = None
        self._publish_nav_control_active(False, "goal_result")
        try:
            result = future.result()
            status = result.status
            self.get_logger().info(f"FollowPath completed: {status}")
        except Exception as e:
            self.get_logger().warn(f"Goal result error: {e}")
            status = 6

        # 4=SUCCEEDED, 5=CANCELED, 6=ABORTED (action_msgs/GoalStatus)
        with self._lock:
            if status == 6 and goal_sig is not None:
                self._blocked_signature = goal_sig
                self._blocked_until_sec = (
                    self.get_clock().now().nanoseconds * 1e-9
                    + self._abort_retry_backoff_sec
                )
                # backoff 窗口内由 _blocked_signature 阻止立即重发；窗口结束后
                # 必须允许同一路径重试，否则一次瞬态 TEB abort 会永久终止导航。
                self._last_sent_signature = None
            else:
                # 成功/取消/未知完成后允许下一条新路径正常发送。
                self._last_sent_signature = None
                self._blocked_signature = None
                self._blocked_until_sec = 0.0

    def cancel_all_and_shutdown(self):
        """退出前清理：cancel 当前 goal + 等 cancel 完成。"""
        with self._lock:
            handle = self._current_goal_handle
            self._current_goal_handle = None
        self._publish_nav_control_active(False, "shutdown")
        if handle is not None:
            try:
                future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                self.get_logger().info("Canceled in-flight goal on shutdown")
            except Exception as e:
                self.get_logger().warn(f"Shutdown cancel failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PathFeederNode()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except (Exception, KeyboardInterrupt) as e:
        print(f"[path_feeder_node] FATAL: {e}")
    finally:
        # I4: 退出前 cancel 当前 goal，避免 TEB 继续跑
        if rclpy.ok():
            node.cancel_all_and_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
