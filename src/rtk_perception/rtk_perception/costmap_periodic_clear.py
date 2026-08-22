"""周期性清空 local_costmap 机器人周围 N 米的残留障碍。

背景：Nav2 ObstacleLayer 是"显式 raytrace 清除"模型——cell 一旦 mark 为 LETHAL
就保留，直到下一条 clearing ray 沿同一角度扫过才清回 FREE。轮椅自身遮挡 +
scan_min_range_filter 屏蔽区会导致部分 cell 永远不被 ray 覆盖 → 障碍移走后
inflation 半径残留数秒甚至更久。

解决方案：调用 Nav2 自带的 clear_around_robot service，每 N 秒清空机器人周围
reset_distance 米范围。清空瞬间真实障碍会"消失"约 33ms（10Hz×3 路 scan = 30Hz
重新 mark），TEB 来不及反应过来，可控。

service 自动发现：local_costmap 可能是顶层节点（service 名 /local_costmap/clear_around_robot）
也可能是 controller_server 内嵌子组件（/controller_server/local_costmap/clear_around_robot）。
节点用 get_service_names_and_types() 找匹配 */clear_around_robot 的 service，
每 5s 重试，无论哪种部署都能工作。

用法：
  ros2 run rtk_perception costmap_periodic_clear \
    --ros-args \
    -p clear_period_s:=2.0 \
    -p reset_distance_m:=1.5 \
    -p service_suffix:=clear_around_robot
"""
import rclpy
from nav2_msgs.srv import ClearCostmapAroundRobot
from rclpy.node import Node


class CostmapPeriodicClearNode(Node):
    def __init__(self):
        super().__init__("costmap_periodic_clear")

        self.declare_parameter("clear_period_s", 2.0)
        self.declare_parameter("reset_distance_m", 1.5)
        # service 后缀，匹配 */clear_around_local_costmap（Humble ROS2 Nav 命名约定）
        # 历史：ROS1 时代叫 clear_around_robot，ROS2 Humble 改名了
        self.declare_parameter("service_suffix", "clear_around_local_costmap")
        # 如果想精确指定某个 service（覆盖自动发现），设这个；空字符串=自动发现
        self.declare_parameter("service_name", "")

        self._clear_period_s = float(self.get_parameter("clear_period_s").value)
        self._reset_distance_m = float(self.get_parameter("reset_distance_m").value)
        self._service_suffix = str(self.get_parameter("service_suffix").value)
        self._forced_name = str(self.get_parameter("service_name").value)

        self._client = None  # 延迟到发现 service 后创建
        self._resolved_name = ""
        self._pending = False
        self._last_warn = 0.0

        # service 发现 + 周期清除都用 timer
        self._discover_timer = self.create_timer(5.0, self._discover_service)
        self._clear_timer = None  # 发现到 service 后才启用

        # 立即触发一次发现
        self._discover_service()
        self.get_logger().info(
            f"costmap_periodic_clear: period={self._clear_period_s}s "
            f"reset_distance={self._reset_distance_m}m "
            f"suffix={self._service_suffix!r}"
        )

    def _discover_service(self):
        """发现匹配后缀的 clear_around_robot service，找到后停止发现 timer，启动清除 timer。"""
        if self._client is not None and self._client.service_is_ready():
            # 已就绪，无需重复发现
            return

        # 用户强制指定优先
        if self._forced_name:
            candidates = [self._forced_name]
        else:
            try:
                services = self.get_service_names_and_types()
            except Exception:
                services = []
            candidates = [
                name for name, _ in services
                if name.rstrip("/").endswith("/" + self._service_suffix)
                or name == self._service_suffix
            ]

        if not candidates:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_warn > 10.0:
                self.get_logger().warn(
                    f"未发现匹配 */{self._service_suffix} 的 service，"
                    f"costmap 可能还没 activate；5s 后重试"
                )
                self._last_warn = now
            return

        # 选第一个候选
        target = candidates[0]
        if target != self._resolved_name:
            self._resolved_name = target
            self._client = self.create_client(ClearCostmapAroundRobot, target)
            self.get_logger().info(f"已绑定 service: {target}")
            # 启动清除 timer（一次性）
            if self._clear_timer is None:
                self._clear_timer = self.create_timer(self._clear_period_s, self._on_tick)

    def _on_tick(self):
        if self._client is None or not self._client.service_is_ready():
            # service 暂时不可用（costmap 重启过？），交给 _discover_timer 重绑
            return

        if self._pending:
            # 上次请求还没回，跳过本次避免堆积
            return

        req = ClearCostmapAroundRobot.Request()
        req.reset_distance = self._reset_distance_m
        future = self._client.call_async(req)
        future.add_done_callback(self._on_response)
        self._pending = True

    def _on_response(self, future):
        self._pending = False
        try:
            future.result()  # 检查异常
        except Exception as e:
            self.get_logger().warn(f"clear_around_robot 调用失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CostmapPeriodicClearNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
