#!/usr/bin/env python3
import array
import math
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Dict, List, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rosidl_runtime_py.utilities import get_message
from rosidl_parser.definition import AbstractNestedType, Array as RosArray, BasicType, NamespacedType
from minitrone_interfaces.msg import CenterOfPressure

try:
    from PyQt5 import QtCore, QtWidgets
    QT_BINDING = "PyQt5"
except ImportError:
    try:
        from PySide6 import QtCore, QtWidgets
        QT_BINDING = "PySide6"
    except ImportError as exc:
        raise SystemExit(
            "Qt binding not found. Install `python3-pyqt5` or `PySide6`, and `pyqtgraph`."
        ) from exc

try:
    import pyqtgraph as pg
except ImportError as exc:
    raise SystemExit("`pyqtgraph` is not installed. Install it before running this node.") from exc


COLORS = (
    "#d1495b",
    "#edae49",
    "#00798c",
    "#66a182",
    "#9c6644",
    "#8d99ae",
    "#ef476f",
    "#118ab2",
    "#ffd166",
    "#06d6a0",
)


@dataclass
class TopicInfo:
    name: str
    type_name: str
    message_class: type
    metadata_fields: List[str]
    observed_fields: Set[str]
    latest_flattened: Dict[str, float]
    message_count: int = 0
    first_message_logged: bool = False

    def all_fields(self) -> List[str]:
        return sorted(set(self.metadata_fields) | self.observed_fields)


@dataclass
class CurveRecord:
    key: str
    topic: str
    field_path: str
    display_name: str
    color: str
    panel_index: int
    plot_item: pg.PlotDataItem
    times: Deque[float]
    values: Deque[float]
    dirty: bool = True


def is_numeric_value(value: object) -> bool:
    if isinstance(value, bool):
        return False
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


def is_numeric_sequence(value: object) -> bool:
    if isinstance(value, (str, bytes)):
        return False
    if isinstance(value, dict):
        return False
    try:
        items = list(value) if not isinstance(value, array.array) else list(value)
    except TypeError:
        return False
    if len(items) == 0:
        return False
    return all(is_numeric_value(item) for item in items)


def flatten_numeric_message(msg: object, prefix: str = "") -> Dict[str, float]:
    data: Dict[str, float] = {}
    if is_numeric_value(msg):
        data[prefix] = float(msg)
        return data

    if is_numeric_sequence(msg):
        for index, item in enumerate(msg):
            data[f"{prefix}[{index}]"] = float(item)
        return data

    if hasattr(msg, "__slots__"):
        for slot in msg.__slots__:
            field_name = slot.lstrip("_")
            child_prefix = field_name if not prefix else f"{prefix}.{field_name}"
            child_value = getattr(msg, slot)
            data.update(flatten_numeric_message(child_value, child_prefix))
    return data


def sanitize_numeric_map(values: Dict[str, float]) -> Dict[str, float]:
    sanitized: Dict[str, float] = {}
    for key, value in values.items():
        if math.isfinite(value):
            sanitized[key] = value
    return sanitized


NUMERIC_PRIMITIVE_TYPES = {
    "float",
    "double",
    "long double",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
}


def namespaced_type_to_string(slot_type: NamespacedType) -> str:
    return "/".join([*slot_type.namespaces, slot_type.name])


def discover_numeric_fields(message_class: type, prefix: str = "") -> List[str]:
    fields: List[str] = []
    slot_types = getattr(message_class, "SLOT_TYPES", [])
    slots = getattr(message_class, "__slots__", [])

    for slot_name, slot_type in zip(slots, slot_types):
        field_name = slot_name.lstrip("_")
        field_prefix = field_name if not prefix else f"{prefix}.{field_name}"

        if isinstance(slot_type, BasicType):
            if slot_type.typename in NUMERIC_PRIMITIVE_TYPES:
                fields.append(field_prefix)
            continue

        if isinstance(slot_type, AbstractNestedType):
            value_type = slot_type.value_type
            if isinstance(value_type, BasicType) and value_type.typename in NUMERIC_PRIMITIVE_TYPES:
                if isinstance(slot_type, RosArray):
                    for index in range(slot_type.size):
                        fields.append(f"{field_prefix}[{index}]")
                else:
                    fields.append(f"{field_prefix}[]")
            elif isinstance(value_type, NamespacedType) and isinstance(slot_type, RosArray):
                try:
                    nested_class = get_message(namespaced_type_to_string(value_type))
                except Exception:
                    continue
                nested_fields = discover_numeric_fields(nested_class)
                for index in range(slot_type.size):
                    for nested_field in nested_fields:
                        fields.append(f"{field_prefix}[{index}].{nested_field}")
            else:
                fields.append(f"{field_prefix}[]")
            continue

        if isinstance(slot_type, NamespacedType):
            try:
                nested_class = get_message(namespaced_type_to_string(slot_type))
            except Exception:
                continue
            fields.extend(discover_numeric_fields(nested_class, field_prefix))

    return sorted(fields)


class TopicPlotNode(Node):
    def __init__(self, *, initial_topic: str = "") -> None:
        super().__init__("minitrone_topic_plot")
        self.window_sec = float(self.declare_parameter("window_sec", 10.0).value)
        self.refresh_hz = float(self.declare_parameter("refresh_hz", 20.0).value)
        self.discovery_period_sec = float(self.declare_parameter("discovery_period_sec", 2.0).value)
        self.auto_add_limit = max(1, int(self.declare_parameter("auto_add_limit", 8).value))
        self.plot_rows = max(1, min(3, int(self.declare_parameter("plot_rows", 2).value)))
        self.plot_columns = max(1, min(3, int(self.declare_parameter("plot_columns", 2).value)))
        self.cop_hat_topic = str(self.declare_parameter(
            "cop_hat_topic", "/minitrone/cop_hat").value)
        self.cop_real_topic = str(self.declare_parameter(
            "cop_real_topic", "/minitrone/cop_real").value)
        self.plate_size_y = abs(float(self.declare_parameter("plate_size_y", 0.40).value))
        self.plate_size_z = abs(float(self.declare_parameter("plate_size_z", 0.38).value))
        self.cop_force_min = abs(float(self.declare_parameter("cop_force_min", 0.50).value))
        self.cop_y_sign = float(self.declare_parameter("cop_y_sign", 1.0).value)
        self.cop_z_sign = float(self.declare_parameter("cop_z_sign", -1.0).value)
        self.cop_trail_length = max(
            1, int(self.declare_parameter("cop_trail_length", 100).value)
        )
        default_topic = initial_topic or ""
        self.default_topic = str(self.declare_parameter("topic", default_topic).value)
        self.history_limit = max(200, int(self.window_sec * max(self.refresh_hz, 1.0) * 8.0))
        self.start_time: Optional[float] = None
        self.lock = threading.RLock()
        self.executor: Optional[SingleThreadedExecutor] = None

        self.topic_infos: Dict[str, TopicInfo] = {}
        self.subscriptions_by_topic: Dict[str, object] = {}
        self.curves_by_key: Dict[str, CurveRecord] = {}
        self.cop_valid = False
        self.cop_y = 0.0
        self.cop_z = 0.0
        self.cop_normal_force = 0.0
        self.cop_trail_y: Deque[float] = deque(maxlen=self.cop_trail_length)
        self.cop_trail_z: Deque[float] = deque(maxlen=self.cop_trail_length)
        self.cop_dirty = True
        self.cop_real_valid = False
        self.cop_real_y = 0.0
        self.cop_real_z = 0.0
        self.cop_real_normal_force = 0.0
        self.cop_real_trail_y: Deque[float] = deque(maxlen=self.cop_trail_length)
        self.cop_real_trail_z: Deque[float] = deque(maxlen=self.cop_trail_length)
        self.cop_hat_subscription = self.create_subscription(
            CenterOfPressure, self.cop_hat_topic, self._on_cop_hat, 10
        )
        self.cop_real_subscription = self.create_subscription(
            CenterOfPressure, self.cop_real_topic, self._on_cop_real, 10
        )

    def discover_topics(self) -> Dict[str, TopicInfo]:
        discovered: Dict[str, TopicInfo] = {}
        previous_infos = self.topic_infos
        for topic_name, type_names in self.get_topic_names_and_types():
            if not type_names:
                continue
            type_name = type_names[0]
            try:
                message_class = get_message(type_name)
            except Exception:
                continue

            fields = discover_numeric_fields(message_class)
            if not fields:
                continue
            observed_fields = set()
            if topic_name in previous_infos:
                observed_fields = set(previous_infos[topic_name].observed_fields)
            discovered[topic_name] = TopicInfo(
                name=topic_name,
                type_name=type_name,
                message_class=message_class,
                metadata_fields=fields,
                observed_fields=observed_fields,
                latest_flattened=dict(previous_infos[topic_name].latest_flattened) if topic_name in previous_infos else {},
                message_count=previous_infos[topic_name].message_count if topic_name in previous_infos else 0,
                first_message_logged=previous_infos[topic_name].first_message_logged if topic_name in previous_infos else False,
            )
        with self.lock:
            self.topic_infos = dict(sorted(discovered.items()))
            return dict(self.topic_infos)

    def ensure_subscription(self, topic_name: str) -> None:
        with self.lock:
            if topic_name in self.subscriptions_by_topic:
                return
            topic_info = self.topic_infos.get(topic_name)
            if topic_info is None:
                return
            self.subscriptions_by_topic[topic_name] = self.create_subscription(
                topic_info.message_class,
                topic_name,
                self._make_topic_callback(topic_name),
                10,
            )
            self.get_logger().info(f"subscribed to {topic_name} [{topic_info.type_name}]")
            if self.executor is not None:
                self.executor.wake()

    def _make_topic_callback(self, topic_name: str) -> Callable[[object], None]:
        def callback(msg: object) -> None:
            try:
                now = time.perf_counter()
                with self.lock:
                    if self.start_time is None:
                        self.start_time = now
                    t = now - self.start_time
                flattened = sanitize_numeric_map(flatten_numeric_message(msg))
                with self.lock:
                    topic_info = self.topic_infos.get(topic_name)
                    if topic_info is not None:
                        topic_info.message_count += 1
                        topic_info.observed_fields.update(flattened.keys())
                        topic_info.latest_flattened = dict(flattened)
                        if not topic_info.first_message_logged:
                            preview_keys = sorted(flattened.keys())[:6]
                            self.get_logger().info(
                                f"first message on {topic_name}: {len(flattened)} numeric fields, sample keys={preview_keys}"
                            )
                            topic_info.first_message_logged = True
                    for curve in self.curves_by_key.values():
                        if curve.topic != topic_name:
                            continue
                        value = flattened.get(curve.field_path)
                        if value is None:
                            continue
                        curve.times.append(t)
                        curve.values.append(value)
                        curve.dirty = True
            except Exception as exc:
                self.get_logger().error(f"plot callback failed for {topic_name}: {exc}")

        return callback

    def _on_cop_hat(self, msg: CenterOfPressure) -> None:
        with self.lock:
            self.cop_normal_force = float(msg.normal_force)
            self.cop_valid = bool(msg.valid)
            if self.cop_valid:
                self.cop_y = self.cop_y_sign * float(msg.y)
                self.cop_z = -self.cop_z_sign * float(msg.z)
                self.cop_trail_y.append(self.cop_y)
                self.cop_trail_z.append(self.cop_z)
            self.cop_dirty = True

    def _on_cop_real(self, msg: CenterOfPressure) -> None:
        with self.lock:
            self.cop_real_normal_force = float(msg.normal_force)
            self.cop_real_valid = bool(msg.valid)
            if self.cop_real_valid:
                self.cop_real_y = float(msg.y)
                self.cop_real_z = float(msg.z)
                self.cop_real_trail_y.append(self.cop_real_y)
                self.cop_real_trail_z.append(self.cop_real_z)
            self.cop_dirty = True


class TopicPlotWindow(QtWidgets.QWidget):
    def __init__(self, node: TopicPlotNode, spawn_window: Optional[Callable[[], None]] = None) -> None:
        super().__init__()
        self.node = node
        self.spawn_window = spawn_window
        self.is_closing = False
        self.color_index = 0
        self.last_discovery_time = 0.0
        self.initial_auto_add_done = False
        self.current_topic_names: List[str] = []
        self.plot_rows = self.node.plot_rows
        self.plot_columns = self.node.plot_columns

        self.setWindowTitle("ROS 2 Topic Plot")
        self.resize(1500, 900)
        pg.setConfigOptions(antialias=False, foreground="#d8dee9", background="#11151c")

        root_layout = QtWidgets.QHBoxLayout(self)
        controls_layout = QtWidgets.QVBoxLayout()
        root_layout.addLayout(controls_layout, 0)

        self.topic_filter_edit = QtWidgets.QLineEdit()
        self.topic_filter_edit.setPlaceholderText("filter topic")
        controls_layout.addWidget(self.topic_filter_edit)

        self.topic_list = QtWidgets.QListWidget()
        controls_layout.addWidget(self.topic_list, 2)

        self.topic_type_label = QtWidgets.QLabel("type: -")
        controls_layout.addWidget(self.topic_type_label)

        self.field_list = QtWidgets.QListWidget()
        self.field_list.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        controls_layout.addWidget(self.field_list, 2)

        self.add_button = QtWidgets.QPushButton("Add Selected Fields")
        controls_layout.addWidget(self.add_button)

        self.layout_combo = QtWidgets.QComboBox()
        for rows in range(1, 4):
            for columns in range(1, 4):
                self.layout_combo.addItem(f"Layout {rows} x {columns}", (rows, columns))
        initial_layout_index = (self.plot_rows - 1) * 3 + (self.plot_columns - 1)
        self.layout_combo.setCurrentIndex(initial_layout_index)
        controls_layout.addWidget(self.layout_combo)

        self.panel_combo = QtWidgets.QComboBox()
        controls_layout.addWidget(self.panel_combo)

        self.active_curve_list = QtWidgets.QListWidget()
        self.active_curve_list.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        controls_layout.addWidget(self.active_curve_list, 2)

        self.remove_button = QtWidgets.QPushButton("Remove Selected")
        self.clear_button = QtWidgets.QPushButton("Clear All")
        controls_layout.addWidget(self.remove_button)
        controls_layout.addWidget(self.clear_button)

        self.window_spin = QtWidgets.QDoubleSpinBox()
        self.window_spin.setRange(1.0, 120.0)
        self.window_spin.setValue(self.node.window_sec)
        self.window_spin.setSuffix(" s")
        controls_layout.addWidget(self.window_spin)

        self.refresh_topics_button = QtWidgets.QPushButton("Refresh Topics")
        controls_layout.addWidget(self.refresh_topics_button)
        if self.spawn_window is not None:
            self.new_window_button = QtWidgets.QPushButton("Open New Window")
            controls_layout.addWidget(self.new_window_button)
        controls_layout.addStretch(1)

        plot_layout = QtWidgets.QVBoxLayout()
        root_layout.addLayout(plot_layout, 1)

        self.status_label = QtWidgets.QLabel(f"ready with {QT_BINDING} + pyqtgraph")
        plot_layout.addWidget(self.status_label)

        self.cop_plot_widget = pg.PlotWidget(
            title="Center of Pressure on Contact Surface"
        )
        self.cop_plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self.cop_plot_widget.setLabel("bottom", "y", units="m")
        self.cop_plot_widget.setLabel("left", "z", units="m")
        self.cop_plot_widget.setAspectLocked(True)
        self.cop_plot_widget.setMinimumHeight(280)

        half_y = 0.5 * self.node.plate_size_y
        half_z = 0.5 * self.node.plate_size_z
        boundary_y = [-half_y, half_y, half_y, -half_y, -half_y]
        boundary_z = [-half_z, -half_z, half_z, half_z, -half_z]
        self.cop_boundary_item = self.cop_plot_widget.plot(
            boundary_y,
            boundary_z,
            pen=pg.mkPen(width=2),
            name="contact surface",
        )
        self.cop_center_item = pg.ScatterPlotItem(
            [0.0], [0.0], symbol="+", size=14, pen=pg.mkPen(width=2)
        )
        self.cop_plot_widget.addItem(self.cop_center_item)
        self.cop_trail_item = self.cop_plot_widget.plot([], [], pen=pg.mkPen(width=1))
        self.cop_point_item = pg.ScatterPlotItem(
            [],
            [],
            symbol="o",
            size=14,
            pen=pg.mkPen(width=2),
            brush=pg.mkBrush(255, 210, 0, 210),
        )
        self.cop_plot_widget.addItem(self.cop_point_item)
        self.cop_real_trail_item = self.cop_plot_widget.plot(
            [], [], pen=pg.mkPen(color="#00d9ff", width=1)
        )
        self.cop_real_point_item = pg.ScatterPlotItem(
            [],
            [],
            symbol="o",
            size=14,
            pen=pg.mkPen(color="#00d9ff", width=2),
            brush=pg.mkBrush(0, 217, 255, 210),
        )
        self.cop_plot_widget.addItem(self.cop_real_point_item)

        y_margin = max(0.02, 0.10 * self.node.plate_size_y)
        z_margin = max(0.02, 0.10 * self.node.plate_size_z)
        self.cop_plot_widget.setXRange(
            -half_y - y_margin, half_y + y_margin, padding=0.0
        )
        self.cop_plot_widget.setYRange(
            -half_z - z_margin, half_z + z_margin, padding=0.0
        )
        self.cop_status_label = QtWidgets.QLabel(
            f"CoP_hat waiting: {self.node.cop_hat_topic} | "
            f"CoP_real waiting: {self.node.cop_real_topic}"
        )
        plot_layout.addWidget(self.cop_plot_widget)
        plot_layout.addWidget(self.cop_status_label)

        self.plot_grid = QtWidgets.QGridLayout()
        plot_layout.addLayout(self.plot_grid, 1)
        self.plot_widgets: List[pg.PlotWidget] = []
        for panel_index in range(9):
            plot_widget = pg.PlotWidget(title=f"Plot {panel_index + 1}")
            plot_widget.showGrid(x=True, y=True, alpha=0.25)
            plot_widget.setLabel("bottom", "time", units="s")
            plot_widget.addLegend()
            plot_widget.setClipToView(True)
            plot_widget.setDownsampling(mode="peak")
            if self.plot_widgets:
                plot_widget.setXLink(self.plot_widgets[0])
            self.plot_widgets.append(plot_widget)
        self._apply_plot_layout()

        self.topic_filter_edit.textChanged.connect(self._rebuild_topic_list)
        self.topic_list.currentRowChanged.connect(self._on_topic_selection_changed)
        self.add_button.clicked.connect(self._add_selected_fields)
        self.layout_combo.currentIndexChanged.connect(self._on_layout_changed)
        self.field_list.itemDoubleClicked.connect(self._on_field_double_clicked)
        self.remove_button.clicked.connect(self._remove_selected_curves)
        self.clear_button.clicked.connect(self._clear_curves)
        self.refresh_topics_button.clicked.connect(self._refresh_topics)
        if self.spawn_window is not None:
            self.new_window_button.clicked.connect(self.spawn_window)

        interval_ms = max(10, int(1000.0 / max(self.node.refresh_hz, 1.0)))
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.start(interval_ms)

        self._refresh_topics()

    @property
    def active_panel_count(self) -> int:
        return self.plot_rows * self.plot_columns

    def _apply_plot_layout(self) -> None:
        for panel_index, plot_widget in enumerate(self.plot_widgets):
            self.plot_grid.removeWidget(plot_widget)
            if panel_index < self.active_panel_count:
                self.plot_grid.addWidget(
                    plot_widget,
                    panel_index // self.plot_columns,
                    panel_index % self.plot_columns,
                )
                plot_widget.show()
            else:
                plot_widget.hide()

        previous_panel = self.panel_combo.currentIndex()
        self.panel_combo.clear()
        self.panel_combo.addItems(
            [f"Plot {index + 1}" for index in range(self.active_panel_count)]
        )
        self.panel_combo.setCurrentIndex(
            max(0, min(previous_panel, self.active_panel_count - 1))
        )

    def _on_layout_changed(self, _: int) -> None:
        layout = self.layout_combo.currentData()
        if layout is None:
            return
        self.plot_rows, self.plot_columns = layout

        with self.node.lock:
            curves_to_move = [
                curve for curve in self.node.curves_by_key.values()
                if curve.panel_index >= self.active_panel_count
            ]
        for curve in curves_to_move:
            old_widget = self.plot_widgets[curve.panel_index]
            old_widget.removeItem(curve.plot_item)
            curve.panel_index %= self.active_panel_count
            curve.display_name = (
                f"[P{curve.panel_index + 1}] {curve.topic} :: {curve.field_path}"
            )
            new_widget = self.plot_widgets[curve.panel_index]
            new_widget.addItem(curve.plot_item)
            if new_widget.plotItem.legend is not None:
                new_widget.plotItem.legend.addItem(curve.plot_item, curve.display_name)

        self._apply_plot_layout()
        self._rebuild_active_curve_list()
        self._set_status_text(
            f"layout changed to {self.plot_rows} x {self.plot_columns}"
        )

    def _rebuild_active_curve_list(self) -> None:
        selected_names = {
            item.text() for item in self.active_curve_list.selectedItems()
        }
        self.active_curve_list.clear()
        with self.node.lock:
            display_names = [
                curve.display_name for curve in self.node.curves_by_key.values()
            ]
        for display_name in display_names:
            item = QtWidgets.QListWidgetItem(display_name)
            self.active_curve_list.addItem(item)
            if display_name in selected_names:
                item.setSelected(True)

    def _refresh_topics(self) -> None:
        if self.is_closing or not rclpy.ok():
            return
        previous_topic = self._selected_topic_name()
        previous_field_selection = self._selected_field_names()
        self.node.discover_topics()
        with self.node.lock:
            self.current_topic_names = list(self.node.topic_infos.keys())
        self._rebuild_topic_list()
        if previous_topic:
            self._select_topic_by_name(previous_topic)
            self._restore_field_selection(previous_field_selection)
        self._auto_add_default_topic_if_needed()

    def _rebuild_topic_list(self) -> None:
        selected_topic = self._selected_topic_name()
        pattern = self.topic_filter_edit.text().strip().lower()
        self.topic_list.blockSignals(True)
        self.topic_list.clear()
        for topic_name in self.current_topic_names:
            if pattern and pattern not in topic_name.lower():
                continue
            self.topic_list.addItem(topic_name)
        self.topic_list.blockSignals(False)

        if selected_topic:
            self._select_topic_by_name(selected_topic)
        elif self.topic_list.count() > 0:
            self.topic_list.setCurrentRow(0)
            self._on_topic_selection_changed(0)
        else:
            self.field_list.clear()
            self.topic_type_label.setText("type: -")

    def _selected_topic_name(self) -> str:
        item = self.topic_list.currentItem()
        return "" if item is None else item.text()

    def _select_topic_by_name(self, topic_name: str) -> None:
        matches = self.topic_list.findItems(topic_name, QtCore.Qt.MatchExactly)
        if matches:
            self.topic_list.setCurrentItem(matches[0])
            self._on_topic_selection_changed(self.topic_list.currentRow())

    def _on_topic_selection_changed(self, _: int) -> None:
        topic_name = self._selected_topic_name()
        with self.node.lock:
            topic_info = self.node.topic_infos.get(topic_name)
        previous_field_selection = self._selected_field_names()
        self.field_list.clear()
        if topic_info is None:
            self.topic_type_label.setText("type: -")
            return

        self.node.ensure_subscription(topic_name)
        self.topic_type_label.setText(f"type: {topic_info.type_name}")
        for field_name in topic_info.all_fields():
            item = QtWidgets.QListWidgetItem(field_name)
            self.field_list.addItem(item)
            if field_name in previous_field_selection:
                item.setSelected(True)

    def _auto_add_default_topic_if_needed(self) -> None:
        if self.initial_auto_add_done:
            return
        topic_name = self.node.default_topic or self._selected_topic_name()
        if not topic_name:
            return
        with self.node.lock:
            topic_info = self.node.topic_infos.get(topic_name)
        if topic_info is None:
            return

        self._select_topic_by_name(topic_info.name)
        added_count = 0
        for field_name in topic_info.all_fields():
            if added_count >= self.node.auto_add_limit:
                break
            if field_name.endswith("[]"):
                continue
            panel_index = added_count % self.active_panel_count
            if self._add_curve(topic_info.name, field_name, panel_index):
                added_count += 1
        if added_count > 0:
            suffix = ""
            if len(topic_info.all_fields()) > added_count:
                suffix = f" (limited to {added_count})"
            self._set_status_text(f"auto-added {added_count} field(s) from {topic_info.name}{suffix}")
            self.initial_auto_add_done = True

    def _add_selected_fields(self) -> None:
        topic_name = self._selected_topic_name()
        if not topic_name:
            self.status_label.setText("select a topic first")
            return
        selected_items = self.field_list.selectedItems()
        if not selected_items and self.field_list.currentItem() is not None:
            selected_items = [self.field_list.currentItem()]
        if not selected_items:
            self.status_label.setText("select one or more fields first")
            return

        added_count = 0
        for item in selected_items:
            if item.text().endswith("[]"):
                self.status_label.setText("wait for topic data so dynamic array indices appear")
                continue
            if self._add_curve(topic_name, item.text()):
                added_count += 1
        if added_count > 0:
            self.status_label.setText(f"added {added_count} field(s)")

    def _add_curve(
        self,
        topic_name: str,
        field_path: str,
        panel_index: Optional[int] = None,
    ) -> bool:
        key = f"{topic_name}:{field_path}"
        if key in self.node.curves_by_key:
            self.status_label.setText("field already added")
            return False

        with self.node.lock:
            topic_info = self.node.topic_infos.get(topic_name)
        if topic_info is None:
            self.status_label.setText("topic info is not available")
            return False

        self.node.ensure_subscription(topic_name)
        color = COLORS[self.color_index % len(COLORS)]
        self.color_index += 1
        if panel_index is None:
            panel_index = self.panel_combo.currentIndex()
        panel_index = max(0, min(panel_index, self.active_panel_count - 1))
        display_name = f"[P{panel_index + 1}] {topic_name} :: {field_path}"
        plot_item = self.plot_widgets[panel_index].plot(
            pen=pg.mkPen(color=color, width=2),
            name=display_name,
            autoDownsample=True,
            downsampleMethod="peak",
            clipToView=True,
        )
        record = CurveRecord(
            key=key,
            topic=topic_name,
            field_path=field_path,
            display_name=display_name,
            color=color,
            panel_index=panel_index,
            plot_item=plot_item,
            times=deque(maxlen=self.node.history_limit),
            values=deque(maxlen=self.node.history_limit),
        )
        with self.node.lock:
            self.node.curves_by_key[key] = record
        self.active_curve_list.addItem(display_name)
        with self.node.lock:
            topic_info = self.node.topic_infos.get(topic_name)
            if topic_info is not None and field_path in topic_info.latest_flattened:
                if self.node.start_time is None:
                    self.node.start_time = time.perf_counter()
                seeded_time = 0.0 if self.node.start_time is None else max(0.0, time.perf_counter() - self.node.start_time)
                record.times.append(seeded_time)
                record.values.append(topic_info.latest_flattened[field_path])
                record.dirty = True
        return True

    def _on_field_double_clicked(self, item: QtWidgets.QListWidgetItem) -> None:
        topic_name = self._selected_topic_name()
        if not topic_name:
            self.status_label.setText("select a topic first")
            return
        if item.text().endswith("[]"):
            self.status_label.setText("wait for topic data so dynamic array indices appear")
            return
        if self._add_curve(topic_name, item.text()):
            self.status_label.setText(f"added {item.text()}")

    def _remove_selected_curves(self) -> None:
        for item in list(self.active_curve_list.selectedItems()):
            self._remove_curve_by_name(item.text())

    def _remove_curve_by_name(self, display_name: str) -> None:
        key_to_remove = None
        with self.node.lock:
            curve_items = list(self.node.curves_by_key.items())
        for key, curve in curve_items:
            if curve.display_name == display_name:
                key_to_remove = key
                break
        if key_to_remove is None:
            return

        with self.node.lock:
            curve = self.node.curves_by_key.pop(key_to_remove)
        self.plot_widgets[curve.panel_index].removeItem(curve.plot_item)
        matches = self.active_curve_list.findItems(display_name, QtCore.Qt.MatchExactly)
        for item in matches:
            row = self.active_curve_list.row(item)
            self.active_curve_list.takeItem(row)

    def _clear_curves(self) -> None:
        with self.node.lock:
            display_names = [curve.display_name for curve in self.node.curves_by_key.values()]
        for display_name in display_names:
            self._remove_curve_by_name(display_name)

    def _update_plot(self) -> None:
        self._sync_selected_topic_fields()
        with self.node.lock:
            curve_records = list(self.node.curves_by_key.values())
        if not curve_records:
            self._set_status_text("no active curves")
            return

        latest_time = 0.0
        visible_values_by_panel: List[List[float]] = [
            [] for _ in self.plot_widgets
        ]
        latest_summaries: List[str] = []
        curves_with_data = 0
        curve_sample_counts: List[str] = []
        for curve in curve_records:
            with self.node.lock:
                dirty = curve.dirty
                times = list(curve.times)
                values = list(curve.values)
                curve.dirty = False
            if times and values:
                if dirty:
                    curve.plot_item.setData(times, values)
                latest_time = max(latest_time, times[-1])
                visible_values_by_panel[curve.panel_index].append(min(values))
                visible_values_by_panel[curve.panel_index].append(max(values))
                latest_summaries.append(f"{curve.field_path}={values[-1]:+.3f}")
                curve_sample_counts.append(f"{curve.field_path}:{len(values)}")
                curves_with_data += 1

        if curves_with_data == 0:
            first_curve = curve_records[0]
            with self.node.lock:
                topic_info = self.node.topic_infos.get(first_curve.topic)
            if topic_info is not None and topic_info.message_count > 0:
                self._set_status_text(
                    f"messages seen on {first_curve.topic} ({topic_info.message_count}), but `{first_curve.field_path}` has no numeric samples yet"
                )
            else:
                self._set_status_text(
                    f"waiting for messages on {first_curve.topic} | active curves={len(self.node.curves_by_key)}"
                )
            return

        xmin = max(0.0, latest_time - self.window_spin.value())
        xmax = latest_time if latest_time > xmin else xmin + 1.0
        self.plot_widgets[0].setXRange(xmin, xmax, padding=0.01)
        for panel_index, visible_values in enumerate(visible_values_by_panel):
            if not visible_values:
                continue
            ymin = min(visible_values)
            ymax = max(visible_values)
            if ymin == ymax:
                pad = 1.0 if abs(ymin) < 1e-9 else abs(ymin) * 0.1
                ymin -= pad
                ymax += pad
            else:
                pad = 0.1 * (ymax - ymin)
                ymin -= pad
                ymax += pad
            self.plot_widgets[panel_index].setYRange(ymin, ymax, padding=0.0)
        summary = " | ".join(latest_summaries[:4])
        if len(latest_summaries) > 4:
            summary += f" | +{len(latest_summaries) - 4} more"
        sample_summary = ", ".join(curve_sample_counts[:3])
        if len(curve_sample_counts) > 3:
            sample_summary += f", +{len(curve_sample_counts) - 3} more"
        self._set_status_text(
            (summary or "waiting for data...") + (f" | samples {sample_summary}" if sample_summary else "")
        )

    def _update_cop_plot(self) -> None:
        with self.node.lock:
            dirty = self.node.cop_dirty
            valid = self.node.cop_valid
            cop_y = self.node.cop_y
            cop_z = self.node.cop_z
            normal_force = self.node.cop_normal_force
            trail_y = list(self.node.cop_trail_y)
            trail_z = list(self.node.cop_trail_z)
            real_valid = self.node.cop_real_valid
            real_y = self.node.cop_real_y
            real_z = self.node.cop_real_z
            real_normal_force = self.node.cop_real_normal_force
            real_trail_y = list(self.node.cop_real_trail_y)
            real_trail_z = list(self.node.cop_real_trail_z)
            self.node.cop_dirty = False

        if not dirty:
            return
        if valid:
            self.cop_point_item.setData([cop_y], [cop_z])
            self.cop_trail_item.setData(trail_y, trail_z)
            half_y = 0.5 * self.node.plate_size_y
            half_z = 0.5 * self.node.plate_size_z
            inside = abs(cop_y) <= half_y and abs(cop_z) <= half_z
            region = "inside plate" if inside else "outside plate"
            self.cop_status_label.setText(
                f"CoP: y={cop_y:+.4f} m, z={cop_z:+.4f} m | "
                f"Fn={normal_force:.3f} N | {region}"
            )
        else:
            self.cop_point_item.setData([], [])
        if real_valid:
            self.cop_real_point_item.setData([real_y], [real_z])
            self.cop_real_trail_item.setData(real_trail_y, real_trail_z)
        else:
            self.cop_real_point_item.setData([], [])

        half_y = 0.5 * self.node.plate_size_y
        half_z = 0.5 * self.node.plate_size_z
        hat_region = "inside" if valid and abs(cop_y) <= half_y and abs(cop_z) <= half_z else "outside/invalid"
        real_region = "inside" if real_valid and abs(real_y) <= half_y and abs(real_z) <= half_z else "outside/invalid"
        self.cop_status_label.setText(
            f"CoP_hat: y={cop_y:+.4f}, z={cop_z:+.4f} m, Fn={normal_force:.3f} N ({hat_region}) | "
            f"CoP_real: y={real_y:+.4f}, z={real_z:+.4f} m, Fn={real_normal_force:.3f} N ({real_region})"
        )

    def _sync_selected_topic_fields(self) -> None:
        topic_name = self._selected_topic_name()
        with self.node.lock:
            topic_info = self.node.topic_infos.get(topic_name)
        if topic_info is None:
            return

        visible_fields = [self.field_list.item(index).text() for index in range(self.field_list.count())]
        expected_fields = topic_info.all_fields()
        if visible_fields == expected_fields:
            return

        selected_fields = {item.text() for item in self.field_list.selectedItems()}
        self.field_list.blockSignals(True)
        self.field_list.clear()
        for field_name in expected_fields:
            item = QtWidgets.QListWidgetItem(field_name)
            self.field_list.addItem(item)
            if field_name in selected_fields:
                item.setSelected(True)
        self.field_list.blockSignals(False)

    def _selected_field_names(self) -> Set[str]:
        return {item.text() for item in self.field_list.selectedItems()}

    def _restore_field_selection(self, field_names: Set[str]) -> None:
        if not field_names:
            return
        for index in range(self.field_list.count()):
            item = self.field_list.item(index)
            if item.text() in field_names:
                item.setSelected(True)

    def _set_status_text(self, text: str) -> None:
        if self.status_label.text() != text:
            self.status_label.setText(text)

    def _on_timer(self) -> None:
        if self.is_closing or not rclpy.ok():
            self.timer.stop()
            return
        try:
            now = time.perf_counter()
            if now - self.last_discovery_time >= self.node.discovery_period_sec:
                self.last_discovery_time = now
                self._refresh_topics()
            self._update_cop_plot()
            self._update_plot()
        except Exception as exc:
            self._set_status_text(f"plot update error: {exc}")
            if rclpy.ok():
                self.node.get_logger().error(f"plot update error: {exc}")

    def closeEvent(self, event) -> None:  # type: ignore[override]
        if self.is_closing:
            event.accept()
            return
        self.is_closing = True
        self.timer.stop()
        if self.node is not None:
            self.node.destroy_node()
        super().closeEvent(event)


def run_app(initial_topic: str = "") -> None:
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    executor = SingleThreadedExecutor()
    stop_event = threading.Event()
    windows: List[TopicPlotWindow] = []

    def create_window(topic_name: Optional[str] = None) -> TopicPlotWindow:
        node = TopicPlotNode(initial_topic=topic_name or initial_topic)
        node.executor = executor
        executor.add_node(node)
        window = TopicPlotWindow(node, spawn_window=create_window)
        windows.append(window)
        window.destroyed.connect(lambda *_ , w=window: windows.remove(w) if w in windows else None)
        window.show()
        return window

    def spin_executor() -> None:
        while not stop_event.is_set() and rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                break

    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()
    create_window()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    exit_code = app.exec()
    stop_event.set()
    executor.shutdown()
    if spin_thread.is_alive():
        spin_thread.join(timeout=1.0)
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


def main() -> None:
    run_app()


def external_wrench_main() -> None:
    run_app("/minitrone/external_wrench_hat_second_order")


if __name__ == "__main__":
    main()
