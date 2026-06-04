"""Tail / code chain TA tool for Maya 2026.

Run in Maya Script Editor:

    import tail_code_ta_tool
    tail_code_ta_tool.show()

The implementation is intentionally self-contained so it can be dropped into a
Maya scripts folder or loaded directly from the Script Editor.
"""

from __future__ import annotations

import csv
import json
import math
import traceback
from dataclasses import dataclass, field


try:
    from maya import cmds
    from maya.api import OpenMaya as om
    from maya import OpenMayaUI as omui
except Exception:  # Allows syntax checks outside Maya.
    cmds = None
    om = None
    omui = None

try:
    from PySide6 import QtCore, QtGui, QtWidgets
    import shiboken6
except Exception:
    try:
        from PySide2 import QtCore, QtGui, QtWidgets
        import shiboken2 as shiboken6
    except Exception:
        QtCore = QtGui = QtWidgets = shiboken6 = None


WINDOW_OBJECT_NAME = "tailCodeTAToolWindow"
DATA_NODE = "tailCodeTATool_sceneData"
DATA_ATTR = "chainsJson"
DISPLAY_GROUP = "tailCodeTATool_display_GRP"
TWEAKER_GROUP = "TailTweaker_GRP"
TWEAKER_PREFIX = "TailTweaker_"


COLORS = [
    (0.22, 0.70, 0.95),
    (0.95, 0.45, 0.18),
    (0.60, 0.82, 0.25),
    (0.78, 0.45, 0.92),
    (0.98, 0.76, 0.20),
    (0.25, 0.85, 0.72),
]


def _require_maya():
    if cmds is None or om is None:
        raise RuntimeError("This tool must be run inside Maya.")


def _safe_name(text):
    cleaned = []
    for ch in text:
        if ch.isalnum() or ch == "_":
            cleaned.append(ch)
        else:
            cleaned.append("_")
    return "".join(cleaned).strip("_") or "chain"


def _maya_main_window():
    if omui is None or QtWidgets is None:
        return None
    ptr = omui.MQtUtil.mainWindow()
    if ptr is None:
        return None
    return shiboken6.wrapInstance(int(ptr), QtWidgets.QWidget)


def _ensure_data_node():
    _require_maya()
    if not cmds.objExists(DATA_NODE):
        cmds.createNode("network", name=DATA_NODE)
    if not cmds.attributeQuery(DATA_ATTR, node=DATA_NODE, exists=True):
        cmds.addAttr(DATA_NODE, longName=DATA_ATTR, dataType="string")
    return DATA_NODE


def _joint_exists(name):
    return bool(cmds and name and cmds.objExists(name) and cmds.nodeType(name) == "joint")


def _world_position(joint):
    return tuple(float(v) for v in cmds.xform(joint, query=True, worldSpace=True, translation=True))


def _rotate_values(joint):
    values = cmds.getAttr(joint + ".rotate")[0]
    return tuple(float(v) for v in values)


def _joint_orient_values(joint):
    if not cmds.objExists(joint + ".jointOrient"):
        return (0.0, 0.0, 0.0)
    values = cmds.getAttr(joint + ".jointOrient")[0]
    return tuple(float(v) for v in values)


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _length(v):
    return math.sqrt(_dot(v, v))


def _angle_degrees(a, b):
    la = _length(a)
    lb = _length(b)
    if la <= 1e-8 or lb <= 1e-8:
        return 0.0
    c = max(-1.0, min(1.0, _dot(a, b) / (la * lb)))
    return math.degrees(math.acos(c))


def _dag_path(node):
    paths = cmds.ls(node, long=True) or []
    return paths[0] if paths else node


def _add_string_attr(node, attr, value):
    if not cmds.attributeQuery(attr, node=node, exists=True):
        cmds.addAttr(node, longName=attr, dataType="string")
    cmds.setAttr("%s.%s" % (node, attr), value, type="string")


def _add_bool_attr(node, attr, value=True):
    if not cmds.attributeQuery(attr, node=node, exists=True):
        cmds.addAttr(node, longName=attr, attributeType="bool")
    cmds.setAttr("%s.%s" % (node, attr), bool(value))


def _get_string_attr(node, attr, default=""):
    if not cmds.objExists(node) or not cmds.attributeQuery(attr, node=node, exists=True):
        return default
    return cmds.getAttr("%s.%s" % (node, attr)) or default


def _is_tweaker(node):
    return bool(cmds.objExists(node) and cmds.attributeQuery("tailTweaker", node=node, exists=True) and cmds.getAttr(node + ".tailTweaker"))


def _find_orient_constraint_driving(node):
    constraints = []
    for axis in "XYZ":
        attr = "%s.rotate%s" % (node, axis)
        if not cmds.objExists(attr):
            continue
        found = cmds.listConnections(attr, source=True, destination=False, type="orientConstraint") or []
        for constraint in found:
            if constraint not in constraints:
                constraints.append(constraint)
    return constraints[0] if constraints else None


def _attr_has_incoming_connection(attr):
    return bool(cmds.objExists(attr) and (cmds.listConnections(attr, source=True, destination=False) or []))


def _incoming_plug(attr):
    plugs = cmds.listConnections(attr, source=True, destination=False, plugs=True) or []
    return plugs[0] if plugs else ""


def sort_root_to_tip(joints):
    """Sort selected joints into root-to-tip order while preserving intent.

    If the joints are in one DAG chain, depth sorting gives root to tip. For
    mixed or partial selections, the user selection order is kept.
    """
    _require_maya()
    valid = []
    for joint in joints:
        if _joint_exists(joint) and joint not in valid:
            valid.append(joint)
    if len(valid) < 2:
        return valid

    long_paths = {j: _dag_path(j) for j in valid}
    depths = {j: long_paths[j].count("|") for j in valid}
    sorted_by_depth = sorted(valid, key=lambda j: depths[j])

    chain_ok = True
    for parent, child in zip(sorted_by_depth, sorted_by_depth[1:]):
        parent_path = long_paths[parent] + "|"
        if not long_paths[child].startswith(parent_path):
            chain_ok = False
            break
    return sorted_by_depth if chain_ok else valid


@dataclass
class ChainData:
    label: str
    joints: list[str]
    threshold: float = 15.0
    visible: bool = True
    color_index: int = 0
    last_score: float = 0.0
    problem_joints: list[str] = field(default_factory=list)

    def to_dict(self):
        return {
            "label": self.label,
            "joints": self.joints,
            "threshold": self.threshold,
            "visible": self.visible,
            "color_index": self.color_index,
        }

    @classmethod
    def from_dict(cls, data):
        return cls(
            label=str(data.get("label") or "chain"),
            joints=[str(j) for j in data.get("joints", []) if j],
            threshold=float(data.get("threshold", 15.0)),
            visible=bool(data.get("visible", True)),
            color_index=int(data.get("color_index", 0)),
        )


class JointAngleAnalyzer:
    @staticmethod
    def evaluate(joints, threshold):
        valid = [j for j in joints if _joint_exists(j)]
        positions = [_world_position(j) for j in valid]
        rows = []

        for index, joint in enumerate(valid):
            rotate = _rotate_values(joint)
            orient = _joint_orient_values(joint)
            rows.append(
                {
                    "joint": joint,
                    "rotate": rotate,
                    "joint_orient": orient,
                    "bend": 0.0,
                    "delta": (0.0, 0.0, 0.0),
                    "max_delta": 0.0,
                }
            )

        for i in range(1, len(positions) - 1):
            before = _sub(positions[i - 1], positions[i])
            after = _sub(positions[i + 1], positions[i])
            angle = _angle_degrees(before, after)
            rows[i]["bend"] = abs(180.0 - angle)

        axis_deltas = []
        problem_joints = []
        for i in range(1, len(rows)):
            prev_rotate = rows[i - 1]["rotate"]
            rotate = rows[i]["rotate"]
            delta = tuple(abs(rotate[axis] - prev_rotate[axis]) for axis in range(3))
            max_delta = max(delta)
            rows[i]["delta"] = delta
            rows[i]["max_delta"] = max_delta
            axis_deltas.append(delta)

        bend_values = [row["bend"] for row in rows]
        bend_deltas = []
        for i in range(1, len(rows)):
            delta = abs(rows[i]["bend"] - rows[i - 1]["bend"])
            bend_deltas.append(delta)
            if rows[i]["bend"] > threshold:
                problem_joints.append(rows[i]["joint"])

        score = sum(max(0.0, bend - threshold) for bend in bend_values)
        return {
            "score": score,
            "angle_rows": rows,
            "axis_deltas": axis_deltas,
            "max_deltas": [row["max_delta"] for row in rows],
            "bend_values": bend_values,
            "bend_deltas": bend_deltas,
            "problem_joints": problem_joints,
            "positions": positions,
        }


CurvatureAnalyzer = JointAngleAnalyzer


class HeatMapWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.values = []
        self.threshold = 15.0
        self.setMinimumHeight(52)

    def set_values(self, values, threshold):
        values = list(values)
        self.values = values[1:-1] if len(values) > 2 else []
        self.threshold = max(float(threshold), 0.001)
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor(36, 38, 42))
        if not self.values:
            painter.setPen(QtGui.QColor(150, 150, 150))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "曲がりデータなし")
            return

        count = len(self.values)
        width = max(1, self.width() / float(count))
        max_value = max(max(self.values), 1.0)
        for i, value in enumerate(self.values):
            ratio = max(0.0, min(1.0, value / max_value))
            r = int(55 + ratio * 200)
            g = int(170 - ratio * 120)
            b = int(105 - ratio * 45)
            painter.fillRect(QtCore.QRectF(i * width, 0, width + 1, self.height()), QtGui.QColor(r, g, b))


class BendGraphWidget(QtWidgets.QWidget):
    bendEditStarted = QtCore.Signal()
    bendEdited = QtCore.Signal(int, float)
    bendEditFinished = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.rows = []
        self.threshold = 15.0
        self.drag_index = None
        self.highlight_index = None
        self.setMinimumHeight(210)
        self.setMouseTracking(True)

    def set_angle_rows(self, rows, threshold):
        self.rows = list(rows)
        self.threshold = float(threshold)
        if self.highlight_index is not None and self.highlight_index >= len(self.rows):
            self.highlight_index = None
        self.update()

    def set_highlight_index(self, index):
        if index is None or not (0 <= index < len(self.rows)):
            self.highlight_index = None
        else:
            self.highlight_index = int(index)
        self.update()

    def _event_pos(self, event):
        return event.position() if hasattr(event, "position") else QtCore.QPointF(event.x(), event.y())

    def _plot_rect(self):
        return self.rect().adjusted(44, 18, -18, -34)

    def _max_value(self):
        values = [row["bend"] for row in self.rows]
        max_value = max(values + [self.threshold * 2.0, 1.0])
        return max(50.0, math.ceil(max_value * 1.25 / 10.0) * 10.0)

    def _point(self, index, value):
        plot = self._plot_rect()
        max_value = self._max_value()
        x = plot.left() + (index / max(1, len(self.rows) - 1)) * plot.width()
        y = plot.bottom() - (value / max_value) * plot.height()
        return QtCore.QPointF(x, y)

    def _value_from_y(self, y):
        plot = self._plot_rect()
        max_value = self._max_value()
        y = max(plot.top(), min(plot.bottom(), y))
        return max(0.0, (plot.bottom() - y) / max(1.0, plot.height()) * max_value)

    def _nearest_index(self, pos):
        if not self.rows:
            return None
        best_index = None
        best_distance = 999999.0
        for index, row in enumerate(self.rows):
            p = self._point(index, row["bend"])
            distance = math.hypot(pos.x() - p.x(), pos.y() - p.y())
            if distance < best_distance:
                best_index = index
                best_distance = distance
        return best_index if best_distance <= 14.0 else None

    def mousePressEvent(self, event):
        pos = self._event_pos(event)
        index = self._nearest_index(pos)
        if index is None:
            return
        self.drag_index = index
        self.bendEditStarted.emit()
        self.bendEdited.emit(index, self._value_from_y(pos.y()))

    def mouseMoveEvent(self, event):
        if self.drag_index is None:
            return
        pos = self._event_pos(event)
        self.bendEdited.emit(self.drag_index, self._value_from_y(pos.y()))

    def mouseReleaseEvent(self, event):
        if self.drag_index is not None:
            self.bendEditFinished.emit()
        self.drag_index = None

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect()
        painter.fillRect(rect, QtGui.QColor(28, 30, 34))
        plot = self._plot_rect()
        painter.setPen(QtGui.QColor(86, 90, 98))
        painter.drawRect(plot)

        if not self.rows:
            painter.setPen(QtGui.QColor(155, 155, 155))
            painter.drawText(rect, QtCore.Qt.AlignCenter, "曲がりグラフ")
            return

        values = [row["bend"] for row in self.rows]
        max_value = self._max_value()

        threshold_y = plot.bottom() - (self.threshold / max_value) * plot.height()
        painter.setPen(QtGui.QPen(QtGui.QColor(210, 80, 70), 1, QtCore.Qt.DashLine))
        painter.drawLine(QtCore.QPointF(plot.left(), threshold_y), QtCore.QPointF(plot.right(), threshold_y))

        path = QtGui.QPainterPath()
        for index, value in enumerate(values):
            p = self._point(index, value)
            if index == 0:
                path.moveTo(p)
            else:
                path.lineTo(p)
        painter.setPen(QtGui.QPen(QtGui.QColor(245, 190, 75), 2))
        painter.drawPath(path)

        for index, row in enumerate(self.rows):
            value = row["bend"]
            p = self._point(index, value)
            if value > self.threshold:
                painter.setBrush(QtGui.QColor(225, 70, 60))
                painter.setPen(QtGui.QColor(225, 70, 60))
                painter.drawEllipse(p, 5, 5)
            else:
                painter.setBrush(QtGui.QColor(245, 190, 75))
                painter.setPen(QtGui.QColor(245, 190, 75))
                painter.drawEllipse(p, 4, 4)
            if index == self.highlight_index:
                painter.setBrush(QtCore.Qt.NoBrush)
                painter.setPen(QtGui.QPen(QtGui.QColor(95, 190, 255), 3))
                painter.drawEllipse(p, 9, 9)

        painter.setPen(QtGui.QColor(210, 212, 216))
        painter.drawText(QtCore.QPointF(plot.left(), rect.bottom() - 13), "曲がり（点を上下ドラッグで調整）")
        painter.setPen(QtGui.QColor(225, 90, 80))
        painter.drawText(QtCore.QPointF(plot.left() + 190, rect.bottom() - 13), "しきい値超え")

        painter.setPen(QtGui.QColor(170, 174, 182))
        painter.drawText(QtCore.QPointF(8, plot.top() + 10), "%.0f" % max_value)
        painter.drawText(QtCore.QPointF(12, plot.bottom()), "0")


class ScoreGraphWidget(QtWidgets.QWidget):
    frameClicked = QtCore.Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.results = {}
        self.threshold = 15.0
        self.setMinimumHeight(160)

    def set_results(self, results, threshold):
        self.results = results
        self.threshold = float(threshold)
        self.update()

    def mousePressEvent(self, event):
        if not self.results:
            return
        frames = sorted({f for scores in self.results.values() for f in scores})
        if not frames:
            return
        margin = 32
        usable = max(1, self.width() - margin * 2)
        x = max(margin, min(self.width() - margin, event.position().x() if hasattr(event, "position") else event.x()))
        index = int(round((x - margin) / usable * (len(frames) - 1)))
        self.frameClicked.emit(frames[max(0, min(index, len(frames) - 1))])

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect()
        painter.fillRect(rect, QtGui.QColor(30, 32, 36))
        margin = 32
        plot = rect.adjusted(margin, 16, -16, -28)
        painter.setPen(QtGui.QColor(90, 92, 98))
        painter.drawRect(plot)
        if not self.results:
            painter.setPen(QtGui.QColor(150, 150, 150))
            painter.drawText(rect, QtCore.Qt.AlignCenter, "スキャン結果なし")
            return

        frames = sorted({f for scores in self.results.values() for f in scores})
        max_score = max([self.threshold] + [v for scores in self.results.values() for v in scores.values()])
        max_score = max(max_score, 1.0)

        def point(frame, score):
            xi = frames.index(frame) if frame in frames else 0
            x = plot.left() + (xi / max(1, len(frames) - 1)) * plot.width()
            y = plot.bottom() - (score / max_score) * plot.height()
            return QtCore.QPointF(x, y)

        threshold_y = plot.bottom() - (self.threshold / max_score) * plot.height()
        painter.setPen(QtGui.QPen(QtGui.QColor(220, 80, 70), 1, QtCore.Qt.DashLine))
        painter.drawLine(QtCore.QPointF(plot.left(), threshold_y), QtCore.QPointF(plot.right(), threshold_y))

        for chain_index, (label, scores) in enumerate(self.results.items()):
            color = COLORS[chain_index % len(COLORS)]
            qcolor = QtGui.QColor.fromRgbF(*color)
            painter.setPen(QtGui.QPen(qcolor, 2))
            path = QtGui.QPainterPath()
            sorted_scores = sorted(scores.items())
            for i, (frame, score) in enumerate(sorted_scores):
                p = point(frame, score)
                if i == 0:
                    path.moveTo(p)
                else:
                    path.lineTo(p)
            painter.drawPath(path)
            painter.setBrush(qcolor)
            for frame, score in sorted_scores:
                if score > self.threshold:
                    painter.drawEllipse(point(frame, score), 4, 4)


class TailCodeTATool(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        _require_maya()
        self.setObjectName(WINDOW_OBJECT_NAME)
        self.setWindowTitle("尻尾・コード TA ツール")
        self.resize(920, 820)
        self.chains = []
        self.callbacks = []
        self.is_monitoring = False
        self._monitored_joint_paths = set()
        self._pending_score_update = False
        self._tweaker_scan_cache = None
        self.scan_results = {}
        self.graph_undo_open = False
        self._build_ui()
        self._install_undo_redo_shortcuts()
        self.load_scene_data()
        self.refresh_all()

    def _install_undo_redo_shortcuts(self):
        undo_shortcut = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+Z"), self)
        undo_shortcut.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
        undo_shortcut.activated.connect(self._maya_undo)

        redo_shortcut = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+Shift+Z"), self)
        redo_shortcut.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
        redo_shortcut.activated.connect(self._maya_redo)

        redo_y_shortcut = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+Y"), self)
        redo_y_shortcut.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
        redo_y_shortcut.activated.connect(self._maya_redo)

    def _maya_undo(self):
        self.end_graph_undo()
        try:
            cmds.undo()
            self.update_scores_and_display()
        except Exception as exc:
            self.tweaker_status.setText("Undo できません: %s" % exc)

    def _maya_redo(self):
        self.end_graph_undo()
        try:
            cmds.redo()
            self.update_scores_and_display()
        except Exception as exc:
            self.tweaker_status.setText("Redo できません: %s" % exc)

    def _build_ui(self):
        main = QtWidgets.QVBoxLayout(self)

        register_box = QtWidgets.QGroupBox("チェーン登録")
        reg_layout = QtWidgets.QGridLayout(register_box)
        self.label_edit = QtWidgets.QLineEdit()
        self.label_edit.setPlaceholderText("例：右手_親指 / 尻尾A")
        self.joints_edit = QtWidgets.QPlainTextEdit()
        self.joints_edit.setPlaceholderText("ジョイント名を1行ずつ入力。空欄の場合は現在の選択を使います。")
        self.joints_edit.setMaximumHeight(78)
        self.threshold_spin = QtWidgets.QDoubleSpinBox()
        self.threshold_spin.setRange(0.01, 9999.0)
        self.threshold_spin.setValue(15.0)
        self.threshold_spin.setDecimals(2)
        self.threshold_spin.setSuffix(" 度（曲がり）")
        add_button = QtWidgets.QPushButton("選択チェーンを登録")
        add_button.clicked.connect(self.register_chain)
        update_button = QtWidgets.QPushButton("選択チェーンを更新")
        update_button.clicked.connect(self.update_selected_chain)
        remove_button = QtWidgets.QPushButton("削除")
        remove_button.clicked.connect(self.remove_selected_chain)
        reg_layout.addWidget(QtWidgets.QLabel("ラベル"), 0, 0)
        reg_layout.addWidget(self.label_edit, 0, 1, 1, 3)
        reg_layout.addWidget(QtWidgets.QLabel("ジョイント"), 1, 0)
        reg_layout.addWidget(self.joints_edit, 1, 1, 1, 3)
        reg_layout.addWidget(QtWidgets.QLabel("角度差しきい値"), 2, 0)
        reg_layout.addWidget(self.threshold_spin, 2, 1)
        reg_layout.addWidget(add_button, 2, 2)
        reg_layout.addWidget(update_button, 2, 3)
        reg_layout.addWidget(remove_button, 3, 3)
        main.addWidget(register_box)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        self.chain_list = QtWidgets.QTreeWidget()
        self.chain_list.setHeaderLabels(["登録済みチェーン / ジョイント"])
        self.chain_list.setRootIsDecorated(True)
        self.chain_list.currentItemChanged.connect(self.on_chain_selected)
        self.chain_list.itemChanged.connect(self._visibility_changed)
        left_layout.addWidget(QtWidgets.QLabel("登録済みチェーン（上ほど優先）"))
        left_layout.addWidget(self.chain_list)
        priority_layout = QtWidgets.QHBoxLayout()
        up_button = QtWidgets.QPushButton("上へ")
        up_button.clicked.connect(self.move_selected_chain_up)
        down_button = QtWidgets.QPushButton("下へ")
        down_button.clicked.connect(self.move_selected_chain_down)
        priority_layout.addWidget(up_button)
        priority_layout.addWidget(down_button)
        left_layout.addLayout(priority_layout)
        splitter.addWidget(left)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        self.score_label = QtWidgets.QLabel("曲がりスコア: -")
        self.angle_table = QtWidgets.QTableWidget(0, 9)
        self.angle_table.setHorizontalHeaderLabels(
            ["ジョイント", "曲がり", "回転 X", "回転 Y", "回転 Z", "差分 X", "差分 Y", "差分 Z", "最大差"]
        )
        self.angle_table.horizontalHeader().setStretchLastSection(True)
        self.angle_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        for column in range(1, 8):
            self.angle_table.horizontalHeader().setSectionResizeMode(column, QtWidgets.QHeaderView.ResizeToContents)
        self.angle_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.angle_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.angle_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.angle_table.itemSelectionChanged.connect(self.on_angle_table_selection_changed)
        self.angle_table.cellClicked.connect(self.on_angle_table_cell_clicked)
        self.angle_table.currentCellChanged.connect(self.on_angle_table_current_cell_changed)
        self.angle_table.setMinimumHeight(190)
        self.heatmap = HeatMapWidget()
        self.angle_graph = BendGraphWidget()
        self.angle_graph.bendEditStarted.connect(self.begin_graph_undo)
        self.angle_graph.bendEdited.connect(self.apply_bend_edit)
        self.angle_graph.bendEditFinished.connect(self.end_graph_undo)
        self.graph = ScoreGraphWidget()
        self.graph.frameClicked.connect(self.goto_frame)
        match_layout = QtWidgets.QHBoxLayout()
        match_button = QtWidgets.QPushButton("選択曲がりに近似")
        match_button.clicked.connect(self.match_bend_to_selected)
        align_button = QtWidgets.QPushButton("選択曲がりに揃える")
        align_button.clicked.connect(self.align_bend_to_selected)
        add_button = QtWidgets.QPushButton("曲がり差分を加算")
        add_button.clicked.connect(self.add_bend_delta_to_others)
        match_layout.addWidget(match_button)
        match_layout.addWidget(align_button)
        match_layout.addWidget(add_button)
        match_layout.addStretch()
        right_layout.addWidget(self.score_label)
        right_layout.addWidget(self.angle_table)
        right_layout.addLayout(match_layout)
        right_layout.addWidget(QtWidgets.QLabel("現在フレームの曲がり折れ線グラフ"))
        right_layout.addWidget(self.angle_graph)
        right_layout.addWidget(QtWidgets.QLabel("曲がりヒートマップ"))
        right_layout.addWidget(self.heatmap)
        right_layout.addWidget(QtWidgets.QLabel("範囲スキャン結果"))
        right_layout.addWidget(self.graph)
        splitter.addWidget(right)
        splitter.setStretchFactor(1, 2)
        main.addWidget(splitter, 1)

        monitor_layout = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("監視開始")
        self.stop_button = QtWidgets.QPushButton("監視停止")
        self.start_button.clicked.connect(self.start_monitoring)
        self.stop_button.clicked.connect(self.stop_monitoring)
        self.stop_button.setEnabled(False)
        monitor_layout.addWidget(self.start_button)
        monitor_layout.addWidget(self.stop_button)
        monitor_layout.addStretch()
        main.addLayout(monitor_layout)

        tweaker_box = QtWidgets.QGroupBox("Tail Tweaker")
        tweaker_layout = QtWidgets.QGridLayout(tweaker_box)
        delete_tweaker_button = QtWidgets.QPushButton("選択 Tweaker 削除")
        delete_tweaker_button.clicked.connect(self.delete_selected_tweakers)
        bake_tweaker_button = QtWidgets.QPushButton("選択 Tweaker Bake")
        bake_tweaker_button.clicked.connect(self.bake_selected_tweakers)
        self.tweaker_status = QtWidgets.QLabel("チェーン登録時に Tweaker を自動作成します。表またはグラフでジョイントを選び、曲がりを調整できます。")
        self.tweaker_status.setWordWrap(True)
        tweaker_layout.addWidget(delete_tweaker_button, 0, 0)
        tweaker_layout.addWidget(bake_tweaker_button, 0, 1)
        tweaker_layout.addWidget(self.tweaker_status, 1, 0, 1, 2)
        main.addWidget(tweaker_box)

        scan_box = QtWidgets.QGroupBox("アニメーション範囲評価")
        scan_layout = QtWidgets.QGridLayout(scan_box)
        self.start_frame = QtWidgets.QSpinBox()
        self.end_frame = QtWidgets.QSpinBox()
        for spin in (self.start_frame, self.end_frame):
            spin.setRange(-100000, 100000)
        self.start_frame.setValue(int(cmds.playbackOptions(query=True, minTime=True)))
        self.end_frame.setValue(int(cmds.playbackOptions(query=True, maxTime=True)))
        self.scan_progress = QtWidgets.QProgressBar()
        self.cancel_scan_button = QtWidgets.QPushButton("キャンセル")
        self.cancel_scan_button.setEnabled(False)
        self.cancel_scan = False
        self.cancel_scan_button.clicked.connect(self._cancel_scan)
        scan_button = QtWidgets.QPushButton("範囲スキャン")
        scan_button.clicked.connect(self.scan_range)
        export_button = QtWidgets.QPushButton("CSV書き出し")
        export_button.clicked.connect(self.export_csv)
        scan_layout.addWidget(QtWidgets.QLabel("開始"), 0, 0)
        scan_layout.addWidget(self.start_frame, 0, 1)
        scan_layout.addWidget(QtWidgets.QLabel("終了"), 0, 2)
        scan_layout.addWidget(self.end_frame, 0, 3)
        scan_layout.addWidget(scan_button, 0, 4)
        scan_layout.addWidget(export_button, 0, 5)
        scan_layout.addWidget(self.scan_progress, 1, 0, 1, 5)
        scan_layout.addWidget(self.cancel_scan_button, 1, 5)
        main.addWidget(scan_box)

    def _selected_chain_index(self):
        item = self.chain_list.currentItem()
        if item is None:
            return -1
        if item.parent() is not None:
            item = item.parent()
        index = self.chain_list.indexOfTopLevelItem(item)
        return index if 0 <= index < len(self.chains) else -1

    def _selected_chain(self):
        index = self._selected_chain_index()
        if 0 <= index < len(self.chains):
            return self.chains[index]
        return None

    def _unique_chain_label(self, label, exclude_index=None):
        base = (label or "").strip() or "chain_%02d" % (len(self.chains) + 1)
        used = {chain.label for i, chain in enumerate(self.chains) if i != exclude_index}
        if base not in used:
            return base
        suffix = 2
        while True:
            candidate = "%s_%02d" % (base, suffix)
            if candidate not in used:
                return candidate
            suffix += 1

    def _input_joints(self):
        raw = self.joints_edit.toPlainText().strip()
        if raw:
            joints = [line.strip() for line in raw.replace(",", "\n").splitlines() if line.strip()]
        else:
            joints = cmds.ls(selection=True, type="joint") or []
        return sort_root_to_tip(joints)

    def register_chain(self):
        try:
            joints = self._input_joints()
            if len(joints) < 3:
                cmds.warning("3つ以上のジョイントを指定してください。")
                return
            requested_label = self.label_edit.text().strip() or "chain_%02d" % (len(self.chains) + 1)
            label = self._unique_chain_label(requested_label)
            chain = ChainData(
                label=label,
                joints=joints,
                threshold=float(self.threshold_spin.value()),
                color_index=len(self.chains) % len(COLORS),
            )
            self.chains.append(chain)
            # Reason: chain-owned tweakers prevent overlap from reusing owner-less joint tweakers.
            created = self._create_tweakers_for_chain(chain)
            if created:
                self.tweaker_status.setText("チェーン登録時に Tweaker 作成: %s" % ", ".join(created))
            self.save_scene_data()
            self.refresh_all()
            self._register_callbacks_for_chain(chain)
            self.label_edit.setText(chain.label)
            self.joints_edit.clear()
        except Exception:
            self._show_error("チェーン登録に失敗しました")

    def update_selected_chain(self):
        chain = self._selected_chain()
        if chain is None:
            return
        try:
            joints = self._input_joints()
            if len(joints) >= 3:
                chain.joints = joints
            index = self._selected_chain_index()
            old_label = chain.label
            chain.label = self._unique_chain_label(self.label_edit.text().strip() or chain.label, exclude_index=index)
            # Reason: label edits must keep existing chain-owned tweakers addressable.
            self._update_chain_tweaker_owner_label(old_label, chain.label)
            chain.threshold = float(self.threshold_spin.value())
            # Reason: newly added joints in an updated chain need owner-specific tweakers.
            created = self._create_tweakers_for_chain(chain)
            if created:
                self.tweaker_status.setText("チェーン更新時に Tweaker 作成: %s" % ", ".join(created))
            self.save_scene_data()
            self.restart_monitoring_if_needed()
            self.refresh_all()
            self.label_edit.setText(chain.label)
            self.joints_edit.clear()
        except Exception:
            self._show_error("チェーン更新に失敗しました")

    def remove_selected_chain(self):
        row = self._selected_chain_index()
        if not (0 <= row < len(self.chains)):
            return
        chain = self.chains.pop(row)
        deleted = self._delete_tweakers_for_chain(chain)
        if deleted:
            self.tweaker_status.setText("チェーン削除時に Tweaker 削除: %s" % ", ".join(deleted))
        self._delete_display_curve(chain)
        self.save_scene_data()
        self.restart_monitoring_if_needed()
        self.refresh_all()

    def on_chain_selected(self, current=None, previous=None):
        chain = self._selected_chain()
        if chain is None:
            return
        self.label_edit.setText(chain.label)
        self.joints_edit.setPlainText("\n".join(chain.joints))
        self.threshold_spin.setValue(chain.threshold)
        self.refresh_details(chain)

    def refresh_all(self):
        self.chain_list.blockSignals(True)
        current = self._selected_chain_index()
        self.chain_list.clear()
        # Reason: tree rebuild asks for tweakers per joint; cache one scene scan for this refresh only.
        self._tweaker_scan_cache = self._scan_all_tweakers()
        try:
            for index, chain in enumerate(self.chains):
                label = "%02d  %s" % (index + 1, chain.label)
                item = QtWidgets.QTreeWidgetItem([label])
                item.setData(0, QtCore.Qt.UserRole, index)
                item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
                item.setCheckState(0, QtCore.Qt.Checked if chain.visible else QtCore.Qt.Unchecked)
                color = QtGui.QColor.fromRgbF(*COLORS[chain.color_index % len(COLORS)])
                item.setForeground(0, color)
                self.chain_list.addTopLevelItem(item)
                for joint in chain.joints:
                    tweakers = self._tweakers_for_joint(joint)
                    suffix = "  ->  %s" % tweakers[0] if tweakers else ""
                    child = QtWidgets.QTreeWidgetItem(["%s%s" % (joint, suffix)])
                    child.setData(0, QtCore.Qt.UserRole, index)
                    child.setForeground(0, QtGui.QColor(185, 188, 194))
                    item.addChild(child)
                item.setExpanded(True)
        finally:
            self._tweaker_scan_cache = None
            self.chain_list.blockSignals(False)
        if self.chains:
            index = max(0, min(current, len(self.chains) - 1))
            self.chain_list.setCurrentItem(self.chain_list.topLevelItem(index))
        self.update_scores_and_display()

    def _visibility_changed(self, item, column=0):
        if item is None:
            return
        if item.parent() is not None:
            return
        row = self.chain_list.indexOfTopLevelItem(item)
        if 0 <= row < len(self.chains):
            self.chains[row].visible = item.checkState(0) == QtCore.Qt.Checked
            self.save_scene_data()
            self.update_scores_and_display()

    def move_selected_chain_up(self):
        self._move_selected_chain(-1)

    def move_selected_chain_down(self):
        self._move_selected_chain(1)

    def _move_selected_chain(self, offset):
        index = self._selected_chain_index()
        new_index = index + offset
        if not (0 <= index < len(self.chains)) or not (0 <= new_index < len(self.chains)):
            return
        self.chains[index], self.chains[new_index] = self.chains[new_index], self.chains[index]
        self.save_scene_data()
        self.refresh_all()
        self.chain_list.setCurrentItem(self.chain_list.topLevelItem(new_index))
        self.tweaker_status.setText("チェーン優先順位を更新しました。上にあるチェーンの Tweaker を優先します。")

    def begin_graph_undo(self):
        if self.graph_undo_open:
            return
        try:
            cmds.undoInfo(openChunk=True, chunkName="TA Tool Graph Bend Edit")
            self.graph_undo_open = True
        except Exception:
            self.graph_undo_open = False

    def end_graph_undo(self):
        if not self.graph_undo_open:
            return
        try:
            cmds.undoInfo(closeChunk=True)
        finally:
            self.graph_undo_open = False

    def refresh_details(self, chain):
        result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold)
        chain.last_score = result["score"]
        chain.problem_joints = result["problem_joints"]
        self.score_label.setText("曲がりスコア: %.3f    チェーン: %s" % (chain.last_score, chain.label))
        self._fill_angle_table(result["angle_rows"], chain.threshold)
        self.angle_graph.set_angle_rows(result["angle_rows"], chain.threshold)
        self.heatmap.set_values(result["bend_values"], chain.threshold)

    def apply_bend_edit(self, row_index, target_bend):
        changed = self._apply_bend_edit(row_index, target_bend, select_target=True)
        if changed:
            self.update_scores_and_display()

    def _apply_bend_edit(self, row_index, target_bend, select_target=False):
        chain = self._selected_chain()
        if chain is None:
            return False
        result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold)
        rows = result["angle_rows"]
        if not (0 <= row_index < len(rows)):
            return False
        if row_index == 0 or row_index == len(rows) - 1:
            self.tweaker_status.setText("Root/Tip は前後ジョイントが足りないため曲がり調整できません。")
            return False

        row = rows[row_index]
        joint = row["joint"]
        current_bend = max(row["bend"], 0.001)
        ratio = max(0.0, min(3.0, float(target_bend) / current_bend))
        if abs(ratio - 1.0) < 0.005:
            return False

        rotate = row["rotate"]
        direction = rotate
        if max(abs(v) for v in direction) < 0.001 and row_index > 0:
            prev_rotate = rows[row_index - 1]["rotate"]
            direction = tuple(rotate[i] - prev_rotate[i] for i in range(3))
        if max(abs(v) for v in direction) < 0.001:
            self.tweaker_status.setText("XYZ 比率を保つための回転方向がありません。先に少し回転を付けてください。")
            return False

        delta = tuple(direction[i] * (ratio - 1.0) for i in range(3))
        tweakers = self._tweakers_for_joint(joint)
        has_incoming = any(_attr_has_incoming_connection("%s.rotate%s" % (joint, axis)) for axis in "XYZ")

        if tweakers:
            target_node = tweakers[0]
            current_tweaker_rotate = _rotate_values(target_node)
            new_values = tuple(current_tweaker_rotate[i] + delta[i] for i in range(3))
            self._set_rotate_values(target_node, new_values)
        elif has_incoming:
            target_node = self._create_tweaker(joint)
            self._set_rotate_values(target_node, delta)
        else:
            new_values = tuple(rotate[i] * ratio for i in range(3))
            self._set_rotate_values(joint, new_values)
            target_node = joint

        if select_target:
            cmds.select(target_node, replace=True)
        self.tweaker_status.setText(
            "曲がり調整: %s  %.2f -> %.2f / XYZ 比率維持" % (joint, row["bend"], target_bend)
        )
        return True

    def _bend_direction(self, rows, row_index):
        row = rows[row_index]
        direction = row["rotate"]
        if max(abs(v) for v in direction) < 0.001 and row_index > 0:
            prev_rotate = rows[row_index - 1]["rotate"]
            direction = tuple(row["rotate"][i] - prev_rotate[i] for i in range(3))
        max_axis = max(abs(v) for v in direction)
        if max_axis < 0.001:
            return None
        return tuple(v / max_axis for v in direction)

    def _apply_bend_delta_edit(self, row_index, bend_delta, select_target=False):
        chain = self._selected_chain()
        if chain is None:
            return False
        rows = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold)["angle_rows"]
        if not (0 <= row_index < len(rows)):
            return False
        if row_index == 0 or row_index == len(rows) - 1:
            return False

        row = rows[row_index]
        direction = self._bend_direction(rows, row_index)
        if direction is None:
            return False

        joint = row["joint"]
        delta = tuple(direction[i] * bend_delta for i in range(3))
        tweakers = self._tweakers_for_joint(joint)
        has_incoming = any(_attr_has_incoming_connection("%s.rotate%s" % (joint, axis)) for axis in "XYZ")

        if tweakers:
            target_node = tweakers[0]
            current_values = _rotate_values(target_node)
            self._set_rotate_values(target_node, tuple(current_values[i] + delta[i] for i in range(3)))
        elif has_incoming:
            target_node = self._create_tweaker(joint)
            current_values = _rotate_values(target_node)
            self._set_rotate_values(target_node, tuple(current_values[i] + delta[i] for i in range(3)))
        else:
            target_node = joint
            current_values = _rotate_values(joint)
            self._set_rotate_values(joint, tuple(current_values[i] + delta[i] for i in range(3)))

        if select_target:
            cmds.select(target_node, replace=True)
        return True

    def match_bend_to_selected(self):
        self._match_or_align_bend_to_selected(iterations=1, tolerance=0.25, exact=False)

    def align_bend_to_selected(self):
        self._match_or_align_bend_to_selected(iterations=6, tolerance=0.10, exact=True)

    def _match_or_align_bend_to_selected(self, iterations=1, tolerance=0.25, exact=False):
        chain = self._selected_chain()
        if chain is None:
            cmds.warning("曲がりを揃えるチェーンを選択してください。")
            return
        selected_rows = sorted({item.row() for item in self.angle_table.selectedItems()})
        if not selected_rows:
            cmds.warning("基準にするジョイント行を選択してください。")
            return

        result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold)
        rows = result["angle_rows"]
        source_index = selected_rows[0]
        if not (0 < source_index < len(rows) - 1):
            self.tweaker_status.setText("Root/Tip は基準にできません。Root/Tip 以外のジョイントを選択してください。")
            return

        target_bend = rows[source_index]["bend"]
        changed = set()
        skipped = set()
        action_name = "TA Tool Align Bend" if exact else "TA Tool Match Bend"
        cmds.undoInfo(openChunk=True, chunkName=action_name)
        try:
            for index in range(1, len(rows) - 1):
                if index == source_index:
                    continue
                adjusted = False
                for _ in range(max(1, int(iterations))):
                    current_rows = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold)["angle_rows"]
                    if index >= len(current_rows):
                        break
                    current_bend = current_rows[index]["bend"]
                    if abs(current_bend - target_bend) <= tolerance:
                        adjusted = True
                        break
                    if self._apply_bend_edit(index, target_bend, select_target=False):
                        adjusted = True
                    else:
                        break
                if adjusted:
                    changed.add(index)
                else:
                    skipped.add(index)
        finally:
            cmds.undoInfo(closeChunk=True)

        final_rows = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold)["angle_rows"]
        errors = [abs(final_rows[i]["bend"] - target_bend) for i in changed if i < len(final_rows)]
        max_error = max(errors) if errors else 0.0
        self.angle_graph.set_highlight_index(source_index)
        label = "選択曲がりに揃える" if exact else "選択曲がりに近似"
        self.tweaker_status.setText(
            "%s: 基準 %s %.2f 度 / 調整 %d件 / スキップ %d件 / 最大差 %.2f 度"
            % (label, rows[source_index]["joint"], target_bend, len(changed), len(skipped), max_error)
        )
        self.update_scores_and_display()

    def add_bend_delta_to_others(self):
        chain = self._selected_chain()
        if chain is None:
            cmds.warning("曲がり差分を加算するチェーンを選択してください。")
            return
        selected_rows = sorted({item.row() for item in self.angle_table.selectedItems()})
        if not selected_rows:
            cmds.warning("基準にするジョイント行を選択してください。")
            return

        result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold)
        rows = result["angle_rows"]
        source_index = selected_rows[0]
        if not (0 < source_index < len(rows) - 1):
            self.tweaker_status.setText("Root/Tip は曲がり値が取れないため基準にできません。中間ジョイントを選択してください。")
            return

        target_bend = rows[source_index]["bend"]
        changed = 0
        skipped = 0
        cmds.undoInfo(openChunk=True, chunkName="TA Tool Add Bend Delta")
        try:
            for index in range(1, len(rows) - 1):
                if index == source_index:
                    continue
                current_bend = rows[index]["bend"]
                bend_delta = target_bend - current_bend
                if abs(bend_delta) < 0.01:
                    skipped += 1
                    continue
                if self._apply_bend_delta_edit(index, bend_delta, select_target=False):
                    changed += 1
                else:
                    skipped += 1
        finally:
            cmds.undoInfo(closeChunk=True)

        self.angle_graph.set_highlight_index(source_index)
        self.tweaker_status.setText(
            "曲がり差分を加算: 基準 %s %.2f 度 / 調整 %d件 / スキップ %d件"
            % (rows[source_index]["joint"], target_bend, changed, skipped)
        )
        self.update_scores_and_display()

    def _set_rotate_values(self, node, values):
        for axis, value in zip("XYZ", values):
            attr = "%s.rotate%s" % (node, axis)
            if cmds.objExists(attr) and not cmds.getAttr(attr, lock=True):
                if not _attr_has_incoming_connection(attr):
                    cmds.setAttr(attr, value)

    def on_angle_table_cell_clicked(self, row, column):
        self._select_joint_from_angle_table_row(row)

    def on_angle_table_current_cell_changed(self, current_row, current_column, previous_row, previous_column):
        if current_row >= 0:
            self._select_joint_from_angle_table_row(current_row)

    def on_angle_table_selection_changed(self):
        selected_rows = sorted({item.row() for item in self.angle_table.selectedItems()})
        if not selected_rows:
            self.angle_graph.set_highlight_index(None)
            return
        self._select_joint_from_angle_table_row(selected_rows[0])

    def _select_joint_from_angle_table_row(self, row_index):
        if row_index < 0 or row_index >= self.angle_table.rowCount():
            self.angle_graph.set_highlight_index(None)
            return

        item = self.angle_table.item(row_index, 0)
        joint = item.text() if item else ""
        if not joint:
            chain = self._selected_chain()
            joint = chain.joints[row_index] if chain and 0 <= row_index < len(chain.joints) else ""

        self.angle_graph.set_highlight_index(row_index)
        if not _joint_exists(joint):
            self.tweaker_status.setText("ジョイントが見つかりません: %s" % joint)
            return

        tweakers = self._tweakers_for_joint(joint)
        if tweakers:
            select_name = tweakers[0]
            status = "選択 Tweaker: %s  / 対象: %s" % (select_name, joint)
        else:
            long_names = cmds.ls(joint, long=True) or [joint]
            select_name = long_names[0]
            status = "Tweaker がないため元ジョイントを選択: %s" % joint

        def _deferred_select():
            if cmds.objExists(select_name):
                cmds.select(select_name, replace=True)

        try:
            cmds.evalDeferred(_deferred_select)
        except Exception:
            cmds.select(select_name, replace=True)
        self.tweaker_status.setText(status)


    def _fill_angle_table(self, rows, threshold):
        self.angle_table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            values = [
                row["joint"],
                "%.2f" % row["bend"],
                "%.2f" % row["rotate"][0],
                "%.2f" % row["rotate"][1],
                "%.2f" % row["rotate"][2],
                "%.2f" % row["delta"][0],
                "%.2f" % row["delta"][1],
                "%.2f" % row["delta"][2],
                "%.2f" % row["max_delta"],
            ]
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                if column > 0:
                    item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                self.angle_table.setItem(row_index, column, item)

    def update_scores_and_display(self):
        selected = self._selected_chain()
        for chain in self.chains:
            if not chain.visible:
                self._delete_display_curve(chain)
                continue
            result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold)
            chain.last_score = result["score"]
            chain.problem_joints = result["problem_joints"]
            self._update_display_curve(chain, result["positions"])
        if selected:
            self.refresh_details(selected)
        self.graph.set_results(self.scan_results, self.threshold_spin.value())

    def _display_curve_name(self, chain):
        return "tailCodeTATool_%s_CRV" % _safe_name(chain.label)

    def _ensure_display_group(self):
        if not cmds.objExists(DISPLAY_GROUP):
            cmds.group(empty=True, name=DISPLAY_GROUP)
            cmds.setAttr(DISPLAY_GROUP + ".inheritsTransform", 0)
        return DISPLAY_GROUP

    def _delete_display_curve(self, chain):
        name = self._display_curve_name(chain)
        if cmds.objExists(name):
            cmds.delete(name)

    def _display_curve_shape(self, curve):
        shapes = cmds.listRelatives(curve, shapes=True, fullPath=True) or []
        return shapes[0] if shapes else ""

    def _display_curve_can_reuse(self, curve, degree, positions):
        shape = self._display_curve_shape(curve)
        if not shape:
            return False
        cvs = cmds.ls("%s.cv[*]" % curve, flatten=True) or []
        if len(cvs) != len(positions):
            return False
        if cmds.objExists(shape + ".degree") and int(cmds.getAttr(shape + ".degree")) != degree:
            return False
        return True

    def _style_display_curve(self, curve, chain):
        shape = self._display_curve_shape(curve)
        color = COLORS[chain.color_index % len(COLORS)]
        if shape:
            cmds.setAttr(shape + ".overrideEnabled", 1)
            cmds.setAttr(shape + ".overrideRGBColors", 1)
            cmds.setAttr(shape + ".overrideColorRGB", color[0], color[1], color[2])
            if cmds.objExists(shape + ".lineWidth"):
                cmds.setAttr(shape + ".lineWidth", 3)
        cmds.setAttr(curve + ".template", 1)

    def _update_display_curve(self, chain, positions):
        name = self._display_curve_name(chain)
        if len(positions) < 2:
            self._delete_display_curve(chain)
            return
        self._ensure_display_group()
        degree = 1 if len(positions) < 4 else 3
        if cmds.objExists(name) and self._display_curve_can_reuse(name, degree, positions):
            # Reason: updating CVs avoids Maya delete/create churn during interactive rotation.
            for index, position in enumerate(positions):
                cmds.xform("%s.cv[%d]" % (name, index), worldSpace=True, translation=position)
            self._style_display_curve(name, chain)
            return
        if cmds.objExists(name):
            cmds.delete(name)
        curve = cmds.curve(name=name, degree=degree, point=positions)
        cmds.parent(curve, DISPLAY_GROUP)
        self._style_display_curve(curve, chain)

    def save_scene_data(self):
        node = _ensure_data_node()
        payload = json.dumps([c.to_dict() for c in self.chains], ensure_ascii=False)
        cmds.setAttr("%s.%s" % (node, DATA_ATTR), payload, type="string")

    def load_scene_data(self):
        self.chains = []
        if not cmds.objExists(DATA_NODE) or not cmds.attributeQuery(DATA_ATTR, node=DATA_NODE, exists=True):
            return
        raw = cmds.getAttr("%s.%s" % (DATA_NODE, DATA_ATTR)) or "[]"
        try:
            self.chains = [ChainData.from_dict(item) for item in json.loads(raw)]
        except Exception:
            cmds.warning("尻尾・コード TA ツール: シーン内データの復元に失敗しました。")
            self.chains = []

    def start_monitoring(self):
        try:
            self.stop_monitoring()
            self.is_monitoring = True
            self._monitored_joint_paths = set()
            for chain in self.chains:
                self._register_callbacks_for_chain(chain)
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
        except Exception:
            self._show_error("監視開始に失敗しました")

    def _register_callbacks_for_chain(self, chain):
        if not self.is_monitoring:
            return
        for joint in chain.joints:
            if not _joint_exists(joint):
                continue
            joint_path = _dag_path(joint)
            # Reason: overlapping chains can share joints; one callback per joint is enough.
            if joint_path in self._monitored_joint_paths:
                continue
            self._monitored_joint_paths.add(joint_path)
            sel = om.MSelectionList()
            sel.add(joint)
            obj = sel.getDependNode(0)
            cb = om.MNodeMessage.addAttributeChangedCallback(obj, self._on_attribute_changed)
            self.callbacks.append(cb)

    def _request_scores_update(self):
        if self._pending_score_update:
            return
        self._pending_score_update = True
        # Reason: attribute changes arrive in bursts while rotating; coalesce them into one refresh.
        QtCore.QTimer.singleShot(40, self._run_pending_scores_update)

    def _run_pending_scores_update(self):
        self._pending_score_update = False
        self.update_scores_and_display()

    def _on_attribute_changed(self, msg, plug, other_plug, client_data):
        if not (msg & om.MNodeMessage.kAttributeSet):
            return
        self._request_scores_update()

    def stop_monitoring(self):
        if self.callbacks and om is not None:
            for cb in self.callbacks:
                try:
                    om.MMessage.removeCallback(cb)
                except Exception:
                    pass
        self.callbacks = []
        self._monitored_joint_paths = set()
        self._pending_score_update = False
        self.is_monitoring = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def restart_monitoring_if_needed(self):
        if self.is_monitoring:
            self.start_monitoring()

    def _ensure_tweaker_group(self):
        if not cmds.objExists(TWEAKER_GROUP):
            cmds.group(empty=True, name=TWEAKER_GROUP)
            cmds.setAttr(TWEAKER_GROUP + ".inheritsTransform", 0)
        return TWEAKER_GROUP

    def _tweaker_name_for_joint(self, joint, owner_label=None):
        joint_name = _safe_name(joint.split("|")[-1])
        # Reason: overlapping chains need separate tweakers for the same joint.
        if owner_label:
            return "%s%s_%s" % (TWEAKER_PREFIX, _safe_name(owner_label), joint_name)
        return TWEAKER_PREFIX + joint_name

    def _scan_all_tweakers(self):
        transforms = cmds.ls(type="transform") or []
        return [node for node in transforms if _is_tweaker(node)]

    def _all_tweakers(self):
        if self._tweaker_scan_cache is not None:
            return list(self._tweaker_scan_cache)
        return self._scan_all_tweakers()

    def _tweakers_from_selection(self):
        selected = cmds.ls(selection=True, long=False) or []
        tweakers = []
        for node in selected:
            if _is_tweaker(node):
                tweakers.append(node)
                continue
            if _joint_exists(node):
                tweakers.extend(self._tweakers_for_joint(node))
        return list(dict.fromkeys(tweakers))

    def _chain_priority_by_label(self, label):
        for index, chain in enumerate(self.chains):
            if chain.label == label:
                return index
        return len(self.chains)

    def _joint_priority_index(self, joint):
        joint_long = _dag_path(joint)
        for index, chain in enumerate(self.chains):
            for chain_joint in chain.joints:
                chain_joint_long = _dag_path(chain_joint) if cmds.objExists(chain_joint) else chain_joint
                if chain_joint == joint or chain_joint == joint_long or chain_joint_long == joint_long:
                    return index
        return len(self.chains)

    def _tweaker_priority(self, tweaker, fallback_joint):
        owner = _get_string_attr(tweaker, "ownerChainLabel")
        if owner:
            return self._chain_priority_by_label(owner)
        return self._joint_priority_index(_get_string_attr(tweaker, "targetJoint") or fallback_joint)

    def _tweakers_for_joint(self, joint):
        joint_long = _dag_path(joint)
        result = []
        for tweaker in self._all_tweakers():
            target = _get_string_attr(tweaker, "targetJoint")
            target_long = _dag_path(target) if target and cmds.objExists(target) else target
            if target == joint or target == joint_long or target_long == joint_long:
                result.append(tweaker)
        result.sort(key=lambda tweaker: (self._tweaker_priority(tweaker, joint), tweaker))
        return result

    def _tweaker_for_joint_and_label(self, joint, label):
        # Reason: creation status should ignore already-owned tweakers without falling back to owner-less ones.
        for tweaker in self._tweakers_for_joint(joint):
            if _get_string_attr(tweaker, "ownerChainLabel") == label:
                return tweaker
        return ""

    def _update_chain_tweaker_owner_label(self, old_label, new_label):
        if old_label == new_label:
            return
        for tweaker in self._all_tweakers():
            if _get_string_attr(tweaker, "ownerChainLabel") != old_label:
                continue
            attr = tweaker + ".ownerChainLabel"
            if cmds.objExists(attr):
                cmds.setAttr(attr, new_label, type="string")

    def _selected_tweaker_targets(self):
        joints = cmds.ls(selection=True, type="joint") or []
        if joints:
            return sort_root_to_tip(joints)
        chain = self._selected_chain()
        selected_rows = sorted({item.row() for item in self.angle_table.selectedItems()})
        if chain and selected_rows:
            row = selected_rows[0]
            if 0 <= row < len(chain.joints) and _joint_exists(chain.joints[row]):
                return [chain.joints[row]]
        return []

    def _create_tweakers_for_chain(self, chain):
        created = []
        failed = []
        for joint in chain.joints:
            try:
                if self._tweaker_for_joint_and_label(joint, chain.label):
                    continue
                tweaker = self._create_tweaker(joint, owner_label=chain.label)
                if tweaker not in created:
                    created.append(tweaker)
            except Exception as exc:
                failed.append("%s (%s)" % (joint, exc))
        if failed:
            message = "一部 Tweaker を作成できませんでした: %s" % ", ".join(failed)
            cmds.warning(message)
            try:
                self.tweaker_status.setText(message)
            except Exception:
                pass
        return created

    def _delete_tweakers_for_chain(self, chain):
        tweakers = []
        remaining_joints = {j for other in self.chains for j in other.joints}
        remaining_longs = {_dag_path(j) for j in remaining_joints if cmds.objExists(j)}
        for joint in chain.joints:
            joint_long = _dag_path(joint) if cmds.objExists(joint) else joint
            for tweaker in self._tweakers_for_joint(joint):
                owner = _get_string_attr(tweaker, "ownerChainLabel")
                if owner == chain.label:
                    tweakers.append(tweaker)
                elif not owner and joint not in remaining_joints and joint_long not in remaining_longs:
                    tweakers.append(tweaker)
        tweakers = list(dict.fromkeys(tweakers))
        if tweakers:
            self._delete_tweakers(tweakers)
        return tweakers

    def create_tweaker_for_selection(self):
        try:
            targets = self._selected_tweaker_targets()
            if not targets:
                cmds.warning("Tweaker を作成するジョイントを選択してください。")
                return
            chain = self._selected_chain()
            owner_label = chain.label if chain else None
            created = []
            for joint in targets:
                created.append(self._create_tweaker(joint, owner_label=owner_label))
            cmds.select(created, replace=True)
            self.tweaker_status.setText("作成: %s" % ", ".join(created))
        except Exception:
            self._show_error("Tweaker 作成に失敗しました")

    def _create_tweaker(self, joint, owner_label=None):
        if not _joint_exists(joint):
            raise RuntimeError("ジョイントが見つかりません: %s" % joint)

        existing = self._tweakers_for_joint(joint)
        if owner_label:
            # Reason: legacy owner-less tweakers must not block chain-owned ones.
            for tweaker in existing:
                if _get_string_attr(tweaker, "ownerChainLabel") == owner_label:
                    return tweaker
        elif existing:
            return existing[0]

        group = self._ensure_tweaker_group()
        name = self._tweaker_name_for_joint(joint, owner_label=owner_label)
        tweaker = cmds.spaceLocator(name=name)[0]
        joint_matrix = cmds.xform(joint, query=True, worldSpace=True, matrix=True)
        cmds.xform(tweaker, worldSpace=True, matrix=joint_matrix)
        for axis in "XYZ":
            if cmds.objExists(tweaker + ".localScale" + axis):
                cmds.setAttr(tweaker + ".localScale" + axis, 1.5)
        # Reason: keeping tweakers directly under the stable root avoids chain subgroup transforms changing constraint results.
        cmds.parent(tweaker, group)
        _add_bool_attr(tweaker, "tailTweaker", True)
        _add_string_attr(tweaker, "targetJoint", _dag_path(joint))
        _add_string_attr(tweaker, "ownerChainLabel", owner_label or "")

        rotate_attrs = ["%s.rotate%s" % (joint, axis) for axis in "XYZ"]
        if any(_attr_has_incoming_connection(attr) for attr in rotate_attrs):
            for axis in "XYZ":
                rotate_attr = "%s.rotate%s" % (tweaker, axis)
                if cmds.objExists(rotate_attr) and not cmds.getAttr(rotate_attr, lock=True):
                    cmds.setAttr(rotate_attr, 0.0)
            self._connect_additive_tweaker(tweaker, joint)
            _add_string_attr(tweaker, "connectionMode", "additiveRotate")
        else:
            existing_constraint = _find_orient_constraint_driving(joint)
            if existing_constraint:
                cmds.orientConstraint(tweaker, joint, edit=True, maintainOffset=True, weight=1.0)
                constraint = existing_constraint
                _add_bool_attr(tweaker, "usesExistingConstraint", True)
            else:
                constraint = cmds.orientConstraint(tweaker, joint, maintainOffset=True, name=tweaker + "_orientConstraint")[0]
                _add_bool_attr(tweaker, "usesExistingConstraint", False)
            _add_string_attr(tweaker, "constraintNode", constraint)
            _add_string_attr(tweaker, "connectionMode", "constraint")
        return tweaker

    def _connect_additive_tweaker(self, tweaker, joint):
        nodes = []
        for axis in "XYZ":
            target_attr = "%s.rotate%s" % (joint, axis)
            tweaker_attr = "%s.rotate%s" % (tweaker, axis)
            source_attr = _incoming_plug(target_attr)
            base_value = cmds.getAttr(target_attr)
            plus = cmds.createNode("plusMinusAverage", name="%s_addRotate%s_PMA" % (tweaker, axis))
            cmds.setAttr(plus + ".operation", 1)
            if source_attr:
                cmds.disconnectAttr(source_attr, target_attr)
                cmds.connectAttr(source_attr, plus + ".input1D[0]", force=True)
            else:
                cmds.setAttr(plus + ".input1D[0]", base_value)
            cmds.connectAttr(tweaker_attr, plus + ".input1D[1]", force=True)
            cmds.connectAttr(plus + ".output1D", target_attr, force=True)
            _add_string_attr(tweaker, "sourceRotate%s" % axis, source_attr)
            _add_string_attr(tweaker, "addNode%s" % axis, plus)
            _add_string_attr(tweaker, "baseRotate%s" % axis, str(base_value))
            nodes.append(plus)
        _add_string_attr(tweaker, "additiveNodes", "|".join(nodes))

    def _disconnect_additive_tweaker(self, tweaker, keep_final=False):
        target = _get_string_attr(tweaker, "targetJoint")
        final_rotate = _rotate_values(target) if _joint_exists(target) else (0.0, 0.0, 0.0)
        for index, axis in enumerate("XYZ"):
            target_attr = "%s.rotate%s" % (target, axis)
            source_attr = _get_string_attr(tweaker, "sourceRotate%s" % axis)
            plus = _get_string_attr(tweaker, "addNode%s" % axis)
            if plus and cmds.objExists(plus + ".output1D") and cmds.objExists(target_attr):
                try:
                    if cmds.isConnected(plus + ".output1D", target_attr):
                        cmds.disconnectAttr(plus + ".output1D", target_attr)
                except Exception:
                    pass
            if source_attr and cmds.objExists(source_attr) and cmds.objExists(target_attr):
                try:
                    cmds.connectAttr(source_attr, target_attr, force=True)
                except Exception:
                    cmds.warning("元の回転接続を復元できませんでした: %s -> %s" % (source_attr, target_attr))
            elif keep_final and cmds.objExists(target_attr) and not cmds.getAttr(target_attr, lock=True):
                cmds.setAttr(target_attr, final_rotate[index])
            elif cmds.objExists(target_attr) and not cmds.getAttr(target_attr, lock=True):
                try:
                    base_value = float(_get_string_attr(tweaker, "baseRotate%s" % axis, "0"))
                    cmds.setAttr(target_attr, base_value)
                except Exception:
                    pass
            if plus and cmds.objExists(plus):
                cmds.delete(plus)

    def _delete_constraint_tweaker_link(self, tweaker):
        constraint = _get_string_attr(tweaker, "constraintNode")
        uses_existing = bool(
            cmds.attributeQuery("usesExistingConstraint", node=tweaker, exists=True)
            and cmds.getAttr(tweaker + ".usesExistingConstraint")
        )
        target = _get_string_attr(tweaker, "targetJoint")
        if uses_existing and constraint and cmds.objExists(constraint) and _joint_exists(target):
            try:
                cmds.orientConstraint(tweaker, target, edit=True, remove=True)
            except Exception:
                cmds.warning("既存 constraint から Tweaker ターゲットを外せませんでした: %s" % tweaker)
        elif constraint and cmds.objExists(constraint):
            cmds.delete(constraint)
        child_constraints = cmds.listConnections(tweaker, source=False, destination=True, type="orientConstraint") or []
        for constraint_node in child_constraints:
            if cmds.objExists(constraint_node) and constraint_node != constraint:
                cmds.delete(constraint_node)

    def delete_selected_tweakers(self):
        try:
            tweakers = self._tweakers_from_selection()
            if not tweakers:
                cmds.warning("削除する Tweaker、または対象ジョイントを選択してください。")
                return
            self._delete_tweakers(tweakers)
            self.tweaker_status.setText("削除: %s" % ", ".join(tweakers))
            self.update_scores_and_display()
        except Exception:
            self._show_error("Tweaker 削除に失敗しました")

    def _delete_tweakers(self, tweakers):
        for tweaker in tweakers:
            if not cmds.objExists(tweaker):
                continue
            mode = _get_string_attr(tweaker, "connectionMode", "constraint")
            if mode == "additiveRotate":
                self._disconnect_additive_tweaker(tweaker, keep_final=False)
            else:
                self._delete_constraint_tweaker_link(tweaker)
            if cmds.objExists(tweaker):
                cmds.delete(tweaker)

    def bake_selected_tweakers(self):
        try:
            tweakers = self._tweakers_from_selection()
            if not tweakers:
                cmds.warning("Bake する Tweaker、または対象ジョイントを選択してください。")
                return
            baked = []
            for tweaker in tweakers:
                target = _get_string_attr(tweaker, "targetJoint")
                if not _joint_exists(target):
                    cmds.warning("対象ジョイントが見つかりません: %s" % target)
                    continue
                final_rotate = _rotate_values(target)
                mode = _get_string_attr(tweaker, "connectionMode", "constraint")
                if mode == "additiveRotate":
                    self._disconnect_additive_tweaker(tweaker, keep_final=True)
                else:
                    self._delete_constraint_tweaker_link(tweaker)
                for axis, value in zip("XYZ", final_rotate):
                    attr = "%s.rotate%s" % (target, axis)
                    if cmds.objExists(attr) and not cmds.getAttr(attr, lock=True) and not _attr_has_incoming_connection(attr):
                        cmds.setAttr(attr, value)
                if cmds.autoKeyframe(query=True, state=True):
                    cmds.setKeyframe(target, attribute=["rotateX", "rotateY", "rotateZ"])
                if cmds.objExists(tweaker):
                    cmds.delete(tweaker)
                baked.append(target)
            self.tweaker_status.setText("Bake 完了: %s" % ", ".join(baked))
            self.update_scores_and_display()
        except Exception:
            self._show_error("Tweaker Bake に失敗しました")

    def auto_create_tweaker(self):
        try:
            chain = self._selected_chain()
            if chain is None:
                cmds.warning("Auto Tweaker 用のチェーンを選択してください。")
                return
            result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold)
            rows = [row for row in result["angle_rows"] if row["bend"] > chain.threshold]
            if not rows:
                cmds.warning("しきい値を超える曲がり箇所がありません。")
                return
            target_row = max(rows, key=lambda row: row["bend"])
            tweaker = self._create_tweaker(target_row["joint"], owner_label=chain.label)
            cmds.select(tweaker, replace=True)
            self.tweaker_status.setText("Auto Tweaker: %s  曲がり %.2f 度" % (target_row["joint"], target_row["bend"]))
        except Exception:
            self._show_error("Auto Tweaker に失敗しました")

    def smart_suggest(self):
        try:
            chain = self._selected_chain()
            if chain is None:
                cmds.warning("Smart Suggest 用のチェーンを選択してください。")
                return
            result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold)
            rows = result["angle_rows"]
            if not rows:
                return
            indexed = [(index, row) for index, row in enumerate(rows)]
            index, row = max(indexed, key=lambda item: item[1]["bend"])
            if row["bend"] <= chain.threshold:
                self.tweaker_status.setText("Smart Suggest: 目立つ曲がり超過はありません。")
                return
            prev_rotate = rows[index - 1]["rotate"] if index > 0 else (0.0, 0.0, 0.0)
            rotate = row["rotate"]
            deltas = [rotate[i] - prev_rotate[i] for i in range(3)]
            axis_index = max(range(3), key=lambda i: abs(deltas[i]))
            axis = "XYZ"[axis_index]
            amount = max(-15.0, min(15.0, -deltas[axis_index] * 0.5))
            self.tweaker_status.setText(
                "Smart Suggest: %s の曲がり %.2f 度。TailTweaker で R%s %+0.2f 度を目安に調整してください。"
                % (row["joint"], row["bend"], axis, amount)
            )
        except Exception:
            self._show_error("Smart Suggest に失敗しました")

    def scan_range(self):
        if not self.chains:
            cmds.warning("登録済みチェーンがありません。")
            return
        was_monitoring = self.is_monitoring
        self.stop_monitoring()
        self.cancel_scan = False
        self.cancel_scan_button.setEnabled(True)
        start = min(self.start_frame.value(), self.end_frame.value())
        end = max(self.start_frame.value(), self.end_frame.value())
        current = cmds.currentTime(query=True)
        frames = list(range(start, end + 1))
        self.scan_progress.setRange(0, len(frames))
        results = {chain.label: {} for chain in self.chains if chain.visible}
        try:
            for index, frame in enumerate(frames, 1):
                if self.cancel_scan:
                    break
                cmds.currentTime(frame, edit=True)
                for chain in self.chains:
                    if chain.visible:
                        result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold)
                        results[chain.label][frame] = max(result["bend_values"] or [0.0])
                self.scan_progress.setValue(index)
                QtWidgets.QApplication.processEvents()
            self.scan_results = results
            max_threshold = max([c.threshold for c in self.chains] or [self.threshold_spin.value()])
            self.graph.set_results(self.scan_results, max_threshold)
        except Exception:
            self._show_error("範囲スキャンに失敗しました")
        finally:
            cmds.currentTime(current, edit=True)
            self.cancel_scan_button.setEnabled(False)
            if was_monitoring:
                self.start_monitoring()
            self.update_scores_and_display()

    def _cancel_scan(self):
        self.cancel_scan = True

    def goto_frame(self, frame):
        cmds.currentTime(frame, edit=True)
        self.update_scores_and_display()

    def export_csv(self):
        if not self.scan_results:
            cmds.warning("書き出すスキャン結果がありません。")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "CSV書き出し", "tail_code_ta_bend_scores.csv", "CSV Files (*.csv)")
        if not path:
            return
        frames = sorted({f for scores in self.scan_results.values() for f in scores})
        labels = list(self.scan_results.keys())
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as handle:
                writer = csv.writer(handle)
                writer.writerow(["frame"] + labels)
                for frame in frames:
                    writer.writerow([frame] + [self.scan_results[label].get(frame, "") for label in labels])
        except Exception:
            self._show_error("CSV書き出しに失敗しました")

    def closeEvent(self, event):
        self.end_graph_undo()
        self.stop_monitoring()
        self.save_scene_data()
        super().closeEvent(event)

    def _show_error(self, title):
        detail = traceback.format_exc()
        cmds.warning("%s: %s" % (title, detail))
        QtWidgets.QMessageBox.critical(self, title, detail)


def show():
    _require_maya()
    if QtWidgets is None:
        raise RuntimeError("PySide6 / PySide2 is not available.")
    for widget in QtWidgets.QApplication.topLevelWidgets():
        if widget.objectName() == WINDOW_OBJECT_NAME:
            widget.close()
            widget.deleteLater()
    dialog = TailCodeTATool(parent=_maya_main_window())
    dialog.show()
    return dialog
