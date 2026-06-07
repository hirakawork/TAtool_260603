"""Tail / code chain TA tool for Maya 2026.

Run in Maya Script Editor:

    import tail_code_ta_tool
    tail_code_ta_tool.show()

The implementation is intentionally self-contained so it can be dropped into a
Maya scripts folder or loaded directly from the Script Editor.
"""

from __future__ import annotations

import json
import math
import time
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
WINDOW_DEFAULT_WIDTH = 920
WINDOW_DEFAULT_HEIGHT = 720
WINDOW_MIN_WIDTH = 760
WINDOW_MIN_HEIGHT = 420
WINDOW_SCREEN_MARGIN = 96
DATA_NODE = "tailCodeTATool_sceneData"
DATA_ATTR = "chainsJson"
DISPLAY_GROUP = "tailCodeTATool_display_GRP"
TWEAKER_GROUP = "TailTweaker_GRP"
TWEAKER_PREFIX = "TailTweaker_"
TWIST_EDIT_MAX_STEP_DEGREES = 2.0
TWIST_EDIT_MAX_RESPONSE_DEGREES = 25.0
DRIVER_CONSTRAINT_TYPES = ("orientConstraint", "parentConstraint", "pointConstraint", "aimConstraint")
NON_DRIVER_GRAPH_TYPES = ("objectSet", "displayLayer", "renderLayer", "shadingEngine")
CONTROLLER_CHILD_EXPAND_DEFAULT_DEPTH = 1
CONTROLLER_CHILD_EXPAND_MAX_DEPTH = 99


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


def _dominant_translate_axis(joint):
    if not cmds or not cmds.objExists(joint + ".translate"):
        return -1, (0.0, 0.0, 0.0)
    values = cmds.getAttr(joint + ".translate")[0]
    translate = tuple(float(v) for v in values)
    axis = max(range(3), key=lambda index: abs(translate[index]))
    if abs(translate[axis]) <= 1e-8:
        return -1, (0.0, 0.0, 0.0)
    sign = -1.0 if translate[axis] < 0.0 else 1.0
    vector = [0.0, 0.0, 0.0]
    vector[axis] = sign
    return axis, tuple(vector)


def _axis_override_name(axis):
    axis = str(axis or "").strip().upper()
    return axis if axis in ("X", "Y", "Z") else ""


def _twist_axis_vector(joint, axis_override=None):
    axis_override = _axis_override_name(axis_override)
    if not axis_override:
        return _dominant_translate_axis(joint)
    axis = "XYZ".index(axis_override)
    sign = 1.0
    if cmds and cmds.objExists(joint + ".translate"):
        values = cmds.getAttr(joint + ".translate")[0]
        value = float(values[axis])
        if abs(value) > 1e-8:
            sign = -1.0 if value < 0.0 else 1.0
    vector = [0.0, 0.0, 0.0]
    vector[axis] = sign
    return axis, tuple(vector)


def _world_rotation_quaternion(node):
    if om is None or not cmds:
        return None
    matrix_values = cmds.xform(node, query=True, worldSpace=True, matrix=True)
    matrix = om.MMatrix(matrix_values)
    transform = om.MTransformationMatrix(matrix)
    try:
        return transform.rotation(asQuaternion=True)
    except TypeError:
        rotation = transform.rotation()
        return rotation.asQuaternion() if hasattr(rotation, "asQuaternion") else rotation


def _relative_quaternion(parent, child):
    parent_quat = _world_rotation_quaternion(parent)
    child_quat = _world_rotation_quaternion(child)
    if parent_quat is None or child_quat is None:
        return None
    return parent_quat.inverse() * child_quat


def _quaternion_angle_degrees(quat):
    if quat is None:
        return 0.0
    try:
        normalized = quat.normal()
    except Exception:
        normalized = quat
    w = max(-1.0, min(1.0, float(normalized.w)))
    angle = math.degrees(2.0 * math.acos(w))
    if angle > 180.0:
        angle -= 360.0
    return angle


def _swing_twist_degrees(parent, child, axis_override=None):
    axis_index, axis_vector = _twist_axis_vector(child, axis_override=axis_override)
    if axis_index < 0 or om is None:
        return 0.0, 0.0, "-"
    relative = _relative_quaternion(parent, child)
    if relative is None:
        return 0.0, 0.0, "-"

    axis = om.MVector(axis_vector)
    vector = om.MVector(relative.x, relative.y, relative.z)
    projection = axis * (vector * axis)
    twist = om.MQuaternion(projection.x, projection.y, projection.z, relative.w)
    try:
        twist = twist.normal()
    except Exception:
        try:
            twist.normalizeIt()
        except Exception:
            return 0.0, 0.0, "-"

    twist_degrees = _quaternion_angle_degrees(twist)
    if (vector * axis) < 0.0:
        twist_degrees = -abs(twist_degrees)
    else:
        twist_degrees = abs(twist_degrees)

    try:
        swing = relative * twist.inverse()
    except Exception:
        swing = None
    swing_degrees = abs(_quaternion_angle_degrees(swing))
    return swing_degrees, twist_degrees, "XYZ"[axis_index]


def _dag_path(node):
    paths = cmds.ls(node, long=True) or []
    return paths[0] if paths else node


def _same_dag_node(a, b):
    if a == b:
        return True
    a_path = _dag_path(a) if cmds and a and cmds.objExists(a) else a
    b_path = _dag_path(b) if cmds and b and cmds.objExists(b) else b
    return a_path == b_path


def _append_unique_joint(joints, joint):
    if not _joint_exists(joint):
        return
    if any(_same_dag_node(existing, joint) for existing in joints):
        return
    joints.append(joint)


def _child_joints(joint):
    if not _joint_exists(joint):
        return []
    return cmds.listRelatives(joint, children=True, type="joint", fullPath=False) or []


def _with_child_joints(joints, depth):
    depth = max(0, min(int(depth), CONTROLLER_CHILD_EXPAND_MAX_DEPTH))
    expanded = []
    frontier = []
    for joint in joints:
        _append_unique_joint(expanded, joint)
        frontier.append(joint)
    for _level in range(depth):
        next_frontier = []
        for joint in frontier:
            for child in _child_joints(joint):
                next_frontier.append(child)
                _append_unique_joint(expanded, child)
        frontier = next_frontier
        if not frontier:
            break
    return expanded


def _add_string_attr(node, attr, value):
    if not cmds.attributeQuery(attr, node=node, exists=True):
        cmds.addAttr(node, longName=attr, dataType="string")
    cmds.setAttr("%s.%s" % (node, attr), value, type="string")


def _add_bool_attr(node, attr, value=True):
    if not cmds.attributeQuery(attr, node=node, exists=True):
        cmds.addAttr(node, longName=attr, attributeType="bool")
    cmds.setAttr("%s.%s" % (node, attr), bool(value))


def _add_float_attr(node, attr, value=0.0, min_value=None, max_value=None, keyable=True):
    if not cmds.attributeQuery(attr, node=node, exists=True):
        kwargs = {
            "longName": attr,
            "attributeType": "double",
            "defaultValue": float(value),
            "keyable": keyable,
        }
        if min_value is not None:
            kwargs["minValue"] = float(min_value)
        if max_value is not None:
            kwargs["maxValue"] = float(max_value)
        cmds.addAttr(node, **kwargs)
    cmds.setAttr("%s.%s" % (node, attr), float(value))


def _get_string_attr(node, attr, default=""):
    if not cmds.objExists(node) or not cmds.attributeQuery(attr, node=node, exists=True):
        return default
    return cmds.getAttr("%s.%s" % (node, attr)) or default


def _is_tweaker(node):
    return bool(cmds.objExists(node) and cmds.attributeQuery("tailTweaker", node=node, exists=True) and cmds.getAttr(node + ".tailTweaker"))


def _node_from_input_name(name):
    return str(name or "").strip().split(".", 1)[0]


def _transform_or_self(node):
    if not cmds.objExists(node):
        return node
    try:
        if cmds.nodeType(node) in ("joint", "transform"):
            return node
    except Exception:
        return node
    parents = cmds.listRelatives(node, parent=True, fullPath=False) or []
    return parents[0] if parents else node


def _downstream_joints(node, max_depth=4):
    found = []
    seen = {node}
    frontier = [node]
    for _depth in range(max_depth):
        next_frontier = []
        for current in frontier:
            if not cmds.objExists(current):
                continue
            connections = cmds.listConnections(current, source=False, destination=True) or []
            for connected in connections:
                if connected in seen:
                    continue
                seen.add(connected)
                if _joint_exists(connected):
                    _append_unique_joint(found, connected)
                elif cmds.objExists(connected) and cmds.nodeType(connected) not in NON_DRIVER_GRAPH_TYPES:
                    next_frontier.append(connected)
        frontier = next_frontier
        if not frontier:
            break
    return found


def _controller_constraints(node):
    constraints = []
    for constraint_type in DRIVER_CONSTRAINT_TYPES:
        found = cmds.listConnections(node, source=False, destination=True, type=constraint_type) or []
        for constraint in found:
            if constraint not in constraints:
                constraints.append(constraint)
    return constraints


def _controller_driven_joints(node, child_depth=CONTROLLER_CHILD_EXPAND_DEFAULT_DEPTH):
    transform = _transform_or_self(node)
    joints = []
    for constraint in _controller_constraints(transform):
        for joint in _downstream_joints(constraint, max_depth=3):
            _append_unique_joint(joints, joint)
    if not joints:
        for joint in cmds.listRelatives(transform, children=True, type="joint", fullPath=False) or []:
            _append_unique_joint(joints, joint)
    if not joints:
        for joint in _downstream_joints(transform, max_depth=4):
            _append_unique_joint(joints, joint)
    return _with_child_joints(joints, child_depth)


def _resolve_chain_joint_input(name, child_depth=CONTROLLER_CHILD_EXPAND_DEFAULT_DEPTH):
    node = _node_from_input_name(name)
    if not node or not cmds.objExists(node):
        return []
    node = _transform_or_self(node)
    if _joint_exists(node):
        return [node]
    if _is_tweaker(node):
        target = _get_string_attr(node, "targetJoint")
        return [target] if _joint_exists(target) else []
    return _controller_driven_joints(node, child_depth=child_depth)


def _resolve_chain_joints(inputs, child_depth=CONTROLLER_CHILD_EXPAND_DEFAULT_DEPTH):
    joints = []
    unresolved = []
    for name in inputs:
        resolved = _resolve_chain_joint_input(name, child_depth=child_depth)
        if not resolved:
            unresolved.append(str(name))
            continue
        for joint in resolved:
            _append_unique_joint(joints, joint)
    return sort_root_to_tip(joints), unresolved


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
    twist_axis_overrides: dict[str, str] = field(default_factory=dict)
    last_score: float = 0.0
    problem_joints: list[str] = field(default_factory=list)

    def to_dict(self):
        return {
            "label": self.label,
            "joints": self.joints,
            "threshold": self.threshold,
            "visible": self.visible,
            "color_index": self.color_index,
            "twist_axis_overrides": self.twist_axis_overrides,
        }

    @classmethod
    def from_dict(cls, data):
        overrides = {}
        for joint, axis in dict(data.get("twist_axis_overrides") or {}).items():
            axis = _axis_override_name(axis)
            if axis:
                overrides[str(joint)] = axis
        return cls(
            label=str(data.get("label") or "chain"),
            joints=[str(j) for j in data.get("joints", []) if j],
            threshold=float(data.get("threshold", 15.0)),
            visible=bool(data.get("visible", True)),
            color_index=int(data.get("color_index", 0)),
            twist_axis_overrides=overrides,
        )


class JointAngleAnalyzer:
    @staticmethod
    def twist_axis_override_for_joint(joint, twist_axis_overrides):
        if not twist_axis_overrides:
            return ""
        direct = _axis_override_name(twist_axis_overrides.get(joint))
        if direct:
            return direct
        joint_long = _dag_path(joint) if cmds and cmds.objExists(joint) else joint
        for key, axis in twist_axis_overrides.items():
            key_long = _dag_path(key) if cmds and cmds.objExists(key) else key
            if key == joint or key == joint_long or key_long == joint_long:
                return _axis_override_name(axis)
        return ""

    @staticmethod
    def bend_twist_axis(rows, index):
        axis = rows[index].get("twist_axis", "-") if 0 <= index < len(rows) else "-"
        if axis not in "XYZ" and index == 0 and len(rows) > 1:
            axis = rows[1].get("twist_axis", "-")
        return axis if axis in "XYZ" else "-"

    @staticmethod
    def bend_direction(rows, index):
        if not (0 <= index < len(rows)):
            return (0.0, 0.0, 0.0)
        axis = JointAngleAnalyzer.bend_twist_axis(rows, index)
        excluded_axis_index = "XYZ".index(axis) if axis in "XYZ" else -1

        def _without_twist(values):
            result = list(values)
            if excluded_axis_index >= 0:
                result[excluded_axis_index] = 0.0
            return result

        row = rows[index]
        direction = _without_twist(row["rotate"])
        if max(abs(v) for v in direction) < 0.001:
            if index == 0 and len(rows) > 1:
                direction = _without_twist(tuple(-v for v in rows[1].get("signed_delta", (0.0, 0.0, 0.0))))
            elif index > 0:
                direction = _without_twist(row.get("signed_delta", (0.0, 0.0, 0.0)))
        return tuple(direction)

    @staticmethod
    def endpoint_bend_value(rows, index):
        if not rows or index not in (0, len(rows) - 1):
            return 0.0
        direction = JointAngleAnalyzer.bend_direction(rows, index)
        return max(abs(v) for v in direction)

    @staticmethod
    def evaluate(joints, threshold, twist_axis_overrides=None):
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
                    "signed_delta": (0.0, 0.0, 0.0),
                    "max_delta": 0.0,
                    "swing": 0.0,
                    "twist": 0.0,
                    "twist_abs": 0.0,
                    "twist_axis": "-",
                    "twist_axis_manual": False,
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
            signed_delta = tuple(rotate[axis] - prev_rotate[axis] for axis in range(3))
            delta = tuple(abs(value) for value in signed_delta)
            max_delta = max(delta)
            axis_override = JointAngleAnalyzer.twist_axis_override_for_joint(rows[i]["joint"], twist_axis_overrides)
            swing, twist, twist_axis = _swing_twist_degrees(rows[i - 1]["joint"], rows[i]["joint"], axis_override)
            rows[i]["delta"] = delta
            rows[i]["signed_delta"] = signed_delta
            rows[i]["max_delta"] = max_delta
            rows[i]["swing"] = swing
            rows[i]["twist"] = twist
            rows[i]["twist_abs"] = abs(twist)
            rows[i]["twist_axis"] = twist_axis
            rows[i]["twist_axis_manual"] = bool(axis_override)
            axis_deltas.append(delta)

        if rows:
            for index in sorted({0, len(rows) - 1}):
                rows[index]["bend"] = JointAngleAnalyzer.endpoint_bend_value(rows, index)

        bend_values = [row["bend"] for row in rows]
        twist_values = [row["twist_abs"] for row in rows]
        bend_deltas = []
        twist_problem_joints = []
        for i in range(1, len(rows)):
            delta = abs(rows[i]["bend"] - rows[i - 1]["bend"])
            bend_deltas.append(delta)
            if rows[i]["twist_abs"] > threshold:
                twist_problem_joints.append(rows[i]["joint"])
        problem_joints = [row["joint"] for row in rows if row["bend"] > threshold]

        score = sum(max(0.0, bend - threshold) for bend in bend_values)
        twist_score = sum(max(0.0, twist - threshold) for twist in twist_values)
        return {
            "score": score,
            "twist_score": twist_score,
            "angle_rows": rows,
            "axis_deltas": axis_deltas,
            "max_deltas": [row["max_delta"] for row in rows],
            "bend_values": bend_values,
            "twist_values": twist_values,
            "bend_deltas": bend_deltas,
            "problem_joints": problem_joints,
            "twist_problem_joints": twist_problem_joints,
            "positions": positions,
        }


CurvatureAnalyzer = JointAngleAnalyzer


class BendGraphWidget(QtWidgets.QWidget):
    bendEditStarted = QtCore.Signal()
    bendEdited = QtCore.Signal(int, float)
    bendEditFinished = QtCore.Signal()

    def __init__(
        self,
        parent=None,
        value_key="bend",
        label="曲がり",
        color=None,
        min_display_value=50.0,
        signed=False,
    ):
        super().__init__(parent)
        self.rows = []
        self.threshold = 15.0
        self.value_key = value_key
        self.label = label
        self.line_color = color or QtGui.QColor(245, 190, 75)
        self.min_display_value = float(min_display_value)
        self.signed = bool(signed)
        self.drag_index = None
        self.drag_max_value = None
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
        if self.drag_max_value is not None:
            return self.drag_max_value
        values = [self._row_value(row) for row in self.rows]
        if self.signed:
            values = [abs(value) for value in values]
        max_value = max(values + [self.threshold * 2.0, 1.0])
        return max(self.min_display_value, math.ceil(max_value * 1.25 / 10.0) * 10.0)

    def _row_value(self, row):
        try:
            value = float(row.get(self.value_key, 0.0))
            return value if self.signed else max(0.0, value)
        except Exception:
            return 0.0

    def _point(self, index, value):
        plot = self._plot_rect()
        max_value = self._max_value()
        x = plot.left() + (index / max(1, len(self.rows) - 1)) * plot.width()
        if self.signed:
            center = plot.center().y()
            y = center - (value / max_value) * (plot.height() * 0.5)
        else:
            y = plot.bottom() - (value / max_value) * plot.height()
        return QtCore.QPointF(x, y)

    def _value_from_y(self, y):
        plot = self._plot_rect()
        max_value = self._max_value()
        y = max(plot.top(), min(plot.bottom(), y))
        if self.signed:
            center = plot.center().y()
            return (center - y) / max(1.0, plot.height() * 0.5) * max_value
        return max(0.0, (plot.bottom() - y) / max(1.0, plot.height()) * max_value)

    def _nearest_index(self, pos):
        if not self.rows:
            return None
        best_index = None
        best_distance = 999999.0
        for index, row in enumerate(self.rows):
            p = self._point(index, self._row_value(row))
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
        self.drag_max_value = self._max_value()
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
        self.drag_max_value = None
        self.update()

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
            painter.drawText(rect, QtCore.Qt.AlignCenter, "%sグラフ" % self.label)
            return

        values = [self._row_value(row) for row in self.rows]
        max_value = self._max_value()

        painter.setPen(QtGui.QPen(QtGui.QColor(210, 80, 70), 1, QtCore.Qt.DashLine))
        if self.signed:
            center_y = plot.center().y()
            painter.setPen(QtGui.QPen(QtGui.QColor(120, 125, 135), 1))
            painter.drawLine(QtCore.QPointF(plot.left(), center_y), QtCore.QPointF(plot.right(), center_y))
            painter.setPen(QtGui.QPen(QtGui.QColor(210, 80, 70), 1, QtCore.Qt.DashLine))
            positive_y = self._point(0, self.threshold).y()
            negative_y = self._point(0, -self.threshold).y()
            painter.drawLine(QtCore.QPointF(plot.left(), positive_y), QtCore.QPointF(plot.right(), positive_y))
            painter.drawLine(QtCore.QPointF(plot.left(), negative_y), QtCore.QPointF(plot.right(), negative_y))
        else:
            threshold_y = plot.bottom() - (self.threshold / max_value) * plot.height()
            painter.drawLine(QtCore.QPointF(plot.left(), threshold_y), QtCore.QPointF(plot.right(), threshold_y))

        path = QtGui.QPainterPath()
        for index, value in enumerate(values):
            p = self._point(index, value)
            if index == 0:
                path.moveTo(p)
            else:
                path.lineTo(p)
        painter.setPen(QtGui.QPen(self.line_color, 2))
        painter.drawPath(path)

        for index, row in enumerate(self.rows):
            value = self._row_value(row)
            p = self._point(index, value)
            if abs(value) > self.threshold:
                painter.setBrush(QtGui.QColor(225, 70, 60))
                painter.setPen(QtGui.QColor(225, 70, 60))
                painter.drawEllipse(p, 5, 5)
            else:
                painter.setBrush(self.line_color)
                painter.setPen(self.line_color)
                painter.drawEllipse(p, 4, 4)
            if index == self.highlight_index:
                painter.setBrush(QtCore.Qt.NoBrush)
                painter.setPen(QtGui.QPen(QtGui.QColor(95, 190, 255), 3))
                painter.drawEllipse(p, 9, 9)

        painter.setPen(QtGui.QColor(210, 212, 216))
        painter.drawText(QtCore.QPointF(plot.left(), rect.bottom() - 13), "%s（点を上下ドラッグで調整）" % self.label)
        painter.setPen(QtGui.QColor(225, 90, 80))
        painter.drawText(QtCore.QPointF(plot.left() + 190, rect.bottom() - 13), "しきい値超え")

        painter.setPen(QtGui.QColor(170, 174, 182))
        painter.drawText(QtCore.QPointF(8, plot.top() + 10), "%.0f" % max_value)
        if self.signed:
            painter.drawText(QtCore.QPointF(8, plot.bottom()), "%.0f" % -max_value)
            painter.drawText(QtCore.QPointF(12, plot.center().y() - 2), "0")
        else:
            painter.drawText(QtCore.QPointF(12, plot.bottom()), "0")


class TailCodeTATool(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        _require_maya()
        self.setObjectName(WINDOW_OBJECT_NAME)
        self._configure_window()
        self.chains = []
        self.callbacks = []
        self.is_monitoring = False
        self._monitored_joint_paths = set()
        self._pending_score_update = False
        self._last_auto_score_update = 0.0
        self._auto_score_update_interval_ms = 160
        self._tweaker_scan_cache = None
        self._pending_graph_edits = {}
        self._pending_graph_edit_label = ""
        self._pending_graph_drag = None
        self._pending_graph_undo_stack = []
        self._pending_graph_redo_stack = []
        self._graph_edit_undo_stack = []
        self._graph_edit_redo_stack = []
        self._live_graph_drag_snapshot = None
        self._live_twist_drag = None
        self.graph_undo_open = False
        self._build_ui()
        self._install_undo_redo_shortcuts()
        self.load_scene_data()
        self.refresh_all()

    def _configure_window(self):
        self.setWindowTitle("尻尾・コード TA ツール")
        flags = (
            self.windowFlags()
            | QtCore.Qt.Window
            | QtCore.Qt.WindowSystemMenuHint
            | QtCore.Qt.WindowMinimizeButtonHint
        )
        self.setWindowFlags(flags)
        self.setSizeGripEnabled(True)
        self.setMinimumSize(WINDOW_MIN_WIDTH, WINDOW_MIN_HEIGHT)
        self.resize(WINDOW_DEFAULT_WIDTH, self._initial_window_height())

    def _initial_window_height(self):
        geometry = self._available_screen_geometry()
        if geometry is None:
            return WINDOW_DEFAULT_HEIGHT
        available_height = max(WINDOW_MIN_HEIGHT, geometry.height() - WINDOW_SCREEN_MARGIN)
        return min(WINDOW_DEFAULT_HEIGHT, available_height)

    def _available_screen_geometry(self):
        screen = self.screen() if hasattr(self, "screen") else None
        app = QtWidgets.QApplication.instance()
        if screen is None and app is not None and hasattr(app, "primaryScreen"):
            screen = app.primaryScreen()
        if screen is not None:
            return screen.availableGeometry()
        if hasattr(QtWidgets.QApplication, "desktop"):
            desktop = QtWidgets.QApplication.desktop()
            return desktop.availableGeometry(self)
        return None

    def _create_scroll_layout(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        self.scroll_area = QtWidgets.QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.scroll_area.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)

        self.scroll_content = QtWidgets.QWidget()
        main = QtWidgets.QVBoxLayout(self.scroll_content)
        self.scroll_area.setWidget(self.scroll_content)
        outer.addWidget(self.scroll_area)
        return main

    def minimize_window(self):
        self.showMinimized()

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
        if self.undo_graph_edit():
            return
        try:
            cmds.undo()
            self.update_scores_and_display()
        except Exception as exc:
            self.tweaker_status.setText("Undo failed: %s" % exc)

    def _maya_redo(self):
        if self.redo_graph_edit():
            return
        try:
            cmds.redo()
            self.update_scores_and_display()
        except Exception as exc:
            self.tweaker_status.setText("Redo failed: %s" % exc)

    def _graph_edit_snapshot(self):
        chain = self._selected_chain()
        if chain is None:
            return {}
        nodes = []
        for joint in chain.joints:
            if cmds.objExists(joint) and joint not in nodes:
                nodes.append(joint)
            for tweaker in self._tweakers_for_joint(joint):
                if cmds.objExists(tweaker) and tweaker not in nodes:
                    nodes.append(tweaker)
        snapshot = {}
        for node in nodes:
            if cmds.objExists(node + ".rotate"):
                snapshot[node] = _rotate_values(node)
        return snapshot

    def _graph_snapshots_differ(self, before, after):
        if set(before) != set(after):
            return True
        for node, before_values in before.items():
            after_values = after.get(node)
            if after_values is None:
                return True
            if any(abs(before_values[i] - after_values[i]) > 0.0001 for i in range(3)):
                return True
        return False

    def _push_graph_edit_snapshot(self, before, after):
        if not self._graph_snapshots_differ(before, after):
            return
        # Reason: graph edits need a tool-local history because Maya undo records drag setAttr steps too finely.
        self._graph_edit_undo_stack.append((before, after))
        self._graph_edit_redo_stack = []

    def _restore_graph_edit_snapshot(self, snapshot, fallback_snapshot=None):
        fallback_snapshot = fallback_snapshot or {}
        for node in fallback_snapshot:
            if node not in snapshot and cmds.objExists(node + ".rotate"):
                self._set_rotate_values(node, (0.0, 0.0, 0.0))
        for node, values in snapshot.items():
            if cmds.objExists(node + ".rotate"):
                self._set_rotate_values(node, values)
        self.update_scores_and_display()

    def undo_graph_edit(self):
        self.end_graph_undo()
        if not self.is_monitoring and self._undo_pending_graph_edit():
            return True
        if not self._graph_edit_undo_stack:
            return False
        before, after = self._graph_edit_undo_stack.pop()
        self._graph_edit_redo_stack.append((before, after))
        self._restore_graph_edit_snapshot(before, after)
        self.tweaker_status.setText("グラフ編集をUndoしました。")
        return True

    def redo_graph_edit(self):
        self.end_graph_undo()
        if not self.is_monitoring and self._redo_pending_graph_edit():
            return True
        if not self._graph_edit_redo_stack:
            return False
        before, after = self._graph_edit_redo_stack.pop()
        self._graph_edit_undo_stack.append((before, after))
        self._restore_graph_edit_snapshot(after, before)
        self.tweaker_status.setText("グラフ編集をRedoしました。")
        return True
    def _build_ui(self):
        main = self._create_scroll_layout()

        register_box = QtWidgets.QGroupBox("チェーン登録")
        reg_layout = QtWidgets.QGridLayout(register_box)
        self.label_edit = QtWidgets.QLineEdit()
        self.label_edit.setPlaceholderText("例：右手_親指 / 尻尾A")
        self.joints_edit = QtWidgets.QPlainTextEdit()
        self.joints_edit.setPlaceholderText("ジョイント名 / コントローラ名を1行ずつ入力。空欄の場合は現在の選択を使います。")
        self.joints_edit.setMaximumHeight(78)
        self.threshold_spin = QtWidgets.QDoubleSpinBox()
        self.threshold_spin.setRange(0.01, 9999.0)
        self.threshold_spin.setValue(15.0)
        self.threshold_spin.setDecimals(2)
        self.threshold_spin.setSuffix(" 度（曲がり）")
        self.threshold_spin.valueChanged.connect(self.on_threshold_changed)
        self.controller_child_depth_spin = QtWidgets.QSpinBox()
        self.controller_child_depth_spin.setRange(0, CONTROLLER_CHILD_EXPAND_MAX_DEPTH)
        self.controller_child_depth_spin.setValue(CONTROLLER_CHILD_EXPAND_DEFAULT_DEPTH)
        self.controller_child_depth_spin.setSuffix(" 段")
        self.controller_child_depth_spin.setToolTip("コントローラから見つかったジョイントの子方向を追加する段数です。")
        add_button = QtWidgets.QPushButton("選択チェーンを登録")
        add_button.clicked.connect(self.register_chain)
        update_button = QtWidgets.QPushButton("選択チェーンを更新")
        update_button.clicked.connect(self.update_selected_chain)
        remove_button = QtWidgets.QPushButton("削除")
        remove_button.clicked.connect(self.remove_selected_chain)
        reg_layout.addWidget(QtWidgets.QLabel("ラベル"), 0, 0)
        reg_layout.addWidget(self.label_edit, 0, 1, 1, 3)
        reg_layout.addWidget(QtWidgets.QLabel("ジョイント / コントローラ"), 1, 0)
        reg_layout.addWidget(self.joints_edit, 1, 1, 1, 3)
        reg_layout.addWidget(QtWidgets.QLabel("角度差しきい値"), 2, 0)
        reg_layout.addWidget(self.threshold_spin, 2, 1)
        reg_layout.addWidget(add_button, 2, 2)
        reg_layout.addWidget(update_button, 2, 3)
        reg_layout.addWidget(QtWidgets.QLabel("コントローラ子ジョイント"), 3, 0)
        reg_layout.addWidget(self.controller_child_depth_spin, 3, 1)
        reg_layout.addWidget(remove_button, 3, 3)
        main.addWidget(register_box)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        self.chain_list = QtWidgets.QTreeWidget()
        self.chain_list.setColumnCount(2)
        self.chain_list.setHeaderLabels(["登録済みチェーン / ジョイント", "Tweaker"])
        self.chain_list.header().setStretchLastSection(False)
        self.chain_list.header().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.chain_list.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.chain_list.setRootIsDecorated(True)
        self.chain_list.currentItemChanged.connect(self.on_chain_selected)
        self.chain_list.itemChanged.connect(self._visibility_changed)
        left_layout.addWidget(QtWidgets.QLabel("登録済みチェーン（右列でTweaker状態を確認）"))
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
        self.angle_table = QtWidgets.QTableWidget(0, 12)
        self.angle_table.setHorizontalHeaderLabels(
            [
                "ジョイント",
                "曲がり",
                "ねじれ",
                "|ねじれ|",
                "ねじれ軸",
                "回転 X",
                "回転 Y",
                "回転 Z",
                "差分 X",
                "差分 Y",
                "差分 Z",
                "最大差",
            ]
        )
        self.angle_table.horizontalHeader().setStretchLastSection(True)
        self.angle_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        for column in range(1, 11):
            self.angle_table.horizontalHeader().setSectionResizeMode(column, QtWidgets.QHeaderView.ResizeToContents)
        self.angle_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.angle_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.angle_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.angle_table.itemSelectionChanged.connect(self.on_angle_table_selection_changed)
        self.angle_table.cellClicked.connect(self.on_angle_table_cell_clicked)
        self.angle_table.currentCellChanged.connect(self.on_angle_table_current_cell_changed)
        self.angle_table.setMinimumHeight(190)
        axis_layout = QtWidgets.QHBoxLayout()
        axis_layout.addWidget(QtWidgets.QLabel("選択ジョイントのねじれ軸"))
        self.twist_axis_combo = QtWidgets.QComboBox()
        for label, value in [("Auto", ""), ("X", "X"), ("Y", "Y"), ("Z", "Z")]:
            self.twist_axis_combo.addItem(label, value)
        self.twist_axis_combo.setEnabled(False)
        self.twist_axis_combo.currentIndexChanged.connect(self.on_twist_axis_override_changed)
        axis_layout.addWidget(self.twist_axis_combo)
        axis_layout.addStretch()
        self.angle_graph = BendGraphWidget()
        self.angle_graph.bendEditStarted.connect(self.begin_graph_undo)
        self.angle_graph.bendEdited.connect(self.apply_bend_edit)
        self.angle_graph.bendEditFinished.connect(self.end_graph_undo)
        self.twist_graph = BendGraphWidget(
            value_key="twist",
            label="ねじれ",
            color=QtGui.QColor(100, 205, 210),
            min_display_value=10.0,
            signed=True,
        )
        self.twist_graph.bendEditStarted.connect(self.begin_graph_undo)
        self.twist_graph.bendEdited.connect(self.apply_twist_edit)
        self.twist_graph.bendEditFinished.connect(self.end_graph_undo)
        right_layout.addWidget(self.score_label)
        right_layout.addLayout(axis_layout)
        right_layout.addWidget(self.angle_table)
        right_layout.addWidget(QtWidgets.QLabel("現在フレームの曲がり折れ線グラフ"))
        right_layout.addWidget(self.angle_graph)
        right_layout.addWidget(QtWidgets.QLabel("現在フレームのねじれグラフ"))
        right_layout.addWidget(self.twist_graph)
        splitter.addWidget(right)
        splitter.setStretchFactor(1, 2)
        main.addWidget(splitter, 1)

        monitor_layout = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("編集・自動更新 開始")
        self.stop_button = QtWidgets.QPushButton("編集・自動更新 停止")
        self.graph_undo_button = QtWidgets.QPushButton("戻る")
        self.graph_redo_button = QtWidgets.QPushButton("進む")
        minimize_button = QtWidgets.QPushButton("最小化")
        self.start_button.clicked.connect(self.start_monitoring)
        self.stop_button.clicked.connect(self.stop_monitoring)
        self.graph_undo_button.clicked.connect(self.undo_graph_edit)
        self.graph_redo_button.clicked.connect(self.redo_graph_edit)
        minimize_button.clicked.connect(self.minimize_window)
        self.stop_button.setEnabled(False)
        monitor_layout.addWidget(self.start_button)
        monitor_layout.addWidget(self.stop_button)
        # Reason: graph edits use a dedicated history so users can undo one drag/apply step from the tool.
        monitor_layout.addWidget(self.graph_undo_button)
        monitor_layout.addWidget(self.graph_redo_button)
        monitor_layout.addWidget(minimize_button)
        monitor_layout.addStretch()
        main.addLayout(monitor_layout)

        tweaker_box = QtWidgets.QGroupBox("Tail Tweaker")
        tweaker_layout = QtWidgets.QGridLayout(tweaker_box)
        enable_label_button = QtWidgets.QPushButton("ラベル Tweaker ON")
        enable_label_button.clicked.connect(self.enable_selected_label_tweakers)
        disable_label_button = QtWidgets.QPushButton("ラベル Tweaker OFF")
        disable_label_button.clicked.connect(self.disable_selected_label_tweakers)
        self.tweaker_status = QtWidgets.QLabel("選択チェーンの Tail Tweaker を ON/OFF できます。")
        self.tweaker_status.setWordWrap(True)
        tweaker_layout.addWidget(enable_label_button, 0, 0)
        tweaker_layout.addWidget(disable_label_button, 0, 1)
        tweaker_layout.addWidget(self.tweaker_status, 1, 0, 1, 2)
        main.addWidget(tweaker_box)
        self.cancel_scan = False

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

    def _current_angle_row(self):
        row = self.angle_table.currentRow()
        if row >= 0:
            return row
        selected_rows = sorted({item.row() for item in self.angle_table.selectedItems()})
        return selected_rows[0] if selected_rows else -1

    def _set_twist_axis_combo(self, axis, enabled):
        if not hasattr(self, "twist_axis_combo"):
            return
        axis = _axis_override_name(axis)
        index = self.twist_axis_combo.findData(axis)
        if index < 0:
            index = 0
        self.twist_axis_combo.blockSignals(True)
        try:
            self.twist_axis_combo.setCurrentIndex(index)
            self.twist_axis_combo.setEnabled(bool(enabled))
        finally:
            self.twist_axis_combo.blockSignals(False)

    def _sync_twist_axis_combo(self, row_index=None):
        chain = self._selected_chain()
        if row_index is None:
            row_index = self._current_angle_row()
        if chain is None or not (0 < row_index < len(chain.joints)):
            self._set_twist_axis_combo("", False)
            return
        joint = chain.joints[row_index]
        self._set_twist_axis_combo(chain.twist_axis_overrides.get(joint, ""), True)

    def on_twist_axis_override_changed(self, *_args):
        chain = self._selected_chain()
        row = self._current_angle_row()
        if chain is None or not (0 < row < len(chain.joints)):
            return
        joint = chain.joints[row]
        axis = _axis_override_name(self.twist_axis_combo.itemData(self.twist_axis_combo.currentIndex()))
        if axis:
            chain.twist_axis_overrides[joint] = axis
        else:
            chain.twist_axis_overrides.pop(joint, None)
        self._live_twist_drag = None
        self.save_scene_data()
        self.refresh_details(chain)
        self.angle_table.selectRow(row)
        self._sync_twist_axis_combo(row)
        label = axis if axis else "Auto"
        self.tweaker_status.setText("ねじれ軸設定: %s -> %s" % (joint, label))

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
            inputs = [line.strip() for line in raw.replace(",", "\n").splitlines() if line.strip()]
        else:
            inputs = cmds.ls(selection=True, long=False) or []
        child_depth = int(self.controller_child_depth_spin.value())
        joints, unresolved = _resolve_chain_joints(inputs, child_depth=child_depth)
        if unresolved:
            cmds.warning("ジョイントへ解決できない入力をスキップしました: %s" % ", ".join(unresolved))
        return joints

    def register_chain(self):
        try:
            joints = self._input_joints()
            if len(joints) < 3:
                cmds.warning("3つ以上のジョイント、または対応コントローラを指定してください。")
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
                chain.twist_axis_overrides = {
                    joint: axis
                    for joint, axis in chain.twist_axis_overrides.items()
                    if any(_same_dag_node(joint, chain_joint) for chain_joint in chain.joints)
                }
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
        # Keep the joint search/input field under user control when browsing chains.
        self.threshold_spin.blockSignals(True)
        try:
            self.threshold_spin.setValue(chain.threshold)
        finally:
            self.threshold_spin.blockSignals(False)
        self.refresh_details(chain)
        self._sync_twist_axis_combo()

    def refresh_all(self):
        self.chain_list.blockSignals(True)
        current = self._selected_chain_index()
        self.chain_list.clear()
        # Reason: tree rebuild asks for tweakers per joint; cache one scene scan for this refresh only.
        self._tweaker_scan_cache = self._scan_all_tweakers()
        try:
            for index, chain in enumerate(self.chains):
                label = "%02d  %s" % (index + 1, chain.label)
                state_label, state_color = self._label_tweaker_state(chain.label)
                item = QtWidgets.QTreeWidgetItem([label, state_label])
                item.setData(0, QtCore.Qt.UserRole, index)
                item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
                item.setCheckState(0, QtCore.Qt.Checked if chain.visible else QtCore.Qt.Unchecked)
                color = QtGui.QColor.fromRgbF(*COLORS[chain.color_index % len(COLORS)])
                item.setForeground(0, color)
                item.setForeground(1, state_color)
                self.chain_list.addTopLevelItem(item)
                for joint in chain.joints:
                    tweaker = self._tweaker_for_joint_and_label(joint, chain.label)
                    other_count = max(0, len(self._tweakers_for_joint(joint)) - (1 if tweaker else 0))
                    suffix = "  [T]" if tweaker else ""
                    if other_count:
                        suffix += " [+%d]" % other_count
                    child_state = self._tweaker_state_label(tweaker) if tweaker else "なし"
                    child = QtWidgets.QTreeWidgetItem(["%s%s" % (joint, suffix), child_state])
                    child.setData(0, QtCore.Qt.UserRole, index)
                    child.setForeground(0, QtGui.QColor(185, 188, 194))
                    child.setForeground(1, self._state_color(child_state))
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
        self._live_twist_drag = None
        if not self.is_monitoring:
            self._pending_graph_drag = None
            return
        if self.graph_undo_open:
            return
        # Reason: capture one before-state per graph drag instead of relying on Maya's per-setAttr undo history.
        self._live_graph_drag_snapshot = self._graph_edit_snapshot()
        self.graph_undo_open = True

    def end_graph_undo(self):
        if not self.is_monitoring:
            self._finish_pending_graph_drag()
            return
        if not self.graph_undo_open:
            return
        before = self._live_graph_drag_snapshot or {}
        self._live_graph_drag_snapshot = None
        self._live_twist_drag = None
        self.graph_undo_open = False
        self._push_graph_edit_snapshot(before, self._graph_edit_snapshot())

    def on_threshold_changed(self, value):
        self.apply_threshold_to_selected_chain(value, save=True, refresh=True)

    def apply_threshold_to_selected_chain(self, value, save=True, refresh=True):
        chain = self._selected_chain()
        if chain is None:
            return
        threshold = float(value)
        if abs(chain.threshold - threshold) < 0.0001:
            return
        chain.threshold = threshold
        if save:
            self.save_scene_data()
        if refresh:
            self.refresh_details(chain)

    def refresh_details(self, chain):
        result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold, chain.twist_axis_overrides)
        chain.last_score = result["score"]
        chain.problem_joints = result["problem_joints"]
        self.score_label.setText(
            "曲がりスコア: %.3f    ねじれスコア: %.3f    チェーン: %s"
            % (chain.last_score, result["twist_score"], chain.label)
        )
        self._fill_angle_table(result["angle_rows"], chain.threshold)
        self.angle_graph.set_angle_rows(result["angle_rows"], chain.threshold)
        self.twist_graph.set_angle_rows(result["angle_rows"], chain.threshold)

    def _graph_for_metric(self, metric):
        return self.twist_graph if metric == "twist" else self.angle_graph

    def _graph_value_key(self, metric):
        return "twist" if metric == "twist" else "bend"

    def _preview_pending_graph_edit(self, metric, row_index, target_value):
        graph = self._graph_for_metric(metric)
        value_key = self._graph_value_key(metric)
        if 0 <= row_index < len(graph.rows):
            graph.rows[row_index][value_key] = float(target_value)
            graph.set_highlight_index(row_index)
            other_graph = self.angle_graph if graph is self.twist_graph else self.twist_graph
            other_graph.set_highlight_index(row_index)
            graph.update()

    def _pending_graph_value(self, metric, row_index):
        graph = self._graph_for_metric(metric)
        value_key = self._graph_value_key(metric)
        if 0 <= row_index < len(graph.rows):
            return float(graph.rows[row_index].get(value_key, 0.0))
        return 0.0

    def _finish_pending_graph_drag(self):
        if not self._pending_graph_drag:
            return
        metric, row_index, start_value = self._pending_graph_drag
        key = (metric, row_index)
        end_value = float(self._pending_graph_edits.get(key, self._pending_graph_value(metric, row_index)))
        self._pending_graph_drag = None
        if abs(end_value - start_value) < 0.001:
            return
        # Reason: one graph drag should become one preview undo step, not many mouse-move steps.
        self._pending_graph_undo_stack.append((metric, row_index, start_value, end_value))
        self._pending_graph_redo_stack = []

    def _set_pending_graph_edit_value(self, metric, row_index, target_value):
        self._pending_graph_edits[(metric, int(row_index))] = float(target_value)
        self._preview_pending_graph_edit(metric, row_index, target_value)

    def _undo_pending_graph_edit(self):
        self._finish_pending_graph_drag()
        if not self._pending_graph_undo_stack:
            return False
        metric, row_index, old_value, new_value = self._pending_graph_undo_stack.pop()
        self._pending_graph_redo_stack.append((metric, row_index, old_value, new_value))
        self._set_pending_graph_edit_value(metric, row_index, old_value)
        self.tweaker_status.setText("停止中グラフ編集をUndoしました。")
        return True

    def _redo_pending_graph_edit(self):
        if not self._pending_graph_redo_stack:
            return False
        metric, row_index, old_value, new_value = self._pending_graph_redo_stack.pop()
        self._pending_graph_undo_stack.append((metric, row_index, old_value, new_value))
        self._set_pending_graph_edit_value(metric, row_index, new_value)
        self.tweaker_status.setText("停止中グラフ編集をRedoしました。")
        return True

    def apply_bend_edit(self, row_index, target_bend):
        self._apply_graph_metric_edit("bend", row_index, target_bend)

    def apply_twist_edit(self, row_index, target_twist):
        self._apply_graph_metric_edit("twist", row_index, target_twist)

    def _apply_graph_metric_edit(self, metric, row_index, target_value):
        if not self.is_monitoring:
            chain = self._selected_chain()
            if chain is None:
                return
            if self._pending_graph_edit_label != chain.label:
                self._pending_graph_edits = {}
                self._pending_graph_drag = None
                self._pending_graph_undo_stack = []
                self._pending_graph_redo_stack = []
                self._pending_graph_edit_label = chain.label
            if (
                self._pending_graph_drag is None
                or self._pending_graph_drag[0] != metric
                or self._pending_graph_drag[1] != int(row_index)
            ):
                self._pending_graph_drag = (
                    metric,
                    int(row_index),
                    self._pending_graph_value(metric, int(row_index)),
                )
            # Reason: stopped mode previews graph edits and defers scene changes into one undoable apply step.
            self._set_pending_graph_edit_value(metric, row_index, target_value)
            self.tweaker_status.setText("編集・自動更新 開始時にグラフ編集をまとめて反映します。")
            return
        if metric == "twist":
            changed = self._apply_twist_edit(row_index, target_value, select_target=True)
        else:
            changed = self._apply_bend_edit(row_index, target_value, select_target=True)
        if changed:
            self.update_scores_and_display()

    def _apply_bend_edit(self, row_index, target_bend, select_target=False):
        chain = self._selected_chain()
        if chain is None:
            return False
        result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold, chain.twist_axis_overrides)
        rows = result["angle_rows"]
        if not (0 <= row_index < len(rows)):
            return False

        row = rows[row_index]
        joint = row["joint"]
        current_bend = max(row["bend"], 0.001)
        ratio = max(0.0, min(3.0, float(target_bend) / current_bend))
        if abs(ratio - 1.0) < 0.005:
            return False

        rotate = row["rotate"]
        twist_axis = JointAngleAnalyzer.bend_twist_axis(rows, row_index)
        direction = list(JointAngleAnalyzer.bend_direction(rows, row_index))
        if max(abs(v) for v in direction) < 0.001:
            self.tweaker_status.setText("ねじれ軸を除いた曲がり方向がありません。先に曲がり側の軸へ少し回転を付けてください。")
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
            new_values = tuple(rotate[i] + delta[i] for i in range(3))
            self._set_rotate_values(joint, new_values)
            target_node = joint

        if select_target:
            cmds.select(target_node, replace=True)
        axis_note = " / ねじれ軸 %s は固定" % twist_axis if twist_axis in "XYZ" else ""
        self.tweaker_status.setText(
            "曲がり調整: %s  %.2f -> %.2f / 曲がり軸のみ%s" % (joint, row["bend"], target_bend, axis_note)
        )
        return True

    def _twist_drag_state(self, row_index, row):
        if not self.graph_undo_open:
            return None
        joint = row["joint"]
        state = self._live_twist_drag
        if state and state["row_index"] == int(row_index) and state["joint"] == joint:
            return state
        axis = row.get("twist_axis", "-")
        if axis not in "XYZ":
            return None
        current_twist = float(row.get("twist", 0.0))
        state = {
            "row_index": int(row_index),
            "joint": joint,
            "axis": axis,
            "last_twist": current_twist,
        }
        self._live_twist_drag = state
        return state

    def _twist_delta_value(self, current_twist, target_twist, drag_state):
        if drag_state:
            desired = target_twist - drag_state["last_twist"]
            return max(-TWIST_EDIT_MAX_STEP_DEGREES, min(TWIST_EDIT_MAX_STEP_DEGREES, desired))
        return target_twist - current_twist

    def _twist_response_value(self, chain, row_index):
        result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold, chain.twist_axis_overrides)
        rows = result["angle_rows"]
        if not (0 <= row_index < len(rows)):
            return None
        return float(rows[row_index].get("twist", 0.0))

    def _apply_twist_edit(self, row_index, target_twist, select_target=False):
        chain = self._selected_chain()
        if chain is None:
            return False
        result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold, chain.twist_axis_overrides)
        rows = result["angle_rows"]
        if not (0 <= row_index < len(rows)):
            return False
        if row_index == 0:
            self.tweaker_status.setText("Root は親との差分がないためねじれ調整できません。")
            return False

        row = rows[row_index]
        joint = row["joint"]
        drag_state = self._twist_drag_state(row_index, row)
        axis = drag_state["axis"] if drag_state else row.get("twist_axis", "-")
        if axis not in "XYZ":
            self.tweaker_status.setText("ねじれ軸を判定できません。先に少し回転差を付けてください。")
            return False
        current_twist = float(row.get("twist", 0.0))
        target_twist = float(target_twist)
        delta_value = self._twist_delta_value(current_twist, target_twist, drag_state)
        if abs(delta_value) < 0.005:
            return False

        tweakers = self._tweakers_for_joint(joint)
        has_incoming = any(_attr_has_incoming_connection("%s.rotate%s" % (joint, check_axis)) for check_axis in "XYZ")

        previous_values = None
        if tweakers:
            target_node = tweakers[0]
            current_tweaker_rotate = _rotate_values(target_node)
            previous_values = current_tweaker_rotate
            values = list(current_tweaker_rotate)
            values["XYZ".index(axis)] += delta_value
            self._set_rotate_values(target_node, tuple(values))
        elif has_incoming:
            target_node = self._create_tweaker(joint)
            previous_values = _rotate_values(target_node)
            values = [0.0, 0.0, 0.0]
            values["XYZ".index(axis)] = delta_value
            self._set_rotate_values(target_node, tuple(values))
        else:
            rotate = list(row["rotate"])
            previous_values = tuple(rotate)
            rotate["XYZ".index(axis)] += delta_value
            self._set_rotate_values(joint, tuple(rotate))
            target_node = joint

        response_twist = None
        if drag_state:
            response_twist = self._twist_response_value(chain, row_index)
            if response_twist is None or abs(response_twist - drag_state["last_twist"]) > TWIST_EDIT_MAX_RESPONSE_DEGREES:
                if previous_values is not None:
                    self._set_rotate_values(target_node, previous_values)
                self._live_twist_drag = None
                self.tweaker_status.setText("ねじれ値が急変したため調整を止めました。ねじれ軸を確認してください: %s" % joint)
                return False
            drag_state["last_twist"] = response_twist

        if select_target:
            cmds.select(target_node, replace=True)
        display_target = response_twist if response_twist is not None else target_twist
        self.tweaker_status.setText(
            "ねじれ調整: %s  %s %.2f -> %.2f"
            % (joint, axis, current_twist, display_target)
        )
        return True

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
            self.twist_graph.set_highlight_index(None)
            self._set_twist_axis_combo("", False)
            return
        self._select_joint_from_angle_table_row(selected_rows[0])

    def _select_joint_from_angle_table_row(self, row_index):
        if row_index < 0 or row_index >= self.angle_table.rowCount():
            self.angle_graph.set_highlight_index(None)
            self.twist_graph.set_highlight_index(None)
            return

        item = self.angle_table.item(row_index, 0)
        joint = item.text() if item else ""
        if not joint:
            chain = self._selected_chain()
            joint = chain.joints[row_index] if chain and 0 <= row_index < len(chain.joints) else ""

        self.angle_graph.set_highlight_index(row_index)
        self.twist_graph.set_highlight_index(row_index)
        self._sync_twist_axis_combo(row_index)
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
            twist_axis = row["twist_axis"]
            if row.get("twist_axis_manual") and twist_axis in "XYZ":
                twist_axis += "*"
            values = [
                row["joint"],
                "%.2f" % row["bend"],
                "%.2f" % row["twist"],
                "%.2f" % row["twist_abs"],
                twist_axis,
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
            result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold, chain.twist_axis_overrides)
            chain.last_score = result["score"]
            chain.problem_joints = result["problem_joints"]
            self._update_display_curve(chain, result["positions"])
        if selected:
            self.refresh_details(selected)

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
            self._apply_pending_graph_edits()
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
        elapsed_ms = (time.monotonic() - self._last_auto_score_update) * 1000.0
        delay = max(40, int(self._auto_score_update_interval_ms - elapsed_ms))
        # Reason: monitored rotate edits can fire continuously; throttle auto refresh while keeping manual refresh immediate.
        QtCore.QTimer.singleShot(delay, self._run_pending_scores_update)

    def _run_pending_scores_update(self):
        self._pending_score_update = False
        if not self.is_monitoring:
            return
        self._last_auto_score_update = time.monotonic()
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
        self._last_auto_score_update = 0.0
        self._live_twist_drag = None
        self.is_monitoring = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def _clear_pending_graph_edits(self):
        self._pending_graph_edits = {}
        self._pending_graph_drag = None
        self._pending_graph_undo_stack = []
        self._pending_graph_redo_stack = []
        self._pending_graph_edit_label = ""

    def _apply_pending_graph_edits(self):
        if not self._pending_graph_edits:
            return
        chain = self._selected_chain()
        if chain is None or chain.label != self._pending_graph_edit_label:
            self._clear_pending_graph_edits()
            return
        self._finish_pending_graph_drag()
        edits = sorted(self._pending_graph_edits.items())
        if not edits:
            self._clear_pending_graph_edits()
            return
        before = self._graph_edit_snapshot()
        changed = False
        for key, target_value in edits:
            metric, row_index = key
            if metric == "twist":
                changed = self._apply_twist_edit(row_index, target_value, select_target=False) or changed
            else:
                changed = self._apply_bend_edit(row_index, target_value, select_target=False) or changed

        after = self._graph_edit_snapshot()
        self._clear_pending_graph_edits()
        if changed:
            # Reason: applying stopped graph edits should undo/redo as one tool action even when Maya undo chunks are unreliable.
            self._push_graph_edit_snapshot(before, after)
            self.update_scores_and_display()

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

    def _tweakers_for_label(self, label):
        return [tweaker for tweaker in self._all_tweakers() if _get_string_attr(tweaker, "ownerChainLabel") == label]

    def _disconnect_attr_if_connected(self, source, destination):
        if source and destination and cmds.objExists(source) and cmds.objExists(destination):
            try:
                if cmds.isConnected(source, destination):
                    cmds.disconnectAttr(source, destination)
            except Exception:
                pass

    def _connect_attr_if_possible(self, source, destination):
        if source and destination and cmds.objExists(source) and cmds.objExists(destination):
            try:
                if not cmds.isConnected(source, destination):
                    cmds.connectAttr(source, destination, force=True)
                return True
            except Exception:
                return False
        return False

    def _ensure_tweaker_enable_attr(self, tweaker, enabled=True):
        _add_float_attr(tweaker, "enableWeight", 1.0 if enabled else 0.0, min_value=0.0, max_value=1.0)
        _add_bool_attr(tweaker, "tailTweakerEnabled", bool(enabled))

    def _tweaker_is_enabled(self, tweaker):
        if not tweaker or not cmds.objExists(tweaker):
            return False
        if cmds.attributeQuery("enableWeight", node=tweaker, exists=True):
            try:
                return float(cmds.getAttr(tweaker + ".enableWeight")) > 0.5
            except Exception:
                pass
        if cmds.attributeQuery("tailTweakerEnabled", node=tweaker, exists=True):
            try:
                return bool(cmds.getAttr(tweaker + ".tailTweakerEnabled"))
            except Exception:
                pass
        return True

    def _tweaker_state_label(self, tweaker):
        return "ON" if self._tweaker_is_enabled(tweaker) else "OFF"

    def _state_color(self, state):
        if state == "ON":
            return QtGui.QColor(95, 210, 130)
        if state == "OFF":
            return QtGui.QColor(230, 105, 95)
        if state == "MIX":
            return QtGui.QColor(235, 190, 80)
        return QtGui.QColor(150, 150, 150)

    def _label_tweaker_state(self, label):
        tweakers = self._tweakers_for_label(label)
        if not tweakers:
            return "なし", self._state_color("なし")
        enabled_count = sum(1 for tweaker in tweakers if self._tweaker_is_enabled(tweaker))
        if enabled_count == len(tweakers):
            return "ON", self._state_color("ON")
        if enabled_count == 0:
            return "OFF", self._state_color("OFF")
        return "MIX", self._state_color("MIX")

    def _ensure_additive_weight_network(self, tweaker, target, axis):
        target_attr = "%s.rotate%s" % (target, axis)
        tweaker_attr = "%s.rotate%s" % (tweaker, axis)
        plus = _get_string_attr(tweaker, "addNode%s" % axis)
        if not plus or not cmds.objExists(plus) or not cmds.objExists(target_attr):
            return ""

        source_attr = _get_string_attr(tweaker, "sourceRotate%s" % axis)
        base_attr = "%s.input1D[0]" % plus
        add_attr = "%s.input1D[1]" % plus
        output_attr = "%s.output1D" % plus
        mult = _get_string_attr(tweaker, "enableNode%s" % axis)
        if not mult or not cmds.objExists(mult):
            mult = cmds.createNode("multDoubleLinear", name="%s_enable%s_MDL" % (tweaker, axis))
            _add_string_attr(tweaker, "enableNode%s" % axis, mult)

        if source_attr:
            self._disconnect_attr_if_connected(source_attr, target_attr)
            base_sources = cmds.listConnections(base_attr, source=True, destination=False, plugs=True) or []
            for src in base_sources:
                if src != source_attr:
                    self._disconnect_attr_if_connected(src, base_attr)
            self._connect_attr_if_possible(source_attr, base_attr)
        elif not (cmds.listConnections(base_attr, source=True, destination=False, plugs=True) or []):
            try:
                cmds.setAttr(base_attr, float(_get_string_attr(tweaker, "baseRotate%s" % axis, "0")))
            except Exception:
                pass

        add_sources = cmds.listConnections(add_attr, source=True, destination=False, plugs=True) or []
        for src in add_sources:
            if src != "%s.output" % mult:
                self._disconnect_attr_if_connected(src, add_attr)
        self._connect_attr_if_possible(tweaker_attr, "%s.input1" % mult)
        self._connect_attr_if_possible("%s.enableWeight" % tweaker, "%s.input2" % mult)
        self._connect_attr_if_possible("%s.output" % mult, add_attr)
        self._connect_attr_if_possible(output_attr, target_attr)
        return mult

    def _set_additive_tweaker_enabled(self, tweaker, enabled):
        target = _get_string_attr(tweaker, "targetJoint")
        if not _joint_exists(target):
            return
        self._ensure_tweaker_enable_attr(tweaker, enabled)
        for axis in "XYZ":
            self._ensure_additive_weight_network(tweaker, target, axis)
        cmds.setAttr("%s.enableWeight" % tweaker, 1.0 if enabled else 0.0)
        cmds.setAttr("%s.tailTweakerEnabled" % tweaker, bool(enabled))

    def _constraint_has_tweaker_target(self, constraint, tweaker):
        if not constraint or not cmds.objExists(constraint):
            return False
        try:
            targets = cmds.orientConstraint(constraint, query=True, targetList=True) or []
        except Exception:
            return False
        tweaker_long = _dag_path(tweaker)
        for target in targets:
            target_long = _dag_path(target) if cmds.objExists(target) else target
            if target == tweaker or target == tweaker_long or target_long == tweaker_long:
                return True
        return False

    def _set_constraint_tweaker_weight(self, constraint, tweaker, value):
        try:
            targets = cmds.orientConstraint(constraint, query=True, targetList=True) or []
            weights = cmds.orientConstraint(constraint, query=True, weightAliasList=True) or []
        except Exception:
            return
        tweaker_long = _dag_path(tweaker)
        for target, weight in zip(targets, weights):
            target_long = _dag_path(target) if cmds.objExists(target) else target
            if target == tweaker or target == tweaker_long or target_long == tweaker_long:
                attr = "%s.%s" % (constraint, weight)
                if cmds.objExists(attr):
                    cmds.setAttr(attr, float(value))

    def _set_constraint_tweaker_enabled(self, tweaker, enabled):
        target = _get_string_attr(tweaker, "targetJoint")
        if not _joint_exists(target):
            return
        self._ensure_tweaker_enable_attr(tweaker, enabled)
        constraint = _get_string_attr(tweaker, "constraintNode")
        if not constraint or not cmds.objExists(constraint):
            try:
                constraint = cmds.orientConstraint(tweaker, target, maintainOffset=True, name=tweaker + "_orientConstraint")[0]
                _add_string_attr(tweaker, "constraintNode", constraint)
                _add_bool_attr(tweaker, "usesExistingConstraint", False)
            except Exception as exc:
                cmds.warning("Tweaker constraint を再接続できませんでした: %s / %s" % (tweaker, exc))
                return
        elif not self._constraint_has_tweaker_target(constraint, tweaker):
            try:
                cmds.orientConstraint(tweaker, target, edit=True, maintainOffset=True, weight=1.0)
            except Exception:
                pass
        self._set_constraint_tweaker_weight(constraint, tweaker, 1.0 if enabled else 0.0)
        cmds.setAttr("%s.enableWeight" % tweaker, 1.0 if enabled else 0.0)
        cmds.setAttr("%s.tailTweakerEnabled" % tweaker, bool(enabled))

    def _set_tweaker_enabled(self, tweaker, enabled):
        if not cmds.objExists(tweaker):
            return
        mode = _get_string_attr(tweaker, "connectionMode", "constraint")
        if mode == "additiveRotate":
            self._set_additive_tweaker_enabled(tweaker, enabled)
        else:
            self._set_constraint_tweaker_enabled(tweaker, enabled)

    def _set_selected_label_tweakers_enabled(self, enabled):
        chain = self._selected_chain()
        if chain is None:
            cmds.warning("対象ラベルのチェーンを選択してください。")
            return
        tweakers = self._tweakers_for_label(chain.label)
        if not tweakers:
            cmds.warning("ラベルに紐づく Tweaker がありません: %s" % chain.label)
            return
        cmds.undoInfo(openChunk=True, chunkName="TA Tool Label Tweaker %s" % ("ON" if enabled else "OFF"))
        try:
            for tweaker in tweakers:
                self._set_tweaker_enabled(tweaker, enabled)
        finally:
            cmds.undoInfo(closeChunk=True)
        self.tweaker_status.setText(
            "ラベル Tweaker %s: %s / %d件" % ("ON" if enabled else "OFF", chain.label, len(tweakers))
        )
        self.refresh_all()

    def enable_selected_label_tweakers(self):
        self._set_selected_label_tweakers_enabled(True)

    def disable_selected_label_tweakers(self):
        self._set_selected_label_tweakers_enabled(False)

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
        self._ensure_tweaker_enable_attr(tweaker, True)

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
        self._set_tweaker_enabled(tweaker, True)
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
            mult = _get_string_attr(tweaker, "enableNode%s" % axis)
            if mult and cmds.objExists(mult):
                cmds.delete(mult)

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

    def auto_create_tweaker(self):
        try:
            chain = self._selected_chain()
            if chain is None:
                cmds.warning("Auto Tweaker 用のチェーンを選択してください。")
                return
            result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold, chain.twist_axis_overrides)
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
            result = JointAngleAnalyzer.evaluate(chain.joints, chain.threshold, chain.twist_axis_overrides)
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
