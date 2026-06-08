# -*- coding: utf-8 -*-
"""
Image Plane Hold Tool - Maya 2026 / Python 3

Purpose:
    Apply stepped image-sequence playback to an Image Plane.
    Default:
        frameExtension = floor((frame - 2) / 2) + 1

Safety:
    - Does not modify frameOffset.
    - Does not blindly delete unknown connections.
    - Reuses an existing expression only when it clearly drives frameExtension.
    - Reset removes only the assignment managed by this tool.
"""

from __future__ import annotations

import math
import re
from typing import List, Optional, Tuple

import maya.cmds as cmds


WINDOW_NAME = "imagePlaneHoldToolWindow"
WINDOW_TITLE = "Image Plane Hold Tool"
EXPR_MARKER = "// IPHT_IMAGE_PLANE_HOLD_TOOL"
DEFAULT_HOLD_FRAMES = 2
DEFAULT_START_FRAME = 2
DEFAULT_FIRST_IMAGE = 1

_UI = {}


def _short_name(node: str) -> str:
    return node.rsplit("|", 1)[-1]


def _safe_node_token(node: str) -> str:
    token = re.sub(r"[^0-9A-Za-z_]+", "_", _short_name(node))
    return token.strip("_") or "imagePlane"


def _all_image_plane_shapes() -> List[str]:
    return sorted(set(cmds.ls(type="imagePlane", long=True) or []))


def _shape_from_node(node: str) -> List[str]:
    """Return imagePlane shapes represented by node."""
    if not cmds.objExists(node):
        return []

    node_type = cmds.nodeType(node)

    if node_type == "imagePlane":
        return [cmds.ls(node, long=True)[0]]

    shapes = cmds.listRelatives(
        node,
        shapes=True,
        fullPath=True,
        noIntermediate=True,
    ) or []

    return [
        shape
        for shape in shapes
        if cmds.nodeType(shape) == "imagePlane"
    ]


def _selected_image_plane_shapes() -> List[str]:
    results = []
    for node in cmds.ls(selection=True, long=True) or []:
        results.extend(_shape_from_node(node))
    return sorted(set(results))


def _expression_body(expr: str) -> str:
    return cmds.expression(expr, query=True, string=True) or ""


def _target_assignment_pattern(target_attr: str) -> re.Pattern:
    return re.compile(
        rf"(?m)^[ \t]*{re.escape(target_attr)}[ \t]*=[ \t]*[^;]+;[ \t]*$"
    )


def _expressions_assigning(target_attr: str) -> List[str]:
    """Search expression text because listConnections may miss indirect wiring."""
    hits = []
    pattern = _target_assignment_pattern(target_attr)

    for expr in cmds.ls(type="expression") or []:
        try:
            body = _expression_body(expr)
        except Exception:
            continue

        if pattern.search(body):
            hits.append(expr)

    return sorted(set(hits))


def _incoming_sources(target_attr: str) -> List[str]:
    return sorted(
        set(
            cmds.listConnections(
                target_attr,
                source=True,
                destination=False,
                plugs=True,
                skipConversionNodes=True,
            )
            or []
        )
    )


def _build_assignment(
    shape: str,
    hold_frames: int,
    start_frame: int,
    first_image: int,
) -> str:
    return (
        f"{shape}.frameExtension = "
        f"floor((frame - {start_frame}) / {hold_frames}) + {first_image};"
    )


def _replace_or_append_assignment(
    body: str,
    target_attr: str,
    new_assignment: str,
) -> str:
    pattern = _target_assignment_pattern(target_attr)

    if pattern.search(body):
        body = pattern.sub(new_assignment, body, count=1)
    else:
        body = body.rstrip() + "\n" + new_assignment

    if EXPR_MARKER not in body:
        body = body.rstrip() + "\n" + EXPR_MARKER

    return body.rstrip() + "\n"


def _remove_managed_assignment(body: str, target_attr: str) -> str:
    pattern = _target_assignment_pattern(target_attr)
    body = pattern.sub("", body, count=1)
    body = body.replace(EXPR_MARKER, "")
    lines = [line.rstrip() for line in body.splitlines()]
    lines = [line for line in lines if line.strip()]
    return "\n".join(lines).strip()


def _set_log(message: str) -> None:
    print(message)

    field = _UI.get("log")
    if field and cmds.control(field, exists=True):
        cmds.scrollField(field, edit=True, text=message)


def _ui_selected_shape() -> Optional[str]:
    menu = _UI.get("shape_menu")
    if not menu or not cmds.control(menu, exists=True):
        return None

    value = cmds.optionMenu(menu, query=True, value=True)
    if not value or value == "<Image Planeなし>":
        return None

    return value


def _refresh_shape_menu(*_args) -> None:
    menu = _UI.get("shape_menu")
    if not menu or not cmds.control(menu, exists=True):
        return

    items = cmds.optionMenu(menu, query=True, itemListLong=True) or []
    for item in items:
        cmds.deleteUI(item)

    selected = _selected_image_plane_shapes()
    all_shapes = _all_image_plane_shapes()

    shapes = selected + [
        shape for shape in all_shapes if shape not in selected
    ]

    if not shapes:
        cmds.menuItem(label="<Image Planeなし>", parent=menu)
        _set_log(
            "Image Planeが見つかりません。\n"
            "先に Image Plane を読み込んでください。"
        )
        return

    for shape in shapes:
        cmds.menuItem(label=shape, parent=menu)

    cmds.optionMenu(menu, edit=True, value=shapes[0])

    _set_log(
        "対象候補を更新しました。\n"
        f"Image Plane数: {len(all_shapes)}\n"
        f"現在の対象: {shapes[0]}"
    )


def _read_ui_values() -> Tuple[int, int, int]:
    hold_frames = cmds.intField(
        _UI["hold_frames"], query=True, value=True
    )
    start_frame = cmds.intField(
        _UI["start_frame"], query=True, value=True
    )
    first_image = cmds.intField(
        _UI["first_image"], query=True, value=True
    )

    if hold_frames < 1:
        raise ValueError("保持フレーム数は1以上にしてください。")

    return hold_frames, start_frame, first_image


def apply_hold(*_args) -> None:
    shape = _ui_selected_shape()

    if not shape:
        cmds.warning("Image Planeを選択してください。")
        _set_log("適用失敗: Image Planeが選択されていません。")
        return

    try:
        hold_frames, start_frame, first_image = _read_ui_values()
    except ValueError as exc:
        cmds.warning(str(exc))
        _set_log(f"適用失敗: {exc}")
        return

    target_attr = f"{shape}.frameExtension"
    use_sequence_attr = f"{shape}.useFrameExtension"

    if not cmds.objExists(target_attr):
        cmds.warning(f"frameExtensionが見つかりません: {shape}")
        _set_log(f"適用失敗: {target_attr} がありません。")
        return

    try:
        cmds.setAttr(use_sequence_attr, True)
    except Exception as exc:
        cmds.warning(str(exc))
        _set_log(f"適用失敗: useFrameExtensionを有効化できません。\n{exc}")
        return

    assignments = _expressions_assigning(target_attr)
    incoming = _incoming_sources(target_attr)
    new_assignment = _build_assignment(
        shape,
        hold_frames,
        start_frame,
        first_image,
    )

    if len(assignments) > 1:
        message = (
            "適用を中止しました。\n"
            "frameExtensionを制御するexpressionが複数あります。\n"
            + "\n".join(assignments)
        )
        cmds.warning(message)
        _set_log(message)
        return

    if assignments:
        expr = assignments[0]
        old_body = _expression_body(expr)
        new_body = _replace_or_append_assignment(
            old_body,
            target_attr,
            new_assignment,
        )

        try:
            cmds.expression(expr, edit=True, string=new_body)
        except Exception as exc:
            cmds.warning(str(exc))
            _set_log(f"既存expressionの更新に失敗しました。\n{exc}")
            return

        action = f"既存expressionを更新: {expr}"

    else:
        if incoming:
            message = (
                "適用を中止しました。\n"
                "frameExtensionに未知の接続があります。\n"
                "自動削除は行いません。\n"
                "接続元:\n  - "
                + "\n  - ".join(incoming)
            )
            cmds.warning(message)
            _set_log(message)
            return

        expr_name = f"IPHT_{_safe_node_token(shape)}_holdExpr"
        body = new_assignment + "\n" + EXPR_MARKER

        try:
            expr = cmds.expression(
                name=expr_name,
                string=body,
                alwaysEvaluate=True,
                unitConversion="all",
            )
        except Exception as exc:
            cmds.warning(str(exc))
            _set_log(f"expressionの作成に失敗しました。\n{exc}")
            return

        action = f"新規expressionを作成: {expr}"

    _set_log(
        "適用完了\n"
        f"対象: {shape}\n"
        f"{action}\n"
        f"式: {new_assignment}\n"
        f"frameOffset: 変更していません"
    )


def reset_hold(*_args) -> None:
    shape = _ui_selected_shape()

    if not shape:
        cmds.warning("Image Planeを選択してください。")
        _set_log("解除失敗: Image Planeが選択されていません。")
        return

    target_attr = f"{shape}.frameExtension"
    assignments = _expressions_assigning(target_attr)

    if len(assignments) > 1:
        message = (
            "解除を中止しました。\n"
            "frameExtensionを制御するexpressionが複数あります。\n"
            + "\n".join(assignments)
        )
        cmds.warning(message)
        _set_log(message)
        return

    if not assignments:
        _set_log(
            "解除対象はありません。\n"
            f"対象: {shape}\n"
            "frameOffset: 変更していません"
        )
        return

    expr = assignments[0]
    body = _expression_body(expr)

    if EXPR_MARKER not in body:
        message = (
            "解除を中止しました。\n"
            "このexpressionは本ツールが管理していません。\n"
            f"expression: {expr}"
        )
        cmds.warning(message)
        _set_log(message)
        return

    remaining = _remove_managed_assignment(body, target_attr)

    try:
        if remaining:
            cmds.expression(expr, edit=True, string=remaining)
            action = f"管理対象の行だけ削除: {expr}"
        else:
            cmds.delete(expr)
            action = f"expressionを削除: {expr}"

        if not _incoming_sources(target_attr):
            cmds.setAttr(target_attr, 1)

    except Exception as exc:
        cmds.warning(str(exc))
        _set_log(f"解除処理に失敗しました。\n{exc}")
        return

    _set_log(
        "解除完了\n"
        f"対象: {shape}\n"
        f"{action}\n"
        "frameExtension: 固定値 1\n"
        "frameOffset: 変更していません"
    )


def show_status(*_args) -> None:
    shape = _ui_selected_shape()

    if not shape:
        cmds.warning("Image Planeを選択してください。")
        _set_log("状態確認失敗: Image Planeが選択されていません。")
        return

    try:
        hold_frames, start_frame, first_image = _read_ui_values()
    except ValueError as exc:
        cmds.warning(str(exc))
        _set_log(f"状態確認失敗: {exc}")
        return

    target_attr = f"{shape}.frameExtension"
    use_sequence_attr = f"{shape}.useFrameExtension"
    offset_attr = f"{shape}.frameOffset"

    assignments = _expressions_assigning(target_attr)
    incoming = _incoming_sources(target_attr)

    try:
        use_sequence = cmds.getAttr(use_sequence_attr)
    except Exception:
        use_sequence = "<取得失敗>"

    try:
        frame_offset = cmds.getAttr(offset_attr)
    except Exception:
        frame_offset = "<取得失敗>"

    preview_lines = []
    for current_frame in range(start_frame, start_frame + 6):
        image_number = (
            math.floor((current_frame - start_frame) / hold_frames)
            + first_image
        )
        preview_lines.append(
            f"frame {current_frame:>4} -> image {image_number}"
        )

    assignment_text = _build_assignment(
        shape,
        hold_frames,
        start_frame,
        first_image,
    )

    report = (
        "=== 状態確認 ===\n"
        f"対象: {shape}\n"
        f"useFrameExtension: {use_sequence}\n"
        f"frameOffset: {frame_offset}  ※変更対象外\n"
        f"incoming sources: {incoming or 'なし'}\n"
        f"expressions: {assignments or 'なし'}\n"
        f"\n予定式:\n{assignment_text}\n"
        "\nプレビュー:\n"
        + "\n".join(preview_lines)
    )

    _set_log(report)


def show() -> None:
    if cmds.window(WINDOW_NAME, exists=True):
        cmds.deleteUI(WINDOW_NAME)

    window = cmds.window(
        WINDOW_NAME,
        title=WINDOW_TITLE,
        sizeable=False,
        widthHeight=(460, 390),
    )

    cmds.columnLayout(
        adjustableColumn=True,
        rowSpacing=7,
        columnOffset=("both", 10),
    )

    cmds.text(
        label="Image Planeの連番画像へコマ打ちを設定します。",
        align="left",
    )
    cmds.text(
        label="frameOffsetは変更しません。",
        align="left",
    )

    cmds.separator(height=6, style="in")

    cmds.rowLayout(
        numberOfColumns=2,
        adjustableColumn=1,
        columnAttach=[(1, "both", 0), (2, "both", 4)],
    )
    _UI["shape_menu"] = cmds.optionMenu(label="対象 Image Plane")
    cmds.button(label="更新", width=70, command=_refresh_shape_menu)
    cmds.setParent("..")

    cmds.separator(height=4, style="none")

    cmds.rowColumnLayout(
        numberOfColumns=2,
        columnWidth=[(1, 180), (2, 110)],
        columnSpacing=[(1, 4), (2, 4)],
    )

    cmds.text(label="1枚の保持フレーム数", align="left")
    _UI["hold_frames"] = cmds.intField(value=DEFAULT_HOLD_FRAMES)

    cmds.text(label="開始フレーム", align="left")
    _UI["start_frame"] = cmds.intField(value=DEFAULT_START_FRAME)

    cmds.text(label="最初の画像番号", align="left")
    _UI["first_image"] = cmds.intField(value=DEFAULT_FIRST_IMAGE)

    cmds.setParent("..")

    cmds.separator(height=8, style="in")

    cmds.rowLayout(
        numberOfColumns=3,
        adjustableColumn=1,
        columnWidth3=(145, 145, 145),
        columnAttach=[(1, "both", 2), (2, "both", 2), (3, "both", 2)],
    )
    cmds.button(label="適用", command=apply_hold)
    cmds.button(label="解除", command=reset_hold)
    cmds.button(label="状態確認", command=show_status)
    cmds.setParent("..")

    cmds.separator(height=8, style="in")

    _UI["log"] = cmds.scrollField(
        editable=False,
        wordWrap=False,
        height=190,
        text="",
    )

    cmds.showWindow(window)
    _refresh_shape_menu()


if __name__ == "__main__":
    show()
