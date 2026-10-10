"""公开事件及内部数据结构的类型定义，便于 IDE 补全和静态检查。"""

from __future__ import annotations

from collections.abc import Callable
from typing import Literal, TypeAlias, TypedDict

from panda3d.core import LVecBase3f, NodePath, Point2, Quat, Vec3


class MouseInput(TypedDict, total=False):
    """外部输入支持标准化坐标，或左上角为原点的像素坐标。"""

    ndc: tuple[float, float]
    x: float
    y: float
    width: float
    height: float


class _EventTarget(TypedDict):
    gizmo: str
    target: NodePath


class DragEvent(_EventTarget, total=False):
    """按模式和拖拽阶段携带不同字段；可选字段通过 get() 读取。"""

    axis: int | None
    plane: int | None
    mode: str | None
    mouse: Point2
    start_pos: LVecBase3f | None
    new_pos: LVecBase3f
    final_pos: LVecBase3f | None
    start_quat: Quat | None
    final_hpr: LVecBase3f | None
    center: LVecBase3f
    axis_world: Vec3
    delta_deg: float
    start_scale: LVecBase3f | None
    new_scale: LVecBase3f
    final_scale: LVecBase3f | None
    scale_factor: float


class MoveAction(TypedDict):
    kind: Literal["move"]
    node: NodePath
    old_pos: Vec3
    new_pos: Vec3


class RotateAction(TypedDict):
    kind: Literal["rotate"]
    node: NodePath
    old_hpr: Vec3
    new_hpr: Vec3


class ScaleAction(TypedDict):
    kind: Literal["scale"]
    node: NodePath
    old_scale: Vec3
    new_scale: Vec3


class AxisVisual(TypedDict):
    """缩放轴的节点与基准尺寸，避免混用节点类型和浮点类型。"""

    line: NodePath
    cube: NodePath
    length: float
    cube_size: float


TransformAction: TypeAlias = MoveAction | RotateAction | ScaleAction
ActionHandler: TypeAlias = Callable[[TransformAction], None]
EventHandler: TypeAlias = Callable[[DragEvent], None]
EventHooks: TypeAlias = dict[str, list[EventHandler]]
GroupedEventHooks: TypeAlias = dict[str, EventHooks]
