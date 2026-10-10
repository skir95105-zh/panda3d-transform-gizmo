"""插件共享的模式及拖拽事件常量。"""


class GizmoEvent:
    """String constants for gizmo event hook names."""

    DRAG_START = "drag_start"
    DRAG_MOVE = "drag_move"
    DRAG_END = "drag_end"

    ALL = (DRAG_START, DRAG_MOVE, DRAG_END)


class TransformGizmoMode:
    """Simple string constants for gizmo modes."""

    NONE = "none"
    MOVE = "move"
    ROTATE = "rotate"
    SCALE = "scale"
    ALL = "all"  # 同时显示移动、旋转和缩放控制柄


__all__ = ["GizmoEvent", "TransformGizmoMode"]
