"""统一管理变换控制柄的输入、叠加渲染、事件和撤销历史。"""

from __future__ import annotations

from collections.abc import Callable

from direct.showbase.DirectObject import DirectObject
from direct.showbase.ShowBase import ShowBase
from direct.task.Task import Task
from panda3d.core import Camera, NodePath

from ._utils import camera_path
from .events import GizmoEvent, TransformGizmoMode
from .move_gizmo import MoveGizmo
from .rotate_gizmo import RotateGizmo
from .scale_gizmo import ScaleGizmo
from .types import EventHandler, GroupedEventHooks, MouseInput, TransformAction

SubGizmo = MoveGizmo | RotateGizmo | ScaleGizmo


class TransformGizmo(DirectObject):
    """独立的 Panda3D 变换插件，不依赖宿主工程的 Qt/ImGui/全局状态。

    input_blocked 用于判断宿主 UI 是否占用输入；hotkeys 可关闭内置快捷键。
    overlay_sort 默认为 10，位于默认 3D 场景之后、二维界面之前。
    """

    def __init__(
        self,
        world: ShowBase,
        camera_np: NodePath | None = None,
        on_action_handler_changed: Callable[[str], None] | None = None,
        event_hooks: GroupedEventHooks | None = None,
        *,
        input_blocked: Callable[[], bool] | None = None,
        hotkeys: bool = True,
        overlay_sort: int = 10,
        history_limit: int = 100,
    ) -> None:
        super().__init__()
        self.world = world
        self.camera = camera_path(world, camera_np)
        if self.camera.isEmpty() or not isinstance(self.camera.node(), Camera):
            raise ValueError("camera_np 必须是 Panda3D Camera 节点")
        if world.win is None:
            raise RuntimeError("请先创建 Panda3D 窗口，再创建 TransformGizmo")
        if history_limit < 1:
            raise ValueError("history_limit 必须大于 0")
        self.on_action_handler_changed = on_action_handler_changed
        self.input_blocked = input_blocked or (lambda: False)
        self.history_limit = history_limit
        self.enabled = True
        self._destroyed = False
        self._rmb_down = False
        self._history: list[TransformAction] = []
        self._redo_history: list[TransformAction] = []
        self.target_node: NodePath | None = None
        self._mode = TransformGizmoMode.MOVE
        self._active: SubGizmo | None = None
        self._handle_enabled: dict[str, bool] = dict.fromkeys(("move", "rotate", "scale"), True)
        self._task_name = f"transform-gizmo-{id(self)}"

        # 独立场景使用单位变换；控制柄显式按 render 世界坐标定位。
        # 共享实际 Camera 的镜头，兼容外部相机和正交镜头。
        self.gizmo_render = NodePath("gizmo-overlay")
        camera = Camera("gizmo-overlay-camera")
        camera.setScene(self.gizmo_render)
        camera.setLens(self.camera.node().getLens())
        self.overlay_cam_np = camera_path(world, self.gizmo_render.attachNewNode(camera))
        self.overlay_dr = world.win.makeDisplayRegion()
        self.overlay_dr.setSort(overlay_sort)
        self.overlay_dr.setClearColorActive(False)
        self.overlay_dr.setClearDepthActive(True)
        self.overlay_dr.setCamera(self.overlay_cam_np)
        self.overlay_cam_np.setTransform(self.camera.getTransform(world.render))

        hooks = event_hooks or {}
        self.move_gizmo = MoveGizmo(
            world, self.overlay_cam_np, self._on_subgizmo_action, hooks.get("move")
        )
        self.rotate_gizmo = RotateGizmo(
            world, self.overlay_cam_np, self._on_subgizmo_action, hooks.get("rotate")
        )
        self.scale_gizmo = ScaleGizmo(
            world, self.overlay_cam_np, self._on_subgizmo_action, hooks.get("scale")
        )
        self._gizmos: dict[str, SubGizmo] = {
            "move": self.move_gizmo,
            "rotate": self.rotate_gizmo,
            "scale": self.scale_gizmo,
        }
        for gizmo in self._gizmos.values():
            gizmo.root.reparentTo(self.gizmo_render)
            gizmo.managed_input = True

        self.accept("mouse1", self._mouse_down)
        self.accept("mouse1-up", self._mouse_up)
        self.accept("mouse-move", self._mouse_move)
        # Panda3D 的 mouse2 是中键，mouse3 才是右键。
        self.accept("mouse3", self._right_down)
        self.accept("mouse3-up", self._right_up)
        if hotkeys:
            for key, mode in (("w", "move"), ("e", "rotate"), ("r", "scale")):
                self.accept(key, self._hotkey, [mode])
            self.accept("control-z", self._history_hotkey, [False])
            self.accept("control-y", self._history_hotkey, [True])
        world.taskMgr.add(self._update_task, self._task_name, sort=40)

    @property
    def is_hovering(self) -> bool:
        return any(g.attached and g.is_hovering for g in self._gizmos.values())

    @property
    def is_dragging(self) -> bool:
        return self._active is not None and self._active.dragging

    @property
    def history_size(self) -> int:
        return len(self._history)

    @property
    def is_local(self) -> bool:
        return self.move_gizmo.is_local

    def set_local(self, local: bool = True) -> None:
        """切换移动/旋转的局部或世界坐标；缩放始终沿目标自身轴。"""
        self._mouse_up()
        self.move_gizmo.is_local = bool(local)
        self.rotate_gizmo.is_local = bool(local)
        self.update()

    def attach(self, node_path: NodePath) -> None:
        if self._destroyed:
            raise RuntimeError("已销毁的 TransformGizmo 不能重新使用")
        if not isinstance(node_path, NodePath) or node_path.isEmpty():
            raise ValueError("目标必须是有效的 NodePath")
        self._mouse_up()
        self.target_node = node_path
        self._apply_mode_to_subgizmos()

    def detach(self) -> None:
        self._mouse_up()
        self.target_node = None
        for gizmo in self._gizmos.values():
            gizmo.detach()

    def set_enabled(self, enabled: bool) -> None:
        self._mouse_up()
        self.enabled = bool(enabled)
        self._apply_mode_to_subgizmos()

    def set_mode(self, mode: str) -> None:
        mode = mode.lower()
        if mode not in ("none", "move", "rotate", "scale", "all"):
            raise ValueError(f"不支持的控制柄模式：{mode}")
        if mode == self._mode:
            return
        self._mouse_up()
        self._mode = mode
        self._apply_mode_to_subgizmos()
        if self.on_action_handler_changed:
            self.on_action_handler_changed(mode)

    def get_mode(self) -> str:
        return self._mode

    def set_handles_enabled(
        self, *, move: bool | None = None, rotate: bool | None = None, scale: bool | None = None
    ) -> None:
        """None 保留原值，避免修改一个开关时意外关闭其他控制柄。"""
        self._mouse_up()
        for name, value in (("move", move), ("rotate", rotate), ("scale", scale)):
            if value is not None:
                self._handle_enabled[name] = bool(value)
        self._apply_mode_to_subgizmos()

    def get_handles_enabled(self) -> dict[str, bool]:
        return dict(self._handle_enabled)

    def set_debug(self, enabled: bool) -> None:
        for gizmo in self._gizmos.values():
            gizmo.debug = bool(enabled)

    def add_event_hook(self, mode: str, event: str, callback: EventHandler) -> None:
        """运行时注册回调，无需访问子控制柄的私有列表。"""
        if mode not in self._gizmos or event not in GizmoEvent.ALL:
            raise ValueError("请使用 move/rotate/scale 和 GizmoEvent 常量")
        self._gizmos[mode]._event_hooks[event].append(callback)

    def remove_event_hook(self, mode: str, event: str, callback: EventHandler) -> None:
        self._gizmos[mode]._event_hooks[event].remove(callback)

    def _apply_mode_to_subgizmos(self) -> None:
        for name, gizmo in self._gizmos.items():
            gizmo.detach()
            if (
                self.target_node is not None
                and not self.target_node.isEmpty()
                and self.enabled
                and self._handle_enabled[name]
                and self._mode in (name, "all")
            ):
                gizmo.attach(self.target_node)
                # 子控制柄独立使用时监听输入；组合使用时由本类统一分派。
                gizmo._ignore_events()
        self.update()

    def _mouse_down(self, extra: MouseInput | None = None) -> None:
        if not self.enabled or self.input_blocked() or self._rmb_down or self.is_dragging:
            return
        self.update()
        candidates: list[tuple[float, SubGizmo]] = []
        for gizmo in self._gizmos.values():
            if not gizmo.attached:
                continue
            mouse = gizmo._get_normalized_mouse(extra)
            if mouse is None:
                continue
            gizmo._update_hover_highlight(mouse)
            if gizmo.c_queue.getNumEntries():
                gizmo.c_queue.sortEntries()
                entry = gizmo.c_queue.getEntry(0)
                distance = entry.getSurfacePoint(self.overlay_cam_np).lengthSquared()
                candidates.append((distance, gizmo))
        # ALL 模式先选最近的类型，再由该控制柄处理轴/平面优先级。
        if candidates:
            gizmo = min(candidates, key=lambda item: item[0])[1]
            gizmo._on_mouse_down(extra)
            if gizmo.dragging:
                self._active = gizmo

    def _mouse_up(self, extra: MouseInput | None = None) -> None:
        if self._active is not None:
            if self._active.target_node is not None and not self._active.target_node.isEmpty():
                self._active._on_mouse_move(extra)
                self._active._on_mouse_up(extra)
            self._active = None

    def _mouse_move(self, extra: MouseInput | None = None) -> None:
        """支持宿主 UI 主动传入鼠标坐标；普通窗口在任务中自动更新。"""
        if self.input_blocked() or self._rmb_down or not self.enabled:
            return
        if self._active:
            self._active._on_mouse_move(extra)
        else:
            for gizmo in self._gizmos.values():
                if gizmo.attached:
                    gizmo._on_mouse_move(extra)

    def _right_down(self) -> None:
        self._mouse_up()
        self._rmb_down = True

    def _right_up(self) -> None:
        self._rmb_down = False

    def _hotkey(self, mode: str) -> None:
        if self.enabled and not self._rmb_down and not self.input_blocked():
            self.set_mode(mode)

    def _history_hotkey(self, redo: bool) -> None:
        if not self.input_blocked() and not self._rmb_down:
            (self.redo_last if redo else self.undo_last)()

    def _on_subgizmo_action(self, action: TransformAction) -> None:
        self._history.append(action)
        del self._history[: -self.history_limit]
        self._redo_history.clear()

    def _restore(
        self, source: list[TransformAction], destination: list[TransformAction], prefix: str
    ) -> bool:
        self._mouse_up()
        while source:
            action = source.pop()
            node = action["node"]
            if node.isEmpty():
                continue
            if action["kind"] == "move":
                node.setPos(
                    self.world.render, action["old_pos"] if prefix == "old" else action["new_pos"]
                )
            elif action["kind"] == "rotate":
                node.setHpr(
                    self.world.render, action["old_hpr"] if prefix == "old" else action["new_hpr"]
                )
            else:
                node.setScale(action["old_scale"] if prefix == "old" else action["new_scale"])
            destination.append(action)
            self.update()
            return True
        return False

    def undo_last(self) -> bool:
        return self._restore(self._history, self._redo_history, "old")

    def redo_last(self) -> bool:
        return self._restore(self._redo_history, self._history, "new")

    def clear_history(self) -> None:
        self._history.clear()
        self._redo_history.clear()
        for gizmo in self._gizmos.values():
            for name in ("_move_undo_stack", "_rotate_undo_stack", "_scale_undo_stack"):
                if hasattr(gizmo, name):
                    getattr(gizmo, name).clear()

    def update(self) -> None:
        if self._destroyed:
            return
        if self.target_node is not None and self.target_node.isEmpty():
            self._active = None
            self.detach()
        self.overlay_cam_np.setTransform(self.camera.getTransform(self.world.render))
        self.overlay_cam_np.node().setLens(self.camera.node().getLens())
        for gizmo in self._gizmos.values():
            if gizmo.attached:
                gizmo.update()
        if self.input_blocked() or self._rmb_down:
            for gizmo in self._gizmos.values():
                gizmo.is_hovering = False
            return
        self._mouse_move()

    def _update_task(self, task: Task) -> int:
        self.update()
        return task.cont

    def destroy(self) -> None:
        """释放事件、任务、射线、显示区域和历史；允许重复调用。"""
        if self._destroyed:
            return
        self.detach()
        self.ignoreAll()
        self.world.taskMgr.remove(self._task_name)
        for gizmo in self._gizmos.values():
            gizmo.destroy()
        self.world.win.removeDisplayRegion(self.overlay_dr)
        self.gizmo_render.removeNode()
        self.clear_history()
        self._destroyed = True


__all__ = ["TransformGizmo", "TransformGizmoMode"]
