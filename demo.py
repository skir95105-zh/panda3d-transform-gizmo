"""TransformGizmo 完整演示：选择物体、变换、镜头导航及事件实时反馈。

运行：.venv/Scripts/python.exe demo.py
界面使用英文以兼容 Panda3D 默认字体；实现和接入说明采用中文注释。
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Literal

from direct.showbase.ShowBase import ShowBase
from direct.task.Task import Task, cont, done
from panda3d.core import (
    AmbientLight,
    BitMask32,
    CardMaker,
    CollisionBox,
    CollisionHandlerQueue,
    CollisionNode,
    CollisionRay,
    CollisionTraverser,
    DirectionalLight,
    Filename,
    Geom,
    GeomNode,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    LineSegs,
    NodePath,
    Point2,
    Point3,
    TransformState,
    Vec3,
    loadPrcFileData,
)

from demo_ui import Color, DemoUI
from TransformGizmo import GizmoEvent, TransformGizmo
from TransformGizmo.types import DragEvent, EventHandler, GroupedEventHooks, MouseInput


def make_box(name: str, color: Color) -> GeomNode:
    """程序化生成带法线的立方体，无需下载模型或依赖绝对资源路径。"""
    data = GeomVertexData(name, GeomVertexFormat.getV3n3c4(), Geom.UHStatic)
    vertices = GeomVertexWriter(data, "vertex")
    normals = GeomVertexWriter(data, "normal")
    colors = GeomVertexWriter(data, "color")
    triangles = GeomTriangles(Geom.UHStatic)
    faces = [
        ((0, 0, 1), [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]),
        ((0, 0, -1), [(-1, 1, -1), (1, 1, -1), (1, -1, -1), (-1, -1, -1)]),
        ((1, 0, 0), [(1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)]),
        ((-1, 0, 0), [(-1, 1, -1), (-1, -1, -1), (-1, -1, 1), (-1, 1, 1)]),
        ((0, 1, 0), [(1, 1, -1), (-1, 1, -1), (-1, 1, 1), (1, 1, 1)]),
        ((0, -1, 0), [(-1, -1, -1), (1, -1, -1), (1, -1, 1), (-1, -1, 1)]),
    ]
    for index, (normal, corners) in enumerate(faces):
        for corner in corners:
            vertices.addData3(*(value * 0.5 for value in corner))
            normals.addData3(*normal)
            colors.addData4(*color)
        first = index * 4
        triangles.addVertices(first, first + 1, first + 2)
        triangles.addVertices(first, first + 2, first + 3)
    geometry = Geom(data)
    geometry.addPrimitive(triangles)
    node = GeomNode(name)
    node.addGeom(geometry)
    return node


class GizmoDemo(ShowBase):
    """独立 ShowBase 示例，插件与演示的选择/镜头逻辑互相分离。"""

    def __init__(
        self, *, offscreen: bool = False, frames: int = 0, screenshot: str | None = None
    ) -> None:
        super().__init__(windowType="offscreen" if offscreen else None)
        self.disableMouse()
        self.setBackgroundColor(0.075, 0.09, 0.12)
        self.camLens.setFov(48)
        self.camLens.setNearFar(0.1, 1000)
        self.orbit_center = Vec3(0, 0, 0.8)
        self.yaw, self.pitch, self.distance = 38.0, 27.0, 19.0
        self.navigation: Literal["orbit", "pan"] | None = None
        self.last_mouse: Point2 | None = None
        self.selected: NodePath | None = None
        self.last_event = "Ready - drag a colored handle"
        self.event_count = 0
        self.debug_enabled = False
        self._frame_count = 0
        self._max_frames = frames
        self._screenshot = screenshot
        self._build_scene()
        self._build_picker()
        self._build_ui()
        hooks: GroupedEventHooks = {
            mode: {event: [self._event_handler(event)] for event in GizmoEvent.ALL}
            for mode in ("move", "rotate", "scale")
        }
        self.gizmo = TransformGizmo(self, event_hooks=hooks, input_blocked=self._ui_has_mouse)
        self._bind_controls()
        self._update_camera()
        # 启动时默认绑定第一个物体，并显示移动控制柄。
        self.select_object(self.objects[0])
        self.taskMgr.add(self._update, "demo-update", sort=50)

    def _build_scene(self) -> None:
        ambient = AmbientLight("ambient")
        ambient.setColor((0.52, 0.55, 0.62, 1))
        self.render.setLight(self.render.attachNewNode(ambient))
        sun = DirectionalLight("sun")
        sun.setColor((0.8, 0.78, 0.73, 1))
        light = self.render.attachNewNode(sun)
        light.setHpr(-30, -55, 0)
        self.render.setLight(light)
        card = CardMaker("floor")
        card.setFrame(-12, 12, -12, 12)
        floor = self.render.attachNewNode(card.generate())
        floor.setP(-90)
        floor.setZ(-0.03)
        floor.setColor(0.115, 0.14, 0.18, 1)
        floor.setLightOff()
        grid = LineSegs("grid")
        grid.setThickness(1)
        for i in range(-12, 13):
            grid.setColor(0.22, 0.27, 0.32, 1)
            grid.moveTo(i, -12, 0)
            grid.drawTo(i, 12, 0)
            grid.moveTo(-12, i, 0)
            grid.drawTo(12, i, 0)
        for color, endpoint in [
            ((0.7, 0.23, 0.23, 1), (5, 0, 0.01)),
            ((0.23, 0.7, 0.35, 1), (0, 5, 0.01)),
        ]:
            grid.setColor(*color)
            grid.moveTo(0, 0, 0.01)
            grid.drawTo(*endpoint)
        self.render.attachNewNode(grid.create()).setLightOff()

        # 第三个目标放在带旋转的父节点下，用于观察父子坐标关系。
        parent = self.render.attachNewNode("rotated-parent")
        parent.setPos(3, 1, 0)
        parent.setH(25)
        descriptions: list[
            tuple[
                str,
                NodePath,
                tuple[float, float, float],
                tuple[float, float, float],
                tuple[float, float, float],
                Color,
            ]
        ] = [
            (
                "Amber Cube",
                self.render,
                (0, 0, 0.8),
                (0, 0, 0),
                (1.6, 1.6, 1.6),
                (0.96, 0.56, 0.2, 1),
            ),
            ("Teal Block", self.render, (-3, 1, 1), (30, 0, 0), (1.4, 1.4, 2), (0.2, 0.76, 0.7, 1)),
            (
                "Nested Violet",
                parent,
                (0, 0, 0.7),
                (15, 0, 10),
                (1.5, 1.2, 1.4),
                (0.59, 0.46, 0.94, 1),
            ),
        ]
        self.objects: list[NodePath] = []
        self.initial_transforms: list[TransformState] = []
        for index, (name, owner, position, hpr, scale, color) in enumerate(descriptions):
            target = owner.attachNewNode(name)
            target.setPos(*position)
            target.setHpr(*hpr)
            target.setScale(*scale)
            visual = target.attachNewNode(make_box(name, color))
            # 几何关闭默认拾取，使用与 Gizmo 分离的 bit(1) 碰撞盒。
            visual.setCollideMask(BitMask32.allOff())
            collision = CollisionNode(f"pick-{index}")
            collision.addSolid(CollisionBox(Point3(0), 0.5, 0.5, 0.5))
            collision.setFromCollideMask(BitMask32.allOff())
            collision.setIntoCollideMask(BitMask32.bit(1))
            path = target.attachNewNode(collision)
            path.setTag("demo-object", str(index))
            self.objects.append(target)
            self.initial_transforms.append(target.getTransform())

    def _build_picker(self) -> None:
        self.pick_ray = CollisionRay()
        node = CollisionNode("scene-picker")
        node.addSolid(self.pick_ray)
        node.setFromCollideMask(BitMask32.bit(1))
        node.setIntoCollideMask(BitMask32.allOff())
        self.pick_path = self.cam.attachNewNode(node)
        self.pick_queue = CollisionHandlerQueue()
        self.pick_traverser = CollisionTraverser()
        self.pick_traverser.addCollider(self.pick_path, self.pick_queue)

    def _build_ui(self) -> None:
        self.ui: DemoUI = DemoUI(
            self,
            [node.getName() for node in self.objects],
            on_mode=lambda mode: self.gizmo.set_mode(mode),
            on_select=lambda index: self.select_object(self.objects[index]),
        )

    def _ui_has_mouse(self) -> bool:
        if self.mouseWatcherNode is None or not self.mouseWatcherNode.hasMouse():
            return False
        mouse = self.mouseWatcherNode.getMouse()
        return self.ui.contains_mouse(
            (mouse.x + 1) * self.win.getXSize() / 2,
            (1 - mouse.y) * self.win.getYSize() / 2,
        )

    def _bind_controls(self) -> None:
        for key, mode in (("a", "all"), ("q", "none")):
            self.accept(key, self.gizmo.set_mode, [mode])
        for index in range(3):
            self.accept(str(index + 1), self.select_object, [self.objects[index]])
        for key, name in (("7", "move"), ("8", "rotate"), ("9", "scale")):
            self.accept(key, self._toggle_handle, [name])
        self.accept("l", lambda: self.gizmo.set_local(not self.gizmo.is_local))
        self.accept("g", lambda: self.gizmo.set_enabled(not self.gizmo.enabled))
        self.accept("d", self._toggle_debug)
        self.accept("tab", self._next_object)
        self.accept("escape", self.select_object, [None])
        self.accept("backspace", self._reset_scene)
        self.accept("f", self._focus)
        self.accept("home", self._reset_camera)
        self.accept("mouse1", self._schedule_pick)
        self.accept("mouse3", self._start_navigation, ["orbit"])
        self.accept("mouse2", self._start_navigation, ["pan"])
        self.accept("mouse3-up", self._stop_navigation)
        self.accept("mouse2-up", self._stop_navigation)
        self.accept("wheel_up", self._zoom, [0.88])
        self.accept("wheel_down", self._zoom, [1 / 0.88])

    def _schedule_pick(self, extra: MouseInput | None = None) -> None:
        # 将选择延后一帧，使插件先判断是否点中了控制柄。
        self.taskMgr.remove("demo-pick")
        self.taskMgr.doMethodLater(0, self._pick, "demo-pick")

    def _pick(self, task: Task) -> int:
        if (
            self._ui_has_mouse()
            or self.gizmo.is_dragging
            or self.gizmo.is_hovering
            or self.mouseWatcherNode is None
            or not self.mouseWatcherNode.hasMouse()
        ):
            return done
        mouse = self.mouseWatcherNode.getMouse()
        self.pick_ray.setFromLens(self.cam.node(), mouse.x, mouse.y)
        self.pick_queue.clearEntries()
        self.pick_traverser.traverse(self.render)
        self.pick_queue.sortEntries()
        if self.pick_queue.getNumEntries():
            path = self.pick_queue.getEntry(0).getIntoNodePath()
            self.select_object(self.objects[int(path.getTag("demo-object"))])
        else:
            self.select_object(None)
        return done

    def select_object(self, target: NodePath | None) -> None:
        self.selected = target
        if target is None:
            self.gizmo.detach()
        else:
            self.gizmo.attach(target)

    def _next_object(self) -> None:
        current = self.objects.index(self.selected) if self.selected is not None else -1
        self.select_object(self.objects[(current + 1) % len(self.objects)])

    def _toggle_handle(self, name: str) -> None:
        self.gizmo.set_handles_enabled(**{name: not self.gizmo.get_handles_enabled()[name]})

    def _toggle_debug(self) -> None:
        self.debug_enabled = not self.debug_enabled
        self.gizmo.set_debug(self.debug_enabled)

    def _reset_scene(self) -> None:
        self.gizmo.detach()
        for target, transform in zip(self.objects, self.initial_transforms):
            target.setTransform(transform)
        self.gizmo.clear_history()
        self.select_object(self.selected or self.objects[0])
        self.last_event = "Scene reset; transform history cleared"

    def _event_handler(self, event: str) -> EventHandler:
        def callback(info: DragEvent) -> None:
            self.event_count += 1
            axis = info.get("axis")
            handle = "XYZU"[axis] if axis is not None else info.get("mode", "plane")
            plane = info.get("plane")
            if plane is not None:
                handle = ("XY", "YZ", "ZX")[plane]
            self.last_event = f"{info['gizmo']} / {event} / {handle}"
            if "delta_deg" in info:
                self.last_event += f" / {info['delta_deg']:.1f} deg"
            if "scale_factor" in info:
                self.last_event += f" / x{info['scale_factor']:.2f}"

        return callback

    def _start_navigation(self, kind: Literal["orbit", "pan"]) -> None:
        if not self._ui_has_mouse() and not self.gizmo.is_dragging:
            self.navigation = kind
            self.last_mouse = None

    def _stop_navigation(self) -> None:
        self.navigation = None
        self.last_mouse = None

    def _zoom(self, factor: float) -> None:
        if not self._ui_has_mouse() and not self.gizmo.is_dragging:
            self.distance = max(2, min(80, self.distance * factor))
            self._update_camera()

    def _focus(self) -> None:
        if self.selected is not None:
            self.orbit_center = Vec3(self.selected.getPos(self.render))
            self.distance = max(5, self.selected.getScale(self.render).length() * 3)
            self._update_camera()

    def _reset_camera(self) -> None:
        self.orbit_center = Vec3(0, 0, 0.8)
        self.yaw, self.pitch, self.distance = 38, 27, 19
        self._update_camera()

    def _update_camera(self) -> None:
        yaw, pitch = math.radians(self.yaw), math.radians(self.pitch)
        offset = Vec3(
            math.sin(yaw) * math.cos(pitch), -math.cos(yaw) * math.cos(pitch), math.sin(pitch)
        )
        self.camera.setPos(self.orbit_center + offset * self.distance)
        self.camera.lookAt(self.orbit_center)

    def _update(self, task: Task) -> int:
        if (
            self.navigation
            and self.mouseWatcherNode is not None
            and self.mouseWatcherNode.hasMouse()
        ):
            mouse = self.mouseWatcherNode.getMouse()
            if self.last_mouse is not None:
                delta = mouse - self.last_mouse
                if self.navigation == "orbit":
                    self.yaw -= delta.x * 120
                    self.pitch = max(-80, min(80, self.pitch + delta.y * 90))
                else:
                    quat = self.camera.getQuat(self.render)
                    self.orbit_center -= (
                        (quat.getRight() * delta.x * self.getAspectRatio() + quat.getUp() * delta.y)
                        * self.distance
                        * 0.4
                    )
                self._update_camera()
            self.last_mouse = Point2(mouse)
        elif self.navigation:
            self.last_mouse = None

        target = self.selected
        selected_index = self.objects.index(target) if target is not None else None
        self.ui.update(
            selection=selected_index,
            name=target.getName() if target is not None else "No selection",
            mode=self.gizmo.get_mode(),
            local=self.gizmo.is_local,
            enabled=self.gizmo.enabled,
            debug=self.debug_enabled,
            handles=self.gizmo.get_handles_enabled(),
            history=self.gizmo.history_size,
            position=target.getPos(self.render) if target is not None else None,
            rotation=target.getHpr(self.render) if target is not None else None,
            scale=target.getScale() if target is not None else None,
            event_count=self.event_count,
            event=self.last_event,
        )
        self._frame_count += 1
        if self._max_frames and self._frame_count >= self._max_frames:
            if self._screenshot:
                destination = Path(self._screenshot).resolve()
                destination.parent.mkdir(parents=True, exist_ok=True)
                self.graphicsEngine.renderFrame()
                self.win.saveScreenshot(Filename.fromOsSpecific(str(destination)))
            self.taskMgr.stop()
            return done
        return cont

    def destroy(self) -> None:
        """先释放插件，再销毁窗口，避免遗留显示区域或全局事件。"""
        if hasattr(self, "gizmo"):
            self.gizmo.destroy()
        if hasattr(self, "ui"):
            self.ui.destroy()
        super().destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="Panda3D Transform Gizmo demo")
    parser.add_argument("--offscreen", action="store_true", help="离屏渲染，无交互窗口")
    parser.add_argument("--frames", type=int, default=0, help="渲染指定帧数后退出，0 为持续运行")
    parser.add_argument("--screenshot", help="退出前保存 PNG，需配合 --frames")
    args = parser.parse_args()
    if args.frames < 0 or (args.screenshot and not args.frames):
        parser.error("--frames 必须非负；--screenshot 需要指定大于 0 的 --frames")
    loadPrcFileData(
        "demo",
        "window-title Panda3D Transform Gizmo\nwin-size 1440 900\nsync-video true\naudio-library-name null",
    )
    app = GizmoDemo(offscreen=args.offscreen, frames=args.frames, screenshot=args.screenshot)
    try:
        app.run()
    finally:
        app.destroy()


if __name__ == "__main__":
    main()
