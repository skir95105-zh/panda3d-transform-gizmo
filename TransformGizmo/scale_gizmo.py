"""缩放控制柄：程序化几何、碰撞拾取与拖拽计算。"""

from __future__ import annotations

import math

from direct.showbase.DirectObject import DirectObject
from direct.showbase.ShowBase import ShowBase
from direct.task.Task import Task
from panda3d.core import (
    BitMask32,
    CollisionBox,
    CollisionHandlerQueue,
    CollisionNode,
    CollisionRay,
    CollisionTraverser,
    CollisionTube,
    Geom,
    GeomLines,
    GeomNode,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    Lens,
    LPoint2f,
    LVecBase3f,
    Material,
    NodePath,
    Point2,
    Point3,
    Quat,
    TransparencyAttrib,
    Vec3,
    Vec4,
)

from ._utils import camera_path, destroy_gizmo, normalized_mouse, screen_scale
from .events import GizmoEvent
from .types import ActionHandler, AxisVisual, DragEvent, EventHooks, MouseInput

"""
缩放控制柄（Scale Gizmo）
=========================

功能概述：
    - 模仿 Unity 的缩放控制柄，使用 3 个彩色轴线表示沿 X/Y/Z 轴缩放
      - X 轴：红色线条 + 方块（沿局部 X 轴缩放）
      - Y 轴：绿色线条 + 方块（沿局部 Y 轴缩放）
      - Z 轴：蓝色线条 + 方块（沿局部 Z 轴缩放）
    - 中心有一个白色小方块，用于均匀缩放（沿所有轴同时缩放）
    - 使用 Panda3D 的碰撞系统进行鼠标拾取与拖拽
    - 控件会根据摄像机距离自动缩放，屏幕大小保持近似不变

集成方式（示例）：
    from direct.showbase.ShowBase import ShowBase
    from TransformGizmo.scale_gizmo import ScaleGizmo

    world = ShowBase()
    model_np = world.render.attachNewNode("Box")
    # ... 在 model_np 下加载模型

    gizmo = ScaleGizmo(world)          # 创建缩放 Gizmo
    gizmo.attach(model_np)             # 绑定到某个模型

    # 当需要切换目标时：
    gizmo.attach(another_np)

    # 当不需要时，可隐藏：
    gizmo.detach()

鼠标事件要求：
    - 本类默认监听 Panda3D 的 "mouse1" / "mouse1-up" / "mouse-move" 事件
    - 如果从 Qt / 自定义 UI 传递鼠标像素坐标，可以在发送事件时传入 extra 字典：
        messenger.send("mouse1", [{"x": mouse_x, "y": mouse_y}])
      本类会自动将像素坐标转换到 [-1, 1] 的标准化设备坐标
"""


class ScaleGizmo(DirectObject):
    """
    Unity 风格的缩放控制柄：
        - 3 个轴线 + 方块分别代表沿局部 X/Y/Z 轴缩放
        - 中心方块代表均匀缩放
        - 左键点击并拖拽，可以缩放绑定的 NodePath
    """

    def __init__(
        self,
        world: ShowBase,
        camera_np: NodePath | None = None,
        on_action_committed: ActionHandler | None = None,
        event_hooks: EventHooks | None = None,
    ) -> None:
        super().__init__()

        self.world: ShowBase = world
        self.is_local = True
        self.managed_input: bool = False
        self.debug: bool = False
        self.is_hovering = False
        self._picker_added: bool = False
        self._debug_drag_counter: int = 0
        self.on_action_committed = on_action_committed
        # Event hooks: drag_start / drag_move / drag_end
        self._event_hooks = self._normalize_event_hooks(event_hooks)

        self.camera = camera_path(world, camera_np)
        if not self.camera:
            self._log("No camera found for ScaleGizmo")
        else:
            self._log(f"Camera found for ScaleGizmo: {self.camera}")

        self.root: NodePath = NodePath("ScaleGizmo")
        self.color_higher = 0.05
        self.color_normal = 0.01
        self.last_mouse_pos: Point2 | None = None
        # self.root.setBin("fixed", 40)
        # self.root.setDepthTest(False)
        # self.root.setDepthWrite(False)
        # self.root.setLightOff()

        self.target_node: NodePath | None = None
        self.attached: bool = False

        # 控制柄视觉参数
        self.axis_length: float = 0.5
        self.axis_thickness: float = 0.02
        self.cube_size: float = 0.04
        self.center_cube_size: float = 0.06
        self.axis_alpha: float = 0.75
        # 记录基准尺寸，用于拖拽时的可视放缩
        self._axis_length0: float = self.axis_length
        self._cube_size0: float = self.cube_size
        self._center_cube_size0: float = self.center_cube_size

        # 拾取射线与碰撞系统
        self.picker_ray = CollisionRay()
        self.picker_node = CollisionNode("scale_gizmo_picker_ray")
        self.picker_node.addSolid(self.picker_ray)
        self.picker_node.setFromCollideMask(BitMask32.bit(20))
        self.picker_node.setIntoCollideMask(BitMask32.allOff())
        self.picker_np: NodePath = NodePath(self.picker_node)

        self.c_trav = CollisionTraverser()
        self.c_queue = CollisionHandlerQueue()

        # 拖拽状态
        self.dragging: bool = False
        self.drag_axis: int | None = None  # 0=X, 1=Y, 2=Z, 3=Uniform
        self.start_mouse: Point2 | None = None
        self.start_scale: Vec3 | None = None
        self.drag_axis_dir: Vec3 = Vec3(0, 0, 0)
        self.drag_axis_screen: Point2 = Point2(1, 0)
        self.drag_base_dist: float = 1.0
        self._start_param_on_ray: float = 0.0

        # 撤销栈
        self._scale_undo_stack: list[tuple[NodePath, Vec3, Vec3]] = []
        self._undo_scale_epsilon: float = 1e-4

        # 轴和中心控制柄节点
        self.axes: list[NodePath] = []
        self.axes_colors: list[tuple[NodePath, NodePath, Vec4]] = []
        # 可视节点（线与端点立方体），与碰撞体分离，便于单独放缩
        self.axis_visuals: list[AxisVisual] = []
        self.axis_visual_colors: list[tuple[NodePath, Vec4]] = []

        self.center_handle: NodePath | None = None
        self.center_handle_color = Vec4(1, 1, 1, 1)

        self._build_gizmo()

        self.root.reparentTo(self.world.render)
        self.root.setBin("fixed", 40)
        self.root.setDepthTest(False)
        self.root.setDepthWrite(False)
        self.root.setLightOff()
        self.root.hide()

        # taskMgr.add(self._update_task, "ScaleGizmoUpdateTask")

    def _log(self, msg: str) -> None:
        if self.debug:
            print(f"[ScaleGizmo] {msg}")

    def _normalize_event_hooks(self, hooks: EventHooks | None) -> EventHooks:
        base: EventHooks = {name: [] for name in GizmoEvent.ALL}
        if not hooks:
            return base
        for name in list(base.keys()):
            cbs = hooks.get(name)
            if cbs:
                base[name] = list(cbs)
        return base

    def _emit_event(self, name: str, payload: DragEvent) -> None:
        handlers = self._event_hooks.get(name, [])
        for cb in handlers:
            try:
                cb(payload)
            except Exception as exc:
                self._log(f"event hook '{name}' error: {exc}")

    def _build_gizmo(self) -> None:
        """构建三个轴控制柄和中心均匀缩放控制柄。"""
        # X 轴（红）
        axis_root, line_np, cube_np = self._create_axis(
            axis_dir=Vec3(1, 0, 0),
            color=Vec4(1, 0, 0, 1),
            length=self.axis_length,
            thickness=self.axis_thickness,
            cube_size=self.cube_size,
            axis_id=0,
        )
        self.axes.append(axis_root)
        self.axis_visuals.append(
            {
                "line": line_np,
                "cube": cube_np,
                "length": self.axis_length,
                "cube_size": self.cube_size,
            }
        )
        # Y 轴（绿）
        axis_root, line_np, cube_np = self._create_axis(
            axis_dir=Vec3(0, 1, 0),
            color=Vec4(0, 1, 0, 1),
            length=self.axis_length,
            thickness=self.axis_thickness,
            cube_size=self.cube_size,
            axis_id=1,
        )
        self.axes.append(axis_root)
        self.axis_visuals.append(
            {
                "line": line_np,
                "cube": cube_np,
                "length": self.axis_length,
                "cube_size": self.cube_size,
            }
        )
        # Z 轴（蓝）
        axis_root, line_np, cube_np = self._create_axis(
            axis_dir=Vec3(0, 0, 1),
            color=Vec4(0, 0, 1, 1),
            length=self.axis_length,
            thickness=self.axis_thickness,
            cube_size=self.cube_size,
            axis_id=2,
        )
        self.axes.append(axis_root)
        self.axis_visuals.append(
            {
                "line": line_np,
                "cube": cube_np,
                "length": self.axis_length,
                "cube_size": self.cube_size,
            }
        )
        # 中心控制柄（白色方块，用于均匀缩放）
        self.center_handle = self._create_center_handle(
            size=self.center_cube_size,
            color=Vec4(1, 1, 1, 1),
        )

    def _set_node_dept(self, node: NodePath, color: Vec4 | None = None) -> None:
        if node is None or node.is_empty():
            return
        mat = Material("default")
        mat.set_emission((1, 0, 1, 1))
        if color:
            mat.base_color = color * self.color_normal
        node.set_material(mat)
        node.set_bin("fixed", 40)
        node.set_depth_test(False)
        node.set_depth_write(False)

    def _create_axis(
        self,
        axis_dir: Vec3,
        color: Vec4,
        length: float,
        thickness: float,
        cube_size: float,
        axis_id: int,
    ) -> tuple[NodePath, NodePath, NodePath]:
        """创建一个轴控制柄：线条 + 末端方块。"""
        axis_root = self.root.attachNewNode(f"scale_axis_{axis_id}")

        align_quat = self._quat_from_z(axis_dir)
        axis_root.setQuat(align_quat)

        # 几何：线条
        line_geom = self._create_line_geom(length, color)
        line_np: NodePath = axis_root.attachNewNode(line_geom)
        line_np.setTransparency(TransparencyAttrib.MAlpha)
        line_np.setAlphaScale(self.axis_alpha)
        line_np_copy = line_np.copyTo(line_np)
        line_np_copy.set_pos(0, 0, 0)
        line_np_copy.set_hpr(0, 0, 0)

        # 几何：末端方块
        cube_geom = self._create_cube_geom(cube_size, color)
        cube_np: NodePath = axis_root.attachNewNode(cube_geom)
        cube_np.setPos(0, 0, length)
        cube_np.setTransparency(TransparencyAttrib.MAlpha)
        cube_np.setAlphaScale(self.axis_alpha)
        cube_np_copy = cube_np.copy_to(cube_np)
        cube_np_copy.set_pos(0, 0, 0)
        cube_np_copy.set_hpr(0, 0, 0)

        # 碰撞体：线条使用 Tube
        c_node = CollisionNode(f"scale_axis_col_{axis_id}")
        tube = CollisionTube(Point3(0, 0, 0), Point3(0, 0, length), thickness * 2)
        c_node.addSolid(tube)
        c_node.setIntoCollideMask(BitMask32.bit(20))
        c_node.setFromCollideMask(BitMask32.allOff())
        c_node.setTag("gizmo_axis", str(axis_id))
        axis_root.attachNewNode(c_node)

        # 碰撞体：末端方块
        c_cube_node = CollisionNode(f"scale_cube_col_{axis_id}")
        half = cube_size * 0.5
        box = CollisionBox(Point3(0, 0, length), half, half, half)
        c_cube_node.addSolid(box)
        c_cube_node.setIntoCollideMask(BitMask32.bit(20))
        c_cube_node.setFromCollideMask(BitMask32.allOff())
        c_cube_node.setTag("gizmo_axis", str(axis_id))
        axis_root.attachNewNode(c_cube_node)

        return axis_root, line_np, cube_np

    def _create_center_handle(self, size: float, color: Vec4) -> NodePath:
        """创建中心均匀缩放控制柄（方块）。"""
        center_root = self.root.attachNewNode("scale_center")

        cube_geom = self._create_cube_geom(size, color)
        cube_np: NodePath = center_root.attachNewNode(cube_geom)
        cube_np.setTransparency(TransparencyAttrib.MAlpha)
        cube_np.setAlphaScale(self.axis_alpha)

        c_node = CollisionNode("scale_center_col")
        half = size * 0.5
        box = CollisionBox(Point3(0, 0, 0), half, half, half)
        c_node.addSolid(box)
        c_node.setIntoCollideMask(BitMask32.bit(20))
        c_node.setFromCollideMask(BitMask32.allOff())
        c_node.setTag("gizmo_center", "1")
        center_root.attachNewNode(c_node)

        return center_root

    def _create_line_geom(self, length: float, color: Vec4) -> GeomNode:
        """创建一条沿 +Z 方向的线段几何。"""
        vdata = GeomVertexData("line", GeomVertexFormat.getV3c4(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, "vertex")
        vcolor = GeomVertexWriter(vdata, "color")

        vertex.addData3(0, 0, 0)
        vcolor.addData4(color)
        vertex.addData3(0, 0, length)
        vcolor.addData4(color)

        prim = GeomLines(Geom.UHStatic)
        prim.addVertices(0, 1)

        geom = Geom(vdata)
        geom.addPrimitive(prim)
        node = GeomNode("line_geom")
        node.addGeom(geom)
        return node

    def _create_cube_geom(self, size: float, color: Vec4) -> GeomNode:
        """创建一个以原点为中心的立方体几何。"""
        vdata = GeomVertexData("cube", GeomVertexFormat.getV3c4(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, "vertex")
        vcolor = GeomVertexWriter(vdata, "color")

        half = size * 0.5

        # 8个顶点
        vertices = [
            Point3(-half, -half, -half),  # 0
            Point3(half, -half, -half),  # 1
            Point3(half, half, -half),  # 2
            Point3(-half, half, -half),  # 3
            Point3(-half, -half, half),  # 4
            Point3(half, -half, half),  # 5
            Point3(half, half, half),  # 6
            Point3(-half, half, half),  # 7
        ]

        for v in vertices:
            vertex.addData3(v)
            vcolor.addData4(color)

        prim = GeomTriangles(Geom.UHStatic)

        # 6个面，每个面2个三角形
        faces = [
            (0, 1, 2, 3),  # 底面 -Z
            (4, 7, 6, 5),  # 顶面 +Z
            (0, 4, 5, 1),  # 前面 -Y
            (2, 6, 7, 3),  # 后面 +Y
            (0, 3, 7, 4),  # 左面 -X
            (1, 5, 6, 2),  # 右面 +X
        ]

        for f in faces:
            prim.addVertices(f[0], f[1], f[2])
            prim.addVertices(f[0], f[2], f[3])

        geom = Geom(vdata)
        geom.addPrimitive(prim)
        node = GeomNode("cube_geom")
        node.addGeom(geom)
        return node

    def _quat_from_z(self, direction: Vec3) -> Quat:
        """返回一个四元数，使本地 +Z 旋转到指定 direction。"""
        dir_norm = Vec3(direction)
        if dir_norm.length_squared() == 0:
            return Quat.identQuat()
        dir_norm.normalize()
        z_axis = Vec3(0, 0, 1)
        dot = z_axis.dot(dir_norm)
        if abs(dot - 1.0) < 1e-6:
            return Quat.identQuat()
        if abs(dot + 1.0) < 1e-6:
            ortho = Vec3(1, 0, 0) if abs(z_axis.dot(Vec3(1, 0, 0))) < 0.9 else Vec3(0, 1, 0)
            axis = z_axis.cross(ortho)
            axis.normalize()
            q = Quat()
            q.setFromAxisAngle(180.0, axis)
            return q
        axis = z_axis.cross(dir_norm)
        axis_len = axis.length()
        if axis_len == 0:
            return Quat.identQuat()
        axis.normalize()
        angle_rad = math.atan2(axis_len, dot)
        q = Quat()
        q.setFromAxisAngle(math.degrees(angle_rad), axis)
        return q

    def attach(self, node_path: NodePath) -> None:
        """绑定控制柄到指定 NodePath。"""
        if not node_path:
            return

        self.target_node = node_path
        self.attached = True
        self.last_mouse_pos = None
        self._update_root_transform()
        self.root.show()
        self._log(
            f"attached to {node_path.getName()} "
            f"world_pos={self.root.getPos(self.world.render)} "
            f"target_scale={node_path.getScale()}"
        )
        self._register_events()

    def detach(self) -> None:
        """解绑并隐藏 Gizmo。"""
        self.target_node = None
        self.root.hide()
        self.attached = False
        self.dragging = False
        self.drag_axis = None
        self.is_hovering = False
        self._reset_axis_visuals()
        self._reset_highlights()
        self._ignore_events()
        self._log("detached")

    def _record_scale_action(
        self, node: NodePath, old_scale: LVecBase3f, new_scale: LVecBase3f
    ) -> None:
        """将一次缩放操作压入撤销栈。"""
        if node is None or node.isEmpty():
            return
        if old_scale is None or new_scale is None:
            return

        delta = new_scale - old_scale
        if delta.length() < self._undo_scale_epsilon:
            return

        old_copy = Vec3(old_scale)
        new_copy = Vec3(new_scale)
        # 组合控制器已有统一历史，避免重复保留目标节点。
        if self.on_action_committed is None:
            self._scale_undo_stack.append((node, old_copy, new_copy))
        self._log(
            f"record scale action: node={node.getName()} "
            f"old_scale={old_copy} new_scale={new_copy} stack_size={len(self._scale_undo_stack)}"
        )

        if self.on_action_committed is not None:
            try:
                self.on_action_committed(
                    {
                        "kind": "scale",
                        "node": node,
                        "old_scale": Vec3(old_copy),
                        "new_scale": Vec3(new_copy),
                    }
                )
            except Exception as exc:
                self._log(f"on_action_committed(scale) error: {exc}")

    def undo_last_scale(self) -> None:
        """撤销最近一次缩放操作。"""
        if not self._scale_undo_stack:
            self._log("undo_last_scale: stack empty")
            return
        node, old_scale, new_scale = self._scale_undo_stack.pop()
        if node is None or node.isEmpty():
            self._log("undo_last_scale: target node invalid")
            return
        node.setScale(old_scale)
        self._log(
            f"undo_last_scale: node={node.getName()} "
            f"old_scale={old_scale} new_scale={new_scale} remaining={len(self._scale_undo_stack)}"
        )

    def _register_events(self) -> None:
        """注册鼠标事件监听。"""
        self.accept("mouse1", self._on_mouse_down)
        self.accept("mouse1-up", self._on_mouse_up)
        self.accept("mouse2-up", self._on_mouse_up)
        self.accept("mouse3-up", self._on_mouse_up)
        self.accept("mouse-move", self._on_mouse_move)
        self._log("event listeners registered (mouse)")

    def _ignore_events(self) -> None:
        """取消事件监听。"""
        self.ignore("mouse1")
        self.ignore("mouse1-up")
        self.ignore("mouse2-up")
        self.ignore("mouse3-up")
        self.ignore("mouse-move")
        self._log("event listeners removed")

    def _get_normalized_mouse(self, extra: MouseInput | None = None) -> Point3 | None:
        """获取标准化鼠标坐标，兼容 Panda3D 和外部 UI。"""
        return normalized_mouse(self.world, extra)

    def _on_mouse_down(self, extra: MouseInput | None = None) -> None:
        if not self.attached or not self.camera or self.target_node is None:
            return

        mpos = self._get_normalized_mouse(extra)
        if mpos is None:
            self._log("mouse_down ignored: no mouse pos")
            return

        self.picker_np.reparentTo(self.camera)
        if not self._picker_added:
            self.c_trav.addCollider(self.picker_np, self.c_queue)
            self._picker_added = True
        self.picker_ray.setFromLens(self.camera.node(), mpos.x, mpos.y)

        self.c_queue.clearEntries()
        self.c_trav.traverse(self.root)

        self._log(
            f"mouse_down mpos={mpos} cam_pos={self.camera.getPos(self.world.render)} "
            f"entries={self.c_queue.getNumEntries()}"
        )

        num_entries = self.c_queue.getNumEntries()
        if num_entries <= 0:
            self._log("mouse_down: no collision entries on scale gizmo")
            return

        self.c_queue.sortEntries()

        axis_entry = None
        center_entry = None
        for i in range(num_entries):
            e = self.c_queue.getEntry(i)
            if not center_entry and e.getIntoNode().getTag("gizmo_center"):
                center_entry = e
            if not axis_entry and e.getIntoNode().getTag("gizmo_axis"):
                axis_entry = e
            if center_entry and axis_entry:
                break

        # 优先选择中心控制柄
        if center_entry is not None:
            self.dragging = True
            self.drag_axis = 3  # Uniform scale
            self.start_scale = Vec3(self.target_node.getScale())
            self.start_mouse = Point2(mpos.x, mpos.y)
            node_pos_w = self.target_node.getPos(self.world.render)
            cam_pos = self.camera.getPos(self.world.render)
            self.drag_base_dist = max((cam_pos - node_pos_w).length(), 0.5)
            self._debug_drag_counter = 0
            self._highlight_center()
            self._log(
                f"pick center (uniform scale) target={self.target_node.getName()} "
                f"start_scale={self.start_scale}"
            )
            self._emit_event(
                GizmoEvent.DRAG_START,
                {
                    "gizmo": "scale",
                    "target": self.target_node,
                    "axis": self.drag_axis,
                    "mouse": Point2(mpos.x, mpos.y),
                    "start_scale": self.start_scale,
                },
            )
        elif axis_entry is not None:
            axis_tag: str = axis_entry.getIntoNode().getTag("gizmo_axis")
            self.dragging = True
            self.drag_axis = int(axis_tag)
            self.start_scale = Vec3(self.target_node.getScale())
            self.start_mouse = Point2(mpos.x, mpos.y)

            axis_index = int(axis_tag)
            axis_np: NodePath = self.axes[axis_index]
            axis_dir = Vec3(axis_np.getQuat(self.world.render).xform(Vec3(0, 0, 1)))
            if axis_dir.length_squared() == 0:
                axis_dir = Vec3(0, 0, 1)
            axis_dir.normalize()
            self.drag_axis_dir = Vec3(axis_dir)

            cam_pos = self.camera.getPos(self.world.render)
            node_pos_w = self.target_node.getPos(self.world.render)
            lens: Lens = self.camera.node().getLens()
            self.drag_base_dist = max((cam_pos - node_pos_w).length(), 0.5)

            sample_len = max(self.drag_base_dist * 0.25, 0.25)
            p0_cam = self.camera.getRelativePoint(self.world.render, node_pos_w)
            p1_cam = self.camera.getRelativePoint(
                self.world.render, node_pos_w + axis_dir * sample_len
            )
            p0_2d = Point2()
            p1_2d = Point2()
            # 注意：即使 lens.project 返回 False（点可能超出裁剪范围），
            # 它仍然会填充 p0_2d/p1_2d，我们可以使用这些值来计算方向
            proj_ok_0 = lens.project(p0_cam, p0_2d)
            proj_ok_1 = lens.project(p1_cam, p1_2d)

            # 无论 project 返回值如何，都尝试计算屏幕方向
            axis_screen_raw = Point2(p1_2d - p0_2d)
            axis_screen_len = axis_screen_raw.length()
            if axis_screen_len > 1e-6:
                axis_screen = axis_screen_raw / axis_screen_len
            else:
                # 只有当投影长度真的为0时才使用默认值
                axis_screen = Point2(1, 0)
            self.drag_axis_screen = Point2(axis_screen)
            self._axis_screen_len = axis_screen_len  # 保存用于诊断

            # 计算摄像机视线方向与轴的夹角（用于诊断）
            view_dir = node_pos_w - cam_pos
            if view_dir.length_squared() > 0:
                view_dir.normalize()
            axis_view_dot = abs(axis_dir.dot(view_dir))

            self._debug_drag_counter = 0
            self._highlight_axis(self.drag_axis)

            # 诊断打印
            strer: list[str] = []
            strer.append("[ScaleGizmo DEBUG] ========== AXIS PICK ==========")
            strer.append(f"  axis_id: {self.drag_axis} ({'XYZ'[self.drag_axis]})")
            strer.append(f"  axis_dir (world): {axis_dir}")
            strer.append(f"  cam_pos: {cam_pos}")
            strer.append(f"  node_pos: {node_pos_w}")
            strer.append(f"  view_dir: {view_dir}")
            strer.append(f"  axis_view_dot (|axis·view|): {axis_view_dot:.4f}")
            strer.append(f"  p0_cam: {p0_cam}, p1_cam: {p1_cam}")
            strer.append(f"  proj_ok: p0={proj_ok_0}, p1={proj_ok_1}")
            strer.append(f"  p0_2d: {p0_2d}, p1_2d: {p1_2d}")
            strer.append(f"  axis_screen_raw: ({axis_screen_raw.x:.4f}, {axis_screen_raw.y:.4f})")
            strer.append(f"  axis_screen_len (before normalize): {axis_screen_len:.6f}")
            strer.append(f"  axis_screen (normalized): ({axis_screen.x:.4f}, {axis_screen.y:.4f})")
            if axis_view_dot > 0.9:
                strer.append("  WARNING: Axis nearly parallel to view direction!")
            if axis_screen_len < 0.01:
                strer.append("  WARNING: Axis screen projection very short!")
            strer.append("  ==========================================")
            self._log("\n".join(strer))

            self._log(
                f"pick axis={self.drag_axis} target={self.target_node.getName()} "
                f"start_scale={self.start_scale} axis_dir={axis_dir}"
            )
            self._emit_event(
                GizmoEvent.DRAG_START,
                {
                    "gizmo": "scale",
                    "target": self.target_node,
                    "axis": self.drag_axis,
                    "mouse": Point2(mpos.x, mpos.y),
                    "start_scale": self.start_scale,
                },
            )
        else:
            self._log("mouse_down: hit something but no gizmo_axis or gizmo_center tag")

    def _on_mouse_up(self, extra: MouseInput | None = None) -> None:
        if self.dragging and self.target_node is not None:
            try:
                final_scale = self.target_node.getScale()
            except Exception:
                final_scale = None
            old_scale = self.start_scale
            if old_scale is not None and final_scale is not None:
                self._record_scale_action(self.target_node, old_scale, final_scale)

            axis_id = self.drag_axis
            self._emit_event(
                GizmoEvent.DRAG_END,
                {
                    "gizmo": "scale",
                    "target": self.target_node,
                    "axis": axis_id,
                    "start_scale": old_scale,
                    "final_scale": final_scale,
                },
            )
        self.dragging = False
        self.drag_axis = None
        self._reset_axis_visuals()
        self._reset_highlights()
        self._log("mouse_up -> stop scaling")

    def _on_mouse_move(self, extra: MouseInput | None = None) -> None:
        if not self.attached or self.target_node is None:
            return

        mpos = self._get_normalized_mouse(extra)
        if mpos is None:
            if self.dragging:
                self._log("mouse_move ignored: no mouse pos while scaling")
            return

        # Hover highlight when idle.
        if not self.dragging:
            self._update_hover_highlight(mpos)
            return

        if not self.target_node:
            return

        if self.start_mouse is None or self.start_scale is None:
            return
        mouse_ndc = Point2(mpos.x, mpos.y)
        delta = mouse_ndc - self.start_mouse

        if self.drag_axis == 3:
            # 均匀缩放：基于鼠标在屏幕上的垂直移动
            scale_factor = 1.0 + delta.y * 2.0
            scale_factor = max(scale_factor, 0.01)  # 防止负缩放
            new_scale = self.start_scale * scale_factor
            self.target_node.setScale(new_scale)

            if self._debug_drag_counter % 4 == 0:
                self._log(f"uniform scale factor={scale_factor:.4f} new_scale={new_scale}")
            self._debug_drag_counter += 1
            self._emit_event(
                GizmoEvent.DRAG_MOVE,
                {
                    "gizmo": "scale",
                    "target": self.target_node,
                    "axis": self.drag_axis,
                    "mouse": Point2(mpos.x, mpos.y),
                    "new_scale": new_scale,
                    "scale_factor": scale_factor,
                },
            )
            self._update_axis_visuals_for_scale(new_scale)
        else:
            # 单轴缩放
            s_scalar = delta.dot(self.drag_axis_screen)
            scale_factor = 1.0 + s_scalar * 2.0
            scale_factor = max(scale_factor, 0.01)

            new_scale = Vec3(self.start_scale)
            if self.drag_axis == 0:
                new_scale.x = self.start_scale.x * scale_factor
            elif self.drag_axis == 1:
                new_scale.y = self.start_scale.y * scale_factor
            elif self.drag_axis == 2:
                new_scale.z = self.start_scale.z * scale_factor

            self.target_node.setScale(new_scale)

            # 诊断打印（每16帧打印一次，避免刷屏）
            # if self._debug_drag_counter % 16 == 0:
            #     axis_screen_len = getattr(self, '_axis_screen_len', 0.0)
            #     print(f"[ScaleGizmo DRAG] axis={self.drag_axis}({'XYZ'[self.drag_axis]}) "
            #           f"delta=({delta.x:.4f}, {delta.y:.4f}) "
            #           f"axis_screen=({self.drag_axis_screen.x:.4f}, {self.drag_axis_screen.y:.4f}) "
            #           f"axis_screen_len={axis_screen_len:.4f} "
            #           f"s_scalar={s_scalar:.4f} "
            #           f"scale_factor={scale_factor:.4f}")
            self._debug_drag_counter += 1
            self._emit_event(
                GizmoEvent.DRAG_MOVE,
                {
                    "gizmo": "scale",
                    "target": self.target_node,
                    "axis": self.drag_axis,
                    "mouse": Point2(mpos.x, mpos.y),
                    "new_scale": new_scale,
                    "scale_factor": scale_factor,
                },
            )
            self._update_axis_visuals_for_scale(new_scale)

    def _update_hover_highlight(self, mpos: Point3) -> None:
        """Highlight hovered handle during mouse move without dragging."""
        self.picker_np.reparentTo(self.camera)
        if not self._picker_added:
            self.c_trav.addCollider(self.picker_np, self.c_queue)
            self._picker_added = True
        self.picker_ray.setFromLens(self.camera.node(), mpos.x, mpos.y)

        self.c_queue.clearEntries()
        self.c_trav.traverse(self.root)

        num_entries = self.c_queue.getNumEntries()
        if num_entries <= 0:
            self._reset_highlights()
            self.is_hovering = False
            return

        self.c_queue.sortEntries()

        axis_entry = None
        center_entry = None
        for i in range(num_entries):
            e = self.c_queue.getEntry(i)
            if center_entry is None and e.getIntoNode().getTag("gizmo_center"):
                center_entry = e
            if axis_entry is None and e.getIntoNode().getTag("gizmo_axis"):
                axis_entry = e
            if center_entry and axis_entry:
                break

        if center_entry is not None:
            self._highlight_center()
            self.is_hovering = True
        elif axis_entry is not None:
            axis_tag: str = axis_entry.getIntoNode().getTag("gizmo_axis")
            self._highlight_axis(int(axis_tag))
            self.is_hovering = True
        else:
            self._reset_highlights()
            self.is_hovering = False

    def _highlight_axis(self, axis_id: int | None) -> None:
        """高亮被选中的轴。"""
        self._reset_highlights()
        for i, ax in enumerate(self.axes):
            if i == axis_id:
                ax.setColor(1, 1, 0, 1)
                ax.clearColorScale()
            else:
                ax.clearColor()
                ax.setAlphaScale(0.3)
        if self.center_handle:
            self.center_handle.setAlphaScale(0.3)

    def _highlight_center(self) -> None:
        """高亮中心控制柄。"""
        for ax in self.axes:
            ax.clearColor()
            ax.setAlphaScale(0.3)
        if self.center_handle:
            self.center_handle.setColor(1, 1, 0, 1)
            self.center_handle.clearColorScale()

    def _reset_highlights(self) -> None:
        """重置所有高亮。"""
        for ax in self.axes:
            ax.clearColor()
            ax.clearColorScale()
            ax.setAlphaScale(self.axis_alpha)
        if self.center_handle:
            self.center_handle.clearColor()
            self.center_handle.clearColorScale()
            self.center_handle.setAlphaScale(self.axis_alpha)

    # --- 可视轴动态长度 -------------------------------------------------
    def _set_axis_visual_factor(self, axis_id: int, factor: float) -> None:
        """仅调整可视几何长度/末端大小，不改碰撞体。"""
        if axis_id < 0 or axis_id >= len(self.axis_visuals):
            return
        info = self.axis_visuals[axis_id]
        line = info.get("line")
        cube = info.get("cube")
        base_len = info.get("length", self._axis_length0)
        if line is None or cube is None:
            return
        factor = max(0.3, min(factor, 3.0))
        line.setScale(1, 1, factor)
        cube.setZ(base_len * factor)
        cube.setScale(factor)

    def _reset_axis_visuals(self) -> None:
        """恢复所有轴的可视长度/端点大小。"""
        for i in range(len(self.axis_visuals)):
            self._set_axis_visual_factor(i, 1.0)

    def _update_axis_visuals_for_scale(self, new_scale: Vec3) -> None:
        """根据当前缩放比例动态调整可视轴长度。"""
        if self.start_scale is None:
            return

        def safe_ratio(a: float, b: float) -> float:
            return a / b if abs(b) > 1e-6 else 1.0

        sx0, sy0, sz0 = self.start_scale
        sx1, sy1, sz1 = new_scale

        if self.drag_axis == 3:
            ratios: list[float] = []
            if abs(sx0) > 1e-6:
                ratios.append(sx1 / sx0)
            if abs(sy0) > 1e-6:
                ratios.append(sy1 / sy0)
            if abs(sz0) > 1e-6:
                ratios.append(sz1 / sz0)
            if not ratios:
                return
            factor = sum(ratios) / len(ratios)
            for i in range(3):
                self._set_axis_visual_factor(i, factor)
        elif self.drag_axis is not None and 0 <= self.drag_axis <= 2:
            ratio = [safe_ratio(sx1, sx0), safe_ratio(sy1, sy0), safe_ratio(sz1, sz0)]
            self._set_axis_visual_factor(self.drag_axis, ratio[self.drag_axis])

    def update(self) -> None:
        """根据摄像机距离自动缩放 Gizmo。"""
        if (
            not getattr(self, "managed_input", False)
            and self.world.mouseWatcherNode is not None
            and self.world.mouseWatcherNode.has_mouse()
        ):
            current_mouse = LPoint2f(self.world.mouseWatcherNode.get_mouse())
            if current_mouse != self.last_mouse_pos or self.last_mouse_pos is None:
                self._on_mouse_move()
                self.last_mouse_pos = current_mouse

        if self.attached and self.camera:
            self._update_root_transform()
            self.root.setScale(self.world.render, screen_scale(self))

    def _update_task(self, task: Task) -> int:
        """每帧调用，用于持续更新 Gizmo 的缩放。"""
        self.update()
        return Task.cont

    # ------------------------------------------------------------------ #
    # 内部：跟随目标更新根节点变换
    # ------------------------------------------------------------------ #
    def _update_root_transform(self) -> None:
        if not self.attached or not self.target_node:
            return
        render = self.world.render
        tgt = self.target_node
        self.root.setPos(render, tgt.getPos(render))
        if self.is_local:
            self.root.setQuat(render, tgt.getQuat(render))
        else:
            self.root.setQuat(render, Quat.identQuat())

    def destroy(self) -> None:
        """释放控制柄及输入资源。"""
        destroy_gizmo(self)


__all__ = ["ScaleGizmo"]
