"""旋转控制柄：程序化几何、碰撞拾取与拖拽计算。"""

from __future__ import annotations

import math
from typing import cast

from direct.showbase.DirectObject import DirectObject
from direct.showbase.ShowBase import ShowBase
from direct.task.Task import Task
from panda3d.core import (
    BitMask32,
    CollisionEntry,
    CollisionHandlerQueue,
    CollisionNode,
    CollisionRay,
    CollisionSphere,
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
from .types import ActionHandler, DragEvent, EventHooks, MouseInput

"""
旋转控制柄（Rotate Gizmo）
=========================

功能概述：
    - 模仿 Unity 的旋转控制柄，使用 3 个彩色圆环表示绕 X/Y/Z 轴旋转
      - X 轴：红色圆环（绕局部 X 轴旋转）
      - Y 轴：绿色圆环（绕局部 Y 轴旋转）
      - Z 轴：蓝色圆环（绕局部 Z 轴旋转）
    - 额外提供屏幕轨迹球（trackball）用于自由旋转
    - 拖拽时展示半透明角度扇形与箭头，辅助观察旋转量
    - 中心轴线辅助观察当前坐标系朝向
    - 使用 Panda3D 的碰撞系统进行鼠标拾取与拖拽
    - 控件跟随目标节点的局部坐标系（即“局部模式”），
      当目标有旋转时，三个圆环会随之旋转
    - 控件会根据摄像机距离自动缩放，屏幕大小保持近似不变

集成方式（示例）：
    from direct.showbase.ShowBase import ShowBase
    from TransformGizmo.rotate_gizmo import RotateGizmo

    world = ShowBase()
    model_np = world.render.attachNewNode("Box")
    # ... 在 model_np 下加载模型

    gizmo = RotateGizmo(world)         # 创建旋转 Gizmo
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


class RotateGizmo(DirectObject):
    """
    Unity 风格的旋转控制柄：
        - 3 个圆环分别代表绕局部 X/Y/Z 轴旋转
        - 额外的屏幕轨迹球，便于自由旋转
        - 左键点击圆环并拖拽，可以绕对应轴旋转绑定的 NodePath
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
        self.is_local: bool = True
        self.managed_input: bool = False
        self.debug: bool = False
        self.is_hovering = False
        self._picker_added: bool = False
        # Optional callback to report completed rotate actions to a higher-level
        # manager (e.g. TransformGizmo) so it can build a global undo stack.
        self.on_action_committed = on_action_committed
        # Event hooks: drag_start / drag_move / drag_end
        self._event_hooks = self._normalize_event_hooks(event_hooks)

        # 摄像机节点，如果未手动传入，则尝试从 world/base 中自动获取
        self.camera = camera_path(world, camera_np)
        if not self.camera:
            self._log("No camera found for RotateGizmo")
        else:
            self._log(f"Camera found for RotateGizmo: {self.camera}")

        # 控制柄根节点
        self.root: NodePath = NodePath("RotateGizmo")
        self.color_higher = 0.05
        self.color_normal = 0.01

        # 当前绑定的目标节点
        self.target_node: NodePath | None = None
        self.attached: bool = False

        self.last_mouse_pos: Point2 | None = None

        # 控制柄视觉参数
        self.ring_radius: float = 0.45  # 圆环半径（局部空间）
        self.ring_thickness: float = 0.04  # 圆环粗细（碰撞体半径）
        self.ring_alpha: float = 0.9  # 圆环透明度
        self.trackball_radius: float = self.ring_radius * 1.0  # 轨迹球半径（略小，避免遮挡轴拾取）
        self.trackball_color: Vec4 = Vec4(0.25, 0.25, 0.25, 0.45)
        self.trackball_edge_color: Vec4 = Vec4(1, 1, 1, 0.75)
        self.highlight_color: Vec4 = Vec4(1.0, 1.0, 1.0, 1.0)
        self.arrow_length_factor: float = 0.35  # 箭头长度相对圆环半径的倍数
        self.arrow_head_width_factor: float = 0.25  # 箭头头部宽度相对箭长的倍数

        # 拾取射线与碰撞系统
        self.picker_ray = CollisionRay()
        self.picker_node = CollisionNode("rotate_gizmo_picker_ray")
        self.picker_node.addSolid(self.picker_ray)
        self.picker_node.setFromCollideMask(BitMask32.bit(20))
        self.picker_node.setIntoCollideMask(BitMask32.allOff())
        self.picker_np: NodePath = NodePath(self.picker_node)

        self.c_trav = CollisionTraverser()
        self.c_queue = CollisionHandlerQueue()

        # 拖拽状态
        self.dragging: bool = False
        self.drag_axis: int | None = None  # 0 = X, 1 = Y, 2 = Z
        self.drag_mode: str | None = None  # "axis" / "trackball"
        self.drag_local_axis: Vec3 | None = None
        self.start_mouse: Point2 | None = None
        self._axis_screen_dir: Point2 | None = None
        # 起始姿态：世界与父坐标系两份，方便选用一致的参考系进行旋转
        self.start_quat: Quat | None = None  # 世界四元数
        self.start_quat_local: Quat | None = None  # 父坐标系四元数
        self.start_root_axis_dir: Vec3 | None = None
        self.rotate_axis_world: Vec3 = Vec3(0, 0, 0)  # 当前轴的世界方向
        self.start_vec_on_plane: Vec3 = Vec3(0, 0, 0)  # 旋转平面上的起始向量
        self.center_world: Point3 = Point3(0, 0, 0)  # Gizmo 中心（世界坐标）
        self._trackball_start_vec: Vec3 | None = None  # 屏幕空间轨迹球起始向量
        self._angle_start_offset: float = 0.0  # 扇形起点（弧度）
        self._drag_root_quat_world: Quat | None = None  # 拖拽时保持的 Gizmo 世界朝向
        self._arrow_tangent_angle_deg: float = 0.0  # 箭头朝向（在扇形平面内的角度）
        self._start_vec_local: Vec3 | None = None  # 起始向量（扇形局部坐标）

        # 撤销栈：每项为 (node, old_hpr(world), new_hpr(world))
        self._rotate_undo_stack: list[tuple[NodePath, Vec3, Vec3]] = []
        self._undo_angle_epsilon: float = 0.01  # 角度变化阈值（防止浮点抖动）

        # 边缘视角（grazing angle）时使用屏幕空间旋转的参数
        self._grazing_threshold: float = 0.15  # denom 小于此值时启用屏幕空间计算
        self._center_screen: Point2 | None = None  # Gizmo 中心的屏幕投影
        self._start_screen_angle: float = 0.0  # 起始鼠标相对于屏幕中心的角度
        self._axis_screen_sign: float = 1.0  # 屏幕空间旋转方向符号
        self._last_screen_angle: float | None = None
        self._last_screen_mouse: Point2 | None = None
        self._drag_accumulated_deg: float = 0.0
        self._last_drag_vec_on_plane: Vec3 | None = None

        # 三个圆环节点：0=X, 1=Y, 2=Z
        self.rings: list[NodePath] = []
        self.ring_colors: list[tuple[NodePath, Vec4]] = []
        # (front_geom_np, back_geom_np, front_col_np, back_col_np)
        self.ring_halves: list[tuple[NodePath, NodePath, NodePath, NodePath]] = []
        self.center_axes: list[NodePath] = []
        self.center_axes_colors: list[tuple[NodePath, Vec4]] = []
        self.trackball_np: NodePath | None = None
        self.angle_disc_root: NodePath | None = None
        self.angle_fill_np: NodePath | None = None
        self.angle_fill_np_copy: NodePath | None = None

        self.angle_arrow_np: NodePath | None = None

        # 构建几何与碰撞体
        self._build_gizmo()

        # 初始挂在 world.render，后续通过更新同步到目标
        self.root.reparentTo(self.world.render)
        self.root.setBin("fixed", 40)
        self.root.setDepthTest(False)
        self.root.setDepthWrite(False)
        self.root.setLightOff()
        self.root.hide()

        # 每帧更新 Gizmo 缩放
        # taskMgr.add(self._update_task, "RotateGizmoUpdateTask")

    # ------------------------------------------------------------------ #
    # 内部工具
    # ------------------------------------------------------------------ #
    def _log(self, msg: str) -> None:
        if self.debug:
            print(f"[RotateGizmo] {msg}")

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
        """构建旋转 Gizmo 的可视化元素（轴环、轨迹球、角度扇形等）。"""
        # 屏幕对齐的轨迹球，用于自由旋转
        # 轨迹球：默认隐藏可视几何，仅在选中时显示
        self.trackball_np = self._create_trackball_disc(
            radius=self.trackball_radius,
            color=self.trackball_color,
            visual=True,
        )
        if self.trackball_np:
            trackball_geom = self.trackball_np.find("**/trackball_geom")
            if not trackball_geom.isEmpty():
                trackball_geom.hide()
        # X 轴（红）—— 绕局部 X 轴旋转
        self.rings.append(
            self._create_ring(
                axis_dir=Vec3(1, 0, 0),
                color=Vec4(1, 0, 0, 1),
                radius=self.ring_radius,
                thickness=self.ring_thickness,
                axis_id=0,
            )
        )
        # Y 轴（绿）—— 绕局部 Y 轴旋转
        self.rings.append(
            self._create_ring(
                axis_dir=Vec3(0, 1, 0),
                color=Vec4(0, 1, 0, 1),
                radius=self.ring_radius,
                thickness=self.ring_thickness,
                axis_id=1,
            )
        )
        # Z 轴（蓝）—— 绕局部 Z 轴旋转
        self.rings.append(
            self._create_ring(
                axis_dir=Vec3(0, 0, 1),
                color=Vec4(0, 0, 1, 1),
                radius=self.ring_radius,
                thickness=self.ring_thickness,
                axis_id=2,
            )
        )
        # 轨迹球边缘辅助圆，标示球的尺寸（无碰撞）
        self.trackball_edge = self._create_trackball_edge(
            radius=self.trackball_radius,
            color=self.trackball_edge_color,
        )
        # 中心轴线，帮助观察旋转轴朝向
        self.center_axes = [
            self._create_center_axis(Vec3(1, 0, 0), Vec4(0.3, 0.0, 0.0, 1.0)),
            self._create_center_axis(Vec3(0, 1, 0), Vec4(0.0, 0.3, 0.0, 1.0)),
            self._create_center_axis(Vec3(0, 0, 1), Vec4(0.0, 0.0, 0.3, 1.0)),
        ]
        # 角度可视化扇形
        self._build_angle_disc()

    def _create_ring(
        self,
        axis_dir: Vec3,
        color: Vec4,
        radius: float,
        thickness: float,
        axis_id: int,
    ) -> NodePath:
        """
        构建一个圆环：
            - 几何：使用 GeomLines 画出若干线段组成的圆
            - 碰撞体：使用多段 CollisionTube 近似圆环
        默认圆环在局部 XY 平面，法线为 +Z，通过四元数旋转到指定 axis_dir。
        """
        ring_root = self.root.attachNewNode(f"ring_{axis_id}")

        # 本地 +Z 对齐到目标“旋转轴方向”（即圆环法线）
        align_quat = self._quat_from_z(axis_dir)
        ring_root.setQuat(align_quat)

        # -------------------- 几何圆环 --------------------
        vdata = GeomVertexData("ring", GeomVertexFormat.getV3c4(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, "vertex")
        vcolor = GeomVertexWriter(vdata, "color")

        segments = 64
        for i in range(segments + 1):
            angle = 2 * math.pi * i / segments
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            vertex.addData3(x, y, 0.0)
            vcolor.addData4(color)

        # 将圆拆分为两个半圆环，方便根据摄像机方向隐藏背面的半圆。
        half_segments = segments // 2

        # 前半圆：索引 [0, half_segments]
        prim_front = GeomLines(Geom.UHStatic)
        for i in range(half_segments):
            prim_front.addVertices(i, i + 1)
        geom_front = Geom(vdata)
        geom_front.addPrimitive(prim_front)
        node_front = GeomNode("ring_geom_front")
        node_front.addGeom(geom_front)
        np_front = ring_root.attachNewNode(node_front)
        np_front.setTransparency(TransparencyAttrib.MAlpha)
        np_front.setColorScale(1, 1, 1, self.ring_alpha)

        # 后半圆：索引 [half_segments, segments]
        prim_back = GeomLines(Geom.UHStatic)
        for i in range(half_segments, segments):
            prim_back.addVertices(i, i + 1)
        geom_back = Geom(vdata)
        geom_back.addPrimitive(prim_back)
        node_back = GeomNode("ring_geom_back")
        node_back.addGeom(geom_back)
        np_back = ring_root.attachNewNode(node_back)
        np_back.setTransparency(TransparencyAttrib.MAlpha)
        np_back.setColorScale(1, 1, 1, self.ring_alpha)
        # -------------------- 碰撞圆环拆分为两半 --------------------
        col_segments = 24
        half_col = col_segments // 2

        c_node_front = CollisionNode(f"ring_col_front_{axis_id}")
        c_node_front.setIntoCollideMask(BitMask32.bit(20))
        c_node_front.setFromCollideMask(BitMask32.allOff())
        c_node_front.setTag("gizmo_axis", str(axis_id))
        for i in range(half_col):
            a0 = 2 * math.pi * i / col_segments
            a1 = 2 * math.pi * (i + 1) / col_segments
            p0 = Point3(radius * math.cos(a0), radius * math.sin(a0), 0.0)
            p1 = Point3(radius * math.cos(a1), radius * math.sin(a1), 0.0)
            c_node_front.addSolid(CollisionTube(p0, p1, thickness))
        col_front_np = ring_root.attachNewNode(c_node_front)

        c_node_back = CollisionNode(f"ring_col_back_{axis_id}")
        c_node_back.setIntoCollideMask(BitMask32.bit(20))
        c_node_back.setFromCollideMask(BitMask32.allOff())
        c_node_back.setTag("gizmo_axis", str(axis_id))
        for i in range(half_col, col_segments):
            a0 = 2 * math.pi * i / col_segments
            a1 = 2 * math.pi * (i + 1) / col_segments
            p0 = Point3(radius * math.cos(a0), radius * math.sin(a0), 0.0)
            p1 = Point3(radius * math.cos(a1), radius * math.sin(a1), 0.0)
            c_node_back.addSolid(CollisionTube(p0, p1, thickness))
        col_back_np = ring_root.attachNewNode(c_node_back)

        # 默认先都显示，后续在 update() 中根据摄像机位置只显示靠近摄像机的一半（含碰撞）
        self.ring_halves.append((np_front, np_back, col_front_np, col_back_np))

        return ring_root

    def _create_center_axis(self, axis_dir: Vec3, color: Vec4) -> NodePath:
        """创建中心轴线，方便观察轴向。"""
        vdata = GeomVertexData("center_axis", GeomVertexFormat.getV3c4(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, "vertex")
        vcolor = GeomVertexWriter(vdata, "color")

        vertex.addData3(0.0, 0.0, 0.0)
        vcolor.addData4(color)
        tip = axis_dir * (self.ring_radius * 0.8)
        vertex.addData3(tip)
        vcolor.addData4(color)

        lines = GeomLines(Geom.UHStatic)
        lines.addVertices(0, 1)
        geom = Geom(vdata)
        geom.addPrimitive(lines)
        node = GeomNode("center_axis_geom")
        node.addGeom(geom)
        np = self.root.attachNewNode(node)
        np.setTransparency(TransparencyAttrib.MAlpha)
        np.setColorScale(1, 1, 1, 0.9)
        return np

    def _create_trackball_edge(self, radius: float, color: Vec4) -> NodePath:
        """创建轨迹球尺寸辅助圆（无碰撞，仅显示）。"""
        edge_root = self.root.attachNewNode("trackball_edge")
        vdata = GeomVertexData("trackball_edge", GeomVertexFormat.getV3c4(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, "vertex")
        vcolor = GeomVertexWriter(vdata, "color")

        segments = 96
        for i in range(segments + 1):
            ang = 2 * math.pi * i / segments
            x = radius * math.cos(ang)
            z = radius * math.sin(ang)
            vertex.addData3(x, 0.0, z)
            vcolor.addData4(color)

        prim = GeomLines(Geom.UHStatic)
        for i in range(segments):
            prim.addVertices(i, i + 1)
        geom = Geom(vdata)
        geom.addPrimitive(prim)
        node = GeomNode("trackball_edge_geom")
        node.addGeom(geom)
        np_geom = edge_root.attachNewNode(node)
        np_geom.setRenderModeThickness(1.5)
        np_geom.setBillboardPointEye()
        np_geom.setTransparency(TransparencyAttrib.MAlpha)
        np_geom.setColorScale(color)
        return edge_root

    def _create_trackball_disc(self, radius: float, color: Vec4, visual: bool = True) -> NodePath:
        """创建屏幕对齐的轨迹球圆盘。visual=False 时仅保留拾取体。"""
        disc_root = self.root.attachNewNode("trackball_disc")

        if visual:
            np_geom = self.make_sphere(radius=radius, color=color)
            np_geom.set_name("trackball_geom")
            np_geom.wrt_reparent_to(disc_root)
            np_geom.setTransparency(TransparencyAttrib.MAlpha)
        # 拾取使用球体，方便捕获整个圆盘区域
        c_node = CollisionNode("trackball_col")
        c_node.setIntoCollideMask(BitMask32.bit(20))
        c_node.setFromCollideMask(BitMask32.allOff())
        c_node.setTag("gizmo_handle", "trackball")
        c_node.addSolid(CollisionSphere(0, 0, 0, radius + self.ring_thickness * 0.25))
        disc_root.attachNewNode(c_node)

        return disc_root

    def make_sphere(
        self,
        radius: float = 0.5,
        lat_steps: int = 32,
        lon_steps: int = 64,
        color: Vec4 | tuple[float, float, float, float] = (1, 1, 1, 0.5),
    ) -> NodePath:
        fmt = GeomVertexFormat.getV3c4()
        vdata = GeomVertexData("sphere", fmt, Geom.UHStatic)
        vw = GeomVertexWriter(vdata, "vertex")
        cw = GeomVertexWriter(vdata, "color")
        tris = GeomTriangles(Geom.UHStatic)

        # 顶点
        for i in range(lat_steps + 1):  # 纬度 0..pi
            theta = math.pi * i / lat_steps
            st, ct = math.sin(theta), math.cos(theta)
            for j in range(lon_steps):  # 经度 0..2pi
                phi = 2 * math.pi * j / lon_steps
                sp, cp = math.sin(phi), math.cos(phi)
                x, y, z = radius * cp * st, radius * sp * st, radius * ct
                vw.addData3(x, y, z)
                cw.addData4(*color)

        # 索引
        def vid(i: int, j: int) -> int:
            """将纬度/经度索引转换为顶点编号，经度索引循环闭合。"""
            return i * lon_steps + (j % lon_steps)

        for i in range(lat_steps):
            for j in range(lon_steps):
                a = vid(i, j)
                b = vid(i + 1, j)
                c = vid(i + 1, j + 1)
                d = vid(i, j + 1)
                tris.addVertices(a, b, c)
                tris.addVertices(a, c, d)

        geom = Geom(vdata)
        geom.addPrimitive(tris)
        node = GeomNode("hi_sphere")
        node.addGeom(geom)
        np = NodePath(node)
        return np

    def _build_angle_disc(self) -> None:
        """搭建旋转角度的可视化扇形。"""
        self.angle_disc_root = self.root.attachNewNode("angle_disc_root")
        self.angle_disc_root.setTransparency(TransparencyAttrib.M_alpha)
        self.angle_disc_root.setTwoSided(True)
        self.angle_disc_root.hide()

        self.angle_fill_np = self.angle_disc_root.attachNewNode(GeomNode("angle_sector"))
        self.angle_fill_np.setTransparency(TransparencyAttrib.M_alpha)
        self.angle_fill_np.setColor(0.9, 0.9, 0.9, 0.5)
        # 末端箭头
        self.angle_arrow_np = self._create_angle_arrow(
            self.angle_disc_root,
            self.ring_radius * self.arrow_length_factor,
            self.arrow_head_width_factor,
        )
        self.angle_arrow_np.setColor(1.0, 1.0, 1.0, 1.0)
        self.angle_arrow_np.hide()

    def _create_angle_disc_half(self, parent: NodePath, mirror: bool = False) -> NodePath:
        """创建角度盘的一半，方便做基础背景。"""
        segments = 50
        angle = math.pi / segments
        angle = -angle if mirror else angle
        offset = math.pi * 0.5

        vdata = GeomVertexData("angle_disc_half", GeomVertexFormat.getV3(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, "vertex")

        for i in range(segments + 1):
            x = math.cos(angle * i - offset)
            y = math.sin(angle * i - offset)
            vertex.addData3(x, y, 0.0)

        prim = GeomTriangles(Geom.UHStatic)
        for i in range(1, segments):
            prim.addVertices(0, i, i + 1)
        geom = Geom(vdata)
        geom.addPrimitive(prim)
        node = GeomNode("angle_disc_half_node")
        node.addGeom(geom)
        half_np = parent.attachNewNode(node)
        half_np.setTransparency(TransparencyAttrib.M_alpha)
        half_np.setColor(1, 1, 1, 0.1)
        return half_np

    def _create_angle_arrow(
        self, parent: NodePath, length: float, head_width_factor: float
    ) -> NodePath:
        """创建显示末端角度的箭头。"""
        vdata = GeomVertexData("angle_arrow", GeomVertexFormat.getV3(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, "vertex")

        # 箭头沿 +X 方向，从原点出发，便于通过平面内旋转对齐切线
        head_w = max(length * head_width_factor, length * 0.02)
        vertex.addData3(0.0, 0.0, 0.0)
        vertex.addData3(length, 0.0, 0.0)
        vertex.addData3(length * 0.7, head_w, 0.0)
        vertex.addData3(length * 0.7, -head_w, 0.0)

        prim = GeomLines(Geom.UHStatic)
        prim.addVertices(0, 1)
        prim.addVertices(1, 2)
        prim.addVertices(1, 3)
        geom = Geom(vdata)
        geom.addPrimitive(prim)
        node = GeomNode("angle_arrow_geom")
        node.addGeom(geom)
        np = parent.attachNewNode(node)
        np.setTransparency(TransparencyAttrib.M_alpha)
        np.setTwoSided(True)
        return np

    def _build_sector_geom(self, start_rad: float, end_rad: float) -> Geom:
        """根据起止弧度创建扇形几何。"""
        radius = self.ring_radius
        span = end_rad - start_rad
        direction = 1.0 if span >= 0.0 else -1.0
        span = abs(span)
        segments = max(8, int(48 * (span / (2 * math.pi))))

        vdata = GeomVertexData("angle_sector_data", GeomVertexFormat.getV3(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, "vertex")

        # 中心点
        vertex.addData3(0.0, 0.0, 0.0)
        for i in range(segments + 1):
            a = start_rad + direction * (span * i / segments)
            x = radius * math.cos(a)
            y = radius * math.sin(a)
            vertex.addData3(x, y, 0.0)

        prim = GeomTriangles(Geom.UHStatic)
        for i in range(1, segments + 1):
            prim.addVertices(0, i, i + 1)
        geom = Geom(vdata)
        geom.addPrimitive(prim)

        return geom

    def _quat_from_z(self, direction: Vec3) -> Quat:
        """
        与 MoveGizmo 保持一致：
        返回一个四元数，使本地 +Z 旋转到指定 direction（自动归一化）。
        """
        dir_norm = Vec3(direction)
        if dir_norm.length_squared() == 0:
            return Quat.identQuat()
        dir_norm.normalize()
        z_axis = Vec3(0, 0, 1)
        dot = z_axis.dot(dir_norm)
        if abs(dot - 1.0) < 1e-6:
            return Quat.identQuat()  # 已经对齐
        if abs(dot + 1.0) < 1e-6:
            # 反向，绕任意垂直于 Z 的轴旋转 180°
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

    def _begin_angle_visual(self, axis_world: Vec3) -> None:
        """开始显示角度扇形，基于当前旋转轴和起始向量。"""
        if self.angle_disc_root is None:
            return
        if axis_world.length_squared() == 0:
            return

        align_q = self._quat_from_z(axis_world)
        self.angle_disc_root.setQuat(self.world.render, align_q)
        self.angle_disc_root.setPos(self.root, 0, 0, 0)

        # 计算起始向量在扇形平面上的角度，作为扇形起点
        if self.start_vec_on_plane.length_squared() > 0:
            q_inv = Quat(self.angle_disc_root.getQuat(self.world.render))
            q_inv.invertInPlace()
            local_vec = q_inv.xform(self.start_vec_on_plane)
            if local_vec.length_squared() == 0:
                local_vec = Vec3(1, 0, 0)
            local_vec.normalize()
            self._start_vec_local = Vec3(local_vec)
            self._angle_start_offset = math.atan2(local_vec.y, local_vec.x)

            # 箭头朝向：起始向量的切线方向（轴 x start_vec），并记录为局部平面角度
            axis_world_norm = Vec3(axis_world)
            axis_world_norm.normalize()
            tangent_world = axis_world_norm.cross(self.start_vec_on_plane)
            if tangent_world.length_squared() == 0:
                tangent_world = Vec3(-self.start_vec_on_plane.y, self.start_vec_on_plane.x, 0)
            tangent_world.normalize()
            tangent_local = q_inv.xform(tangent_world)
            self._arrow_tangent_angle_deg = math.degrees(
                math.atan2(tangent_local.y, tangent_local.x)
            )

            # 将箭头放在起始点位置，面向相机
            if self.angle_arrow_np:
                self.angle_arrow_np.setPos(
                    self.angle_disc_root,
                    Vec3(local_vec.x * self.ring_radius, local_vec.y * self.ring_radius, 0.0),
                )
                # 初始朝向即切线方向
                self.angle_arrow_np.setH(self._arrow_tangent_angle_deg)
        else:
            self._start_vec_local = Vec3(1, 0, 0)
            self._angle_start_offset = 0.0
            self._arrow_tangent_angle_deg = 90.0

        # 初始化扇形（零度）
        self._update_angle_sector(self._angle_start_offset, self._angle_start_offset)
        self.angle_disc_root.show()
        if self.angle_arrow_np:
            self.angle_arrow_np.hide()

    def _update_angle_visual(self, delta_deg: float) -> None:
        """根据当前旋转角度更新扇形与箭头。"""
        if self.angle_disc_root is None or self.angle_fill_np is None:
            return
        start_rad = self._angle_start_offset
        end_rad = start_rad + math.radians(delta_deg)
        self._update_angle_sector(start_rad, end_rad)
        if self.angle_arrow_np:
            arrow_angle = self._arrow_tangent_angle_deg
            if delta_deg < 0.0:
                arrow_angle += 180.0
            self.angle_arrow_np.setH(arrow_angle)
            if self.angle_arrow_np.isHidden():
                self.angle_arrow_np.show()

    def _update_angle_sector(self, start_rad: float, end_rad: float) -> None:
        """生成并替换当前的角度扇形几何。"""
        if self.angle_fill_np is None:
            return
        geom = self._build_sector_geom(start_rad, end_rad)
        node = cast(GeomNode, self.angle_fill_np.node())
        node.removeAllGeoms()
        node.addGeom(geom)

        self.angle_fill_np.setTwoSided(True)

    def _hide_angle_disc(self) -> None:
        """结束旋转时隐藏角度扇形。"""
        if self.angle_disc_root:
            self.angle_disc_root.hide()
        if self.angle_fill_np:
            cast(GeomNode, self.angle_fill_np.node()).removeAllGeoms()
        if self.angle_arrow_np:
            self.angle_arrow_np.hide()
        self._angle_start_offset = 0.0

    def _map_to_trackball(self, mpos: Point3) -> Vec3:
        """将屏幕坐标映射到轨迹球上的向量。"""
        x = mpos.x
        y = mpos.y
        length_sq = x * x + y * y
        if length_sq <= 1.0:
            z = math.sqrt(max(0.0, 1.0 - length_sq))
            v = Vec3(x, y, z)
        else:
            v = Vec3(x, y, 0.0)
            v.normalize()
        return v

    def _apply_trackball_rotation(self, mpos: Point3) -> None:
        """轨迹球旋转计算：基于屏幕向量的弧球算法。"""
        if self._trackball_start_vec is None or self.start_quat is None or self.target_node is None:
            return
        curr_vec = self._map_to_trackball(mpos)
        axis_cam = self._trackball_start_vec.cross(curr_vec)
        axis_len_sq = axis_cam.length_squared()
        if axis_len_sq < 1e-8:
            return
        axis_cam.normalize()
        dot = max(min(self._trackball_start_vec.dot(curr_vec), 1.0), -1.0)
        angle_rad = math.acos(dot)
        axis_world = self.camera.getQuat(self.world.render).xform(axis_cam)
        q_delta = Quat()
        q_delta.setFromAxisAngle(math.degrees(angle_rad), axis_world)
        # Panda3D 四元数从左向右组合：先应用起始姿态，再绕世界轴旋转。
        new_quat_world = self.start_quat * q_delta
        self.target_node.setQuat(self.world.render, new_quat_world)
        self._emit_event(
            GizmoEvent.DRAG_MOVE,
            {
                "gizmo": "rotate",
                "mode": "trackball",
                "axis": None,
                "target": self.target_node,
                "mouse": Point2(mpos.x, mpos.y),
                "delta_deg": math.degrees(angle_rad),
            },
        )

    # ------------------------------------------------------------------ #
    # 公共接口：绑定 / 解绑
    # ------------------------------------------------------------------ #
    def attach(self, node_path: NodePath) -> None:
        """
        绑定控制柄到指定 NodePath。
        is_local=True 时沿目标局部轴旋转，否则沿世界轴旋转。
        """
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
            f"target_hpr={node_path.getHpr(self.world.render)}"
        )
        # 根据当前相机位置初始化半圆环的正反显示
        self._update_ring_halves_visibility()
        self._register_events()

    def detach(self) -> None:
        """解绑并隐藏 Gizmo。"""
        self.target_node = None
        self.root.hide()
        self.attached = False
        self.dragging = False
        self.drag_axis = None
        self.drag_mode = None
        self._trackball_start_vec = None
        self._drag_root_quat_world = None
        self._last_drag_vec_on_plane = None
        self._last_screen_mouse = None
        self._last_screen_angle = None
        self._drag_accumulated_deg = 0.0
        self.is_hovering = False
        self._hide_angle_disc()
        self._reset_highlights()
        self._ignore_events()
        self._log("detached")

    def _update_ring_halves_visibility(self) -> None:
        """根据摄像机与 Gizmo 相对位置，切换每个轴的前/后半圆显示。"""
        if not self.camera:
            return

        cam_pos = self.camera.getPos(self.world.render)
        center = self.root.getPos(self.world.render)
        view_vec = cam_pos - center

        for axis_id, ring_root in enumerate(self.rings):
            if axis_id >= len(self.ring_halves):
                continue
            front_np, back_np, col_front_np, col_back_np = self.ring_halves[axis_id]
            if front_np.isEmpty() or back_np.isEmpty():
                continue

            # 圆环所在平面的法线（世界坐标）
            plane_normal = ring_root.getQuat(self.world.render).xform(Vec3(0, 0, 1))
            if plane_normal.length_squared() == 0:
                continue
            plane_normal.normalize()

            # 将摄像机方向投影到圆环平面内，确定“近半圆”切分线的朝向
            d = view_vec - plane_normal * view_vec.dot(plane_normal)
            if d.length_squared() == 0:
                # 摄像机几乎正对法线，保持显示近侧半圆
                front_np.show()
                back_np.hide()
                if not col_front_np.isEmpty():
                    col_front_np.node().setIntoCollideMask(BitMask32.bit(20))
                if not col_back_np.isEmpty():
                    col_back_np.node().setIntoCollideMask(BitMask32.allOff())
                front_np.setHpr(ring_root, 0, 0, 0)
                back_np.setHpr(ring_root, 0, 0, 0)
                continue

            q_world = ring_root.getQuat(self.world.render)
            q_inv = Quat(q_world)
            q_inv.invertInPlace()
            d_local = q_inv.xform(d)
            if d_local.length_squared() == 0:
                front_np.show()
                back_np.hide()
                if not col_front_np.isEmpty():
                    col_front_np.node().setIntoCollideMask(BitMask32.bit(20))
                if not col_back_np.isEmpty():
                    col_back_np.node().setIntoCollideMask(BitMask32.allOff())
                if not col_front_np.isEmpty():
                    col_front_np.setHpr(ring_root, 0, 0, 0)
                if not col_back_np.isEmpty():
                    col_back_np.setHpr(ring_root, 0, 0, 0)
                continue
            d_local.normalize()

            # 将半圆几何整体绕轴旋转，让切分线对齐相机投影方向
            angle = math.atan2(d_local.y, d_local.x)
            # 将近侧半圆的中心对准相机投影方向（边界垂直于视线投影）
            angle_deg = math.degrees(angle) - 90.0
            front_np.setHpr(ring_root, angle_deg, 0, 0)
            back_np.setHpr(ring_root, angle_deg, 0, 0)
            if not col_front_np.isEmpty():
                col_front_np.setHpr(ring_root, angle_deg, 0, 0)
            if not col_back_np.isEmpty():
                col_back_np.setHpr(ring_root, angle_deg, 0, 0)

            # 近侧半圆显示，远侧隐藏
            front_np.show()
            back_np.hide()
            if not col_front_np.isEmpty():
                col_front_np.node().setIntoCollideMask(BitMask32.bit(20))
            if not col_back_np.isEmpty():
                col_back_np.node().setIntoCollideMask(BitMask32.allOff())

    # ------------------------------------------------------------------ #
    # 撤销支持（可选使用）
    # ------------------------------------------------------------------ #

    def _record_rotate_action(
        self, node: NodePath, old_hpr: LVecBase3f, new_hpr: LVecBase3f
    ) -> None:
        """将一次旋转操作压入撤销栈，并在需要时通知外部监听者。"""
        if node is None or node.isEmpty():
            return
        if old_hpr is None or new_hpr is None:
            return

        delta = new_hpr - old_hpr
        if (
            abs(delta.x) < self._undo_angle_epsilon
            and abs(delta.y) < self._undo_angle_epsilon
            and abs(delta.z) < self._undo_angle_epsilon
        ):
            return

        old_copy = Vec3(old_hpr)
        new_copy = Vec3(new_hpr)
        # 组合控制器已有统一历史，避免重复保留目标节点。
        if self.on_action_committed is None:
            self._rotate_undo_stack.append((node, old_copy, new_copy))
        self._log(
            f"record rotate action: node={node.getName()} "
            f"old_hpr={old_copy} new_hpr={new_copy} stack_size={len(self._rotate_undo_stack)}"
        )

        # 把这次旋转操作上报给 TransformGizmo，用于全局撤销栈
        if self.on_action_committed is not None:
            try:
                self.on_action_committed(
                    {
                        "kind": "rotate",
                        "node": node,
                        "old_hpr": Vec3(old_copy),
                        "new_hpr": Vec3(new_copy),
                    }
                )
            except Exception as exc:
                self._log(f"on_action_committed(rotate) error: {exc}")

    def undo_last_rotate(self) -> None:
        """
        撤销最近一次旋转操作。
        可在 UI 中绑定快捷键调用。
        """
        if not self._rotate_undo_stack:
            self._log("undo_last_rotate: stack empty")
            return
        node, old_hpr, new_hpr = self._rotate_undo_stack.pop()
        if node is None or node.isEmpty():
            self._log("undo_last_rotate: target node invalid")
            return
        node.setHpr(self.world.render, old_hpr)
        self._log(
            f"undo_last_rotate: node={node.getName()} "
            f"old_hpr={old_hpr} new_hpr={new_hpr} remaining={len(self._rotate_undo_stack)}"
        )

    # ------------------------------------------------------------------ #
    # 事件注册 / 取消
    # ------------------------------------------------------------------ #
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

    # ------------------------------------------------------------------ #
    # 鼠标坐标工具
    # ------------------------------------------------------------------ #
    def _get_normalized_mouse(self, extra: MouseInput | None = None) -> Point3 | None:
        """获取标准化鼠标坐标，兼容 Panda3D 和外部 UI。"""
        return normalized_mouse(self.world, extra)

    @staticmethod
    def _wrap_angle_rad(angle_rad: float) -> float:
        """将弧度角包裹到 (-pi, pi]，避免跨越 180 度时跳变。"""
        while angle_rad <= -math.pi:
            angle_rad += math.tau
        while angle_rad > math.pi:
            angle_rad -= math.tau
        return angle_rad

    def _get_mouse_screen_angle(self, mpos: Point3) -> float | None:
        """返回鼠标相对 Gizmo 屏幕中心的极角。"""
        if self._center_screen is None:
            return None

        offset_x = mpos.x - self._center_screen.x
        offset_y = mpos.y - self._center_screen.y
        if offset_x * offset_x + offset_y * offset_y < 1e-10:
            return None
        return math.atan2(offset_y, offset_x)

    # ------------------------------------------------------------------ #
    # 屏幕空间旋转辅助方法
    # ------------------------------------------------------------------ #
    def _compute_screen_space_params(self, mpos: Point3) -> None:
        """
        计算屏幕空间旋转所需的参数：
        - Gizmo 中心在屏幕上的投影位置
        - 旋转轴在屏幕上的投影方向（2D向量）
        - 起始鼠标位置
        """
        if not self.camera:
            return

        lens = self.camera.node().getLens()

        # 将 Gizmo 中心投影到屏幕空间
        center_cam = self.camera.getRelativePoint(self.world.render, self.center_world)
        center_2d = Point2()
        if lens.project(center_cam, center_2d):
            self._center_screen = center_2d
        else:
            self._center_screen = Point2(0, 0)

        # 将旋转轴末端投影到屏幕空间，计算轴在屏幕上的方向
        axis_end_world = self.center_world + self.rotate_axis_world
        axis_end_cam = self.camera.getRelativePoint(self.world.render, axis_end_world)
        axis_end_2d = Point2()
        if lens.project(axis_end_cam, axis_end_2d):
            # 屏幕上的轴方向向量
            self._axis_screen_dir = Point2(
                axis_end_2d.x - self._center_screen.x, axis_end_2d.y - self._center_screen.y
            )
            # 归一化
            axis_len = math.sqrt(self._axis_screen_dir.x**2 + self._axis_screen_dir.y**2)
            if axis_len > 1e-6:
                self._axis_screen_dir = Point2(
                    self._axis_screen_dir.x / axis_len, self._axis_screen_dir.y / axis_len
                )
            else:
                self._axis_screen_dir = Point2(0, 1)
        else:
            self._axis_screen_dir = Point2(0, 1)

        # 记录起始鼠标位置
        self._start_mouse_screen = Point2(mpos.x, mpos.y)
        self._last_screen_mouse = Point2(mpos.x, mpos.y)

        # 旋转灵敏度（屏幕单位到角度的转换系数）
        self._screen_sensitivity = 180.0  # 移动1个屏幕单位 = 180度
        start_screen_angle = self._get_mouse_screen_angle(mpos)
        self._start_screen_angle = start_screen_angle or 0.0
        self._last_screen_angle = start_screen_angle
        self._axis_screen_sign = self._compute_screen_rotation_sign()

        self._log(
            f"screen params: center={self._center_screen} "
            f"axis_dir={self._axis_screen_dir} "
            f"start_mouse={self._start_mouse_screen} "
            f"start_angle={self._start_screen_angle:.4f} "
            f"sign={self._axis_screen_sign:.1f}"
        )

    def _compute_screen_rotation_sign(self) -> float:
        """计算屏幕极角增长方向与世界正旋转方向之间的映射符号。"""
        if (
            not self.camera
            or self._center_screen is None
            or self.start_vec_on_plane.length_squared() == 0
            or self.rotate_axis_world.length_squared() == 0
        ):
            return 1.0

        lens = self.camera.node().getLens()
        start_world = self.center_world + self.start_vec_on_plane * self.ring_radius
        positive_tangent = self.rotate_axis_world.cross(self.start_vec_on_plane)
        if positive_tangent.length_squared() == 0:
            return 1.0
        positive_tangent.normalize()

        sample_world = start_world + positive_tangent * (self.ring_radius * 0.15)
        start_cam = self.camera.getRelativePoint(self.world.render, start_world)
        sample_cam = self.camera.getRelativePoint(self.world.render, sample_world)

        start_2d = Point2()
        sample_2d = Point2()
        if not lens.project(start_cam, start_2d) or not lens.project(sample_cam, sample_2d):
            return 1.0

        center = self._center_screen
        start_angle = math.atan2(start_2d.y - center.y, start_2d.x - center.x)
        sample_angle = math.atan2(sample_2d.y - center.y, sample_2d.x - center.x)
        delta = self._wrap_angle_rad(sample_angle - start_angle)
        if abs(delta) < 1e-6:
            return 1.0
        return 1.0 if delta > 0.0 else -1.0

    def _calc_screen_space_angle(self, mpos: Point3) -> float:
        """
        使用屏幕空间累计旋转角度（度）。
        优先根据 Gizmo 中心的极角连续展开；投影退化时回退到切线位移。
        """
        current_angle = self._get_mouse_screen_angle(mpos)
        if current_angle is not None:
            if self._last_screen_angle is not None:
                step_rad = self._wrap_angle_rad(current_angle - self._last_screen_angle)
                step_deg = math.degrees(step_rad) * self._axis_screen_sign
                self._drag_accumulated_deg += step_deg
                self._log(
                    f"screen calc(polar): mouse=({mpos.x:.3f},{mpos.y:.3f}) "
                    f"step_deg={step_deg:.2f} total={self._drag_accumulated_deg:.2f}"
                )
            self._last_screen_angle = current_angle
            self._last_screen_mouse = Point2(mpos.x, mpos.y)
            return self._drag_accumulated_deg

        if self._axis_screen_dir is None or self._last_screen_mouse is None:
            self._log("screen params not ready")
            return self._drag_accumulated_deg

        move_x = mpos.x - self._last_screen_mouse.x
        move_y = mpos.y - self._last_screen_mouse.y
        perp_x = -self._axis_screen_dir.y
        perp_y = self._axis_screen_dir.x
        perp_component = move_x * perp_x + move_y * perp_y
        step_deg = -(perp_component * self._screen_sensitivity)
        self._drag_accumulated_deg += step_deg
        self._last_screen_mouse = Point2(mpos.x, mpos.y)

        self._log(
            f"screen calc(linear): mouse=({mpos.x:.3f},{mpos.y:.3f}) "
            f"move=({move_x:.3f},{move_y:.3f}) "
            f"step_deg={step_deg:.2f} total={self._drag_accumulated_deg:.2f}"
        )
        return self._drag_accumulated_deg

    def _calc_plane_space_angle(self, curr_vec: Vec3, axis_world: Vec3) -> float:
        """使用平面向量的相邻帧有符号夹角累计旋转量。"""
        prev_vec = self._last_drag_vec_on_plane
        curr = Vec3(curr_vec)
        if curr.length_squared() == 0:
            return self._drag_accumulated_deg
        curr.normalize()

        if prev_vec is None or prev_vec.length_squared() == 0:
            self._last_drag_vec_on_plane = Vec3(curr)
            return self._drag_accumulated_deg

        prev = Vec3(prev_vec)
        prev.normalize()
        dot = max(min(prev.dot(curr), 1.0), -1.0)
        cross = prev.cross(curr)
        step_rad = math.atan2(cross.dot(axis_world), dot)
        step_deg = math.degrees(step_rad)
        self._drag_accumulated_deg += step_deg
        self._last_drag_vec_on_plane = Vec3(curr)

        self._log(f"plane calc: step_deg={step_deg:.2f} total={self._drag_accumulated_deg:.2f}")
        return self._drag_accumulated_deg

    # ------------------------------------------------------------------ #
    # 鼠标事件：按下 / 弹起 / 移动
    # ------------------------------------------------------------------ #
    def _on_mouse_down(self, extra: MouseInput | None = None) -> None:
        if not self.attached or not self.camera or self.target_node is None:
            return

        mpos = self._get_normalized_mouse(extra)
        if mpos is None:
            self._log("mouse_down ignored: no mouse pos")
            return

        # 设置拾取射线
        self.picker_np.reparentTo(self.camera)
        if not self._picker_added:
            self.c_trav.addCollider(self.picker_np, self.c_queue)
            self._picker_added = True
        self.picker_ray.setFromLens(self.camera.node(), mpos.x, mpos.y)

        self.c_queue.clearEntries()
        self.c_trav.traverse(self.root)

        self._log(
            f"mouse_down mpos={mpos} cam_pos={self.camera.getPos(self.world.render)} "
            f"cam_hpr={self.camera.getHpr(self.world.render)} "
            f"entries={self.c_queue.getNumEntries()}"
        )

        num_entries = self.c_queue.getNumEntries()
        if num_entries <= 0:
            self._log("mouse_down: no collision entries on rotate gizmo")
            return

        self.c_queue.sortEntries()

        axis_entry: CollisionEntry | None = None
        trackball_entry: CollisionEntry | None = None
        picked_kind: str | None = None  # "axis" / "trackball"
        picked_axis_id: int | None = None
        for i in range(num_entries):
            e = self.c_queue.getEntry(i)
            axis_tag = e.getIntoNode().getTag("gizmo_axis")
            handle_tag = e.getIntoNode().getTag("gizmo_handle")
            if axis_tag:
                axis_entry = e
                picked_axis_id = int(axis_tag)
                break
            if handle_tag == "trackball" and trackball_entry is None:
                trackball_entry = e

        if axis_entry is not None:
            picked_kind = "axis"
        elif trackball_entry is not None:
            axis_entry = trackball_entry
            picked_kind = "trackball"

        if axis_entry is None or picked_kind is None:
            self._log("mouse_down: hit something but no gizmo handle tag")
            return

        self.dragging = True
        self.drag_mode = "axis" if picked_kind == "axis" else picked_kind
        self.drag_axis = picked_axis_id if self.drag_mode == "axis" else None

        self.center_world = self.root.getPos(self.world.render)
        self.start_quat = self.target_node.getQuat(self.world.render)
        self.start_quat_local = self.target_node.getQuat()
        self.start_mouse = Point2(mpos.x, mpos.y)
        self._trackball_start_vec = None
        self._drag_root_quat_world = self.root.getQuat(self.world.render)
        self._hide_angle_disc()

        # 轨迹球模式：屏幕空间自由旋转，不需要射线平面求交
        if self.drag_mode == "trackball":
            self._trackball_start_vec = self._map_to_trackball(mpos)
            self._highlight_axis("trackball")
            self._emit_event(
                GizmoEvent.DRAG_START,
                {
                    "gizmo": "rotate",
                    "mode": self.drag_mode,
                    "axis": self.drag_axis,
                    "target": self.target_node,
                    "mouse": Point2(mpos.x, mpos.y),
                    "start_quat": self.start_quat,
                },
            )
            return

        axis_dir = Vec3(0, 0, 1)
        # 以“可见圆环”的朝向为准，避免目标节点已有旋转时出现视觉/数学轴不一致
        if self.drag_mode == "axis" and self.drag_axis is not None:
            ring_np = self.rings[self.drag_axis]
            # 环所在平面法线：本地 +Z
            ring_axis_local = ring_np.getQuat().xform(Vec3(0, 0, 1))
            if ring_axis_local.length_squared() == 0:
                ring_axis_local = Vec3(0, 0, 1)
            ring_axis_local.normalize()
            self.drag_local_axis = Vec3(ring_axis_local)  # 目标自身坐标系中的轴

            if self.drag_axis == 0:
                self.start_root_axis_dir = Quat(self.root.getQuat()).getRight()
            elif self.drag_axis == 1:
                self.start_root_axis_dir = Quat(self.root.getQuat()).getForward()
            elif self.drag_axis == 2:
                self.start_root_axis_dir = Quat(self.root.getQuat()).getUp()

            # 使用圆环本身的朝向来定义平面法线（可见轴与数学轴一致）
            ring_np_world_quat = ring_np.getQuat(self.world.render)
            axis_dir = Vec3(ring_np_world_quat.xform(Vec3(0, 0, 1)))
        if axis_dir.length_squared() == 0:
            axis_dir = Vec3(0, 0, 1)
        axis_dir.normalize()
        self.rotate_axis_world = axis_dir

        # 计算当前鼠标射线与“旋转平面”（法线为当前轴，过 center_world）之间的交点
        lens: Lens = self.camera.node().getLens()
        p_from = Point3()
        p_to = Point3()
        hit_point: Point3 | None = None
        if lens.extrude(Point2(mpos.x, mpos.y), p_from, p_to):
            ray_origin = self.world.render.getRelativePoint(self.camera, p_from)
            ray_to = self.world.render.getRelativePoint(self.camera, p_to)
            ray_dir = ray_to - ray_origin
            if ray_dir.length_squared() != 0:
                ray_dir.normalize()
                n = self.rotate_axis_world
                denom = ray_dir.dot(n)
                if abs(denom) > 1e-6:
                    t = (self.center_world - ray_origin).dot(n) / denom
                    hit_point = ray_origin + ray_dir * t

        # 如果无法求交（几乎平行），回退到“摄像机投影向量”
        if hit_point is None:
            cam_pos = self.camera.getPos(self.world.render)
            v = cam_pos - self.center_world
            n = self.rotate_axis_world
            proj = v - n * v.dot(n)
            if proj.length_squared() == 0:
                proj = Vec3(1, 0, 0) if abs(n.x) < 0.9 else Vec3(0, 1, 0)
            hit_point = self.center_world + proj

        start_vec = hit_point - self.center_world
        if start_vec.length_squared() == 0:
            start_vec = Vec3(1, 0, 0)
        start_vec.normalize()
        self.start_vec_on_plane = start_vec
        self._last_drag_vec_on_plane = Vec3(start_vec)
        self._drag_accumulated_deg = 0.0
        self._last_screen_mouse = Point2(mpos.x, mpos.y)

        # 计算屏幕空间参数，用于边缘视角时的备用旋转计算
        self._compute_screen_space_params(mpos)

        # 初始化角度扇形
        self._begin_angle_visual(self.rotate_axis_world)

        self._highlight_axis(self.drag_axis if self.drag_mode == "axis" else self.drag_mode)
        if self.debug:
            t_quat_world = self.target_node.getQuat(self.world.render)
            t_quat_local = self.target_node.getQuat()
            self._log(
                " / ".join(
                    [
                        f"pick handle={self.drag_mode} axis={self.drag_axis} target={self.target_node.getName()}",
                        f"center={self.center_world}",
                        f"axis_world(from ring)={self.rotate_axis_world}",
                        f"axis_local(from ring)={self.drag_local_axis}",
                        f"start_vec={self.start_vec_on_plane}",
                        f"t_quat_world={t_quat_world}",
                        f"t_quat_local={t_quat_local}",
                    ]
                )
            )
        self._emit_event(
            GizmoEvent.DRAG_START,
            {
                "gizmo": "rotate",
                "mode": self.drag_mode,
                "axis": self.drag_axis,
                "target": self.target_node,
                "mouse": Point2(mpos.x, mpos.y),
                "start_quat": self.start_quat,
                "center": Point3(self.center_world),
                "axis_world": Vec3(self.rotate_axis_world),
            },
        )

    def _on_mouse_up(self, extra: MouseInput | None = None) -> None:
        self._update_ring_halves_visibility()
        if self.dragging and self.target_node is not None:
            try:
                final_hpr = self.target_node.getHpr(self.world.render)
            except Exception:
                final_hpr = None
            old_hpr = None
            if self.start_quat is not None:
                # 将起始四元数转换为 HPR（世界坐标系）
                tmp_np = NodePath("tmp_quat_to_hpr")
                tmp_np.reparentTo(self.world.render)
                tmp_np.setQuat(self.world.render, self.start_quat)
                old_hpr = tmp_np.getHpr(self.world.render)
                tmp_np.removeNode()

            if old_hpr is not None and final_hpr is not None:
                self._record_rotate_action(self.target_node, old_hpr, final_hpr)

            mode = self.drag_mode
            axis_id = self.drag_axis
            self._emit_event(
                GizmoEvent.DRAG_END,
                {
                    "gizmo": "rotate",
                    "mode": mode,
                    "axis": axis_id,
                    "target": self.target_node,
                    "start_quat": self.start_quat,
                    "final_hpr": final_hpr if "final_hpr" in locals() else None,
                },
            )
        self.dragging = False
        self.drag_axis = None
        self.drag_mode = None
        self._trackball_start_vec = None
        self._drag_root_quat_world = None
        self._last_drag_vec_on_plane = None
        self._last_screen_mouse = None
        self._last_screen_angle = None
        self._drag_accumulated_deg = 0.0
        self._hide_angle_disc()
        # 结束拖拽后恢复 Gizmo 与目标的对齐方式
        if self.is_local:
            self.root.setHpr(self.root.getParent(), 0, 0, 0)
        else:
            self.root.setHpr(self.world.render, 0, 0, 0)
        self._reset_highlights()
        self._log("mouse_up -> stop rotating")

    def _on_mouse_move(self, extra: MouseInput | None = None) -> None:
        if not self.attached or self.target_node is None:
            return

        mpos = self._get_normalized_mouse(extra)
        if mpos is None:
            if self.dragging:
                self._log("mouse_move ignored: no mouse pos while rotating")
            return

        # Hover highlight when idle (not dragging).
        if not self.dragging:
            self._update_ring_halves_visibility()
            self._update_hover_highlight(mpos)
            return

        if not self.target_node:
            return

        # 拖拽过程中持续更新半圆可见性与分割线朝向
        self._update_ring_halves_visibility()

        # 轨迹球模式：直接基于屏幕向量计算旋转
        if self.drag_mode == "trackball":
            self._apply_trackball_rotation(mpos)
            return

        lens: Lens = self.camera.node().getLens()
        p_from = Point3()
        p_to = Point3()
        if not lens.extrude(Point2(mpos.x, mpos.y), p_from, p_to):
            self._log("mouse_move: lens.extrude failed")
            return

        ray_origin = self.world.render.getRelativePoint(self.camera, p_from)
        ray_to = self.world.render.getRelativePoint(self.camera, p_to)
        ray_dir = ray_to - ray_origin
        if ray_dir.length_squared() == 0:
            return
        ray_dir.normalize()

        # 与旋转平面求交：平面法线为当前轴向，过 Gizmo 中心
        axis_world = Vec3(self.rotate_axis_world)
        if axis_world.length_squared() == 0:
            return
        axis_world.normalize()

        denom = ray_dir.dot(axis_world)

        curr_vec: Vec3 | None = None
        if abs(denom) > 1e-6:
            t = (self.center_world - ray_origin).dot(axis_world) / denom
            hit_point = ray_origin + ray_dir * t
            curr_vec = hit_point - self.center_world
            if curr_vec.length_squared() != 0:
                curr_vec.normalize()
            else:
                curr_vec = None

        # 判断是否处于边缘视角（grazing angle）
        use_screen_space = abs(denom) < self._grazing_threshold or curr_vec is None

        if use_screen_space or curr_vec is None:
            delta_deg = self._calc_screen_space_angle(mpos)
            if curr_vec is not None:
                self._last_drag_vec_on_plane = Vec3(curr_vec)
            if self.debug:
                self._log(
                    f"grazing angle detected: denom={denom:.4f}, "
                    f"using screen space, delta_deg={delta_deg:.2f}"
                )
        else:
            delta_deg = self._calc_plane_space_angle(curr_vec, axis_world)
            screen_angle = self._get_mouse_screen_angle(mpos)
            if screen_angle is not None:
                self._last_screen_angle = screen_angle
            self._last_screen_mouse = Point2(mpos.x, mpos.y)

        # 轴向旋转根据 is_local 切换本地/世界应用方式
        if self.drag_mode == "axis" and self.is_local:
            # 本地模式：先绕目标自身轴旋转，再应用起始的父坐标系姿态。
            if self.drag_local_axis is None or self.start_quat_local is None:
                return
            q_delta = Quat()
            q_delta.setFromAxisAngle(delta_deg, self.drag_local_axis)
            if self.debug:
                self._log(
                    f"[RotateGizmo] local q_delta={q_delta} delta_deg={delta_deg} axis_world={axis_world} axis_local={self.drag_local_axis}"
                )
            new_quat_local = q_delta * self.start_quat_local
            self.target_node.setQuat(new_quat_local)
        else:
            # 世界模式：绕固定的世界轴旋转，直接在世界坐标系应用。
            q_delta = Quat()
            axis_for_world = self.start_root_axis_dir or axis_world
            if axis_for_world.length_squared() == 0:
                axis_for_world = axis_world
            else:
                axis_for_world.normalize()
            q_delta.setFromAxisAngle(delta_deg, axis_for_world)
            if self.debug:
                self._log(
                    f"[RotateGizmo] world q_delta={q_delta} delta_deg={delta_deg} axis_world={axis_world}"
                )
            if self.start_quat is None:
                return
            new_quat_world = self.start_quat * q_delta
            self.target_node.setQuat(self.world.render, new_quat_world)

        # 更新角度扇形可视化
        self._update_angle_visual(delta_deg)
        # 拖拽时锁定 Gizmo 的世界朝向，避免扇形跟随目标旋转
        if self._drag_root_quat_world is not None:
            self.root.setQuat(self.world.render, self._drag_root_quat_world)

        self._emit_event(
            GizmoEvent.DRAG_MOVE,
            {
                "gizmo": "rotate",
                "mode": self.drag_mode,
                "axis": self.drag_axis,
                "target": self.target_node,
                "mouse": Point2(mpos.x, mpos.y),
                "delta_deg": delta_deg,
            },
        )

        if self.debug:
            if use_screen_space or curr_vec is None:
                self._log(
                    f"rotate handle={self.drag_mode} angle={delta_deg:.2f} center={self.center_world} n={axis_world} (screen space)"
                )
            else:
                self._log(
                    f"rotate handle={self.drag_mode} axis={self.drag_axis} angle={delta_deg:.2f} "
                    f"center={self.center_world} n={axis_world} "
                    f"start_vec={self.start_vec_on_plane} curr_vec={curr_vec}"
                )

    def _update_hover_highlight(self, mpos: Point3) -> None:
        """Highlight the hovered rotate handle without starting a drag."""
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

        axis_entry: CollisionEntry | None = None
        trackball_entry: CollisionEntry | None = None
        for i in range(num_entries):
            e = self.c_queue.getEntry(i)
            axis_tag = e.getIntoNode().getTag("gizmo_axis")
            handle_tag = e.getIntoNode().getTag("gizmo_handle")
            if axis_tag:
                axis_entry = e
                break
            if handle_tag == "trackball" and trackball_entry is None:
                trackball_entry = e

        if axis_entry is not None:
            self._highlight_axis(int(axis_entry.getIntoNode().getTag("gizmo_axis")))
            self.is_hovering = True
        elif trackball_entry is not None:
            self._highlight_axis("trackball")
            self.is_hovering = True
        else:
            self._reset_highlights()
            self.is_hovering = False

    # ------------------------------------------------------------------ #
    # 高亮与更新
    # ------------------------------------------------------------------ #
    def _highlight_axis(self, axis_id: int | str | None) -> None:
        """高亮指定的旋转手柄（轴 / 轨迹球）。"""
        for i, ring in enumerate(self.rings):
            if i == axis_id:
                ring.setColorScale(2, 2, 2, 1.0)
                ring.setAlphaScale(1.0)
            else:
                ring.clearColorScale()
                ring.setAlphaScale(0.3)

        for idx, axis_np in enumerate(self.center_axes):
            if axis_id == idx:
                axis_np.setColorScale(1.5, 1.5, 1.5, 1.0)
            else:
                axis_np.clearColorScale()

        if self.trackball_np:
            geom_np = self.trackball_np.find("**/trackball_geom")
            if not geom_np.isEmpty():
                if axis_id == "trackball":
                    geom_np.show()
                    geom_np.setColorScale(
                        self.highlight_color.x, self.highlight_color.y, self.highlight_color.z, 0.4
                    )
                    if self.trackball_edge:
                        self.trackball_edge.setColorScale(
                            self.highlight_color.x,
                            self.highlight_color.y,
                            self.highlight_color.z,
                            0.6,
                        )
                else:
                    geom_np.hide()
                    if self.trackball_edge:
                        self.trackball_edge.setColorScale(self.trackball_edge_color)

    def _reset_highlights(self) -> None:
        for ring in self.rings:
            ring.clearColorScale()
            ring.setAlphaScale(self.ring_alpha)
        if self.trackball_edge:
            self.trackball_edge.setColorScale(self.trackball_edge_color)
        for axis_np in self.center_axes:
            axis_np.clearColorScale()

        if self.trackball_np:
            geom_np = self.trackball_np.find("**/trackball_geom")
            if not geom_np.isEmpty():
                geom_np.hide()

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
            # 持续调整半圆的可见性与分割线朝向
            self._update_ring_halves_visibility()
            self.root.setScale(self.world.render, screen_scale(self))

            # print(f"target_node_hpr -> {self.target_node.getHpr()}")

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
        # 拖拽过程中保持根节点朝向锁定，避免 gizmo 轴随目标旋转而跳动
        if self.dragging and self._drag_root_quat_world is not None:
            self.root.setQuat(render, self._drag_root_quat_world)
        else:
            if self.is_local:
                self.root.setQuat(render, tgt.getQuat(render))
            else:
                self.root.setQuat(render, Quat.identQuat())

    def destroy(self) -> None:
        """释放控制柄及输入资源。"""
        destroy_gizmo(self)


__all__ = ["RotateGizmo"]
