"""移动控制柄：程序化几何、碰撞拾取与拖拽计算。"""

from __future__ import annotations

import math

from direct.showbase.DirectObject import DirectObject
from direct.showbase.ShowBase import ShowBase
from direct.task.Task import Task
from panda3d.core import (
    BitMask32,
    CardMaker,
    CollisionEntry,
    CollisionHandlerQueue,
    CollisionNode,
    CollisionPolygon,
    CollisionRay,
    CollisionSphere,
    CollisionTraverser,
    CollisionTube,
    Geom,
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


class MoveGizmo(DirectObject):
    """
    A Unity-like Move Gizmo for Panda3D.
    Features:
    - 3 Axes (X=Red, Y=Green, Z=Blue) with Cylinder + Cone.
    - Renders on top of other geometry.
    - Handles mouse interaction for dragging.
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
        self.debug = False
        self._debug_drag_counter = 0
        self._picker_added = False
        self.is_hovering = False
        # Optional callback to report completed move actions to a higher-level
        # manager (e.g. TransformGizmo) so it can build a global undo stack.
        self.on_action_committed = on_action_committed
        # Event hooks: drag_start / drag_move / drag_end
        self._event_hooks = self._normalize_event_hooks(event_hooks)
        self.camera = camera_path(world, camera_np)
        if not self.camera:
            self._log("No camera found")
        else:
            self._log(f"Camera found: {self.camera}")

        self.root = NodePath("MoveGizmo")
        self.color_higher = 0.05
        self.color_normal = 0.01

        self.target_node: NodePath | None = None
        self.attached = False

        # 控制柄外观设置
        self.arrow_cylider_length = 0.45  # 圆柱长度
        self.arrow_cylider_radius = 0.005  # 圆柱粗细
        # 碰撞体半径，可独立于可视半径调整
        self.arrow_cylider_col_radius = max(0.02, self.arrow_cylider_radius * 5.0)
        self.arrow_cone_height = 0.1  # 圆锥高度
        self.arrow_cone_radius = 0.03  # 圆锥半径
        self.arrow_transparency: float = 0.8  # 箭头透明度

        self.panel_transparency: float = 0.65
        self.plane_size: float = 0.1
        self.plane_offset: float = self.plane_size * 0.5 + 0.01

        # Collision setup for picking
        self.picker_ray = CollisionRay()
        self.picker_node = CollisionNode("gizmo_picker_ray")
        self.picker_node.addSolid(self.picker_ray)
        self.picker_node.setFromCollideMask(BitMask32.bit(20))  # Specific mask for gizmo
        self.picker_node.setIntoCollideMask(BitMask32.allOff())
        self.picker_np = NodePath(self.picker_node)

        self.c_trav = CollisionTraverser()
        self.c_queue = CollisionHandlerQueue()

        # Dragging state
        self.dragging: bool = False
        self.drag_axis: int | None = None  # 0=X, 1=Y, 2=Z
        self.start_mouse: Point2 | None = None  # NDC Point2
        self.start_node_pos: Point3 | None = None
        # param value of closest point on axis at drag start
        self._start_param_on_ray: float = 0.0
        # sign so that dragging along axis arrow matches mouse direction
        self._axis_param_sign: float = 1.0
        self.last_mouse_pos: Point2 | None = None
        self.drag_axis_dir: Vec3 = Vec3(0, 0, 0)  # world-space axis dir
        self.drag_axis_screen: Point2 = Point2(1, 0)  # axis direction in screen space
        self.drag_base_dist: float = 1.0  # camera distance at drag start

        # Visuals
        self.axes: list[NodePath] = []
        self.planes: list[NodePath] = []  # plane handles: 0=XY,1=YZ,2=ZX
        self.axes_base_colors: list[tuple[NodePath, Vec4]] = []
        self.planes_base_colors: list[tuple[NodePath, Vec4]] = []
        # Plane drag state
        self.drag_plane_id: int | None = None  # 0=XY,1=YZ,2=ZX
        self.drag_plane_normal: Vec3 = Vec3(0, 0, 0)
        self.drag_plane_origin: Point3 = Point3(0, 0, 0)  # point on plane at drag start
        # target_pos - hit_point at drag start
        self._plane_drag_offset: Vec3 = Vec3(0, 0, 0)

        # --- Undo stack for move operations ---------------------------------
        # Each entry: (target_node, old_pos(world), new_pos(world))
        self._move_undo_stack: list[tuple[NodePath, Vec3, Vec3]] = []
        # Minimal distance to consider as a real move (avoid float noise)
        self._undo_pos_epsilon: float = 1e-4

        self._build_gizmo()

        # 初始挂在 world.render，后续通过更新同步到目标
        self.root.reparentTo(self.world.render)
        self.root.setBin("fixed", 40)
        self.root.setDepthTest(False)
        self.root.setDepthWrite(False)
        self.root.setLightOff()

        self.root.hide()

        # Update gizmo scale every frame so that it
        # stays roughly the same size on screen
        # taskMgr.add(self._update_task, "MoveGizmoUpdateTask")

    def _log(self, msg: str) -> None:
        if self.debug:
            print(f"[MoveGizmo] {msg}")

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
        """Builds the 3 axes geometry and plane handles."""
        # Axis params

        # X Axis (Red)
        self.axes.append(
            self._create_axis(
                Vec3(1, 0, 0),
                Vec4(1, 0, 0, 1),
                self.arrow_cylider_length,
                self.arrow_cylider_radius,
                self.arrow_cylider_col_radius,
                self.arrow_cone_height,
                self.arrow_cone_radius,
                0,
            )
        )
        # Y Axis (Green)
        self.axes.append(
            self._create_axis(
                Vec3(0, 1, 0),
                Vec4(0, 1, 0, 1),
                self.arrow_cylider_length,
                self.arrow_cylider_radius,
                self.arrow_cylider_col_radius,
                self.arrow_cone_height,
                self.arrow_cone_radius,
                1,
            )
        )
        # Z Axis (Blue)
        self.axes.append(
            self._create_axis(
                Vec3(0, 0, 1),
                Vec4(0, 0, 1, 1),
                self.arrow_cylider_length,
                self.arrow_cylider_radius,
                self.arrow_cylider_col_radius,
                self.arrow_cone_height,
                self.arrow_cone_radius,
                2,
            )
        )

        # Plane handles: small squares between axes, slightly offset from origin.
        # 0 = XY, 1 = YZ, 2 = ZX
        half = self.plane_size * 0.5
        inner = self.plane_offset + half
        # XY plane：贴近 +X/+Y 象限
        self.planes.append(
            self._create_plane_handle(
                name="plane_xy",
                plane_id=0,
                axis_a=0,
                axis_b=1,
                color=Vec4(1, 1, 0, self.panel_transparency),  # 黄色
                size=self.plane_size,
                # 以平面中心为轴心，让靠近原点的角距离原点 plane_offset
                offset_vec=Vec3(inner, inner, 0.0),
                hpr=Vec3(0, -90, 0),
            )
        )
        # YZ plane (normal +X)
        self.planes.append(
            self._create_plane_handle(
                name="plane_yz",
                plane_id=1,
                axis_a=1,
                axis_b=2,
                color=Vec4(0, 1, 1, self.panel_transparency),  # 青色
                size=self.plane_size,
                offset_vec=Vec3(0.0, inner, inner),
                hpr=Vec3(90, 0, 0),  # rotate XY card to YZ
            )
        )
        # ZX plane (normal +Y)
        self.planes.append(
            self._create_plane_handle(
                name="plane_zx",
                plane_id=2,
                axis_a=2,
                axis_b=0,
                color=Vec4(0.5, 0, 0, self.panel_transparency),  # 深红
                size=self.plane_size,
                offset_vec=Vec3(inner, 0.0, inner),
                # Rotate XY card so it lies in the ZX plane (normal ±Y)
                hpr=Vec3(0, 0, 0),
            )
        )

    def _create_axis(
        self,
        direction: Vec3,
        color: Vec4,
        length: float,
        radius_visual: float,
        radius_collision: float,
        cone_height: float,
        cone_radius: float,
        axis_id: int,
    ) -> NodePath:
        """Creates a single axis arrow."""
        axis_root: NodePath = self.root.attachNewNode(f"axis_{axis_id}")

        # Align axis (default geometry faces +Z) to the requested direction.
        align_quat = self._quat_from_z(direction)
        axis_root.setQuat(align_quat)
        self._log(
            f"axis {axis_id} dir={direction} quat={align_quat} "
            f"world_dir={axis_root.getQuat(self.world.render).xform(Vec3(0, 0, 1))}"
        )

        # Visual Geometry
        geom = self._create_arrow_geom(length, radius_visual, cone_height, cone_radius, color)
        geom_np: NodePath = axis_root.attachNewNode(geom)
        geom_np.setTransparency(TransparencyAttrib.MAlpha)
        geom_np.setAlphaScale(self.arrow_transparency)
        self._log(f"created geom_np -> {geom_np}")

        # Collision Geometry (Tube)
        # Tube is defined by point A and B.
        shaft_len = max(length - cone_height, 0.0)
        shaft_radius = max(1e-6, radius_collision)
        c_node = CollisionNode(f"axis_col_{axis_id}")
        # Shaft collider
        if shaft_len > 0.0:
            c_node.addSolid(CollisionTube(Point3(0, 0, 0), Point3(0, 0, shaft_len), shaft_radius))
        # Cone collider: a slightly wider tube plus a small sphere at the tip
        if cone_height > 0.0:
            cone_start = Point3(0, 0, shaft_len)
            cone_tip = Point3(0, 0, length)
            cone_radius_pad = max(cone_radius, shaft_radius)
            c_node.addSolid(CollisionTube(cone_start, cone_tip, cone_radius_pad))
            c_node.addSolid(CollisionSphere(cone_tip, cone_radius_pad))
        c_node.setIntoCollideMask(BitMask32.bit(20))
        c_node.setFromCollideMask(BitMask32.allOff())
        c_node.setTag("gizmo_axis", str(axis_id))

        col_np = axis_root.attachNewNode(c_node)
        # col_np.show()  # Debug collision
        self._log(f"created col_np -> {col_np}")

        return axis_root

    def _create_plane_handle(
        self,
        name: str,
        plane_id: int,
        axis_a: int,
        axis_b: int,
        color: Vec4,
        size: float,
        offset_vec: Vec3,
        hpr: Vec3 = Vec3(0, 0, 0),
    ) -> NodePath:
        """
        Create a small square plane handle between two axes.
        plane_id: 0=XY, 1=YZ, 2=ZX
        axis_a / axis_b: not currently used for math, kept for clarity.
        """
        cm = CardMaker(name)
        # Square centered at origin in local X/Z (pivot at center)
        half = size * 0.5
        cm.setFrame(-half, half, -half, half)
        card_np: NodePath = self.root.attachNewNode(cm.generate())
        card_np.setPos(offset_vec)
        card_np.setHpr(hpr)
        card_np.setColor(color)
        card_np.setTransparency(TransparencyAttrib.MAlpha)
        # Double-sided rendering so the plane looks like Unity handles
        card_np.setTwoSided(True)

        # Collision polygon matching the card in local space
        # Define square in the same local coordinates as the CardMaker frame.
        p0 = Point3(-half, 0.0, -half)
        p1 = Point3(half, 0.0, -half)
        p2 = Point3(half, 0.0, half)
        p3 = Point3(-half, 0.0, half)
        # Create two polygons with opposite winding so the plane is hit
        # from both sides by the picking ray.
        poly_front = CollisionPolygon(p0, p1, p2, p3)
        poly_back = CollisionPolygon(p3, p2, p1, p0)

        c_node = CollisionNode(f"plane_col_{plane_id}")
        c_node.addSolid(poly_front)
        c_node.addSolid(poly_back)
        c_node.setIntoCollideMask(BitMask32.bit(20))
        c_node.setFromCollideMask(BitMask32.allOff())
        c_node.setTag("gizmo_plane", str(plane_id))

        col_np = card_np.attachNewNode(c_node)
        # col_np.show()
        self._log(f"created plane {name} col_np -> {col_np}")

        return card_np

    def _create_arrow_geom(
        self, length: float, radius: float, cone_height: float, cone_radius: float, color: Vec4
    ) -> GeomNode:
        """Creates the mesh for cylinder + cone."""
        # Using simple line or loading model is easier, but let's try procedural for "no external assets" requirement
        # Actually, Panda has `loader.loadModel("models/misc/cylinder")` but let's make a simple one or use LineSegs for shaft?
        # User requested "Cylinder + Cone".

        # Let's use a helper function to generate a cylinder and cone using GeomVertexWriter
        # For simplicity in this script, I will use a very thin box or simple geometry.
        # Or better, load a basic shape if available, but procedural is safer to avoid missing assets.

        # Simplified: Use Line for shaft (thick) and Triangle fan for cone?
        # No, user wants "Cylinder".

        # Let's use CardMaker for a cross-section look or just build a simple mesh.
        # To save code space, I'll implement a basic procedural cylinder/cone generator.

        vdata = GeomVertexData("name", GeomVertexFormat.getV3c4(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, "vertex")
        vcolor = GeomVertexWriter(vdata, "color")

        # Shaft (Cylinder) - Simplified as a prism with 8 sides
        segments = 12
        shaft_len = length - cone_height

        def add_circle_verts(z: float, r: float) -> int:
            start_idx = vertex.getWriteRow()
            for i in range(segments):
                angle = 2 * math.pi * i / segments
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                vertex.addData3(x, y, z)
                vcolor.addData4(color)
            return start_idx

        # Bottom circle
        base_idx = add_circle_verts(0, radius)
        # Top of shaft
        top_shaft_idx = add_circle_verts(shaft_len, radius)

        prim = GeomTriangles(Geom.UHStatic)
        for i in range(segments):
            next_i = (i + 1) % segments
            # Side quads (2 tris)
            prim.addVertices(base_idx + i, base_idx + next_i, top_shaft_idx + next_i)
            prim.addVertices(base_idx + i, top_shaft_idx + next_i, top_shaft_idx + i)

        # Cone base
        cone_base_idx = add_circle_verts(shaft_len, cone_radius)
        # Cone tip
        vertex.addData3(0, 0, length)
        vcolor.addData4(color)
        tip_idx = vertex.getWriteRow() - 1

        # Cone sides
        for i in range(segments):
            next_i = (i + 1) % segments
            prim.addVertices(cone_base_idx + i, cone_base_idx + next_i, tip_idx)
            # Cone bottom cap (optional, usually hidden by shaft)
            prim.addVertices(
                cone_base_idx + next_i, cone_base_idx + i, top_shaft_idx + i
            )  # Stitching?

        geom = Geom(vdata)
        geom.addPrimitive(prim)
        node = GeomNode("axis_geom")
        node.addGeom(geom)
        return node

    def _quat_from_z(self, direction: Vec3) -> Quat:
        """Return a quaternion rotating +Z to the given (normalized) direction."""
        dir_norm = Vec3(direction)
        if dir_norm.length_squared() == 0:
            return Quat.identQuat()
        dir_norm.normalize()
        z_axis = Vec3(0, 0, 1)
        dot = z_axis.dot(dir_norm)
        if abs(dot - 1.0) < 1e-6:
            return Quat.identQuat()  # Already aligned
        if abs(dot + 1.0) < 1e-6:
            # Opposite; rotate 180° around any axis perpendicular to Z
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
        """Attach gizmo to a node."""
        if not node_path:
            return
        self.target_node = node_path
        self.attached = True
        self.last_mouse_pos: Point2 | None = None
        self._update_root_transform()
        self.root.show()
        self._log(
            f"attached to {node_path.getName()} world_pos={self.root.getPos(self.world.render)} "
            f"target_hpr={node_path.getHpr(self.world.render)}"
        )
        self._register_events()
        self._reset_planels()

    def detach(self) -> None:
        """Detach gizmo."""
        self.target_node = None
        self.root.hide()
        self.attached = False
        self.dragging = False
        self.drag_axis = None
        self.drag_plane_id = None
        self.is_hovering = False
        self._reset_highlights()
        self._log("detached")
        self._ignore_events()

    # --- Undo support -------------------------------------------------
    def _record_move_action(self, node: NodePath, old_pos: LVecBase3f, new_pos: LVecBase3f) -> None:
        """
        Push a move operation to the undo stack if the position changed
        by more than a small epsilon.
        """
        if not node or node.isEmpty():
            return
        if not old_pos or new_pos is None:
            return
        delta = new_pos - old_pos
        if delta.length() < self._undo_pos_epsilon:
            # Ignore tiny moves caused by float noise
            return
        # Store copies so later modifications don't affect history
        old_copy = Vec3(old_pos)
        new_copy = Vec3(new_pos)
        # 组合控制器已有统一历史，避免重复保留目标节点。
        if self.on_action_committed is None:
            self._move_undo_stack.append((node, old_copy, new_copy))
        self._log(
            f"record move action: node={node.getName()} "
            f"old_pos={old_copy} new_pos={new_copy} stack_size={len(self._move_undo_stack)}"
        )
        # Report to external listener (e.g. TransformGizmo) to build a
        # unified, cross-gizmo undo history.
        if self.on_action_committed is not None:
            try:
                self.on_action_committed(
                    {
                        "kind": "move",
                        "node": node,
                        "old_pos": Vec3(old_copy),
                        "new_pos": Vec3(new_copy),
                    }
                )
            except Exception as exc:
                # Avoid breaking interaction if user callback fails.
                self._log(f"on_action_committed(move) error: {exc}")

    def undo_last_move(self) -> None:
        """
        Undo the last committed move.
        Default hotkey (from Qt -> Panda3D translation): Ctrl+Z => 'control-z'.
        """
        if not self._move_undo_stack:
            self._log("undo_last_move: stack empty")
            return
        node, old_pos, new_pos = self._move_undo_stack.pop()
        if node is None or node.isEmpty():
            self._log("undo_last_move: target node invalid")
            return
        node.setPos(self.world.render, old_pos)
        self._log(
            f"undo_last_move: node={node.getName()} "
            f"old_pos={old_pos} new_pos={new_pos} remaining={len(self._move_undo_stack)}"
        )

    def _register_events(self) -> None:
        self.accept("mouse1", self._on_mouse_down)
        self.accept("mouse1-up", self._on_mouse_up)
        self.accept("mouse2-up", self._on_mouse_up)
        self.accept("mouse3-up", self._on_mouse_up)
        self.accept("mouse-move", self._on_mouse_move)
        self._log("event listeners registered (mouse)")

    def _ignore_events(self) -> None:
        self.ignore("mouse1")
        self.ignore("mouse1-up")
        self.ignore("mouse2-up")
        self.ignore("mouse3-up")
        self.ignore("mouse-move")
        self._log("event listeners removed")

    # --- Mouse helpers -------------------------------------------------
    def _get_normalized_mouse(self, extra: MouseInput | None = None) -> Point3 | None:
        """获取标准化鼠标坐标，兼容 Panda3D 和外部 UI。"""
        return normalized_mouse(self.world, extra)

    def _on_mouse_down(self, extra: MouseInput | None = None) -> None:
        if not self.attached or self.target_node is None:
            return

        # Check for picking
        mpos = self._get_normalized_mouse(extra)

        if mpos is None:
            self._log("mouse_down ignored: no mouse pos")
            return

        # Setup ray
        self.picker_np.reparentTo(self.camera)
        if not self._picker_added:
            self.c_trav.addCollider(self.picker_np, self.c_queue)
            self._picker_added = True
        self.picker_ray.setFromLens(self.camera.node(), mpos.x, mpos.y)

        self.c_queue.clearEntries()
        self.c_trav.traverse(self.root)

        self._log(
            f"mouse_down mpos={mpos} cam_pos={self.camera.getPos(self.world.render)} "
            f"cam_hpr={self.camera.getHpr(self.world.render)} entries={self.c_queue.getNumEntries()}"
        )

        num_entries = self.c_queue.getNumEntries()
        if num_entries > 0:
            self.c_queue.sortEntries()

            # Unity 风格：优先选择平面，其次才是轴。
            plane_entry: CollisionEntry | None = None
            axis_entry: CollisionEntry | None = None
            for i in range(num_entries):
                e: CollisionEntry = self.c_queue.getEntry(i)
                if not plane_entry and e.getIntoNode().getTag("gizmo_plane"):
                    plane_entry = e
                if not axis_entry and e.getIntoNode().getTag("gizmo_axis"):
                    axis_entry = e
                if plane_entry and axis_entry:
                    break

            # Prefer plane hit when available
            if plane_entry is not None:
                entry = plane_entry
                plane_tag: str = entry.getIntoNode().getTag("gizmo_plane")
                # --- Begin plane dragging ---
                # --- Begin plane dragging ---
                self.dragging = True
                self.drag_plane_id = int(plane_tag)  # 0=XY,1=YZ,2=ZX
                # print(f"[Gizmo] drag_plane_id={self.drag_plane_id}")
                self.drag_axis = None

                cam_pos = self.camera.getPos(self.world.render)
                node_pos_w = self.target_node.getPos(self.world.render)
                lens: Lens = self.camera.node().getLens()

                # Determine plane normal in world space from the actual plane node,
                # so that dragging follows the visual orientation even when the
                # target node is rotated (local mode).
                plane_np = self.planes[self.drag_plane_id]
                plane_normal = Vec3(Quat(plane_np.getQuat(self.world.render)).xform(Vec3(0, 1, 0)))
                plane_normal.normalize()
                self.drag_plane_normal = plane_normal
                self.drag_plane_origin = node_pos_w
                self.start_node_pos = node_pos_w
                self.start_mouse = Point2(mpos.x, mpos.y)

                # Build initial mouse ray and compute intersection with plane
                p_from = Point3()
                p_to = Point3()
                hit_point = None
                if lens.extrude(Point2(mpos.x, mpos.y), p_from, p_to):
                    ray_origin = self.world.render.getRelativePoint(self.camera, p_from)
                    ray_to = self.world.render.getRelativePoint(self.camera, p_to)
                    ray_dir = ray_to - ray_origin
                    if ray_dir.length_squared() != 0:
                        ray_dir.normalize()
                        denom = ray_dir.dot(plane_normal)
                        if abs(denom) > 1e-6:
                            t = (self.drag_plane_origin - ray_origin).dot(plane_normal) / denom
                            hit_point = ray_origin + ray_dir * t

                if hit_point is None:
                    # Fallback: use current node position as hit point
                    hit_point = node_pos_w

                self._plane_drag_offset = node_pos_w - hit_point
                self._debug_drag_counter = 0
                self._highlight_plane(self.drag_plane_id)
                self._log(
                    f"pick plane={self.drag_plane_id} target={self.target_node.getName()} "
                    f"plane_origin={self.drag_plane_origin} plane_normal={self.drag_plane_normal} "
                    f"hit_point={hit_point}"
                )
                self._emit_event(
                    GizmoEvent.DRAG_START,
                    {
                        "gizmo": "move",
                        "target": self.target_node,
                        "axis": None,
                        "plane": self.drag_plane_id,
                        "mouse": Point2(mpos.x, mpos.y),
                        "start_pos": self.start_node_pos,
                    },
                )

            elif axis_entry is not None:
                entry = axis_entry
                axis_tag: str = entry.getIntoNode().getTag("gizmo_axis")
                # --- Begin axis dragging (existing behavior) ---
                self.dragging = True
                self.drag_axis = int(axis_tag)
                self.drag_plane_id = None

                # --- Freeze axis direction & screen-space mapping at drag start ---
                axis_index = int(axis_tag)
                axis_np: NodePath = self.axes[axis_index]
                axis_dir = Vec3(axis_np.getQuat(self.world.render).xform(Vec3(0, 0, 1)))
                if axis_dir.length_squared() == 0:
                    axis_dir = Vec3(0, 0, 1)
                axis_dir.normalize()
                cam_pos = self.camera.getPos(self.world.render)
                node_pos_w = self.target_node.getPos(self.world.render)
                view_dir = node_pos_w - cam_pos
                if view_dir.length_squared() != 0:
                    view_dir.normalize()

                self.drag_axis_dir = Vec3(axis_dir)

                lens: Lens = self.camera.node().getLens()
                self.drag_base_dist = (cam_pos - node_pos_w).length()
                sample_len = max(self.drag_base_dist * 0.25, 0.25)

                p0_cam = self.camera.getRelativePoint(self.world.render, node_pos_w)
                p1_cam = self.camera.getRelativePoint(
                    self.world.render, node_pos_w + axis_dir * sample_len
                )
                p0_2d = Point2()
                p1_2d = Point2()
                if lens.project(p0_cam, p0_2d) and lens.project(p1_cam, p1_2d):
                    axis_screen = Point2(p1_2d - p0_2d)
                    if axis_screen.length_squared() > 0:
                        axis_screen.normalize()
                    else:
                        axis_screen = Point2(1, 0)
                else:
                    axis_screen = Point2(1, 0)
                self.drag_axis_screen = Point2(axis_screen)

                self.drag_base_dist = max(self.drag_base_dist, 0.5)
                self.start_mouse = Point2(mpos.x, mpos.y)
                self.start_node_pos = node_pos_w
                self.last_mouse_pos = Point2(mpos.x, mpos.y)

                p_from = Point3()
                p_to = Point3()
                if lens.extrude(Point2(mpos.x, mpos.y), p_from, p_to):
                    ray_origin: Point3 = self.world.render.getRelativePoint(self.camera, p_from)
                    ray_to: Point3 = self.world.render.getRelativePoint(self.camera, p_to)
                    ray_dir: Vec3 = ray_to - ray_origin
                    if ray_dir.length_squared() != 0:
                        ray_dir.normalize()
                        axis_origin = self.start_node_pos
                        a = self.drag_axis_dir
                        d = ray_dir
                        w0 = ray_origin - axis_origin
                        a_dot_a = a.dot(a)
                        d_dot_d = d.dot(d)
                        a_dot_d = a.dot(d)
                        a_dot_w0 = a.dot(w0)
                        d_dot_w0 = d.dot(w0)
                        denom = a_dot_a * d_dot_d - a_dot_d * a_dot_d
                        if abs(denom) >= 1e-6:
                            s0 = (a_dot_d * d_dot_w0 - d_dot_d * a_dot_w0) / denom
                            self._start_param_on_ray = s0
                        else:
                            self._start_param_on_ray = 0.0
                    else:
                        self._start_param_on_ray = 0.0
                else:
                    self._start_param_on_ray = 0.0

                self._debug_drag_counter = 0
                self._highlight_axis(self.drag_axis)
                self._log(
                    f"pick axis={self.drag_axis} target={self.target_node.getName()} "
                    f"start_pos={self.start_node_pos} world_root={self.root.getPos(self.world.render)} "
                    f"axis_dir={axis_dir} axis_screen={self.drag_axis_screen}"
                )
                self._emit_event(
                    GizmoEvent.DRAG_START,
                    {
                        "gizmo": "move",
                        "target": self.target_node,
                        "axis": self.drag_axis,
                        "plane": None,
                        "mouse": Point2(mpos.x, mpos.y),
                        "start_pos": self.start_node_pos,
                    },
                )
            else:
                self._log("pick hit but no gizmo_axis or gizmo_plane tag on node")
        else:
            self._log("mouse_down: no collision entries on gizmo")

    def _on_mouse_up(self, extra: MouseInput | None = None) -> None:
        if self.dragging and self.target_node is not None:
            # Commit a move action to the undo stack if position changed
            try:
                final_pos = self.target_node.getPos(self.world.render)
            except Exception:
                final_pos = None
            old_pos = self.start_node_pos
            if old_pos is not None and final_pos is not None:
                self._record_move_action(self.target_node, old_pos, final_pos)

            axis_id = self.drag_axis
            plane_id = self.drag_plane_id
            self._emit_event(
                GizmoEvent.DRAG_END,
                {
                    "gizmo": "move",
                    "target": self.target_node,
                    "axis": axis_id,
                    "plane": plane_id,
                    "final_pos": final_pos,
                    "start_pos": old_pos,
                },
            )
            self.dragging = False
            self.drag_axis = None
            self.drag_plane_id = None
            if not self.is_local:
                self.root.setHpr(self.world.render, 0, 0, 0)
            self._reset_highlights()
            self._log("mouse_up -> stop dragging")

        self._reset_planels()

    def _reset_planels(self) -> None:
        if not self.camera:
            return

        # 摄像机在 gizmo 局部坐标系中的位置（会跟随 root 旋转）
        local_cam_pos: Point3 = self.camera.getPos(self.root)

        # 三个轴向的符号：摄像机在 gizmo 局部坐标系里的哪一侧
        sx = 1.0 if local_cam_pos.x >= 0.0 else -1.0
        sy = 1.0 if local_cam_pos.y >= 0.0 else -1.0
        sz = 1.0 if local_cam_pos.z >= 0.0 else -1.0

        o = self.plane_offset

        # XY 平面：在 X/Y 方向选离摄像机最近的那一角，Z 固定 0
        self.planes[0].setPos(sx * o, sy * o, 0.0)

        # YZ 平面：在 Y/Z 方向选离摄像机最近的那一角，X 固定 0
        self.planes[1].setPos(0.0, sy * o, sz * o)

        # ZX 平面：在 Z/X 方向选离摄像机最近的那一角，Y 固定 0
        self.planes[2].setPos(sx * o, 0.0, sz * o)

    def _on_mouse_move(self, extra: MouseInput | None = None) -> None:
        if not self.attached or self.target_node is None:
            return

        mpos = self._get_normalized_mouse(extra)
        if mpos is None:
            if self.dragging:
                self._log("mouse_move ignored: no mouse pos while dragging")
            return

        # Hover highlight when not dragging.
        if not self.dragging:
            self._update_hover_highlight(mpos)
            return

        if not self.target_node:
            return

        # Build world-space mouse ray from current mouse position
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

        # Plane dragging: move freely within the plane defined at drag start.
        if self.drag_plane_id is not None:
            n = self.drag_plane_normal
            if n.length_squared() == 0:
                return
            denom = ray_dir.dot(n)
            if abs(denom) < 1e-6:
                # Ray nearly parallel to plane; skip this move to avoid jumps.
                return
            t = (self.drag_plane_origin - ray_origin).dot(n) / denom
            hit_point = ray_origin + ray_dir * t
            new_pos = hit_point + self._plane_drag_offset
            self.target_node.setPos(self.world.render, new_pos)

            if self._debug_drag_counter % 4 == 0:
                self._log(
                    f"drag plane={self.drag_plane_id} mpos={mpos} "
                    f"ray_origin={ray_origin} ray_dir={ray_dir} "
                    f"plane_origin={self.drag_plane_origin} plane_normal={n} "
                    f"hit_point={hit_point} new_pos={new_pos}"
                )
            self._debug_drag_counter += 1
            self._emit_event(
                GizmoEvent.DRAG_MOVE,
                {
                    "gizmo": "move",
                    "target": self.target_node,
                    "axis": None,
                    "plane": self.drag_plane_id,
                    "mouse": Point2(mpos.x, mpos.y),
                    "new_pos": new_pos,
                },
            )
            return

        # --- 3D-based drag along axis: use closest point between mouse ray and axis line ---
        axis_dir = self.drag_axis_dir
        if axis_dir.length_squared() == 0:
            return

        axis_origin = self.start_node_pos
        if axis_origin is None or self.start_mouse is None:
            return
        a = axis_dir
        d = ray_dir
        w0 = ray_origin - axis_origin

        a_dot_a = a.dot(a)
        d_dot_d = d.dot(d)
        a_dot_d = a.dot(d)
        a_dot_w0 = a.dot(w0)
        d_dot_w0 = d.dot(w0)

        denom = a_dot_a * d_dot_d - a_dot_d * a_dot_d
        if abs(denom) < 1e-6:
            mouse_ndc = Point2(mpos.x, mpos.y)
            delta = mouse_ndc - self.start_mouse
            s_scalar = delta.dot(self.drag_axis_screen)
            move_amount = s_scalar * self.drag_base_dist * 0.5
            new_pos = axis_origin + a * move_amount
            self._log(f"drag fallback (parallel): s={s_scalar:.4f} move={move_amount:.4f}")
        else:
            s = (a_dot_d * d_dot_w0 - d_dot_d * a_dot_w0) / denom
            delta_s = self._start_param_on_ray - s
            new_pos = axis_origin + a * delta_s

        self.target_node.setPos(self.world.render, new_pos)

        if self._debug_drag_counter % 4 == 0:
            self._log(
                f"drag axis={self.drag_axis} mpos={mpos} "
                f"ray_origin={ray_origin} ray_dir={ray_dir} "
                f"axis_origin={axis_origin} axis_dir={axis_dir} new_pos={new_pos}"
            )
        self._debug_drag_counter += 1
        self._emit_event(
            GizmoEvent.DRAG_MOVE,
            {
                "gizmo": "move",
                "target": self.target_node,
                "axis": self.drag_axis,
                "plane": None,
                "mouse": Point2(mpos.x, mpos.y),
                "new_pos": new_pos,
            },
        )

    def _update_hover_highlight(self, mpos: Point3) -> None:
        """Highlight the handle under cursor during hover (no drag)."""
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

        plane_entry: CollisionEntry | None = None
        axis_entry: CollisionEntry | None = None
        for i in range(num_entries):
            e = self.c_queue.getEntry(i)
            if plane_entry is None and e.getIntoNode().getTag("gizmo_plane"):
                plane_entry = e
            if axis_entry is None and e.getIntoNode().getTag("gizmo_axis"):
                axis_entry = e
            if plane_entry and axis_entry:
                break

        if plane_entry is not None:
            plane_tag = plane_entry.getIntoNode().getTag("gizmo_plane")
            self._highlight_plane(int(plane_tag))
            self.is_hovering = True
        elif axis_entry is not None:
            axis_tag = axis_entry.getIntoNode().getTag("gizmo_axis")
            self._highlight_axis(int(axis_tag))
            self.is_hovering = True
        else:
            self._reset_highlights()
            self.is_hovering = False

    def _highlight_axis(self, axis_id: int | None) -> None:
        """高亮被选中的轴，其他轴和平面变暗。"""
        self._reset_highlights()
        for i, ax in enumerate(self.axes):
            if i == axis_id:
                ax.setColor(1, 1, 0, 1)
                ax.clearColorScale()
            else:
                ax.clearColor()
                ax.setAlphaScale(0.3)
        # 所有平面也变暗
        for plane in self.planes:
            plane.setAlphaScale(0.3)

    def _reset_highlights(self) -> None:
        """重置所有高亮效果。"""
        for ax in self.axes:
            ax.clearColor()
            ax.clearColorScale()
            ax.setAlphaScale(self.arrow_transparency)
        for plane in self.planes:
            plane.clearColorScale()
            plane.setAlphaScale(self.panel_transparency)

    def _highlight_plane(self, plane_id: int) -> None:
        """高亮被选中的平面，其他平面和轴变暗。"""
        self._reset_highlights()
        for i, plane in enumerate(self.planes):
            if i == plane_id:
                plane.setColorScale(1.6, 1.6, 1.6, 2.0)
            else:
                plane.setAlphaScale(0.3)
        # 所有轴也变暗
        for ax in self.axes:
            ax.clearColor()
            ax.setAlphaScale(0.3)

    def update(self) -> None:
        """Update gizmo scale based on camera distance."""
        if (
            not getattr(self, "managed_input", False)
            and self.world.mouseWatcherNode is not None
            and self.world.mouseWatcherNode.has_mouse()
        ):
            current_mouse = LPoint2f(self.world.mouseWatcherNode.get_mouse())
            if current_mouse != self.last_mouse_pos:
                self._on_mouse_move()
                self.last_mouse_pos = current_mouse
        if self.attached and self.camera:
            self._update_root_transform()
            self.root.setScale(self.world.render, screen_scale(self))

    def _update_task(self, task: Task) -> int:
        """Panda3D task that keeps the gizmo updated every frame."""
        self.update()
        return Task.cont

    # ------------------------------------------------------------------ #
    # Internal transform sync
    # ------------------------------------------------------------------ #
    def _update_root_transform(self) -> None:
        """Keep gizmo root aligned to target without inheriting its render state."""
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
