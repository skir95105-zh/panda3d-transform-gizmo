"""子控制柄共享的鼠标输入和屏幕尺寸计算，避免绑定特定 UI 框架。"""

from __future__ import annotations

from typing import TYPE_CHECKING, cast

from direct.showbase.ShowBase import ShowBase
from panda3d.core import Camera, NodePath, OrthographicLens, Point3

from .types import MouseInput

if TYPE_CHECKING:
    from .move_gizmo import MoveGizmo
    from .rotate_gizmo import RotateGizmo
    from .scale_gizmo import ScaleGizmo


def camera_path(world: ShowBase, path: NodePath | None = None) -> NodePath[Camera]:
    """先验证实际节点类型，再收窄 NodePath 泛型，避免 IDE 只识别 PandaNode。"""
    camera = path if path is not None else world.cam
    if camera is None or camera.isEmpty() or not isinstance(camera.node(), Camera):
        raise ValueError("必须提供有效的 Panda3D Camera 节点")
    return cast("NodePath[Camera]", camera)


def normalized_mouse(world: ShowBase, extra: MouseInput | None = None) -> Point3 | None:
    """优先使用外部坐标；像素原点在左上角，ndc 范围为 [-1, 1]。"""
    if isinstance(extra, dict):
        if "ndc" in extra:
            x, y = extra["ndc"]
            return Point3(x, y, 0)
        if "x" in extra and "y" in extra:
            width = extra.get("width", world.win.getXSize())
            height = extra.get("height", world.win.getYSize())
            return Point3(
                2 * extra["x"] / max(width, 1) - 1, 1 - 2 * extra["y"] / max(height, 1), 0
            )
    watcher = world.mouseWatcherNode
    if watcher is not None and watcher.hasMouse():
        mouse = watcher.getMouse()
        return Point3(mouse.x, mouse.y, 0)
    return None


def screen_scale(gizmo: MoveGizmo | RotateGizmo | ScaleGizmo) -> float:
    """以视线深度/镜头视野估算尺寸，兼容透视和正交镜头。"""
    lens = gizmo.camera.node().getLens()
    film_height = lens.getFilmSize().y
    if isinstance(lens, OrthographicLens):
        return max(0.001, film_height * 0.28)
    depth = gizmo.root.getPos(gizmo.camera).y
    return max(0.001, abs(depth) * film_height / lens.getFocalLength() * 0.28)


def destroy_gizmo(gizmo: MoveGizmo | RotateGizmo | ScaleGizmo) -> None:
    """销毁独立子控制柄，释放拾取射线以及碰撞遍历器引用。"""
    gizmo.detach()
    gizmo.ignoreAll()
    gizmo.c_trav.clearColliders()
    gizmo.c_queue.clearEntries()
    gizmo.picker_np.removeNode()
    gizmo.root.removeNode()
