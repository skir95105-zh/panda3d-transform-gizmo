# TransformGizmo 事件钩子使用说明

本文档说明如何在 `TransformGizmo` 初始化时注入拖拽事件回调（类似 C# 委托），以及各事件的负载字段。

## 事件常量

所有事件名集中在 `TransformGizmo.events.GizmoEvent`，避免拼写错误：

```python
from TransformGizmo import DragEvent, GizmoEvent

GizmoEvent.DRAG_START  # "drag_start"
GizmoEvent.DRAG_MOVE  # "drag_move"
GizmoEvent.DRAG_END  # "drag_end"
GizmoEvent.ALL  # ("drag_start", "drag_move", "drag_end")
```

## 基础用法示例

在构造 `TransformGizmo` 时传入 `event_hooks`，按 handle 类型分组（`"move"`, `"rotate"`, `"scale"` 或对应 `TransformGizmoMode` 值），每组里是事件名到回调列表的映射：

```python
from TransformGizmo import DragEvent, TransformGizmo, TransformGizmoMode
from TransformGizmo import GizmoEvent


def on_move_start(info: DragEvent) -> None:
    print("[move] start", "axis", info.get("axis"), "plane", info.get("plane"))


def on_move_drag(info: DragEvent) -> None:
    print("[move] pos ->", info.get("new_pos"))


def on_rotate_drag(info: DragEvent) -> None:
    print("[rotate] delta_deg =", info.get("delta_deg"))


def on_scale_end(info: DragEvent) -> None:
    print("[scale] final_scale =", info.get("final_scale"))


hooks = {
    TransformGizmoMode.MOVE: {
        GizmoEvent.DRAG_START: [on_move_start],
        GizmoEvent.DRAG_MOVE: [on_move_drag],
    },
    TransformGizmoMode.ROTATE: {
        GizmoEvent.DRAG_MOVE: [on_rotate_drag],
    },
    TransformGizmoMode.SCALE: {
        GizmoEvent.DRAG_END: [on_scale_end],
    },
}

# world: 已创建窗口的 ShowBase 实例
gizmo = TransformGizmo(world, event_hooks=hooks)
```

### 运行时追加 / 移除

初始化后通过公开接口注册和移除回调：

```python
from TransformGizmo import GizmoEvent

gizmo.add_event_hook("move", GizmoEvent.DRAG_MOVE, on_move_drag)
gizmo.remove_event_hook("move", GizmoEvent.DRAG_MOVE, on_move_drag)
```

## 回调负载字段

各事件都会传入一个 `dict`，常见字段如下（不存在的字段将为 `None` 或缺省）：

### Move
- `axis`：0/1/2（X/Y/Z），平面拖拽时为 `None`
- `plane`：0/1/2（XY/YZ/ZX），轴拖拽时为 `None`
- `mouse`：`Point2`，拖拽开始/过程中鼠标 NDC 坐标
- `start_pos` / `new_pos` / `final_pos`：世界坐标 `Vec3`
- `target`：当前绑定的 `NodePath`
- `gizmo`：字符串 `"move"`

### Rotate
- `mode`：`"axis"` 或 `"trackball"`
- `axis`：0/1/2（X/Y/Z），轨迹球时为 `None`
- `delta_deg`：相对拖拽起点累计的旋转角度（度），轨迹球为起始/当前向量夹角
- `start_quat`：拖拽开始时的世界四元数
- `final_hpr`：拖拽结束时的世界 HPR（仅 `drag_end`）
- `center`：gizmo 中心点（世界坐标）
- `axis_world`：当前旋转轴的世界方向
- `mouse`：`Point2`
- `target`，`gizmo` 同上

### Scale
- `axis`：0/1/2 对应 X/Y/Z，3 表示中心均匀缩放
- `scale_factor`：本次 `drag_move` 计算出的缩放倍率
- `start_scale` / `new_scale` / `final_scale`：目标相对父节点的缩放 `Vec3`
- `mouse`：`Point2`
- `target`，`gizmo` 同上

## 小贴士
- 事件字典的键可以用字符串 `"move"`/`"rotate"`/`"scale"`，也可以用 `TransformGizmoMode` 的对应值，内部都会匹配。
- 每个事件支持多个回调（列表），内部隔离单个回调异常，互不影响；`set_debug(True)` 可查看异常信息。
- 如果只想监听部分事件，留空即可，内部会自动填充为空列表。

## 输入与生命周期

默认通过 Panda3D 鼠标观察器获取坐标，外部 UI 可发送以下事件：

```python
# ndc 为 [-1, 1]，Y 向上。像素坐标原点在左上角。
base.messenger.send("mouse1", [{"ndc": (0.2, -0.1)}])
base.messenger.send("mouse-move", [{"x": 500, "y": 300, "width": 1280, "height": 800}])
base.messenger.send("mouse1-up", [{"ndc": (0.3, -0.1)}])
```

统一控制器监听 `mouse-move` 以支持外部坐标输入，普通窗口则每帧轮询。外部坐标优先；统一控制器自动更新并管理输入，宿主关闭窗口前调用 `destroy()`。

开始拖拽后切换模式、目标、坐标系或禁用插件，会先结束并记录当前拖拽。目标已删除时直接解除绑定。撤销/重做及宿主直接修改节点不会触发拖拽回调。回调应快速返回，不应在回调内部切换目标或销毁插件。

## 类型标注

`DragEvent`、`EventHandler`、`EventHooks`、`GroupedEventHooks` 和 `MouseInput` 可从 `TransformGizmo` 公开导入。`DragEvent` 为 TypedDict，所有事件均包含 `gizmo` 与 `target`，其他字段按阶段可选；通过 `get()` 并判断 `None` 后读取，IDE 可补全字段及 Panda3D 向量成员。类型定义不会改变运行时的字典回调形式。
