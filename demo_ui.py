"""演示专用界面：像素布局、统一样式和动态信息，与插件实现分离。"""

from __future__ import annotations

import sys
from collections.abc import Callable
from pathlib import Path

from direct.gui import DirectGuiGlobals as DGG
from direct.gui.DirectGui import DirectButton, DirectFrame
from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from panda3d.core import Filename, LVecBase3f, NodePath, TextFont, TextNode

Color = tuple[float, float, float, float]
BACKGROUND: Color = (0.045, 0.061, 0.085, 0.97)
SURFACE: Color = (0.075, 0.093, 0.123, 1)
BORDER: Color = (0.16, 0.19, 0.24, 1)
TEXT: Color = (0.9, 0.93, 0.97, 1)
MUTED: Color = (0.51, 0.59, 0.69, 1)
ACCENT: Color = (1, 0.66, 0.29, 1)
AXIS_COLORS: tuple[Color, Color, Color] = (
    (0.98, 0.4, 0.38, 1),
    (0.36, 0.82, 0.6, 1),
    (0.42, 0.66, 1, 1),
)


class DemoUI:
    """基于 pixel2d 的演示 HUD，窗口缩小时整体缩放并重新锚定侧栏。"""

    def __init__(
        self,
        world: ShowBase,
        object_names: list[str],
        on_mode: Callable[[str], None],
        on_select: Callable[[int], None],
    ) -> None:
        self.world: ShowBase = world
        self.root: NodePath = world.pixel2d.attachNewNode("demo-hud")
        self.font: TextFont = self._load_font("segoeui.ttf")
        self.number_font: TextFont = self._load_font("consola.ttf")
        self.labels: dict[str, OnscreenText] = {}
        self.mode_buttons: dict[str, DirectButton] = {}
        self.object_buttons: list[DirectButton] = []
        self._texts: dict[str, str] = {}
        self._bounds: list[tuple[float, float, float, float]] = []
        self.scale: float = 1.0
        self._size: tuple[int, int] = (0, 0)
        self._button_state: tuple[str, int | None] | None = None

        self.header = self._card(self.root, 0, 0, 1280, 68)
        self._text(self.header, "TRANSFORM GIZMO", 24, 30, 19, TEXT)
        self._text(self.header, "PANDA3D  /  INTERACTIVE DEMO", 24, 51, 10, MUTED)
        for index, (key, title, mode) in enumerate(
            (("W", "Move", "move"), ("E", "Rotate", "rotate"), ("R", "Scale", "scale"))
        ):
            button = DirectButton(
                parent=self.header,
                pos=(285 + index * 115, 0, -16),
                frameSize=(0, 105, -36, 0),
                relief=DGG.FLAT,
                frameColor=SURFACE,
                text=f"{key}   {title}",
                text_font=self.font,
                text_scale=13,
                text_pos=(52.5, -23),
                text_fg=TEXT,
                command=on_mode,
                extraArgs=[mode],
                pressEffect=False,
            )
            self.mode_buttons[mode] = button
        self.labels["mode"] = self._text(self.header, "", 660, 39, 13, ACCENT)

        self.object_card = self._card(self.root, 18, 88, 222, 179)
        self._text(self.object_card, "SCENE OBJECTS", 16, 27, 11, MUTED)
        for index, name in enumerate(object_names):
            button = DirectButton(
                parent=self.object_card,
                pos=(12, 0, -(40 + index * 39)),
                frameSize=(0, 198, -33, 0),
                relief=DGG.FLAT,
                frameColor=SURFACE,
                text=f"{index + 1}    {name}",
                text_align=TextNode.ALeft,
                text_font=self.font,
                text_scale=12,
                text_pos=(12, -22),
                text_fg=TEXT,
                command=on_select,
                extraArgs=[index],
                pressEffect=False,
            )
            self.object_buttons.append(button)
        self._text(self.object_card, "Click a model / keys 1, 2, 3", 16, 170, 10, MUTED)

        self.inspector = self._card(self.root, 982, 88, 280, 290)
        self._text(self.inspector, "INSPECTOR", 18, 27, 11, MUTED)
        self.labels["selection"] = self._text(self.inspector, "", 18, 56, 18, TEXT)
        self.labels["space"] = self._text(self.inspector, "", 18, 80, 11, ACCENT)
        self._divider(self.inspector, 18, 96, 244)
        for index, axis in enumerate("XYZ"):
            self._text(self.inspector, axis, 98 + index * 68, 119, 11, AXIS_COLORS[index])
        for row, (label, key) in enumerate(
            (("Position", "pos"), ("Rotation", "hpr"), ("Scale", "scale"))
        ):
            y = 145 + row * 30
            self._text(self.inspector, label, 18, y, 12, MUTED)
            for axis in range(3):
                self.labels[f"{key}-{axis}"] = self._text(
                    self.inspector,
                    "",
                    82 + axis * 68,
                    y,
                    12,
                    TEXT,
                    numeric=True,
                )
        self._divider(self.inspector, 18, 221, 244)
        self.labels["handles"] = self._text(self.inspector, "", 18, 244, 11, MUTED)
        self.labels["history"] = self._text(self.inspector, "", 18, 268, 11, TEXT)

        self.help_card = self._card(self.root, 982, 392, 280, 345)
        self._text(self.help_card, "KEYBOARD & CAMERA", 18, 27, 11, MUTED)
        rows = (
            ("A / Q", "All handles / hide"),
            ("L / G", "Local-world / enable"),
            ("7 / 8 / 9", "Toggle handle types"),
            ("Tab / Esc", "Next object / deselect"),
            ("Ctrl Z / Y", "Undo / redo"),
            ("Backspace", "Reset scene"),
            ("RMB drag", "Orbit camera"),
            ("MMB drag", "Pan camera"),
            ("Wheel", "Zoom camera"),
            ("F / Home", "Focus / reset camera"),
            ("D", "Toggle debug log"),
        )
        for row, (key, description) in enumerate(rows):
            y = 53 + row * 24
            self._text(self.help_card, key, 18, y, 11, ACCENT)
            self._text(self.help_card, description, 112, y, 11, TEXT)

        self.footer = self._card(self.root, 18, 752, 1244, 32)
        self.labels["event"] = self._text(self.footer, "", 14, 21, 11, TEXT)
        self.labels["hint"] = self._text(self.root, "", 262, 110, 12, MUTED)
        self.resize()

    def _load_font(self, filename: str) -> TextFont:
        """优先使用 Windows 系统字体；其他平台回退默认字体，不打包系统字体。"""
        if sys.platform == "win32":
            path = Path("C:/Windows/Fonts") / filename
            if path.is_file():
                font = self.world.loader.loadFont(Filename.fromOsSpecific(str(path)).getFullpath())
                if font is not None:
                    return font
        return TextNode.getDefaultFont()

    @staticmethod
    def _card(parent: NodePath, x: float, y: float, width: float, height: float) -> DirectFrame:
        return DirectFrame(
            parent=parent,
            pos=(x, 0, -y),
            frameSize=(0, width, -height, 0),
            frameColor=BACKGROUND,
            relief=DGG.FLAT,
        )

    @staticmethod
    def _divider(parent: NodePath, x: float, y: float, width: float) -> DirectFrame:
        return DirectFrame(
            parent=parent,
            pos=(x, 0, -y),
            frameSize=(0, width, -1, 0),
            frameColor=BORDER,
            relief=DGG.FLAT,
        )

    def _text(
        self,
        parent: NodePath,
        text: str,
        x: float,
        y: float,
        size: float,
        color: Color,
        *,
        numeric: bool = False,
    ) -> OnscreenText:
        return OnscreenText(
            parent=parent,
            text=text,
            pos=(x, -y),
            scale=size,
            fg=color,
            font=self.number_font if numeric else self.font,
            align=TextNode.ALeft,
            mayChange=True,
        )

    def resize(self) -> None:
        """按当前窗口尺寸缩放，维持字号与留白比例，避免侧栏离开视口。"""
        width, height = self.world.win.getXSize(), self.world.win.getYSize()
        if (width, height) == self._size:
            return
        self._size = (width, height)
        self.scale = max(0.1, min(width / 1280, height / 800))
        self.root.setScale(self.scale)
        ui_width, ui_height = width / self.scale, height / self.scale
        right = ui_width - 298
        self.header["frameSize"] = (0, ui_width, -68, 0)
        self.inspector.setX(right)
        self.help_card.setX(right)
        self.footer.setZ(-(ui_height - 48))
        self.footer["frameSize"] = (0, ui_width - 36, -32, 0)
        self._bounds = [
            (0, 0, ui_width, 68),
            (18, 88, 222, 179),
            (right, 88, 280, 290),
            (right, 392, 280, 345),
            (18, ui_height - 48, ui_width - 36, 32),
        ]

    def contains_mouse(self, x: float, y: float) -> bool:
        """仅屏蔽实际面板区域，物体列表下方仍可操作场景。"""
        px, py = x / self.scale, y / self.scale
        return any(
            left <= px <= left + w and top <= py <= top + h for left, top, w, h in self._bounds
        )

    def _set(self, key: str, value: str) -> None:
        # 只在内容变化时更新 TextNode，减少每帧的文字几何重建。
        if self._texts.get(key) != value:
            self.labels[key].setText(value)
            self._texts[key] = value

    def update(
        self,
        *,
        selection: int | None,
        name: str,
        mode: str,
        local: bool,
        enabled: bool,
        debug: bool,
        handles: dict[str, bool],
        history: int,
        position: LVecBase3f | None,
        rotation: LVecBase3f | None,
        scale: LVecBase3f | None,
        event_count: int,
        event: str,
    ) -> None:
        self.resize()
        self._set("selection", name)
        self._set("mode", f"{mode.upper()}  /  {'LOCAL' if local else 'WORLD'}")
        self._set(
            "space",
            f"{'LOCAL AXES' if local else 'WORLD AXES'}  |  GIZMO {'ON' if enabled else 'OFF'}",
        )
        for key, vector in (("pos", position), ("hpr", rotation), ("scale", scale)):
            for axis in range(3):
                self._set(f"{key}-{axis}", f"{vector[axis]:.2f}" if vector is not None else "--")
        self._set(
            "handles",
            "   ".join(
                f"{kind.title()} {'on' if value else 'off'}" for kind, value in handles.items()
            ),
        )
        self._set("history", f"Undo steps  {history}     Debug  {'on' if debug else 'off'}")
        self._set("event", f"EVENT {event_count:03d}    {event}")
        hint = {
            "move": "Drag an arrow for one axis, or a square for plane movement.",
            "rotate": "Drag a ring to rotate, or the inner area for free trackball rotation.",
            "scale": "Drag an axis tip; drag the white center up/down for uniform scale.",
            "all": "Combined handles  /  W, E, R select a precise transform tool.",
            "none": "Handles hidden  /  Press W, E or R to resume.",
        }.get(mode, "")
        self._set("hint", hint)
        if self._button_state != (mode, selection):
            self._button_state = (mode, selection)
            for kind, button in self.mode_buttons.items():
                button["frameColor"] = (0.29, 0.2, 0.12, 1) if kind == mode else SURFACE
                button["text_fg"] = ACCENT if kind == mode else MUTED
            for index, button in enumerate(self.object_buttons):
                button["frameColor"] = (0.2, 0.16, 0.12, 1) if index == selection else SURFACE
                button["text_fg"] = ACCENT if index == selection else TEXT

    def destroy(self) -> None:
        for label in self.labels.values():
            label.destroy()
        self.header.destroy()
        self.object_card.destroy()
        self.inspector.destroy()
        self.help_card.destroy()
        self.footer.destroy()
        self.root.removeNode()
