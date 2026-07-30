#!/usr/bin/env python3
# cyber_monitor_ros2.py

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import threading
from textual.app import App, ComposeResult
from textual import events
from textual.widgets import Header, Footer, DataTable, Static
from textual.containers import Horizontal, ScrollableContainer
from rich.markup import escape


class Ros2Monitor(Node):
    def __init__(self):
        super().__init__('ros2_cyber_monitor')
        self.topic_data = {}      # topic_name -> 解析后的 dict/list
        self.topic_text = {}      # topic_name -> 无法解析时的文本
        self.topic_types = {}     # topic_name -> msg type
        self.topic_subs = {}      # topic_name -> Subscription
        self.lock = threading.Lock()

    def update_topics(self):
        """动态发现新的 topic 并订阅"""
        topic_names_types = self.get_topic_names_and_types()
        current_topics = {name for name, _ in topic_names_types}

        # 取消已经不存在的 topic
        for name in list(self.topic_subs.keys()):
            if name not in current_topics:
                self.destroy_subscription(self.topic_subs[name])
                del self.topic_subs[name]
                with self.lock:
                    self.topic_data.pop(name, None)
                    self.topic_text.pop(name, None)
                    self.topic_types.pop(name, None)

        # 订阅新 topic
        for name, types in topic_names_types:
            if name in self.topic_subs or name.startswith('/rosout'):
                continue
            msg_type_str = types[0]
            try:
                msg_class = get_message(msg_type_str)
                sub = self.create_subscription(
                    msg_class,
                    name,
                    lambda msg, topic=name: self.callback(msg, topic),
                    10
                )
                self.topic_subs[name] = sub
                with self.lock:
                    self.topic_types[name] = msg_type_str
                    self.topic_data[name] = None
                    self.topic_text[name] = '<no data>'
            except Exception as e:
                with self.lock:
                    self.topic_types[name] = msg_type_str
                    self.topic_data[name] = None
                    self.topic_text[name] = f'<unsupported: {e}>'

    def callback(self, msg, topic):
        from rosidl_runtime_py.convert import message_to_ordereddict
        try:
            d = message_to_ordereddict(msg)
            with self.lock:
                self.topic_data[topic] = d
                self.topic_text[topic] = ''
        except Exception as e:
            text = str(msg)
            with self.lock:
                self.topic_data[topic] = None
                self.topic_text[topic] = text[:20000]


class MonitorApp(App):
    CSS = """
    #topic_table { width: 40%; height: 100%; }
    #detail_scroll { width: 60%; height: 100%; border: solid green; }
    #detail { width: 100%; height: auto; }
    """

    def __init__(self, monitor: Ros2Monitor, **kwargs):
        super().__init__(**kwargs)
        self.monitor = monitor
        self.selected_topic = None

        # 层级浏览状态
        self.view_path = []       # 进入层级用的路径，例如 ['header', 'stamp']
        self.selected_key = None  # 当前 dict 中选中的 key
        self.array_index = 0      # 当前 list 中显示的索引
        self._last_detail_content = None

    def compose(self) -> ComposeResult:
        yield Header()
        with Horizontal():
            yield DataTable(id="topic_table")
            with ScrollableContainer(id="detail_scroll"):
                yield Static("Select a topic to view data", id="detail")
        yield Footer()

    def on_mount(self):
        table = self.query_one("#topic_table", DataTable)
        table.add_columns("Topic", "Type")
        table.cursor_type = "row"
        table.focus()

        scroll = self.query_one("#detail_scroll", ScrollableContainer)
        scroll.can_focus = True

        self.update_timer = self.set_interval(2.0, self.refresh_topics)
        self.data_timer = self.set_interval(0.1, self.refresh_detail)

    def refresh_topics(self):
        self.monitor.update_topics()
        table = self.query_one("#topic_table", DataTable)
        with self.monitor.lock:
            topics = sorted(self.monitor.topic_types.items())

        current_topics = {name for name, _ in topics}
        existing_keys = set(table.rows.keys())

        # 增量更新：只删除消失的、添加新增的
        for key in existing_keys - current_topics:
            table.remove_row(key)

        for name, msg_type in topics:
            if name not in existing_keys:
                table.add_row(name, msg_type, key=name)

        # 保持光标在已选 topic 上
        if self.selected_topic and self.selected_topic in table.rows:
            row_keys = list(table.rows.keys())
            if self.selected_topic in row_keys:
                table.move_cursor(row=row_keys.index(self.selected_topic))

    def refresh_detail(self):
        if self.selected_topic is None:
            return
        detail = self.query_one("#detail", Static)
        new_content = self.render_view()
        if self._last_detail_content != new_content:
            self._last_detail_content = new_content
            detail.update(new_content)

    def render_view(self):
        if self.selected_topic is None:
            return "Select a topic to view data"

        with self.monitor.lock:
            data = self.monitor.topic_data.get(self.selected_topic)
            fallback = self.monitor.topic_text.get(self.selected_topic, "")

        if data is None:
            return fallback or "No data"

        # 定位到当前层级
        current = data
        for step in self.view_path:
            current = current[step]

        path_str = ".".join(str(p) for p in self.view_path) or "root"
        lines = [f"[b]{escape(self.selected_topic)}[/b] > {escape(path_str)}\n"]

        if isinstance(current, dict):
            keys = list(current.keys())
            if not keys:
                lines.append("(empty object)")
            else:
                if self.selected_key not in keys:
                    self.selected_key = keys[0]
                for k in keys:
                    marker = "> " if k == self.selected_key else "  "
                    lines.append(f"{marker}{escape(str(k))}: {self.format_value(current[k])}")
        elif isinstance(current, list):
            if not current:
                lines.append("(empty array)")
            else:
                idx = self.array_index % len(current)
                self.array_index = idx
                lines.append(f"[{idx + 1}/{len(current)}]: {self.format_value(current[idx])}")
        else:
            lines.append(self.format_value(current))

        lines.append("\n[dim]→/Enter/l:进入  ←/Esc/h:返回  ↑/k ↓/j:选择  n:下一个 m:上一个[/dim]")
        return "\n".join(lines)

    @staticmethod
    def format_value(v):
        if isinstance(v, dict):
            return f"{{...}} ({len(v)} fields)"
        elif isinstance(v, list):
            return f"[...] ({len(v)} items)"
        elif isinstance(v, str):
            return f'"{escape(v)}"'
        else:
            return escape(str(v))

    def get_current_data(self):
        if self.selected_topic is None:
            return None
        with self.monitor.lock:
            data = self.monitor.topic_data.get(self.selected_topic)
        if data is None:
            return None
        current = data
        for step in self.view_path:
            current = current[step]
        return current

    # ==================== 层级导航动作 ====================

    def action_enter(self):
        """进入当前选中的嵌套对象或数组元素"""
        current = self.get_current_data()
        if isinstance(current, dict) and self.selected_key in current:
            v = current[self.selected_key]
            if isinstance(v, (dict, list)):
                self.view_path.append(self.selected_key)
                self.selected_key = None
                self.array_index = 0
                self._last_detail_content = None
        elif isinstance(current, list) and current:
            v = current[self.array_index % len(current)]
            if isinstance(v, (dict, list)):
                self.view_path.append(self.array_index % len(current))
                self.selected_key = None
                self.array_index = 0
                self._last_detail_content = None

    def action_back(self):
        """返回上一层"""
        if self.view_path:
            last = self.view_path.pop()
            self._last_detail_content = None
            parent = self.get_current_data()
            if isinstance(parent, dict):
                self.selected_key = last if last in parent else None
                self.array_index = 0
            elif isinstance(parent, list):
                self.array_index = last if isinstance(last, int) and last < len(parent) else 0
                self.selected_key = None
            else:
                self.selected_key = None
                self.array_index = 0

    def action_up(self):
        current = self.get_current_data()
        if isinstance(current, dict):
            keys = list(current.keys())
            if keys and self.selected_key in keys:
                i = keys.index(self.selected_key)
                if i > 0:
                    self.selected_key = keys[i - 1]
                    self._last_detail_content = None

    def action_down(self):
        current = self.get_current_data()
        if isinstance(current, dict):
            keys = list(current.keys())
            if keys and self.selected_key in keys:
                i = keys.index(self.selected_key)
                if i < len(keys) - 1:
                    self.selected_key = keys[i + 1]
                    self._last_detail_content = None

    def action_next_array(self):
        """数组中显示下一个元素"""
        current = self.get_current_data()
        if isinstance(current, list) and current:
            self.array_index = (self.array_index + 1) % len(current)
            self._last_detail_content = None

    def action_prev_array(self):
        """数组中显示上一个元素"""
        current = self.get_current_data()
        if isinstance(current, list) and current:
            self.array_index = (self.array_index - 1) % len(current)
            self._last_detail_content = None

    # ==================== 事件处理 ====================

    def on_data_table_row_selected(self, event: DataTable.RowSelected):
        self.selected_topic = event.row_key.value
        self.view_path = []
        self.selected_key = None
        self.array_index = 0
        self._last_detail_content = None

    def on_key(self, event: events.Key):
        """只在右侧详情区有焦点时处理层级导航键

        由于 ScrollableContainer 可能会消费方向键，额外提供 vim 风格按键作为备选。
        """
        focused = self.focused
        if focused is None or focused.id != "detail_scroll":
            return

        key = event.key
        if key in ("right", "enter", "l"):
            self.action_enter()
        elif key in ("left", "escape", "h"):
            self.action_back()
        elif key in ("up", "k"):
            self.action_up()
        elif key in ("down", "j"):
            self.action_down()
        elif key == "n":
            self.action_next_array()
        elif key == "m":
            self.action_prev_array()
        else:
            return

        event.stop()


def main(args=None):
    rclpy.init(args=args)
    monitor = Ros2Monitor()

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spin_thread.start()

    app = MonitorApp(monitor)
    app.run()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
