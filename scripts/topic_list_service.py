#!/usr/bin/env python3
"""Publish a periodically refreshed list of ROS 2 topics.

Publishes a single-line summary of all visible topics to the ``topics``
topic (i.e., ``/topics``). Useful for small displays like the SSD1306
OLED ticker to give a quick overview of active topics.

Examples:
    Run the service and inspect output::

        $ python3 scripts/topic_list_service.py  # doctest: +SKIP
        $ ros2 topic echo topics                 # doctest: +SKIP
"""
from __future__ import annotations

import argparse
from typing import Iterable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _format_topics(names: Iterable[str], *, max_len: int | None = None) -> str:
    """Format topic names into a single ticker line.

    Args:
        names: Iterable of topic names.
        max_len: Optional max character length; if provided, the output is
            truncated with an ellipsis when longer.

    Returns:
        A single-line string like ``"/asr | /logs | /chat"``.

    Examples:
        >>> _format_topics(["/a", "/b", "/c"])  # doctest: +NORMALIZE_WHITESPACE
        '/a | /b | /c'
    """
    line = " | ".join(sorted(set(map(str, names))))
    if max_len is not None and len(line) > max_len:
        return line[: max(0, max_len - 1)] + "…"
    return line


class TopicList(Node):
    """Publish the current topic list on ``/topics`` at an interval."""

    def __init__(self, *, interval: float = 2.0) -> None:
        super().__init__("topic_list")
        self._pub = self.create_publisher(String, "topics", 10)
        # Timer to refresh and publish the list
        self.create_timer(float(interval), self._tick)

    def _tick(self) -> None:
        names = [name for name, _types in self.get_topic_names_and_types()]
        msg = String()
        msg.data = _format_topics(names)
        self._pub.publish(msg)


def main(argv: list[str] | None = None) -> None:
    """Run the topic list publisher.

    Args:
        argv: Optional CLI args. Supports ``--interval`` seconds.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--interval", type=float, default=2.0,
                        help="publish interval in seconds")
    ns = parser.parse_args(argv)
    rclpy.init()
    node = TopicList(interval=ns.interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

