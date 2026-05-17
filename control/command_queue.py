#!/usr/bin/env python3
"""Thread-safe command queue used by ControlThread.

This queue provides an explicit command API:
- enqueue
- peek
- clear

Only ControlThread should dequeue/process commands.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from queue import Empty
from threading import Lock
from typing import Any, Deque, Optional


class CommandType(str, Enum):
    """Supported high-level control commands."""

    JOG = "JOG"
    EXECUTE = "EXECUTE"
    ABORT = "ABORT"
    RETURN = "RETURN"
    RETURN_HOME = "RETURN_HOME"


@dataclass(frozen=True)
class ControlCommand:
    """Normalized command object for control processing."""

    command_type: CommandType
    payload: Any = None


class ThreadSafeCommandQueue:
    """A small lock-based queue for command passing to ControlThread."""

    def __init__(self) -> None:
        self._queue: Deque[Any] = deque()
        self._lock = Lock()

    def enqueue(self, command: Any) -> None:
        """Append a command item to the queue."""
        with self._lock:
            self._queue.append(command)

    def peek(self) -> Optional[Any]:
        """Peek at the next command without removing it."""
        with self._lock:
            return self._queue[0] if self._queue else None

    def clear(self) -> int:
        """Clear all queued commands and return number removed."""
        with self._lock:
            removed = len(self._queue)
            self._queue.clear()
            return removed

    def dequeue_nowait(self) -> Any:
        """Remove and return next command; raise Empty if queue is empty."""
        with self._lock:
            if not self._queue:
                raise Empty()
            return self._queue.popleft()

    def qsize(self) -> int:
        with self._lock:
            return len(self._queue)

    def empty(self) -> bool:
        return self.qsize() == 0

    # Compatibility aliases for existing call sites
    def put(self, item: Any) -> None:
        self.enqueue(item)

    def get_nowait(self) -> Any:
        return self.dequeue_nowait()

    def task_done(self) -> None:
        # This queue is not join/task-tracking based.
        return None
