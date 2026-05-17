#!/usr/bin/env python3
"""Robot state machine for the control layer.

All control-state transitions are defined here in one place.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from threading import Lock
from typing import Dict


class ControlState(str, Enum):
    IDLE = "IDLE"
    JOG = "JOG"
    EXECUTE = "EXECUTE"
    ABORT = "ABORT"
    RETURN_HOME = "RETURN_HOME"


class ControlEvent(str, Enum):
    START_JOG = "START_JOG"
    START_EXECUTE = "START_EXECUTE"
    START_RETURN_HOME = "START_RETURN_HOME"
    ABORT = "ABORT"
    COMPLETE = "COMPLETE"
    RESET = "RESET"


@dataclass(frozen=True)
class StateTransition:
    previous_state: ControlState
    new_state: ControlState
    event: ControlEvent
    changed: bool


_TRANSITIONS: Dict[ControlState, Dict[ControlEvent, ControlState]] = {
    ControlState.IDLE: {
        ControlEvent.START_JOG: ControlState.JOG,
        ControlEvent.START_EXECUTE: ControlState.EXECUTE,
        ControlEvent.START_RETURN_HOME: ControlState.RETURN_HOME,
        ControlEvent.ABORT: ControlState.ABORT,
        ControlEvent.COMPLETE: ControlState.IDLE,
        ControlEvent.RESET: ControlState.IDLE,
    },
    ControlState.JOG: {
        ControlEvent.START_JOG: ControlState.JOG,
        ControlEvent.START_EXECUTE: ControlState.EXECUTE,
        ControlEvent.START_RETURN_HOME: ControlState.RETURN_HOME,
        ControlEvent.ABORT: ControlState.ABORT,
        ControlEvent.COMPLETE: ControlState.IDLE,
        ControlEvent.RESET: ControlState.IDLE,
    },
    ControlState.EXECUTE: {
        ControlEvent.START_JOG: ControlState.JOG,
        ControlEvent.START_EXECUTE: ControlState.EXECUTE,
        ControlEvent.START_RETURN_HOME: ControlState.RETURN_HOME,
        ControlEvent.ABORT: ControlState.ABORT,
        ControlEvent.COMPLETE: ControlState.IDLE,
        ControlEvent.RESET: ControlState.IDLE,
    },
    ControlState.ABORT: {
        ControlEvent.START_JOG: ControlState.JOG,
        ControlEvent.START_EXECUTE: ControlState.EXECUTE,
        ControlEvent.START_RETURN_HOME: ControlState.RETURN_HOME,
        ControlEvent.ABORT: ControlState.ABORT,
        ControlEvent.COMPLETE: ControlState.ABORT,
        ControlEvent.RESET: ControlState.IDLE,
    },
    ControlState.RETURN_HOME: {
        ControlEvent.START_JOG: ControlState.JOG,
        ControlEvent.START_EXECUTE: ControlState.EXECUTE,
        ControlEvent.START_RETURN_HOME: ControlState.RETURN_HOME,
        ControlEvent.ABORT: ControlState.ABORT,
        ControlEvent.COMPLETE: ControlState.IDLE,
        ControlEvent.RESET: ControlState.IDLE,
    },
}


class RobotStateMachine:
    """Thread-safe robot control state machine."""

    def __init__(self) -> None:
        self._state = ControlState.IDLE
        self._lock = Lock()

    @property
    def state(self) -> ControlState:
        with self._lock:
            return self._state

    def transition(self, event: ControlEvent) -> StateTransition:
        with self._lock:
            previous_state = self._state
            new_state = _TRANSITIONS[previous_state][event]
            self._state = new_state
            return StateTransition(
                previous_state=previous_state,
                new_state=new_state,
                event=event,
                changed=new_state != previous_state,
            )

    def start_jog(self) -> StateTransition:
        return self.transition(ControlEvent.START_JOG)

    def start_execute(self) -> StateTransition:
        return self.transition(ControlEvent.START_EXECUTE)

    def start_return_home(self) -> StateTransition:
        return self.transition(ControlEvent.START_RETURN_HOME)

    def abort(self) -> StateTransition:
        return self.transition(ControlEvent.ABORT)

    def complete(self) -> StateTransition:
        return self.transition(ControlEvent.COMPLETE)

    def reset(self) -> StateTransition:
        return self.transition(ControlEvent.RESET)

    def is_idle(self) -> bool:
        return self.state == ControlState.IDLE

    def is_abort(self) -> bool:
        return self.state == ControlState.ABORT

    def is_motion_state(self) -> bool:
        return self.state in {
            ControlState.JOG,
            ControlState.EXECUTE,
            ControlState.RETURN_HOME,
        }
