from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

CODE_SUCCESS = 0
CODE_INVALID_JSON = 1001
CODE_INVALID_REQUEST = 1002
CODE_UNSUPPORTED_COMMAND = 1003
CODE_INVALID_PARAMS = 1004
CODE_STATE_UNAVAILABLE = 2001
CODE_COMMAND_FAILED = 2002
CODE_MOTION_BUSY = 2003

COMMANDS = {
    "enable_motor",
    "disable_motor",
    "reset_motor",
    "set_zero",
    "movej",
    "movejs",
    "movel",
    "stop_motion",
    "move",
    "motor_move",
    "motor_stop",
}


@dataclass(frozen=True)
class Request:
    msg_id: int
    command: str
    params: dict[str, Any]


class ProtocolError(Exception):
    def __init__(self, code: int, message: str, msg_id: int = 0) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.msg_id = msg_id


def parse_request(raw: str) -> Request:
    try:
        value = json.loads(raw)
    except (json.JSONDecodeError, TypeError) as exc:
        raise ProtocolError(CODE_INVALID_JSON, "请求体不是合法 JSON") from exc
    if not isinstance(value, dict):
        raise ProtocolError(CODE_INVALID_REQUEST, "请求体必须是对象")
    msg_id = _integer(value.get("msg_id", 0), "msg_id", allow_zero=True)
    if value.get("type", "req") != "req":
        raise ProtocolError(CODE_INVALID_REQUEST, "请求 type 必须为 req", msg_id)
    command = value.get("cmd")
    if not isinstance(command, str) or not command.strip():
        raise ProtocolError(CODE_INVALID_REQUEST, "cmd 不能为空", msg_id)
    command = command.strip().lower()
    if command not in COMMANDS:
        raise ProtocolError(CODE_UNSUPPORTED_COMMAND, f"不支持的命令: {command}", msg_id)
    params = value.get("params", {})
    if params is None:
        params = {}
    if not isinstance(params, dict):
        raise ProtocolError(CODE_INVALID_REQUEST, "params 必须是对象", msg_id)
    return Request(msg_id, command, params)


def response(msg_id: int, code: int, message: str, data: dict[str, Any] | None = None) -> str:
    return json.dumps(
        {"type": "rsp", "msg_id": msg_id, "code": code, "ms": message, "data": data or {}},
        ensure_ascii=False,
        separators=(",", ":"),
    )


def required_float(params: dict[str, Any], name: str, *, positive: bool = False) -> float:
    if name not in params:
        raise ProtocolError(CODE_INVALID_PARAMS, f"缺少参数: {name}")
    try:
        value = float(params[name])
    except (TypeError, ValueError) as exc:
        raise ProtocolError(CODE_INVALID_PARAMS, f"{name} 必须是数字") from exc
    if not math.isfinite(value) or (positive and value <= 0.0):
        suffix = "大于 0 的有限数字" if positive else "有限数字"
        raise ProtocolError(CODE_INVALID_PARAMS, f"{name} 必须是{suffix}")
    return value


def required_vector(
    params: dict[str, Any], name: str, length: int | None = None
) -> list[float]:
    value = params.get(name)
    if not isinstance(value, list) or (length is not None and len(value) != length):
        expected = f"且包含 {length} 个元素" if length is not None else ""
        raise ProtocolError(CODE_INVALID_PARAMS, f"{name} 必须是数组{expected}")
    result: list[float] = []
    for index, item in enumerate(value):
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise ProtocolError(CODE_INVALID_PARAMS, f"{name}[{index}] 必须是数字") from exc
        if not math.isfinite(number):
            raise ProtocolError(CODE_INVALID_PARAMS, f"{name}[{index}] 必须是有限数字")
        result.append(number)
    return result


def integer_param(
    params: dict[str, Any], name: str, *, default: int | None = None, allow_zero: bool = False
) -> int:
    if name not in params:
        if default is None:
            raise ProtocolError(CODE_INVALID_PARAMS, f"缺少参数: {name}")
        return default
    return _integer(params[name], name, allow_zero=allow_zero)


def _integer(value: Any, name: str, *, allow_zero: bool) -> int:
    if isinstance(value, bool) or isinstance(value, float) and not value.is_integer():
        raise ProtocolError(CODE_INVALID_PARAMS, f"{name} 必须是整数")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(CODE_INVALID_PARAMS, f"{name} 必须是整数") from exc
    if parsed < 0 or (parsed == 0 and not allow_zero):
        raise ProtocolError(CODE_INVALID_PARAMS, f"{name} 必须大于{'等于 ' if allow_zero else ' '}0")
    return parsed
