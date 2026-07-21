import json

import pytest

from pi_arm_websocket.protocol import (
    COMMANDS,
    CODE_INVALID_JSON,
    CODE_INVALID_REQUEST,
    CODE_UNSUPPORTED_COMMAND,
    ProtocolError,
    parse_request,
    response,
)


def test_parse_request_normalizes_command_and_defaults_params():
    request = parse_request('{"type":"req","msg_id":7,"cmd":" MoveJ "}')
    assert request.msg_id == 7
    assert request.command == "movej"
    assert request.params == {}


@pytest.mark.parametrize("command", sorted(COMMANDS))
def test_all_legacy_commands_are_accepted_by_parser(command):
    request = parse_request(json.dumps({"type": "req", "msg_id": 1, "cmd": command}))
    assert request.command == command


@pytest.mark.parametrize(
    ("payload", "code"),
    [
        ("not-json", CODE_INVALID_JSON),
        ('{"type":"state","cmd":"movej"}', CODE_INVALID_REQUEST),
        ('{"type":"req","cmd":"unknown"}', CODE_UNSUPPORTED_COMMAND),
    ],
)
def test_parse_request_reports_stable_error_codes(payload, code):
    with pytest.raises(ProtocolError) as caught:
        parse_request(payload)
    assert caught.value.code == code


def test_response_keeps_legacy_ms_field():
    value = json.loads(response(3, 0, "ok", {"accepted": True}))
    assert value == {
        "type": "rsp",
        "msg_id": 3,
        "code": 0,
        "ms": "ok",
        "data": {"accepted": True},
    }
