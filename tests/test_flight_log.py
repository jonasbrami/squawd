# tests/test_flight_log.py
import json
from dronebot.flight_log import FlightLog


def test_log_writes_jsonl_records(tmp_path):
    path = tmp_path / "flight.jsonl"
    log = FlightLog(str(path))
    log.record("utterance", {"text": "take off"})
    log.record("command_result", {"ok": True, "message": "climbing"})
    lines = path.read_text().strip().splitlines()
    assert len(lines) == 2
    first = json.loads(lines[0])
    assert first["kind"] == "utterance"
    assert first["data"]["text"] == "take off"
    assert "ts" in first
