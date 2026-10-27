import json
import math

from fire_inspection_orchestrator.detection_validation import parse_detections


def valid_detection(**updates):
    detection = {
        "cls": "smoking",
        "conf": 0.91,
        "cx": 0.5,
        "cy": 0.4,
        "w": 0.2,
        "h": 0.3,
    }
    detection.update(updates)
    return detection


def test_valid_detection_is_accepted():
    detection = valid_detection()
    assert parse_detections(json.dumps([detection])) == [detection]


def test_empty_list_is_valid():
    assert parse_detections("[]") == []


def test_invalid_json_is_rejected():
    assert parse_detections("{broken json") is None


def test_json_null_is_rejected():
    assert parse_detections("null") is None


def test_json_object_is_rejected():
    assert parse_detections('{"cls":"smoking"}') is None


def test_non_dictionary_detection_is_rejected():
    assert parse_detections(json.dumps(["smoking"])) is None


def test_missing_required_field_is_rejected():
    detection = valid_detection()
    del detection["conf"]
    assert parse_detections(json.dumps([detection])) is None


def test_empty_class_is_rejected():
    assert parse_detections(json.dumps([valid_detection(cls=" ")])) is None


def test_string_confidence_is_rejected():
    assert parse_detections(json.dumps([valid_detection(conf="0.9")])) is None


def test_boolean_confidence_is_rejected():
    assert parse_detections(json.dumps([valid_detection(conf=True)])) is None


def test_nan_confidence_is_rejected():
    assert parse_detections(
        '[{"cls":"smoking","conf":NaN,"cx":0.5,"cy":0.5,"w":0.2,"h":0.2}]'
    ) is None


def test_confidence_out_of_range_is_rejected():
    assert parse_detections(json.dumps([valid_detection(conf=1.1)])) is None


def test_invalid_coordinate_is_rejected():
    assert parse_detections(json.dumps([valid_detection(cx=1.5)])) is None


def test_negative_box_size_is_rejected():
    assert parse_detections(json.dumps([valid_detection(w=-0.1)])) is None


def test_one_bad_detection_invalidates_whole_message():
    detections = [valid_detection(), valid_detection(conf="bad")]
    assert parse_detections(json.dumps(detections)) is None
