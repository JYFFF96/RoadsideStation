from __future__ import print_function

import json


def encode_object_list(object_list):
    return json.dumps(object_list.to_dict(), ensure_ascii=False, separators=(",", ":"))


def encode_rsm(object_list):
    """Encode an RSM-like JSON envelope for algorithm integration.

    This is NOT yet ASN.1/UPER GB/T 31024 compliant. V0.1 keeps the interface
    stable so a standards-compliant encoder can replace this implementation.
    """
    participants = []
    for obj in object_list.objects:
        participants.append({
            "ptcId": obj.object_id,
            "ptcType": obj.object_type,
            "pos": {"x": obj.x, "y": obj.y, "z": obj.z},
            "speed": (obj.vx * obj.vx + obj.vy * obj.vy) ** 0.5,
            "confidence": obj.confidence,
        })
    message = {
        "msgType": "RSM",
        "version": "V0.1-json",
        "stationId": object_list.station_id,
        "timestamp": object_list.timestamp,
        "participants": participants,
    }
    return json.dumps(message, ensure_ascii=False, separators=(",", ":"))
