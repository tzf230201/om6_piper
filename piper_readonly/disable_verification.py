"""Offline evidence checks only; no USB, SDK calls, or motion authorization."""
import math


def verify_disabled(snapshot, command_completed_perf_s):
    """Require six fresh raw driver frames received AFTER command completion.

    snapshot_perf_s and rx_age_s must use the same perf_counter clock as the
    command record. An SDK boolean or adapter echo is not execution evidence.
    This does not establish braking, holding, or a certified emergency stop.
    """
    reasons = []
    raw = snapshot.get('raw_latest') or {}
    ages = snapshot.get('rx_age_s') or {}
    stamp = snapshot.get('snapshot_perf_s')
    def finite(value):
        return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    if not finite(stamp) or not finite(command_completed_perf_s):
        return dict(disabled_confirmed=False, reasons=['Invalid clock evidence'], motion_release=False)
    for ident in range(0x261, 0x267):
        key = f'0x{ident:03X}'
        age = ages.get(key)
        if not finite(age) or age < 0 or age > .15:
            reasons.append(f'{key}: missing/invalid/stale feedback')
            continue
        if stamp-age <= command_completed_perf_s:
            reasons.append(f'{key}: feedback predates command completion')
            continue
        try:
            payload = bytes.fromhex(raw[key])
            if len(payload) != 8:
                raise ValueError('length')
        except (KeyError, TypeError, ValueError):
            reasons.append(f'{key}: invalid payload')
            continue
        if payload[5] & 0x40:
            reasons.append(f'{key}: enable bit still active')
        if payload[5] & 0xBF:
            reasons.append(f'{key}: fault/protection flag active')
    return dict(disabled_confirmed=not reasons, reasons=reasons,
                motion_release=False, physical_stop_verified=False,
                note='Driver disable feedback only; not proof of physical holding or stopping a moving arm.')
