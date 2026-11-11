"""Тесты гейта фото-улик: улика снимается по уровню угрозы, а не по координате."""

from mchs_vision.events import should_save_evidence


def test_evidence_is_taken_for_critical_and_warning():
    """CRITICAL и WARNING всегда оставляют фото-улику в reports/incidents."""
    assert should_save_evidence({"threat_level": "CRITICAL"}) is True
    assert should_save_evidence({"threat_level": "warning"}) is True


def test_evidence_is_skipped_below_warning():
    """ADVISORY и события без уровня не создают файлов-улик."""
    assert should_save_evidence({"threat_level": "ADVISORY"}) is False
    assert should_save_evidence({}) is False
    assert should_save_evidence(None) is False
