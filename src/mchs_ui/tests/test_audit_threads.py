"""UI workers must remain compatible with threading.Thread lifecycle."""
import pytest
import app as ui


@pytest.mark.parametrize("factory", [
    lambda path: ui.VideoClient(enabled=False),
    lambda path: ui.StateClient(enabled=False),
    lambda path: ui.RecordingWriter(path, size=(32, 32)),
])
def test_worker_stop_join(tmp_path, factory):
    worker = factory(tmp_path / "clip.mp4")
    worker.start()
    worker.stop()
    worker.join(timeout=3)
    assert not worker.is_alive()
