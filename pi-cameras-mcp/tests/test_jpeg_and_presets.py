from pi_cameras_mcp.jpegutil import JpegSplitter, jpeg_size
from pi_cameras_mcp.presets import choose_mode, estimate_mbps, fallback_c920_modes

from tests.support import make_jpeg


def test_splitter_handles_chunks_and_garbage_prefix():
    first = make_jpeg(64, 48, b"\x01")
    second = make_jpeg(80, 60, b"\x02")
    blob = b"junk\xff" + first + b"\x00" + second
    splitter = JpegSplitter()
    frames = []
    for index in range(0, len(blob), 3):
        frames.extend(splitter.feed(blob[index : index + 3]))
    assert frames == [first, second]


def test_jpeg_size_reads_sof():
    assert jpeg_size(make_jpeg(1920, 1080)) == (1920, 1080)
    assert jpeg_size(b"nope") is None


def test_sweet_spot_bitrate_and_native_choice():
    assert 3.5 <= estimate_mbps(1280, 720, 15, 75) <= 4.5
    mode, fps, note = choose_mode(list(fallback_c920_modes()), 1280, 720, 15)
    assert (mode.width, mode.height, fps) == (1280, 720, 15)
    assert "exact" in note

    snapped, fps, note = choose_mode(list(fallback_c920_modes()), 1000, 700, 30)
    assert (snapped.width, snapped.height) == (1280, 720)
    assert fps == 30
    assert "snapped" in note

    slow, fps, note = choose_mode(list(fallback_c920_modes()), 1920, 1080, 5)
    assert (slow.width, slow.height) == (1920, 1080)
    assert fps == 15
    assert "15" in note
