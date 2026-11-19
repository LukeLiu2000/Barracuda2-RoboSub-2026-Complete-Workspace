from auv_perception.capture_clock import CaptureClock


def test_fixed_offset_preserves_capture_intervals_and_inference_delay():
    clock=CaptureClock(enabled=True)
    for i in range(8):
        raw=100+i*.2
        mapped=clock.observe(raw,raw-1.5+(i%3)*.02)
    assert mapped is not None and abs(clock.offset+1.5)<.03
    assert abs(clock.convert(105)-clock.convert(104)-1)<1e-9
    # A delayed inference packet keeps its original capture time.
    assert 105-clock.convert(105)>1.4
    assert clock.observe(106,108)==clock.convert(106)
    assert clock.offset< -1.4


def test_clock_jump_recalibrates_and_normal_clock_does_not_absorb_latency():
    clock=CaptureClock(enabled=True)
    for i in range(8):clock.observe(i*.2,i*.2-.02)
    assert clock.offset==0
    assert clock.observe(20,18) is None
    assert clock.offset is None
    clock.reset();assert clock.convert(100) is None
    assert CaptureClock().convert(100)==100


def test_unstable_clock_does_not_lock():
    clock=CaptureClock(enabled=True)
    for i in range(8):clock.observe(i,i-1-(i%2))
    assert clock.offset is None
