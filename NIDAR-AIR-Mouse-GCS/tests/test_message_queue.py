"""Unit tests for PriorityMessageQueue (Buffer Overflow Prevention & Priority Order)."""
import asyncio
import pytest
from backend.message_queue import PriorityMessageQueue


@pytest.mark.asyncio
async def test_bounded_queue_drop_oldest():
    # Create small video queue of size 2
    q = PriorityMessageQueue(video_limit=2)
    q.start()

    # Enqueue 10 video frames rapidly without dropping or blocking
    for i in range(10):
        await q.put_video_frame("rgb", {"frame_id": i})

    # Total frames in queue must not exceed limit 2
    assert q.video_rgb_queue.qsize() <= 2
    assert q.dropped_frames_count == 8  # 8 oldest frames safely dropped

    q.stop()


@pytest.mark.asyncio
async def test_high_priority_message_preservation():
    q = PriorityMessageQueue(high_limit=50)
    q.start()

    # High priority messages (Survivors / Aborts) are never dropped
    for i in range(5):
        await q.put_high({"type": "survivor_detected", "id": f"S{i}"})

    assert q.high_queue.qsize() == 5
    q.stop()
