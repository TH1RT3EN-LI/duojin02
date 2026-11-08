"""Bounded producer control for offline playback, independent of scan admission.

The player keeps its requested maximum rate and original messages. Pauses only
change wall time. A pending future-transform dependency prevents a steady pause;
if an in-flight message opens one after acknowledgement, resume the producer.
The native queue limit remains a hard failure, not extra storage or a drop rule.
"""

import math


class PlaybackFlow:
    SCHEMA = "slam-input-status.v2"

    def __init__(self):
        self.capacity = self.high = self.low = None
        self.paused = False
        self.paused_since = None
        self.paused_wall_sec = 0.0
        self.pause_count = 0
        self.dependency_reopens = 0
        self.peak_outstanding = 0
        self.peak_queued = 0
        self.peak_pause_overshoot = 0
        self.events = []
        self.last_counts = None
        self.finished_wall_s = None
        self.paused_at_eof = False

    def inspect(self, status):
        if status.get("schema") != self.SCHEMA:
            raise RuntimeError("Offline producer control requires slam-input-status.v2")
        names = ("capacity", "physical_pending", "received", "accepted", "completed", "queued", "invalid", "outstanding")
        if any(type(status.get(name)) is not int or status[name] < 0 for name in names):
            raise RuntimeError("Invalid bounded-queue counters")
        if type(status.get("processing")) is not bool:
            raise RuntimeError("Invalid bounded-queue processing state")
        if status.get("clock_active") is not True or type(status.get("clock_nanoseconds")) is not int or status["clock_nanoseconds"] < 0:
            raise RuntimeError("Recorded source clock is unavailable")
        # ClearQueue preserves historical admission counters and marks cancelled
        # work invalid. Its old counters need not satisfy the live FIFO identity.
        if status["invalid"]:
            raise RuntimeError("Native input rejection during bounded playback")
        capacity = status["capacity"]
        if self.capacity is None:
            self.capacity = capacity
            self.high = max(1, capacity // 4)
            self.low = capacity // 16
            if not 0 <= self.low < self.high < capacity:
                raise RuntimeError("Queue capacity cannot provide producer-control headroom")
        if capacity != self.capacity:
            raise RuntimeError("Native queue capacity changed during playback")
        accepted, completed = status["accepted"], status["completed"]
        if not completed <= accepted <= status["received"]:
            raise RuntimeError("Native admission/completion exceeds received inputs")
        outstanding = accepted - completed
        if outstanding != status["outstanding"] or outstanding != status["queued"] + int(status["processing"]):
            raise RuntimeError("Native outstanding-work invariant failed")
        counts = status["received"], accepted, completed
        if self.last_counts is not None and any(a < b for a, b in zip(counts, self.last_counts)):
            raise RuntimeError("Native admission/completion counters went backwards")
        self.last_counts = counts
        self.peak_outstanding = max(self.peak_outstanding, outstanding)
        self.peak_queued = max(self.peak_queued, status["queued"])
        if outstanding >= capacity:
            raise RuntimeError("Producer could not reach a safe pause barrier before queue headroom was exhausted")
        return outstanding

    def request(self, status):
        if self.finished_wall_s is not None:
            raise RuntimeError("Producer control requested after EOF")
        outstanding = self.inspect(status)
        if self.paused:
            if status["physical_pending"]:
                return "resume_dependencies"
            if outstanding <= self.low:
                return "resume_low_watermark"
        elif outstanding >= self.high and status["physical_pending"] == 0:
            return "pause_high_watermark"
        return None

    def acknowledged(self, action, status, now):
        if not math.isfinite(now):
            raise RuntimeError("Invalid producer-control wall time")
        outstanding = self.inspect(status)
        if action == "pause_high_watermark":
            if self.paused:
                raise RuntimeError("Duplicate producer pause acknowledgement")
            self.paused = True
            self.paused_since = now
            self.pause_count += 1
            self.peak_pause_overshoot = max(self.peak_pause_overshoot, max(0, outstanding - self.high))
        elif action in ("resume_dependencies", "resume_low_watermark"):
            if not self.paused or self.paused_since is None or now < self.paused_since:
                raise RuntimeError("Producer resume without a valid pause interval")
            self.paused_wall_sec += now - self.paused_since
            self.paused_since = None
            self.paused = False
            self.dependency_reopens += int(action == "resume_dependencies")
        else:
            raise RuntimeError("Unknown producer-control action")
        self.events.append({"action": action, "wall_monotonic_s": now, "source_clock_ns": status["clock_nanoseconds"],
                            "accepted": status["accepted"], "completed": status["completed"],
                            "outstanding": outstanding, "queued": status["queued"],
                            "physical_pending": status["physical_pending"]})
        # A message already in DDS can arrive after the player's pause response.
        # Reopen the producer promptly instead of waiting for unavailable future TF.
        return self.request(status)

    def finished(self, now):
        """Close producer wall-time accounting at observed successful EOF."""
        if not math.isfinite(now) or self.finished_wall_s is not None:
            raise RuntimeError("Invalid or repeated producer EOF boundary")
        self.paused_at_eof = self.paused
        if self.paused_since is not None:
            if now < self.paused_since:
                raise RuntimeError("Producer EOF precedes its pause interval")
            self.paused_wall_sec += now - self.paused_since
        self.paused_since = None
        self.paused = False
        self.finished_wall_s = now

    def report(self, now):
        active = 0.0 if self.paused_since is None else max(0.0, now - self.paused_since)
        return {"schema": "offline-producer-flow.v1", "policy": "dependency_closed_hysteresis",
                "rate_semantics": "requested maximum playback rate; pauses add wall time without changing source timestamps or order",
                "capacity": self.capacity, "high_watermark": self.high, "low_watermark": self.low,
                "pause_count": self.pause_count, "paused_wall_sec": self.paused_wall_sec + active,
                "dependency_reopens": self.dependency_reopens, "peak_outstanding": self.peak_outstanding,
                "peak_queued": self.peak_queued, "peak_pause_overshoot": self.peak_pause_overshoot,
                "paused_at_report": self.paused, "events": list(self.events),
                "producer_finished_wall_monotonic_s": self.finished_wall_s,
                "paused_at_eof": self.paused_at_eof,
                "limits": "Middleware in-flight bursts are not inferred from polling frequency. Native rejection, headroom exhaustion, unavailable status or missing pause acknowledgement fails explicitly."}
