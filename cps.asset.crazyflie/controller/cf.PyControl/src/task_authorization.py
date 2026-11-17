"""Execute a client's short-lived permission. No position or collision policy."""
import math


class TaskAuthorization:
    MAX_LIFETIME = .5

    def __init__(self):
        self.task_id = None
        self.expires_at = None
        self.reason = None
        self.stop_task_id = None

    def authorize(self, expires_at, now):
        if self.expires_at is not None and now > self.expires_at:
            raise ValueError("Authorization already expired; start a new task")
        if isinstance(expires_at, bool):
            raise ValueError("expires_at must be a finite controller-clock time")
        expiry = float(expires_at)
        if not math.isfinite(expiry) or not now < expiry <= now + self.MAX_LIFETIME + 1e-9:
            raise ValueError("expires_at must be within the next 0.5 controller-clock seconds")
        if self.expires_at is not None and expiry < self.expires_at:
            raise ValueError("expires_at cannot move backwards")
        self.expires_at = expiry

    def check(self, now):
        if self.expires_at is None:
            return "authorization_missing"
        if now > self.expires_at:
            return "authorization_expired"
        return None

    def snapshot(self):
        return dict(task_id=self.task_id, expires_at=self.expires_at,
                    stop_reason=self.reason, stop_task_id=self.stop_task_id)
