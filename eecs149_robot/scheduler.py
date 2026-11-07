"""Highest-priority-first dispatcher; never preempts while holding an object."""
from __future__ import annotations
import heapq
from dataclasses import dataclass, field
from .model import Request


@dataclass
class Scheduler:
    _pending: list[tuple[int, int, Request]] = field(default_factory=list)
    _seq: int = 0
    _ids: set[str] = field(default_factory=set)

    def submit(self, req: Request) -> None:
        if req.request_id in self._ids:
            raise ValueError(f"Duplicate request ID {req.request_id}")
        self._ids.add(req.request_id)
        heapq.heappush(self._pending, (-req.priority, self._seq, req))
        self._seq += 1

    def pop(self) -> Request | None:
        return heapq.heappop(self._pending)[2] if self._pending else None

    def peek(self) -> Request | None:
        return self._pending[0][2] if self._pending else None

    @property
    def pending(self) -> int:
        return len(self._pending)
