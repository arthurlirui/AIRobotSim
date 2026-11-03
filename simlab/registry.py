"""
Process-wide algorithm registry.

Algorithms register themselves at import time via :func:`register`::

    @register("slam", "occupancy_grid")
    class _Registered(OccupancyGridSLAM): ...

Scenarios and benchmarks look them up by ``(kind, name)`` without importing
the implementing module, which keeps the platform decoupled from the set of
available algorithms.

A single shared :data:`registry` instance is exported; tests may construct a
fresh :class:`AlgorithmRegistry` to verify isolation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable

if TYPE_CHECKING:
    from algorithms.base import AlgorithmBase


@dataclass
class _Entry:
    cls: type  # type: AlgorithmBase
    factory: Callable[..., Any] | None = None


class AlgorithmRegistry:
    """Maps ``(kind, name)`` to an algorithm class."""

    def __init__(self) -> None:
        self._entries: dict[tuple[str, str], _Entry] = {}

    def register(self, kind: str, name: str, cls: type) -> type:
        # Duck-typed check: avoid importing algorithms.base at runtime to keep
        # this module import-cycle-free (algorithms/* import `register`).
        if not hasattr(cls, "setup") or not hasattr(cls, "step"):
            raise TypeError(f"{cls!r} does not look like an AlgorithmBase")
        key = (kind, name)
        if key in self._entries:
            # idempotent re-registration (re-import) — keep first binding
            return cls
        self._entries[key] = _Entry(cls=cls)
        return cls

    def create(self, kind: str, name: str, **kwargs):
        key = (kind, name)
        if key not in self._entries:
            available = ", ".join(f"{k[0]}/{k[1]}" for k in self._entries)
            raise KeyError(
                f"algorithm {kind}/{name} not registered "
                f"(available: {available})")
        return self._entries[key].cls(**kwargs)

    def list(self, kind: str | None = None) -> list[tuple[str, str]]:
        if kind is None:
            return sorted(self._entries.keys())
        return sorted(k for k in self._entries if k[0] == kind)

    def has(self, kind: str, name: str) -> bool:
        return (kind, name) in self._entries

    def __len__(self) -> int:
        return len(self._entries)


#: The shared, process-wide registry.
registry = AlgorithmRegistry()


def register(kind: str, name: str):
    """Decorator that binds an algorithm class into :data:`registry`."""
    def deco(cls: type[AlgorithmBase]) -> type[AlgorithmBase]:
        registry.register(kind, name, cls)
        return cls
    return deco
