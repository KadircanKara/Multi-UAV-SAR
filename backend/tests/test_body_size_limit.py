"""The request-body size cap must hold regardless of framing.

The first version checked only the Content-Length header, so a chunked body
(Transfer-Encoding: chunked, no Content-Length) sailed past it and buffered
unbounded in memory. These tests drive the ASGI middleware directly with a
synthetic receive channel — the only way to exercise a body that arrives in
chunks with no honest Content-Length — plus one end-to-end check.
"""
import asyncio

import pytest

from app import settings
from app.limits import BodySizeLimitMiddleware


def _run(coro):
    return asyncio.run(coro)


class _Recorder:
    """Minimal downstream ASGI app that drains the body it is handed."""

    def __init__(self):
        self.called = False
        self.body = bytearray()

    async def app(self, scope, receive, send):
        self.called = True
        while True:
            message = await receive()
            if message["type"] != "http.request":
                break
            self.body += message.get("body", b"")
            if not message.get("more_body", False):
                break


def _chunked_receive(*sizes):
    """A receive() that streams len(sizes) body chunks, then disconnects. No
    chunk carries a Content-Length — this is the framing the header check missed."""
    messages = [
        {"type": "http.request", "body": b"x" * n, "more_body": i < len(sizes) - 1}
        for i, n in enumerate(sizes)
    ]

    async def receive():
        if messages:
            return messages.pop(0)
        return {"type": "http.disconnect"}

    return receive


def test_rejects_chunked_body_over_limit(monkeypatch):
    monkeypatch.setattr(settings, "MAX_UPLOAD_BYTES", 10)
    rec = _Recorder()
    mw = BodySizeLimitMiddleware(rec.app)
    sent = []

    async def send(message):
        sent.append(message)

    # Two 6-byte chunks = 12 bytes > 10, arriving with no Content-Length.
    _run(mw({"type": "http", "headers": []}, _chunked_receive(6, 6), send))

    assert rec.called is False, "downstream app must not run for an oversized body"
    assert sent[0]["type"] == "http.response.start"
    assert sent[0]["status"] == 413


def test_passes_body_under_limit_and_replays_it(monkeypatch):
    monkeypatch.setattr(settings, "MAX_UPLOAD_BYTES", 100)
    rec = _Recorder()
    mw = BodySizeLimitMiddleware(rec.app)

    async def send(message):
        pass

    _run(mw({"type": "http", "headers": []}, _chunked_receive(6, 6), send))

    assert rec.called is True
    assert bytes(rec.body) == b"x" * 12, "the full body must reach the app intact"


def test_honest_oversized_content_length_rejected_without_reading(monkeypatch):
    monkeypatch.setattr(settings, "MAX_UPLOAD_BYTES", 10)
    rec = _Recorder()
    mw = BodySizeLimitMiddleware(rec.app)
    sent = []

    async def send(message):
        sent.append(message)

    async def receive():  # must never be called — CL header alone rejects
        raise AssertionError("receive() should not run when Content-Length is over limit")

    scope = {"type": "http", "headers": [(b"content-length", b"999")]}
    _run(mw(scope, receive, send))

    assert rec.called is False
    assert sent[0]["status"] == 413


def test_non_http_scope_passes_through(monkeypatch):
    rec = _Recorder()
    mw = BodySizeLimitMiddleware(rec.app)
    seen = {}

    async def app(scope, receive, send):
        seen["type"] = scope["type"]

    mw = BodySizeLimitMiddleware(app)

    async def receive():
        return {}

    async def send(message):
        pass

    _run(mw({"type": "lifespan"}, receive, send))
    assert seen["type"] == "lifespan"


def test_end_to_end_413_on_oversized_post(client, monkeypatch):
    """A real oversized POST is rejected at the middleware before the route."""
    monkeypatch.setattr(settings, "MAX_UPLOAD_BYTES", 32)
    resp = client.post("/api/optimize/check", content=b"{\"junk\": \"" + b"y" * 200 + b"\"}")
    assert resp.status_code == 413
