"""Request-body size enforcement that does not trust the Content-Length header.

The original guard (a Content-Length check in main.py) could be bypassed with a
chunked body: no Content-Length, so nothing rejected it, and Starlette then
buffered the whole thing into memory — an unauthenticated OOM. This ASGI
middleware counts the bytes it actually receives and aborts with 413 the moment
they exceed the cap, so the memory one request can pin is bounded by the cap
itself, whatever the framing or a lying header claims.

Caddy enforces the same ceiling at the edge in production (request_body
max_size); this is the defense-in-depth layer for any path that does not go
through Caddy (local dev, direct container access).
"""
from __future__ import annotations

from starlette.responses import JSONResponse

from app import settings


class BodySizeLimitMiddleware:
    """Pure-ASGI middleware bounding the request body at settings.MAX_UPLOAD_BYTES.

    The limit is read per request so tests (and a live SIGHUP-free reconfigure)
    see changes without reinstantiation.
    """

    def __init__(self, app) -> None:
        self.app = app

    async def __call__(self, scope, receive, send) -> None:
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        max_bytes = settings.MAX_UPLOAD_BYTES

        # Fast path: an honest Content-Length over the cap is rejected without
        # reading a single body byte.
        for name, value in scope.get("headers", []):
            if name == b"content-length":
                try:
                    if int(value) > max_bytes:
                        await self._reject(scope, receive, send)
                        return
                except ValueError:
                    pass
                break

        # Otherwise count what actually arrives (this is what catches a chunked
        # or a lying Content-Length). Buffer is bounded by max_bytes + one chunk.
        body = bytearray()
        while True:
            message = await receive()
            if message["type"] != "http.request":
                # Disconnect or similar before the body completed: hand off with
                # whatever we have; the app will see it as an empty/short body.
                break
            body += message.get("body", b"")
            if len(body) > max_bytes:
                await self._reject(scope, receive, send)
                return
            if not message.get("more_body", False):
                break

        # Replay the buffered body to the app as a single message.
        replayed = False

        async def replay_receive():
            nonlocal replayed
            if not replayed:
                replayed = True
                return {"type": "http.request", "body": bytes(body), "more_body": False}
            return await receive()

        await self.app(scope, replay_receive, send)

    @staticmethod
    async def _reject(scope, receive, send) -> None:
        response = JSONResponse(
            status_code=413, content={"detail": "request body too large"}
        )
        await response(scope, receive, send)
