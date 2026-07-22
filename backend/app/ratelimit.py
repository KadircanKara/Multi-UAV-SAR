"""Shared per-IP rate limiter.

Defined in its own module so both the app (main.py — registers state + the
429 handler) and the routers (which decorate individual endpoints) can import
the same Limiter without a circular import.

Deployment requirement — the key is ``request.client.host``, so behind a
reverse proxy / load balancer (nginx, AWS ALB) every request arrives with the
proxy's IP and all clients share ONE bucket. Uvicorn must be told to restore
the real client IP from X-Forwarded-For, and to trust that header ONLY from
the proxy, or clients can spoof it to bypass the limit entirely:

    uvicorn app.main:app --proxy-headers --forwarded-allow-ips=<proxy IP/CIDR>

Never use ``--forwarded-allow-ips='*'`` on a host directly reachable from the
internet. Buckets are in-memory and per-process (a second reason the API must
run as a single worker — see optimizer_service).
"""
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)
