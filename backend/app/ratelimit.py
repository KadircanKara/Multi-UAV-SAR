"""Shared per-IP rate limiter.

Defined in its own module so both the app (main.py — registers state + the
429 handler) and the routers (which decorate individual endpoints) can import
the same Limiter without a circular import.
"""
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)
