# FastAPI backend. Build from the REPO ROOT (the service imports the flat
# research modules that live there):
#
#   docker build -f backend/Dockerfile -t sar-backend .
#
# Mission data is NOT baked in — mount it at runtime (or set SAR_RESULTS_ROOT):
#
#   docker run -p 8000:8000 -v "$PWD/Results:/app/Results" sar-backend
FROM python:3.11-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    MPLBACKEND=Agg \
    MPLCONFIGDIR=/tmp/matplotlib

WORKDIR /app

COPY requirements.lock ./
RUN pip install --no-cache-dir -r requirements.lock

# Flat research modules at the repo root, then the FastAPI service.
COPY *.py ./
COPY backend/ ./backend/

# Results/ is a runtime mount; the app also writes temp run dirs under it
# (RESULTS_ROOT/.runs), so the unprivileged user must own it.
RUN useradd --uid 1000 --create-home appuser \
    && mkdir -p /app/Results \
    && chown -R appuser /app/Results
USER appuser

ENV PYTHONPATH=/app/backend
EXPOSE 8000

HEALTHCHECK --interval=30s --timeout=5s --start-period=30s --retries=3 \
    CMD python -c "import urllib.request as u; u.urlopen('http://127.0.0.1:8000/api/health', timeout=4)"

# Exactly ONE uvicorn worker — optimizer job state, the run queue, and
# rate-limit buckets are all per-process (see backend/app/optimizer_service.py).
# Never add --workers. Concurrency for optimizations comes from the pool inside
# this process instead: set SAR_OPTIMIZE_WORKERS (default 2, one core each).
# Behind a proxy/ALB, set FORWARDED_ALLOW_IPS to the proxy IP/CIDR (uvicorn
# reads it natively); it defaults to 127.0.0.1, never set it to '*'.
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000", "--proxy-headers"]
