# Next.js frontend. Build from web/:
#
#   docker build -t sar-web web
#
# No build arg is needed: NEXT_PUBLIC_API_BASE defaults to "" (same origin),
# which is what the production stack serves — Caddy puts the API and the UI on
# one hostname. Pass --build-arg NEXT_PUBLIC_API_BASE=https://api.example.com
# ONLY for a split-origin deployment; see the ARG below.
#
# NEXT_PUBLIC_API_BASE is inlined into the JS bundle at BUILD time — a runtime
# env var cannot change it. Rebuild the image to point at a different backend.
FROM node:20-alpine AS deps
WORKDIR /app
COPY package.json package-lock.json ./
RUN npm ci

FROM node:20-alpine AS build
WORKDIR /app
COPY --from=deps /app/node_modules ./node_modules
COPY . .
# Empty means "same origin": lib/api.ts prefixes every call with this value, so
# "" produces relative URLs like /api/health. That is what the production stack
# wants — Caddy serves the API and the UI on one hostname — and it keeps the
# image environment-independent, so the same artifact runs anywhere instead of
# having a domain baked in at build time.
#
# Set it explicitly only when the API lives on a DIFFERENT origin, e.g.
#   docker build --build-arg NEXT_PUBLIC_API_BASE=https://api.example.com
# in which case the backend's SAR_CORS_ORIGINS must allow the web origin.
ARG NEXT_PUBLIC_API_BASE=""
ENV NEXT_PUBLIC_API_BASE=$NEXT_PUBLIC_API_BASE
RUN npm run build

FROM node:20-alpine AS run
WORKDIR /app
ENV NODE_ENV=production \
    PORT=3000 \
    HOSTNAME=0.0.0.0
COPY --from=build /app/.next/standalone ./
COPY --from=build /app/.next/static ./.next/static
COPY --from=build /app/public ./public
USER node
EXPOSE 3000
CMD ["node", "server.js"]
