# Next.js frontend. Build from web/:
#
#   docker build --build-arg NEXT_PUBLIC_API_BASE=https://api.example.com -t sar-web web
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
ARG NEXT_PUBLIC_API_BASE
RUN test -n "$NEXT_PUBLIC_API_BASE" \
    || { echo "ERROR: build arg NEXT_PUBLIC_API_BASE is required (the API URL is baked in at build time)"; exit 1; }
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
