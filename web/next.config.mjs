/** @type {import('next').NextConfig} */
const nextConfig = {
  // Self-contained production server for the Docker image (web/Dockerfile
  // runs `node server.js` from .next/standalone). `next dev` is unaffected.
  output: "standalone",
  async redirects() {
    return [
      // Flattened hubs (2026-07 route re-architecture) — keep old URLs alive.
      { source: "/missions/seeded-results", destination: "/missions", permanent: true },
      { source: "/missions/playground", destination: "/optimize", permanent: true },
      { source: "/compare/seeded-results", destination: "/compare", permanent: true },
      { source: "/compare/playground", destination: "/compare", permanent: true },
      { source: "/model/:modelKey", destination: "/missions/:modelKey", permanent: true },
    ];
  },
};

export default nextConfig;
