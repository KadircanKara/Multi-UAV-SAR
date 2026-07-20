/** @type {import('next').NextConfig} */
const nextConfig = {
  async redirects() {
    return [
      // Flattened hubs (2026-07 route re-architecture) — keep old URLs alive.
      { source: "/missions/seeded-results", destination: "/missions", permanent: true },
      { source: "/missions/playground", destination: "/optimize", permanent: true },
      { source: "/compare/seeded-results", destination: "/compare", permanent: true },
      { source: "/compare/playground", destination: "/compare", permanent: true },
    ];
  },
};

export default nextConfig;
