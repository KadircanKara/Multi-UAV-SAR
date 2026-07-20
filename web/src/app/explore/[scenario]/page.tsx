"use client";

/**
 * /explore/[scenario] — legacy deep-link redirector.
 *
 * Old bookmarks address a scenario by its FULL backend name. The canonical URL
 * now nests under the model (/missions/[modelKey]/scenario/[suffix]) and only
 * the front payload knows the model, so this cannot be a next.config redirect:
 * fetch the front, read model_key, strip the prefix, replace the URL.
 */

import { useEffect } from "react";
import { useParams, useRouter } from "next/navigation";
import { getFront } from "@/lib/api";

export default function LegacyExploreRedirect() {
  const params = useParams();
  const router = useRouter();
  const raw = params?.scenario;
  const scenario = decodeURIComponent(
    Array.isArray(raw) ? raw[0] ?? "" : raw ?? ""
  );

  useEffect(() => {
    if (!scenario) return;
    let cancelled = false;
    getFront(scenario)
      .then((front) => {
        if (cancelled) return;
        const suffix = "g_" + (scenario.split("_g_")[1] ?? "");
        if (front.model_key && suffix !== "g_") {
          router.replace(
            `/missions/${encodeURIComponent(front.model_key)}/scenario/${encodeURIComponent(suffix)}`
          );
        } else {
          router.replace("/missions");
        }
      })
      .catch(() => {
        if (!cancelled) router.replace("/missions");
      });
    return () => {
      cancelled = true;
    };
  }, [scenario, router]);

  return (
    <div className="mx-auto max-w-7xl px-4 py-6">
      <p className="text-sm text-muted-foreground">Redirecting…</p>
    </div>
  );
}
