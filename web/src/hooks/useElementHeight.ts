"use client";

/**
 * Live pixel height of an element, via a callback ref.
 *
 * Exists for the sticky panel's offset. Every page that uses SectionPanelLayout
 * pins its own identity header at `top-14` first, and the panel has to pin
 * BELOW that header rather than underneath it — but the header's height is not
 * a constant anyone can hardcode: the model page's objective badges wrap to a
 * second row on a narrow window, and /optimize's header grows a RunSummary row
 * once a result is loaded. A ResizeObserver is the only thing that tracks that.
 *
 * A callback ref, not a RefObject, is load-bearing. Every one of these headers
 * renders behind a loading guard, so at first commit the element does not
 * exist; an effect that reads `ref.current` would find null and — with the ref
 * object as its only dependency — would never run again once the header
 * appeared, leaving the offset permanently 0. A callback ref makes the node
 * itself the state that drives the effect.
 *
 * Height is 0 until the element mounts, which is the right fallback: the panel
 * then pins at the bare nav offset, exactly where it used to.
 */

import { useCallback, useEffect, useState } from "react";

export function useElementHeight(): {
  ref: (el: HTMLElement | null) => void;
  height: number;
} {
  const [node, setNode] = useState<HTMLElement | null>(null);
  const [height, setHeight] = useState(0);

  const ref = useCallback((el: HTMLElement | null) => setNode(el), []);

  useEffect(() => {
    if (!node) {
      setHeight(0);
      return;
    }

    // Seed synchronously — the observer's first callback is a frame away, and
    // a frame of the panel sitting under the header is a visible jump.
    setHeight(node.getBoundingClientRect().height);

    const observer = new ResizeObserver((entries) => {
      const box = entries[0]?.target.getBoundingClientRect();
      if (box) setHeight(box.height);
    });
    observer.observe(node);
    return () => observer.disconnect();
  }, [node]);

  return { ref, height };
}
