/**
 * Comm-range humanization.
 *
 * A comm-range key is the comm_cell_range expressed in CELL units:
 *   "2"        → 2 cells
 *   "sqrt(8)"  → 2√2 cells ≈ the diagonal span of 2 cells ("2 diagonal cells")
 *   "4"        → 4 cells
 * The metre distance = cells × cell_side_length (default 50 m — the only cell
 * side present in the seeded library).
 */

const SQRT2 = Math.SQRT2;

/** Numeric comm_cell_range (cell units) for a comm-range key — used for sorting
 *  smallest→largest (so "2" < "sqrt(8)" ≈ 2.83 < "4"). */
export function commCellValue(comm: string): number {
  const m = /^sqrt\(([0-9.]+)\)$/i.exec(comm.trim());
  if (m) return Math.sqrt(parseFloat(m[1]!));
  const n = parseFloat(comm);
  return Number.isFinite(n) ? n : 0;
}

/**
 * Human-readable comm label.
 *   full:  "2 cells · 100 m", "2 diagonal cells · 141 m", "4 cells · 200 m"
 *   short: "2 cells", "2 diagonal cells", "4 cells" (for tight axes/legends)
 */
export function commLabel(
  comm: string,
  cellSide = 50,
  opts?: { short?: boolean }
): string {
  const cells = commCellValue(comm);
  const metres = Math.round(cells * cellSide);
  const diagonal = /^sqrt\(/i.test(comm.trim());

  let core: string;
  if (diagonal) {
    const diag = Math.round(cells / SQRT2);
    core = `${diag} diagonal ${diag === 1 ? "cell" : "cells"}`;
  } else {
    const n = Number.isInteger(cells) ? String(cells) : cells.toFixed(2);
    core = `${n} ${cells === 1 ? "cell" : "cells"}`;
  }
  return opts?.short ? core : `${core} · ${metres} m`;
}
