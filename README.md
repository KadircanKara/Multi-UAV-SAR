# Multi-UAV SAR — Path Optimizer

**An interactive, explainable web app for multi-objective path planning of cooperative UAV swarms in Search-and-Rescue (SAR) missions.**

Browse the results of my thesis research, compare optimization models head-to-head, or **configure and run the evolutionary optimizers yourself** — and watch them converge live, generation by generation.

<p>
  <img alt="Python" src="https://img.shields.io/badge/Python-3.11-3776AB?logo=python&logoColor=white">
  <img alt="FastAPI" src="https://img.shields.io/badge/FastAPI-0.13x-009688?logo=fastapi&logoColor=white">
  <img alt="pymoo" src="https://img.shields.io/badge/pymoo-0.6-5b21b6">
  <img alt="Next.js" src="https://img.shields.io/badge/Next.js-14-000000?logo=nextdotjs&logoColor=white">
  <img alt="React" src="https://img.shields.io/badge/React-18-61DAFB?logo=react&logoColor=black">
  <img alt="TypeScript" src="https://img.shields.io/badge/TypeScript-5-3178C6?logo=typescript&logoColor=white">
  <img alt="Tailwind CSS" src="https://img.shields.io/badge/Tailwind-3-06B6D4?logo=tailwindcss&logoColor=white">
</p>

---

## The problem

A swarm of UAVs has to **sweep a grid environment to find targets**, while staying **connected** (so they can relay what they sense back to a base station) and finishing **quickly**. These goals trade off against each other — a fast sweep scatters the drones and breaks the comms network; a tightly-connected swarm searches slowly. There is no single best plan, only a **Pareto front of trade-offs**.

This project frames that as a **multi-objective combinatorial optimization** problem and solves it with evolutionary algorithms (NSGA-II / NSGA-III, weighted-sum GA) via [`pymoo`](https://pymoo.org/). The web app makes the whole thing **explorable and reproducible** instead of a folder of static plots.

---

## What you can do in the web app

The app has three pillars. No setup knowledge required — pick a model, or describe a scenario and hit run.

### 🛰️ Mission Browser & Explore
Browse **20 pre-optimized models** and dig into the results of the thesis runs.
- **Parameter-effect analysis** — see how each objective responds to the number of drones, communication range, and revisit count (`n_visits`), with multi-line overlays and selectable sweep values.
- **Pareto fronts** — interactive scatter; click any solution to inspect its objective values.
- **Belief-merging analysis** — compare information-sharing strategies (no merging vs. drone-to-drone vs. base-relayed).
- **Live mission playback** — a canvas animation of the swarm flying its plan, with the search-area **belief map** updating in real time as cells get covered (variable playback speed).

### ⚙️ Optimizer — run your own
Configure an optimization end-to-end and **watch it solve live**:
- **Single- or multi-objective** (SOO / MOO) → method (**Weighted-Sum** or **GA** for SOO; **NSGA-II / NSGA-III** for MOO; MOEA/D flagged *coming soon*).
- **Objectives** with the rules enforced for you (one objective for SOO-GA; weights that must sum to 1 for weighted-sum; ≥2 for MOO).
- **Constraints** — a speed-feasibility constraint is always on; you can add a **max mission time** and **min % connectivity**.
- **Realistic comm range** — pick it in intuitive units (cells, *N diagonal cells*, or raw **metres**); connectivity is computed from true inter-drone distances.
- **Live progress** — while it runs, the config panel is replaced by **per-objective best-value trajectories** and, for MOO, a **live-updating Pareto front** whose axes you choose.
- **Stop anytime** and keep the best-so-far result, then **save it to the library** so it joins the browsable missions (with a duplicate pre-check on the exact model + parameter combination).

### 📊 Model Comparison
Put models **head-to-head across every objective** — even objectives a given model never optimized for (each solution is evaluated on all five). Choose the view that tells the story best: **bar, line, radar, or table**, plus a dedicated comparison of sensing **time-metrics** (mission time, detection time, inform time, time-until-one-drone-knows-all).

> 💡 The whole UI ships with **light and dark modes** and a clean, responsive design.

---

## Screenshots

<!--
  Add images to docs/screenshots/ and uncomment:

  ![Mission Explore](docs/screenshots/explore.png)
  ![Optimizer — live progress](docs/screenshots/optimizer.png)
  ![Model Comparison](docs/screenshots/compare.png)
-->

_Run it locally (below) and drop screenshots of the Explore, Optimizer, and Compare views into `docs/screenshots/`._

---

## The science (in brief)

**Five objectives** are computed for every solution:

| Objective | Meaning | Goal |
|---|---|---|
| **Mission Time** | time to complete the sweep | minimize |
| **Percentage Connectivity** | mean fraction of the mission the swarm is connected to base | **maximize** |
| **Max / Mean Disconnected Time** | worst-case / average time a drone is cut off | minimize |
| **Max Mean TBV** | time-between-visits to cells (revisit freshness) | minimize |

**Sensing & information merging** — drones run a Bayesian belief update over the grid; connected drones fuse their maps (`none` / `onboard` drone-to-drone / `gcs` base-relayed), under either a `discrete` (per cell-step) or `realtime` (continuous, per-second) connectivity model. These drive the **time-metrics** shown in the app.

**Optimization core** — `pymoo` algorithms over a custom permutation encoding with problem-specific sampling, crossover, mutation, and repair operators. Optimizer runs execute in an isolated worker process and stream per-generation progress to the UI.

---

## Tech stack

| Layer | Stack |
|---|---|
| **Optimization engine** | Python · pymoo · NumPy · SciPy · pandas |
| **Backend API** | FastAPI · Pydantic · Uvicorn (background-run + poll for live optimizer jobs) |
| **Frontend** | Next.js 14 (App Router) · React 18 · TypeScript · Tailwind CSS · shadcn/ui · Recharts · canvas animation |

---

## Getting started

Two processes: the **FastAPI backend** (the optimization engine + API) and the **Next.js frontend**.

### 1. Backend — API on `http://localhost:8000`

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# run from the repo root (root + backend/ on PYTHONPATH)
PYTHONPATH="$PWD/backend:$PWD" uvicorn app.main:app --port 8000
```

### 2. Frontend — app on `http://localhost:3000`

```bash
cd web
npm install
npm run dev
```

Open **http://localhost:3000**. The frontend talks to the backend at `http://localhost:8000` by default (override with `NEXT_PUBLIC_API_BASE` in `web/.env.local`).

> The pre-computed mission results live under `Results/`. The Optimizer page lets you generate new ones; saved runs become browsable in the Mission Browser.

---

## Project layout

```
.                       # optimization engine (PathProblem, PathSolution, Sensing, …)
├─ backend/app/         # FastAPI service: routers, services, schemas, optimizer worker
├─ backend/tests/       # API tests (pytest + FastAPI TestClient)
├─ web/                 # Next.js 14 frontend (App Router, components, hooks)
├─ Results/             # pre-optimized mission data the app browses
└─ docs/                # implementation plan, screenshots
```

## Tests

```bash
# backend API
PYTHONPATH="$PWD/backend:$PWD" pytest backend/ -q

# frontend type-check
cd web && npx tsc --noEmit
```

---

## About

This is the interactive companion to my **thesis research on cooperative multi-UAV path planning for Search-and-Rescue** — built to make the results explorable and the optimizers runnable by anyone, not just reproducible from a script. Feedback and questions are welcome.
