# Proposal: stuck detection + leg-origin pathing fallback

Status: idea — not implemented. Stolen from **SpidertronEnhancements** 1.11.0 (`scripts/pathfinder.lua`) and **Spidertron Hunter** 0.2.10 (`scripts/pathfinder.lua:415-483`).

## Problem

`nav.lua` requests one path from the entity position. If the origin is wedged (entity bounding box clipped on a rock/cliff edge) the request fails and the spider just... waits. There's also no watchdog: a spider that stops making progress with a goal pending stays stuck forever unless something else notices.

## What to steal

**Leg-origin path requests** (SE `pathfinder.lua:57-73`): spidertron legs straddle obstacles, so `request_path` once per *odd-indexed leg* (`i % 2 == 1`) using the leg's own position and `leg.prototype.collision_mask` (`colliding_with_tiles_only`, `consider_tile_transitions`). First success wins; failures aggregate per `(unit_number, start_tick)`. Fallback non-tile mask `{large_entity=true}` for legs that collide with "player" layer (`:12-22`).

**Goal snapping** (SE `:60-66`): `surface.find_non_colliding_position(leg_name, clicked_position, 10, 2)` relocates water/lava-clicks to walkable land before requesting.

**Waypoint decimation** (SE `:177-187`): re-sample path waypoints so each is ≥ `(prototype.height + 0.5) * 7.5` apart — spacing scales with leg spread; skip waypoints too close to the current position.

**Stuck watchdog** (SH `:415-483`): sample position per think; if the entity moved <1.5 tiles in ~180 ticks while a goal is pending → clear `autopilot_destination` and re-request the path. Pairs with SE's `try_again_later` same-path re-request (`:132-134`) and escalation to coarser resolution before declaring no-path (`:136-150`).

**Abort-on-external-control** (SE `:126-131`): if `autopilot_destination` changed while a request is in flight, a player (or another mod) took over — cancel the pending path application.

## How it fits the lib

`nav.lua` already does the distinctive part (leg `colliding_with_tiles_only` mask, async refine, `add_autopilot_destination` queue). Additions:

1. Multi-origin fallback: entity-origin request fails → retry from leg positions
2. `find_non_colliding_position` goal snap before requesting
3. Stuck watchdog in the per-entity record (`rec`), opt-in per consumer or built into `Pathing` — aranhas gets it free
4. External-control abort check before applying a finished path

## Open questions

- Watchdog tick budget: position sampling is cheap; which nth_tick cadence?
- Does the external-control abort interact badly with aranhas brain re-issuing orders? Need an ownership flag (last writer was Pathing).
