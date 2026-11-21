# lib-spidertron-pathing

Shared spider-vehicle pathing and movement execution for Factorio 2.1:
vanilla autopilot kick for instant response, then async
`surface.request_path` refinement so routes go around impassable tiles
(water, lava, oceans) instead of pushing into them. Includes waypoint
thinning and queueing, a stall watchdog, and a goal blacklist.

This is a **library mod**: it ships no `control.lua` and registers no
events. Your mod declares it as a dependency, requires the module, and
wires the events itself.

## Usage

```json
"dependencies": ["base >= 2.1.0", "lib-spidertron-pathing >= 0.1.0"]
```

```lua
local Pathing = require("__lib-spidertron-pathing__.nav")
```

### Records

Keep one record per entity, shaped `{entity = LuaEntity, nav = <table>}`.
`nav` is lib-owned: the library writes it in `order_move`, and consumers
read `nav.failed`/`nav.goal` but never replace it directly (use
`Pathing.reset`/`Pathing.cancel`).

### Ordering a move

```lua
Pathing.order_move(rec, {x = 100, y = -40}, {
  purpose = "patrol",                    -- free string, rides on nav.purpose
  collision_mask = my_mask,              -- default: spider mask (blocks on
                                         --   `player`-layer tiles, ignores
                                         --   entities, tile transitions)
  snap_to_land = true,                   -- default true: water goals snap to
                                         --   nearest shore within 32 tiles
  min_path_dist_sq = 400,                -- below this, skip the path request
  skip_straightline_fallback = {"patrol"}, -- purposes that never straight-line
                                         --   on failure (set or array)
  request_tag = rec_key,                 -- stored in the request map; returned
                                         --   by take_request (default:
                                         --   entity.unit_number)
})
```

A bare string as the third argument is shorthand for `{purpose = str}`.

### Event wiring

```lua
script.on_event(defines.events.on_script_path_request_finished, function(event)
  local info = Pathing.take_request(event.id)
  if not info then
    return -- not a request this lib made
  end
  local rec = my_records[info.tag]       -- tag = request_tag or unit_number
  if rec then
    Pathing.on_path_finished(rec, event)
  end
end)

script.on_event(defines.events.on_spider_command_completed, function(event)
  local rec = my_records[event.vehicle.unit_number]
  if rec then
    Pathing.on_command_completed(rec)
  end
end)

-- per-record stall watchdog, on whatever cadence you already tick:
-- Pathing.tick(rec) every ~20 ticks is the proven cadence.
```

### API

| Function | Purpose |
|---|---|
| `Pathing.order_move(rec, goal, opts)` | Order a move; kicks autopilot, then refines via async path request |
| `Pathing.take_request(request_id)` | Resolve a finished path request to `{tag, purpose}`; nil = foreign request |
| `Pathing.on_path_finished(rec, event)` | Feed the raw `on_script_path_request_finished` event; staleness resolved internally |
| `Pathing.on_command_completed(rec)` | Feed `on_spider_command_completed`; advances the waypoint queue |
| `Pathing.tick(rec)` | Stall watchdog — bump waypoints, repath, or fail |
| `Pathing.arrived(rec)` | True when within ~4 tiles of `nav.goal` and ~stopped |
| `Pathing.externally_driven(rec)` | True while a foreign destination (player route, another mod) drives the entity |
| `Pathing.append_goal(rec, pos)` | Extend the current route with an extra goal (shift-click semantics) |
| `Pathing.reset(rec)` | Drop route bookkeeping; entity keeps its current autopilot orders |
| `Pathing.cancel(rec)` | Drop route bookkeeping and stop the entity |

`nav.failed` is set when a leg gives up (blacklisted goal, exhausted
retries); `nav.request_id`, `nav.waypoints`, `nav.wp_index` are lib-internal
but readable for diagnostics.

### Non-spider vehicles

Defaults target spidertrons: the collision mask blocks on `player`-layer
tiles while stepping over entities and cliffs, `entity.speed` feeds the
stall watchdog, and `entity.stop_spider()` is called on cancel. Override per
order via `opts.speed_fn(entity) -> number` and `opts.stop_fn(entity)`, plus
`opts.collision_mask`.

## License

MIT.
