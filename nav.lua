--- CONTRACT
--- Pathfinding and movement execution for one vehicle record.
--- Goals: the entity starts moving immediately (vanilla autopilot), then the
--- route is refined async via surface.request_path so it routes around
--- impassable tiles instead of pushing into them. Stalls are detected and
--- re-paths issued.
---
--- Library module: no control.lua, no event registrations, no lifecycle
--- hooks. Consumers require("__lib-spidertron-pathing__.nav"), keep
--- per-entity records shaped {entity=LuaEntity, nav=<lib-owned table>},
--- and wire three entry points:
---   on_script_path_request_finished -> Pathing.take_request(event.id) to
---     find the owning record (nil = a request this lib never made), then
---     Pathing.on_path_finished(rec, event)
---   on_spider_command_completed     -> Pathing.on_command_completed(rec)
---   a per-record tick loop          -> Pathing.tick(rec)
local Util = require("__lib-spidertron-pathing__.lib.util")

local Pathing = {}

local STALL_TICKS = 120        --- ~2s of ~zero speed while far from goal -> stalled
local STALL_SPEED = 0.05       --- |speed| below this counts as stopped
local STALL_GOAL_DIST_SQ = 36  --- stall watchdog only watches beyond ~6 tiles
local MAX_REPATHS = 3          --- stall-driven re-requests before giving up
local MAX_RETRIES = 2          --- re-requests after failed/try_again results
local ARRIVE_DIST_SQ = 16      --- ~4 tiles
local WAYPOINT_REACH_SQ = 9    --- waypoint counts as reached within ~3 tiles
local MIN_PATH_DIST_SQ = 400   --- closer than ~20 tiles: vanilla autopilot is enough
local REQUEST_TIMEOUT = 300    --- ~5s: a request older than this is treated as lost
local GOAL_BLACKLIST_TTL = 18000 --- ~5min: must outlast a fallback round-trip or the next spider just retries the dead goal
local GOAL_KEY_Q = 8            --- goal positions quantized to 8-tile cells

--- Lib-owned persistent state, namespaced so consumers' storage keys stay
--- untouched. Both tables are created lazily — the lib owns no lifecycle,
--- so nothing may run at init/load time.
local function lib_storage()
  storage.lib_pathing = storage.lib_pathing or {}
  return storage.lib_pathing
end

local function request_map()
  local s = lib_storage()
  s.requests = s.requests or {}
  return s.requests
end

local function blacklist_map()
  local s = lib_storage()
  s.blacklist = s.blacklist or {}
  return s.blacklist
end

local function goal_key(surface_index, pos)
  return surface_index .. ":" .. math.floor(pos.x / GOAL_KEY_Q) .. ":" .. math.floor(pos.y / GOAL_KEY_Q)
end

local function goal_blacklisted(entity, pos)
  local s = storage.lib_pathing
  local bl = s and s.blacklist
  local t = bl and bl[goal_key(entity.surface.index, pos)]
  return t ~= nil and (game.tick - t) < GOAL_BLACKLIST_TTL
end

--- Collision mask for the default spider path requests. `player` is the
--- layer that makes a tile impassable to a walker: deep water, lava,
--- ammonia ocean and out-of-map all carry it, while Fulgora oil ocean and
--- Gleba shallows lack it — spiders walk those (crust, reduced speed).
--- colliding_with_tiles_only ignores every entity: legs step over cliffs,
--- trees, buildings, trains. consider_tile_transitions checks the box
--- center rather than the whole box, so the torso may overhang a shoreline
--- while legs stay on land.
local SPIDER_PATH_MASK = {
  layers = {["player"] = true},
  colliding_with_tiles_only = true,
  consider_tile_transitions = true,
}

local DEFAULT_SKIP_FALLBACK = {patrol = true}

--- Legs of a spider vehicle, nil for non-spider callers (the API errors
--- off SpiderVehicle — a documented failure mode, not defensive niling).
local function spider_legs(entity)
  local ok, legs = pcall(function()
    return entity.get_spider_legs()
  end)
  if ok then
    return legs
  end
end

--- Snap a goal sitting on blocked terrain to the nearest spot a leg can
--- stand. find_non_colliding_position models the leg's real box and mask,
--- so shorelines the tile probe in nearest_land can miss still resolve.
--- Falls back to the ring scan for legless callers or no spot found.
local function snap_goal(entity, goal)
  local legs = spider_legs(entity)
  if legs and legs[1] and legs[1].valid then
    local pos = entity.surface.find_non_colliding_position(legs[1].name, goal, 10, 2)
    if pos then
      return pos
    end
  end
  return Util.nearest_land(entity.surface, goal, 32)
end

--- Vehicle hooks: spidertron defaults, overridable per order via opts so
--- future non-spider callers can plug in their own accessors.
local function entity_speed(entity, opts)
  if opts and opts.speed_fn then
    return math.abs(opts.speed_fn(entity) or 0)
  end
  return math.abs(entity.speed or 0)
end

local function stop_entity(entity, opts)
  if opts and opts.stop_fn then
    opts.stop_fn(entity)
    return
  end
  entity.stop_spider()
end

--- Normalize order_move opts once; the result rides on nav.opts so tick,
--- on_path_finished and cancel reuse the same settings for the whole leg.
--- A bare string opts is accepted as shorthand for {purpose = string}.
local function resolve_opts(opts)
  if type(opts) == "string" then
    opts = {purpose = opts}
  end
  opts = opts or {}
  local skip = opts.skip_straightline_fallback or DEFAULT_SKIP_FALLBACK
  if skip[1] ~= nil then
    local set = {}
    for _, p in ipairs(skip) do
      set[p] = true
    end
    skip = set
  end
  return {
    purpose = opts.purpose,
    collision_mask = opts.collision_mask or SPIDER_PATH_MASK,
    snap_to_land = opts.snap_to_land ~= false,
    min_path_dist_sq = opts.min_path_dist_sq or MIN_PATH_DIST_SQ,
    skip_straightline_fallback = skip,
    request_tag = opts.request_tag,
    speed_fn = opts.speed_fn,
    stop_fn = opts.stop_fn,
  }
end

--- Resolve a finished path request id to its owner ({tag, purpose}).
--- Consumers call this from on_script_path_request_finished to find their
--- record; nil means the request wasn't made by this lib. Also un-tracks
--- the id, so stale completions die here.
---@param request_id number
---@return table?  {tag = opts.request_tag or entity.unit_number, purpose = string}
function Pathing.take_request(request_id)
  local s = storage.lib_pathing
  local reqs = s and s.requests
  if not reqs then
    return nil
  end
  local info = reqs[request_id]
  reqs[request_id] = nil
  return info
end

local function track_request(request_id, tag, purpose)
  request_map()[request_id] = {tag = tag, purpose = purpose}
end

--- Issue an async path request for the entity's current nav.goal and track
--- it. Any previous pending request is un-tracked first so at most one
--- request per record is live: stale completions then die inside
--- Pathing.take_request. Failures are silent: vanilla autopilot is already
--- walking to nav.goal.
local function request_path(rec)
  local nav = rec.nav
  local entity = rec.entity
  if not (nav and nav.goal and entity and entity.valid) then
    return
  end
  local opts = nav.opts or {}
  local min_dist_sq = opts.min_path_dist_sq or MIN_PATH_DIST_SQ
  --- Short hops gain nothing from pathfinding; keep the request budget for
  --- the long water/cliff detours that actually need it.
  if Util.distance_sq(entity.position, nav.goal) < min_dist_sq then
    return
  end
  --- Combat orbit/evade updates shift the goal a few tiles every 20 ticks;
  --- don't churn the pathfinder for them. When the goal barely moved and a
  --- request is already in flight, keep waiting for it — orphaning it and
  --- refusing to re-request left spiders standing forever.
  if nav.path_goal and Util.distance_sq(nav.path_goal, nav.goal) < min_dist_sq then
    return
  end
  if nav.request_id then
    Pathing.take_request(nav.request_id)
    nav.request_id = nil
  end
  nav.path_goal = {x = nav.goal.x, y = nav.goal.y}
  local ok, request_id = pcall(function()
    return entity.surface.request_path {
      bounding_box = entity.prototype.collision_box,
      collision_mask = opts.collision_mask or SPIDER_PATH_MASK,
      start = entity.position,
      goal = nav.goal,
      force = entity.force,
      --- Without this the requester's own box blocks the start position:
      --- every path comes back empty.
      entity_to_ignore = entity,
    }
  end)
  if ok and request_id then
    nav.request_id = request_id
    nav.request_since = game.tick
    track_request(request_id, opts.request_tag or entity.unit_number, nav.purpose or "command")
  elseif not ok then
    log("lib-spidertron-pathing: request_path failed: " .. tostring(request_id))
  end
end

--- Advance nav.wp_index past every waypoint the entity has effectively
--- reached (bounded ~3 tiles, so it can't skip across water).
local function sync_wp_index(entity, nav)
  local wps = nav.waypoints
  if not (wps and nav.wp_index) then
    return
  end
  local pos = entity.position
  while nav.wp_index <= #wps
    and Util.distance_sq(wps[nav.wp_index], pos) <= WAYPOINT_REACH_SQ do
    nav.wp_index = nav.wp_index + 1
  end
end

--- Point the autopilot at wps[wp_index]; queue the remainder behind it when
--- add_autopilot_destination is available (2.1), else the queue is fed one
--- node at a time from on_command_completed.
local function feed_waypoints(entity, nav)
  local wps = nav.waypoints
  local idx = nav.wp_index
  if not (wps and idx and wps[idx]) then
    return
  end
  local ok = pcall(function()
    entity.autopilot_destination = wps[idx]
  end)
  if not ok then
    return
  end
  if nav.use_queue ~= false then
    for i = idx + 1, #wps do
      local added, err = pcall(function()
        entity.add_autopilot_destination(wps[i])
      end)
      if not added then
        log("lib-spidertron-pathing: add_autopilot_destination failed: " .. tostring(err))
        nav.use_queue = false --- API unavailable: fall back to manual stepping
        break
      end
    end
  end
end

--- Order an entity to `goal`. Must:
---  - set rec.nav = {goal=goal, request_id=?, waypoints=nil, wp_index=?,
---    stall_since=?, repath_count=0, opts=resolved}
---  - immediately set entity.autopilot_destination = goal so it starts walking
---  - fire surface.request_path (bounding_box from entity prototype,
---    collision mask appropriate for spider-vehicle: blocks on `player`-layer
---    tiles — water/lava/oceans — and steps over every entity, cliffs
---    included); register via track_request so take_request can resolve it
---  - if request_path errors (pcall), leave vanilla autopilot to handle it
---@param rec table    {entity=LuaEntity, nav=lib-owned table}
---@param goal MapPosition
---@param opts table|string?  {purpose=string, collision_mask=table
---       (default SPIDER_PATH_MASK), snap_to_land=bool (default true),
---       min_path_dist_sq=n, skip_straightline_fallback=set|array
---       (default {"patrol"}), request_tag=any (default unit_number),
---       speed_fn=fun(entity)->n, stop_fn=fun(entity)};
---       a bare string is treated as {purpose = string}
function Pathing.order_move(rec, goal, opts)
  local entity = rec.entity
  if not (entity and entity.valid and goal) then
    return
  end
  local o = resolve_opts(opts)
  --- A superseding order invalidates any in-flight request for the old goal.
  local old_nav = rec.nav
  if old_nav and old_nav.request_id then
    Pathing.take_request(old_nav.request_id)
  end
  --- A goal sitting on water is unreachable: snap it to the nearest shore
  --- (patrol/engage points, stray remote clicks) instead of stalling.
  local snapped = o.snap_to_land and snap_goal(entity, goal) or goal
  rec.nav = {
    goal = {x = snapped.x, y = snapped.y},
    request_id = nil,
    waypoints = nil,
    wp_index = nil,
    stall_since = nil,
    repath_count = 0,
    retry_count = 0,
    use_queue = nil,
    purpose = o.purpose,
    opts = o,
  }
  local nav = rec.nav
  --- Compute the route FIRST: the entity waits for the path and then walks
  --- every queued node, instead of straight-lining toward the goal and
  --- pressing water while the request is in flight. Clear any queued
  --- destinations first so a superseded leg isn't walked.
  pcall(function()
    entity.autopilot_destination = nil
  end)
  --- Entity standing on a blocked tile: the requester position collides, so
  --- every request fails. Point it at the goal — with shore in leg reach it
  --- walks itself out on vanilla autopilot; otherwise it stalls into the
  --- watchdog and blacklist.
  if Util.is_blocked(entity.surface, entity.position) then
    pcall(function()
      entity.autopilot_destination = snapped
    end)
    return
  end
  --- This goal just failed for someone: don't burn another request on it —
  --- straight-line there (empty-path rate was ~5/s of retry spam).
  if goal_blacklisted(entity, snapped) then
    pcall(function()
      entity.autopilot_destination = snapped
    end)
    nav.failed = true
    return
  end
  request_path(rec)
  --- No request fired: short hop, or the API rejected the call — straight
  --- line is the only option left.
  if not nav.request_id then
    pcall(function()
      entity.autopilot_destination = snapped
    end)
  end
end

--- Feed a finished path to the owning record: convert path nodes into
--- autopilot waypoints (entity.add_autopilot_destination if available in
--- 2.1, else sequence autopilot_destination as nodes are reached). Call it
--- from the consumer's on_script_path_request_finished handler after
--- Pathing.take_request resolves the record; pass the raw event — staleness
--- (a superseded leg's request id) is resolved internally.
---@param rec table
---@param event table  on_script_path_request_finished event
---       (event.path may be nil on failure, event.try_again_later, event.id)
function Pathing.on_path_finished(rec, event)
  local nav = rec.nav
  if not nav then
    return
  end
  if event.id and nav.request_id ~= event.id then
    return --- stale: a newer request owns this route
  end
  nav.request_id = nil
  local entity = rec.entity
  if not (entity and entity.valid and nav.goal) then
    return
  end
  local opts = nav.opts or {}
  local path = event.path

  if event.try_again_later then
    --- Pathfinder is busy, not failed — with many spiders queuing requests
    --- this is the common case. Keep re-asking on a separate budget;
    --- treating it as a failure was pushing every spider to vanilla
    --- straight-lines over water.
    nav.busy_retries = (nav.busy_retries or 0) + 1
    if nav.busy_retries <= 10 then
      nav.path_goal = nil
      request_path(rec)
    else
      nav.failed = true
    end
    return
  end
  if not path or #path == 0 then
    --- Rate-limited diagnostic: position+surface pinpoints the failing
    --- area without flooding the log at ~5/sec.
    if game.tick % 600 == 0 then
      log("lib-spidertron-pathing: path empty on " .. entity.surface.name .. " goal=" .. math.floor(nav.goal.x) .. "," .. math.floor(nav.goal.y))
    end
    nav.retry_count = (nav.retry_count or 0) + 1
    if nav.retry_count <= MAX_RETRIES then
      nav.path_goal = nil --- retry must bypass the same-goal dedupe
      request_path(rec)
    else
      --- No passable route exists. Goals whose purpose is in the
      --- skip_straightline_fallback set (patrol by default: the ring point
      --- is dead-marked and skipped anyway) don't get the fallback — only
      --- purposes that need arrival (mission, resupply, command) do.
      blacklist_map()[goal_key(entity.surface.index, nav.goal)] = game.tick
      if not opts.skip_straightline_fallback[nav.purpose] then
        pcall(function()
          entity.autopilot_destination = nav.goal
        end)
      end
      nav.failed = true
    end
    return
  end

  --- Thin to >=MIN_NODE_GAP between queued nodes: the pathfinder returns
  --- nodes ~1.4 tiles apart and the autopilot orbits each one — a fast
  --- spider visibly loops around every micro-destination (verified live:
  --- one spider carried a 103-node queue). Keep every ~10th tile and
  --- always the final node.
  local MIN_NODE_GAP_SQ = 100
  local wps = {}
  local last = entity.position
  local raw = {}
  for _, node in pairs(path) do
    if node.position then
      raw[#raw + 1] = {x = node.position.x, y = node.position.y}
    end
  end
  for i, node in ipairs(raw) do
    if i == #raw or Util.distance_sq(last, node) >= MIN_NODE_GAP_SQ then
      wps[#wps + 1] = node
      last = node
    end
  end
  if #wps == 0 then
    nav.retry_count = (nav.retry_count or 0) + 1
    if nav.retry_count <= MAX_RETRIES then
      nav.path_goal = nil
      request_path(rec)
    else
      nav.failed = true
    end
    return
  end
  --- The pathfinder only gets within `radius` of the goal; append the exact
  --- goal so the final leg doesn't stop short.
  if Util.distance_sq(wps[#wps], nav.goal) > WAYPOINT_REACH_SQ then
    wps[#wps + 1] = {x = nav.goal.x, y = nav.goal.y}
  end
  nav.waypoints = wps
  nav.wp_index = 1
  nav.stall_since = nil
  nav.retry_count = 0
  sync_wp_index(entity, nav)
  if nav.wp_index <= #wps then
    feed_waypoints(entity, nav)
  end
end

--- Per-call-site stall watchdog. If speed ~0 while far from goal for
--- N ticks, bump waypoint or re-request path (max ~3 repaths, then give up
--- and let vanilla autopilot grind). Must be cheap: runs every ~20 ticks
--- per entity.
function Pathing.tick(rec)
  local nav = rec.nav
  if not (nav and nav.goal) then
    return
  end
  local entity = rec.entity
  if not (entity and entity.valid) then
    return
  end

  --- A path request is in flight: standing still is expected, not a stall —
  --- unless the request is older than the timeout, in which case it is lost
  --- (event never resolves) and we fall through to repath/fail handling.
  if nav.request_id then
    if (game.tick - (nav.request_since or 0)) <= REQUEST_TIMEOUT then
      return
    end
    Pathing.take_request(nav.request_id)
    nav.request_id = nil
    nav.path_goal = nil
  end

  --- Last-meter settle: the engine drops the autopilot destination at its
  --- own reach tolerance, so a fast spider coasts past the goal. Once the
  --- route is consumed (or a straight-line order), re-assert the goal
  --- every time the destination lapses or the spider stops short — each
  --- assert is a real brake-and-settle approach like the player clicking
  --- again, not a mid-flight pass-through. Bounded; beyond it the stall
  --- watchdog repaths as before.
  local route_done = not nav.waypoints or (nav.wp_index or 1) > #nav.waypoints
  if route_done and nav.goal and not nav.failed and not Pathing.arrived(rec) then
    local dest = entity.autopilot_destination
    if (not dest or Util.distance_sq(dest, nav.goal) > 4)
      and (nav.settle_count or 0) < 6 then
      nav.settle_count = (nav.settle_count or 0) + 1
      pcall(function()
        entity.autopilot_destination = nav.goal
      end)
      return
    end
  end

  local speed = entity_speed(entity, nav.opts)
  if speed > STALL_SPEED then
    nav.stall_since = nil
    return
  end
  if Util.distance_sq(entity.position, nav.goal) <= STALL_GOAL_DIST_SQ then
    nav.stall_since = nil
    return
  end

  local now = game.tick
  nav.stall_since = nav.stall_since or now
  if now - nav.stall_since < STALL_TICKS then
    return
  end
  nav.stall_since = now

  --- Stalled: first try skipping the waypoint it's stuck on.
  if nav.waypoints and nav.wp_index then
    sync_wp_index(entity, nav)
    if nav.wp_index < #nav.waypoints then
      nav.wp_index = nav.wp_index + 1
      feed_waypoints(entity, nav)
      return
    end
  end

  --- Out of waypoints (or none yet): re-request the path a few times, then
  --- give up — never straight-line the goal.
  nav.repath_count = (nav.repath_count or 0) + 1
  if nav.repath_count <= MAX_REPATHS then
    nav.retry_count = 0
    nav.path_goal = nil --- re-request must bypass the same-goal dedupe
    request_path(rec)
  else
    --- Same rule: straight-line fallback beats freezing — the autopilot
    --- straddles what it can — but only for goals that need arrival.
    --- Skipped purposes (patrol by default) get dead-marked and skipped.
    if not nav.opts.skip_straightline_fallback[nav.purpose] then
      pcall(function()
        entity.autopilot_destination = nav.goal
      end)
    end
    nav.failed = true
  end
end

--- True when the entity's current autopilot destination wasn't issued by us
--- (a vanilla waypoint queue, another mod). Consumers gate auto-orders on
--- this so player-set routes are never stomped; it clears when the queue
--- drains (autopilot_destination goes nil) and control resumes.
local EXTERNAL_GRACE = 3600 --- a foreign route older than ~60s is released

function Pathing.externally_driven(rec)
  local entity = rec.entity
  if not (entity and entity.valid) then
    return false
  end
  local nav = rec.nav or {}

  --- Vanilla patrol loops and queued waypoint chains are never claimed —
  --- they refresh forever and belong to whoever set them.
  local ok, sticky = pcall(function()
    local queued = entity.autopilot_destinations
    return (entity.autopilot_patrol_size or 0) > 0 or (queued and #queued > 1)
  end)
  if ok and sticky then
    return true
  end

  local ok2, dest = pcall(function()
    return entity.autopilot_destination
  end)
  if not ok2 or not dest then
    nav.ext_since = nil
    return false --- nothing driving it
  end
  local function same(p)
    return p and math.abs(p.x - dest.x) < 0.75 and math.abs(p.y - dest.y) < 0.75
  end
  local external
  if not nav.goal then
    external = true --- destination exists but we never ordered one
  elseif nav.waypoints then
    external = true
    for i = nav.wp_index or 1, #nav.waypoints do
      if same(nav.waypoints[i]) then
        external = false --- walking one of our refined nodes
        break
      end
    end
  else
    external = not same(nav.goal)
  end
  if not external then
    nav.ext_since = nil
    return false
  end
  --- Grace release: a stale destination held for a while lets the consumer
  --- take over (order_move clears it anyway). Live user routes keep
  --- changing their destination, which resets the clock implicitly.
  nav.ext_since = nav.ext_since or game.tick
  return (game.tick - nav.ext_since) <= EXTERNAL_GRACE
end

--- True when the entity is within ~4 tiles of nav.goal and ~stopped.
function Pathing.arrived(rec)
  local nav = rec.nav
  if not (nav and nav.goal) then
    return false
  end
  local entity = rec.entity
  if not (entity and entity.valid) then
    return false
  end
  if Util.distance_sq(entity.position, nav.goal) > ARRIVE_DIST_SQ then
    return false
  end
  return entity_speed(entity, nav.opts) <= STALL_SPEED
end

--- Call on defines.events.on_spider_command_completed (entity reached its
--- current autopilot destination). Advance to the next queued waypoint or
--- mark arrival; consumers notice arrival via Pathing.arrived.
function Pathing.on_command_completed(rec)
  local nav = rec.nav
  if not (nav and nav.goal) then
    return
  end
  local entity = rec.entity
  if not (entity and entity.valid) then
    return
  end
  nav.stall_since = nil
  if not (nav.waypoints and nav.wp_index) then
    return
  end
  sync_wp_index(entity, nav)
  if nav.wp_index > #nav.waypoints then
    --- The engine drops the last destination the moment its reach
    --- tolerance hits, so a fast spider coasts past the goal with nothing
    --- pulling it back. Re-assert the goal (bounded) so the autopilot does
    --- a fresh brake-and-settle onto it — same as the player clicking the
    --- destination again. Skipped once actually arrived/stopped.
    if (nav.settle_count or 0) < 3 and not Pathing.arrived(rec) then
      nav.settle_count = (nav.settle_count or 0) + 1
      pcall(function()
        entity.autopilot_destination = nav.goal
      end)
    end
    return
  end
  --- With queue feeding the engine is already targeting this node; only push
  --- it ourselves when nothing matching is queued (manual-stepping fallback).
  local ok, queued = pcall(function()
    return entity.autopilot_destinations
  end)
  if ok and queued then
    for _, q in pairs(queued) do
      if Util.distance_sq(q, nav.waypoints[nav.wp_index]) <= 4 then
        return
      end
    end
  end
  feed_waypoints(entity, nav)
end

--- Append a player-clicked waypoint to the end of the current route
--- (shift+click). Vanilla already queued the node on the autopilot, so the
--- entity walks the existing path then continues to the click — we only
--- extend the nav bookkeeping so arrival/stall logic treats it as part of
--- this route.
function Pathing.append_goal(rec, position)
  local nav = rec.nav
  if not nav then
    return
  end
  nav.waypoints = nav.waypoints or {}
  nav.waypoints[#nav.waypoints + 1] = position
  nav.goal = position
  nav.failed = nil
  nav.settle_count = nil --- extended route gets its own settle budget
end

--- Drop route bookkeeping without touching the entity: un-tracks any
--- in-flight path request and wipes nav. Use instead of bare `nav = {}` so
--- stale request completions can't corrupt a newer route.
function Pathing.reset(rec)
  local nav = rec.nav
  if nav and nav.request_id then
    Pathing.take_request(nav.request_id)
  end
  rec.nav = {}
end

--- Cancel any pending route (clears autopilot_destination too).
function Pathing.cancel(rec)
  if not rec then
    return
  end
  local nav = rec.nav
  if nav and nav.request_id then
    Pathing.take_request(nav.request_id)
  end
  local entity = rec.entity
  if entity and entity.valid then
    pcall(function()
      entity.autopilot_destination = nil
      stop_entity(entity, nav and nav.opts)
    end)
  end
  rec.nav = {}
end

return Pathing
