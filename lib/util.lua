--- Vendored geometry/tile helpers used by nav.lua. Kept private to the
--- library so consumers never resolve a second module.
local Util = {}

function Util.distance_sq(a, b)
  local dx = a.x - b.x
  local dy = a.y - b.y
  return dx * dx + dy * dy
end

--- True when the tile at pos blocks legged traversal. `player` is the
--- discriminating layer: deep water, lava, ammonia ocean and out-of-map all
--- carry it; Fulgora's oil ocean and Gleba shallows don't — they're crust,
--- spiders (and players) walk them at reduced speed.
function Util.is_blocked(surface, pos)
  local ok, res = pcall(function()
    local tile = surface.get_tile(pos.x, pos.y)
    if not tile then
      return false
    end
    local mask = tile.prototype.collision_mask
    return mask and mask.layers and mask.layers["player"] == true or false
  end)
  return ok and res or false
end

--- Nearest non-blocked tile to pos within max_radius (expanding rings on
--- 8 spokes). Returns pos unchanged when it's already passable, and pos
--- unchanged as a last resort when only blocked tiles are found.
function Util.nearest_land(surface, pos, max_radius)
  max_radius = max_radius or 32
  if not Util.is_blocked(surface, pos) then
    return pos
  end
  for r = 4, max_radius, 4 do
    for i = 0, 7 do
      local a = i * (math.pi / 4)
      local p = {x = pos.x + math.cos(a) * r, y = pos.y + math.sin(a) * r}
      if not Util.is_blocked(surface, p) then
        return p
      end
    end
  end
  return pos
end

return Util
