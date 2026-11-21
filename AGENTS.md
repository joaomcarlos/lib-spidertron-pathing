# AGENTS.md

Guidance for AI agents working on this Factorio mod.

## Project Context

`lib-spidertron-pathing` is a **library mod** for Factorio 2.1. There is no
`control.lua` and nothing is registered on load — the mod only ships Lua
modules that consumers `require`. Consumers declare it in their `info.json`
dependencies, keep per-entity records shaped `{entity=LuaEntity, nav=...}`,
and wire their own events (`on_script_path_request_finished`,
`on_spider_command_completed`, and a per-record tick loop that calls
`Pathing.tick`).

Lua 5.2 (Factorio's modified variant — `goto` is legal even if the local
`luac` rejects it).

## Layout

- `nav.lua` — the public `Pathing` module (`require("__lib-spidertron-pathing__.nav")`)
- `lib/util.lua` — vendored helpers private to the library; consumers must
  not rely on it

Persistent lib state lives under `storage.lib_pathing` (`requests`,
`blacklist`), created lazily. The lib owns no lifecycle — never add
`on_init`/`on_load`/event handlers to this mod.

## Factorio API Rules

- Verify API shapes against `https://lua-api.factorio.com/latest/` before
  using them. A plausible Lua shape that is not the real shape is the most
  common bug source.
- `storage` persists through save/load; ordinary globals do not.
- LuaObjects stored in `storage` must be checked with `.valid` before use.

## Coding Style

- Fail fast. No defensive nil checks unless the API documents the value can
  actually be nil.
- No backwards-compatibility shims. If it is not used, delete it.
- Keep docstring/comment style (`--- CONTRACT` headers).
- Changes here affect every consumer — keep the `Pathing` surface stable
  and prefer new opts over new required arguments.
