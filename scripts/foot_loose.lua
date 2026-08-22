-- 宽松步行 profile：对中国 OSM 数据（access tag 缺失）专门优化
-- 与默认 foot.lua 的差异：
--   1. access_tag_whitelist 为空 → 不强制要求 access=yes
--   2. access_tag_blacklist 只排除明确的"禁止" → 容忍缺失 tag
--   3. barrier 黑名单只排除 wall/fence（保留门、柱子等可通过的）

api_version = 2

Set = require('lib/set')
Sequence = require('lib/sequence')
Handlers = require("lib/way_handlers")
find_access_tag = require("lib/access").find_access_tag

function setup()
  local walking_speed = 5
  return {
    properties = {
      weight_name                   = 'duration',
      max_speed_for_map_matching    = 40/3.6,
      call_tagless_node_function    = false,
      traffic_light_penalty         = 2,
      u_turn_penalty                = 2,
      continue_straight_at_waypoint = false,
      use_turn_restrictions         = false,
    },

    default_mode            = mode.walking,
    default_speed           = walking_speed,
    oneway_handling         = 'specific',

    -- 关键修改：空 whitelist + 最小 blacklist
    -- 中国 OSM 数据大量 way 没有 access tag，默认 profile 会拒绝
    barrier_blacklist = Set {
      'wall',
      'fence',
      'yes'
    },

    -- 空 whitelist：不强制要求 access 标签
    access_tag_whitelist = Set {},

    -- 只黑名单明确拒绝的
    access_tag_blacklist = Set {
      'no',
      'private',
    },

    restricted_access_tag_list = Set { },
    restricted_highway_whitelist = Set { },
    construction_whitelist = Set{},
    service_access_tag_blacklist = Set {},

    access_tags_hierarchy = Sequence { 'foot', 'access' },
    restrictions = Sequence { 'foot' },

    suffix_list = Set {
      'N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW',
      'North', 'South', 'West', 'East'
    },

    avoid = Set { 'impassable' },

    speeds = Sequence {
      highway = {
        primary         = walking_speed,
        primary_link    = walking_speed,
        secondary       = walking_speed,
        secondary_link  = walking_speed,
        tertiary        = walking_speed,
        tertiary_link   = walking_speed,
        unclassified    = walking_speed,
        residential     = walking_speed,
        road            = walking_speed,
        living_street   = walking_speed,
        service         = walking_speed,
        track           = walking_speed,
        path            = walking_speed,
        steps           = walking_speed,
        pedestrian      = walking_speed,
        footway         = walking_speed,
        pier            = walking_speed,
      },
      railway = { platform = walking_speed },
      amenity = { parking = walking_speed, parking_entrance = walking_speed },
      man_made = { pier = walking_speed },
      leisure = { track = walking_speed }
    },

    route_speeds = { ferry = 5 },
    bridge_speeds = {},
    surface_speeds = {
      fine_gravel = walking_speed*0.75,
      gravel = walking_speed*0.75,
      pebblestone = walking_speed*0.75,
      mud = walking_speed*0.5,
      sand = walking_speed*0.5
    },
    tracktype_speeds = {},
    smoothness_speeds = {}
  }
end

function process_node(profile, node, result)
  local access = find_access_tag(node, profile.access_tags_hierarchy)
  if access then
    if profile.access_tag_blacklist[access] then
      result.barrier = true
    end
  else
    local barrier = node:get_value_by_key("barrier")
    if barrier then
      local bollard = node:get_value_by_key("bollard")
      local rising_bollard = bollard and "rising" == bollard
      if profile.barrier_blacklist[barrier] and not rising_bollard then
        result.barrier = true
      end
    end
  end
  local tag = node:get_value_by_key("highway")
  if "traffic_signals" == tag then
    result.traffic_lights = true
  end
end

function process_way(profile, way, result)
  local data = {
    highway = way:get_value_by_key('highway'),
    bridge = way:get_value_by_key('bridge'),
    route = way:get_value_by_key('route'),
    leisure = way:get_value_by_key('leisure'),
    man_made = way:get_value_by_key('man_made'),
    railway = way:get_value_by_key('railway'),
    platform = way:get_value_by_key('platform'),
    amenity = way:get_value_by_key('amenity'),
    public_transport = way:get_value_by_key('public_transport')
  }
  if next(data) == nil then return end

  local handlers = Sequence {
    WayHandlers.default_mode,
    WayHandlers.blocked_ways,
    WayHandlers.access,
    WayHandlers.oneway,
    WayHandlers.destinations,
    WayHandlers.ferries,
    WayHandlers.movables,
    WayHandlers.speed,
    WayHandlers.surface,
    WayHandlers.classification,
    WayHandlers.roundabouts,
    WayHandlers.startpoint,
    WayHandlers.names,
    WayHandlers.weights
  }
  WayHandlers.run(profile, way, result, data, handlers)
end

function process_turn (profile, turn)
  turn.duration = 0.
  if turn.direction_modifier == direction_modifier.u_turn then
     turn.duration = turn.duration + profile.properties.u_turn_penalty
  end
end

return {
  setup = setup,
  process_way = process_way,
  process_node = process_node,
  process_turn = process_turn
}
