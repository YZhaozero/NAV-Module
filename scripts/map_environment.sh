#!/usr/bin/env bash
# Sourced by the device launcher. Values are data; never execute this file.
nav_map_config="${NAV_MAP_CONFIG:-$HOME/.config/nav/maps.env}"
if [[ -f "$nav_map_config" ]]; then
  while IFS= read -r nav_map_setting || [[ -n "$nav_map_setting" ]]; do
    nav_map_setting="${nav_map_setting%$'\r'}"
    nav_map_setting="${nav_map_setting#$'\xef\xbb\xbf'}"
    case "$nav_map_setting" in
      NAV_BAIDU_BROWSER_AK=*|NAV_BAIDU_SERVER_AK=*|NAV_BAIDU_SERVER_SK=*|NAV_MAP_MODE=*|NAV_MAP_GATEWAY=*|NAV_MAP_TOKEN=*|NAV_OUTDOOR_DATA=*|NAV_OUTDOOR_CALIBRATION=*)
        nav_map_name="${nav_map_setting%%=*}"
        nav_map_value="${nav_map_setting#*=}"
        if [[ ${#nav_map_value} -ge 2 ]] && [[ "$nav_map_value" == \"*\" || "$nav_map_value" == \'*\' ]]; then
          nav_map_value="${nav_map_value:1:${#nav_map_value}-2}"
        fi
        export "$nav_map_name=$nav_map_value"
        ;;
    esac
  done < "$nav_map_config"
fi
unset nav_map_config nav_map_setting nav_map_name nav_map_value
