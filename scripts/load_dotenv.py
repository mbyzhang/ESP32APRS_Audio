Import("env")
import os


def _parse_dotenv(path):
    values = {}
    with open(path, "r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, val = line.split("=", 1)
            key = key.strip()
            val = val.strip()
            if len(val) >= 2 and ((val[0] == '"' and val[-1] == '"') or (val[0] == "'" and val[-1] == "'")):
                val = val[1:-1]
            values[key] = val
    return values


def _cpp_define_string(name, value):
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return f'{name}=\\"{escaped}\\"'


project_dir = env.subst("$PROJECT_DIR")
dotenv_path = os.path.join(project_dir, ".env")

if os.path.isfile(dotenv_path):
    data = _parse_dotenv(dotenv_path)

    sta_ssid = data.get("WIFI_STA_SSID") or data.get("WIFISSID") or data.get("WIFI_SSID")
    sta_pass = data.get("WIFI_STA_PASS") or data.get("WIFIPWD") or data.get("WIFI_PASSWORD")
    ap_ssid = data.get("WIFI_AP_SSID") or data.get("APSSID")
    ap_pass = data.get("WIFI_AP_PASS") or data.get("APPWD")
    host_name = data.get("WIFI_HOSTNAME") or data.get("DEVICE_HOSTNAME") or data.get("HOST_NAME")

    defines = []
    if sta_ssid:
        defines.append(_cpp_define_string("DEFAULT_WIFI_STA_SSID", sta_ssid))
        defines.append(_cpp_define_string("DEFAULT_WIFI_STA_PASS", sta_pass or ""))
        defines.append("DOTENV_WIFI_DEFAULTS")
    if ap_ssid:
        defines.append(_cpp_define_string("DEFAULT_WIFI_AP_SSID", ap_ssid))
    if ap_pass:
        defines.append(_cpp_define_string("DEFAULT_WIFI_AP_PASS", ap_pass))
    if host_name:
        defines.append(_cpp_define_string("DEFAULT_HOST_NAME", host_name))

    if defines:
        env.Append(CPPDEFINES=defines)
        print("[dotenv] Loaded WiFi defaults from .env")
