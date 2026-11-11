"""모니터 접속 설정.

데이터 처리부와 같은 .env 파일에서 서버 주소와 모니터 주소를 읽는다.
처리부는 수락한 연결을 원격 주소와 포트로 구분하므로, 모니터는
COMM_MONITOR_ADDRESS:COMM_MONITOR_PORT에 바인딩한 뒤 접속해야 한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

Endpoint = Tuple[str, int]


@dataclass(frozen=True)
class MonitorConfig:
    server: Endpoint
    local: Endpoint


class ConfigError(ValueError):
    """설정 파일이나 인자가 잘못되었다."""


def read_env_file(path: Path) -> Dict[str, str]:
    """KEY=VALUE 형식의 .env 파일을 읽는다. 주석과 따옴표를 처리한다."""
    values: Dict[str, str] = {}
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ConfigError(f"{path}:{line_number}: expected KEY=VALUE")
        key, value = line.split("=", 1)
        value = value.strip()
        if value[:1] in ("'", '"') and value[-1:] == value[:1] and len(value) >= 2:
            value = value[1:-1]
        elif "#" in value:
            value = value.split("#", 1)[0].strip()
        values[key.strip()] = value
    return values


def parse_endpoint(text: str) -> Endpoint:
    """'주소:포트' 문자열을 엔드포인트로 변환한다."""
    address, separator, port_text = text.rpartition(":")
    if not separator or not address:
        raise ConfigError(f"endpoint must be ADDRESS:PORT: {text}")
    return address, _parse_port(port_text, text)


def _parse_port(text: str, source: str) -> int:
    try:
        port = int(text)
    except ValueError as error:
        raise ConfigError(f"invalid port: {source}") from error
    if not 0 < port < 65536:
        raise ConfigError(f"port out of range: {source}")
    return port


def _require(values: Dict[str, str], key: str) -> str:
    value = values.get(key, "")
    if not value:
        raise ConfigError(f"missing {key}")
    return value


def load_config(
    env_path: Optional[Path],
    server: Optional[str] = None,
    local: Optional[str] = None,
) -> MonitorConfig:
    """.env 파일 값을 읽고 명령행에서 지정한 값으로 덮어쓴다."""
    values: Dict[str, str] = {}
    if env_path is not None:
        if not env_path.is_file():
            raise ConfigError(f"env file not found: {env_path}")
        values = read_env_file(env_path)

    if server is not None:
        server_endpoint = parse_endpoint(server)
    else:
        server_endpoint = (
            _require(values, "COMM_SERVER_ADDRESS"),
            _parse_port(_require(values, "COMM_SERVER_PORT"), "COMM_SERVER_PORT"),
        )

    if local is not None:
        local_endpoint = parse_endpoint(local)
    else:
        local_endpoint = (
            _require(values, "COMM_MONITOR_ADDRESS"),
            _parse_port(_require(values, "COMM_MONITOR_PORT"), "COMM_MONITOR_PORT"),
        )

    return MonitorConfig(server_endpoint, local_endpoint)
