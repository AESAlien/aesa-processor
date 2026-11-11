import tempfile
import unittest
from pathlib import Path

from monitor.config import ConfigError, load_config, parse_endpoint

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


class ConfigTest(unittest.TestCase):
    def write_env(self, text):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / ".env"
        path.write_text(text, encoding="utf-8")
        return path

    def test_reads_server_and_monitor_endpoints_from_example(self):
        config = load_config(REPOSITORY_ROOT / ".env.example")

        self.assertEqual(config.server, ("192.168.10.20", 3200))
        self.assertEqual(config.local, ("192.168.10.30", 3400))

    def test_handles_comments_and_quotes(self):
        path = self.write_env(
            "# comment\n"
            "COMM_SERVER_ADDRESS='127.0.0.1'\n"
            "COMM_SERVER_PORT=43200 # inline\n"
            'COMM_MONITOR_ADDRESS="127.0.0.4"\n'
            "COMM_MONITOR_PORT=43203\n"
        )

        config = load_config(path)

        self.assertEqual(config.server, ("127.0.0.1", 43200))
        self.assertEqual(config.local, ("127.0.0.4", 43203))

    def test_arguments_override_env_values(self):
        config = load_config(REPOSITORY_ROOT / ".env.example", "127.0.0.1:1", "127.0.0.2:2")

        self.assertEqual(config.server, ("127.0.0.1", 1))
        self.assertEqual(config.local, ("127.0.0.2", 2))

    def test_rejects_missing_or_invalid_values(self):
        with self.assertRaises(ConfigError):
            load_config(self.write_env("COMM_SERVER_ADDRESS=127.0.0.1\n"))
        with self.assertRaises(ConfigError):
            load_config(None)
        with self.assertRaises(ConfigError):
            parse_endpoint("127.0.0.1")
        with self.assertRaises(ConfigError):
            parse_endpoint("127.0.0.1:70000")


if __name__ == "__main__":
    unittest.main()
