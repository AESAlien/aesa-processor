import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from tools.eval.__main__ import main
from tools.eval.tests.test_scenario import FIXTURE


class CliTest(unittest.TestCase):
    def test_inspect_options_and_query_with_space_in_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ini = root / "scenario with spaces.ini"
            ini.write_bytes(FIXTURE.replace("\n", "\r").encode())
            options = root / "model.json"
            main(["options", "--output", str(options)])
            model = json.loads(options.read_text(encoding="utf-8"))
            model["maneuver_actions"] = ["ignore"] * 4
            options.write_text(json.dumps(model), encoding="utf-8")
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                main(["inspect", "--ini", str(ini)])
            self.assertEqual(len(json.loads(output.getvalue())["targets"]), 1)
            warning = io.StringIO()
            destination = root / "result.json"
            with contextlib.redirect_stderr(warning):
                main(["predict", "--ini", str(ini), "--options", str(options), "--target", "1",
                      "--time", "0", "--time", "2", "--output", str(destination)])
            result = json.loads(destination.read_text(encoding="utf-8"))
            self.assertEqual(len(result["predictions"]), 2)
            self.assertEqual(result["model_status"], "unverified_hypothesis")
            self.assertIn("unverified", warning.getvalue())

    def test_option_typo_does_not_silently_use_default(self):
        with tempfile.TemporaryDirectory() as directory:
            ini = Path(directory) / "scenario.ini"
            ini.write_text(FIXTURE, encoding="utf-8")
            options = Path(directory) / "options.json"
            options.write_text('{"range_scal_m": 1852}', encoding="utf-8")
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                main(["predict", "--ini", str(ini), "--options", str(options), "--time", "0"])
            self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
