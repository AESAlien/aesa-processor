import unittest

from tools.eval.scenario import parse_scenario


FIXTURE = """SCENARIO_INFORMATION
1 38 127 100 0 0 0
TARGET_1
10 0 100 100 90 0 1 10 1 1 100
MANEUVER_1
4 4 1 90
"""


class ScenarioTest(unittest.TestCase):
    def test_line_endings_and_raw_fields(self):
        for newline in ("\r", "\n", "\r\n"):
            with self.subTest(newline=repr(newline)):
                result = parse_scenario(FIXTURE.replace("\n", newline))
                self.assertEqual(result.information, (1, 38, 127, 100, 0, 0, 0))
                self.assertEqual(len(result.target(1).values), 11)
                self.assertEqual(result.target(1).maneuvers[0].kind, 4)

    def test_optional_brackets_comments_and_bom(self):
        result = parse_scenario("\ufeff" + FIXTURE.replace("SCENARIO_INFORMATION", "[SCENARIO_INFORMATION] ; comment"))
        self.assertEqual(result.target(1).values[8], 1)

    def test_invalid_structure_and_nonfinite_numbers(self):
        cases = [
            FIXTURE + "TARGET_1\n",
            FIXTURE.replace("1 10 1 1 100", "1 10 2 1 100"),
            FIXTURE.replace("4 4 1 90", "4 4 1"),
            FIXTURE.replace("4 4 1 90", "4 4 1 nan"),
            FIXTURE.replace("MANEUVER_1", "MANEUVER_2"),
            FIXTURE.replace("1 38 127", "1.5 38 127"),
            FIXTURE.replace("4 4 1 90", "4 4.5 1 90"),
            FIXTURE.replace("4 4 1 90", "4 5 1 90"),
            FIXTURE.replace("10 0 100", "10 0"),
            FIXTURE.replace("SCENARIO_INFORMATION", "UNKNOWN"),
            "1 2 3\n" + FIXTURE,
            FIXTURE + "MANEUVER_2\n0 1 1 1\n",
        ]
        for text in cases:
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_scenario(text)

    def test_zero_maneuvers_and_missing_target(self):
        scenario = parse_scenario(FIXTURE.replace("1 10 1 1 100", "1 10 0 1 100").replace("4 4 1 90", ""))
        self.assertEqual(scenario.target(1).maneuvers, ())
        with self.assertRaises(ValueError):
            scenario.target(2)


if __name__ == "__main__":
    unittest.main()
