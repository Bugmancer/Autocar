"""不启动 ROS 和硬件，验证两个 launch 的策略选择、兼容参数及无效选项拒绝。"""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


SRC = Path(__file__).resolve().parents[2]
LOCAL = SRC / "fw_mid_local_planner/launch/local_planner.launch"
BRINGUP = SRC / "fw_mid_bringup/launch/navigation.launch"


def resolve(value, arguments):
    if value.startswith("$(eval "):
        return eval(value[7:-1], {"__builtins__": {}}, {"arg": arguments.__getitem__})
    if value.startswith("$(arg "):
        return arguments[value[6:-1]]
    return value


def launch_arguments(root, overrides):
    arguments = dict(overrides)
    for argument in root.findall("arg"):
        name = argument.get("name")
        if name not in arguments:
            arguments[name] = resolve(argument.get("default"), arguments)
    return arguments


class LaunchSelectionTests(unittest.TestCase):
    def selected_node(self, launch, **overrides):
        root = ET.parse(launch).getroot()
        arguments = launch_arguments(root, overrides)
        if launch == BRINGUP:
            include = next(element for element in root.findall("include")
                           if "fw_mid_local_planner" in element.get("file"))
            forwarded = {arg.get("name"): resolve(arg.get("value"), arguments)
                         for arg in include.findall("arg")}
            return self.selected_node(LOCAL, **forwarded)
        nodes = [node for node in root.findall("node")
                 if node.get("pkg") == "fw_mid_local_planner"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("name"), "path_follower_node")
        return resolve(nodes[0].get("type"), arguments)

    def test_both_launches_select_exactly_one_follower(self):
        for launch in (LOCAL, BRINGUP):
            with self.subTest(launch=launch):
                self.assertEqual(self.selected_node(launch), "path_follower_node.py")
                self.assertEqual(self.selected_node(launch, follower_variant="classic"),
                                 "path_follower_node.py")
                self.assertEqual(self.selected_node(launch, follower_variant="apf"),
                                 "path_follower_node_v1.py")

    def test_existing_script_override_remains_supported(self):
        for launch in (LOCAL, BRINGUP):
            for script in ("path_follower_node.py", "path_follower_node_v1.py"):
                with self.subTest(launch=launch, script=script):
                    self.assertEqual(self.selected_node(launch, follower_node=script), script)

    def test_unknown_strategy_or_executable_fails_closed(self):
        for launch in (LOCAL, BRINGUP):
            with self.subTest(launch=launch):
                with self.assertRaises(KeyError):
                    self.selected_node(launch, follower_variant="typo")
                with self.assertRaises(KeyError):
                    self.selected_node(launch, follower_node="other.py")


if __name__ == "__main__":
    unittest.main()
