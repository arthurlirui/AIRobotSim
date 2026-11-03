"""Smoke tests for the server API layer (REST + WebSocket).

Run:  python -m tests.test_server_api
"""

import json
import sys
import unittest

from fastapi.testclient import TestClient

from server.app import app


class TestServerAPI(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.client = TestClient(app)

    def test_catalog(self):
        r = self.client.get("/api/catalog")
        self.assertEqual(r.status_code, 200)
        cat = r.json()
        for key in ("robots", "terrains", "algorithms", "comm_channels"):
            self.assertIn(key, cat)
        self.assertIn("wheeled", cat["robots"]["mobile"])
        self.assertIn("flat", cat["terrains"])
        # all four algorithm families registered
        for kind in ("slam", "navigation", "detection", "swarm"):
            self.assertTrue(len(cat["algorithms"][kind]) > 0,
                            f"no {kind} algorithms registered")

    def test_scenario_run(self):
        cfg = {
            "env": {"procedural": {"n_obstacles": 4, "size": 10}},
            "robots": [{"name": "r0", "type": "wheeled",
                        "position": [-4, 0, 0]}],
            "algorithms": [
                {"kind": "navigation", "name": "potential_field",
                 "config": {"robot_id": "r0", "goal": [4, 0, 0]}},
            ],
            "steps": 20,
        }
        r = self.client.post("/api/scenarios/run", json={
            "name": "smoke", "config": cfg})
        self.assertEqual(r.status_code, 200)
        res = r.json()
        self.assertEqual(res["steps"], 20)
        self.assertTrue(len(res["history"]) > 0)
        self.assertIn("wall_time_sec", res)

    def test_comm_experiments(self):
        r = self.client.get("/api/comm/scenarios")
        self.assertEqual(r.status_code, 200)
        names = [s["name"] for s in r.json()["scenarios"]]
        self.assertIn("swarm_relay", names)
        r2 = self.client.post("/api/comm/simulate",
                              json={"experiment": "swarm_relay"})
        self.assertEqual(r2.status_code, 200)
        self.assertNotIn("error", r2.json())

    def test_design_presets_and_simulate(self):
        r = self.client.get("/api/design/presets")
        self.assertEqual(r.status_code, 200)
        presets = [p["name"] for p in r.json()["presets"]]
        self.assertIn("quadruped", presets)
        r2 = self.client.post("/api/design/simulate",
                              json={"preset": "quadruped", "steps": 10,
                                    "dt": 0.05})
        self.assertEqual(r2.status_code, 200)
        frames = r2.json()["frames"]
        self.assertEqual(len(frames), 10)
        self.assertIn("links", frames[0])

    def test_ws_sim_stream(self):
        cfg = {
            "env": {"procedural": {"n_obstacles": 3, "size": 8}},
            "robots": [
                {"name": "r0", "type": "wheeled", "position": [-3, 0, 0]},
                {"name": "r1", "type": "wheeled", "position": [-3, 1, 0]},
            ],
            "algorithms": [
                {"kind": "navigation", "name": "potential_field",
                 "config": {"robot_id": "r0", "goal": [3, 0, 0]}},
            ],
            "comm": {"channel": "free_space"},
            "world": {"seed": 1},
        }
        with self.client.websocket_connect("/ws/sim") as ws:
            ws.send_text(json.dumps({"type": "create", "config": cfg}))
            created = json.loads(ws.receive_text())
            self.assertEqual(created["type"], "created")
            self.assertEqual(len(created["robots"]), 2)
            self.assertTrue(created["has_comm"])
            self.assertTrue(len(created["terrain"]["splats"]) > 0)

            ws.send_text(json.dumps({"type": "step", "n": 3}))
            positions = []
            for _ in range(3):
                frame = json.loads(ws.receive_text())
                self.assertEqual(frame["type"], "frame")
                self.assertIn("r0", frame["robots"])
                positions.append(frame["robots"]["r0"]["position"])
            # robot should be moving toward the goal
            self.assertGreater(positions[-1][0], positions[0][0])
            # comm links present since comm configured
            self.assertTrue(len(frame["comm_links"]) > 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)


def run():
    """Entry point for tests.run_all."""
    unittest.main(module="tests.test_server_api",
                  argv=["test_server_api", "-v"], exit=False)
