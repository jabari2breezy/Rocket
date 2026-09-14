import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


class UrlTests(unittest.TestCase):
    def test_static_pages_exist(self):
        for f in ("ui/static/index.html", "ui/static/tutorial.html",
                  "ui/static/app.js", "ui/static/tutorial.js"):
            self.assertTrue((ROOT / f).exists(), f)

    def test_readme_exists(self):
        self.assertTrue((ROOT / "README.md").exists())


if __name__ == "__main__":
    unittest.main()