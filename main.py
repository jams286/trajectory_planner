"""Entry point for the Trajectory Planner application."""

import sys
import os

# Ensure project root is on sys.path so imports work when running from any cwd
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ui.app import MainApp


def main() -> None:
    app = MainApp()
    app.run()


if __name__ == "__main__":
    main()
