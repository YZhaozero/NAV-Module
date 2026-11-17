"""Run the ROS desktop with the system NumPy/SciPy ABI pair."""
import os
import sys


def main():
    os.execve(sys.executable,
              [sys.executable, '-s', '-m', 'nav_app.qt_nav_node', *sys.argv[1:]],
              {**os.environ, 'PYTHONNOUSERSITE': '1'})
