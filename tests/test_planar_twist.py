"""Numeric regression for the base-origin vs Gazebo center-of-mass twist."""
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PlanarTwistTest(unittest.TestCase):
    def test_turn_keeps_commanded_link_origin_stationary(self):
        flags = subprocess.check_output(['pkg-config', '--cflags', '--libs', 'ignition-math6'], text=True)
        source = '''#include "planar_twist.hh"
#include <cassert>
int main() {
  using ignition::math::Vector3d;
  Vector3d r(.024387, .003131, -.102779), w(0,0,.3), v(0,0,0);
  const auto cog = gazebo::CoGVelocity(v,w,r);
  assert((cog - Vector3d(-.0009393,.0073161,0)).Length() < 1e-12);
  assert((cog - w.Cross(r)).Length() < 1e-12);
  v.Set(.2,-.1,0);
  assert((gazebo::CoGVelocity(v,w,r)-w.Cross(r)-v).Length() < 1e-12);
}'''
        with tempfile.TemporaryDirectory(prefix='planar-twist-') as directory:
            executable = str(Path(directory)/'test')
            subprocess.run(['g++', '-std=c++14', '-x', 'c++', '-', '-o', executable,
                            '-I'+str(ROOT/'src/platform/bunker_sim_runtime/src')]+shlex.split(flags),
                           input=source, text=True, check=True, capture_output=True)
            subprocess.run([executable], check=True)


if __name__ == '__main__': unittest.main()
