import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// Body.createMesh.

const b3 = await Box3D();

const DT = 1 / 60;
const SUBSTEPS = 4;

test('createMesh: a sphere dropped on a static two-triangle quad comes to rest on it', () => {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const ground = world.createBody({ type: 'static', position: { x: 0, y: 0, z: 0 } });
  // A 20 x 20 quad at y = 0, wound so that both triangles face up.
  const vertices = [-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10];
  const indices = [0, 2, 1, 0, 3, 2];
  assert.ok(ground.createMesh({ vertices, indices }).isValid());
  const ball = world.createBody({ type: 'dynamic', position: { x: 1, y: 3, z: -2 } });
  ball.createSphere({ radius: 0.5, density: 1 });
  for (let i = 0; i < 180; i++) world.step(DT, SUBSTEPS);
  const p = ball.getPosition();
  assert.ok(Math.abs(p.y - 0.5) < 0.02, `rests at y = ${p.y}, expected 0.5`);
  assert.ok(Math.abs(p.x - 1) < 1e-3 && Math.abs(p.z + 2) < 1e-3, `drifted to ${p.x}, ${p.z}`);
  assert.ok(Math.abs(ball.getLinearVelocity().y) < 1e-2);
  world.destroy();
});
