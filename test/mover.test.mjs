import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// The capsule mover: World.castMover, World.collideMover, solvePlanes and clipVector.

const b3 = await Box3D();

const near = (a, b, eps = 1e-3) => Math.abs(a - b) <= eps;

/** A static 20 x 1 x 20 floor with its top at y = 0, and a static unit box centred at (5, 0.5, 0) on layer 2. */
function level() {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const floor = world.createBody({ type: 'static', position: { x: 0, y: -0.5, z: 0 } });
  floor.createBox({ halfExtents: { x: 10, y: 0.5, z: 10 } });
  const wall = world.createBody({ type: 'static', position: { x: 5, y: 0.5, z: 0 } });
  wall.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 }, filter: { categoryBits: 2, maskBits: 0xffffffff, groupIndex: 0 } });
  return { world, floor, wall };
}

test('the mover: collideMover gives the floor plane, solvePlanes keeps a move out of it, castMover stops at the box', () => {
  const { world } = level();
  const capsule = { center1: { x: 0, y: -0.5, z: 0 }, center2: { x: 0, y: 0.5, z: 0 }, radius: 0.3 };
  const planes = world.collideMover({ x: 0, y: 0.79, z: 0 }, capsule, undefined);
  assert.equal(planes.length, 1);
  assert.ok(near(planes[0].normal.y, 1));
  const solved = b3.solvePlanes({ x: 1, y: -1, z: 0 }, planes);
  assert.ok(near(solved.delta.x, 1) && solved.delta.y > -0.02, `delta ${JSON.stringify(solved.delta)}`);
  const f = world.castMover({ x: 0, y: 0.81, z: 0 }, capsule, { x: 10, y: 0, z: 0 }, undefined);
  assert.ok(near(f * 10, 4.2, 0.02), `travel ${f * 10}`);
  assert.ok(near(world.castMover({ x: 0, y: 0.81, z: 0 }, capsule, { x: 10, y: 0, z: 0 }, { maskBits: 1 }), 1));
  assert.equal(solved.push.length, 1);
  assert.ok(solved.push[0] > 0);
  assert.deepEqual(b3.clipVector({ x: 1, y: -1, z: 0 }, planes), { x: 1, y: -1, z: 0 }, 'a plane without push does not clip');
  const clipped = b3.clipVector({ x: 1, y: -1, z: 0 }, planes.map((p, i) => ({ ...p, push: solved.push[i] })));
  assert.ok(near(clipped.y, 0) && near(clipped.x, 1), `clipped ${JSON.stringify(clipped)}`);
  world.destroy();
});
