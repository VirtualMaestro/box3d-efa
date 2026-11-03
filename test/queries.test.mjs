import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// World.castRay, castShape, overlapShape, closestPoint and Shape.getClosestPoint. Each test builds its own world.

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

test('castShape: a sphere swept along x stops at the box, a mask leaves it out', () => {
  const { world, wall } = level();
  const sphere = { points: [{ x: 0, y: 0, z: 0 }], radius: 0.25 };
  const hit = world.castShape({ x: 0, y: 0.5, z: 0 }, sphere, { x: 10, y: 0, z: 0 }, undefined);
  assert.equal(hit.hit, true);
  // Box3D's shape cast stops within the linear slop (0.005) past the touch.
  assert.ok(near(hit.fraction * 10, 4.25, 0.006), `travel ${hit.fraction * 10}`);
  assert.ok(near(hit.normal.x, -1));
  assert.equal(hit.bodyIndex, wall.getIndex());
  const masked = world.castShape({ x: 0, y: 0.5, z: 0 }, sphere, { x: 10, y: 0, z: 0 }, { maskBits: 1 });
  assert.equal(masked.hit, false);
  world.destroy();
});

test('castRay and castShape leave sensors out unless includeSensors', () => {
  const { world } = level();
  const zone = world.createBody({ type: 'static', position: { x: 2, y: 0.5, z: 0 } });
  zone.createSphere({ radius: 0.5, isSensor: true });
  const from = { x: 0, y: 0.5, z: 0 }, along = { x: 10, y: 0, z: 0 };
  assert.ok(near(world.castRay(from, along, undefined).fraction * 10, 4.5));
  assert.ok(near(world.castRay(from, along, { includeSensors: true }).fraction * 10, 1.5));
  assert.equal(world.castRay(from, along, { includeSensors: true }).bodyIndex, zone.getIndex());
  world.destroy();
});

test('overlapShape: every shape a capsule touches', () => {
  const { world, floor, wall } = level();
  const capsule = { points: [{ x: 0, y: -0.5, z: 0 }, { x: 0, y: 0.5, z: 0 }], radius: 0.3 };
  const hits = world.overlapShape({ x: 4.6, y: 0.7, z: 0 }, capsule, undefined).map(h => h.bodyIndex).sort();
  assert.deepEqual(hits, [floor.getIndex(), wall.getIndex()].sort());
  assert.deepEqual(world.overlapShape({ x: 0, y: 3, z: 0 }, capsule, undefined), []);
  world.destroy();
});

test('closestPoint and Shape.getClosestPoint', () => {
  const { world, wall } = level();
  const r = world.closestPoint({ x: 5, y: 3, z: 0 }, 5, { maskBits: 2 });
  assert.equal(r.hit, true);
  assert.equal(r.bodyIndex, wall.getIndex());
  assert.ok(near(r.distance, 2) && near(r.point.y, 1));
  assert.equal(world.closestPoint({ x: 5, y: 3, z: 0 }, 1.5, { maskBits: 2 }).hit, false);
  const body = world.createBody({ type: 'static', position: { x: -5, y: 0, z: 0 } });
  const shape = body.createSphere({ radius: 1 });
  assert.ok(near(shape.getClosestPoint({ x: -5, y: 4, z: 0 }).y, 1));
  world.destroy();
});
