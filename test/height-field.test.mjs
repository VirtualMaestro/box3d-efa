import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// Body.createHeightField.

const b3 = await Box3D();

const near = (a, b, eps = 1e-3) => Math.abs(a - b) <= eps;

test('createHeightField: rows along z from the body origin, a ray finds each sample', () => {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const n = 5, h = (x, z) => 0.5 * x + 0.25 * z;
  const heights = [];
  for (let z = 0; z < n; z++) for (let x = 0; x < n; x++) heights.push(h(x, z));
  const body = world.createBody({ type: 'static', position: { x: 0, y: 0, z: 0 } });
  const shape = body.createHeightField({ heights, countX: n, countZ: n, scale: { x: 2, y: 1, z: 2 } });
  assert.ok(shape.isValid());
  for (const [x, z] of [[1, 1], [3, 2], [2, 3]]) {
    const r = world.castRay({ x: 2 * x, y: 10, z: 2 * z }, { x: 0, y: -20, z: 0 }, undefined);
    assert.equal(r.hit, true);
    assert.ok(near(r.point.y, h(x, z)), `(${x}, ${z}): ${r.point.y} against ${h(x, z)}`);
    assert.ok(r.normal.y > 0.8);
  }
  assert.equal(body.createHeightField({ heights: [0, 0, 0], countX: 2, countZ: 2 }).isValid(), false);
  world.destroy();
});
