import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// World.getBodyMoveEvents.

const b3 = await Box3D();

const DT = 1 / 60;
const SUBSTEPS = 4;

test('getBodyMoveEvents: typed views equal the object events', () => {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  for (let i = 0; i < 3; i++) world.createBody({ type: 'dynamic', position: { x: 2 * i, y: 5, z: 0 } }).createSphere({ radius: 0.5 });
  world.step(DT, SUBSTEPS);
  const objects = world.getBodyEvents();
  const typed = world.getBodyMoveEvents();
  assert.equal(typed.count, 3);
  for (let i = 0; i < 3; i++) {
    assert.equal(typed.userData[i], objects[i].userData);
    assert.equal(typed.transform[7 * i + 1], Math.fround(objects[i].position.y));
    assert.equal(typed.transform[7 * i + 6], Math.fround(objects[i].rotation.w));
  }
  world.destroy();
});
