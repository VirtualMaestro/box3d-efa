import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// World.save, World.restore and World.getBody.

const b3 = await Box3D();

const DT = 1 / 60;
const SUBSTEPS = 4;

test('save and restore: the restored world replays the original bit for bit, twice', () => {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  world.createBody({ type: 'static', position: { x: 0, y: -0.5, z: 0 } }).createBox({ halfExtents: { x: 10, y: 0.5, z: 10 } });
  const bodies = [];
  for (let i = 0; i < 6; i++) {
    const b = world.createBody({ type: 'dynamic', position: { x: 0.3 * i, y: 1 + 1.1 * i, z: 0.1 * i } });
    b.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 } });
    bodies.push(b);
  }
  for (let i = 0; i < 30; i++) world.step(DT, SUBSTEPS);
  const state = () => bodies.flatMap(b => { const p = b.getPosition(), q = b.getRotation(); return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]; });
  const saved = world.save();
  assert.ok(saved instanceof Uint8Array && saved.length > 0);
  const ids = bodies.map(b => [b.getIndex(), b.getGeneration()]);
  const run = () => { const out = []; for (let i = 0; i < 60; i++) { world.step(DT, SUBSTEPS); out.push(state()); } return out; };
  const first = run();
  for (let k = 0; k < 2; k++) {
    assert.equal(world.restore(saved), true);
    for (let i = 0; i < bodies.length; i++) bodies[i] = world.getBody(ids[i][0], ids[i][1]);
    assert.ok(bodies.every(b => b.isValid()));
    assert.deepEqual(run(), first);
  }
  assert.equal(world.restore(new Uint8Array([1, 2, 3])), false);
  assert.ok(world.isValid());
  world.destroy();
});
