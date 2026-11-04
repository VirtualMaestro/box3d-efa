import assert from 'node:assert/strict';
import { test } from 'node:test';
import { fileURLToPath } from 'node:url';

import Box3D, { hasSimd } from '../dist/entry.mjs';

test('a host without SIMD gets the scalar build', async (t) => {
  assert.equal(hasSimd(), true);
  t.mock.method(WebAssembly, 'validate', () => false);
  const files = [];
  const b3 = await Box3D({ locateFile: (file) => { files.push(file); return fileURLToPath(new URL(`../dist/${file}`, import.meta.url)); } });
  assert.deepEqual(files, ['box3d.scalar.wasm']);
  assert.equal(typeof b3.World, 'function');
});

test('auto entry picks the threaded build in node', async () => {
  // Node has SIMD and SharedArrayBuffer, so the auto entry should resolve
  // to the deluxe flavour.
  const b3 = await Box3D();
  assert.equal(b3.threaded, true);

  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 }, workerCount: 2 });
  assert.equal(world.getWorkerCount(), 2);

  const ground = world.createBody({ type: 'static', position: { x: 0, y: -0.5, z: 0 } });
  ground.createBox({ halfExtents: { x: 20, y: 0.5, z: 20 } });
  const box = world.createBody({ type: 'dynamic', position: { x: 0, y: 3, z: 0 } });
  box.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 }, density: 1 });

  for (let i = 0; i < 200; i++) {
    world.step(1 / 60, 4);
  }
  assert.ok(Math.abs(box.getPosition().y - 0.5) < 0.01);

  world.destroy();
  world.delete();
});
