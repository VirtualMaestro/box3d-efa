import assert from 'node:assert/strict';
import { test } from 'node:test';

import Standard from '../dist/box3d.mjs';
import Scalar from '../dist/box3d.scalar.mjs';

for (const [name, load] of [['standard', Standard], ['scalar', Scalar]]) {
  test(`${name}: cooked geometry is shared by many shapes, and memory stays flat over many worlds`, async () => {
    const b3 = await load();
    const side = 40, vertices = [], indices = [];
    for (let z = 0; z <= side; z++) for (let x = 0; x <= side; x++) vertices.push(x, 0, z);
    for (let z = 0; z < side; z++) for (let x = 0; x < side; x++) { const a = z * (side + 1) + x, b = a + 1, c = a + side + 1, d = c + 1; indices.push(a, c, b, b, c, d); }
    const heights = new Array(33 * 33).fill(1);
    const mesh = b3.cookMesh({ vertices, indices }), field = b3.cookHeightField({ heights, countX: 33, countZ: 33, scale: { x: 1, y: 1, z: 1 } });
    assert.notEqual(mesh, 0);
    assert.notEqual(field, 0);
    const round = () => {
      const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
      for (let i = 0; i < 4; i++) {
        const a = world.createBody({ type: 'static', position: { x: i * 100, y: 0, z: 0 } }), b = world.createBody({ type: 'static', position: { x: i * 100, y: 0, z: 100 } });
        assert.ok(a.createMesh({ cooked: mesh }).isValid());
        assert.ok(b.createHeightField({ cooked: field }).isValid());
      }
      const ball = world.createBody({ type: 'dynamic', position: { x: 5, y: 1, z: 5 } });
      ball.createSphere({ radius: 0.5 });
      for (let t = 0; t < 60; t++) world.step(1 / 60, 4);
      const y = ball.getPosition().y;
      world.destroy();
      return y;
    };
    assert.ok(Math.abs(round() - 0.5) < 0.05, 'a ball rests on the shared mesh');
    const before = b3.HEAPU8.length;
    for (let i = 0; i < 40; i++) round();
    assert.equal(b3.HEAPU8.length, before);
    b3.freeMesh(mesh);
    b3.freeHeightField(field);
  });

  test(`${name}: an end event names a shape destroyed since by its index`, async () => {
    const b3 = await load();
    const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
    world.createBody({ type: 'static', position: { x: 0, y: -1, z: 0 } }).createBox({ halfExtents: { x: 50, y: 1, z: 50 } });
    const ball = world.createBody({ type: 'dynamic', position: { x: 0, y: 0.5, z: 0 } }), shape = ball.createSphere({ radius: 0.5 });
    shape.enableContactEvents(true);
    for (let t = 0; t < 30; t++) world.step(1 / 60, 4);
    const index = shape.getIndex();
    ball.destroy();
    world.step(1 / 60, 4);
    const n = world.readContacts(), u = b3.HEAPU32, at = b3.flatOut() >> 2;
    let found = false;
    for (let i = 0; i < n; i++) {
      const r = at + 12 * i;
      if (u[r] !== 2) continue;
      for (const w of [r + 1, r + 3]) if (u[w] === 0xffffffff && u[w + 1] === index) found = true;
    }
    assert.ok(found, 'an end with the destroyed shape');
    world.destroy();
  });
}
