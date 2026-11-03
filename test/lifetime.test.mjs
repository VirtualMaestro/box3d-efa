import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// destroy() twice, or after the owner went, does nothing.

const b3 = await Box3D();

test('a second destroy of a body, shape, joint or world does nothing', () => {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const ground = world.createBody({ type: 'static', position: { x: 0, y: -1, z: 0 } });
  ground.createBox({ halfExtents: { x: 20, y: 1, z: 20 } });
  const bodies = [];
  for (let i = 0; i < 4; i++) {
    const b = world.createBody({ type: 'dynamic', position: { x: 2 * i, y: 1, z: 0 } });
    b.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 } });
    bodies.push(b);
  }
  const joint = world.createRevoluteJoint(bodies[0], bodies[1], {});
  const shape = bodies[2].createSphere({ radius: 0.3, center: { x: 0, y: 0.6, z: 0 } });
  for (let s = 0; s < 10; s++) world.step(1 / 60, 4);
  bodies[0].destroy();
  assert.equal(joint.isValid(), false, 'the joint went with its body');
  joint.destroy(true);
  bodies[0].destroy();
  shape.destroy(true);
  shape.destroy(true);
  for (let s = 0; s < 60; s++) world.step(1 / 60, 4);
  for (const b of bodies.slice(1)) {
    assert.equal(b.isValid(), true);
    assert.ok(Math.abs(b.getPosition().y - 0.5) < 0.05, `body ${b.getIndex()} rests on the ground`);
  }
  assert.equal(bodies[2].getShapeCount(), 1);
  world.destroy();
  world.destroy();
  const again = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  assert.equal(again.isValid(), true);
  again.destroy();
});
