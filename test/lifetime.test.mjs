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

test('a handle of a destroyed body, shape, joint or world throws instead of reaching the object in its slot', () => {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const ground = world.createBody({ type: 'static', position: { x: 0, y: -1, z: 0 } });
  ground.createBox({ halfExtents: { x: 20, y: 1, z: 20 } });
  const a = world.createBody({ type: 'dynamic', position: { x: 0, y: 1, z: 0 } });
  a.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 } });
  const twin = world.getBody(a.getIndex(), a.getGeneration());
  a.destroy();
  assert.throws(() => a.setTransform({ x: 50, y: 50, z: 50 }, { x: 0, y: 0, z: 0, w: 1 }), /used after/);
  assert.deepEqual(ground.getPosition(), { x: 0, y: -1, z: 0 }, 'the ground did not move');
  // A new body takes the slot: the second handle to the old one must not write into it.
  const b = world.createBody({ type: 'dynamic', position: { x: 3, y: 1, z: 0 } });
  b.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 } });
  assert.equal(b.getIndex(), twin.getIndex(), 'the slot was reused');
  assert.equal(twin.isValid(), false);
  assert.throws(() => twin.setLinearVelocity({ x: 9, y: 9, z: 9 }), /used after/);
  assert.deepEqual(b.getLinearVelocity(), { x: 0, y: 0, z: 0 });
  const shape = b.createSphere({ radius: 0.2 });
  const joint = world.createRevoluteJoint(ground, b, {});
  shape.destroy(true);
  assert.throws(() => shape.getFriction(), /Shape handle/);
  b.destroy();
  assert.throws(() => joint.getConstraintForce(), /Joint handle/);
  world.destroy();
  // A new world in the same slot: the old world's handles stay dead, and their destroy does nothing.
  const next = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const c = next.createBody({ type: 'dynamic', position: { x: 0, y: 1, z: 0 } });
  assert.equal(c.getIndex(), ground.getIndex(), 'the new world reuses the slot index');
  assert.equal(ground.isValid(), false);
  ground.destroy();
  assert.equal(c.isValid(), true, 'the old handle did not destroy the new body');
  assert.throws(() => ground.getPosition(), /Body handle/);
  next.destroy();
});

test('readMoved leaves out a body destroyed after the step', () => {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const bodies = [0, 1, 2].map(i => { const b = world.createBody({ type: 'dynamic', position: { x: 2 * i, y: 5, z: 0 } }); b.createSphere({ radius: 0.5 }); return b; });
  world.step(1 / 60, 4);
  bodies[1].destroy();
  assert.equal(world.readMoved(), 2);
  const at = b3.flatOut() >> 2, u = b3.HEAPU32;
  assert.deepEqual([u[at], u[at + 16]].sort(), [bodies[0].getIndex(), bodies[2].getIndex()].sort());
  world.destroy();
});
