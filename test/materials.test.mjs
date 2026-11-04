import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// Materials written on a contact at rest, and the rules that mix them.

const b3 = await Box3D();
const DT = 1 / 60;

/** A box on a static plate tilted 20 degrees about z, each with its friction; returns how far it slid after `steps`. */
function slope({ plate = 0.8, box = 0.8, rule, write }) {
  const world = new b3.World({ gravity: { x: 0, y: -9.8, z: 0 } });
  if (rule !== undefined) world.setCombine(rule, 3);
  const a = 20 * Math.PI / 180, rotation = { x: 0, y: 0, z: Math.sin(a / 2), w: Math.cos(a / 2) };
  world.createBody({ type: 'static', rotation }).createBox({ halfExtents: { x: 20, y: 0.5, z: 2 }, friction: plate });
  const body = world.createBody({ type: 'dynamic', position: { x: -Math.sin(a) * 1.01, y: Math.cos(a) * 1.01, z: 0 }, rotation, enableSleep: false });
  const shape = body.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 }, friction: box });
  for (let i = 0; i < 90; i++) world.step(DT, 4);
  const x = body.getPosition().x;
  write?.(shape, body);
  for (let i = 0; i < 60; i++) world.step(DT, 4);
  const slid = x - body.getPosition().x;
  world.destroy();
  return slid;
}

test('a friction written on a contact at rest does not reach it until refreshContacts', () => {
  assert.ok(slope({}) < 0.02);
  assert.ok(slope({ write: shape => shape.setFriction(0.05) }) < 0.02);
  assert.ok(slope({ write: (shape, body) => { shape.setFriction(0.05); body.refreshContacts(); } }) > 0.5);
});

test('setRollingResistance reads back through the contact: a ball stops', () => {
  const world = new b3.World({ gravity: { x: 0, y: -9.8, z: 0 } });
  world.createBody({ type: 'static', position: { x: 0, y: -0.5, z: 0 } }).createBox({ halfExtents: { x: 100, y: 0.5, z: 100 } });
  const ball = world.createBody({ type: 'dynamic', position: { x: 0, y: 0.5, z: 0 }, linearVelocity: { x: 3, y: 0, z: 0 } });
  const shape = ball.createSphere({ radius: 0.5 });
  for (let i = 0; i < 30; i++) world.step(DT, 4);
  assert.ok(ball.getLinearVelocity().x > 1);
  shape.setRollingResistance(0.5);
  ball.refreshContacts();
  for (let i = 0; i < 240; i++) world.step(DT, 4);
  assert.ok(Math.abs(ball.getLinearVelocity().x) < 0.1);
  world.destroy();
});

test('setCombine: friction 0.2 and 0.8 hold under geometric (0.4), average and max; slide under min and multiply; a rule out of range throws', () => {
  const slid = rule => slope({ plate: 0.2, box: 0.8, rule });
  for (const holds of [0, 1, 3]) assert.ok(slid(holds) < 0.02, `rule ${holds}`);
  for (const slides of [2, 4]) assert.ok(slid(slides) > 0.5, `rule ${slides}`);
  const world = new b3.World({ gravity: { x: 0, y: 0, z: 0 } });
  assert.throws(() => world.setCombine(5, 0));
  world.destroy();
});
