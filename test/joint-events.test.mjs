import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// Joint events (World.getJointEvents) and Shape.getContactData.

const b3 = await Box3D();

const DT = 1 / 60;
const SUBSTEPS = 4;

test('joint events above the force threshold, and contact data', () => {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const anchor = world.createBody({ type: 'static', position: { x: 0, y: 5, z: 0 } });
  const weight = world.createBody({ type: 'dynamic', position: { x: 0, y: 4, z: 0 } });
  weight.createSphere({ radius: 0.25, density: 1000 });
  const joint = world.createSphericalJoint(anchor, weight, { localFrameA: { position: { x: 0, y: -1, z: 0 } } });
  joint.setForceThreshold(1);
  world.step(DT, SUBSTEPS);
  const events = world.getJointEvents();
  assert.equal(events.length, 1);
  assert.equal(events[0].jointIndex, joint.getIndex());
  joint.setForceThreshold(1e9);
  world.step(DT, SUBSTEPS);
  assert.equal(world.getJointEvents().length, 0);

  const floor = world.createBody({ type: 'static', position: { x: 10, y: -0.5, z: 0 } });
  const ground = floor.createBox({ halfExtents: { x: 2, y: 0.5, z: 2 } });
  const box = world.createBody({ type: 'dynamic', position: { x: 10, y: 0.5, z: 0 } });
  box.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 } });
  for (let i = 0; i < 30; i++) world.step(DT, SUBSTEPS);
  const contacts = ground.getContactData();
  assert.equal(contacts.length, 1);
  const m = contacts[0].manifolds[0];
  assert.equal(m.points.length, 4);
  assert.ok(Math.abs(m.normal.y) > 0.99);
  assert.ok(m.points.every(p => p.normalImpulse > 0));
  world.destroy();
});
