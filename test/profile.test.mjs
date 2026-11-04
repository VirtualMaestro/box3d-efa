import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// The step timers: off by default (each clock read crosses into JavaScript), World.getProfile reads them when on.

const b3 = await Box3D();

function steppedProfile() {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const floor = world.createBody({ type: 'static', position: { x: 0, y: -0.5, z: 0 } });
  floor.createBox({ halfExtents: { x: 10, y: 0.5, z: 10 } });
  for (let i = 0; i < 200; i++) {
    const body = world.createBody({ type: 'dynamic', position: { x: 0, y: 0.5 + i * 1.01, z: 0 } });
    body.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 } });
  }
  for (let t = 0; t < 10; t++) world.step(1 / 60, 4);
  const profile = world.getProfile();
  world.destroy();
  return profile;
}

test('the profile is all zeros without the timers, and has every stage', () => {
  const profile = steppedProfile();
  assert.ok(Object.keys(profile).length >= 23);
  for (const [stage, ms] of Object.entries(profile)) assert.equal(ms, 0, stage);
});

test('enableStepTimers(true) fills the profile, false stops it again', () => {
  b3.enableStepTimers(true);
  const on = steppedProfile();
  b3.enableStepTimers(false);
  assert.ok(on.step > 0, 'step');
  assert.ok(on.solve > 0, 'solve');
  assert.equal(steppedProfile().step, 0);
});
