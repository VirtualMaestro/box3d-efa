import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// The flat calls (flatIn, flatOut, World.queryBatch, readMoved, readEvents, writeBodies) against the object calls
// they stand for: the same answers, word for word.

const b3 = await Box3D();

const DT = 1 / 60;
const SUBSTEPS = 4;
const f32 = Math.fround;

/** Writes records of 12 words (numbers; an integer word is given as { u }) at flatIn. */
function writeIn(records) {
  const at = b3.flatIn(12 * records.length) >> 2;
  const u = b3.HEAPU32, f = b3.HEAPF32;
  records.forEach((r, i) => r.forEach((w, j) => {
    if (typeof w === 'object') u[at + 12 * i + j] = w.u;
    else f[at + 12 * i + j] = w;
  }));
}

/** The output words, as unsigned integers and as floats. */
function out(words) {
  const at = b3.flatOut() >> 2;
  return { u: b3.HEAPU32.slice(at, at + words), f: b3.HEAPF32.slice(at, at + words) };
}

/** A floor, a box on layer 2, a sensor sphere, and a grid of dynamic boxes falling onto the floor. */
function level() {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const floor = world.createBody({ type: 'static', position: { x: 0, y: -0.5, z: 0 } });
  floor.createBox({ halfExtents: { x: 10, y: 0.5, z: 10 }, enableContactEvents: true });
  const wall = world.createBody({ type: 'static', position: { x: 5, y: 0.5, z: 0 } });
  wall.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 }, filter: { categoryBits: 2, maskBits: 0xffffffff, groupIndex: 0 } });
  const zone = world.createBody({ type: 'static', position: { x: -3, y: 0.5, z: 0 } });
  zone.createSphere({ radius: 0.5, isSensor: true, enableSensorEvents: true });
  const boxes = [];
  for (let i = 0; i < 9; i++) {
    const b = world.createBody({ type: 'dynamic', position: { x: (i % 3) * 1.2 - 1.2, y: 0.6 + 0.3 * i, z: Math.floor(i / 3) * 1.2 - 1.2 } });
    b.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 }, enableContactEvents: true, enableSensorEvents: true });
    boxes.push(b);
  }
  return { world, floor, wall, zone, boxes };
}

const U = u => ({ u });
const SPHERE = 1, CAPSULE = 2, SENSORS = 4, MASK = 8;

test('queryBatch: rays, shape casts, overlaps and closest points equal the object calls', () => {
  const { world, boxes } = level();
  for (let i = 0; i < 20; i++) world.step(DT, SUBSTEPS);
  const sphere = { points: [{ x: 0, y: 0, z: 0 }], radius: 0.25 };
  const capsule = { points: [{ x: 0, y: -0.4, z: 0 }, { x: 0, y: 0.4, z: 0 }], radius: 0.3 };
  const queries = [];
  for (let i = 0; i < 12; i++) {
    const x = (i % 4) * 1.5 - 3, z = Math.floor(i / 4) * 1.2 - 1.2, sensors = i % 3 === 0, mask = i % 4 === 1 ? 2 : undefined;
    const filter = { ...sensors ? { includeSensors: true } : {}, ...mask ? { maskBits: mask } : {} };
    const flags = (sensors ? SENSORS : 0) | (mask ? MASK : 0);
    queries.push({ call: () => world.castRay({ x, y: 5, z }, { x: 0, y: -10, z: 0 }, filter), rec: [U(0), U(flags), 0, 0, x, 5, z, 0, -10, 0, 0, U(mask ?? 0)] });
    queries.push({ call: () => world.castShape({ x: -6, y: 0.5, z }, sphere, { x: 12, y: 0, z: 0 }, filter), rec: [U(1), U(flags | SPHERE), 0.25, 0, -6, 0.5, z, 12, 0, 0, 0, U(mask ?? 0)] });
    queries.push({ call: () => world.overlapShape({ x, y: 0.7, z }, capsule, filter), rec: [U(2), U(flags | CAPSULE), 0.3, 0.4, x, 0.7, z, 0, 0, 0, 0, U(mask ?? 0)], overlap: true });
    queries.push({ call: () => world.closestPoint({ x, y: 2.5, z }, 3, filter), rec: [U(3), U(flags), 0, 0, x, 2.5, z, 0, 0, 0, 3, U(mask ?? 0)] });
  }
  writeIn(queries.map(q => q.rec));
  const words = world.queryBatch(queries.length);
  const { u, f } = out(words);
  let at = 0, hits = 0;
  for (const q of queries) {
    const want = q.call();
    if (q.overlap) {
      assert.equal(u[at], want.length);
      for (let i = 0; i < want.length; i++) assert.equal(u[at + 1 + 2 * i], want[i].bodyIndex);
      at += 1 + 2 * u[at];
      hits += want.length;
      continue;
    }
    assert.equal(u[at], want.hit ? 1 : 0);
    if (want.hit) {
      hits++;
      const v = want.normal ?? want.point;
      assert.equal(f[at + 1], f32(want.fraction ?? want.distance));
      assert.deepEqual([f[at + 2], f[at + 3], f[at + 4]], [f32(v.x), f32(v.y), f32(v.z)]);
      assert.equal(u[at + 5], want.bodyIndex);
      assert.equal(u[at + 6], world.getBody(want.bodyIndex, u[at + 6]).getGeneration());
    } else {
      assert.equal(u[at + 5], 0xffffffff);
    }
    at += 7;
  }
  assert.equal(at, words);
  assert.ok(hits > 20, `only ${hits} hits`);
  assert.ok(boxes.length === 9);
  world.destroy();
});

test('queryBatch: an unknown kind ends the batch, a count beyond the input gives nothing', () => {
  const { world } = level();
  writeIn([[U(0), U(0), 0, 0, 0, 5, 0, 0, -10, 0, 0, U(0)], [U(9), U(0), 0, 0, 0, 0, 0, 0, 0, 0, 0, U(0)], [U(0), U(0), 0, 0, 0, 5, 0, 0, -10, 0, 0, U(0)]]);
  assert.equal(world.queryBatch(3), 7);
  assert.equal(world.queryBatch(1 << 20), 0);
  assert.equal(world.queryBatch(-1), 0);
  world.destroy();
});

test('readMoved: the move events with the velocities', () => {
  const { world } = level();
  world.step(DT, SUBSTEPS);
  const typed = world.getBodyMoveEvents();
  const n = typed.count, index = typed.bodyIndex.slice(), transform = typed.transform.slice();
  assert.equal(world.readMoved(), n);
  const { u, f } = out(16 * n);
  for (let i = 0; i < n; i++) {
    const r = 16 * i, body = world.getBody(u[r], u[r + 1]);
    assert.equal(u[r], index[i]);
    assert.equal(body.getIndex(), index[i]);
    assert.deepEqual([...f.slice(r + 2, r + 9)], [...transform.slice(7 * i, 7 * i + 7)]);
    const v = body.getLinearVelocity(), w = body.getAngularVelocity();
    assert.deepEqual([...f.slice(r + 9, r + 15)], [v.x, v.y, v.z, w.x, w.y, w.z].map(f32));
    assert.equal(u[r + 15], 0);
  }
  world.destroy();
});

test('readEvents: contact begin and end, sensor begin and end, as the object events', () => {
  const { world, zone, boxes } = level();
  const seen = { 1: 0, 2: 0, 3: 0, 4: 0 };
  const zoneIndex = zone.getIndex();
  for (let s = 0; s < 120; s++) {
    // A box crosses the sensor and leaves: the sensor events; launched upward it ends its contacts (a teleport does not end them, and does not wake a sleeping body).
    if (s === 30) boxes[0].setTransform({ x: -3, y: 0.5, z: 0 }, { x: 0, y: 0, z: 0, w: 1 });
    if (s === 40) boxes[0].setLinearVelocity({ x: 0, y: 12, z: 0 });
    world.step(DT, SUBSTEPS);
    const contacts = world.getContactEvents(), sensors = world.getSensorEvents();
    const n = world.readEvents();
    assert.equal(n, contacts.begin.length + contacts.end.length + sensors.begin.length + sensors.end.length);
    const { u, f } = out(10 * n);
    const expected = [
      ...contacts.begin.map(e => [1, e.shapeUserDataA, e.shapeUserDataB]),
      ...contacts.end.map(e => [2, e.shapeUserDataA, e.shapeUserDataB]),
      ...sensors.begin.map(e => [3, e.sensorUserData, e.visitorUserData]),
      ...sensors.end.map(e => [4, e.sensorUserData, e.visitorUserData]),
    ];
    for (let i = 0; i < n; i++) {
      const r = 10 * i, [kind] = expected[i];
      assert.equal(u[r], kind);
      seen[kind]++;
      if (kind >= 3) assert.equal(u[r + 3], zoneIndex);
      if (kind === 1) assert.ok(Math.abs(Math.hypot(f[r + 7], f[r + 8], f[r + 9]) - 1) < 1e-5, 'a unit normal');
      else assert.deepEqual([...f.slice(r + 7, r + 10)], [0, 0, 0]);
    }
  }
  for (const k of [1, 2, 3, 4]) assert.ok(seen[k] > 0, `no event of kind ${k}`);
  world.destroy();
});

test('writeBodies: each op equals its object call; stale and unknown records are skipped', () => {
  const twin = () => {
    const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
    const bodies = [0, 1, 2, 3, 4, 5].map(i => {
      const b = world.createBody({ type: i === 3 ? 'kinematic' : 'dynamic', position: { x: 3 * i, y: 2, z: 0 } });
      b.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 } });
      return b;
    });
    return { world, bodies };
  };
  const a = twin(), b = twin();
  const q = { x: 0, y: Math.SQRT1_2, z: 0, w: Math.SQRT1_2 };
  a.bodies[0].setLinearVelocity({ x: 1, y: 2, z: 3 });
  a.bodies[1].setAngularVelocity({ x: 0.5, y: -1, z: 2 });
  a.bodies[2].setTransform({ x: 1, y: 4, z: 1 }, q);
  a.bodies[3].setTargetTransform({ position: { x: 10, y: 3, z: 1 }, rotation: q }, DT, true);
  a.bodies[4].applyLinearImpulseToCenter({ x: 0, y: 5, z: 1 }, true);
  a.bodies[5].applyAngularImpulse({ x: 1, y: 0, z: 0.5 }, true);
  const id = i => [U(b.bodies[i].getIndex()), U(b.bodies[i].getGeneration())];
  writeIn([
    [U(0), ...id(0), 1, 2, 3, 0, 0, 0, 0, 0, 0],
    [U(1), ...id(1), 0.5, -1, 2, 0, 0, 0, 0, 0, 0],
    [U(2), ...id(2), 1, 4, 1, q.x, q.y, q.z, q.w, 0, 0],
    [U(3), ...id(3), 10, 3, 1, q.x, q.y, q.z, q.w, DT, 0],
    [U(4), ...id(4), 0, 5, 1, 0, 0, 0, 0, 0, 0],
    [U(5), ...id(5), 1, 0, 0.5, 0, 0, 0, 0, 0, 0],
    [U(6), ...id(0), 9, 9, 9, 0, 0, 0, 0, 0, 0],
    [U(0), U(b.bodies[0].getIndex()), U(b.bodies[0].getGeneration() + 1), 9, 9, 9, 0, 0, 0, 0, 0, 0],
    [U(0), U(4096), U(0), 9, 9, 9, 0, 0, 0, 0, 0, 0],
  ]);
  assert.equal(b.world.writeBodies(9), 6);
  for (let s = 0; s < 10; s++) { a.world.step(DT, SUBSTEPS); b.world.step(DT, SUBSTEPS); }
  for (let i = 0; i < 6; i++) {
    assert.deepEqual(b.bodies[i].getTransform(), a.bodies[i].getTransform(), `body ${i}`);
    assert.deepEqual(b.bodies[i].getLinearVelocity(), a.bodies[i].getLinearVelocity(), `body ${i}`);
  }
  b.bodies[0].destroy();
  writeIn([[U(0), ...[U(a.bodies[0].getIndex()), U(0)], 1, 1, 1, 0, 0, 0, 0, 0, 0]]);
  assert.equal(b.world.writeBodies(1), 0, 'a destroyed body');
  assert.equal(b.world.writeBodies(1 << 20), 0);
  a.world.destroy();
  b.world.destroy();
});
