import assert from 'node:assert/strict';
import { test } from 'node:test';

import Box3D from '../dist/box3d.mjs';

// The flat calls (flatIn, flatOut, World.queryBatch, readMoved, readEvents, writeBodies) against the object calls
// they stand for: the same answers, word for word.

const b3 = await Box3D();

const DT = 1 / 60;
const SUBSTEPS = 4;
const f32 = Math.fround;

/** Writes records of equal length (numbers; an integer word is given as { u }) at flatIn. */
function writeIn(records) {
  const n = records[0].length;
  const at = b3.flatIn(n * records.length) >> 2;
  const u = b3.HEAPU32, f = b3.HEAPF32;
  records.forEach((r, i) => r.forEach((w, j) => {
    if (typeof w === 'object') u[at + n * i + j] = w.u;
    else f[at + n * i + j] = w;
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

test('moverBatch: a move equals the loop over collideMover, solvePlanes and castMover; cast and ground as the object calls', () => {
  const { world, wall } = level();
  // A ramp at 35 degrees: a wall for a mover whose least ground normal y is cos 30.
  const ramp = world.createBody({ type: 'static', position: { x: -2, y: 0, z: 3 }, rotation: { x: 0, y: 0, z: Math.sin(-0.3054), w: Math.cos(-0.3054) } });
  ramp.createBox({ halfExtents: { x: 2, y: 0.2, z: 1 } });
  const capsule = { center1: { x: 0, y: -0.5, z: 0 }, center2: { x: 0, y: 0.5, z: 0 }, radius: 0.3 };
  const minY = Math.fround(Math.cos(Math.PI / 6));
  const f = Math.fround;
  // The loop as a JS caller writes it over the object calls, positions in doubles.
  const loop = (from, d, walls) => {
    let p = from;
    let run = 0;
    for (; run < 5;) {
      run++;
      const planes = world.collideMover({ x: p[0], y: p[1], z: p[2] }, capsule, undefined).map(pl => {
        const n = pl.normal, h = Math.hypot(n.x, n.z), flat = walls && n.y < minY && h > 1e-6;
        return { normal: flat ? { x: n.x / h, y: 0, z: n.z / h } : n, offset: pl.offset };
      });
      const e = b3.solvePlanes({ x: from[0] + d[0] - p[0], y: from[1] + d[1] - p[1], z: from[2] + d[2] - p[2] }, planes).delta;
      const t = world.castMover({ x: p[0], y: p[1], z: p[2] }, capsule, e, undefined);
      p = [p[0] + t * e.x, p[1] + t * e.y, p[2] + t * e.z];
      if (Math.hypot(t * e.x, t * e.y, t * e.z) < 1e-4) break;
    }
    return { p, run };
  };
  const rec = (kind, flags, at, d) => [U(kind), U(flags), ...at, 0, -0.5, 0, 0, 0.5, 0, 0.3, ...d, minY, U(0), U(0), U(0), U(0)];
  const moves = [
    [[0, 0.81, 0], [6, 0, 0], 1], [[0, 0.81, 0], [6, -1, 0], 1], [[0, 0.81, 2.5], [-3, 0, 0], 1], [[0, 0.81, 2.5], [-3, 0, 0], 0],
    [[-1, 2, 3], [0, -3, 0], 1], [[3, 0.81, 0.5], [3, 0, -1], 1],
  ].map(([at, d, walls]) => [at.map(f), d.map(f), walls]);
  writeIn([
    ...moves.map(([at, d, walls]) => rec(0, walls, at, d)),
    rec(1, 0, [0, 0.81, 0].map(f), [10, 0, 0]),
    rec(2, 0, [0, 0.79, 0].map(f), [0, 0, 0]),
    rec(2, 0, [8, 5, 8].map(f), [0, 0, 0]),
  ]);
  assert.equal(world.moverBatch(moves.length + 3), moves.length + 3);
  const { u, f: o } = out(4 * (moves.length + 3));
  moves.forEach(([at, d, walls], i) => {
    const want = loop(at, d, walls === 1);
    assert.deepEqual([o[4 * i], o[4 * i + 1], o[4 * i + 2]], want.p.map(f), `move ${i}`);
    assert.equal(u[4 * i + 3], want.run, `move ${i} rounds`);
  });
  const c = 4 * moves.length;
  assert.equal(o[c], f(world.castMover({ x: 0, y: f(0.81), z: 0 }, capsule, { x: 10, y: 0, z: 0 }, undefined)));
  assert.ok(o[c] > 0 && o[c] < 1, 'stops at a box');
  assert.equal(u[c + 4], 1, 'the floor under a standing capsule');
  assert.ok(Math.abs(o[c + 7] - 1) < 1e-3, 'the floor');
  assert.equal(u[c + 8], 0, 'nothing below a capsule in the air');
  assert.equal(u[c + 9], 0xffffffff);
  assert.ok(wall.getIndex() > 0);
  world.destroy();
});

test('flat calls: the memory grows during a call, the results read through new views are right', () => {
  const { world } = level();
  // An overlap of radius 3 at the grid: about ten shapes a query, so 400 000 queries write far more than the module's
  // first memory, and the output grows inside the call.
  const one = [U(2), U(SPHERE), 3, 0, 0, 0.7, 0, 0, 0, 0, 0, U(0)];
  writeIn([one]);
  const words = world.queryBatch(1);
  const want = [...out(words).u];
  assert.ok(want[0] >= 5, `only ${want[0]} shapes`);
  const before = b3.HEAPU32;
  const n = 400000;
  writeIn(Array.from({ length: n }, () => one));
  const total = world.queryBatch(n);
  assert.equal(total, n * words);
  assert.equal(before.length, 0, 'the old view was detached: the memory grew');
  const u = b3.HEAPU32, at = b3.flatOut() >> 2;
  for (const i of [0, 1, n >> 1, n - 1]) assert.deepEqual([...u.subarray(at + i * words, at + (i + 1) * words)], want, `query ${i}`);
  world.destroy();
});

test('moverBatch: a mask leaves a layer out, as castMover with maskBits', () => {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const floor = world.createBody({ type: 'static', position: { x: 0, y: -0.5, z: 0 } });
  floor.createBox({ halfExtents: { x: 10, y: 0.5, z: 10 } });
  const wall = world.createBody({ type: 'static', position: { x: 5, y: 0.5, z: 0 } });
  wall.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 }, filter: { categoryBits: 2, maskBits: 0xffffffff, groupIndex: 0 } });
  const capsule = { center1: { x: 0, y: -0.5, z: 0 }, center2: { x: 0, y: 0.5, z: 0 }, radius: 0.3 };
  const at = { x: 0, y: 0.81, z: 0 }, along = { x: 10, y: 0, z: 0 };
  const rec = (flags, mask) => [U(1), U(flags), at.x, at.y, at.z, 0, -0.5, 0, 0, 0.5, 0, 0.3, along.x, along.y, along.z, 0, U(mask), U(0), U(0), U(0)];
  writeIn([rec(0, 0), rec(4, 1), rec(4, 2)]);
  assert.equal(world.moverBatch(3), 3);
  const { f } = out(12);
  assert.equal(f[0], Math.fround(world.castMover(at, capsule, along, undefined)));
  assert.equal(f[4], Math.fround(world.castMover(at, capsule, along, { maskBits: 1 })));
  assert.equal(f[8], Math.fround(world.castMover(at, capsule, along, { maskBits: 2 })));
  assert.ok(f[0] < 0.5 && f[4] === 1 && f[8] === f[0], `fractions ${f[0]} ${f[4]} ${f[8]}`);
  world.destroy();
});

test('flat calls: a count whose words pass 2^32 gives nothing (the bound is not fooled by a wrapped product)', () => {
  const { world } = level();
  writeIn([[U(0), U(0), 0, 0, 0, 5, 0, 0, -10, 0, 0, U(0)], [U(0), U(0), 0, 0, 0, 5, 0, 0, -10, 0, 0, U(0)]]);
  // 12 * 357913942 = 2^32 + 8 and 20 * 214748365 = 2^32 + 4: in 32 bits 8 and 4 words, within any input.
  assert.equal(world.queryBatch(357913942), 0);
  assert.equal(world.writeBodies(357913942), 0);
  assert.equal(world.moverBatch(214748365), 0);
  world.destroy();
});

test('flat buffers: never at address 0; a mover record asking for 2^32 - 1 rounds runs at most 64', () => {
  assert.ok(b3.flatIn(0) > 0);
  assert.ok(b3.flatOut() > 0);
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  const floor = world.createBody({ type: 'static', position: { x: 0, y: -0.5, z: 0 } });
  floor.createBox({ halfExtents: { x: 10, y: 0.5, z: 10 } });
  // A move into the floor that never settles below 1e-4 a round would run every round it is allowed.
  writeIn([[U(0), U(0), 0, 0.81, 0, 0, -0.5, 0, 0, 0.5, 0, 0.3, 3, -2, 1, 0, U(0), U(0xffffffff), U(0), U(0)]]);
  const t0 = performance.now();
  assert.equal(world.moverBatch(1), 1);
  assert.ok(out(4).u[3] <= 64, 'rounds run');
  assert.ok(performance.now() - t0 < 1000);
  world.destroy();
});

test('queryBatch with a mask: a shape is seen by its category, whatever its own mask holds', () => {
  const world = new b3.World({ gravity: { x: 0, y: 0, z: 0 } });
  const body = world.createBody({ type: 'static', position: { x: 5, y: 0, z: 0 } });
  body.createSphere({ radius: 1, filter: { categoryBits: 4, maskBits: 4 } });
  world.step(DT, SUBSTEPS);
  const ray = (mask) => [U(0), U(MASK), 0, 0, 0, 0, 0, 10, 0, 0, 0, U(mask)];
  writeIn([ray(4), ray(8)]);
  const { u } = out(world.queryBatch(2));
  assert.equal(u[0], 1);
  assert.equal(u[7], 0);
  world.destroy();
});

test('a ray that starts inside a shape passes it and hits what is behind; a shape cast stays at distance 0', () => {
  const world = new b3.World({ gravity: { x: 0, y: 0, z: 0 } });
  world.createBody({ type: 'static', position: { x: 0, y: 0, z: 0 } }).createSphere({ radius: 1 });
  world.createBody({ type: 'static', position: { x: 5, y: 0, z: 0 } }).createSphere({ radius: 1 });
  world.step(DT, SUBSTEPS);
  const along = { x: 10, y: 0, z: 0 };
  assert.ok(Math.abs(world.castRay({ x: 0, y: 0, z: 0 }, along, {}).fraction * 10 - 4) < 1e-3);
  writeIn([[U(0), U(0), 0, 0, 0, 0, 0, 10, 0, 0, 0, U(0)]]);
  const { u, f } = out(world.queryBatch(1));
  assert.equal(u[0], 1);
  assert.ok(Math.abs(f[1] * 10 - 4) < 1e-3);
  const cast = world.castShape({ x: 0, y: 0, z: 0 }, { points: [{ x: 0, y: 0, z: 0 }], radius: 0.25 }, along, {});
  assert.equal(cast.hit, true);
  assert.equal(cast.fraction, 0);
  world.destroy();
});

test('queryBatch with flag 16: each hit ends in its shape\'s user data, on every kind', () => {
  const world = new b3.World({ gravity: { x: 0, y: 0, z: 0 } });
  const body = world.createBody({ type: 'static', position: { x: 5, y: 0, z: 0 } });
  body.createSphere({ radius: 1 }).setUserData(7);
  world.step(DT, SUBSTEPS);
  const q = (kind, shape) => [U(kind), U(16 | shape), shape ? 0.25 : 0, 0, 0, 0, 0, 20, 0, 0, 20, U(0)];
  writeIn([q(0, 0), q(1, 1), [U(2), U(16 | 1), 9, 0, 0, 0, 0, 0, 0, 0, 0, U(0)], q(3, 0), q(4, 0), q(5, 1)]);
  const words = world.queryBatch(6);
  assert.equal(words, 8 + 8 + 4 + 8 + 8 + 8);
  const { u } = out(words);
  for (const at of [7, 15, 19, 27, 35, 43]) assert.equal(u[at], 7, `word ${at}`);
  world.destroy();
});

test('queryBatch kinds 4 and 5 give every hit: a mask, the shape a ray starts in, a sensor, and the word count', () => {
  const world = new b3.World({ gravity: { x: 0, y: 0, z: 0 } });
  const at = (x, opts = {}) => {
    const body = world.createBody({ type: 'static', position: { x, y: 0, z: 0 } });
    body.createSphere({ radius: 1, ...opts });
    return body;
  };
  at(0);
  at(5);
  at(8, { filter: { categoryBits: 2, maskBits: 2 } });
  at(11, { isSensor: true, enableSensorEvents: true });
  world.step(DT, SUBSTEPS);
  const q = (kind, mask, shape = 0) => [U(kind), U(MASK | shape), shape ? 0.25 : 0, 0, 0, 0, 0, 20, 0, 0, 0, U(mask)];
  writeIn([q(4, 0xffffffff), q(4, 1), q(5, 0xffffffff, 1), q(0, 1)]);
  const words = world.queryBatch(4);
  assert.equal(words, (1 + 2 * 6) + (1 + 6) + (1 + 3 * 6) + 7);
  const { u, f } = out(words);
  const hits = (from) => Array.from({ length: u[from] }, (_, k) => f[from + 1 + 6 * k] * 20).sort((a, b) => a - b);
  const near = (from, want) => {
    const got = hits(from);
    assert.equal(got.length, want.length);
    got.forEach((d, k) => assert.ok(Math.abs(d - want[k]) < 0.02, `${d} and ${want[k]}`)); // A cast stops about 1 cm short.
  };
  near(0, [4, 7]);
  near(13, [4]);
  near(20, [0, 3.75, 6.75]);
  assert.equal(u[39], 1);
  world.destroy();
});

test('readContacts: a begin with its normal and point, a hit with its speed, a sensor entered and left', () => {
  const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
  world.createBody({ type: 'static', position: { x: 0, y: -0.5, z: 0 } }).createBox({ halfExtents: { x: 10, y: 0.5, z: 10 }, enableHitEvents: true });
  const zone = world.createBody({ type: 'static', position: { x: 3, y: 1, z: 0 } });
  zone.createBox({ halfExtents: { x: 1, y: 1, z: 1 }, isSensor: true, enableSensorEvents: true });
  const ball = world.createBody({ type: 'dynamic', position: { x: 3, y: 4, z: 0 } });
  ball.createSphere({ radius: 0.5, enableContactEvents: true, enableHitEvents: true, enableSensorEvents: true, restitution: 0 });
  const seen = [];
  for (let t = 0; t < 120; t++) {
    world.step(DT, SUBSTEPS);
    const n = world.readContacts(), { u, f } = out(12 * n);
    for (let i = 0; i < n; i++) seen.push({ kind: u[12 * i], ny: f[12 * i + 6], x: f[12 * i + 8], y: f[12 * i + 9], speed: f[12 * i + 11] });
  }
  const of = kind => seen.filter(e => e.kind === kind);
  assert.equal(of(4).length, 1);
  assert.equal(of(5).length, 0);
  assert.equal(of(1).length, 1);
  assert.ok(Math.abs(Math.abs(of(1)[0].ny) - 1) < 1e-3);
  assert.ok(Math.abs(of(1)[0].x - 3) < 0.05 && Math.abs(of(1)[0].y) < 0.05, JSON.stringify(of(1)[0]));
  assert.equal(of(3).length, 1);
  assert.ok(of(3)[0].speed > 5, JSON.stringify(of(3)[0]));
  world.destroy();
});
