import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { test } from 'node:test';
import { pathToFileURL } from 'node:url';
import { clearTimeout, setTimeout } from 'node:timers';
import { isMainThread, parentPort, Worker, workerData } from 'node:worker_threads';

// The object calls against a reference build, the files of a previous release: the same seeded scenarios on both modules, every
// value the calls return compared. Set B3_REFERENCE to a folder with the reference (its box3d.scalar.mjs and .wasm);
// without it the test is skipped. Build one from a worktree, for example of 8f045f2 (the glue before the faster
// conversions): `git worktree add ../box3d-wasm-reference 8f045f2`, then `npm run fetch-deps` and
// `FLAVOURS=scalar npm run build` there. B3_SEEDS sets the number of scenarios (default 40).
//
// The generator varies how JS hands values to the glue (numbers, numeric strings, booleans, null, missing fields,
// integers given as fractions) and keeps the geometry sane, so both builds take the same path through Box3D.
// One difference is known and left out: an explicit NaN for filter bits (the reference converts it to 0, the current
// glue takes the default). Teleports stay above the ground: a body moved deep into it can make Box3D unstable, and both
// builds then hang in step (Box3D at 29bf523c; seeds 28, 30 and 31 of an earlier generator).

const reference = process.env.B3_REFERENCE;
const seeds = Number(process.env.B3_SEEDS ?? 40);
// A scenario runs in a worker: some leave Box3D unstable (it logs 'Box3D: unstable'), and a step can then spin.
const limitMs = Number(process.env.B3_SCENARIO_MS ?? 10000);

const load = async dir => (await import(pathToFileURL(join(dir, 'box3d.scalar.mjs')).href)).default({ wasmBinary: readFileSync(join(dir, 'box3d.scalar.wasm')) });

/** mulberry32 */
function prng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Plain data from whatever a call returns: typed arrays as arrays, -0 kept apart from 0, NaN named. */
function plain(v) {
  if (v === undefined) return '<undefined>';
  if (typeof v === 'number') return Object.is(v, -0) ? '-0' : Number.isNaN(v) ? 'NaN' : v;
  if (ArrayBuffer.isView(v)) return Array.from(v, plain);
  if (Array.isArray(v)) return v.map(plain);
  if (v && typeof v === 'object') return Object.fromEntries(Object.keys(v).sort().map(k => [k, plain(v[k])]));
  return v;
}

/** One scenario: every call and its result, in order. */
function scenario(b3, seed) {
  const r = prng(seed);
  const log = [];
  const rec = (label, f) => {
    let v;
    try {
      v = plain(f());
    } catch (e) {
      v = `<threw ${e?.name ?? typeof e}>`;
    }
    log.push([label, v]);
  };
  const pick = list => list[Math.floor(r() * list.length)];
  const chance = p => r() < p;
  const real = (lo, hi) => lo + (hi - lo) * r();
  // A number as JS callers hand it over: a number, a numeric string, an integer, sometimes nothing.
  const num = (lo, hi) => {
    const x = real(lo, hi);
    return pick([x, x, x, String(x), Math.round(x), undefined, null]);
  };
  const flag = () => pick([true, false, 0, 1, 2, '', 'yes', undefined, null]);
  const int = (lo, hi) => pick([Math.floor(real(lo, hi)), real(lo, hi), String(Math.floor(real(lo, hi))), undefined]);
  const vec = (lo, hi) => {
    if (chance(0.1)) return pick([undefined, null]);
    const v = { x: num(lo, hi), y: num(lo, hi), z: num(lo, hi) };
    if (chance(0.15)) delete v[pick(['x', 'y', 'z'])];
    return v;
  };
  const sizeVec = (lo, hi) => ({ x: real(lo, hi), y: real(lo, hi), z: real(lo, hi) });
  const quat = () => {
    // Only whole unit quaternions or none: a partial one is filled from the fallback and is not a rotation.
    if (chance(0.15)) return pick([undefined, null]);
    const q = [r() - 0.5, r() - 0.5, r() - 0.5, r() - 0.5], n = Math.hypot(...q);
    return { x: q[0] / n, y: q[1] / n, z: q[2] / n, w: q[3] / n };
  };
  const bits = () => pick([undefined, 1, 2, 3, 0xffff, 0xffffffff, 2 ** 40, '3', 0]);
  const filterOpts = () => pick([undefined, null, {}, { maskBits: bits() }, { categoryBits: bits(), maskBits: bits(), includeSensors: flag() }]);

  const world = new b3.World({
    gravity: chance(0.8) ? { x: 0, y: -10, z: 0 } : vec(-12, 12), enableSleep: flag(), enableContinuous: flag(),
    contactHertz: chance(0.5) ? undefined : real(10, 60), contactDampingRatio: chance(0.5) ? undefined : real(1, 20),
    restitutionThreshold: num(0, 2), hitEventThreshold: num(0, 2), maximumLinearSpeed: chance(0.7) ? undefined : real(50, 400),
  });
  const ground = world.createBody({ type: 'static', position: { x: 0, y: -1, z: 0 } });
  ground.createBox({ halfExtents: { x: 30, y: 1, z: 30 }, friction: num(0, 1), enableContactEvents: true });

  const bodies = [ground], shapes = [], joints = [];
  const n = 6 + Math.floor(r() * 8);
  for (let i = 0; i < n; i++) {
    const type = pick(['dynamic', 'dynamic', 'dynamic', 'kinematic', 'static']);
    const body = world.createBody({
      type, position: { x: real(-6, 6), y: real(0.5, 6), z: real(-6, 6) }, rotation: quat(), linearVelocity: vec(-3, 3),
      angularVelocity: vec(-2, 2), linearDamping: num(0, 1), angularDamping: num(0, 1), gravityScale: num(0, 2),
      sleepThreshold: num(0.01, 0.2), enableSleep: flag(), isAwake: flag(), isBullet: flag(), isEnabled: chance(0.9) ? undefined : flag(),
      allowFastRotation: flag(), motionLocks: chance(0.8) ? undefined : { linearX: flag(), angularY: flag(), angularZ: flag() },
      ...chance(0.3) ? { userData: Math.floor(real(1, 1000)) } : {}, ...chance(0.2) ? { name: `b${i}` } : {},
    });
    bodies.push(body);
    const parts = 1 + Math.floor(r() * 2);
    for (let p = 0; p < parts; p++) {
      const material = {
        density: num(0.2, 3), friction: num(0, 1), restitution: num(0, 0.8), rollingResistance: num(0, 0.2), tangentVelocity: chance(0.8) ? undefined : vec(-1, 1),
        isSensor: chance(0.1) ? true : undefined, enableSensorEvents: flag(), enableContactEvents: flag(), enableHitEvents: flag(), updateBodyMass: chance(0.9) ? undefined : flag(),
        ...chance(0.3) ? { filter: { categoryBits: pick([1, 2, 4, '2']), maskBits: pick([0xffffffff, 1, 3, 2 ** 40]), groupIndex: int(-2, 3) } } : {},
      };
      const kind = pick(['box', 'boxHx', 'sphere', 'capsule', 'capsuleHeight', 'hull']);
      rec(`create ${kind}`, () => {
        let s;
        if (kind === 'box') s = body.createBox({ halfExtents: sizeVec(0.2, 1), ...chance(0.3) ? { offset: vec(-0.5, 0.5), rotation: quat() } : {}, ...material });
        else if (kind === 'boxHx') s = body.createBox({ hx: real(0.2, 1), hy: real(0.2, 1), hz: real(0.2, 1), ...material });
        else if (kind === 'sphere') s = body.createSphere({ radius: real(0.2, 0.8), center: vec(-0.3, 0.3), ...material });
        else if (kind === 'capsule') s = body.createCapsule({ radius: real(0.1, 0.5), center1: { x: 0, y: -real(0.2, 0.6), z: 0 }, center2: { x: 0, y: real(0.2, 0.6), z: 0 }, ...material });
        else if (kind === 'capsuleHeight') s = body.createCapsule({ radius: real(0.1, 0.5), height: real(0.4, 1.2), ...material });
        else {
          const pts = [];
          for (let k = 0; k < 8; k++) pts.push({ x: real(-0.6, 0.6), y: real(-0.6, 0.6), z: real(-0.6, 0.6) });
          s = body.createHull({ points: pts, maxVertices: int(4, 9), ...material });
        }
        shapes.push(s);
        return { type: s.getType(), density: s.getDensity(), friction: s.getFriction(), restitution: s.getRestitution(), sensor: s.isSensor(), filter: s.getFilter(), aabb: s.getAABB() };
      });
    }
  }

  const frame = () => chance(0.5) ? undefined : { position: vec(-1, 1), rotation: quat() };
  const jointKinds = ['distance', 'revolute', 'spherical', 'prismatic', 'weld', 'motor', 'wheel', 'parallel', 'filter'];
  for (let j = 0; j < 4 + Math.floor(r() * 6); j++) {
    const a = pick(bodies), b = pick(bodies);
    if (a === b) continue;
    const kind = pick(jointKinds);
    const base = { localFrameA: frame(), localFrameB: frame(), ...chance(0.3) ? { anchorA: vec(-1, 1), anchorB: vec(-1, 1) } : {}, collideConnected: flag(), forceThreshold: chance(0.8) ? undefined : real(1, 100), torqueThreshold: chance(0.8) ? undefined : real(1, 100) };
    const spring = { enableSpring: flag(), hertz: num(0, 8), dampingRatio: num(0, 2) };
    const opts = {
      distance: { length: num(0.5, 3), ...spring, lowerSpringForce: num(-50, 0), upperSpringForce: num(0, 50), enableLimit: flag(), minLength: num(0.2, 1), maxLength: num(1, 4), enableMotor: flag(), maxMotorForce: num(0, 50), motorSpeed: num(-2, 2) },
      revolute: { targetAngle: num(-1, 1), ...spring, enableLimit: flag(), lowerAngle: num(-1, 0), upperAngle: num(0, 1), enableMotor: flag(), maxMotorTorque: num(0, 50), motorSpeed: num(-2, 2) },
      spherical: { ...spring, targetRotation: quat(), enableConeLimit: flag(), coneAngle: num(0.1, 1), enableTwistLimit: flag(), lowerTwistAngle: num(-1, 0), upperTwistAngle: num(0, 1), enableMotor: flag(), maxMotorTorque: num(0, 50), motorVelocity: vec(-1, 1) },
      prismatic: { ...spring, targetTranslation: num(-1, 1), enableLimit: flag(), lowerTranslation: num(-1, 0), upperTranslation: num(0, 1), enableMotor: flag(), maxMotorForce: num(0, 50), motorSpeed: num(-2, 2) },
      weld: { linearHertz: num(0, 8), angularHertz: num(0, 8), linearDampingRatio: num(0, 2), angularDampingRatio: num(0, 2) },
      motor: { linearVelocity: vec(-1, 1), maxVelocityForce: num(0, 50), angularVelocity: vec(-1, 1), maxVelocityTorque: num(0, 50), linearHertz: num(0, 8), linearDampingRatio: num(0, 2), maxSpringForce: num(0, 50), angularHertz: num(0, 8), angularDampingRatio: num(0, 2), maxSpringTorque: num(0, 50) },
      wheel: { enableSuspensionSpring: flag(), suspensionHertz: num(0, 8), suspensionDampingRatio: num(0, 2), enableSuspensionLimit: flag(), lowerSuspensionLimit: num(-0.5, 0), upperSuspensionLimit: num(0, 0.5), enableSpinMotor: flag(), maxSpinTorque: num(0, 50), spinSpeed: num(-5, 5), enableSteering: flag(), steeringHertz: num(0, 8), steeringDampingRatio: num(0, 2), targetSteeringAngle: num(-0.5, 0.5), maxSteeringTorque: num(0, 50), enableSteeringLimit: flag(), lowerSteeringLimit: num(-0.5, 0), upperSteeringLimit: num(0, 0.5) },
      parallel: { hertz: num(0, 8), dampingRatio: num(0, 2), maxTorque: num(0, 50) },
      filter: {},
    }[kind];
    const make = { distance: 'createDistanceJoint', revolute: 'createRevoluteJoint', spherical: 'createSphericalJoint', prismatic: 'createPrismaticJoint', weld: 'createWeldJoint', motor: 'createMotorJoint', wheel: 'createWheelJoint', parallel: 'createParallelJoint', filter: 'createFilterJoint' }[kind];
    rec(`joint ${kind}`, () => {
      const joint = world[make](a, b, { ...base, ...opts });
      joints.push(joint);
      return { type: joint.getType(), collide: joint.getCollideConnected(), frameA: joint.getLocalFrameA(), frameB: joint.getLocalFrameB() };
    });
  }

  // Setters with values as callers hand them over; each picked at random a step.
  const bodySetters = [
    b => b.setLinearVelocity(vec(-3, 3)), b => b.setAngularVelocity(vec(-2, 2)), b => b.setTransform({ x: num(-6, 6), y: real(1, 6), z: num(-6, 6) }, quat()),
    b => b.setTargetTransform({ position: { x: num(-6, 6), y: real(1, 6), z: num(-6, 6) }, rotation: quat() }, 1 / 60, chance(0.5)),
    b => b.applyForce(vec(-20, 20), vec(-1, 1), chance(0.5)), b => b.applyForceToCenter(vec(-20, 20), true), b => b.applyTorque(vec(-5, 5), true),
    b => b.applyLinearImpulse(vec(-2, 2), vec(-1, 1), true), b => b.applyLinearImpulseToCenter(vec(-2, 2), chance(0.5)), b => b.applyAngularImpulse(vec(-1, 1), true),
    b => b.setLinearDamping(real(0, 1)), b => b.setAngularDamping(real(0, 1)), b => b.setGravityScale(real(0, 2)), b => b.setAwake(chance(0.5)),
    b => b.setBullet(chance(0.5)), b => b.setMotionLocks({ linearY: flag(), angularX: flag() }), b => b.enableSleep(chance(0.5)),
  ];
  const shapeSetters = [
    s => s.setFriction(real(0, 1)), s => s.setRestitution(real(0, 0.8)), s => s.setDensity(real(0.2, 3), true),
    s => s.setFilter({ categoryBits: pick([1, 2, '4']), maskBits: pick([0xffffffff, 3, undefined]), groupIndex: int(-1, 2) }), s => s.enableContactEvents(chance(0.5)), s => s.enableHitEvents(chance(0.5)),
  ];
  const jointSetters = {
    setLength: () => real(0.5, 3), setSpringHertz: () => real(0, 8), setSpringDampingRatio: () => real(0, 2), setMotorSpeed: () => real(-2, 2), setTargetAngle: () => real(-1, 1),
    setMaxMotorTorque: () => real(0, 50), setMaxMotorForce: () => real(0, 50), setTargetTranslation: () => real(-1, 1), setTargetRotation: () => quat(), setMotorVelocity: () => vec(-1, 1),
    setConeLimit: () => real(0.1, 1), setLinearVelocity: () => vec(-1, 1), setAngularVelocity: () => vec(-1, 1), setLinearHertz: () => real(0, 8), setAngularHertz: () => real(0, 8),
    setSuspensionHertz: () => real(0, 8), setSpinMotorSpeed: () => real(-5, 5), setTargetSteeringAngle: () => real(-0.5, 0.5), setMaxTorque: () => real(0, 50),
  };
  const capsule = () => ({ center1: { x: 0, y: -0.5, z: 0 }, center2: { x: 0, y: 0.5, z: 0 }, radius: num(0.2, 0.4) });

  for (let step = 0; step < 40; step++) {
    for (let k = 0; k < 3; k++) {
      const live = bodies.slice(1).filter(x => x.isValid());
      if (!live.length) break;
      const b = pick(live);
      const k = Math.floor(r() * bodySetters.length);
      rec(`body set ${k} ${step}`, () => bodySetters[k](b));
    }
    const liveShapes = shapes.filter(s => s.isValid());
    if (liveShapes.length && chance(0.3)) { const k = Math.floor(r() * shapeSetters.length); rec(`shape set ${k} ${step}`, () => shapeSetters[k](pick(liveShapes))); }
    if (joints.length && chance(0.4)) {
      const joint = pick(joints), names = !joint.isValid() ? [] : Object.keys(jointSetters).filter(m => typeof joint[m] === 'function');
      if (names.length) { const m = pick(names); rec(`joint ${m} ${step}`, () => joint[m](jointSetters[m]())); }
    }
    if (chance(0.1)) rec(`explode ${step}`, () => world.explode({ position: vec(-4, 4), radius: num(1, 4), falloff: num(0, 2), impulsePerArea: num(0, 5), maskBits: bits() }));
    world.step(1 / 60, 4);

    for (const [i, b] of bodies.entries()) {
      // A destroyed body (or a joint it took along) is not read: its memory is reused, what a read returns is undefined.
      rec(`body ${i}`, () => !b.isValid() ? 'destroyed' : ({
        valid: true, type: b.getType(), p: b.getPosition(), q: b.getRotation(), xf: b.getTransform(), v: b.getLinearVelocity(), w: b.getAngularVelocity(),
        awake: b.isAwake(), mass: b.getMass(), com: b.getLocalCenterOfMass(), wcom: b.getWorldCenterOfMass(), aabb: b.computeAABB(), locks: b.getMotionLocks(),
        gs: b.getGravityScale(), ld: b.getLinearDamping(), ad: b.getAngularDamping(), bullet: b.isBullet(), shapes: b.getShapeCount(), ud: b.getUserData(), name: b.getName(),
        local: b.getLocalPoint({ x: 1, y: 2, z: 3 }), world: b.getWorldPoint({ x: 1, y: 2, z: 3 }),
      }));
    }
    for (const [i, joint] of joints.entries()) {
      rec(`joint ${i}`, () => {
        if (!joint.isValid()) return 'destroyed';
        const o = { force: joint.getConstraintForce(), torque: joint.getConstraintTorque() };
        for (const m of ['getAngle', 'getLength', 'getCurrentLength', 'getTranslation', 'getSpeed', 'getConeAngle', 'getTwistAngle', 'getSpinSpeed', 'getSteeringAngle', 'getMotorTorque']) {
          if (typeof joint[m] === 'function') o[m] = joint[m]();
        }
        return o;
      });
    }
    rec(`contacts ${step}`, () => world.getContactEvents());
    rec(`sensors ${step}`, () => world.getSensorEvents());
    rec(`moves ${step}`, () => { const e = world.getBodyMoveEvents(); return { count: e.count, index: e.bodyIndex, ud: e.userData, xf: e.transform, asleep: e.fellAsleep }; });
    rec(`body events ${step}`, () => world.getBodyEvents());
    rec(`joint events ${step}`, () => world.getJointEvents());
    rec(`awake ${step}`, () => world.getAwakeBodyCount());

    for (let k = 0; k < 4; k++) {
      const from = vec(-8, 8);
      rec(`ray ${step}`, () => world.castRay(from, vec(-12, 12), filterOpts()));
      rec(`ray closest ${step}`, () => world.castRayClosest(vec(-8, 8), vec(-12, 12), pick([undefined, { categoryBits: 1, maskBits: bits(), groupIndex: 0 }])));
      const proxy = chance(0.5) ? { points: [vec(-0.2, 0.2)], radius: num(0.1, 0.6) } : { points: [{ x: 0, y: -0.4, z: 0 }, { x: 0, y: 0.4, z: 0 }], radius: num(0.1, 0.4) };
      rec(`cast ${step}`, () => world.castShape(vec(-8, 8), proxy, vec(-10, 10), filterOpts()));
      rec(`overlap ${step}`, () => world.overlapShape(vec(-6, 6), proxy, filterOpts()));
      rec(`closest ${step}`, () => world.closestPoint(vec(-6, 6), num(0.5, 4), filterOpts()));
      const at = { x: real(-6, 6), y: real(0.4, 3), z: real(-6, 6) }, cap = capsule(), f = filterOpts();
      rec(`mover ${step}`, () => {
        const planes = world.collideMover(at, cap, f);
        const solved = b3.solvePlanes(vec(-1, 1), planes.map(p => ({ normal: p.normal, offset: p.offset, pushLimit: chance(0.3) ? real(0, 1) : undefined, clipVelocity: flag() })));
        const clipped = b3.clipVector(vec(-1, 1), planes.map((p, i) => ({ ...p, push: solved.push[i] })));
        return { planes, solved, clipped, cast: world.castMover(at, cap, solved.delta, f) };
      });
    }
    if (shapes.length) {
      const s = pick(shapes);
      rec(`shape ${step}`, () => s.isValid() ? { cp: s.getClosestPoint(vec(-6, 6)), ray: s.rayCast(vec(-6, 6), vec(-10, 10)), contacts: s.getContactData() } : 'gone');
    }
    if (chance(0.05) && bodies.length > 3) {
      // A live body only: the reference glue destroyed whatever held slot 0 on a second destroy.
      const i = 1 + Math.floor(r() * (bodies.length - 1));
      if (!bodies[i].isValid()) continue;
      rec(`destroy ${step}`, () => bodies[i].destroy());
    }
  }
  world.destroy();
  return log;
}

/** One scenario on one build in a worker: its log, or 'hang' after limitMs. */
function inWorker(dir, seed) {
  return new Promise((done, fail) => {
    const w = new Worker(new URL(import.meta.url), { workerData: { dir, seed }, stderr: true });
    const timer = setTimeout(() => { w.terminate(); done('hang'); }, limitMs);
    w.once('message', log => { clearTimeout(timer); w.terminate(); done(log); });
    w.once('error', e => { clearTimeout(timer); fail(e); });
  });
}

if (!isMainThread) {
  const m = await load(workerData.dir);
  parentPort.postMessage(scenario(m, workerData.seed));
} else {
  test('reference: the object calls answer as the reference build does', { skip: reference ? false : 'B3_REFERENCE not set' }, async () => {
    const now = resolve(import.meta.dirname, '../dist'), ref = resolve(reference);
    let calls = 0, compared = 0;
    const unstable = [];
    for (let seed = 1; seed <= seeds; seed++) {
      const [a, b] = await Promise.all([inWorker(now, seed), inWorker(ref, seed)]);
      if (a === 'hang' || b === 'hang') {
        assert.equal(a, b, `seed ${seed}: only one build hung (current: ${a === 'hang'}, reference: ${b === 'hang'})`);
        unstable.push(seed);
        continue;
      }
      for (let i = 0; i < Math.max(a.length, b.length); i++) assert.deepEqual(a[i], b[i], `seed ${seed}, call ${i} (${a[i]?.[0]})`);
      calls += a.length;
      compared++;
    }
    assert.ok(compared >= seeds * 0.8, `only ${compared} of ${seeds} scenarios compared (both builds hung on ${unstable.join(', ')})`);
    assert.ok(calls > 1000 * compared, `only ${calls} calls`);
    console.log(`${compared} scenarios, ${calls} calls compared${unstable.length ? `; both builds hung on seeds ${unstable.join(', ')}` : ''}`);
  });
}
