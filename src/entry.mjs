// Picks the build a host can run:
//
//   scalar   no wasm SIMD on this host
//   deluxe   SIMD + threads, where SharedArrayBuffer is usable (browsers: a cross-origin isolated page; Node.js always)
//   standard SIMD, single threaded
//
// Import box3d-efa/scalar, box3d-efa/standard or box3d-efa/deluxe to skip the check.
const SIMD = new Uint8Array([0, 97, 115, 109, 1, 0, 0, 0, 1, 5, 1, 96, 0, 1, 123, 3, 2, 1, 0, 10, 10, 1, 8, 0, 65, 0, 253, 15, 253, 98, 11]);

export const hasSimd = () => WebAssembly.validate(SIMD);

export default async (options) => {
  if (!hasSimd()) return (await import('./box3d.scalar.mjs')).default(options);
  const canThread =
    typeof SharedArrayBuffer !== 'undefined' &&
    (typeof globalThis.crossOriginIsolated === 'undefined' || globalThis.crossOriginIsolated);

  const flavour = canThread ? await import('./box3d.deluxe.mjs') : await import('./box3d.mjs');
  return await flavour.default(options);
};
