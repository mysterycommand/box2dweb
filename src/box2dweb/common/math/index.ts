// tslint:disable variable-name

import Vec2 from './vec2';
import Mat22 from './mat22';
import Transform from './transform';

export const b2Vec2_zero = new Vec2(0.0, 0.0);

export const b2Mat22_identity = Mat22.FromVV(
  new Vec2(1.0, 0.0),
  new Vec2(0.0, 1.0),
);

export const b2Transform_identity = new Transform(
  b2Vec2_zero,
  b2Mat22_identity,
);

export function IsValid(x = 0) {
  return isFinite(x);
}

export function Dot(a: { x: number; y: number }, b: { x: number; y: number }) {
  return a.x * b.x + a.y * b.y;
}

export function CrossVV(
  a: { x: number; y: number },
  b: { x: number; y: number },
) {
  return a.x * b.y - a.y * b.x;
}

export function CrossVF(a: { x: number; y: number }, s = 0) {
  return new Vec2(s * a.y, -s * a.x);
}

export function CrossFV(s = 0, a: { x: number; y: number }) {
  return new Vec2(-s * a.y, s * a.x);
}

export function MulMV(
  A: {
    col1: { x: number; y: number };
    col2: { x: number; y: number };
  },
  v: { x: number; y: number },
) {
  return new Vec2(
    A.col1.x * v.x + A.col2.x * v.y,
    A.col1.y * v.x + A.col2.y * v.y,
  );
}

export function MulTMV(
  A: {
    col1: { x: number; y: number };
    col2: { x: number; y: number };
  },
  v: { x: number; y: number },
) {
  return new Vec2(Dot(v, A.col1), Dot(v, A.col2));
}

export function MulX(
  T: {
    position: { x: number; y: number };
    R: {
      col1: { x: number; y: number };
      col2: { x: number; y: number };
    };
  },
  v: { x: number; y: number },
) {
  const a = MulMV(T.R, v);
  a.x += T.position.x;
  a.y += T.position.y;
  return a;
}

export function MulXT(
  T: {
    position: { x: number; y: number };
    R: {
      col1: { x: number; y: number };
      col2: { x: number; y: number };
    };
  },
  v: { x: number; y: number },
) {
  const a = SubtractVV(v, T.position);
  const tX = a.x * T.R.col1.x + a.y * T.R.col1.y;
  a.y = a.x * T.R.col2.x + a.y * T.R.col2.y;
  a.x = tX;
  return a;
}

export function AddVV(
  a: { x: number; y: number },
  b: { x: number; y: number },
) {
  return new Vec2(a.x + b.x, a.y + b.y);
}

export function SubtractVV(
  a: { x: number; y: number },
  b: { x: number; y: number },
) {
  return new Vec2(a.x - b.x, a.y - b.y);
}

export function Distance(
  a: { x: number; y: number },
  b: { x: number; y: number },
) {
  return Math.hypot(a.x - b.x, a.y - b.y);
  // const cX = a.x - b.x;
  // const cY = a.y - b.y;
  // return Math.sqrt(cX * cX + cY * cY);
}

export function DistanceSquared(
  a: { x: number; y: number },
  b: { x: number; y: number },
) {
  const cX = a.x - b.x;
  const cY = a.y - b.y;
  return cX * cX + cY * cY;
}

export function MulFV(s = 0, a: { x: number; y: number }) {
  return new Vec2(s * a.x, s * a.y);
}

export function AddMM(
  A: {
    col1: { x: number; y: number };
    col2: { x: number; y: number };
  },
  B: {
    col1: { x: number; y: number };
    col2: { x: number; y: number };
  },
) {
  return Mat22.FromVV(AddVV(A.col1, B.col1), AddVV(A.col2, B.col2));
}

export function MulMM(
  A: {
    col1: { x: number; y: number };
    col2: { x: number; y: number };
  },
  B: {
    col1: { x: number; y: number };
    col2: { x: number; y: number };
  },
) {
  return Mat22.FromVV(MulMV(A, B.col1), MulMV(A, B.col2));
}

export function MulTMM(
  A: {
    col1: { x: number; y: number };
    col2: { x: number; y: number };
  },
  B: {
    col1: { x: number; y: number };
    col2: { x: number; y: number };
  },
) {
  const c1 = new Vec2(Dot(A.col1, B.col1), Dot(A.col2, B.col1));
  const c2 = new Vec2(Dot(A.col1, B.col2), Dot(A.col2, B.col2));
  return Mat22.FromVV(c1, c2);
}

export function Abs(a = 0) {
  return a > 0.0 ? a : -a;
}

export function AbsV(a: { x: number; y: number }) {
  return new Vec2(Abs(a.x), Abs(a.y));
}

export function AbsM(A: {
  col1: { x: number; y: number };
  col2: { x: number; y: number };
}) {
  return Mat22.FromVV(AbsV(A.col1), AbsV(A.col2));
}

export function Min(a = 0, b = 0) {
  return a < b ? a : b;
}

export function MinV(a: { x: number; y: number }, b: { x: number; y: number }) {
  return new Vec2(Min(a.x, b.x), Min(a.y, b.y));
}

export function Max(a = 0, b = 0) {
  return a > b ? a : b;
}

export function MaxV(a: { x: number; y: number }, b: { x: number; y: number }) {
  return new Vec2(Max(a.x, b.x), Max(a.y, b.y));
}

export function Clamp(a = 0, low = 0, high = 0) {
  return a < low ? low : a > high ? high : a;
}

export function ClampV(
  a: { x: number; y: number },
  low: { x: number; y: number },
  high: { x: number; y: number },
) {
  return MaxV(low, MinV(a, high));
}

export function Swap(a: any[], b: any[]) {
  const tmp = a[0];
  a[0] = b[0];
  b[0] = tmp;
}

export function Random() {
  return Math.random() * 2 - 1;
}

export function RandomRange(lo = 0, hi = 0) {
  const r = Math.random();
  return (hi - lo) * r + lo;
}

export function NextPowerOfTwo(x = 0) {
  x |= (x >> 1) & 0x7fffffff;
  x |= (x >> 2) & 0x3fffffff;
  x |= (x >> 4) & 0x0fffffff;
  x |= (x >> 8) & 0x00ffffff;
  x |= (x >> 16) & 0x0000ffff;
  return x + 1;
}

export function IsPowerOfTwo(x = 0) {
  return x > 0 && (x & (x - 1)) === 0;
}
