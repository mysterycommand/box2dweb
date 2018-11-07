export default function f(Box2D) {
  const b2Mat33 = Box2D.Common.Math.b2Mat33;
  const b2Sweep = Box2D.Common.Math.b2Sweep;
  const b2Vec2 = Box2D.Common.Math.b2Vec2;
  const b2Vec3 = Box2D.Common.Math.b2Vec3;

  b2Mat33.b2Mat33 = function() {
    this.col1 = new b2Vec3();
    this.col2 = new b2Vec3();
    this.col3 = new b2Vec3();
  };
  b2Mat33.prototype.b2Mat33 = function(c1, c2, c3) {
    if (c1 === undefined) c1 = null;
    if (c2 === undefined) c2 = null;
    if (c3 === undefined) c3 = null;
    if (!c1 && !c2 && !c3) {
      this.col1.SetZero();
      this.col2.SetZero();
      this.col3.SetZero();
    } else {
      this.col1.SetV(c1);
      this.col2.SetV(c2);
      this.col3.SetV(c3);
    }
  };
  b2Mat33.prototype.SetVVV = function(c1, c2, c3) {
    this.col1.SetV(c1);
    this.col2.SetV(c2);
    this.col3.SetV(c3);
  };
  b2Mat33.prototype.Copy = function() {
    return new b2Mat33(this.col1, this.col2, this.col3);
  };
  b2Mat33.prototype.SetM = function(m) {
    this.col1.SetV(m.col1);
    this.col2.SetV(m.col2);
    this.col3.SetV(m.col3);
  };
  b2Mat33.prototype.AddM = function(m) {
    this.col1.x += m.col1.x;
    this.col1.y += m.col1.y;
    this.col1.z += m.col1.z;
    this.col2.x += m.col2.x;
    this.col2.y += m.col2.y;
    this.col2.z += m.col2.z;
    this.col3.x += m.col3.x;
    this.col3.y += m.col3.y;
    this.col3.z += m.col3.z;
  };
  b2Mat33.prototype.SetIdentity = function() {
    this.col1.x = 1.0;
    this.col2.x = 0.0;
    this.col3.x = 0.0;
    this.col1.y = 0.0;
    this.col2.y = 1.0;
    this.col3.y = 0.0;
    this.col1.z = 0.0;
    this.col2.z = 0.0;
    this.col3.z = 1.0;
  };
  b2Mat33.prototype.SetZero = function() {
    this.col1.x = 0.0;
    this.col2.x = 0.0;
    this.col3.x = 0.0;
    this.col1.y = 0.0;
    this.col2.y = 0.0;
    this.col3.y = 0.0;
    this.col1.z = 0.0;
    this.col2.z = 0.0;
    this.col3.z = 0.0;
  };
  b2Mat33.prototype.Solve22 = function(out, bX, bY) {
    if (bX === undefined) bX = 0;
    if (bY === undefined) bY = 0;
    var a11 = this.col1.x;
    var a12 = this.col2.x;
    var a21 = this.col1.y;
    var a22 = this.col2.y;
    var det = a11 * a22 - a12 * a21;
    if (det != 0.0) {
      det = 1.0 / det;
    }
    out.x = det * (a22 * bX - a12 * bY);
    out.y = det * (a11 * bY - a21 * bX);
    return out;
  };
  b2Mat33.prototype.Solve33 = function(out, bX, bY, bZ) {
    if (bX === undefined) bX = 0;
    if (bY === undefined) bY = 0;
    if (bZ === undefined) bZ = 0;
    var a11 = this.col1.x;
    var a21 = this.col1.y;
    var a31 = this.col1.z;
    var a12 = this.col2.x;
    var a22 = this.col2.y;
    var a32 = this.col2.z;
    var a13 = this.col3.x;
    var a23 = this.col3.y;
    var a33 = this.col3.z;
    var det =
      a11 * (a22 * a33 - a32 * a23) +
      a21 * (a32 * a13 - a12 * a33) +
      a31 * (a12 * a23 - a22 * a13);
    if (det != 0.0) {
      det = 1.0 / det;
    }
    out.x =
      det *
      (bX * (a22 * a33 - a32 * a23) +
        bY * (a32 * a13 - a12 * a33) +
        bZ * (a12 * a23 - a22 * a13));
    out.y =
      det *
      (a11 * (bY * a33 - bZ * a23) +
        a21 * (bZ * a13 - bX * a33) +
        a31 * (bX * a23 - bY * a13));
    out.z =
      det *
      (a11 * (a22 * bZ - a32 * bY) +
        a21 * (a32 * bX - a12 * bZ) +
        a31 * (a12 * bY - a22 * bX));
    return out;
  };

  b2Sweep.b2Sweep = function() {
    this.localCenter = new b2Vec2();
    this.c0 = new b2Vec2();
    this.c = new b2Vec2();
  };
  b2Sweep.prototype.Set = function(other) {
    this.localCenter.SetV(other.localCenter);
    this.c0.SetV(other.c0);
    this.c.SetV(other.c);
    this.a0 = other.a0;
    this.a = other.a;
    this.t0 = other.t0;
  };
  b2Sweep.prototype.Copy = function() {
    var copy = new b2Sweep();
    copy.localCenter.SetV(this.localCenter);
    copy.c0.SetV(this.c0);
    copy.c.SetV(this.c);
    copy.a0 = this.a0;
    copy.a = this.a;
    copy.t0 = this.t0;
    return copy;
  };
  b2Sweep.prototype.GetTransform = function(xf, alpha) {
    if (alpha === undefined) alpha = 0;
    xf.position.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
    xf.position.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
    var angle = (1.0 - alpha) * this.a0 + alpha * this.a;
    xf.R.Set(angle);
    var tMat = xf.R;
    xf.position.x -=
      tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y;
    xf.position.y -=
      tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y;
  };
  b2Sweep.prototype.Advance = function(t) {
    if (t === undefined) t = 0;
    if (this.t0 < t && 1.0 - this.t0 > Number.MIN_VALUE) {
      var alpha = (t - this.t0) / (1.0 - this.t0);
      this.c0.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
      this.c0.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
      this.a0 = (1.0 - alpha) * this.a0 + alpha * this.a;
      this.t0 = t;
    }
  };

  b2Vec3.b2Vec3 = function() {};
  b2Vec3.prototype.b2Vec3 = function(x, y, z) {
    if (x === undefined) x = 0;
    if (y === undefined) y = 0;
    if (z === undefined) z = 0;
    this.x = x;
    this.y = y;
    this.z = z;
  };
  b2Vec3.prototype.SetZero = function() {
    this.x = this.y = this.z = 0.0;
  };
  b2Vec3.prototype.Set = function(x, y, z) {
    if (x === undefined) x = 0;
    if (y === undefined) y = 0;
    if (z === undefined) z = 0;
    this.x = x;
    this.y = y;
    this.z = z;
  };
  b2Vec3.prototype.SetV = function(v) {
    this.x = v.x;
    this.y = v.y;
    this.z = v.z;
  };
  b2Vec3.prototype.GetNegative = function() {
    return new b2Vec3(-this.x, -this.y, -this.z);
  };
  b2Vec3.prototype.NegativeSelf = function() {
    this.x = -this.x;
    this.y = -this.y;
    this.z = -this.z;
  };
  b2Vec3.prototype.Copy = function() {
    return new b2Vec3(this.x, this.y, this.z);
  };
  b2Vec3.prototype.Add = function(v) {
    this.x += v.x;
    this.y += v.y;
    this.z += v.z;
  };
  b2Vec3.prototype.Subtract = function(v) {
    this.x -= v.x;
    this.y -= v.y;
    this.z -= v.z;
  };
  b2Vec3.prototype.Multiply = function(a) {
    if (a === undefined) a = 0;
    this.x *= a;
    this.y *= a;
    this.z *= a;
  };
}
