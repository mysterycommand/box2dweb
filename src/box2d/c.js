export default function c(Box2D) {
  const Vector = Array;

  const b2Distance = Box2D.Collision.b2Distance;
  const b2DistanceInput = Box2D.Collision.b2DistanceInput;
  const b2DistanceOutput = Box2D.Collision.b2DistanceOutput;
  const b2DistanceProxy = Box2D.Collision.b2DistanceProxy;
  const b2Manifold = Box2D.Collision.b2Manifold;
  const b2Segment = Box2D.Collision.b2Segment;
  const b2SeparationFunction = Box2D.Collision.b2SeparationFunction;
  const b2SimplexCache = Box2D.Collision.b2SimplexCache;
  const b2TimeOfImpact = Box2D.Collision.b2TimeOfImpact;
  const b2TOIInput = Box2D.Collision.b2TOIInput;
  const b2WorldManifold = Box2D.Collision.b2WorldManifold;

  const b2Settings = Box2D.Common.b2Settings;

  const b2Math = Box2D.Common.Math.b2Math;
  const b2Sweep = Box2D.Common.Math.b2Sweep;
  const b2Transform = Box2D.Common.Math.b2Transform;
  const b2Vec2 = Box2D.Common.Math.b2Vec2;

  // b2Segment
  // b2SeparationFunction
  // b2TimeOfImpact
  // b2TOIInput
  // b2WorldManifold

  /**
   * b2Segment
   */
  b2Segment.b2Segment = function() {
    this.p1 = new b2Vec2();
    this.p2 = new b2Vec2();
  };
  b2Segment.prototype.TestSegment = function(
    lambda,
    normal,
    segment,
    maxLambda,
  ) {
    if (maxLambda === undefined) maxLambda = 0;
    var s = segment.p1;
    var rX = segment.p2.x - s.x;
    var rY = segment.p2.y - s.y;
    var dX = this.p2.x - this.p1.x;
    var dY = this.p2.y - this.p1.y;
    var nX = dY;
    var nY = -dX;
    var k_slop = 100.0 * Number.MIN_VALUE;
    var denom = -(rX * nX + rY * nY);
    if (denom > k_slop) {
      var bX = s.x - this.p1.x;
      var bY = s.y - this.p1.y;
      var a = bX * nX + bY * nY;
      if (0.0 <= a && a <= maxLambda * denom) {
        var mu2 = -rX * bY + rY * bX;
        if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
          a /= denom;
          var nLen = Math.sqrt(nX * nX + nY * nY);
          nX /= nLen;
          nY /= nLen;
          lambda[0] = a;
          normal.Set(nX, nY);
          return true;
        }
      }
    }
    return false;
  };
  b2Segment.prototype.Extend = function(aabb) {
    this.ExtendForward(aabb);
    this.ExtendBackward(aabb);
  };
  b2Segment.prototype.ExtendForward = function(aabb) {
    var dX = this.p2.x - this.p1.x;
    var dY = this.p2.y - this.p1.y;
    var lambda = Math.min(
      dX > 0
        ? (aabb.upperBound.x - this.p1.x) / dX
        : dX < 0
          ? (aabb.lowerBound.x - this.p1.x) / dX
          : Number.POSITIVE_INFINITY,
      dY > 0
        ? (aabb.upperBound.y - this.p1.y) / dY
        : dY < 0
          ? (aabb.lowerBound.y - this.p1.y) / dY
          : Number.POSITIVE_INFINITY,
    );
    this.p2.x = this.p1.x + dX * lambda;
    this.p2.y = this.p1.y + dY * lambda;
  };
  b2Segment.prototype.ExtendBackward = function(aabb) {
    var dX = -this.p2.x + this.p1.x;
    var dY = -this.p2.y + this.p1.y;
    var lambda = Math.min(
      dX > 0
        ? (aabb.upperBound.x - this.p2.x) / dX
        : dX < 0
          ? (aabb.lowerBound.x - this.p2.x) / dX
          : Number.POSITIVE_INFINITY,
      dY > 0
        ? (aabb.upperBound.y - this.p2.y) / dY
        : dY < 0
          ? (aabb.lowerBound.y - this.p2.y) / dY
          : Number.POSITIVE_INFINITY,
    );
    this.p1.x = this.p2.x + dX * lambda;
    this.p1.y = this.p2.y + dY * lambda;
  };

  /**
   * b2SeparationFunction
   */
  b2SeparationFunction.b2SeparationFunction = function() {
    this.m_localPoint = new b2Vec2();
    this.m_axis = new b2Vec2();
  };
  b2SeparationFunction.prototype.Initialize = function(
    cache,
    proxyA,
    transformA,
    proxyB,
    transformB,
  ) {
    this.m_proxyA = proxyA;
    this.m_proxyB = proxyB;
    var count = parseInt(cache.count);
    b2Settings.b2Assert(0 < count && count < 3);
    var localPointA;
    var localPointA1;
    var localPointA2;
    var localPointB;
    var localPointB1;
    var localPointB2;
    var pointAX = 0;
    var pointAY = 0;
    var pointBX = 0;
    var pointBY = 0;
    var normalX = 0;
    var normalY = 0;
    var tMat;
    var tVec;
    var s = 0;
    var sgn = 0;
    if (count == 1) {
      this.m_type = b2SeparationFunction.e_points;
      localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
      localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
      tVec = localPointA;
      tMat = transformA.R;
      pointAX =
        transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointAY =
        transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tVec = localPointB;
      tMat = transformB.R;
      pointBX =
        transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointBY =
        transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      this.m_axis.x = pointBX - pointAX;
      this.m_axis.y = pointBY - pointAY;
      this.m_axis.Normalize();
    } else if (cache.indexB[0] == cache.indexB[1]) {
      this.m_type = b2SeparationFunction.e_faceA;
      localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
      localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
      localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
      this.m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x);
      this.m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y);
      this.m_axis = b2Math.CrossVF(
        b2Math.SubtractVV(localPointA2, localPointA1),
        1.0,
      );
      this.m_axis.Normalize();
      tVec = this.m_axis;
      tMat = transformA.R;
      normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tVec = this.m_localPoint;
      tMat = transformA.R;
      pointAX =
        transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointAY =
        transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tVec = localPointB;
      tMat = transformB.R;
      pointBX =
        transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointBY =
        transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
      if (s < 0.0) {
        this.m_axis.NegativeSelf();
      }
    } else if (cache.indexA[0] == cache.indexA[0]) {
      this.m_type = b2SeparationFunction.e_faceB;
      localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
      localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
      localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
      this.m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x);
      this.m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y);
      this.m_axis = b2Math.CrossVF(
        b2Math.SubtractVV(localPointB2, localPointB1),
        1.0,
      );
      this.m_axis.Normalize();
      tVec = this.m_axis;
      tMat = transformB.R;
      normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tVec = this.m_localPoint;
      tMat = transformB.R;
      pointBX =
        transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointBY =
        transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tVec = localPointA;
      tMat = transformA.R;
      pointAX =
        transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointAY =
        transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
      if (s < 0.0) {
        this.m_axis.NegativeSelf();
      }
    } else {
      localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
      localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
      localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
      localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
      var pA = b2Math.MulX(transformA, localPointA);
      var dA = b2Math.MulMV(
        transformA.R,
        b2Math.SubtractVV(localPointA2, localPointA1),
      );
      var pB = b2Math.MulX(transformB, localPointB);
      var dB = b2Math.MulMV(
        transformB.R,
        b2Math.SubtractVV(localPointB2, localPointB1),
      );
      var a = dA.x * dA.x + dA.y * dA.y;
      var e = dB.x * dB.x + dB.y * dB.y;
      var r = b2Math.SubtractVV(dB, dA);
      var c = dA.x * r.x + dA.y * r.y;
      var f = dB.x * r.x + dB.y * r.y;
      var b = dA.x * dB.x + dA.y * dB.y;
      var denom = a * e - b * b;
      s = 0.0;
      if (denom != 0.0) {
        s = b2Math.Clamp((b * f - c * e) / denom, 0.0, 1.0);
      }
      var t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = b2Math.Clamp((b - c) / a, 0.0, 1.0);
      }
      localPointA = new b2Vec2();
      localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x);
      localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y);
      localPointB = new b2Vec2();
      localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x);
      localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y);
      if (s == 0.0 || s == 1.0) {
        this.m_type = b2SeparationFunction.e_faceB;
        this.m_axis = b2Math.CrossVF(
          b2Math.SubtractVV(localPointB2, localPointB1),
          1.0,
        );
        this.m_axis.Normalize();
        this.m_localPoint = localPointB;
        tVec = this.m_axis;
        tMat = transformB.R;
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tVec = this.m_localPoint;
        tMat = transformB.R;
        pointBX =
          transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointBY =
          transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tVec = localPointA;
        tMat = transformA.R;
        pointAX =
          transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointAY =
          transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        sgn = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
        if (s < 0.0) {
          this.m_axis.NegativeSelf();
        }
      } else {
        this.m_type = b2SeparationFunction.e_faceA;
        this.m_axis = b2Math.CrossVF(
          b2Math.SubtractVV(localPointA2, localPointA1),
          1.0,
        );
        this.m_localPoint = localPointA;
        tVec = this.m_axis;
        tMat = transformA.R;
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tVec = this.m_localPoint;
        tMat = transformA.R;
        pointAX =
          transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointAY =
          transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tVec = localPointB;
        tMat = transformB.R;
        pointBX =
          transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointBY =
          transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        sgn = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
        if (s < 0.0) {
          this.m_axis.NegativeSelf();
        }
      }
    }
  };
  b2SeparationFunction.prototype.Evaluate = function(transformA, transformB) {
    var axisA;
    var axisB;
    var localPointA;
    var localPointB;
    var pointA;
    var pointB;
    var seperation = 0;
    var normal;
    switch (this.m_type) {
      case b2SeparationFunction.e_points: {
        axisA = b2Math.MulTMV(transformA.R, this.m_axis);
        axisB = b2Math.MulTMV(transformB.R, this.m_axis.GetNegative());
        localPointA = this.m_proxyA.GetSupportVertex(axisA);
        localPointB = this.m_proxyB.GetSupportVertex(axisB);
        pointA = b2Math.MulX(transformA, localPointA);
        pointB = b2Math.MulX(transformB, localPointB);
        seperation =
          (pointB.x - pointA.x) * this.m_axis.x +
          (pointB.y - pointA.y) * this.m_axis.y;
        return seperation;
      }
      case b2SeparationFunction.e_faceA: {
        normal = b2Math.MulMV(transformA.R, this.m_axis);
        pointA = b2Math.MulX(transformA, this.m_localPoint);
        axisB = b2Math.MulTMV(transformB.R, normal.GetNegative());
        localPointB = this.m_proxyB.GetSupportVertex(axisB);
        pointB = b2Math.MulX(transformB, localPointB);
        seperation =
          (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
        return seperation;
      }
      case b2SeparationFunction.e_faceB: {
        normal = b2Math.MulMV(transformB.R, this.m_axis);
        pointB = b2Math.MulX(transformB, this.m_localPoint);
        axisA = b2Math.MulTMV(transformA.R, normal.GetNegative());
        localPointA = this.m_proxyA.GetSupportVertex(axisA);
        pointA = b2Math.MulX(transformA, localPointA);
        seperation =
          (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y;
        return seperation;
      }
      default:
        b2Settings.b2Assert(false);
        return 0.0;
    }
  };
  Box2D.postDefs.push(function() {
    Box2D.Collision.b2SeparationFunction.e_points = 0x01;
    Box2D.Collision.b2SeparationFunction.e_faceA = 0x02;
    Box2D.Collision.b2SeparationFunction.e_faceB = 0x04;
  });

  /**
   * b2TimeOfImpact
   */
  b2TimeOfImpact.b2TimeOfImpact = function() {};
  b2TimeOfImpact.TimeOfImpact = function(input) {
    ++b2TimeOfImpact.b2_toiCalls;
    var proxyA = input.proxyA;
    var proxyB = input.proxyB;
    var sweepA = input.sweepA;
    var sweepB = input.sweepB;
    b2Settings.b2Assert(sweepA.t0 == sweepB.t0);
    b2Settings.b2Assert(1.0 - sweepA.t0 > Number.MIN_VALUE);
    var radius = proxyA.m_radius + proxyB.m_radius;
    var tolerance = input.tolerance;
    var alpha = 0.0;
    var k_maxIterations = 1000;
    var iter = 0;
    var target = 0.0;
    b2TimeOfImpact.s_cache.count = 0;
    b2TimeOfImpact.s_distanceInput.useRadii = false;
    for (;;) {
      sweepA.GetTransform(b2TimeOfImpact.s_xfA, alpha);
      sweepB.GetTransform(b2TimeOfImpact.s_xfB, alpha);
      b2TimeOfImpact.s_distanceInput.proxyA = proxyA;
      b2TimeOfImpact.s_distanceInput.proxyB = proxyB;
      b2TimeOfImpact.s_distanceInput.transformA = b2TimeOfImpact.s_xfA;
      b2TimeOfImpact.s_distanceInput.transformB = b2TimeOfImpact.s_xfB;
      b2Distance.Distance(
        b2TimeOfImpact.s_distanceOutput,
        b2TimeOfImpact.s_cache,
        b2TimeOfImpact.s_distanceInput,
      );
      if (b2TimeOfImpact.s_distanceOutput.distance <= 0.0) {
        alpha = 1.0;
        break;
      }
      b2TimeOfImpact.s_fcn.Initialize(
        b2TimeOfImpact.s_cache,
        proxyA,
        b2TimeOfImpact.s_xfA,
        proxyB,
        b2TimeOfImpact.s_xfB,
      );
      var separation = b2TimeOfImpact.s_fcn.Evaluate(
        b2TimeOfImpact.s_xfA,
        b2TimeOfImpact.s_xfB,
      );
      if (separation <= 0.0) {
        alpha = 1.0;
        break;
      }
      if (iter == 0) {
        if (separation > radius) {
          target = b2Math.Max(radius - tolerance, 0.75 * radius);
        } else {
          target = b2Math.Max(separation - tolerance, 0.02 * radius);
        }
      }
      if (separation - target < 0.5 * tolerance) {
        if (iter == 0) {
          alpha = 1.0;
          break;
        }
        break;
      }
      var newAlpha = alpha;
      {
        var x1 = alpha;
        var x2 = 1.0;
        var f1 = separation;
        sweepA.GetTransform(b2TimeOfImpact.s_xfA, x2);
        sweepB.GetTransform(b2TimeOfImpact.s_xfB, x2);
        var f2 = b2TimeOfImpact.s_fcn.Evaluate(
          b2TimeOfImpact.s_xfA,
          b2TimeOfImpact.s_xfB,
        );
        if (f2 >= target) {
          alpha = 1.0;
          break;
        }
        var rootIterCount = 0;
        for (;;) {
          var x = 0;
          if (rootIterCount & 1) {
            x = x1 + ((target - f1) * (x2 - x1)) / (f2 - f1);
          } else {
            x = 0.5 * (x1 + x2);
          }
          sweepA.GetTransform(b2TimeOfImpact.s_xfA, x);
          sweepB.GetTransform(b2TimeOfImpact.s_xfB, x);
          var f = b2TimeOfImpact.s_fcn.Evaluate(
            b2TimeOfImpact.s_xfA,
            b2TimeOfImpact.s_xfB,
          );
          if (b2Math.Abs(f - target) < 0.025 * tolerance) {
            newAlpha = x;
            break;
          }
          if (f > target) {
            x1 = x;
            f1 = f;
          } else {
            x2 = x;
            f2 = f;
          }
          ++rootIterCount;
          ++b2TimeOfImpact.b2_toiRootIters;
          if (rootIterCount == 50) {
            break;
          }
        }
        b2TimeOfImpact.b2_toiMaxRootIters = b2Math.Max(
          b2TimeOfImpact.b2_toiMaxRootIters,
          rootIterCount,
        );
      }
      if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha) {
        break;
      }
      alpha = newAlpha;
      iter++;
      ++b2TimeOfImpact.b2_toiIters;
      if (iter == k_maxIterations) {
        break;
      }
    }
    b2TimeOfImpact.b2_toiMaxIters = b2Math.Max(
      b2TimeOfImpact.b2_toiMaxIters,
      iter,
    );
    return alpha;
  };
  Box2D.postDefs.push(function() {
    Box2D.Collision.b2TimeOfImpact.b2_toiCalls = 0;
    Box2D.Collision.b2TimeOfImpact.b2_toiIters = 0;
    Box2D.Collision.b2TimeOfImpact.b2_toiMaxIters = 0;
    Box2D.Collision.b2TimeOfImpact.b2_toiRootIters = 0;
    Box2D.Collision.b2TimeOfImpact.b2_toiMaxRootIters = 0;
    Box2D.Collision.b2TimeOfImpact.s_cache = new b2SimplexCache();
    Box2D.Collision.b2TimeOfImpact.s_distanceInput = new b2DistanceInput();
    Box2D.Collision.b2TimeOfImpact.s_xfA = new b2Transform();
    Box2D.Collision.b2TimeOfImpact.s_xfB = new b2Transform();
    Box2D.Collision.b2TimeOfImpact.s_fcn = new b2SeparationFunction();
    Box2D.Collision.b2TimeOfImpact.s_distanceOutput = new b2DistanceOutput();
  });

  /**
   * b2TOIInput
   */
  b2TOIInput.b2TOIInput = function() {
    this.proxyA = new b2DistanceProxy();
    this.proxyB = new b2DistanceProxy();
    this.sweepA = new b2Sweep();
    this.sweepB = new b2Sweep();
  };

  /**
   * b2WorldManifold
   */
  b2WorldManifold.b2WorldManifold = function() {
    this.m_normal = new b2Vec2();
  };
  b2WorldManifold.prototype.b2WorldManifold = function() {
    this.m_points = new Vector(b2Settings.b2_maxManifoldPoints);
    for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
      this.m_points[i] = new b2Vec2();
    }
  };
  b2WorldManifold.prototype.Initialize = function(
    manifold,
    xfA,
    radiusA,
    xfB,
    radiusB,
  ) {
    if (radiusA === undefined) radiusA = 0;
    if (radiusB === undefined) radiusB = 0;
    if (manifold.m_pointCount == 0) {
      return;
    }
    var i = 0;
    var tVec;
    var tMat;
    var normalX = 0;
    var normalY = 0;
    var planePointX = 0;
    var planePointY = 0;
    var clipPointX = 0;
    var clipPointY = 0;
    switch (manifold.m_type) {
      case b2Manifold.e_circles:
        {
          tMat = xfA.R;
          tVec = manifold.m_localPoint;
          var pointAX =
            xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          var pointAY =
            xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          tMat = xfB.R;
          tVec = manifold.m_points[0].m_localPoint;
          var pointBX =
            xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          var pointBY =
            xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          var dX = pointBX - pointAX;
          var dY = pointBY - pointAY;
          var d2 = dX * dX + dY * dY;
          if (d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
            var d = Math.sqrt(d2);
            this.m_normal.x = dX / d;
            this.m_normal.y = dY / d;
          } else {
            this.m_normal.x = 1;
            this.m_normal.y = 0;
          }
          var cAX = pointAX + radiusA * this.m_normal.x;
          var cAY = pointAY + radiusA * this.m_normal.y;
          var cBX = pointBX - radiusB * this.m_normal.x;
          var cBY = pointBY - radiusB * this.m_normal.y;
          this.m_points[0].x = 0.5 * (cAX + cBX);
          this.m_points[0].y = 0.5 * (cAY + cBY);
        }
        break;
      case b2Manifold.e_faceA:
        {
          tMat = xfA.R;
          tVec = manifold.m_localPlaneNormal;
          normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          tMat = xfA.R;
          tVec = manifold.m_localPoint;
          planePointX =
            xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          planePointY =
            xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          this.m_normal.x = normalX;
          this.m_normal.y = normalY;
          for (i = 0; i < manifold.m_pointCount; i++) {
            tMat = xfB.R;
            tVec = manifold.m_points[i].m_localPoint;
            clipPointX =
              xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            clipPointY =
              xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            this.m_points[i].x =
              clipPointX +
              0.5 *
                (radiusA -
                  (clipPointX - planePointX) * normalX -
                  (clipPointY - planePointY) * normalY -
                  radiusB) *
                normalX;
            this.m_points[i].y =
              clipPointY +
              0.5 *
                (radiusA -
                  (clipPointX - planePointX) * normalX -
                  (clipPointY - planePointY) * normalY -
                  radiusB) *
                normalY;
          }
        }
        break;
      case b2Manifold.e_faceB:
        {
          tMat = xfB.R;
          tVec = manifold.m_localPlaneNormal;
          normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          tMat = xfB.R;
          tVec = manifold.m_localPoint;
          planePointX =
            xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          planePointY =
            xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          this.m_normal.x = -normalX;
          this.m_normal.y = -normalY;
          for (i = 0; i < manifold.m_pointCount; i++) {
            tMat = xfA.R;
            tVec = manifold.m_points[i].m_localPoint;
            clipPointX =
              xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            clipPointY =
              xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            this.m_points[i].x =
              clipPointX +
              0.5 *
                (radiusB -
                  (clipPointX - planePointX) * normalX -
                  (clipPointY - planePointY) * normalY -
                  radiusA) *
                normalX;
            this.m_points[i].y =
              clipPointY +
              0.5 *
                (radiusB -
                  (clipPointX - planePointX) * normalX -
                  (clipPointY - planePointY) * normalY -
                  radiusA) *
                normalY;
          }
        }
        break;
    }
  };
}
