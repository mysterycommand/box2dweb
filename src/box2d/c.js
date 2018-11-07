export default function c(Box2D) {
  const Vector = Array;

  const b2AABB = Box2D.Collision.b2AABB;
  const b2Distance = Box2D.Collision.b2Distance;
  const b2DistanceInput = Box2D.Collision.b2DistanceInput;
  const b2DistanceOutput = Box2D.Collision.b2DistanceOutput;
  const b2DistanceProxy = Box2D.Collision.b2DistanceProxy;
  const b2DynamicTree = Box2D.Collision.b2DynamicTree;
  const b2DynamicTreeBroadPhase = Box2D.Collision.b2DynamicTreeBroadPhase;
  const b2DynamicTreeNode = Box2D.Collision.b2DynamicTreeNode;
  const b2DynamicTreePair = Box2D.Collision.b2DynamicTreePair;
  const b2Manifold = Box2D.Collision.b2Manifold;
  const b2RayCastInput = Box2D.Collision.b2RayCastInput;
  const b2Segment = Box2D.Collision.b2Segment;
  const b2SeparationFunction = Box2D.Collision.b2SeparationFunction;
  const b2SimplexCache = Box2D.Collision.b2SimplexCache;
  const b2TimeOfImpact = Box2D.Collision.b2TimeOfImpact;
  const b2TOIInput = Box2D.Collision.b2TOIInput;
  const b2WorldManifold = Box2D.Collision.b2WorldManifold;

  const IBroadPhase = Box2D.Collision.IBroadPhase;

  const b2Settings = Box2D.Common.b2Settings;

  const b2Math = Box2D.Common.Math.b2Math;
  const b2Sweep = Box2D.Common.Math.b2Sweep;
  const b2Transform = Box2D.Common.Math.b2Transform;
  const b2Vec2 = Box2D.Common.Math.b2Vec2;

  // b2DynamicTree
  // b2DynamicTreeBroadPhase
  // b2DynamicTreeNode
  // b2DynamicTreePair
  // b2Segment
  // b2SeparationFunction
  // b2TimeOfImpact
  // b2TOIInput
  // b2WorldManifold

  /**
   * b2DynamicTree
   */
  b2DynamicTree.b2DynamicTree = function() {};
  b2DynamicTree.prototype.b2DynamicTree = function() {
    this.m_root = null;
    this.m_freeList = null;
    this.m_path = 0;
    this.m_insertionCount = 0;
  };
  b2DynamicTree.prototype.CreateProxy = function(aabb, userData) {
    var node = this.AllocateNode();
    var extendX = b2Settings.b2_aabbExtension;
    var extendY = b2Settings.b2_aabbExtension;
    node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
    node.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
    node.aabb.upperBound.x = aabb.upperBound.x + extendX;
    node.aabb.upperBound.y = aabb.upperBound.y + extendY;
    node.userData = userData;
    this.InsertLeaf(node);
    return node;
  };
  b2DynamicTree.prototype.DestroyProxy = function(proxy) {
    this.RemoveLeaf(proxy);
    this.FreeNode(proxy);
  };
  b2DynamicTree.prototype.MoveProxy = function(proxy, aabb, displacement) {
    b2Settings.b2Assert(proxy.IsLeaf());
    if (proxy.aabb.Contains(aabb)) {
      return false;
    }
    this.RemoveLeaf(proxy);
    var extendX =
      b2Settings.b2_aabbExtension +
      b2Settings.b2_aabbMultiplier *
        (displacement.x > 0 ? displacement.x : -displacement.x);
    var extendY =
      b2Settings.b2_aabbExtension +
      b2Settings.b2_aabbMultiplier *
        (displacement.y > 0 ? displacement.y : -displacement.y);
    proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
    proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
    proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
    proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;
    this.InsertLeaf(proxy);
    return true;
  };
  b2DynamicTree.prototype.Rebalance = function(iterations) {
    if (iterations === undefined) iterations = 0;
    if (this.m_root == null) return;
    for (var i = 0; i < iterations; i++) {
      var node = this.m_root;
      var bit = 0;
      while (node.IsLeaf() == false) {
        node = (this.m_path >> bit) & 1 ? node.child2 : node.child1;
        bit = (bit + 1) & 31;
      }
      ++this.m_path;
      this.RemoveLeaf(node);
      this.InsertLeaf(node);
    }
  };
  b2DynamicTree.prototype.GetFatAABB = function(proxy) {
    return proxy.aabb;
  };
  b2DynamicTree.prototype.GetUserData = function(proxy) {
    return proxy.userData;
  };
  b2DynamicTree.prototype.Query = function(callback, aabb) {
    if (this.m_root == null) return;
    var stack = new Vector();
    var count = 0;
    stack[count++] = this.m_root;
    while (count > 0) {
      var node = stack[--count];
      if (node.aabb.TestOverlap(aabb)) {
        if (node.IsLeaf()) {
          var proceed = callback(node);
          if (!proceed) return;
        } else {
          stack[count++] = node.child1;
          stack[count++] = node.child2;
        }
      }
    }
  };
  b2DynamicTree.prototype.RayCast = function(callback, input) {
    if (this.m_root == null) return;
    var p1 = input.p1;
    var p2 = input.p2;
    var r = b2Math.SubtractVV(p1, p2);
    r.Normalize();
    var v = b2Math.CrossFV(1.0, r);
    var abs_v = b2Math.AbsV(v);
    var maxFraction = input.maxFraction;
    var segmentAABB = new b2AABB();
    var tX = 0;
    var tY = 0;
    {
      tX = p1.x + maxFraction * (p2.x - p1.x);
      tY = p1.y + maxFraction * (p2.y - p1.y);
      segmentAABB.lowerBound.x = Math.min(p1.x, tX);
      segmentAABB.lowerBound.y = Math.min(p1.y, tY);
      segmentAABB.upperBound.x = Math.max(p1.x, tX);
      segmentAABB.upperBound.y = Math.max(p1.y, tY);
    }
    var stack = new Vector();
    var count = 0;
    stack[count++] = this.m_root;
    while (count > 0) {
      var node = stack[--count];
      if (node.aabb.TestOverlap(segmentAABB) == false) {
        continue;
      }
      var c = node.aabb.GetCenter();
      var h = node.aabb.GetExtents();
      var separation =
        Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) -
        abs_v.x * h.x -
        abs_v.y * h.y;
      if (separation > 0.0) continue;
      if (node.IsLeaf()) {
        var subInput = new b2RayCastInput();
        subInput.p1 = input.p1;
        subInput.p2 = input.p2;
        subInput.maxFraction = input.maxFraction;
        maxFraction = callback(subInput, node);
        if (maxFraction == 0.0) return;
        if (maxFraction > 0.0) {
          tX = p1.x + maxFraction * (p2.x - p1.x);
          tY = p1.y + maxFraction * (p2.y - p1.y);
          segmentAABB.lowerBound.x = Math.min(p1.x, tX);
          segmentAABB.lowerBound.y = Math.min(p1.y, tY);
          segmentAABB.upperBound.x = Math.max(p1.x, tX);
          segmentAABB.upperBound.y = Math.max(p1.y, tY);
        }
      } else {
        stack[count++] = node.child1;
        stack[count++] = node.child2;
      }
    }
  };
  b2DynamicTree.prototype.AllocateNode = function() {
    if (this.m_freeList) {
      var node = this.m_freeList;
      this.m_freeList = node.parent;
      node.parent = null;
      node.child1 = null;
      node.child2 = null;
      return node;
    }
    return new b2DynamicTreeNode();
  };
  b2DynamicTree.prototype.FreeNode = function(node) {
    node.parent = this.m_freeList;
    this.m_freeList = node;
  };
  b2DynamicTree.prototype.InsertLeaf = function(leaf) {
    ++this.m_insertionCount;
    if (this.m_root == null) {
      this.m_root = leaf;
      this.m_root.parent = null;
      return;
    }
    var center = leaf.aabb.GetCenter();
    var sibling = this.m_root;
    if (sibling.IsLeaf() == false) {
      do {
        var child1 = sibling.child1;
        var child2 = sibling.child2;
        var norm1 =
          Math.abs(
            (child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 -
              center.x,
          ) +
          Math.abs(
            (child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 -
              center.y,
          );
        var norm2 =
          Math.abs(
            (child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 -
              center.x,
          ) +
          Math.abs(
            (child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 -
              center.y,
          );
        if (norm1 < norm2) {
          sibling = child1;
        } else {
          sibling = child2;
        }
      } while (sibling.IsLeaf() == false);
    }
    var node1 = sibling.parent;
    var node2 = this.AllocateNode();
    node2.parent = node1;
    node2.userData = null;
    node2.aabb.Combine(leaf.aabb, sibling.aabb);
    if (node1) {
      if (sibling.parent.child1 == sibling) {
        node1.child1 = node2;
      } else {
        node1.child2 = node2;
      }
      node2.child1 = sibling;
      node2.child2 = leaf;
      sibling.parent = node2;
      leaf.parent = node2;
      do {
        if (node1.aabb.Contains(node2.aabb)) break;
        node1.aabb.Combine(node1.child1.aabb, node1.child2.aabb);
        node2 = node1;
        node1 = node1.parent;
      } while (node1);
    } else {
      node2.child1 = sibling;
      node2.child2 = leaf;
      sibling.parent = node2;
      leaf.parent = node2;
      this.m_root = node2;
    }
  };
  b2DynamicTree.prototype.RemoveLeaf = function(leaf) {
    if (leaf == this.m_root) {
      this.m_root = null;
      return;
    }
    var node2 = leaf.parent;
    var node1 = node2.parent;
    var sibling;
    if (node2.child1 == leaf) {
      sibling = node2.child2;
    } else {
      sibling = node2.child1;
    }
    if (node1) {
      if (node1.child1 == node2) {
        node1.child1 = sibling;
      } else {
        node1.child2 = sibling;
      }
      sibling.parent = node1;
      this.FreeNode(node2);
      while (node1) {
        var oldAABB = node1.aabb;
        node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb);
        if (oldAABB.Contains(node1.aabb)) break;
        node1 = node1.parent;
      }
    } else {
      this.m_root = sibling;
      sibling.parent = null;
      this.FreeNode(node2);
    }
  };

  /**
   * b2DynamicTreeBroadPhase
   */
  b2DynamicTreeBroadPhase.b2DynamicTreeBroadPhase = function() {
    this.m_tree = new b2DynamicTree();
    this.m_moveBuffer = new Vector();
    this.m_pairBuffer = new Vector();
    this.m_pairCount = 0;
  };
  b2DynamicTreeBroadPhase.prototype.CreateProxy = function(aabb, userData) {
    var proxy = this.m_tree.CreateProxy(aabb, userData);
    ++this.m_proxyCount;
    this.BufferMove(proxy);
    return proxy;
  };
  b2DynamicTreeBroadPhase.prototype.DestroyProxy = function(proxy) {
    this.UnBufferMove(proxy);
    --this.m_proxyCount;
    this.m_tree.DestroyProxy(proxy);
  };
  b2DynamicTreeBroadPhase.prototype.MoveProxy = function(
    proxy,
    aabb,
    displacement,
  ) {
    var buffer = this.m_tree.MoveProxy(proxy, aabb, displacement);
    if (buffer) {
      this.BufferMove(proxy);
    }
  };
  b2DynamicTreeBroadPhase.prototype.TestOverlap = function(proxyA, proxyB) {
    var aabbA = this.m_tree.GetFatAABB(proxyA);
    var aabbB = this.m_tree.GetFatAABB(proxyB);
    return aabbA.TestOverlap(aabbB);
  };
  b2DynamicTreeBroadPhase.prototype.GetUserData = function(proxy) {
    return this.m_tree.GetUserData(proxy);
  };
  b2DynamicTreeBroadPhase.prototype.GetFatAABB = function(proxy) {
    return this.m_tree.GetFatAABB(proxy);
  };
  b2DynamicTreeBroadPhase.prototype.GetProxyCount = function() {
    return this.m_proxyCount;
  };
  b2DynamicTreeBroadPhase.prototype.UpdatePairs = function(callback) {
    var __this = this;
    __this.m_pairCount = 0;
    var i = 0,
      queryProxy;
    for (i = 0; i < __this.m_moveBuffer.length; ++i) {
      queryProxy = __this.m_moveBuffer[i];

      function QueryCallback(proxy) {
        if (proxy == queryProxy) return true;
        if (__this.m_pairCount == __this.m_pairBuffer.length) {
          __this.m_pairBuffer[__this.m_pairCount] = new b2DynamicTreePair();
        }
        var pair = __this.m_pairBuffer[__this.m_pairCount];
        pair.proxyA = proxy < queryProxy ? proxy : queryProxy;
        pair.proxyB = proxy >= queryProxy ? proxy : queryProxy;
        ++__this.m_pairCount;
        return true;
      }
      var fatAABB = __this.m_tree.GetFatAABB(queryProxy);
      __this.m_tree.Query(QueryCallback, fatAABB);
    }
    __this.m_moveBuffer.length = 0;
    for (var i = 0; i < __this.m_pairCount; ) {
      var primaryPair = __this.m_pairBuffer[i];
      var userDataA = __this.m_tree.GetUserData(primaryPair.proxyA);
      var userDataB = __this.m_tree.GetUserData(primaryPair.proxyB);
      callback(userDataA, userDataB);
      ++i;
      while (i < __this.m_pairCount) {
        var pair = __this.m_pairBuffer[i];
        if (
          pair.proxyA != primaryPair.proxyA ||
          pair.proxyB != primaryPair.proxyB
        ) {
          break;
        }
        ++i;
      }
    }
  };
  b2DynamicTreeBroadPhase.prototype.Query = function(callback, aabb) {
    this.m_tree.Query(callback, aabb);
  };
  b2DynamicTreeBroadPhase.prototype.RayCast = function(callback, input) {
    this.m_tree.RayCast(callback, input);
  };
  b2DynamicTreeBroadPhase.prototype.Validate = function() {};
  b2DynamicTreeBroadPhase.prototype.Rebalance = function(iterations) {
    if (iterations === undefined) iterations = 0;
    this.m_tree.Rebalance(iterations);
  };
  b2DynamicTreeBroadPhase.prototype.BufferMove = function(proxy) {
    this.m_moveBuffer[this.m_moveBuffer.length] = proxy;
  };
  b2DynamicTreeBroadPhase.prototype.UnBufferMove = function(proxy) {
    var i = parseInt(this.m_moveBuffer.indexOf(proxy));
    this.m_moveBuffer.splice(i, 1);
  };
  b2DynamicTreeBroadPhase.prototype.ComparePairs = function(pair1, pair2) {
    return 0;
  };
  b2DynamicTreeBroadPhase.__implements = {};
  b2DynamicTreeBroadPhase.__implements[IBroadPhase] = true;

  /**
   * b2DynamicTreeNode
   */
  b2DynamicTreeNode.b2DynamicTreeNode = function() {
    this.aabb = new b2AABB();
  };
  b2DynamicTreeNode.prototype.IsLeaf = function() {
    return this.child1 == null;
  };

  /**
   * b2DynamicTreePair
   */
  b2DynamicTreePair.b2DynamicTreePair = function() {};

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
