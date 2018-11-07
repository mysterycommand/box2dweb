b2DynamicTreeNode.b2DynamicTreeNode = function() {
  this.aabb = new b2AABB();
};
b2DynamicTreeNode.prototype.IsLeaf = function() {
  return this.child1 == null;
};
