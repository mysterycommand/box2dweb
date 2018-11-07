b2JointDef.b2JointDef = function() {};
b2JointDef.prototype.b2JointDef = function() {
  this.type = b2Joint.e_unknownJoint;
  this.userData = null;
  this.bodyA = null;
  this.bodyB = null;
  this.collideConnected = false;
};
