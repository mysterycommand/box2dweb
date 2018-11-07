Box2D.inherit(b2WeldJointDef, Box2D.Dynamics.Joints.b2JointDef);
b2WeldJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
b2WeldJointDef.b2WeldJointDef = function() {
  Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
  this.localAnchorA = new b2Vec2();
  this.localAnchorB = new b2Vec2();
};
b2WeldJointDef.prototype.b2WeldJointDef = function() {
  this.__super.b2JointDef.call(this);
  this.type = b2Joint.e_weldJoint;
  this.referenceAngle = 0.0;
};
b2WeldJointDef.prototype.Initialize = function(bA, bB, anchor) {
  this.bodyA = bA;
  this.bodyB = bB;
  this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
  this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
  this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
};
