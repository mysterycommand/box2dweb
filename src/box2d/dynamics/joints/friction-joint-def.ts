Box2D.inherit(b2FrictionJointDef, Box2D.Dynamics.Joints.b2JointDef);
b2FrictionJointDef.prototype.__super =
  Box2D.Dynamics.Joints.b2JointDef.prototype;
b2FrictionJointDef.b2FrictionJointDef = function() {
  Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
  this.localAnchorA = new b2Vec2();
  this.localAnchorB = new b2Vec2();
};
b2FrictionJointDef.prototype.b2FrictionJointDef = function() {
  this.__super.b2JointDef.call(this);
  this.type = b2Joint.e_frictionJoint;
  this.maxForce = 0.0;
  this.maxTorque = 0.0;
};
b2FrictionJointDef.prototype.Initialize = function(bA, bB, anchor) {
  this.bodyA = bA;
  this.bodyB = bB;
  this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
  this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
};
