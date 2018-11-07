Box2D.inherit(b2GearJointDef, Box2D.Dynamics.Joints.b2JointDef);
b2GearJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
b2GearJointDef.b2GearJointDef = function() {
  Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
};
b2GearJointDef.prototype.b2GearJointDef = function() {
  this.__super.b2JointDef.call(this);
  this.type = b2Joint.e_gearJoint;
  this.joint1 = null;
  this.joint2 = null;
  this.ratio = 1.0;
};
