export default function g(Box2D) {
  const b2DestructionListener = Box2D.Dynamics.b2DestructionListener;

  /**
   * b2DestructionListener
   */
  b2DestructionListener.b2DestructionListener = function() {};
  b2DestructionListener.prototype.SayGoodbyeJoint = function(joint) {};
  b2DestructionListener.prototype.SayGoodbyeFixture = function(fixture) {};
}
