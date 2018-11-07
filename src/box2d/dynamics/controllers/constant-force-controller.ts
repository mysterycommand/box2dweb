Box2D.inherit(
  b2ConstantForceController,
  Box2D.Dynamics.Controllers.b2Controller,
);

b2ConstantForceController.prototype.__super =
  Box2D.Dynamics.Controllers.b2Controller.prototype;

b2ConstantForceController.b2ConstantForceController = function() {
  Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply(this, arguments);
  this.F = new b2Vec2(0, 0);
};

b2ConstantForceController.prototype.Step = function(step) {
  for (var i = this.m_bodyList; i; i = i.nextBody) {
    var body = i.body;
    if (!body.IsAwake()) continue;
    body.ApplyForce(this.F, body.GetWorldCenter());
  }
};
