b2RayCastInput.b2RayCastInput = function() {
  this.p1 = new b2Vec2();
  this.p2 = new b2Vec2();
};
b2RayCastInput.prototype.b2RayCastInput = function(p1, p2, maxFraction) {
  if (p1 === undefined) p1 = null;
  if (p2 === undefined) p2 = null;
  if (maxFraction === undefined) maxFraction = 1;
  if (p1) this.p1.SetV(p1);
  if (p2) this.p2.SetV(p2);
  this.maxFraction = maxFraction;
};
